#pragma once
// Erase and program the flash through the boot ROM, with XIP off around the ROM call.
//
// Every caller masks interrupts around these (XIP is off: nothing may run from flash), so an operation that takes too
// long starves the watchdog's feed, and its reset leaves no record of its own. Each operation is therefore written to
// .noInit before it starts and cleared when it is done (Flash::Detail::opInFlight): a reset in the middle of one keeps
// SRAM, and the next boot reads which it was (Flash::opInFlightAtBoot(); the boot report's chip notes name it,
// clock_config.hpp Port::StartNotes) - for an image the boot ROM loads itself. An encrypted image's decryption stage
// (picotool) loads over all of SRAM at every boot, .noInit included: there the record never reaches the next boot.
// Each operation's time is measured too (Flash::opTimes(): the last, the longest since boot and where), which needs no
// reset to survive. Two TIMER reads and a few stores each, outside the RAM functions. Bootrom::explicitBuy
// (BootInfo.hpp), the boot ROM's own erase and program of the TBYB sector, is recorded the same way.
//
// And each operation arms the watchdog boot vector while it runs (FlashBusyBoot.hpp): a watchdog reset in the middle
// of an erase would otherwise have the boot ROM read the still busy flash as empty and stay in BOOTSEL.
#include "FlashBusyBoot.hpp"
#include "FreeRunningUs.hpp"
#include "Rom.hpp"
#include "XipControl.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <kvasir/Util/attributes.hpp>
#include <optional>
#include <string_view>

namespace Kvasir {
namespace Flash {
    /// A flash operation (Detail::flashErase / flashWrite, Bootrom::explicitBuy) as it began.
    struct OpInFlight {
        enum class Kind : std::uint32_t { none = 0, erase = 1, program = 2, buy = 3 };
        std::uint32_t magic;
        Kind          kind;
        std::uint32_t
          offset;   // flash bytes from the start of the flash; 0 for a buy (the boot ROM finds its sector)
        std::uint32_t bytes;   // 0 for a buy
        std::uint64_t
          startUs;   // the TIMER in that run (Rp::Detail::timerRawUs, Port::FreeRunningUs)
    };

    [[nodiscard]] constexpr std::string_view name(OpInFlight::Kind k) {
        switch(k) {
        case OpInFlight::Kind::none:    return "none";
        case OpInFlight::Kind::erase:   return "erase";
        case OpInFlight::Kind::program: return "program";
        case OpInFlight::Kind::buy:     return "explicit_buy";
        }
        return "?";
    }

    /// How long this boot's flash operations took, interrupts masked around each by its caller: the last one, and the
    /// longest with its kind and flash offset. Zero before the first.
    struct OpTimes {
        std::chrono::microseconds last;
        std::chrono::microseconds longest;
        OpInFlight::Kind          longestKind;
        std::uint32_t             longestOffset;
    };

    namespace Detail {
        inline constexpr std::uint32_t opInFlightMagic = 0x504F'4C46U;   // "FLOP"
        // the operation in flight, in .noInit: what a reset in the middle of it leaves for the next boot
        [[KVASIR_NOINIT]] inline OpInFlight opInFlight;
        // the previous run's operation, taken from opInFlight once a boot: by the first opInFlightAtBoot(), at the
        // latest by this boot's first operation, before it writes its own
        inline bool                      atBootTaken{};
        inline std::optional<OpInFlight> atBoot{};
        // microseconds in 32 bits (an operation's time fits; one store each, so a reader never sees half of one)
        inline std::uint32_t    lastOpUs{};
        inline std::uint32_t    longestOpUs{};
        inline std::uint32_t    longestOpOffset{};
        inline OpInFlight::Kind longestOpKind{OpInFlight::Kind::none};

        inline void takeAtBoot() {
            OpInFlight const op = opInFlight;
            opInFlight.magic    = 0;
            atBootTaken         = true;
            bool const known    = op.kind == OpInFlight::Kind::erase
                               || op.kind == OpInFlight::Kind::program
                               || op.kind == OpInFlight::Kind::buy;
            // power-on noise in .noInit fails the magic or the kind
            if(op.magic == opInFlightMagic && known) { atBoot = op; }
        }

        [[gnu::always_inline]] inline std::uint64_t beginOp(OpInFlight::Kind kind,
                                                            std::uint32_t    offset,
                                                            std::uint32_t    bytes) {
            if(!atBootTaken) { takeAtBoot(); }
            auto const now = Kvasir::Rp::Detail::timerRawUs();
            opInFlight     = OpInFlight{.magic   = opInFlightMagic,
                                        .kind    = kind,
                                        .offset  = offset,
                                        .bytes   = bytes,
                                        .startUs = now};
            return now;
        }

        [[gnu::always_inline]] inline void endOp(std::uint64_t startUs) {
            auto const us = static_cast<std::uint32_t>(Kvasir::Rp::Detail::timerRawUs() - startUs);
            opInFlight.magic = 0;
            lastOpUs         = us;
            if(us > longestOpUs) {
                longestOpUs     = us;
                longestOpOffset = opInFlight.offset;
                longestOpKind   = opInFlight.kind;
            }
        }
    }   // namespace Detail

    /// The flash operation the previous run was reset in - a watchdog that bit while it ran with interrupts masked, a
    /// reset from outside - or none: after an operation that ended, after a power-on (magic, kind), and always for an
    /// image a decryption stage loads (the top of this header). The same answer for every call of a boot.
    [[nodiscard]] inline std::optional<OpInFlight> opInFlightAtBoot() {
        if(!Detail::atBootTaken) { Detail::takeAtBoot(); }
        return Detail::atBoot;
    }

    /// This boot's flash operation times (OpTimes).
    [[nodiscard]] inline OpTimes opTimes() {
        return OpTimes{std::chrono::microseconds{Detail::lastOpUs},
                       std::chrono::microseconds{Detail::longestOpUs},
                       Detail::longestOpKind,
                       Detail::longestOpOffset};
    }
}   // namespace Flash

namespace Detail {
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flashEraseImpl(Flash::XipDisabler& xipDisabler,
                   void (*erase)(std::uint32_t,
                                 std::size_t,
                                 std::uint32_t,
                                 std::uint8_t),
                   std::uint32_t offset,
                   std::size_t   blocks) {
        KVASIR_RAM_FUNC_MARK();
        Flash::XipGuard const guard{xipDisabler};
        [[maybe_unused]] Flash::BusyBoot::Guard const
          busyBoot{};   // XIP off first, gone before it is back
        erase(offset, blocks * 4096, 1 << 16, 0xD8);
    }

    static inline void flashErase(std::uint32_t offset,
                                  std::size_t   blocks) {
        auto erase = RomFunctions::getRomFunctionPointer<
          'R',
          'E',
          void (*)(std::uint32_t, std::size_t, std::uint32_t, std::uint8_t)>();

        auto const startUs = Flash::Detail::beginOp(Flash::OpInFlight::Kind::erase,
                                                    offset,
                                                    static_cast<std::uint32_t>(blocks * 4096));
        {
            Flash::XipDisabler xipDisabler{};
            flashEraseImpl(xipDisabler, erase, offset, blocks);
        }
        Flash::Detail::endOp(startUs);
    }

    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void
    flashWriteImpl(Flash::XipDisabler& xipDisabler,
                   void (*write)(std::uint32_t,
                                 std::uint8_t const*,
                                 std::size_t),
                   std::uint32_t       offset,
                   std::uint8_t const* data,
                   std::size_t         size) {
        KVASIR_RAM_FUNC_MARK();
        Flash::XipGuard const guard{xipDisabler};
        [[maybe_unused]] Flash::BusyBoot::Guard const
          busyBoot{};   // XIP off first, gone before it is back
        write(offset, data, size);
    }

    static inline void flashWrite(std::uint32_t       offset,
                                  std::uint8_t const* data,
                                  std::size_t         size) {
        auto write = RomFunctions::getRomFunctionPointer<
          'R',
          'P',
          void (*)(std::uint32_t, std::uint8_t const*, std::size_t)>();

        auto const startUs = Flash::Detail::beginOp(Flash::OpInFlight::Kind::program,
                                                    offset,
                                                    static_cast<std::uint32_t>(size));
        {
            Flash::XipDisabler xipDisabler{};
            flashWriteImpl(xipDisabler, write, offset, data, size);
        }
        Flash::Detail::endOp(startUs);
    }
}   // namespace Detail

namespace Bootrom {
    /// Erase `blocks` 4096-byte sectors at flash `offset` (from the start of flash, not the
    /// XIP window). Runs with XIP disabled; mask interrupts around it.
    inline void flashErase(std::uint32_t offset,
                           std::size_t   blocks) {
        Detail::flashErase(offset, blocks);
    }

    /// Program `size` bytes (a multiple of 256, page aligned `offset`) from `data`. Runs
    /// with XIP disabled; mask interrupts around it.
    inline void flashWrite(std::uint32_t       offset,
                           std::uint8_t const* data,
                           std::size_t         size) {
        Detail::flashWrite(offset, data, size);
    }
}   // namespace Bootrom
}   // namespace Kvasir
