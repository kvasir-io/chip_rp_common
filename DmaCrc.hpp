#pragma once
// The DMA controller's CRC sniffer as Kvasir::DmaCrc: the hardware beside the SDK's software engines (Kvasir::Crc,
// kvasir/Util/Crc.hpp), which take the same Params and give the same CRCs.
//
//   SnifferEngine<P, Dma, ch>   Kvasir::Crc::Engine's shape (update/finish/compute) on a channel of the firmware's DmaBase
//   SnifferCalc<P, Dma, ch>     the `type` + `calc(span)` shape (aglio's packager)
//   Sniffer<Dma, ch>            the image's engine set (Kvasir::Crc::ImageEngines): every run-time CRC a library takes
//                               as Kvasir::Crc::ImageEngine<P> - a CellStore's cells, an SD card's blocks - runs here
//                               when the sniffer can compute P. List it in the Startup list: it claims the channel
//                               (a set named in the hook and in no list does not compile: Startup's list rules).
//   Sniffer<Dma, ch>::Crc32Chain
//                               zlib's crc32(previous, piece) on the same channel, for a store that chains its CRC
//                               over pieces (fs::log's StoreDefaults::Crc); only where the image names that Sniffer
//                               in the hook (and so lists it: its claims), or it does not compile
//   sniff<S, Dma, ch>(bytes)    one byte-transfer pass with any SNIFF_CTRL setting S (also the sum and the parity)
//   armSniffer / sniffResult    for a transfer of your own with `sniff` in its ChannelConfig
//   failures()                  the sniffed transfers that timed out or hit a bus error so far
//
// Every sniffed transfer is waited for with a bound (Detail::SniffBusyBound, OnTimeout::Report): one that does not
// finish is aborted, one that hit a bus error gets its error flags cleared, and the call that ran it fails - sniff()
// and calcCrc() return the Failure, SnifferEngine::failed() says so after update() - or, in the one-shot forms that
// have no way to say it (SnifferEngine::compute, SnifferCalc::calc, Crc32Chain), takes the software for that span, so
// they always return the CRC. failures() counts each one.
//
// The hardware knows two polynomials, each also in a bit-reversed form, plus a sum and a parity; seed, output
// reversal and inversion make them a named CRC (the Sniff presets):
//   crc32       CRC-32 (IEEE 802.3, zlib, PNG)     reflected, seed and final xor all ones
//   crc32Bzip2  CRC-32/BZIP2                       the same polynomial, unreflected
//   crc16Ccitt  CRC-16/CCITT-FALSE                 0x1021 unreflected, seed 0xFFFF
//   crc16X25    CRC-16/X-25 (HDLC, PPP)            0x1021 reflected, seed and xor all ones
//   sum32, parity                                  a 32-bit sum, a single parity bit
//
// One sniffer per DMA controller (SNIFF_CTRL, SNIFF_DATA; RP2040 datasheet 2.5.5.2 and Tables 146/147, RP2350 12.6.8.2
// and Tables 1179/1180): it watches one channel, and SNIFF_DATA "will update ... each time it observes a read from the
// indicated channel" (Table 147 md l.5731, Table 1180 md l.55336), so it keeps its sum across transfers. Every user
// arms it, so two users that interleave corrupt each other whatever their channels. Listed in a Startup list, each
// user provides the sniffer (SnifferUse below): users that are done with it when their call returns agree; one that
// keeps its sum across calls (ImageCheck::RpStream's pass) collides with any other. That the users of the image's
// engine (library calls, not list entries) never interleave - one context, one core - is a debug check
// (DmaCrcHold.hpp): a SnifferEngine, sniff() or Crc32Chain armed while another one holds the sniffer panics there.
#include "DMA.hpp"
#include "DmaCrcHold.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Register/Wait.hpp"
#include "kvasir/Runtime/CrcImageEngine.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/Crc.hpp"
#include "kvasir/Util/Result.hpp"

#include <algorithm>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <type_traits>

namespace Kvasir::DmaCrc {

enum class Calc : std::uint32_t {
    crc32  = 0,
    crc32r = 1,
    crc16  = 2,
    crc16r = 3,
    even   = 14,
    sum    = 15
};

struct Sniff {
    Calc          calc;
    std::uint32_t seed;
    bool          reverseResult;      // SNIFF_CTRL.OUT_REV
    bool          invertResult;       // SNIFF_CTRL.OUT_INV
    bool          byteSwap = false;   // SNIFF_CTRL.BSWAP: the data byte-reversed before the sum

    static constexpr Sniff crc32() { return {Calc::crc32r, 0xFFFF'FFFFU, true, true}; }

    static constexpr Sniff crc32Bzip2() { return {Calc::crc32, 0xFFFF'FFFFU, false, true}; }

    static constexpr Sniff crc16Ccitt() { return {Calc::crc16, 0xFFFFU, false, false}; }

    static constexpr Sniff crc16X25() { return {Calc::crc16r, 0xFFFFU, true, true}; }

    static constexpr Sniff sum32() { return {Calc::sum, 0U, false, false}; }

    static constexpr Sniff parity() { return {Calc::even, 0U, false, false}; }

    [[nodiscard]] constexpr bool is16Bit() const {
        return calc == Calc::crc16 || calc == Calc::crc16r;
    }

    friend constexpr bool operator==(Sniff const&,
                                     Sniff const&) = default;
};

// calcCrc's two fixed CRCs: crc16 is CRC-16/CCITT-FALSE, crc32 the IEEE 802.3 CRC-32.
enum class CrcType { crc16, crc32 };

/// Why a sniffed transfer failed: its channel's BUSY did not fall within Detail::SniffBusyBound (the channel is
/// aborted), or the channel hit a bus error (CTRL_TRIG.AHB_ERROR: "the channel halts when it encounters any bus error",
/// RP2040 datasheet Table 124 md l.5410; RP2350 12.6.7.1 md l.54083) - the flags are cleared, or "the DMA refuses to
/// restart the offending channel" (RP2350 md l.54085). Either way the sum in SNIFF_DATA is not the data's.
enum class Failure : std::uint8_t { timeout, busError };

/// The sniffed transfers of the image that failed so far, by reason.
struct Failures {
    std::uint32_t timeouts{};
    std::uint32_t busErrors{};
};

namespace Detail { inline constinit Failures failureCounts{}; }   // namespace Detail

[[nodiscard]] inline Failures failures() { return Detail::failureCounts; }

// The sniffer as a Startup resource (kvasir/StartUp/Resources.hpp): keyed by the controller, the payload says how
// it is used. A user that arms it per call and is done when the call returns provides SnifferUse<perCall>; any number
// of those agree (mergeIdentical) - they must not preempt each other: one context, sequential calls (a debug check,
// DmaCrcHold.hpp). A user that keeps its sum in SNIFF_DATA across calls provides SnifferUse<heldBy(channel)>, which
// any other user's provide collides with: "a hardware resource is provided by two peripherals ... DMA CRC sniffer".
struct SnifferTag {
    static constexpr bool        mergeIdentical = true;
    static constexpr bool        coreLocal      = false;
    static constexpr char const* describeAs     = "DMA CRC sniffer (SNIFF_CTRL/SNIFF_DATA)";
};

inline constexpr unsigned perCall = 0;

template<typename Channel>
constexpr unsigned heldBy(Channel channel) {
    return 1U + static_cast<unsigned>(channel);
}

template<unsigned Use>
using SnifferUse = Kvasir::Startup::Resource<SnifferTag, 0, Use>;

/// Point the sniffer at `channel` with the seed and the result transforms of `S`. The
/// channel's own transfers need `sniff_en` in their CTRL (DMA::ChannelConfig::sniff).
template<Sniff S,
         typename DMA,
         typename DMA::Channel Channel>
static inline void armSniffer() {
    using SC = typename DMA::Regs::SNIFF_CTRL;
    using Kvasir::Register::value;
    apply(write(DMA::Regs::SNIFF_DATA::sniff_data, value<S.seed>()));
    apply(SC::overrideDefaults(
      write(SC::calc, value<typename SC::CALCVal, static_cast<typename SC::CALCVal>(S.calc)>()),
      write(SC::dmach, value<static_cast<std::uint32_t>(Channel)>()),
      write(SC::bswap, value<S.byteSwap ? 1U : 0U>()),
      write(SC::out_rev, value<S.reverseResult ? 1U : 0U>()),
      write(SC::out_inv, value<S.invertResult ? 1U : 0U>()),
      set(SC::en)));
}

/// What the sniffer has accumulated so far, transformed as configured: SNIFF_DATA as read.
template<typename DMA>
[[nodiscard]] static inline std::uint32_t sniffResult() {
    return get<0>(apply(read(DMA::Regs::SNIFF_DATA::sniff_data)));
}

/// The same, as the CRC `S` describes. OUT_REV reverses all 32 bits of SNIFF_DATA also in the
/// CRC-16 modes, so a reflected 16-bit result is in the upper half (measured on an RP2350; the
/// datasheets do not say).
template<Sniff S,
         typename DMA>
[[nodiscard]] static inline std::uint32_t sniffResult() {
    std::uint32_t const raw = sniffResult<DMA>();
    return S.is16Bit() && S.reverseResult ? raw >> 16U : raw;
}

namespace Detail {
    // One start moves at most this many transfers, so every wait has the one bound below.
    inline constexpr std::uint32_t maxTransfersPerStart = 4096;

    // A sniffed transfer's CTRL_TRIG.BUSY: neither datasheet gives a transfer time - it depends on the source (SRAM,
    // the XIP cache, the flash behind it) and on the channels that share the bus. A choice: 10 ms for at most
    // MaxTransfersPerStart transfers, about 170 times the slowest measured (4096 byte transfers from flash, ~1.8
    // clk_sys a byte on both chips: ~60 us at 125 MHz). The bound is polls at the chip's ceiling clock with the
    // poll margin (kvasir/Register/Wait.hpp): 1.33 M polls on the RP2350 (266 MHz), 1.0 M on the RP2040 (200 MHz), so
    // a wait that fails stalls its caller longer than 10 ms at a lower clock - ~55 ms measured at 150 MHz on the
    // RP2350. An image whose watchdog period is not well above that resets where it would have reported the failure.
    using SniffBusyBound = Kvasir::Register::MicrosecondsBound<10'000>;

    // The dummy destination of the sniffed transfers on `channel`, never read: static, one word a channel, so a
    // channel that does not come to rest when it is aborted (DmaBase::abort's false: "restarting it is still not
    // safe") goes on writing into a word of its own, never into a stack frame that has gone.
    template<typename DMA, typename DMA::Channel Channel>
    inline constinit std::uint32_t sniffSink{};

    /// One sniffed memory transfer of `count` transfers of `Size` from `from` into the channel's sniffSink, waited for.
    /// Waits on the channel's BUSY bit, not on its completion interrupt, so it works with interrupts masked, inside
    /// an ISR, and with a DmaBase without callback slots; the completion flag it leaves is cleared by that DmaBase's
    /// ISR. `from` is aligned to the size (RP2040 datasheet 2.5.1.1 md l.4650, RP2350 12.6.2.1 md l.53708). A
    /// transfer whose BUSY stays up past SniffBusyBound is aborted (DmaBase::abort); one that hit a bus error halted
    /// with BUSY low - "poll for a low BUSY status" first, then "clear the error flags by writing 1 to each flag"
    /// (RP2350 12.6.7.2 md l.54097-54103; the RP2040 datasheet says only that the channel halts, Table 124 md l.5410,
    /// so a BUSY that stays up there is the timeout's case, and its flags are cleared after the abort). EN goes to 0 in
    /// the same write, so it starts nothing (a trigger write with EN 0 does not start the channel: RP2040 2.5.2.1 md
    /// l.4742-4744, RP2350 12.6.3 md l.53817-53819); the next start writes the whole CTRL again.
    template<typename DMA,
             typename DMA::Channel      Channel,
             typename DMA::TransferSize Size>
    [[nodiscard]] inline std::optional<Failure> sniffed(std::uint32_t from,
                                                        std::uint32_t count) {
        using Ctrl = typename DMA::Regs::template CH<static_cast<unsigned>(Channel)>::CTRL_TRIG;
        constexpr std::uint32_t step = 1U << static_cast<unsigned>(Size);
        while(count != 0) {
            std::uint32_t const n = std::min(count, maxTransfersPerStart);
            DMA::template start<Channel,
                                ::Kvasir::DMA::ChannelConfig{.priority = DMA::Priority::low,
                                                             .trigger
                                                             = DMA::TriggerSource::permanent,
                                                             .size           = Size,
                                                             .incrementRead  = true,
                                                             .incrementWrite = false,
                                                             .sniff          = true}>(
              reinterpret_cast<std::uint32_t>(std::addressof(sniffSink<DMA, Channel>)),
              from,
              n);
            bool const done = static_cast<bool>(
              Kvasir::Register::waitUntil<SniffBusyBound, Kvasir::Register::OnTimeout::Report>(
                Kvasir::Register::isClear(Ctrl::busy)));
            if(!done) { static_cast<void>(DMA::template abort<Channel>()); }
            if(get<0>(apply(read(Ctrl::ahb_error))) != 0) {
                apply(clear(Ctrl::en), reset(Ctrl::read_error, Ctrl::write_error));
                failureCounts.busErrors = failureCounts.busErrors + 1;
                return Failure::busError;
            }
            if(!done) {
                failureCounts.timeouts = failureCounts.timeouts + 1;
                return Failure::timeout;
            }
            from += n * step;
            count -= n;
        }
        return std::nullopt;
    }

    // A transfer costs ~75 (M33) / ~100 (M0+) cycles to start and wait for; a byte costs 1.25 cycles as a byte
    // transfer and 0.31 inside a word transfer (measured on both chips). The words pay for the extra transfers an
    // unaligned head or tail needs from this many bytes each.
    inline constexpr std::uint32_t wordsPayFrom = 128;

    /// `data` through the armed sniffer on `channel`: the word-aligned body in 32-bit transfers where that pays,
    /// the rest in 8-bit ones. The sum goes on across the transfers (SNIFF_DATA above), and every CRC mode takes a
    /// word's bytes in memory order without SNIFF_CTRL.BSWAP (measured on both chips; the datasheets do not say), so
    /// the result is the byte-transfer one. The first transfer that fails ends it.
    template<typename DMA,
             typename DMA::Channel Channel>
    [[nodiscard]] inline std::optional<Failure> feed(std::span<std::byte const> data) {
        auto const          a     = reinterpret_cast<std::uint32_t>(data.data());
        auto const          n     = static_cast<std::uint32_t>(data.size());
        std::uint32_t const head  = std::min<std::uint32_t>((4U - (a & 3U)) & 3U, n);
        std::uint32_t const words = (n - head) / 4U;
        std::uint32_t const tail  = n - head - 4U * words;
        std::uint32_t const extra = (head != 0 ? 1U : 0U) + (tail != 0 ? 1U : 0U);
        if(words == 0 || 4U * words < wordsPayFrom * extra) {
            if(n == 0) { return std::nullopt; }
            return sniffed<DMA, Channel, DMA::TransferSize::size8>(a, n);
        }
        if(head != 0) {
            if(auto const f = sniffed<DMA, Channel, DMA::TransferSize::size8>(a, head)) {
                return f;
            }
        }
        if(auto const f = sniffed<DMA, Channel, DMA::TransferSize::size32>(a + head, words)) {
            return f;
        }
        if(tail == 0) { return std::nullopt; }
        return sniffed<DMA, Channel, DMA::TransferSize::size8>(a + head + 4U * words, tail);
    }

    /// Whether the sniffer still holds the sum that reads as `result` under `S` on `channel`: SNIFF_CTRL is that
    /// setting, and SNIFF_DATA, through its transforms, is `result`. OUT_REV and OUT_INV apply on read only (RP2040
    /// 2.5.5.2, RP2350 12.6.8.2) and are one-to-one, so one result is one accumulator state: whoever left it there, the
    /// sum goes on from it as from `result`.
    template<Sniff S,
             typename DMA,
             typename DMA::Channel Channel>
    [[nodiscard]] inline bool holds(std::uint32_t result) {
        using SC = typename DMA::Regs::SNIFF_CTRL;
        auto const r
          = apply(read(SC::calc, SC::dmach, SC::en, SC::bswap, SC::out_rev, SC::out_inv));
        return r.template get<0>() == static_cast<typename SC::CALCVal>(S.calc)
            && r.template get<1>() == static_cast<std::uint32_t>(Channel)
            && r.template get<2>() != 0 && (r.template get<3>() != 0) == S.byteSwap
            && (r.template get<4>() != 0) == S.reverseResult
            && (r.template get<5>() != 0) == S.invertResult && sniffResult<S, DMA>() == result;
    }
}   // namespace Detail

/// Run `channel` over `data` (one byte at a time into a dummy destination) with the sniffer
/// armed as `s`, and return the result. Blocks for the transfer, which runs at bus speed:
/// a few microseconds per kilobyte. Polls BUSY (Detail::sniffed): works with interrupts masked and inside an ISR.
/// A transfer that fails is the Failure: what the sniffer holds then is not the data's.
template<Sniff S,
         typename DMA,
         typename DMA::Channel Channel>
[[nodiscard]] static inline Kvasir::Result<std::uint32_t,
                                           Failure>
sniff(std::span<std::byte const> data) {
    // Ad hoc use of a channel from a free function: nothing to put in a Startup list,
    // so only the local check applies. The caller keeps this channel clear of drivers,
    // or lists a SnifferCalc / Sniffer on it, whose Claims makes Startup check that.
    static_assert(DMA::ownsChannel(Channel),
                  "the sniffer's DMA channel is not one of this DmaBase's channels");
    Detail::Held const hold{};
    armSniffer<S, DMA, Channel>();
    if(!data.empty()) {
        if(auto const f = Detail::sniffed<DMA, Channel, DMA::TransferSize::size8>(
             reinterpret_cast<std::uint32_t>(data.data()),
             static_cast<std::uint32_t>(data.size())))
        {
            return std::unexpected{*f};
        }
    }
    return sniffResult<S, DMA>();
}

template<CrcType Type,
         typename DMA,
         typename DMA::Channel Channel>
static inline auto calcCrc(std::span<std::byte const> data) {
    if constexpr(Type == CrcType::crc16) {
        return sniff<Sniff::crc16Ccitt(), DMA, Channel>(data).transform(
          [](std::uint32_t v) { return static_cast<std::uint16_t>(v); });
    } else {
        return sniff<Sniff::crc32(), DMA, Channel>(data);
    }
}

/// Whether the sniffer computes the CRC `P` describes (a Kvasir::Crc::Params): CRC-32 (0x04C11DB7) or CRC-16 (0x1021),
/// each MSB- or LSB-first, seed and xorout 0 or all ones - what SNIFF_CTRL.CALC, OUT_REV and OUT_INV can express
/// (RP2040 2.5.5.2 and Table 146, RP2350 12.6.8.2 and Table 1179). How SNIFF_DATA holds any other seed in the
/// reflected modes is not documented.
template<auto P>
consteval bool sniffable() {
    using T          = std::remove_cvref_t<decltype(P.poly)>;
    T const    all   = Kvasir::Crc::widthMask<T>(P.width);
    bool const crc32 = P.width == 32 && P.poly == 0x04C1'1DB7U;
    bool const crc16 = P.width == 16 && P.poly == 0x1021U;
    return (crc32 || crc16) && P.refin == P.refout && (P.init == 0 || P.init == all)
        && (P.xorout == 0 || P.xorout == all);
}

/// The sniffer setting that computes `P`. A CRC the sniffer cannot do does not compile, the reason in the message.
template<auto P>
consteval Sniff sniffFor() {
    using T                 = std::remove_cvref_t<decltype(P.poly)>;
    constexpr bool    crc32 = P.width == 32 && P.poly == 0x04C1'1DB7U;
    constexpr bool    crc16 = P.width == 16 && P.poly == 0x1021U;
    constexpr T const all   = Kvasir::Crc::widthMask<T>(P.width);
    static_assert(crc32 || crc16,
                  "the DMA sniffer computes CRC-32 (0x04C11DB7) and CRC-16 (0x1021) only");
    static_assert(P.refin == P.refout,
                  "the sniffer reflects input and output together (CALC ...R with OUT_REV)");
    static_assert(P.init == 0 || P.init == all,
                  "the sniffer takes seed 0 or all ones: how SNIFF_DATA holds any other seed in "
                  "the reflected modes is not documented");
    static_assert(P.xorout == 0 || P.xorout == all, "xorout 0 or all ones (OUT_INV)");
    static_assert(sniffable<P>());
    return Sniff{.calc          = crc32 ? (P.refin ? Calc::crc32r : Calc::crc32)
                                        : (P.refin ? Calc::crc16r : Calc::crc16),
                 .seed          = static_cast<std::uint32_t>(P.init),
                 .reverseResult = P.refout,
                 .invertResult  = P.xorout != 0};
}

static_assert(sniffFor<Kvasir::Crc::Presets::crc32IsoHdlc>() == Sniff::crc32());
static_assert(sniffFor<Kvasir::Crc::Presets::crc32Bzip2>() == Sniff::crc32Bzip2());
static_assert(sniffFor<Kvasir::Crc::Presets::crc16Ibm3740>() == Sniff::crc16Ccitt());
static_assert(sniffFor<Kvasir::Crc::Presets::crc16X25>() == Sniff::crc16X25());
static_assert(!sniffable<Kvasir::Crc::Presets::crc8Smbus>()
              && !sniffable<Kvasir::Crc::Presets::crc32c>());

/// Kvasir::Crc::Engine's shape over the sniffer: the same result as the software engine for `P`, computed by
/// `channel` of `DMA` at bus speed. The constructor arms the sniffer and the engine holds it until it is destroyed
/// (finish() may be read part way): one engine at a time, in one context (the sum lives in SNIFF_DATA, not in the
/// object; a second user meanwhile is a debug check, DmaCrcHold.hpp). update() blocks for its transfers
/// (Detail::feed), so it works with interrupts masked; a transfer that fails makes failed() true, finish() is not the
/// CRC then and later update()s feed nothing - the caller treats it as a failed read. compute() is one engine over one
/// span and is the CRC whatever happens: where the transfer fails, the software (Kvasir::Crc::Engine<P, 0>, bitwise:
/// no table for a path that should never run) computes that span. Run time only: a constant expression takes
/// Kvasir::Crc::Engine<P> (Kvasir::Crc::ImageEngine does).
template<auto P, typename DMA, typename DMA::Channel Channel>
class SnifferEngine {
    static_assert(DMA::ownsChannel(Channel),
                  "the sniffer's DMA channel is not one of this DmaBase's channels");

public:
    using value_type               = std::remove_cvref_t<decltype(P.poly)>;
    static constexpr auto  params  = P;
    static constexpr Sniff setting = sniffFor<P>();

    SnifferEngine() { armSniffer<setting, DMA, Channel>(); }

    SnifferEngine(SnifferEngine const&)            = delete;
    SnifferEngine& operator=(SnifferEngine const&) = delete;

    SnifferEngine& update(std::span<std::byte const> data) {
        if(!failure_) { failure_ = Detail::feed<DMA, Channel>(data); }
        return *this;
    }

    /// Whether a transfer of an update() failed (failure() says why): finish() is not the CRC.
    [[nodiscard]] bool failed() const { return failure_.has_value(); }

    [[nodiscard]] std::optional<Failure> failure() const { return failure_; }

    /// The CRC of everything so far; update() may go on.
    [[nodiscard]] value_type finish() const {
        return static_cast<value_type>(sniffResult<setting, DMA>());
    }

    [[nodiscard]] static value_type compute(std::span<std::byte const> data) {
        SnifferEngine e{};
        e.update(data);
        if(e.failed()) { return Kvasir::Crc::Engine<P, 0>::compute(data); }
        return e.finish();
    }

private:
    // first: taken before the constructor arms the sniffer, given after the engine is gone
    [[no_unique_address]] Detail::Held hold_{};
    std::optional<Failure>             failure_{};
};

/// The `type` + `calc(span)` shape (Kvasir::Crc::Calc's) over the DMA sniffer: SnifferEngine<P, DMA, channel>.
template<auto P, typename DMA, typename DMA::Channel Channel>
struct SnifferCalc {
    using type = std::remove_cvref_t<decltype(P.poly)>;

    // The channel runs every calc(): listed in a Startup list, this reserves it across the image, and the sniffer
    // for calls that are done when they return.
    using Claims   = Kvasir::DMA::Claims<DMA, Channel>;
    using Provides = brigand::list<SnifferUse<perCall>>;

    static type calc(std::span<std::byte const> data) {
        return SnifferEngine<P, DMA, Channel>::compute(data);
    }
};

template<typename DMA, typename DMA::Channel Channel>
struct Crc32Chain;

/// The image's CRC engines on the sniffer (the hook Kvasir::Crc::ImageEngines, kvasir/Runtime/CrcImageEngine.hpp):
/// every CRC the sniffer computes (sniffable<P>) runs on `channel` of `DMA`, any other on the software engine. In the
/// Startup list it claims the channel and the sniffer for calls that are done when they return - the CRCs of the
/// libraries that take Kvasir::Crc::ImageEngine are such calls, from the main loop. An RpStream image check in the same
/// list collides with it (the check keeps its sum in the sniffer across turns).
template<typename DMA, typename DMA::Channel Channel>
struct Sniffer {
    static_assert(DMA::ownsChannel(Channel),
                  "the sniffer's DMA channel is not one of this DmaBase's channels");

    using Claims   = Kvasir::DMA::Claims<DMA, Channel>;
    using Provides = brigand::list<SnifferUse<perCall>>;

    template<auto P, std::size_t TableSize>
    using For = std::conditional_t<sniffable<P>(),
                                   SnifferEngine<P, DMA, Channel>,
                                   Kvasir::Crc::Engine<P, TableSize>>;

    /// zlib's crc32(previous, piece) on this set's channel (DmaCrc::Crc32Chain below): fs::log::StoreDefaults::Crc's
    /// shape, `using Crc = ...::Crc32Chain;`.
    using Crc32Chain = DmaCrc::Crc32Chain<DMA, Channel>;
};

/// zlib's crc32(previous, piece) - CRC-32/ISO-HDLC, crc32(crc32(0, a), b) == crc32(0, a + b) - on `channel` of `DMA`,
/// for a store that chains its CRC over pieces read one after the other (fs::log::StoreDefaults::Crc: `using Crc =
/// Sniffer<Dma, ch>::Crc32Chain;`). It runs on the channel the image's engine set claims and is done with the sniffer
/// when it returns (a perCall user): it compiles only where the image names Sniffer<DMA, channel> in the hook
/// Kvasir::Crc::ImageEngines<Tag::User> - which then must be in a Startup list (Startup's list rules), so the channel
/// and the sniffer are claimed; named elsewhere, nothing would keep a driver or an RpStream check off them. A class of
/// its own, not the set's member: an explicit instantiation of the set (a conformance tree) reaches every member's
/// body, and this check holds only in an image. previous == 0 starts a CRC (arms the sniffer); a previous the sniffer
/// still holds (Detail::holds: this setting on this channel, SNIFF_DATA reading as previous - the piece before it was
/// this function's) goes on in SNIFF_DATA. Any other previous - a CRC from storage, or the sniffer used by another user
/// since - and a transfer that fails go on in the software from previous (Kvasir::Crc::Crc32::resume), so the result is
/// always the CRC: the sniffer is never seeded with a value other than its documented ones (sniffable).
template<typename DMA, typename DMA::Channel Channel>
struct Crc32Chain {
    static std::uint32_t crc32(std::uint32_t              previous,
                               std::span<std::byte const> piece) {
        static_assert(
          std::is_same_v<
            typename Kvasir::Port::LateT<Kvasir::Crc::ImageEngines<Kvasir::Tag::User>, DMA>::type,
            Sniffer<DMA, Channel>>,
          "Kvasir::DmaCrc::Crc32Chain runs on the image's CRC engine set: name "
          "Kvasir::DmaCrc::Sniffer<Dma, channel> "
          "in the hook Kvasir::Crc::ImageEngines<Kvasir::Tag::User> and list it in the Startup "
          "list - that claims the "
          "channel and the sniffer");
        constexpr Sniff    s = sniffFor<Kvasir::Crc::Presets::crc32IsoHdlc>();
        Detail::Held const hold{};
        if(previous == 0) {
            armSniffer<s, DMA, Channel>();
        } else if(!Detail::holds<s, DMA, Channel>(previous)) {
            return software(previous, piece);
        }
        if(Detail::feed<DMA, Channel>(piece)) { return software(previous, piece); }
        return sniffResult<s, DMA>();
    }

private:
    static std::uint32_t software(std::uint32_t              previous,
                                  std::span<std::byte const> piece) {
        return Kvasir::Crc::Crc32::resume(previous).update(piece).finish();
    }
};

}   // namespace Kvasir::DmaCrc
