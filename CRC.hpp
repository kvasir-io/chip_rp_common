#pragma once
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace CRC {

    // The DMA controller's sniffer: a checksum over everything one channel moves, computed
    // in hardware while the transfer runs. calcCrc() / sniff() drive a channel over a buffer
    // into a one-byte dummy destination; a transfer of your own gets the same with `sniff_en`
    // in its CTRL (DMA::ChannelConfig::sniff) and armSniffer() / sniffResult().
    //
    // The hardware knows two polynomials, each also in a bit-reversed form, plus a sum and a
    // parity; seed, output reversal and inversion make them a named CRC (the Sniff presets):
    //   crc32       CRC-32 (IEEE 802.3, zlib, PNG)     reflected, seed and final xor all ones
    //   crc32Bzip2  CRC-32/BZIP2                       the same polynomial, unreflected
    //   crc16Ccitt  CRC-16/CCITT-FALSE (SimpleEeprom)  0x1021 unreflected, seed 0xFFFF
    //   crc16X25    CRC-16/X-25 (HDLC, PPP)            0x1021 reflected, seed and xor all ones
    //   sum32, parity                                  a 32-bit sum, a single parity bit
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

        constexpr bool is16Bit() const { return calc == Calc::crc16 || calc == Calc::crc16r; }
    };

    // The old spelling, kept for SimpleEeprom and its users: crc16 is CRC-16/CCITT-FALSE, crc32
    // the IEEE 802.3 CRC-32.
    enum class CRC_Type { crc16, crc32 };

    /// Point the sniffer at `channel` with the seed and the result transforms of `S`. The
    /// channel's own transfers need `sniff_en` in their CTRL (DMA::ChannelConfig::sniff).
    template<Sniff S,
             typename DMA,
             typename DMA::Channel channel>
    static inline void armSniffer() {
        using SC = typename DMA::Regs::SNIFF_CTRL;
        using Kvasir::Register::value;
        apply(write(DMA::Regs::SNIFF_DATA::sniff_data, value<S.seed>()));
        apply(SC::overrideDefaults(
          write(SC::calc, value<typename SC::CALCVal, static_cast<typename SC::CALCVal>(S.calc)>()),
          write(SC::dmach, value<static_cast<std::uint32_t>(channel)>()),
          write(SC::bswap, value<S.byteSwap ? 1U : 0U>()),
          write(SC::out_rev, value<S.reverseResult ? 1U : 0U>()),
          write(SC::out_inv, value<S.invertResult ? 1U : 0U>()),
          set(SC::en)));
    }

    /// What the sniffer has accumulated so far, transformed as configured.
    template<typename DMA>
    [[nodiscard]] static inline std::uint32_t sniffResult() {
        return get<0>(apply(read(DMA::Regs::SNIFF_DATA::sniff_data)));
    }

    /// Run `channel` over `data` (one byte at a time into a dummy destination) with the sniffer
    /// armed as `s`, and return the result. Blocks for the transfer, which runs at bus speed:
    /// a few microseconds per kilobyte.
    template<Sniff S,
             typename DMA,
             typename DMA::Channel channel>
    [[nodiscard]] static inline std::uint32_t sniff(std::span<std::byte const> data) {
        // Ad hoc use of a channel from a free function: nothing to put in a Startup list,
        // so only the local check applies. The caller keeps this channel clear of drivers.
        static_assert(DMA::ownsChannel(channel),
                      "the sniffer's DMA channel is not one of this DmaBase's channels");
        armSniffer<S, DMA, channel>();
        if(data.empty()) { return sniffResult<DMA>(); }

        std::byte         buffer;
        std::atomic<bool> running = true;

        std::atomic_signal_fence(std::memory_order_release);

        DMA::template start<channel,
                            DMA::Priority::low,
                            DMA::TriggerSource::permanent,
                            DMA::TransferSize::_8,
                            false,
                            true>(reinterpret_cast<std::uint32_t>(std::addressof(buffer)),
                                  reinterpret_cast<std::uint32_t>(data.data()),
                                  data.size(),
                                  [&]() { running = false; });

        while(running) {}

        return sniffResult<DMA>();
    }

    template<CRC_Type type,
             typename DMA,
             typename DMA::Channel channel>
    static inline auto calcCrc(std::span<std::byte const> data) {
        if constexpr(type == CRC_Type::crc16) {
            return static_cast<std::uint16_t>(sniff<Sniff::crc16Ccitt(), DMA, channel>(data));
        } else {
            return sniff<Sniff::crc32(), DMA, channel>(data);
        }
    }

    // Software references, for checking the hardware and for hosts without a sniffer.
    namespace Software {
        constexpr std::uint32_t crc32(std::span<std::byte const> data,
                                      std::uint32_t              crc = 0) {
            crc = ~crc;
            for(auto const b : data) {
                crc ^= static_cast<std::uint32_t>(b);
                for(int i = 0; i < 8; ++i) { crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB8'8320U : 0U); }
            }
            return ~crc;
        }

        constexpr std::uint16_t crc16Ccitt(std::span<std::byte const> data,
                                           std::uint16_t              crc = 0xFFFF) {
            for(auto const b : data) {
                crc ^= static_cast<std::uint16_t>(static_cast<std::uint16_t>(b) << 8);
                for(int i = 0; i < 8; ++i) {
                    std::uint32_t const shifted = static_cast<std::uint32_t>(crc) << 1;
                    crc
                      = static_cast<std::uint16_t>((crc & 0x8000U) ? (shifted ^ 0x1021U) : shifted);
                }
            }
            return crc;
        }
    }   // namespace Software

}}   // namespace Kvasir::CRC
