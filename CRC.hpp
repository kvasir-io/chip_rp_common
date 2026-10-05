#pragma once
#include "kvasir/Util/Crc.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

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

        friend constexpr bool operator==(Sniff const&,
                                         Sniff const&) = default;
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

    /// Run `channel` over `data` (one byte at a time into a dummy destination) with the sniffer
    /// armed as `s`, and return the result. Blocks for the transfer, which runs at bus speed:
    /// a few microseconds per kilobyte. Waits on the channel's BUSY bit, not on its completion
    /// interrupt, so it works with interrupts masked, inside an ISR, and with a DmaBase without
    /// callback slots; the completion flag it leaves is cleared by that DmaBase's ISR.
    template<Sniff S,
             typename DMA,
             typename DMA::Channel channel>
    [[nodiscard]] static inline std::uint32_t sniff(std::span<std::byte const> data) {
        // Ad hoc use of a channel from a free function: nothing to put in a Startup list,
        // so only the local check applies. The caller keeps this channel clear of drivers.
        static_assert(DMA::ownsChannel(channel),
                      "the sniffer's DMA channel is not one of this DmaBase's channels");
        armSniffer<S, DMA, channel>();
        if(data.empty()) { return sniffResult<S, DMA>(); }

        std::byte buffer;

        DMA::template start<channel,
                            DMA::Priority::low,
                            DMA::TriggerSource::permanent,
                            DMA::TransferSize::_8,
                            false,
                            true>(reinterpret_cast<std::uint32_t>(std::addressof(buffer)),
                                  reinterpret_cast<std::uint32_t>(data.data()),
                                  data.size());

        while(!DMA::template ready<channel>()) {}

        return sniffResult<S, DMA>();
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

    // Software references, for checking the hardware and for hosts without a sniffer: the SDK's
    // Kvasir::Crc::Engine, bitwise. `crc` continues an earlier result (zlib's crc32(crc, data))
    // and a CRC-16 register (the seed) respectively.
    namespace Software {
        constexpr std::uint32_t crc32(std::span<std::byte const> data,
                                      std::uint32_t              crc = 0) {
            return Kvasir::Crc::Crc32T<0>::resume(crc).update(data).finish();
        }

        constexpr std::uint16_t crc16Ccitt(std::span<std::byte const> data,
                                           std::uint16_t              crc = 0xFFFF) {
            return Kvasir::Crc::Crc16Ccitt<0>::resume(crc).update(data).finish();
        }
    }   // namespace Software

    /// The sniffer setting that computes the CRC `P` describes (a Kvasir::Crc::Params): CRC-32
    /// (0x04C11DB7) or CRC-16 (0x1021), each MSB- or LSB-first, seed and xorout 0 or all ones -
    /// what SNIFF_CTRL.CALC, OUT_REV and OUT_INV can express (RP2040 2.5.5.2 and Table 146,
    /// RP2350 12.6.8.2 and Table 1179). Anything else does not compile.
    template<auto P>
    consteval Sniff sniffFor() {
        using T                 = std::remove_cvref_t<decltype(P.poly)>;
        constexpr bool    crc32 = P.width == 32 && P.poly == 0x04C1'1DB7U;
        constexpr bool    crc16 = P.width == 16 && P.poly == 0x1021U;
        constexpr T const all   = Kvasir::Crc::Detail::mask<T>(P.width);
        static_assert(crc32 || crc16,
                      "the DMA sniffer computes CRC-32 (0x04C11DB7) and CRC-16 (0x1021) only");
        static_assert(P.refin == P.refout,
                      "the sniffer reflects input and output together (CALC ...R with OUT_REV)");
        static_assert(
          P.init == 0 || P.init == all,
          "the sniffer takes seed 0 or all ones: how SNIFF_DATA holds any other seed in "
          "the reflected modes is not documented");
        static_assert(P.xorout == 0 || P.xorout == all, "xorout 0 or all ones (OUT_INV)");
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

    /// The `type` + `calc(span)` shape (Kvasir::Crc::Calc's) over the DMA sniffer: the same result
    /// as the software engine for `P`, computed by `channel` of `DMA` at bus speed.
    template<auto P, typename DMA, typename DMA::Channel channel>
    struct SnifferCalc {
        using type = std::remove_cvref_t<decltype(P.poly)>;

        static type calc(std::span<std::byte const> data) {
            return static_cast<type>(sniff<sniffFor<P>(), DMA, channel>(data));
        }
    };

}}   // namespace Kvasir::CRC
