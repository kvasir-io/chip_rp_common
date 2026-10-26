#pragma once
// The ADC pacer's divider (DIV.INT / DIV.FRAC) for a sample rate, and the rate it gives; the channel table and the
// channel-attribution check both free-running modes (ADC.hpp ADCDmaBehavior, ADCIsrBehavior) share. Register- and
// log-free, so it is host-testable (tests/adc_math_test.cpp) and constant-evaluable; ADC.hpp feeds it register reads.
#include <cstddef>
#include <cstdint>
#include <ratio>

namespace Kvasir { namespace ADC { namespace Detail {
    // ADC sample rate divider calculation
    // Total period in ADC clocks = 1 + INT + FRAC/256, at least 97 (see calcDivRegs), or 96
    // back to back with DIV = 0
    // sample_rate = clockSpeed / period
    // When DIV register is 0 the ADC runs at its maximum rate (96 cycles per conversion).

    struct DivRegs {
        std::uint16_t int_;
        std::uint8_t  frac;
    };

    constexpr DivRegs calcDivRegs(std::uint32_t clockSpeed,
                                  std::uint32_t sampleRate) {
        // target period in ADC clocks (fixed-point: 8 fractional bits)
        // period_fp = clockSpeed * 256 / sampleRate
        std::uint64_t periodFp = (static_cast<std::uint64_t>(clockSpeed) * 256U) / sampleRate;

        // subtract 1 (in fixed-point: 256) to get the divider value
        // DIV register encodes: period = 1 + INT + FRAC/256
        if(periodFp < 96U * 256U + 128U) {
            // One conversion's time (to the nearer clock) or less: DIV = 0, back to back
            // (RP2040 datasheet 4.9.2.2, RP2350 12.4.3.2). Not DIV.INT = 95 (a period of
            // 96): the pacer then fires as the last conversion ends, is "ignored if a
            // conversion is currently in progress", and only every other one starts.
            return {0, 0};
        }
        // The shortest paced period that is not skipped is 97 clocks (DIV.INT = 96).
        // Measured at clk_adc 48 MHz, RP2350 and RP2040 alike: INT 95 gave 249 / 251 kS/s,
        // INT 96 and 97 the expected ~495 kS/s. A period between 96 and 97 (FRAC) would mix in
        // 96s. Neither datasheet says how far apart the pacer must be (4.9.2.2 / 12.4.3.2).
        if(periodFp < std::uint64_t{97} * 256U) { return {96, 0}; }
        std::uint64_t divFp = periodFp - 256U;

        std::uint32_t intPart  = static_cast<std::uint32_t>(divFp >> 8);
        std::uint8_t  fracPart = static_cast<std::uint8_t>(divFp & 0xFFU);

        // clamp INT to 16-bit max
        if(intPart > 0xFFFFU) {
            intPart  = 0xFFFFU;
            fracPart = 0xFFU;
        }

        return {static_cast<std::uint16_t>(intPart), fracPart};
    }

    constexpr double calcActualSampleRate(std::uint32_t clockSpeed,
                                          DivRegs       div) {
        if(div.int_ == 0 && div.frac == 0) {
            // DIV=0: 96 cycles per conversion
            return static_cast<double>(clockSpeed) / 96.0;
        }
        // calcDivRegs never paces faster than 97 clocks, so this is the real period.
        double const period
          = 1.0 + static_cast<double>(div.int_) + static_cast<double>(div.frac) / 256.0;
        return static_cast<double>(clockSpeed) / period;
    }

    template<std::uint32_t ClockSpeed,
             std::uint32_t SampleRate,
             std::intmax_t Num,
             std::intmax_t Denom>
    constexpr bool isValidSampleRateConfig(std::ratio<Num,
                                                      Denom>) {
        constexpr auto div    = calcDivRegs(ClockSpeed, SampleRate);
        constexpr auto actual = calcActualSampleRate(ClockSpeed, div);
        constexpr auto err    = actual - static_cast<double>(SampleRate);
        constexpr auto absErr = err > 0.0 ? err : -err;
        constexpr auto maxErr = static_cast<double>(SampleRate)
                              * (static_cast<double>(Num) / static_cast<double>(Denom));
        return absErr <= maxErr;
    }

    /// The FIFO's depth (RP2040 datasheet 4.9.2.4, RP2350 12.4.3.4): one ISR-mode batch at most.
    inline constexpr std::size_t fifoDepth = 8;

    /// The largest FCS.LEVEL can read: bits 19:16 (RP2040 datasheet Table 570, RP2350 Table 1123).
    inline constexpr std::size_t levelFieldMax = 15;

    /// How far past a slot ChannelTable::near() / slotAfter() reach: a batch plus LEVEL as read,
    /// the largest count an ISR-mode snapshot can give - so they need no range check.
    inline constexpr std::size_t nearReach = fifoDepth + levelFieldMax;

    /// The channels a configuration samples, in round-robin order: the set bits of `mask`
    /// from channel 0 up (RROBIN cycles "to the next input whose bit is set", RP2040 datasheet
    /// 4.9.2.3, RP2350 12.4.3.3).
    template<std::size_t N>
    struct ChannelTable {
        // Built-in arrays, not std::array: its operator[] is a call in the sanitize images (libc++
        // hardening), the interrupt reads these per sample; -fsanitize=bounds checks these inline.
        std::uint8_t channels[N]{};
        // channels[k % N] and k % N for k < N + NearReach: near() and slotAfter() are one load,
        // no division (the RP2040's M0+ has no divide instruction; `%` by a run-time value that
        // is not a power of two calls the runtime's division)
        std::uint8_t unrolled[N + nearReach]{};
        std::uint8_t slotOf[N + nearReach]{};

        /// The channel of the sample `i` places after one in slot `firstSlot`.
        [[nodiscard,
          gnu::always_inline]] constexpr std::uint8_t
        at(std::size_t firstSlot,
           std::size_t i) const {
            return channels[(firstSlot + i) % N];
        }

        /// at() for firstSlot < N and i <= NearReach, by table.
        [[nodiscard,
          gnu::always_inline]] constexpr std::uint8_t
        near(std::size_t firstSlot,
             std::size_t i) const {
            return unrolled[firstSlot + i];
        }

        /// The slot `k` places after `firstSlot` (< N), for k <= NearReach, by table.
        [[nodiscard,
          gnu::always_inline]] constexpr std::size_t
        slotAfter(std::size_t firstSlot,
                  std::size_t k) const {
            return slotOf[firstSlot + k];
        }

        /// The slot `K` places after `firstSlot` (< N), for any constant K: one compare.
        template<std::size_t K>
        [[nodiscard,
          gnu::always_inline]] static constexpr std::size_t
        advance(std::size_t firstSlot) {
            std::size_t const s = firstSlot + K % N;
            return s >= N ? s - N : s;
        }
    };

    template<std::size_t N>
    constexpr ChannelTable<N> buildChannelTable(std::uint32_t mask) {
        ChannelTable<N> t{};
        std::size_t     idx = 0;
        for(std::uint8_t i = 0; i < 9 && idx < N; ++i) {
            if(mask & (1U << i)) { t.channels[idx++] = i; }
        }
        for(std::size_t k = 0; k < N + nearReach; ++k) {
            t.unrolled[k] = t.channels[k % N];
            t.slotOf[k]   = static_cast<std::uint8_t>(k % N);
        }
        return t;
    }

    /// What a channel-attribution check found.
    enum class Alignment : std::uint8_t { aligned, overrun, misaligned };

    /// One look at the hardware for checkAttribution().
    struct AttributionSnapshot {
        bool          flagged;   // FCS.OVER or FCS.UNDER: samples lost or a read of an empty FIFO
        bool          stable;    // the counts read the same before and after AINSEL
        std::size_t   moved;     // samples converted since slot `firstSlot` was the next one
        std::uint32_t ainsel;    // CS.AINSEL
    };

    /// The FIFO carries no channel number (RP2040 datasheet Table 571, RP2350 Table 1124), so a
    /// stream's attribution is a count; this checks the count against the hardware. The round
    /// robin moves AINSEL on "when the ADC completes a conversion" (RP2040 datasheet 4.9.2.3,
    /// RP2350 12.4.3.3; RROBIN in Table 568 / 1121: "updated after each conversion"), so with
    /// nothing lost or gained AINSEL names the channel `moved` samples after `firstSlot` - also
    /// while the next conversion runs, whose channel it is. READY is not asked: at DIV = 0 the next
    /// conversion starts as the last one ends (START_MANY, Table 568 / 1121), and READY is almost
    /// never 1.
    ///
    /// ADC (and DMA) keep going meanwhile, so `snap()` reads the counts before and after AINSEL
    /// and says whether they held still; an unstable snapshot is taken again. A mismatch counts
    /// only when `MismatchesToConfirm` snapshots show one, so a completion whose FIFO write and
    /// AINSEL step are seen a cycle apart cannot drop good samples. A flag ends the check at once.
    /// No stable snapshot in `Attempts`: nothing proves the samples, misaligned.
    /// What one snapshot says on its own.
    enum class Look : std::uint8_t { aligned, flagged, unstable, mismatch };

    /// One snapshot judged; `expected` is the channel `s.moved` places after the first slot.
    template<std::size_t N>
    [[nodiscard,
      gnu::always_inline]] constexpr Look
    judge(AttributionSnapshot const& s,
          std::uint32_t              expected) {
        if(s.flagged) { return Look::flagged; }
        if constexpr(N == 1) {
            return Look::aligned;   // no round robin: every sample is on the one channel
        } else {
            if(!s.stable) { return Look::unstable; }
            return s.ainsel == expected ? Look::aligned : Look::mismatch;
        }
    }

    /// checkAttribution() after its first snapshot was judged `first` (a driver takes that one
    /// inline, where nearly every run ends, and calls this out of line for the rest): the same
    /// attempts, the same confirmation, the same verdict as checkAttribution() over that
    /// snapshot followed by snap()'s.
    template<int         Attempts            = 16,
             int         MismatchesToConfirm = 2,
             std::size_t N,
             typename Snap>
    [[nodiscard]] constexpr Alignment checkAttributionAfter(ChannelTable<N> const& table,
                                                            std::size_t            firstSlot,
                                                            Look                   first,
                                                            Snap&&                 snap) {
        int mismatches = 0;
        for(int attempt = 0;;) {
            switch(first) {
            case Look::flagged:  return Alignment::overrun;
            case Look::aligned:  return Alignment::aligned;
            case Look::unstable: break;
            case Look::mismatch:
                if(++mismatches == MismatchesToConfirm) { return Alignment::misaligned; }
                break;
            }
            if(++attempt == Attempts) { return Alignment::misaligned; }
            AttributionSnapshot const s = snap();
            first                       = judge<N>(s, table.at(firstSlot, s.moved));
        }
    }

    /// The FIFO carries no channel number (RP2040 datasheet Table 571, RP2350 Table 1124), so a
    /// stream's attribution is a count; this checks the count against the hardware. The round
    /// robin moves AINSEL on "when the ADC completes a conversion" (RP2040 datasheet 4.9.2.3,
    /// RP2350 12.4.3.3; RROBIN in Table 568 / 1121: "updated after each conversion"), so with
    /// nothing lost or gained AINSEL names the channel `moved` samples after `firstSlot` - also
    /// while the next conversion runs, whose channel it is. READY is not asked: at DIV = 0 the next
    /// conversion starts as the last one ends (START_MANY, Table 568 / 1121), and READY is almost
    /// never 1.
    ///
    /// ADC (and DMA) keep going meanwhile, so `snap()` reads the counts before and after AINSEL
    /// and says whether they held still; an unstable snapshot is taken again. A mismatch counts
    /// only when `MismatchesToConfirm` snapshots show one, so a completion whose FIFO write and
    /// AINSEL step are seen a cycle apart cannot drop good samples. A flag ends the check at once.
    /// No stable snapshot in `Attempts`: nothing proves the samples, misaligned.
    template<int         Attempts            = 16,
             int         MismatchesToConfirm = 2,
             std::size_t N,
             typename Snap>
    [[nodiscard]] constexpr Alignment checkAttribution(ChannelTable<N> const& table,
                                                       std::size_t            firstSlot,
                                                       Snap&&                 snap) {
        AttributionSnapshot const s = snap();
        return checkAttributionAfter<Attempts, MismatchesToConfirm>(
          table,
          firstSlot,
          judge<N>(s, table.at(firstSlot, s.moved)),
          snap);
    }
}}}   // namespace Kvasir::ADC::Detail
