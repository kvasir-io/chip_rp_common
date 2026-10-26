#pragma once
// The PWM divider/TOP search of pwm.hpp. Register- and log-free (no chip header, no uc_log),
// so it is host-testable and constant-evaluable: a refusal comes back as a reason, and the
// driver's run-time functions log it.
#include <cassert>
#include <cstdint>
#include <optional>
#include <utility>

namespace Kvasir { namespace PWM {
    /// The frequency a divider (8.4 fixed point, `div16` = divider x 16) and a TOP give at `ClockSpeed`.
    template<std::uint32_t ClockSpeed>
    constexpr std::uint32_t calcFrequency(std::uint16_t div16,
                                          std::uint16_t top) {
        return ((ClockSpeed * 16) / (div16)) / (top + 1);
    }
}}   // namespace Kvasir::PWM

namespace Kvasir { namespace PWM { namespace detail {
    enum class DivTopRefusal : std::uint8_t {
        none,
        frequencyTooHigh,    // the divider would be below 1.0
        frequencyTooLow,     // the divider would be 256.0 or more
        minTopUnreachable,   // the TOP found is below topMin
    };

    struct DivTopSearch {
        std::optional<std::pair<std::uint16_t, std::uint16_t>> value;   // (div16, top)
        DivTopRefusal                                          refusal;
    };

    template<std::uint32_t ClockSpeed>
    static constexpr std::uint32_t calcTop(std::uint16_t div16,
                                           std::uint32_t frequency) {
        return ((ClockSpeed * 16) / (div16)) / frequency - 1;
    }

    // Whether a given TOP and frequency need a divider of at least 1.0 (16 in 8.4).
    template<std::uint32_t ClockSpeed>
    static constexpr bool div16Reachable(std::uint16_t top,
                                         std::uint32_t frequency) {
        return (ClockSpeed * 16) / ((top + 1) * frequency) >= 16;
    }

    template<std::uint32_t ClockSpeed>
    static constexpr std::uint32_t calcDiv16(std::uint16_t top,
                                             std::uint32_t frequency) {
        auto const ret = (ClockSpeed * 16) / ((top + 1) * frequency);
        assert(ret >= 16);
        return ret;
    }

    // DIV (8.4 fixed point, div16) and TOP for a frequency: the largest TOP above topMin
    // whose frequency is exact, else a prime factorisation of clk * 16 / frequency. The
    // period is (TOP + 1) * DIV / clk_sys (RP2040 datasheet 4.5.2.6, RP2350 datasheet
    // 12.5.2.6).
    template<std::uint32_t ClockSpeed>
    static constexpr DivTopSearch searchDivAndTop(std::uint32_t frequency,
                                                  std::uint32_t topMin) {
        constexpr std::uint32_t topMax    = 65536 - 1;
        constexpr std::uint32_t periodMax = topMax + 1;
        constexpr std::uint32_t divMax    = (256 * 16) - 1;
        constexpr std::uint32_t div       = 16;

        std::uint32_t div16Top = div * ClockSpeed / frequency;
        std::uint32_t top      = 1;
        std::uint32_t divTmp   = divMax;

        std::uint32_t divBest{};
        std::uint32_t topBest{};
        while(divTmp > div) {
            std::uint32_t tmpTop
              = calcTop<ClockSpeed>(static_cast<std::uint16_t>(divTmp), frequency);
            if(topMax >= tmpTop && tmpTop > topMin) {
                if(tmpTop > topBest
                   && frequency
                        == calcFrequency<ClockSpeed>(static_cast<std::uint16_t>(divTmp),
                                                     static_cast<std::uint16_t>(tmpTop)))
                {
                    topBest = tmpTop;
                    divBest = divTmp;
                }
            }
            --divTmp;
        }
        if(topBest == 0) {
            // Factorization fallback: factor div16_top into div16 * (TOP + 1)
            // Here we use 'period' to track (TOP + 1), then convert to TOP at the end
            div16Top             = div * ClockSpeed / frequency;
            std::uint32_t period = 1;   // Period is (TOP + 1)
            while(true) {
                // Try a few small prime factors to get close to the desired frequency.
                if(div16Top >= div * 7 && div16Top % 7 == 0 && period * 7 <= periodMax) {
                    div16Top /= 7;
                    period *= 7;
                } else if(div16Top >= div * 5 && div16Top % 5 == 0 && period * 5 <= periodMax) {
                    div16Top /= 5;
                    period *= 5;
                } else if(div16Top >= div * 3 && div16Top % 3 == 0 && period * 3 <= periodMax) {
                    div16Top /= 3;
                    period *= 3;
                } else if(div16Top >= div * 2 && period * 2 <= periodMax) {
                    div16Top /= 2;
                    period *= 2;
                } else {
                    break;
                }
            }
            top = period - 1;   // Convert period to TOP register value
        } else {
            div16Top = divBest;
            top      = topBest;
        }

        if(div16Top < div) { return {std::nullopt, DivTopRefusal::frequencyTooHigh}; }
        if(div16Top >= divMax) { return {std::nullopt, DivTopRefusal::frequencyTooLow}; }
        if(topMin > top) { return {std::nullopt, DivTopRefusal::minTopUnreachable}; }

        return {
          std::make_pair(static_cast<std::uint16_t>(div16Top), static_cast<std::uint16_t>(top)),
          DivTopRefusal::none};
    }

    // The search's value alone: what the compile-time sites (InitialDiv16, InitialTop,
    // the static_asserts) use.
    template<std::uint32_t ClockSpeed>
    static constexpr std::optional<std::pair<std::uint16_t,
                                             std::uint16_t>>
    calcDivAndTop(std::uint32_t frequency,
                  std::uint32_t topMin) {
        return searchDivAndTop<ClockSpeed>(frequency, topMin).value;
    }
}}}   // namespace Kvasir::PWM::detail
