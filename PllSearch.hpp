#pragma once
// The PLL settings search of clock_config.hpp. Chip-free (no register header), so the SDK's
// prescaler tests compile this very file against the search it replaced.
#include "kvasir/Util/Prescaler.hpp"

#include <algorithm>
#include <cstdint>
#include <ranges>

namespace Kvasir { namespace DefaultClockSettings { namespace detail {
    struct PllSettings {
        std::uint32_t fbdiv;
        std::uint32_t pd1;
        std::uint32_t pd2;
        std::uint32_t refdiv;

        constexpr bool operator==(PllSettings const&) const = default;
    };

    // FOUTPOSTDIV = (FREF / REFDIV) x FBDIV / (POSTDIV1 x POSTDIV2) with FREF / REFDIV at least
    // 5 MHz, the VCO 750..1600 MHz, FBDIV 16..320, POSTDIV1/2 1..7, REFDIV 1..63 (RP2350 datasheet
    // 8.6.3 and its vcocalc.py; RP2040 datasheet 2.18.2). The search is vcocalc.py's: the output
    // nearest the request, on a tie the higher VCO (less jitter) or with LowVco the lower one (less
    // power), then the first in vcocalc.py's order (REFDIV, FBDIV, POSTDIV2, POSTDIV1: pd1 inner,
    // so a higher pd1:pd2 ratio wins), and only settings whose output is a whole number of mHz.
    template<bool LowVco = false>
    constexpr PllSettings calcPllSettings(std::uint64_t clockSpeed,
                                          std::uint64_t crystalSpeed) {
        constexpr std::uint64_t vcoMax     = 1'600'000'000;
        constexpr std::uint64_t vcoMin     = 750'000'000;
        constexpr std::uint64_t refMin     = 5'000'000;
        constexpr std::uint32_t fbdivMin   = 16;
        constexpr std::uint32_t fbdivMax   = 320;
        constexpr std::uint32_t refdivMax  = 63;
        constexpr std::uint32_t postdivMax = 7;

        std::uint32_t const refdivs = static_cast<std::uint32_t>(
          std::clamp<std::uint64_t>(crystalSpeed / refMin, 1, refdivMax));
        constexpr std::uint32_t pairs = postdivMax * postdivMax;

        // vcocalc.py skips a setting whose VCO in kHz is not a multiple of pd1 x pd2
        auto const wholeMilliHz = [=](std::uint64_t fbdiv, std::uint64_t refdiv, std::uint64_t pd) {
            return Prescaler::mulChecked(crystalSpeed, fbdiv) * 1000U / refdiv % pd == 0;
        };

        // For one REFDIV and pair of post dividers the output rises with FBDIV, so only the
        // nearest allowed FBDIV below the ideal one and the nearest above it can win: two
        // candidates per (REFDIV, POSTDIV2, POSTDIV1) instead of every FBDIV. A setting with the
        // best margin is always one of them, and among equal VCOs the order (REFDIV, POSTDIV2,
        // POSTDIV1) is vcocalc.py's, so the result is the full search's (util_prescaler_tests).
        auto const candidate = [=](std::uint32_t k) {
            std::uint32_t const refdiv = k / (2 * pairs) + 1;
            std::uint32_t const p      = k % (2 * pairs) / 2;
            bool const          upper  = k % 2 == 1;
            PllSettings const   none{.fbdiv = 0, .pd1 = 0, .pd2 = 0, .refdiv = 0};
            if(crystalSpeed < refMin * refdiv) { return none; }   // the reference below 5 MHz
            // vcoMin <= crystal x fbdiv / refdiv <= vcoMax, and FBDIV's own range
            std::uint64_t const lo
              = std::max<std::uint64_t>(fbdivMin,
                                        (vcoMin * refdiv + crystalSpeed - 1) / crystalSpeed);
            std::uint64_t const hi
              = std::min<std::uint64_t>(fbdivMax, vcoMax * refdiv / crystalSpeed);
            std::uint32_t const pd1 = p % postdivMax + 1;
            std::uint32_t const pd2 = p / postdivMax + 1;
            std::uint64_t const pd  = std::uint64_t{pd1} * pd2;
            // the ideal FBDIV: clockSpeed x refdiv x pd / crystal
            std::uint64_t const num = Prescaler::mulChecked(clockSpeed, refdiv * pd);
            std::uint64_t       fb  = num / crystalSpeed;
            if(upper) {
                fb = std::max(lo, fb + (num % crystalSpeed != 0 ? 1U : 0U));
                while(fb <= hi && !wholeMilliHz(fb, refdiv, pd)) { ++fb; }
                if(fb > hi) { return none; }
            } else {
                fb = std::min(hi, fb);
                while(fb >= lo && !wholeMilliHz(fb, refdiv, pd)) { --fb; }
                if(fb < lo || lo > hi) { return none; }
            }
            return PllSettings{.fbdiv  = static_cast<std::uint32_t>(fb),
                               .pd1    = pd1,
                               .pd2    = pd2,
                               .refdiv = refdiv};
        };

        auto candidates = std::views::iota(0U, refdivs * 2 * pairs)
                        | std::views::transform(candidate)
                        | std::views::filter([](PllSettings const& c) { return c.refdiv != 0; });

        // VCO a above VCO b: fbdiv_a / refdiv_a > fbdiv_b / refdiv_b
        auto const prefer = [](PllSettings const& a, PllSettings const& b) {
            auto const va = std::uint64_t{a.fbdiv} * b.refdiv;
            auto const vb = std::uint64_t{b.fbdiv} * a.refdiv;
            return LowVco ? va < vb : va > vb;
        };

        auto const r = Prescaler::search(
          crystalSpeed,
          clockSpeed,
          candidates,
          [](PllSettings const& s) {
              return Prescaler::Rational{s.fbdiv, std::uint64_t{s.refdiv} * s.pd1 * s.pd2};
          },
          Prescaler::Pick::nearest,
          prefer);
        // vcocalc.py starts from a margin of the whole request: nothing closer (on a tie of exactly
        // that, a higher VCO than none, so never with LowVco) leaves the settings all zero
        auto const margin = Prescaler::distanceNum(r.achieved, clockSpeed);
        auto const limit  = Prescaler::mulChecked(clockSpeed, r.achieved.den);
        if(!r.found || margin > limit || (margin == limit && LowVco)) {
            return PllSettings{.fbdiv = 0, .pd1 = 0, .pd2 = 0, .refdiv = 0};
        }
        return r.setting;
    }

    // what the settings give
    constexpr Prescaler::Rational pllOutput(std::uint64_t      crystalSpeed,
                                            PllSettings const& s) {
        return {Prescaler::mulChecked(crystalSpeed, s.fbdiv),
                std::uint64_t{s.refdiv} * s.pd1 * s.pd2};
    }
}}}   // namespace Kvasir::DefaultClockSettings::detail
