// The RP PLL search (PllSearch.hpp, Kvasir::DefaultClockSettings::detail::calcPllSettings) against an exhaustive
// search in exact integers over every REFDIV, FBDIV, POSTDIV2 and POSTDIV1 the datasheets allow, with vcocalc.py's rules
// (nearest output, then the preferred VCO, then the first in its order; only whole-mHz settings): over a grid of crystals
// and targets the two pick the same settings. It reads the SDK's test support (kvasir::test_support: the harness).
#include "PllSearch.hpp"
#include "test_harness.hpp"

#include <array>
#include <cstdint>
#include <cstdio>

namespace {
namespace D = Kvasir::DefaultClockSettings::detail;

// vcocalc.py's search, every setting tried: FREF / REFDIV at least 5 MHz, VCO 750..1600 MHz, FBDIV 16..320,
// POSTDIV1/2 1..7, REFDIV 1..63 (RP2350 datasheet 8.6.3, RP2040 datasheet 2.18.2), in its order (REFDIV, FBDIV,
// POSTDIV2, POSTDIV1), a setting taken when its margin is smaller, or equal with the preferred VCO; the start is a
// margin of the whole request with no VCO. Margins and VCOs are compared cross-multiplied, exactly.
template<bool LowVco>
D::PllSettings exhaustive(std::uint64_t target,
                          std::uint64_t crystal) {
    D::PllSettings      best{.fbdiv = 0, .pd1 = 0, .pd2 = 0, .refdiv = 0};
    std::uint64_t       bestMargin = target;   // over bestDen
    std::uint64_t       bestDen    = 1;
    std::uint64_t       bestFb     = 0;   // the VCO is crystal x bestFb / bestRef
    std::uint64_t       bestRef    = 1;
    std::uint64_t const refdivs    = std::clamp<std::uint64_t>(crystal / 5'000'000, 1, 63);
    for(std::uint64_t refdiv = 1; refdiv <= refdivs; ++refdiv) {
        if(crystal < 5'000'000 * refdiv) { continue; }
        for(std::uint64_t fbdiv = 16; fbdiv <= 320; ++fbdiv) {
            std::uint64_t const vcoNum = crystal * fbdiv;   // over refdiv
            if(vcoNum < 750'000'000 * refdiv || vcoNum > 1'600'000'000 * refdiv) { continue; }
            for(std::uint64_t pd2 = 1; pd2 <= 7; ++pd2) {
                for(std::uint64_t pd1 = 1; pd1 <= 7; ++pd1) {
                    std::uint64_t const pd = pd1 * pd2;
                    if(vcoNum * 1000 / refdiv % pd != 0) { continue; }   // the VCO in mHz, floored
                    std::uint64_t const den       = refdiv * pd;
                    std::uint64_t const want      = target * den;
                    std::uint64_t const margin    = vcoNum > want ? vcoNum - want : want - vcoNum;
                    bool const          vcoBetter = LowVco ? fbdiv * bestRef < bestFb * refdiv
                                                           : fbdiv * bestRef > bestFb * refdiv;
                    if(margin * bestDen < bestMargin * den
                       || (margin * bestDen == bestMargin * den && vcoBetter))
                    {
                        best       = {.fbdiv  = static_cast<std::uint32_t>(fbdiv),
                                      .pd1    = static_cast<std::uint32_t>(pd1),
                                      .pd2    = static_cast<std::uint32_t>(pd2),
                                      .refdiv = static_cast<std::uint32_t>(refdiv)};
                        bestMargin = margin;
                        bestDen    = den;
                        bestFb     = fbdiv;
                        bestRef    = refdiv;
                    }
                }
            }
        }
    }
    return best;
}

// spot values, computed once from the search: 12 MHz crystal, 150 MHz (VCO 1500 MHz / 5 / 2) and 125 MHz (/ 6 / 2)
static_assert(D::calcPllSettings<false>(150'000'000,
                                        12'000'000)
              == D::PllSettings{.fbdiv  = 125,
                                .pd1    = 5,
                                .pd2    = 2,
                                .refdiv = 1});
static_assert(D::calcPllSettings<false>(125'000'000,
                                        12'000'000)
              == D::PllSettings{.fbdiv  = 125,
                                .pd1    = 6,
                                .pd2    = 2,
                                .refdiv = 1});

// the SDK prescaler test's clock grid (its UART search), here extra PLL targets
constexpr std::array<std::uint32_t, 12> clocks{12'000'000,
                                               16'000'000,
                                               24'000'000,
                                               48'000'000,
                                               100'000'000,
                                               120'000'000,
                                               125'000'000,
                                               133'000'000,
                                               150'000'000,
                                               200'000'000,
                                               250'000'000,
                                               7'369'121};

void rpPllGrid() {
    Kvasir::Test::test("RP PLL: the search == the exhaustive exact one");
    {
        constexpr std::array<std::uint32_t, 10> crystals{12'000'000,
                                                         8'000'000,
                                                         10'000'000,
                                                         16'000'000,
                                                         20'000'000,
                                                         24'000'000,
                                                         25'000'000,
                                                         30'000'000,
                                                         48'000'000,
                                                         7'372'800};
        std::array<std::uint32_t, 4> const odd{133'333'333, 99'999'999, 125'000'001, 48'000'000};
        int                                same   = 0;
        int                                other  = 0;
        auto const                         report = [&](std::uint32_t  crystal,
                                                        std::uint32_t  target,
                                                        bool           low,
                                                        D::PllSettings a,
                                                        D::PllSettings b) {
            if(a == b) {
                ++same;
                return;
            }
            ++other;
            std::printf(
              "  pll differ: crystal %u target %u%s exhaustive {ref %u fb %u pd1 %u pd2 %u} "
              "search {ref %u fb %u pd1 %u pd2 %u}\n",
              crystal,
              target,
              low ? " lowvco" : "",
              a.refdiv,
              a.fbdiv,
              a.pd1,
              a.pd2,
              b.refdiv,
              b.fbdiv,
              b.pd1,
              b.pd2);
        };
        auto const check = [&](std::uint32_t crystal, std::uint32_t target) {
            report(crystal,
                   target,
                   false,
                   exhaustive<false>(target, crystal),
                   D::calcPllSettings<false>(target, crystal));
            report(crystal,
                   target,
                   true,
                   exhaustive<true>(target, crystal),
                   D::calcPllSettings<true>(target, crystal));
        };
        for(auto const crystal : crystals) {
            // every MHz with the 12 MHz crystal every RP board here has, every 5 MHz otherwise
            std::uint32_t const step = crystal == 12'000'000 ? 1'000'000 : 5'000'000;
            for(std::uint32_t target = 10'000'000; target <= 300'000'000; target += step) {
                check(crystal, target);
            }
            for(auto const t : odd) { check(crystal, t); }
            for(auto const t : clocks) { check(crystal, t); }
        }
        std::uint64_t state = 0xD1B54A32D192ED03ULL;
        for(int i = 0; i < 400; ++i) {
            state = state * 6364136223846793005ULL + 1442695040888963407ULL;
            auto const target
              = static_cast<std::uint32_t>(16'000'000 + (state >> 33) % 284'000'000);
            check(crystals[static_cast<std::size_t>(i) % crystals.size()], target);
        }
        std::printf("  RP PLL: %d same, %d other\n", same, other);
        CHECK(other == 0);
    }
}
}   // namespace

int main() {
    rpPllGrid();
    return Kvasir::Test::report();
}
