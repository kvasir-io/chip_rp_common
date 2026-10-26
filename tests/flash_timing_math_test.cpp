// The QMI flash timing (FlashTimingMath.hpp) against the RP2350 datasheet's Mx_TIMING fields (12.14, md
// l.60486-60489): CLKDIV [7:0] 1..255 encoded directly (this package never goes below 2), RXDELAY [10:8] three bits
// of half clk_sys cycles, MIN_DESELECT [16:12] five bits. The sample delay is round(7.5 ns x 2 x clk_sys), at least 1
// (clock_config.hpp's choice). Building is the test (static_assert); the refusals are bad/flash_*.cpp.
#include "FlashTimingMath.hpp"

#include <cstdint>

namespace {
namespace D = Kvasir::DefaultClockSettings::detail;

constexpr unsigned long long mhz = 1'000'000;

template<unsigned long long Clk,
         unsigned long long Max>
constexpr bool inFieldRanges() {
    constexpr auto f = D::flashTimingFields<Clk, Max>();
    return f.clkdiv >= 1 && f.clkdiv <= 255 && f.rxdelay <= 7 && f.minDeselect <= 31
        && f.cooldown <= 3;
}

// the divider: SCK no faster than the flash's rating, at least 2
static_assert(D::flashClkdiv<150 * mhz,
                             100 * mhz>()
              == 2);
static_assert(D::flashClkdiv<200 * mhz,
                             100 * mhz>()
              == 2);
static_assert(D::flashClkdiv<266 * mhz,
                             100 * mhz>()
              == 3);
static_assert(D::flashClkdiv<25 * mhz,
                             100 * mhz>()
              == 2);
static_assert(D::flashClkdiv<300 * mhz,
                             1 * mhz>()
              == 255);   // clamped to the field
// the boot divider: the final one, at least 4
static_assert(D::bootClkdiv<150 * mhz,
                            100 * mhz>()
              == 4);
static_assert(D::bootClkdiv<500 * mhz,
                            100 * mhz>()
              == 5);

// the sample delay: 2 at 150 MHz, 3 at 200 MHz (clock_config.hpp's windows), 4 at 266 MHz, clamped to 1 below ~33 MHz
static_assert(D::flashTimingFields<150 * mhz,
                                   100 * mhz>()
                .rxdelay
              == 2);
static_assert(D::flashTimingFields<200 * mhz,
                                   100 * mhz>()
                .rxdelay
              == 3);
static_assert(D::flashTimingFields<266 * mhz,
                                   100 * mhz>()
                .rxdelay
              == 4);
static_assert(D::flashTimingFields<25 * mhz,
                                   100 * mhz>()
                .rxdelay
              == 1);

// the rest is fixed: 2 cycles deselect, 1 cooldown; every field in its register range
static_assert(D::flashTimingFields<150 * mhz,
                                   100 * mhz>()
                .minDeselect
              == 2);
static_assert(D::flashTimingFields<150 * mhz,
                                   100 * mhz>()
                .cooldown
              == 1);
static_assert(D::flashTimingFields<150 * mhz,
                                   100 * mhz>()
                .clkdiv
              == 2);
static_assert(inFieldRanges<25 * mhz,
                            100 * mhz>()
              && inFieldRanges<150 * mhz,
                               100 * mhz>()
              && inFieldRanges<266 * mhz,
                               100 * mhz>()
              && inFieldRanges<400 * mhz,
                               133 * mhz>());
}   // namespace

int main() { return 0; }
