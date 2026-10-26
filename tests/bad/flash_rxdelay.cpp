// expect: rxdelay out of the QMI's range
// 500 MHz: round(7.5 ns x 2 x clk_sys) = 8, RXDELAY has three bits
#include "FlashTimingMath.hpp"

static_assert(Kvasir::DefaultClockSettings::detail::flashTimingFields<500'000'000ULL,
                                                                      250'000'000ULL>()
                .rxdelay
              > 0);
