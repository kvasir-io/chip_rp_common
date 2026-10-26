// expect: Flash frequency exceeds MaxFlashFreq
// 300 MHz with a 1 MHz flash: the divider stops at 255, SCK 1.18 MHz
#include "FlashTimingMath.hpp"

static_assert(Kvasir::DefaultClockSettings::detail::flashTimingFields<300'000'000ULL,
                                                                      1'000'000ULL>()
                .clkdiv
              > 0);
