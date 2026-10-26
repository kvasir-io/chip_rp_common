// expect: more ticks than the 24-bit LOAD holds
// RP2040-E1: 9 s of 1 us ticks is 18 000 000 decrements, past LOAD's 0xffffff
#include "WatchdogMath.hpp"

static_assert(Kvasir::Detail::WatchdogTiming<12'000'000,
                                             9'000'000,
                                             true>::Ticks
              > 0);
