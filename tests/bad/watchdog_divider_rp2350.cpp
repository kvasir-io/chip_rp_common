// expect: CyclesPerTick exceeds the 9-bit tick divider
// RP2350: 720 s at 12 MHz needs 515 cycles a tick, the divider has nine bits
#include "WatchdogMath.hpp"

static_assert(Kvasir::Detail::WatchdogTiming<12'000'000,
                                             720'000'000,
                                             false>::Ticks
              > 0);
