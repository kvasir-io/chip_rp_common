// expect: clk_ref must be a whole number of MHz
#include "WatchdogMath.hpp"

static_assert(Kvasir::Detail::WatchdogTiming<12'500'000,
                                             1'000'000,
                                             true>::Ticks
              > 0);
