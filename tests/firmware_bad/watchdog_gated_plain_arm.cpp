// expect: no matching function for call to 'arm'
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/watchdog.hpp>
#include <chrono>

// a gated watchdog: arm() without the supervisor's key is not there - it reloads the counter, a feed by another name
struct WatchdogConfig {
    static constexpr auto clockSpeed  = HW::CrystalSpeed;
    static constexpr auto overrunTime = std::chrono::milliseconds{500};
    static constexpr bool gatedFeed   = true;
};

using Watchdog = Kvasir::Watchdog<WatchdogConfig>;

[[maybe_unused]] static void sneak() { Watchdog::arm(); }
