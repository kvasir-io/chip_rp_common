// expect: no matching function for call to 'feed'
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/watchdog.hpp>
#include <chrono>

// a gated watchdog (Config::gatedFeed): only Kvasir::Health::Supervisor feeds it, a plain feed() is not there
struct WatchdogConfig {
    static constexpr auto clockSpeed  = HW::CrystalSpeed;
    static constexpr auto overrunTime = std::chrono::milliseconds{500};
    static constexpr bool gatedFeed   = true;
};

using Watchdog = Kvasir::Watchdog<WatchdogConfig>;

[[maybe_unused]] static void keepAlive() { Watchdog::feed(); }
