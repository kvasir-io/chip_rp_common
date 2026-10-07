// expect: private constructor of class 'Kvasir::Health::FeedKey'
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/watchdog.hpp>
#include <chrono>

// nobody but the supervisor makes the key a gated feed takes
struct WatchdogConfig {
    static constexpr auto clockSpeed  = HW::CrystalSpeed;
    static constexpr auto overrunTime = std::chrono::milliseconds{500};
    static constexpr bool gatedFeed   = true;
};

using Watchdog = Kvasir::Watchdog<WatchdogConfig>;

[[maybe_unused]] static void keepAlive() { Watchdog::feed(Kvasir::Health::FeedKey{}); }
