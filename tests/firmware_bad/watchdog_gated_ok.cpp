// expect-ok
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/watchdog.hpp>
#include <chrono>
#include <kvasir/Util/Health.hpp>
#include <string_view>

// gatedFeed off (the default): feed(), arm() and disarm() as ever. On: they are gone, the keyed ones are there,
// and a supervisor over that watchdog compiles - it is the one that holds the key.
template<bool Gated>
struct WatchdogConfig {
    static constexpr auto clockSpeed  = HW::CrystalSpeed;
    static constexpr auto overrunTime = std::chrono::milliseconds{500};
    static constexpr bool gatedFeed   = Gated;
};

struct PlainConfig {
    static constexpr auto clockSpeed  = HW::CrystalSpeed;
    static constexpr auto overrunTime = std::chrono::milliseconds{500};
};

template<typename W>
constexpr bool plainFeed = requires { W::feed(); };
template<typename W>
constexpr bool keyedFeed = requires(Kvasir::Health::FeedKey key) { W::feed(key); };
template<typename W>
constexpr bool plainLife = requires {
    W::arm();
    W::disarm();
};
template<typename W>
constexpr bool keyedLife = requires(Kvasir::Health::FeedKey key) {
    W::arm(key);
    W::disarm(key);
};

static_assert(plainFeed<Kvasir::Watchdog<PlainConfig>>
              && !keyedFeed<Kvasir::Watchdog<PlainConfig>>);
static_assert(plainFeed<Kvasir::Watchdog<WatchdogConfig<false>>>);
static_assert(!plainFeed<Kvasir::Watchdog<WatchdogConfig<true>>>
              && keyedFeed<Kvasir::Watchdog<WatchdogConfig<true>>>);

struct LoopHealth {
    static constexpr std::string_view name        = "loop";
    static constexpr auto             maxInterval = std::chrono::milliseconds{100};
};

struct Loop {
    using Extends = brigand::list<Kvasir::Health::Check<LoopHealth>>;
};

using Startup = Kvasir::Startup::Startup<HW::ClockSettings, HW::SystickClock, Loop>;

using Health
  = Kvasir::Health::Supervisor<Startup, Kvasir::Watchdog<WatchdogConfig<true>>, HW::SystickClock>;

static_assert(plainLife<Kvasir::Watchdog<PlainConfig>>
              && !keyedLife<Kvasir::Watchdog<PlainConfig>>);
static_assert(!plainLife<Kvasir::Watchdog<WatchdogConfig<true>>>
              && keyedLife<Kvasir::Watchdog<WatchdogConfig<true>>>);

[[maybe_unused]] static void turn() {
    Health::holdOff();   // disarm through the key
    Kvasir::Health::checkIn<LoopHealth>();
    Health::service();   // arms through the key at the first turn, feeds after
}
