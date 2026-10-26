#pragma once
// Clocks::DegradedBoot<ClockSettings>: in every RP image, the chip's own Startup entry (Startup::ChipEntries, below).
// A clock init that keeps failing at boot ends in a boot on a degraded tree instead of an endless reboot loop, and in
// the full tree once the crystal and the PLLs come back - live, from the main loop.
#include "Clocks.hpp"
#include "Timer.hpp"
#include "clock_config.hpp"
#include "core/Special.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Hal/Clocks.hpp"
#include "kvasir/Mpl/brigand.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/StartUp/Entry.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/Backoff.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/attributes.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string_view>
#include <uc_log/uc_log.hpp>

namespace Kvasir { namespace Clocks {

    /// The pauses between two tries of a failed repair (Resus) or of a degraded boot's switch-up (DegradedBoot): the
    /// first after 100 ms, at most 10 s.
    struct RetryPauses {
        static constexpr std::chrono::milliseconds first{100};
        static constexpr std::chrono::seconds      longest{10};
    };

    // -------------------------------------------------------------------------------------
    // The degraded boot
    // -------------------------------------------------------------------------------------
    //
    // Every RP image runs it: the chip adds it to every primary Startup list, before the listed entries
    // (Startup::ChipEntries, at the end of this header; listed by hand it is a compile error, ListRules
    // ChipEntryNotListed). Its options are a hook:
    //
    //     struct DegradedConfig { static constexpr std::uint32_t failedBoots = 3; };   // the default
    //     template<> struct Kvasir::Clocks::DegradedBootConfig<Kvasir::Tag::User> { using type = DegradedConfig; };
    //
    //   1. postMemoryInit (after initMemory, before peripheryClockInit) counts the boots in a row whose clock init did
    //      not finish (ClockSettings::initFinished()), in a .noInit record (a validity magic against power-on RAM; the
    //      SRAM survives the panic's reboot); a boot whose init finished clears it. Up to Config::failedBoots of them
    //      panic and reboot (Cause::clockInit, ClockSettings::peripheryClockInit): a crystal that was only slow gets
    //      fresh starts, and each panic is recorded.
    //   2. The next one runs degraded: ClockSettings::degradedClockInit() - only as far down as the failure needs
    //      (DefaultClockSettings::degradedClockInit: the ring oscillator without a crystal; the crystal when only
    //      PLL_SYS failed; PLL_SYS at full speed when only PLL_USB did), what it costs listed there - and
    //      peripheryClockInit() keeps the flash on the boot's divider (it sets the read mode up already). The boot
    //      report says why (Port::StartNotes, "boot: clocks degraded: the clock init failed in N boots in a row
    //      (crystal oscillator not stable), this one runs on the ring oscillator - ...", right after the reset cause
    //      and before the record of the last panic).
    //   3. turn(), a Hook::MainLoop entry, tries the crystal and the PLLs on a doubling pause (Kvasir::Backoff on the
    //      TIMER: 100 ms up to 10 s; on the ROSC its ticks are slow, never fast - degradedClockInit): the crystal is
    //      started without a wait (crystalStarted; one not stable at a try is a failed try, and a missing crystal
    //      never holds the loop), then startSources() with interrupts on (a PLL that does not lock holds the loop for
    //      its bound, Chip::PllLockBound), switchToSources() with interrupts masked, and flashAfterSwitchUp() for the
    //      full clock's flash divider (CLKDIV alone: safe while another core fetches from flash). A live switch, no
    //      reboot: every driver's dividers were computed for the full clock, so they are right from the switch on, and
    //      the application's state, the log and a BootGuard's count stay.
    //   4. USB: a host that met the device while clk_usb ran from the ROSC or the crystal gave up enumerating it. After
    //      the switch-up the device goes off the bus and comes back after USB::Rp::Reattach::pause (its D+ pull-up,
    //      usb/Reattach.hpp, through the hook AfterSwitchUp below; nothing for an image without the USB backend or
    //      without a device), and the host enumerates it again.
    //
    // A boot after the switch-up whose init fails again runs degraded at once: only a boot whose own init finished
    // clears the count. Needs Startup::run<Hook::MainLoop>() in the firmware's loop, or turn() called by hand next to
    // the other hand-called turns (Kvasir::Clocks::DegradedBoot<HW::ClockSettings>::turn()); without either a degraded
    // boot stays degraded until the next reset, whose clock init tries the crystal again. A Clocks::Resus stays
    // independent: it watches clk_sys, which keeps running on the degraded tree.
    struct DegradedBootTag {};

    struct DefaultDegradedBootConfig {};

    /// What runs after a degraded boot's switch-up (step 4 above), a hook: `using type = X;`, X with
    /// `static bool begin()` (true: `X::end()` is due after `X::pause`) and `static void end()`. By default nothing; the
    /// RP USB driver names its re-attach (usb/Reattach.hpp, included with the USB backend), so an image with the backend
    /// takes its device off the bus and back, and one without carries none of it.
    struct NothingAfterSwitchUp {
        static constexpr std::chrono::milliseconds pause{0};

        static bool begin() { return false; }

        static void end() {}
    };

    template<typename T, typename... Ts>
    struct AfterSwitchUp {
        using type = NothingAfterSwitchUp;
    };

    /// The hook for DegradedBoot's options (the comment above): `using type = <a config struct>;`.
    template<typename T, typename... Ts>
    struct DegradedBootConfig {
        using type = DefaultDegradedBootConfig;
    };

    template<typename ClockSettings>
    struct DegradedBoot {
        static_assert(
          DefaultClockSettings::DegradableClockSettings<ClockSettings>,
          "Kvasir::Clocks::DegradedBoot (in every RP image): the ClockSettings does not say how "
          "its tree "
          "degrades - it needs Provides, initFinished(), degradedClockInit(), crystalStarted(), "
          "startSources(), switchToSources() and flashAfterSwitchUp() next to coreClockInit(); "
          "Kvasir::DefaultClockSettings::Settings<ClockSpeed, CrystalSpeed> has them for "
          "DefaultClockSettings' tree (use it, derive from it, or name your tree's own)");

        static constexpr unsigned long long clkSysHz
          = Hal::hzOf<ClkSys, typename ClockSettings::Provides>();
        static constexpr unsigned long long clkRefHz
          = Hal::hzOf<ClkRef, typename ClockSettings::Provides>();

        using Config_ =
          typename Port::LateT<DegradedBootConfig<Kvasir::Tag::User>, ClockSettings>::type;
        // after the switch-up: the RP USB driver's re-attach where the image has its backend (AfterSwitchUp above)
        using After = typename Port::LateT<AfterSwitchUp<Kvasir::Tag::User>, ClockSettings>::type;

        struct Config {
            // boots in a row whose clock init failed (each a panic and a reboot) before the next runs degraded
            static constexpr std::uint32_t failedBoots
              = static_cast<std::uint32_t>(KVASIR_CONFIG_OR(Config_, failedBoots, 3U));
        };

        static_assert(
          Config::failedBoots >= 1 && Config::failedBoots <= 100,
          "DegradedBoot: failedBoots is 1 .. 100 boots that panic before one runs degraded");

        // the chip's own entry (ListRules ChipEntryNotListed)
        static constexpr bool chipEntry = true;

        using Provides = brigand::list<Startup::Resource<DegradedBootTag, 0>>;

        struct Report {
            bool degraded{};       // this boot runs on a degraded tree, the switch-up not done yet
            bool switchedUp{};     // it ran degraded and has switched up since
            DegradedTree tree{};   // the tree it ran on (none: the boot was not degraded)
            InitResult
              cause{};   // what the boot's own clock init stopped at (ok: the boot was not degraded)
            std::uint32_t
              failedBoots{};   // boots in a row whose init failed before this one (each a panic)
            std::uint32_t tries{};     // switch-ups turn() tried
            InitResult    lastTry{};   // what the last try stopped at (ok: it switched up)
            bool
              reattaching{};   // after the switch-up, AfterSwitchUp's end() is still due (USB: off the bus)
        };

        [[nodiscard]] static Report report() {
            auto const s = state_.load(std::memory_order_relaxed);
            return Report{s == State::degraded,
                          switchedUp_.load(std::memory_order_relaxed),
                          detail::degradedOn,
                          detail::degradedBy,
                          detail::failedBootsBefore,
                          tries_.load(std::memory_order_relaxed),
                          lastTry_.load(std::memory_order_relaxed),
                          s == State::reattaching};
        }

        /// Startup's hook after initMemory(), before peripheryClockInit(): steps 1 and 2 above. Registers and this
        /// struct's own state; interrupts are still masked.
        static void postMemoryInit() {
            bool const          valid    = record_.magic == magic;
            std::uint32_t const failures = valid ? record_.failures : 0U;
            if(ClockSettings::initFinished()) {
                if(!valid || failures != 0) { store_(0); }
                return;
            }
            if(failures < Config::failedBoots) {
                store_(failures + 1);   // peripheryClockInit() panics and reboots
                return;
            }
            auto const cause = Clocks::initResult();
            // a switch that does not show: peripheryClockInit() panics, as for the boots before
            auto const tree = ClockSettings::degradedClockInit();
            if(tree == DegradedTree::none) { return; }
            detail::degradedBy        = cause;
            detail::degradedOn        = tree;
            detail::failedBootsBefore = failures;
            state_.store(State::degraded, std::memory_order_relaxed);
        }

        /// The Hook::MainLoop entry: one load while the boot is not degraded (steps 3 and 4 above). What it returns
        /// tells a sleeping loop (Runtime/Executor.hpp, Startup::runTurn) when it is needed next: idle when not
        /// degraded, else at the next try or the end of the re-attach pause.
        static Kvasir::Turn turn() {
            auto const s = state_.load(std::memory_order_relaxed);
            if(s == State::idle) { return Kvasir::Turn::idle(); }
            auto const now = TimeClock::now();
            if(s == State::reattaching) {
                if(now < reattachAt_) { return Kvasir::Turn::within(reattachAt_ - now); }
                After::end();
                state_.store(State::idle, std::memory_order_relaxed);
                UC_LOG_I("clocks: the USB device is back on the bus: the host enumerates it again");
                return Kvasir::Turn::idle();
            }
            // the first turn of a degraded boot starts the crystal and the first pause
            if(backoff_.failures == 0) {
                static_cast<void>(ClockSettings::crystalStarted());
                backoff_.failed(now);
                return Kvasir::Turn::within(backoff_.next - now);
            }
            if(!backoff_.due(now)) { return Kvasir::Turn::within(backoff_.next - now); }
            tries_.fetch_add(1, std::memory_order_relaxed);
            // a crystal that is not stable yet is started (again) and looked at by the next try: no wait for it here
            auto r = ClockSettings::crystalStarted() ? ClockSettings::startSources()
                                                     : InitResult::xoscNotStable;
            if(r == InitResult::ok) {
                auto const irqs = Kvasir::Core::IrqState::saveAndMask();
                r               = ClockSettings::switchToSources();
                irqs.restore();
            }
            lastTry_.store(r, std::memory_order_relaxed);
            if(r != InitResult::ok) {
                auto const after = TimeClock::now();
                backoff_.failed(after);
                return Kvasir::Turn::within(backoff_.next - after);
            }
            ClockSettings::flashAfterSwitchUp();
            backoff_.succeeded();
            switchedUp_.store(true, std::memory_order_relaxed);
            // clk_usb ran from the ROSC or the crystal until now (degradedClockInit's marker)
            bool const reattach = After::begin();
            if(reattach) {
                reattachAt_ = TimeClock::now() + After::pause;
                state_.store(State::reattaching, std::memory_order_relaxed);
            } else {
                state_.store(State::idle, std::memory_order_relaxed);
            }
            UC_LOG_I("clocks: switched up from {} after {} tries: crystal and PLLs running, {}{}",
                     Clocks::name(detail::degradedOn),
                     tries_.load(std::memory_order_relaxed),
                     Clocks::name(Clocks::initResult()),
                     reattach ? std::string_view{"; the USB device off the bus for a re-attach"}
                              : std::string_view{});
            return reattach ? Kvasir::Turn::within(After::pause) : Kvasir::Turn::idle();
        }

        using Extends = Startup::Extend<Kvasir::Hook::MainLoop, &DegradedBoot::turn>;

    private:
        // The TIMER the boot started (coreClockInit, degradedClockInit: in every image, ticking from clk_ref), read as
        // a clock without being a Startup entry
        struct TimerConfig {
            static constexpr unsigned instance    = 0;
            static constexpr auto     tickClockHz = clkRefHz;
        };

        using TimeClock = Kvasir::Timer::TimerClockBase<TimerConfig>;

        enum class State : std::uint8_t { idle, degraded, reattaching };

        // "DGRD" in memory: the record is no power-on garbage
        static constexpr std::uint32_t magic = 0x4452'4744U;

        struct Record {
            std::uint32_t magic;
            std::uint32_t failures;
        };

        static void store_(std::uint32_t failures) {
            record_.magic    = magic;
            record_.failures = failures;
        }

        // volatile: uninitialised, and read back after the reboot
        [[KVASIR_NOINIT_MEMBER]] inline static Record volatile record_;
        inline static std::atomic<State>                      state_{State::idle};
        inline static std::atomic<bool>                       switchedUp_{false};
        inline static std::atomic<std::uint32_t>              tries_{0};
        inline static std::atomic<InitResult>                 lastTry_{InitResult::ok};
        inline static Kvasir::Backoff<TimeClock, RetryPauses> backoff_{};
        inline static typename TimeClock::time_point          reattachAt_{};
    };

}}   // namespace Kvasir::Clocks

/// Every primary Startup list of an RP image runs the degraded boot for its ClockSettings (Startup::ChipEntries,
/// kvasir/StartUp/Entry.hpp).
namespace Kvasir { namespace Clocks {
    struct ChipEntrySet {
        template<typename ClockSettings>
        using For = brigand::list<DegradedBoot<ClockSettings>>;
    };
}}   // namespace Kvasir::Clocks

template<typename... Ts>
struct Kvasir::Startup::ChipEntries<Kvasir::Tag::User, Ts...> {
    using type = Kvasir::Clocks::ChipEntrySet;
};
