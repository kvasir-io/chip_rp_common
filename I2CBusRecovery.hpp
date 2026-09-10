#pragma once

#include "I2C.hpp"
#include "kvasir/Util/RateLimiter.hpp"

#include <string_view>

namespace Kvasir { namespace I2C {

    // Non-blocking I2C bus recovery state machine.
    // Owns: clock-recovery pulses, SDA-stuck detection, post-abort settle gate.
    // Does NOT own the request queue or transaction state — the caller handles
    // draining/failing requests before calling begin().
    template<typename I2CConfig, typename Clock>
    struct I2CBusRecovery {
        using base = Detail::I2CBase<I2CConfig>;
        using tp   = typename Clock::time_point;

        enum class Phase : std::uint8_t {
            Idle,
            Aborting,      // softAbortRequest issued; waiting 100 µs for STOP to propagate
            PinTakeover,   // switch SCL/SDA to GPIO output-high (instantaneous)
            PulseLow,      // SCL low; waiting 20 µs
            PulseHigh,     // SCL high; waiting 20 µs; pulseCount_ decrements back to PulseLow
            StopSdaLow,    // SDA low; waiting 20 µs
            StopSdaHigh,   // SDA high; waiting 20 µs; then -> Reinit
            Reinit,        // restore pin functions; caller must reset the peripheral
        };

        enum class TickResult { busy, idle, needsReinit };

        inline static Phase phase_{Phase::Idle};
        inline static int   pulseCount_{0};
        inline static tp    phaseDeadline_{};
        inline static tp    sickUntil_{};    // post-abort settle gate
        inline static tp    stuckSince_{};   // when a line was first seen low while idle
        inline static Kvasir::RateLimiter<Clock> log_{};   // a held-low SDA recovers in a loop
        inline static std::uint32_t              recoveries_{};

        // A line must be continuously low for this long before recovery fires.
        // Scale with baud rate: ~500 bit-periods gives comfortable margin
        // (a legitimate STOP settles SDA in ~0.5 bit-periods).
        //   100 kHz -> 5 ms,  400 kHz -> 1.25 ms
        // Floor at 1 ms to avoid spurious triggers from noise/polling jitter.
        // explicit std::max<std::uint32_t>: uint32_t differs between gcc and clang, breaking deduction
        static constexpr auto kStuckThreshold = std::chrono::microseconds{
          std::max<std::uint32_t>(1000U, 500'000'000U / I2CConfig::baudRate)};

        /// Backoff for a bus no clocking can free; reset once both lines read high again.
        static constexpr auto kBackoffMin = std::chrono::milliseconds{100};
        static constexpr auto kBackoffMax = std::chrono::milliseconds{2000};

        inline static tp                        retryNotBefore_{};
        inline static std::chrono::milliseconds backoff_{kBackoffMin};

        static bool isActive() { return phase_ != Phase::Idle; }

        static bool sdaIsHigh() {
            return get<0>(apply(read(base::I2CConfig::sdaPinLocation))) != 0;
        }

        /// A slave holding SCL cannot be clocked out of it: for reporting only.
        static bool sclIsHigh() {
            return get<0>(apply(read(base::I2CConfig::sclPinLocation))) != 0;
        }

        /// Recovery sequences begun since reset.
        static std::uint32_t recoveries() { return recoveries_; }

        /// Clocks left over from the nine when SDA came back; 0 means it never did.
        static int clocksLeft() { return pulseCount_; }

        /// SDA held low while the bus is idle: start recovery.
        ///
        /// SDA only. A briefly low SCL between transactions is normal (the block is
        /// disabled after every transaction, a STOP may still be propagating), and a
        /// genuinely held clock cannot be recovered by a master anyway.
        static bool checkBusStuck(tp now) {
            if(sdaIsHigh()) {
                stuckSince_ = tp{};
                backoff_    = kBackoffMin;
                return false;
            }
            if(stuckSince_ == tp{}) { stuckSince_ = now; }
            if(now - stuckSince_ >= kStuckThreshold) {
                KVASIR_LOG_LIMITED(log_.allow(0, now),
                                   UC_LOG_W,
                                   "i2c{} SDA stuck low while idle -- requesting recovery",
                                   base::Instance);
                stuckSince_ = tp{};
                return beginThrottled(now);
            }
            return false;
        }

        // Returns true if the post-abort settle period has elapsed (or was never set).
        static bool isPastSettle(tp now) { return now >= sickUntil_; }

        // Defer the next transaction start by a brief settle period.
        static void deferSettle(tp until) { sickUntil_ = until; }

        /// Begin unless a recent attempt is still backing off; says whether it did.
        /// Every automatic trigger goes through this.
        static bool beginThrottled(tp now) {
            if(now < retryNotBefore_) { return false; }
            retryNotBefore_ = now + backoff_;
            backoff_        = backoff_ * 2 > kBackoffMax ? kBackoffMax : backoff_ * 2;
            begin();
            return true;
        }

        // Begin a full bus recovery sequence, unconditionally: the explicit
        // requestRecovery() asked for it, so it is not subject to the backoff.
        // The caller must have already failed/drained any active transactions.
        static void begin() {
            ++recoveries_;
            apply(base::softAbortRequest);
            phase_         = Phase::Aborting;
            phaseDeadline_ = Clock::now() + std::chrono::microseconds{100};
        }

        // Advance the recovery state machine.  Call once per main-loop tick.
        // Returns:
        //   idle        — recovery is not active, nothing to do
        //   busy        — recovery in progress, caller should return early
        //   needsReinit — recovery finished, caller must reset() the peripheral
        static TickResult tick(tp now) {
            if(phase_ == Phase::Idle) { return TickResult::idle; }
            if(now < phaseDeadline_) { return TickResult::busy; }

            switch(phase_) {
            case Phase::Aborting: phase_ = Phase::PinTakeover; [[fallthrough]];
            case Phase::PinTakeover:
                // SCL is driven, SDA released: the slave must be able to drive SDA while
                // it shifts out, and sdaIsHigh() must read the bus, not our own output.
                apply(makeInput(base::I2CConfig::sdaPinLocation));
                apply(makeOutputInitHigh(base::I2CConfig::sclPinLocation));
                pulseCount_ = 9;
                phase_      = Phase::PulseLow;
                break;
            case Phase::PulseLow:
                apply(clear(base::I2CConfig::sclPinLocation));
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::PulseHigh;
                break;
            case Phase::PulseHigh:
                apply(set(base::I2CConfig::sclPinLocation));
                phaseDeadline_ = now + std::chrono::microseconds{20};
                // Stop as soon as the slave has let SDA go; pulseCount_ keeps the rest.
                phase_ = (--pulseCount_ > 0 && !sdaIsHigh()) ? Phase::PulseLow : Phase::StopSdaLow;
                break;
            case Phase::StopSdaLow:
                // SDA is taken back for the STOP: low while SCL is high, then released.
                apply(makeOutputInitHigh(base::I2CConfig::sdaPinLocation));
                apply(clear(base::I2CConfig::sdaPinLocation));
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::StopSdaHigh;
                break;
            case Phase::StopSdaHigh:
                apply(set(base::I2CConfig::sdaPinLocation));
                phaseDeadline_ = now + std::chrono::microseconds{20};
                phase_         = Phase::Reinit;
                break;
            case Phase::Reinit:
                apply(base::initStepPinConfig);
                phase_ = Phase::Idle;
                return TickResult::needsReinit;
            default: break;
            }
            return TickResult::busy;
        }

        static void resetState() {
            phase_      = Phase::Idle;
            stuckSince_ = tp{};
        }

        /// How long the next attempt is held off for.
        static std::chrono::milliseconds backoff() { return backoff_; }
    };

}}   // namespace Kvasir::I2C
