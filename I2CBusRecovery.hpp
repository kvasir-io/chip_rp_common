#pragma once

#include "I2C.hpp"

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
        inline static tp    sickUntil_{};       // post-abort settle gate
        inline static tp    sdaStuckSince_{};   // when SDA first observed stuck low

        // SDA must be continuously low for this long before recovery fires.
        // Scale with baud rate: ~500 bit-periods gives comfortable margin
        // (a legitimate STOP settles SDA in ~0.5 bit-periods).
        //   100 kHz -> 5 ms,  400 kHz -> 1.25 ms
        // Floor at 1 ms to avoid spurious triggers from noise/polling jitter.
        static constexpr auto kSdaStuckThreshold = std::chrono::microseconds{
          std::max(std::uint32_t{1000}, 500'000'000u / I2CConfig::baudRate)};

        static bool isActive() { return phase_ != Phase::Idle; }

        static bool sdaIsHigh() {
            return get<0>(apply(read(base::I2CConfig::sdaPinLocation))) != 0;
        }

        // Returns true if SDA is stuck and recovery was initiated.
        static bool checkSdaStuck(tp now) {
            if(sdaIsHigh()) {
                sdaStuckSince_ = tp{};
                return false;
            }
            if(sdaStuckSince_ == tp{}) { sdaStuckSince_ = now; }
            if(now - sdaStuckSince_ >= kSdaStuckThreshold) {
                UC_LOG_W("i2c{} SDA stuck low -- requesting recovery", base::Instance);
                sdaStuckSince_ = tp{};
                begin();
                return true;
            }
            return false;
        }

        // Returns true if the post-abort settle period has elapsed (or was never set).
        static bool isPastSettle(tp now) { return now >= sickUntil_; }

        // Defer the next transaction start by a brief settle period.
        static void deferSettle(tp until) { sickUntil_ = until; }

        // Begin a full bus recovery sequence.
        // The caller must have already failed/drained any active transactions.
        static void begin() {
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
                apply(makeOutputInitHigh(base::I2CConfig::sdaPinLocation));
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
                phase_         = (--pulseCount_ > 0) ? Phase::PulseLow : Phase::StopSdaLow;
                break;
            case Phase::StopSdaLow:
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
                apply(
                  action(Kvasir::Io::Action::PinFunction<3>{}, base::I2CConfig::sclPinLocation));
                apply(
                  action(Kvasir::Io::Action::PinFunction<3>{}, base::I2CConfig::sdaPinLocation));
                phase_ = Phase::Idle;
                return TickResult::needsReinit;
            default: break;
            }
            return TickResult::busy;
        }

        static void resetState() {
            phase_         = Phase::Idle;
            sdaStuckSince_ = tp{};
        }
    };

}}   // namespace Kvasir::I2C
