#pragma once

#include "I2CBusRecovery.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Devices/BusTypes.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/StartUp/Hooks.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir { namespace I2C {

    enum class I2CRequestResult : std::uint8_t { failed, notAcknowledged, succeeded };

    /// The result on a bus with `cancellable` or `requestDeadlines`: the first three as I2CRequestResult.
    enum class I2CRequestResultTracked : std::uint8_t {
        failed,
        notAcknowledged,
        succeeded,
        cancelled,
        timedOut
    };

    template<std::size_t CallbackSize>
    struct I2CRequest {
        static constexpr std::size_t MaxPrefix = 2;

        std::uint8_t address{};
        /// Sent ahead of sendData in the same write, with no START or STOP between them: a
        /// register address, an EEPROM memory address, a display's control byte. Up to two
        /// bytes, kept in the padding behind `address`, so the request stays the size it was
        /// (a gather list would need storage that outlives the queued request, for the one or
        /// two bytes every user has).
        std::uint8_t                                         prefixBytes{};
        std::array<std::byte, MaxPrefix>                     prefix{};
        std::span<std::byte const>                           sendData{};
        std::span<std::byte>                                 receiveData{};
        StaticFunction<void(I2CRequestResult), CallbackSize> callback{};

        template<std::integral... B>
            requires(sizeof...(B) <= MaxPrefix)
        constexpr void setPrefix(B... b) {
            prefix      = {std::byte{static_cast<std::uint8_t>(b)}...};
            prefixBytes = sizeof...(B);
        }

        /// Everything that goes out after the address, prefix first.
        [[nodiscard]] constexpr std::size_t sendBytes() const {
            return prefixBytes + sendData.size();
        }

        [[nodiscard]] constexpr std::byte sendByte(std::size_t i) const {
            return i < prefixBytes ? prefix[i] : sendData[i - prefixBytes];
        }
    };

    static_assert(sizeof(void*) != 4 || sizeof(I2CRequest<16>) == 40,
                  "the prefix must stay in the padding behind address");

    /// The request of a bus with I2CConfig::perDeviceClock: the device's SCL counts ride along.
    /// A type of its own, not a parameter of I2CRequest, so that a bus without the feature
    /// keeps the very same type (and names: a sanitize image hashes them).
    ///
    /// `BusDefault` is the timing of the bus's own rate: a request nobody gave a timing - the bus
    /// scan's probes, a raw request of the application's - goes out at baudRate, never at the
    /// all-zero counts a value-initialised member would be.
    template<std::size_t CallbackSize, typename Timing, Timing BusDefault>
    struct I2CTimedRequest : I2CRequest<CallbackSize> {
        Timing timing{BusDefault};
    };

    /// The request of a bus with `cancellable` / `requestDeadlines` (kvasir_devices BusTypes.hpp): I2CRequest's
    /// fields with the tracked result, a ticket, the tombstone flag and an optional deadline. A type of its own, so a
    /// bus without the features keeps I2CRequest and its names.
    template<std::size_t CallbackSize, typename TimePoint, bool Deadlines>
    struct I2CTrackedRequest {
        static constexpr std::size_t MaxPrefix = 2;

        std::uint8_t                                                address{};
        std::uint8_t                                                prefixBytes{};
        std::array<std::byte, MaxPrefix>                            prefix{};
        std::span<std::byte const>                                  sendData{};
        std::span<std::byte>                                        receiveData{};
        StaticFunction<void(I2CRequestResultTracked), CallbackSize> callback{};
        std::uint16_t                                               ticket{};
        bool                                                        cancelled{};
        [[no_unique_address]] Kvasir::I2C::detail::IfFeature<Deadlines, TimePoint, 30> deadline{
          TimePoint::max()};

        template<std::integral... B>
            requires(sizeof...(B) <= MaxPrefix)
        constexpr void setPrefix(B... b) {
            prefix      = {std::byte{static_cast<std::uint8_t>(b)}...};
            prefixBytes = sizeof...(B);
        }

        [[nodiscard]] constexpr std::size_t sendBytes() const {
            return prefixBytes + sendData.size();
        }

        [[nodiscard]] constexpr std::byte sendByte(std::size_t i) const {
            return i < prefixBytes ? prefix[i] : sendData[i - prefixBytes];
        }
    };

    template<std::size_t CallbackSize,
             typename TimePoint,
             bool Deadlines,
             typename Timing,
             Timing BusDefault>
    struct I2CTimedTrackedRequest : I2CTrackedRequest<CallbackSize, TimePoint, Deadlines> {
        Timing timing{BusDefault};
    };

    namespace Detail {
        /// Not constexpr: reaching one in timing() is the compile error that says why.
        inline void i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus() {}

        inline void i2cDeviceClockBelowMinBaudRateOfTheBus() {}
    }   // namespace Detail

    template<typename I2CConfig, typename Clock, std::size_t QueueDepth_, std::size_t CallbackSize_>
    struct I2CBehaviorQueued : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t QueueDepth   = QueueDepth_;
        static constexpr std::size_t CallbackSize = CallbackSize_;
        /// Re-exported from the config so a bus can size its traffic against the bandwidth.
        static constexpr auto BaudRate = I2CConfig::baudRate;
        using base                     = Detail::I2CBase<I2CConfig>;
        using Regs                     = typename base::Regs;
        using tp                       = typename Clock::time_point;
        using Recovery                 = I2CBusRecovery<I2CConfig, Clock>;

        /// Tickets / cancel() and per-request deadlines (I2CConfig::cancellable, ::requestDeadlines).
        static constexpr bool Cancellable = base::I2CConfig::cancellable;
        static constexpr bool Deadlines   = base::I2CConfig::requestDeadlines;
        static constexpr bool Tracked     = Cancellable || Deadlines;
        using Result = std::conditional_t<Tracked, I2CRequestResultTracked, I2CRequestResult>;

        /// Each device at its own clock (I2CConfig::perDeviceClock). Off, a request has no
        /// timing member and startNext() never looks at the SCL counts: nothing of this
        /// is in the image.
        static constexpr bool PerDeviceClock = base::I2CConfig::perDeviceClock;
        /// Count the requests accepted for the wire (I2CConfig::countTransfers, transfers()).
        static constexpr bool CountTransfers = base::I2CConfig::countTransfers;
        using ClockTiming                    = typename base::Config::ClockTiming;
        /// The timing of baudRate itself, what a request without one of its own carries.
        static constexpr ClockTiming DefaultTiming = [] {
            if constexpr(PerDeviceClock) {
                return base::Config::clockTiming(I2CConfig::clockSpeed,
                                                 static_cast<std::uint32_t>(BaudRate),
                                                 base::I2CConfig::maxBaudRateError);
            } else {
                return ClockTiming{};
            }
        }();
        using Request = std::conditional_t<
          Tracked,
          std::conditional_t<
            PerDeviceClock,
            I2CTimedTrackedRequest<CallbackSize, tp, Deadlines, ClockTiming, DefaultTiming>,
            I2CTrackedRequest<CallbackSize, tp, Deadlines>>,
          std::conditional_t<PerDeviceClock,
                             I2CTimedRequest<CallbackSize, ClockTiming, DefaultTiming>,
                             I2CRequest<CallbackSize>>>;

        /// What a request carries for a device clocked at most at `hz` (kvasir_devices'
        /// Device fills it in from Config::BusClock / Chip::I2cMaxClock): the counts for
        /// min(hz, baudRate). On a bus without perDeviceClock a device slower than the bus
        /// is a compile error, and the answer is nothing.
        static consteval auto timing(std::uint32_t hz) {
            if constexpr(PerDeviceClock) {
                auto const f = std::min(hz, static_cast<std::uint32_t>(BaudRate));
                if(f < base::I2CConfig::minBaudRate) {
                    Detail::i2cDeviceClockBelowMinBaudRateOfTheBus();
                }
                return base::Config::clockTiming(I2CConfig::clockSpeed,
                                                 f,
                                                 base::I2CConfig::maxBaudRateError);
            } else {
                if(hz < BaudRate) {
                    Detail::i2cDeviceClockBelowTheBusClockSetPerDeviceClockOnTheBus();
                }
                return std::false_type{};
            }
        }

        /// Bus faults in an unbroken row that mean the bus is dead. A success resets it, and
        /// so does a NAK: the address went out and nobody took it, which is a working wire
        /// with nothing at that address -- an empty header must never look like a dead bus.
        static constexpr std::uint32_t kDeadBusFailures = 40;

        enum class State { idle, sending, receiving };

        inline static Kvasir::Atomic::Queue<Request, QueueDepth> requestQueue_{};
        inline static Request                                    currentRequest_{};
        inline static bool                                       active_{false};
        inline static State                                      state_{State::idle};
        // std::size_t: a request's spans are not capped at 255 bytes.
        inline static std::size_t sendIndex_{0};
        inline static std::size_t receivedCount_{0};
        inline static bool        stop{false};
        inline static tp          timeoutTime_{};
        // Dead-bus watchdog state.
        inline static std::uint32_t consecutiveFailures_{};
        inline static std::uint32_t resuscitations_{};
        // Fault logging goes through this: a bad bus faults on every transaction.
        inline static Kvasir::RateLimiter<Clock> faultLog_{};
        // address NAKs (debug): apart, so a scan does not spend the faults' budget and summary
        inline static Kvasir::LogRateLimiter<Clock> nakLog_{};
        // Set while reset() / requestRecovery() run the failure callbacks: a callback that
        // submits only queues (and maskDepth_ keeps the interrupt masked).
        inline static bool resetting_{};

        // How deep the masked sections are nested: a callback run inside one (expireDeadlines_,
        // a timeout's completion, reset()'s failures) may submit(), and its unmask must not open
        // the outer section to the interrupt. Only the outermost unmask enables it.
        inline static std::uint8_t maskDepth_{};

        // The bus interrupt masked around thread code that shares state with it. The ICER/ISER
        // writes alone are volatile stores that order nothing else: the section's plain loads could
        // move above the mask and its stores below the unmask, into reach of the interrupt. dsb;
        // isb after the mask so one already on its way is not taken after it either. Every
        // maskIsr_() has its unmaskIsr_().
        static void maskIsr_() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            asm volatile(
              "dsb\n"
              "isb\n"
              :
              :
              : "memory");
            ++maskDepth_;
        }

        static void unmaskIsr_() {
            std::atomic_signal_fence(std::memory_order_seq_cst);
            if(--maskDepth_ == 0) { apply(makeEnable(typename base::InterruptIndexs{})); }
        }

        // -- Public API -----------------------------------------------------------

        /// Every submitted request gets exactly one callback: the one on the wire and the
        /// ones queued behind it are failed here, before the block is reset, so nothing waits
        /// on a request the reset threw away. A callback that submits from here only queues:
        /// nothing starts on the block before it is back.
        static void reset() {
            maskIsr_();
            auto const outer = resetting_;
            resetting_       = true;
            failActive_();
            Recovery::resetState();
            drainQueueWithFailure();
            apply(Traits::I2C::getDisable<base::Instance>());
            apply(base::powerClockEnable);
            // RESET_DONE before any register of the block is written: see waitResetDone().
            Traits::I2C::waitResetDone<base::Instance>();
            apply(base::initStepPeripheryConfig);
            // The block is back at baudRate's counts: the next request writes its own.
            if constexpr(PerDeviceClock) { timingValid_ = false; }
            apply(base::initStepInterruptConfig);
            resetting_ = outer;
            // initStepPeripheryEnable's NVIC enable, unless reset() runs inside a masked section
            unmaskIsr_();
        }

        static bool submit(Request const& req) {
            maskIsr_();
            // inside the masked window, the push too: submit() also runs from completion callbacks
            // in the ISR, and the queue has one producer only if the two cannot interleave
            if(requestQueue_.size() >= requestQueue_.max_size()) {
                unmaskIsr_();
                return false;
            }
            requestQueue_.push(req);
            if constexpr(CountTransfers) { ++transfers_; }
            tryStart_(Clock::now());
            // From a callback reset() or requestRecovery() runs, the interrupt stays masked:
            // they unmask it when they are done.
            unmaskIsr_();
            return true;
        }

        /// submit(), with a ticket for cancel(); an invalid ticket (refused, no callback) when submit() would be false.
        static Bus::Ticket submitTracked(Request req)
            requires(Tracked)
        {
            maskIsr_();
            req.ticket = Bus::nextTicket(ticketCounter_);
            unmaskIsr_();
            return submit(req) ? Bus::Ticket{req.ticket} : Bus::Ticket{};
        }

        /// removed: queued, never started - its callback (cancelled) ran before this returns. stopping: on the
        /// wire - ENABLE.ABORT issues a STOP after the current byte, TX_ABRT completes it (cancelled) from the
        /// interrupt; its buffers belong to the bus until then. tooLate: completed already, or an unknown ticket.
        static Bus::Cancel cancel(Bus::Ticket t)
            requires(Cancellable)
        {
            if(!t.valid()) { return Bus::Cancel::tooLate; }
            maskIsr_();
            auto r = Bus::Cancel::tooLate;
            if(active_ && state_ != State::idle && currentRequest_.ticket == t.id) {
                if(!cancelPending_) {
                    cancelPending_ = true;
                    stopAs_        = Result::cancelled;
                    apply(base::cancelAbortRequest);
                }
                r = Bus::Cancel::stopping;
            } else {
                requestQueue_.forEachQueued([&](Request& q) {
                    if(q.ticket == t.id && !q.cancelled) {
                        q.cancelled = true;
                        if(q.callback) { q.callback(Result::cancelled); }
                        r = Bus::Cancel::removed;
                    }
                });
            }
            unmaskIsr_();
            return r;
        }

        // Bus-level handler — call once per main loop per bus.
        // Owns: timeout detection.  Delegates bus-health to Recovery.
        static void handler() {
            auto const now = Clock::now();

            // Recovery state machine — highest priority, blocks normal operation
            auto const rv = Recovery::tick(now);
            if(rv == Recovery::TickResult::needsReinit) {
                reset();
                // The limiter is shared with the ISR's fault lines, and reset() has just
                // unmasked the interrupt: the decision is taken with it masked again.
                maskIsr_();
                [[maybe_unused]] auto const recoveredLine
                  = faultLog_.allow(faultKey(Fault::recovered), now);
                unmaskIsr_();
                // The line states right after the sequence are the diagnosis: both high
                // and the bus is back, SDA low means a slave still holds data, SCL low
                // means no master can help.
                KVASIR_LOG_LIMITED(recoveredLine,
                                   UC_LOG_W,
                                   "i2c{} recovery #{} complete -- SDA {}, SCL {}, {} of 9 "
                                   "clocks unused, {} forced STOP(s) so far",
                                   base::Instance,
                                   Recovery::recoveries(),
                                   std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                                   std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"},
                                   Recovery::clocksLeft(),
                                   Recovery::forcedStops());
                return;
            }
            if(rv == Recovery::TickResult::busy) { return; }

            // What the ISR writes too, taken in one go with its interrupt masked: the faults the
            // log rate limiter dropped (reported now that the bus is quiet), the failure streak,
            // and the post-abort settle gate -- a 64-bit time, which an unmasked read could see
            // half-written. The ISR can land between any two of these.
            maskIsr_();
            auto const droppedFaults = faultLog_.takeSummary(now);
            auto const droppedNaks   = nakLog_.takeSummary(now);
            bool const deadBus       = consecutiveFailures_ >= kDeadBusFailures;
            if(deadBus) { consecutiveFailures_ = 0; }
            bool const settled = Recovery::isPastSettle(now);
            unmaskIsr_();
            if(droppedFaults != 0) {
                UC_LOG_W("i2c{} +{} faults not logged", base::Instance, droppedFaults);
            }
            if(droppedNaks != 0) {
                UC_LOG_D("i2c{} +{} address NAKs not logged", base::Instance, droppedNaks);
            }

            // Dead-bus watchdog: line-state checks miss failures on a healthy wire (an
            // unconfigured peripheral, a dropped address write), so this asks instead
            // whether anything works at all. A NAK counts as a completed transfer, so
            // absent devices cannot trip it.
            if(deadBus) {
                ++resuscitations_;
                // Alternate the cheap fix and the expensive one.
                if((resuscitations_ % 2U) == 1U) {
                    UC_LOG_W(
                      "i2c{} {} transfers failed in a row with no success -- "
                      "re-initialising the peripheral (SDA {}, SCL {})",
                      base::Instance,
                      kDeadBusFailures,
                      std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                      std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"});
                    reset();
                } else {
                    UC_LOG_W("i2c{} still dead after a re-initialise -- full bus recovery",
                             base::Instance);
                    requestRecovery();
                }
                return;
            }

            // Post-abort settle: bus was sick, wait before starting next transaction
            if(!settled) { return; }

            // Nothing active: a stuck SDA is looked for on every idle turn, not only when a
            // request waits, because a stuck bus parks every device as absent and that
            // leaves the queue empty for seconds. Then whatever waits may start, on the same
            // terms submit() starts it (tryStart_).
            if constexpr(Deadlines) {
                maskIsr_();
                expireDeadlines_(now);
                unmaskIsr_();
            }

            if(!active_) {
                if(Recovery::checkBusStuck(now)) { return; }
                maskIsr_();
                tryStart_(now);
                unmaskIsr_();
                return;
            }

            // Active transaction: check for timeout
            if(now > timeoutTime_) {
                maskIsr_();
                // Read again, masked: the interrupt may have finished the request (and started
                // the next, with a later deadline) since the check above, which also read the
                // 64-bit timeoutTime_ unmasked. Failing it now would fail the wrong request or
                // complete the finished one twice.
                if(!active_ || now <= timeoutTime_) {
                    unmaskIsr_();
                    return;
                }
                ++timeouts_;
                lastTimeout_ = snapshot_();
                KVASIR_LOG_LIMITED(
                  faultLog_.allow(faultKey(Fault::timeout), now),
                  UC_LOG_W,
                  "i2c{} timeout addr={:#04x} {}",
                  base::Instance,
                  currentRequest_.address,
                  Kvasir::Register::Flags<typename base::AbrtSrc>{base::abortCause()});
                apply(base::softAbortRequest);
                // a stop the caller asked for (cancel, deadline) whose TX_ABRT never came: still that outcome
                completeCurrentRequest(stoppingResult_(Result::failed));
                unmaskIsr_();
            }
        }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (StartUp/Hooks.hpp);
        // a firmware that runs the hook must not also call handler() by hand
        using Extends
          = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &I2CBehaviorQueued::handler>;

        // Request a full bus recovery sequence non-blocking.
        // Safe to call at any time. Any active transaction is immediately failed.
        static void requestRecovery() {
            maskIsr_();
            auto const outer = resetting_;
            resetting_       = true;
            failActive_();
            drainQueueWithFailure();
            Recovery::begin();
            resetting_ = outer;
            unmaskIsr_();
        }

        static bool isRecovering() { return Recovery::isActive(); }

        /// Failures with no success between them, and how often the watchdog stepped in.
        static std::uint32_t consecutiveFailures() { return consecutiveFailures_; }

        static std::uint32_t resuscitations() { return resuscitations_; }

        // -- fault counters -------------------------------------------------------------------
        //
        // Kept as numbers rather than only logged: the fault log's rate limiter shares one
        // budget across every kind of fault, and the log transport may drop lines under load,
        // so on a busy bus the log cannot say how often any of these happened. A poller reads
        // them and reports when one moves.

        /// What the block looked like when a transaction timed out, read before the abort.
        struct TimeoutSnapshot {
            std::uint32_t status{};         ///< IC_STATUS
            std::uint32_t rawIntr{};        ///< IC_RAW_INTR_STAT
            std::uint32_t intrMask{};       ///< IC_INTR_MASK
            std::uint32_t enableStatus{};   ///< IC_ENABLE_STATUS
            std::uint32_t abortSource{};    ///< IC_TX_ABRT_SOURCE, every bit
            std::uint32_t txLevel{};        ///< IC_TXFLR
            std::uint32_t rxLevel{};        ///< IC_RXFLR
            std::uint16_t sent{};
            std::uint16_t toSend{};
            std::uint16_t received{};
            std::uint16_t toReceive{};
            std::uint8_t  address{};
            std::uint8_t  state{};   ///< 0 idle, 1 sending, 2 receiving
            /// The lines themselves: SCL low with the master stalled is a part holding the
            /// clock (or the master holding it for a command that never came); both high is
            /// the block waiting on nothing the wire shows.
            bool          sdaHigh{};
            bool          sclHigh{};
            std::uint32_t tar{};          ///< IC_TAR: the address the block will actually use
            std::uint32_t enable{};       ///< IC_ENABLE, the ABORT bit included
            std::uint32_t isrEntries{};   ///< interrupt entries since this request started
            std::uint32_t usSinceIsr{};   ///< since the last interrupt entry, of any request
            std::uint32_t usAge{};        ///< since the request started
        };

        /// Transactions the handler gave up on after calcTransferTimeout().
        static std::uint32_t timeouts() { return timeouts_; }

        /// Requests accepted for the wire since boot: what tells "nothing talks on this bus"
        /// from "nothing went wrong on it" (I2CConfig::countTransfers only, 0 without it).
        static std::uint32_t transfers() {
            if constexpr(CountTransfers) {
                return transfers_;
            } else {
                return 0;
            }
        }

        /// Aborts after which SDA read low the moment the master went idle. Each one defers
        /// the next start; whether the line is really held is the idle watchdog's to decide,
        /// and Recovery::idleStuck() counts the times it did.
        static std::uint32_t sdaLowAfterAbort() { return sdaLowAfterAbort_; }

        /// Queued requests failed without going on the wire (a recovery emptying the queue).
        static std::uint32_t drainedRequests() { return drainedRequests_; }

        /// Times waitDisabled_() ran out of spins with the block still enabled, so the next
        /// IC_TAR write may have been dropped.
        static std::uint32_t disableWaitsExhausted() { return disableWaitsExhausted_; }

        /// Times startNext() rewrote the SCL counts for a device at another clock
        /// (perDeviceClock only, 0 without it).
        static std::uint32_t clockSwitches() {
            if constexpr(PerDeviceClock) {
                return clockSwitches_;
            } else {
                return 0;
            }
        }

        static TimeoutSnapshot const& lastTimeout() { return lastTimeout_; }

        /// lastTimeout() as a log line. The snapshot's fields are this block's registers, so
        /// the line is the driver's too: the SAM SERCOM driver has the same call and says
        /// what its block looked like (state 1 is sending, 2 receiving; sent/received are
        /// bytes done of bytes asked).
        static void logLastTimeout() {
            [[maybe_unused]] auto const& t = lastTimeout_;
            UC_LOG_W(
              "i2c{} last timeout: addr {:#04x}, state {}, sent {}/{}, received {}/{}, "
              "IC_STATUS {:#06x}, IC_RAW_INTR_STAT {:#06x}, IC_INTR_MASK {:#06x}, "
              "IC_ENABLE_STATUS {:#x}, IC_TX_ABRT_SOURCE {:#010x}, TX FIFO {}, RX FIFO {}, "
              "SDA {}, SCL {}, IC_TAR {:#x}, IC_ENABLE {:#x}, {} interrupt(s) in the "
              "request, last {} us before, request {} us old",
              base::Instance,
              t.address,
              t.state,
              t.sent,
              t.toSend,
              t.received,
              t.toReceive,
              t.status,
              t.rawIntr,
              t.intrMask,
              t.enableStatus,
              t.abortSource,
              t.txLevel,
              t.rxLevel,
              std::string_view{t.sdaHigh ? "high" : "LOW"},
              std::string_view{t.sclHigh ? "high" : "LOW"},
              t.tar,
              t.enable,
              t.isrEntries,
              t.usSinceIsr,
              t.usAge);
        }

        /// Interrupt entries that found nothing to do for the state the request was in: a
        /// send with TX_EMPTY clear, a receive with the RX FIFO empty. See onIsr().
        static std::uint32_t spuriousIsr() { return spuriousIsr_; }

        /// The longest a request waited for an interrupt since the last call: from its start to
        /// its first entry, and between two entries of one request. Taken and cleared.
        struct Latency {
            std::uint32_t firstIsrUs{};
            std::uint32_t isrGapUs{};
        };

        // The interrupt keeps its waits in the clock's own ticks, 32 bits, saturated (2^32 ticks
        // are 28 s or more: "very long" for a latency). They become microseconds where they
        // are read: in the handler that was a 64-bit division per interrupt.
        static constexpr std::uint32_t ticks32_(typename Clock::duration d) {
            auto const n = static_cast<std::uint64_t>(d.count());
            return n > 0xFFFF'FFFFU ? 0xFFFF'FFFFU : static_cast<std::uint32_t>(n);
        }

        static constexpr std::uint32_t usOfTicks_(std::uint32_t ticks) {
            return static_cast<std::uint32_t>(
              std::chrono::duration_cast<std::chrono::microseconds>(typename Clock::duration{ticks})
                .count());
        }

        static Latency takeLatency() {
            maskIsr_();
            std::uint32_t const first = longestFirstIsrTicks_;
            std::uint32_t const gap   = longestIsrGapTicks_;
            longestFirstIsrTicks_     = 0;
            longestIsrGapTicks_       = 0;
            unmaskIsr_();
            return Latency{usOfTicks_(first), usOfTicks_(gap)};
        }

    private:
        // only odr-used (so only defined) on a tracked bus
        inline static std::uint16_t ticketCounter_{};
        inline static bool   cancelPending_{};   // our ABORT is out; TX_ABRT completes the request
        inline static Result stopAs_{};          // cancelled or timedOut, for that completion

        /// What a request stopped by us ends as when the transfer timeout catches it first.
        static Result stoppingResult_(Result otherwise) {
            if constexpr(Tracked) {
                if(cancelPending_) { return stopAs_; }
            }
            return otherwise;
        }

        /// Interrupt masked: the request on the wire past its deadline is stopped like a cancel (timedOut);
        /// queued ones past theirs become tombstones with their callback (timedOut).
        static void expireDeadlines_(tp now)
            requires(Deadlines)
        {
            if(active_ && state_ != State::idle && !cancelPending_
               && now > currentRequest_.deadline)
            {
                cancelPending_ = true;
                stopAs_        = Result::timedOut;
                apply(base::cancelAbortRequest);
            }
            requestQueue_.forEachQueued([&](Request& q) {
                if(!q.cancelled && now > q.deadline) {
                    q.cancelled = true;
                    ++drainedRequests_;
                    if(q.callback) { q.callback(Result::timedOut); }
                }
            });
        }

        inline static std::uint32_t spuriousIsr_{};
        inline static std::size_t queuedReads_{};   ///< receivedCount_ once the queued reads are in
        inline static std::uint32_t isrEntries_{};
        inline static tp            lastIsr_{};
        inline static tp            requestStart_{};
        inline static std::uint32_t longestFirstIsrTicks_{};
        inline static std::uint32_t longestIsrGapTicks_{};

        inline static std::uint32_t   timeouts_{};
        inline static std::uint32_t   transfers_{};   // countTransfers only: never used without it
        inline static std::uint32_t   sdaLowAfterAbort_{};
        inline static std::uint32_t   drainedRequests_{};
        inline static std::uint32_t   disableWaitsExhausted_{};
        inline static TimeoutSnapshot lastTimeout_{};

        // perDeviceClock: the counts in the block, and whether they are known (not after a
        // reset). Members of a class template: never instantiated on a bus without it.
        inline static ClockTiming   timing_{};
        inline static bool          timingValid_{};
        inline static std::uint32_t clockSwitches_{};

        static TimeoutSnapshot snapshot_() {
            return TimeoutSnapshot{
              .status       = get<0>(apply(read(Regs::IC_STATUS::FULLREGISTER))),
              .rawIntr      = get<0>(apply(read(Regs::IC_RAW_INTR_STAT::FULLREGISTER))),
              .intrMask     = get<0>(apply(read(Regs::IC_INTR_MASK::FULLREGISTER))),
              .enableStatus = get<0>(apply(read(Regs::IC_ENABLE_STATUS::FULLREGISTER))),
              .abortSource  = get<0>(apply(read(Regs::IC_TX_ABRT_SOURCE::FULLREGISTER))),
              .txLevel      = get<0>(apply(read(Regs::IC_TXFLR::FULLREGISTER))),
              .rxLevel      = get<0>(apply(read(Regs::IC_RXFLR::FULLREGISTER))),
              .sent         = static_cast<std::uint16_t>(sendIndex_),
              .toSend       = static_cast<std::uint16_t>(currentRequest_.sendBytes()),
              .received     = static_cast<std::uint16_t>(receivedCount_),
              .toReceive    = static_cast<std::uint16_t>(currentRequest_.receiveData.size()),
              .address      = currentRequest_.address,
              .state        = static_cast<std::uint8_t>(state_),
              .sdaHigh      = Recovery::sdaIsHigh(),
              .sclHigh      = Recovery::sclIsHigh(),
              .tar          = get<0>(apply(read(Regs::IC_TAR::FULLREGISTER))),
              .enable       = get<0>(apply(read(Regs::IC_ENABLE::FULLREGISTER))),
              .isrEntries   = isrEntries_,
              .usSinceIsr   = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - lastIsr_)
                  .count()),
              .usAge = static_cast<std::uint32_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - requestStart_)
                  .count()),
            };
        }

        enum class Fault : std::uint8_t {
            abortSend = 1,
            abortRecv,
            timeout,
            sdaStuck,
            recovered,
        };

        // One kind of fault at one address with one cause, for the log rate limiter.
        static std::uint32_t faultKey(Fault                                 kind,
                                      typename base::AbrtSrc::Addr::RegType cause = 0) {
            return Kvasir::rateLimitKey(kind, currentRequest_.address, cause);
        }

        /// The SCL counts of the request about to start, written only when they differ from
        /// the last ones. HCNT, LCNT and SPKLEN take a write only while the block is disabled
        /// ("Writes at other times have no effect", RP2350 datasheet 12.2, Tables 1062, 1063,
        /// 1093; RP2040 4.3): called after waitDisabled_(). IC_SDA_HOLD has no such rule
        /// (Table 1084). IC_CON.SPEED stays fast for every rate (Config::getSpeedModeRegister).
        /// The pads keep the drive chosen for baudRate (Config::i2cDrive), which is at least
        /// as strong as a slower device's. Interrupts disabled or in the ISR.
        static void applyTiming_(ClockTiming const& t) {
            if(timingValid_ && t == timing_) { return; }
            apply(write(Regs::IC_FS_SCL_HCNT::ic_fs_scl_hcnt, std::uint32_t{t.hcnt}),
                  write(Regs::IC_FS_SCL_LCNT::ic_fs_scl_lcnt, std::uint32_t{t.lcnt}),
                  write(Regs::IC_FS_SPKLEN::ic_fs_spklen, std::uint32_t{t.spklen}));
            apply(write(Regs::IC_SDA_HOLD::ic_sda_tx_hold, std::uint32_t{t.sdaHold}));
            timing_      = t;
            timingValid_ = true;
            ++clockSwitches_;
        }

        /// Spin until IC_ENABLE_STATUS says the block is inactive. Bounded: this runs in
        /// the ISR, and a wedged block is left to the dead-bus watchdog.
        static void waitDisabled_() {
            for(std::uint32_t spins = 0; spins < 100'000U; ++spins) {
                if(!fieldEquals(Regs::IC_ENABLE_STATUS::IC_ENValC::enabled)) { return; }
            }
            ++disableWaitsExhausted_;
        }

        static void drainQueueWithFailure() {
            while(!requestQueue_.empty()) {
                Request req{};
                requestQueue_.pop_into(req);
                if constexpr(Tracked) {
                    if(req.cancelled) { continue; }   // a tombstone: its callback has run
                }
                ++drainedRequests_;
                if(req.callback) { req.callback(Result::failed); }
            }
        }

        /// The request on the wire, if there is one, is over: its callback runs once, as
        /// failed. The block is left for the caller to reset or abort, and the dead-bus
        /// counter is not touched -- this is the bus giving up on a request, not a request
        /// reporting on the bus. Interrupts disabled.
        static void failActive_() {
            if(!active_) { return; }
            active_ = false;
            state_  = State::idle;
            if(currentRequest_.callback) { currentRequest_.callback(Result::failed); }
        }

        /// Start the next queued request if the bus may take one: no reset() or
        /// requestRecovery() failing requests, nothing on the wire, no recovery running, the
        /// post-abort settle over, SDA released, and something waiting. The one definition of
        /// "may start", shared by submit() and handler(); the ISR's own startNext() after a
        /// success needs none of these checks. Interrupts disabled.
        static bool tryStart_(tp now) {
            if(resetting_ || active_ || Recovery::isActive() || !Recovery::isPastSettle(now)) {
                return false;
            }
            if(requestQueue_.empty() || !Recovery::sdaIsHigh()) { return false; }
            startNext();
            return true;
        }

        static void startNext() {
            if constexpr(!Tracked) {
                if(requestQueue_.empty()) {
                    active_ = false;
                    return;
                }
            }

            Request req{};
            if constexpr(!Tracked) {
                requestQueue_.pop_into(req);
            } else {
                while(true) {
                    if(!requestQueue_.pop_into(req)) {
                        active_ = false;
                        return;
                    }
                    if(req.cancelled) { continue; }   // a tombstone: dropped
                    if constexpr(Deadlines) {
                        if(Clock::now() > req.deadline) {   // out of time before it started
                            ++drainedRequests_;
                            if(req.callback) { req.callback(Result::timedOut); }
                            continue;
                        }
                    }
                    break;
                }
            }
            currentRequest_ = req;
            active_         = true;
            sendIndex_      = 0;
            receivedCount_  = 0;
            isrEntries_     = 0;
            // Every start goes through here -- submit(), handler(), and the ISR chaining the
            // next request after a success -- and each one is the bus seen free.
            Recovery::noteBusFree();

            auto const totalBytes
              = currentRequest_.sendBytes() + currentRequest_.receiveData.size();
            requestStart_ = Clock::now();
            if constexpr(PerDeviceClock) {
                timeoutTime_
                  = requestStart_
                  + base::calcTransferTimeout(totalBytes, currentRequest_.timing.usPerByte);
            } else {
                timeoutTime_ = requestStart_ + base::calcTransferTimeout(totalBytes);
            }

            // ENABLE.ABORT after a NAK raises its own TX_ABRT (ABRT_USER_ABRT) once the
            // abort has gone through, which is after the ISR that issued it has cleared the
            // source. Left in place it fires the moment this request unmasks TX_ABRT and
            // fails it with the previous request's cause.
            base::clearAbortSource();

            // IC_TAR is writable only while the block is disabled, and IC_ENABLE=0 takes
            // effect only once the master is done: without this wait the address write
            // below is dropped and the transfer goes out to the previous address.
            waitDisabled_();

            if constexpr(PerDeviceClock) { applyTiming_(currentRequest_.timing); }

            apply(write(Regs::IC_TAR::ic_tar, currentRequest_.address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));

            bool const hasSend = currentRequest_.sendBytes() != 0;
            bool const hasRecv = !currentRequest_.receiveData.empty();

            if(hasSend) {
                stop   = !hasRecv;
                state_ = State::sending;
                apply(base::TxInterrupts);
            } else {
                stop   = true;
                state_ = State::receiving;
                queueReads_(false);
                apply(base::RxInterrupts);
            }
        }

        /// The DW_apb_i2c FIFOs hold 16 entries each way (IC_TX_BUFFER_DEPTH, IC_RX_BUFFER_DEPTH).
        static constexpr std::size_t FifoDepth = 16;

        /// Queues the request's next read commands -- as many as are left, up to a FIFO's worth
        /// -- the first with RESTART after a send, the request's last with STOP, and sets
        /// IC_RX_TL so RX_FULL fires once all of them have come in: one interrupt for up to 16
        /// bytes where there was one a byte. Interrupts disabled or in the ISR.
        static void queueReads_(bool restart) {
            auto const left = currentRequest_.receiveData.size() - receivedCount_;
            auto const n    = std::min<std::size_t>(left, FifoDepth);
            for(std::size_t i = 0; i < n; ++i) {
                bool const first = restart && i == 0;
                bool const last  = i + 1 == left;
                if(first && last) {
                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                      write(Regs::IC_DATA_CMD::RESTARTValC::enable),
                      write(Regs::IC_DATA_CMD::STOPValC::enable),
                      write(Regs::IC_DATA_CMD::CMDValC::read)));
                } else if(first) {
                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                      write(Regs::IC_DATA_CMD::RESTARTValC::enable),
                      write(Regs::IC_DATA_CMD::CMDValC::read)));
                } else if(last) {
                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                      write(Regs::IC_DATA_CMD::STOPValC::enable),
                      write(Regs::IC_DATA_CMD::CMDValC::read)));
                } else {
                    apply(
                      Regs::IC_DATA_CMD::overrideDefaults(write(Regs::IC_DATA_CMD::CMDValC::read)));
                }
            }
            queuedReads_ = receivedCount_ + n;
            apply(write(Regs::IC_RX_TL::rx_tl, static_cast<std::uint32_t>(n - 1)));
        }

        static void completeCurrentRequest(Result result) {
            if constexpr(Tracked) { cancelPending_ = false; }
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::disabled)));
            apply(base::NoInterrupts);

            // The request is over before its callback runs. An interrupt raised between the
            // ISR's status read and the mask above can already be pending in the NVIC
            // (ENABLE.ABORT after a NAK raises a second TX_ABRT, ABRT_USER_ABRT, by itself),
            // and the re-entry has to find no request to finish: left at `receiving` it read
            // the empty RX FIFO as the byte asked for and completed the NAKed request again.
            state_ = State::idle;

            // A NAK is a transfer that completed: the address went out and nobody took it,
            // which says the wire works. Only a bus fault counts towards a dead bus.
            if(result == Result::failed) {
                if(consecutiveFailures_ != std::numeric_limits<std::uint32_t>::max()) {
                    ++consecutiveFailures_;
                }
            } else {
                consecutiveFailures_ = 0;
            }

            if(currentRequest_.callback) { currentRequest_.callback(result); }

            if(result != Result::succeeded) {
                // Guard against cascading timeouts: if the master FSM is still active after
                // aborting, the bus may still be held. Defer startNext() for a brief settle.
                // This is the normal path after a NACK (the STOP is still propagating),
                // so it is not logged; the abort itself was.
                if(fieldEquals(Regs::IC_STATUS::MST_ACTIVITYValC::active)) {
                    active_ = false;
                    Recovery::deferSettle(Clock::now() + std::chrono::milliseconds{1});
                    return;
                }

                // Master FSM is idle: a STOP has gone out, so SDA should be on its way up.
                // One look at it this early is not a stuck bus: on a long, heavily loaded
                // wire the pull-ups take their time, and this read is the instant after the
                // FSM went idle. So it only defers the next start; the idle watchdog
                // (checkBusStuck) then asks for a recovery if the line is still held after
                // kStuckThreshold, and tryStart_ starts nothing while SDA is low.
                //
                // SDA only: a momentarily low SCL here is ordinary while the abort
                // settles. A genuinely held clock is left to the idle watchdog.
                if(!Recovery::sdaIsHigh()) {
                    ++sdaLowAfterAbort_;
                    active_ = false;
                    Recovery::deferSettle(Clock::now() + std::chrono::milliseconds{1});
                    return;
                }
            }
            startNext();
        }

    public:
        // Every entry is checked for the event its state waits on before anything is done with
        // it. A request completed here starts the next one here (completeCurrentRequest ->
        // startNext), so an entry the NVIC latched for the old request can arrive once the
        // new one is already sending or receiving: taken at face value, a receive would store
        // whatever IC_DATA_CMD holds with the FIFO empty and queue one read command too many,
        // and a send would move on before its byte was out. Such an entry is counted and dropped.
        static void onIsr() {
            {
                // How long the request waited for this entry: from its start for the first,
                // from the entry before for the rest. A byte at 400 kHz is ~25 us, so anything
                // in milliseconds is the interrupt held off, or a part stretching the clock.
                auto const now     = Clock::now();
                auto const waited  = ticks32_(now - (isrEntries_ == 0 ? requestStart_ : lastIsr_));
                auto&      longest = isrEntries_ == 0 ? longestFirstIsrTicks_ : longestIsrGapTicks_;
                if(state_ != State::idle && waited > longest) { longest = waited; }
                ++isrEntries_;
                lastIsr_ = now;
            }
            bool const error = fieldEquals(Regs::IC_INTR_STAT::R_TX_ABRTValC::active);

            // our own ABORT (cancel, deadline): ABRT_USER_ABRT alone, not a fault; a NAK or a real fault that won the
            // race keeps its own result (the request was over first)
            if constexpr(Tracked) {
                if(error && cancelPending_ && state_ != State::idle
                   && (base::abortCause() & base::MasterAbortCauses) == 0)
                {
                    base::clearAbortSource();
                    apply(base::abort);
                    completeCurrentRequest(stopAs_);
                    return;
                }
            }

            if(state_ == State::sending) {
                if(error) {
                    auto const cause = base::abortCause();
                    bool const isNak = (cause & base::AbrtSrc::abrt_7b_addr_noack.Mask) != 0;
                    // An address NAK alone is an answer - nobody at that address, every probe
                    // of a scan - not a bus fault: debug. A known part that stops answering is
                    // Presence's warning ("not responding"); a data NAK stays a warning.
                    if(cause == base::AbrtSrc::abrt_7b_addr_noack.Mask) {
                        KVASIR_LOG_LIMITED(nakLog_.allow(faultKey(Fault::abortSend, cause)),
                                           UC_LOG_D,
                                           "i2c{} send addr={:#04x}: no ACK, nobody there",
                                           base::Instance,
                                           currentRequest_.address);
                    } else {
                        KVASIR_LOG_LIMITED(faultLog_.allow(faultKey(Fault::abortSend, cause)),
                                           UC_LOG_W,
                                           "i2c{} abort send addr={:#04x} {}",
                                           base::Instance,
                                           currentRequest_.address,
                                           Kvasir::Register::Flags<typename base::AbrtSrc>{cause});
                    }
                    base::clearAbortSource();
                    apply(base::abort);
                    completeCurrentRequest(isNak ? Result::notAcknowledged : Result::failed);
                    return;
                }

                if(!fieldEquals(Regs::IC_RAW_INTR_STAT::TX_EMPTYValC::active)) {
                    ++spuriousIsr_;
                    return;
                }

                // The prefix and sendData go into the FIFO in one pass: without STOP on a byte
                // the master goes on with the next FIFO entry, or holds SCL low until there is
                // one (RP2350 data sheet 12.2.7.1, IC_DATA_CMD.STOP in Table 1059).
                auto const total = currentRequest_.sendBytes();
                if(sendIndex_ < total) {
                    // As many bytes as the FIFO has room for: TX_EMPTY comes back once they
                    // are all out, where it used to come back for each one.
                    auto room = FifoDepth - get<0>(apply(read(Regs::IC_TXFLR::txflr)));
                    while(room != 0 && sendIndex_ < total) {
                        auto const byte = currentRequest_.sendByte(sendIndex_++);
                        if(sendIndex_ == total && stop) {
                            Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                              write(Regs::IC_DATA_CMD::STOPValC::enable),
                              write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(byte)));
                        } else {
                            Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                              write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(byte)));
                        }
                        --room;
                    }
                } else {
                    if(stop) {
                        completeCurrentRequest(Result::succeeded);
                    } else {
                        state_ = State::receiving;
                        queueReads_(true);
                        apply(base::RxInterrupts);
                    }
                }
            } else if(state_ == State::receiving) {
                if(error) {
                    auto const cause = base::abortCause();
                    bool const isNak = (cause & base::AbrtSrc::abrt_7b_addr_noack.Mask) != 0;
                    // an address NAK alone is an answer, not a fault (see the send side)
                    if(cause == base::AbrtSrc::abrt_7b_addr_noack.Mask) {
                        KVASIR_LOG_LIMITED(nakLog_.allow(faultKey(Fault::abortRecv, cause)),
                                           UC_LOG_D,
                                           "i2c{} recv addr={:#04x}: no ACK, nobody there",
                                           base::Instance,
                                           currentRequest_.address);
                    } else {
                        KVASIR_LOG_LIMITED(faultLog_.allow(faultKey(Fault::abortRecv, cause)),
                                           UC_LOG_W,
                                           "i2c{} abort recv addr={:#04x} {}",
                                           base::Instance,
                                           currentRequest_.address,
                                           Kvasir::Register::Flags<typename base::AbrtSrc>{cause});
                    }
                    base::clearAbortSource();
                    apply(base::abort);
                    completeCurrentRequest(isNak ? Result::notAcknowledged : Result::failed);
                    return;
                }

                if(get<0>(apply(read(Regs::IC_RXFLR::rxflr))) == 0) {
                    ++spuriousIsr_;
                    return;
                }

                // Everything that has come in, never more than was asked for: a byte past the
                // read commands queued is not this request's.
                auto available = get<0>(apply(read(Regs::IC_RXFLR::rxflr)));
                while(available != 0 && receivedCount_ < queuedReads_) {
                    auto const data = apply(read(Regs::IC_DATA_CMD::dat));
                    currentRequest_.receiveData[receivedCount_++]
                      = static_cast<std::byte>(Kvasir::Register::get<0>(data));
                    --available;
                }
                if(receivedCount_ == currentRequest_.receiveData.size()) {
                    completeCurrentRequest(Result::succeeded);
                } else if(receivedCount_ == queuedReads_) {
                    queueReads_(false);
                } else {
                    // Fewer than the threshold asked for (an entry that came early): wait for
                    // the rest of this batch.
                    apply(write(Regs::IC_RX_TL::rx_tl,
                                static_cast<std::uint32_t>(queuedReads_ - receivedCount_ - 1)));
                }
            } else {
                // Idle: a stale entry (see completeCurrentRequest). Whatever it carried is
                // over; clear the abort source so the next request reports its own.
                base::clearAbortSource();
            }
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };
}}   // namespace Kvasir::I2C
