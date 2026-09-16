#pragma once

#include "I2CBusRecovery.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>

namespace Kvasir { namespace I2C {

    enum class I2CRequestResult : std::uint8_t { failed, notAcknowledged, succeeded };

    template<std::size_t CallbackSize>
    struct I2CRequest {
        std::uint8_t                                         address{};
        std::span<std::byte const>                           sendData{};
        std::span<std::byte>                                 receiveData{};
        StaticFunction<void(I2CRequestResult), CallbackSize> callback{};
    };

    template<typename I2CConfig, typename Clock, std::size_t QueueDepth_, std::size_t CallbackSize_>
    struct I2CBehaviorQueued : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t QueueDepth   = QueueDepth_;
        static constexpr std::size_t CallbackSize = CallbackSize_;
        /// Re-exported from the config so a bus can size its traffic against the bandwidth.
        static constexpr auto BaudRate = I2CConfig::baudRate;
        using base                     = Detail::I2CBase<I2CConfig>;
        using Regs                     = typename base::Regs;
        using tp                       = typename Clock::time_point;
        using Request                  = I2CRequest<CallbackSize>;
        using Result                   = I2CRequestResult;
        using Recovery                 = I2CBusRecovery<I2CConfig, Clock>;

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
        // Set while reset() / requestRecovery() run the failure callbacks: a callback that
        // submits only queues, and leaves the interrupt masked.
        inline static bool resetting_{};

        // -- Public API -----------------------------------------------------------

        /// Every submitted request gets exactly one callback: the one on the wire and the
        /// ones queued behind it are failed here, before the block is reset, so nothing waits
        /// on a request the reset threw away. A callback that submits from here only queues:
        /// nothing starts on the block before it is back.
        static void reset() {
            apply(makeDisable(typename base::InterruptIndexs{}));
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
            apply(base::initStepInterruptConfig);
            resetting_ = outer;
            apply(base::initStepPeripheryEnable);
        }

        static bool submit(Request const& req) {
            if(requestQueue_.size() >= requestQueue_.max_size()) { return false; }
            requestQueue_.push(req);

            apply(makeDisable(typename base::InterruptIndexs{}));
            tryStart_(Clock::now());
            // From a callback reset() or requestRecovery() runs, the interrupt stays masked:
            // they unmask it when they are done.
            if(!resetting_) { apply(makeEnable(typename base::InterruptIndexs{})); }
            return true;
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
                apply(makeDisable(typename base::InterruptIndexs{}));
                [[maybe_unused]] auto const recoveredLine
                  = faultLog_.allow(faultKey(Fault::recovered), now);
                apply(makeEnable(typename base::InterruptIndexs{}));
                // The line states right after the sequence are the diagnosis: both high
                // and the bus is back, SDA low means a slave still holds data, SCL low
                // means no master can help.
                KVASIR_LOG_LIMITED(recoveredLine,
                                   UC_LOG_W,
                                   "i2c{} recovery #{} complete -- SDA {}, SCL {}, {} of 9 "
                                   "clocks unused",
                                   base::Instance,
                                   Recovery::recoveries(),
                                   std::string_view{Recovery::sdaIsHigh() ? "high" : "LOW"},
                                   std::string_view{Recovery::sclIsHigh() ? "high" : "LOW"},
                                   Recovery::clocksLeft());
                return;
            }
            if(rv == Recovery::TickResult::busy) { return; }

            // What the ISR writes too, taken in one go with its interrupt masked: the faults the
            // log rate limiter dropped (reported now that the bus is quiet), the failure streak,
            // and the post-abort settle gate -- a 64-bit time, which an unmasked read could see
            // half-written. The ISR can land between any two of these.
            apply(makeDisable(typename base::InterruptIndexs{}));
            auto const droppedFaults = faultLog_.takeSummary(now);
            bool const deadBus       = consecutiveFailures_ >= kDeadBusFailures;
            if(deadBus) { consecutiveFailures_ = 0; }
            bool const settled = Recovery::isPastSettle(now);
            apply(makeEnable(typename base::InterruptIndexs{}));
            if(droppedFaults != 0) {
                UC_LOG_W("i2c{} +{} faults not logged", base::Instance, droppedFaults);
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
            if(!active_) {
                if(Recovery::checkBusStuck(now)) { return; }
                apply(makeDisable(typename base::InterruptIndexs{}));
                tryStart_(now);
                apply(makeEnable(typename base::InterruptIndexs{}));
                return;
            }

            // Active transaction: check for timeout
            if(now > timeoutTime_) {
                apply(makeDisable(typename base::InterruptIndexs{}));
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
                completeCurrentRequest(I2CRequestResult::failed);
                apply(makeEnable(typename base::InterruptIndexs{}));
            }
        }

        // Request a full bus recovery sequence non-blocking.
        // Safe to call at any time. Any active transaction is immediately failed.
        static void requestRecovery() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            auto const outer = resetting_;
            resetting_       = true;
            failActive_();
            drainQueueWithFailure();
            Recovery::begin();
            resetting_ = outer;
            if(!outer) { apply(makeEnable(typename base::InterruptIndexs{})); }
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

        /// Aborts after which SDA read low the moment the master went idle. Each one defers
        /// the next start; whether the line is really held is the idle watchdog's to decide,
        /// and Recovery::idleStuck() counts the times it did.
        static std::uint32_t sdaLowAfterAbort() { return sdaLowAfterAbort_; }

        /// Queued requests failed without going on the wire (a recovery emptying the queue).
        static std::uint32_t drainedRequests() { return drainedRequests_; }

        /// Times waitDisabled_() ran out of spins with the block still enabled, so the next
        /// IC_TAR write may have been dropped.
        static std::uint32_t disableWaitsExhausted() { return disableWaitsExhausted_; }

        static TimeoutSnapshot const& lastTimeout() { return lastTimeout_; }

        /// Interrupt entries that found nothing to do for the state the request was in: a
        /// send with TX_EMPTY clear, a receive with the RX FIFO empty. See onIsr().
        static std::uint32_t spuriousIsr() { return spuriousIsr_; }

        /// The longest a request waited for an interrupt since the last call: from its start to
        /// its first entry, and between two entries of one request. Taken and cleared.
        struct Latency {
            std::uint32_t firstIsrUs{};
            std::uint32_t isrGapUs{};
        };

        static Latency takeLatency() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            Latency const l{longestFirstIsrUs_, longestIsrGapUs_};
            longestFirstIsrUs_ = 0;
            longestIsrGapUs_   = 0;
            apply(makeEnable(typename base::InterruptIndexs{}));
            return l;
        }

    private:
        inline static std::uint32_t spuriousIsr_{};
        inline static std::size_t queuedReads_{};   ///< receivedCount_ once the queued reads are in
        inline static std::uint32_t isrEntries_{};
        inline static tp            lastIsr_{};
        inline static tp            requestStart_{};
        inline static std::uint32_t longestFirstIsrUs_{};
        inline static std::uint32_t longestIsrGapUs_{};

        inline static std::uint32_t   timeouts_{};
        inline static std::uint32_t   sdaLowAfterAbort_{};
        inline static std::uint32_t   drainedRequests_{};
        inline static std::uint32_t   disableWaitsExhausted_{};
        inline static TimeoutSnapshot lastTimeout_{};

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
              .toSend       = static_cast<std::uint16_t>(currentRequest_.sendData.size()),
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
                ++drainedRequests_;
                if(req.callback) { req.callback(I2CRequestResult::failed); }
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
            if(currentRequest_.callback) { currentRequest_.callback(I2CRequestResult::failed); }
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
            if(requestQueue_.empty()) {
                active_ = false;
                return;
            }

            Request req{};
            requestQueue_.pop_into(req);
            currentRequest_ = req;
            active_         = true;
            sendIndex_      = 0;
            receivedCount_  = 0;
            isrEntries_     = 0;

            auto const totalBytes
              = currentRequest_.sendData.size() + currentRequest_.receiveData.size();
            requestStart_ = Clock::now();
            timeoutTime_  = requestStart_ + base::calcTransferTimeout(totalBytes);

            // ENABLE.ABORT after a NAK raises its own TX_ABRT (ABRT_USER_ABRT) once the
            // abort has gone through, which is after the ISR that issued it has cleared the
            // source. Left in place it fires the moment this request unmasks TX_ABRT and
            // fails it with the previous request's cause.
            base::clearAbortSource();

            // IC_TAR is writable only while the block is disabled, and IC_ENABLE=0 takes
            // effect only once the master is done: without this wait the address write
            // below is dropped and the transfer goes out to the previous address.
            waitDisabled_();

            apply(write(Regs::IC_TAR::ic_tar, currentRequest_.address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));

            bool const hasSend = !currentRequest_.sendData.empty();
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

        static void completeCurrentRequest(I2CRequestResult result) {
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
            if(result == I2CRequestResult::failed) {
                if(consecutiveFailures_ != std::numeric_limits<std::uint32_t>::max()) {
                    ++consecutiveFailures_;
                }
            } else {
                consecutiveFailures_ = 0;
            }

            if(currentRequest_.callback) { currentRequest_.callback(result); }

            if(result != I2CRequestResult::succeeded) {
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
                auto const now    = Clock::now();
                auto const waited = static_cast<std::uint32_t>(
                  std::chrono::duration_cast<std::chrono::microseconds>(
                    now - (isrEntries_ == 0 ? requestStart_ : lastIsr_))
                    .count());
                auto& longest = isrEntries_ == 0 ? longestFirstIsrUs_ : longestIsrGapUs_;
                if(state_ != State::idle && waited > longest) { longest = waited; }
                ++isrEntries_;
                lastIsr_ = now;
            }
            bool const error = fieldEquals(Regs::IC_INTR_STAT::R_TX_ABRTValC::active);

            if(state_ == State::sending) {
                if(error) {
                    auto const cause = base::abortCause();
                    bool const isNak = (cause & base::AbrtSrc::abrt_7b_addr_noack.Mask) != 0;
                    KVASIR_LOG_LIMITED(faultLog_.allow(faultKey(Fault::abortSend, cause)),
                                       UC_LOG_W,
                                       "i2c{} abort send addr={:#04x} {}",
                                       base::Instance,
                                       currentRequest_.address,
                                       Kvasir::Register::Flags<typename base::AbrtSrc>{cause});
                    base::clearAbortSource();
                    apply(base::abort);
                    completeCurrentRequest(isNak ? I2CRequestResult::notAcknowledged
                                                 : I2CRequestResult::failed);
                    return;
                }

                if(!fieldEquals(Regs::IC_RAW_INTR_STAT::TX_EMPTYValC::active)) {
                    ++spuriousIsr_;
                    return;
                }

                if(sendIndex_ < currentRequest_.sendData.size()) {
                    // As many bytes as the FIFO has room for: TX_EMPTY comes back once they
                    // are all out, where it used to come back for each one.
                    auto room = FifoDepth - get<0>(apply(read(Regs::IC_TXFLR::txflr)));
                    while(room != 0 && sendIndex_ < currentRequest_.sendData.size()) {
                        auto const byte = currentRequest_.sendData[sendIndex_++];
                        if(sendIndex_ == currentRequest_.sendData.size() && stop) {
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
                        completeCurrentRequest(I2CRequestResult::succeeded);
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
                    KVASIR_LOG_LIMITED(faultLog_.allow(faultKey(Fault::abortRecv, cause)),
                                       UC_LOG_W,
                                       "i2c{} abort recv addr={:#04x} {}",
                                       base::Instance,
                                       currentRequest_.address,
                                       Kvasir::Register::Flags<typename base::AbrtSrc>{cause});
                    base::clearAbortSource();
                    apply(base::abort);
                    completeCurrentRequest(isNak ? I2CRequestResult::notAcknowledged
                                                 : I2CRequestResult::failed);
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
                    completeCurrentRequest(I2CRequestResult::succeeded);
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
