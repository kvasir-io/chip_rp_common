#pragma once

#include "I2CBusRecovery.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <span>

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
        using base                                = Detail::I2CBase<I2CConfig>;
        using Regs                                = typename base::Regs;
        using tp                                  = typename Clock::time_point;
        using Request                             = I2CRequest<CallbackSize>;
        using Result                              = I2CRequestResult;
        using Recovery                            = I2CBusRecovery<I2CConfig, Clock>;

        enum class State { idle, sending, receiving };

        inline static Kvasir::Atomic::Queue<Request, QueueDepth> requestQueue_{};
        inline static Request                                    currentRequest_{};
        inline static bool                                       active_{false};
        inline static State                                      state_{State::idle};
        inline static std::uint8_t                               sendIndex_{0};
        inline static std::uint8_t                               receivedCount_{0};
        inline static bool                                       stop{false};
        inline static tp                                         timeoutTime_{};
        // Fault logging goes through this: a bad bus faults on every transaction.
        inline static Kvasir::RateLimiter<Clock> faultLog_{};

        // -- Public API -----------------------------------------------------------

        static void reset() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            active_ = false;
            state_  = State::idle;
            Recovery::resetState();
            drainQueueWithFailure();
            apply(Traits::I2C::getDisable<base::Instance>());
            apply(base::powerClockEnable);
            apply(base::initStepPeripheryConfig);
            apply(base::initStepInterruptConfig);
            apply(base::initStepPeripheryEnable);
        }

        static bool submit(Request const& req) {
            if(requestQueue_.size() >= requestQueue_.max_size()) { return false; }
            requestQueue_.push(req);

            apply(makeDisable(typename base::InterruptIndexs{}));
            if(!active_ && !Recovery::isActive()) { startNext(); }
            apply(makeEnable(typename base::InterruptIndexs{}));
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
                KVASIR_LOG_LIMITED(faultLog_.allow(faultKey(Fault::recovered), now),
                                   UC_LOG_W,
                                   "i2c{} recovery complete",
                                   base::Instance);
                return;
            }
            if(rv == Recovery::TickResult::busy) { return; }

            // Report the faults the log rate limiter dropped, now that the bus is quiet.
            apply(makeDisable(typename base::InterruptIndexs{}));
            auto const droppedFaults = faultLog_.takeSummary(now);
            apply(makeEnable(typename base::InterruptIndexs{}));
            if(droppedFaults != 0) {
                UC_LOG_W("i2c{} +{} faults not logged", base::Instance, droppedFaults);
            }

            // Post-abort settle: bus was sick, wait before starting next transaction
            if(!Recovery::isPastSettle(now)) { return; }

            // Nothing active: check SDA before starting the next transaction.
            // If SDA is held low the I2C peripheral cannot issue a START condition and the
            // transaction will time out immediately.  Detect this early and recover.
            if(!active_) {
                if(!requestQueue_.empty()) {
                    if(Recovery::checkSdaStuck(now)) { return; }
                    if(!Recovery::sdaIsHigh()) { return; }
                    apply(makeDisable(typename base::InterruptIndexs{}));
                    startNext();
                    apply(makeEnable(typename base::InterruptIndexs{}));
                }
                return;
            }

            // Active transaction: check for timeout
            if(now > timeoutTime_) {
                apply(makeDisable(typename base::InterruptIndexs{}));
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
            if(active_) {
                if(currentRequest_.callback) { currentRequest_.callback(I2CRequestResult::failed); }
                active_ = false;
            }
            drainQueueWithFailure();
            Recovery::begin();
            apply(makeEnable(typename base::InterruptIndexs{}));
        }

        static bool isRecovering() { return Recovery::isActive(); }

    private:
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

        static void drainQueueWithFailure() {
            while(!requestQueue_.empty()) {
                Request req{};
                requestQueue_.pop_into(req);
                if(req.callback) { req.callback(I2CRequestResult::failed); }
            }
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

            auto const totalBytes
              = currentRequest_.sendData.size() + currentRequest_.receiveData.size();
            timeoutTime_ = Clock::now() + base::calcTransferTimeout(totalBytes);

            // ENABLE.ABORT after a NAK raises its own TX_ABRT (ABRT_USER_ABRT) once the
            // abort has gone through, which is after the ISR that issued it has cleared the
            // source. Left in place it fires the moment this request unmasks TX_ABRT and
            // fails it with the previous request's cause.
            base::clearAbortSource();

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
                if(currentRequest_.receiveData.size() == 1) {
                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                      write(Regs::IC_DATA_CMD::STOPValC::enable),
                      write(Regs::IC_DATA_CMD::CMDValC::read)));
                } else {
                    apply(
                      Regs::IC_DATA_CMD::overrideDefaults(write(Regs::IC_DATA_CMD::CMDValC::read)));
                }
                apply(base::RxInterrupts);
            }
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

                // Master FSM is idle: a STOP has propagated so SDA should be released.
                // If it is still low a device is holding the bus — escalate to full recovery.
                if(!Recovery::sdaIsHigh()) {
                    KVASIR_LOG_LIMITED(
                      faultLog_.allow(faultKey(Fault::sdaStuck)),
                      UC_LOG_W,
                      "i2c{} SDA stuck low after abort -- escalating to bus recovery",
                      base::Instance);
                    drainQueueWithFailure();
                    active_ = false;
                    Recovery::begin();
                    return;
                }
            }
            startNext();
        }

    public:
        static void onIsr() {
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

                if(sendIndex_ < currentRequest_.sendData.size()) {
                    auto const byte = currentRequest_.sendData[sendIndex_++];
                    if(sendIndex_ == currentRequest_.sendData.size() && stop) {
                        Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                          write(Regs::IC_DATA_CMD::STOPValC::enable),
                          write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(byte)));
                    } else {
                        Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                          write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(byte)));
                    }
                } else {
                    if(stop) {
                        completeCurrentRequest(I2CRequestResult::succeeded);
                    } else {
                        state_ = State::receiving;
                        if(currentRequest_.receiveData.size() == 1) {
                            apply(Regs::IC_DATA_CMD::overrideDefaults(
                              write(Regs::IC_DATA_CMD::RESTARTValC::enable),
                              write(Regs::IC_DATA_CMD::STOPValC::enable),
                              write(Regs::IC_DATA_CMD::CMDValC::read)));
                        } else {
                            apply(Regs::IC_DATA_CMD::overrideDefaults(
                              write(Regs::IC_DATA_CMD::RESTARTValC::enable),
                              write(Regs::IC_DATA_CMD::CMDValC::read)));
                        }
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

                auto const data = apply(read(Regs::IC_DATA_CMD::dat));
                currentRequest_.receiveData[receivedCount_++]
                  = static_cast<std::byte>(Kvasir::Register::get<0>(data));
                auto const bytesLeft = currentRequest_.receiveData.size() - receivedCount_;
                if(bytesLeft > 1) {
                    apply(
                      Regs::IC_DATA_CMD::overrideDefaults(write(Regs::IC_DATA_CMD::CMDValC::read)));
                } else if(bytesLeft == 1) {
                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                      write(Regs::IC_DATA_CMD::STOPValC::enable),
                      write(Regs::IC_DATA_CMD::CMDValC::read)));
                } else {
                    completeCurrentRequest(I2CRequestResult::succeeded);
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
