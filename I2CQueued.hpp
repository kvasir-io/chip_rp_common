#pragma once
// Builds on kvasir_devices' bus tier: link kvasir::bus (or kvasir::devices).

#include "I2C.hpp"
#include "kvasir/Devices/I2C/Controller/LineRecovery.hpp"
#include "kvasir/Devices/I2C/Controller/QueuedFront.hpp"
#include "kvasir/Log/LogLimited.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/Register/Flags.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>
#include <uc_log/uc_log.hpp>

namespace Kvasir { namespace I2C {

    namespace Detail {
        /// The DW_apb_i2c's half of the queued master: kvasir_devices' QueueCore (the queue, cancel, deadlines,
        /// the timeout, the dead-bus watchdog, the recovery's clock, the counters) over `Hw`, the register work.
        /// I2CBehaviorQueued is this and the front (kvasir_devices I2C/Controller/QueuedFront.hpp says why the two are apart).
        template<typename I2CConfig,
                 typename Clock,
                 std::size_t QueueDepth,
                 std::size_t CallbackSize>
        struct I2CQueuedHalf
          : I2CBase<I2CConfig>
          , QueuedShape<typename I2CBase<I2CConfig>::I2CConfig,
                        Clock,
                        CallbackSize,
                        typename I2CBase<I2CConfig>::Config> {
            using base = I2CBase<I2CConfig>;
            using Shape
              = QueuedShape<typename base::I2CConfig, Clock, CallbackSize, typename base::Config>;
            using Regs     = typename base::Regs;
            using tp       = typename Clock::time_point;
            using Recovery = LineRecovery<base, Clock>;

            /// The block's registers when a transaction timed out, read before the abort (the core's
            /// TimeoutSnapshot holds them as `hw`, with the request's progress and the lines).
            struct Snapshot {
                std::uint32_t status{};         ///< IC_STATUS
                std::uint32_t rawIntr{};        ///< IC_RAW_INTR_STAT
                std::uint32_t intrMask{};       ///< IC_INTR_MASK
                std::uint32_t enableStatus{};   ///< IC_ENABLE_STATUS
                std::uint32_t abortSource{};    ///< IC_TX_ABRT_SOURCE, every bit
                std::uint32_t txLevel{};        ///< IC_TXFLR
                std::uint32_t rxLevel{};        ///< IC_RXFLR
                std::uint32_t tar{};            ///< IC_TAR: the address the block will actually use
                std::uint32_t enable{};         ///< IC_ENABLE, the ABORT bit included
            };

            /// The DW_apb_i2c's half of the queued master (kvasir_devices I2C/Controller/QueueHw.hpp).
            struct Hw {
                static constexpr auto instance       = base::instance;
                static constexpr bool cancellable    = Shape::Cancellable;
                static constexpr bool deadlines      = Shape::Deadlines;
                static constexpr bool perDeviceClock = Shape::PerDeviceClock;
                static constexpr auto busFeatures    = Shape::BusFeatures;
                using Recovery                       = I2CQueuedHalf::Recovery;
                using Request                        = typename Shape::Request;
                using Snapshot                       = I2CQueuedHalf::Snapshot;
                using ClockTiming                    = typename Shape::ClockTiming;

                // The bus interrupt line(s); the core counts the depth (Nvic::LineMask: ICER, dsb; isb - ISER).
                [[gnu::always_inline]] static void mask() {
                    Kvasir::Nvic::LineMask<typename base::InterruptIndexs>::mask();
                }

                [[gnu::always_inline]] static void unmask() {
                    Kvasir::Nvic::LineMask<typename base::InterruptIndexs>::unmask();
                }

                static void reinit() {
                    apply(set(Rp::Block<Regs>::reset));
                    apply(base::powerClockEnable);
                    // RESET_DONE before any register of the block is written: see waitResetDone().
                    Rp::Block<Regs>::waitResetDone();
                    apply(base::initStepPeripheryConfig);
                    // The block is back at baudRate's counts: the next request writes its own.
                    if constexpr(perDeviceClock) { timingCache_.invalidate(); }
                    apply(base::initStepInterruptConfig);
                }

                [[gnu::always_inline]] static auto transferTimeout(std::size_t    totalBytes,
                                                                   Request const& r) {
                    if constexpr(perDeviceClock) {
                        return base::calcTransferTimeout(totalBytes, r.timing.usPerByte);
                    } else {
                        static_cast<void>(r);
                        return base::calcTransferTimeout(totalBytes);
                    }
                }

                static void start(Request const& r) {
                    // ENABLE.ABORT after a NAK raises its own TX_ABRT (ABRT_USER_ABRT) once the
                    // abort has gone through, which is after the ISR that issued it has cleared the
                    // source. Left in place it fires the moment this request unmasks TX_ABRT and
                    // fails it with the previous request's cause.
                    base::clearAbortSource();

                    // IC_TAR is writable only while the block is disabled, and IC_ENABLE=0 takes
                    // effect only once the master is done: without this wait the address write
                    // below is dropped and the transfer goes out to the previous address.
                    waitDisabled_();

                    if constexpr(perDeviceClock) { applyTiming_(r.timing); }

                    apply(write(Regs::IC_TAR::ic_tar, r.address));
                    apply(Regs::IC_ENABLE::overrideDefaults(
                      write(Regs::IC_ENABLE::ENABLEValC::enabled)));

                    bool const hasSend = r.sendBytes() != 0;
                    bool const hasRecv = !r.receiveData.empty();

                    if(hasSend) {
                        stop_        = !hasRecv;
                        Core::state_ = QueueState::sending;
                        apply(base::txInterrupts);
                    } else {
                        stop_        = true;
                        Core::state_ = QueueState::receiving;
                        queueReads_(false);
                        apply(base::rxInterrupts);
                    }
                }

                [[gnu::always_inline]] static void finish() {
                    apply(Regs::IC_ENABLE::overrideDefaults(
                      write(Regs::IC_ENABLE::ENABLEValC::disabled)));
                    apply(base::noInterrupts);
                }

                /// ENABLE.ABORT issues a STOP after the current byte; TX_ABRT completes the request.
                [[gnu::always_inline]] static void requestStop() {
                    apply(base::cancelAbortRequest);
                }

                [[gnu::always_inline]] static void timeoutAbort() { apply(base::softAbortRequest); }

                [[gnu::always_inline]] static bool busActive() {
                    return fieldEquals(Regs::IC_STATUS::MST_ACTIVITYValC::active);
                }

                [[gnu::always_inline]] static void logTimeout([[maybe_unused]] tp now) {
                    KVASIR_LOG_LIMITED(
                      Core::faultLog_.note(Core::Fault::timeout,
                                           now,
                                           Core::currentRequest_.address,
                                           0),
                      UC_LOG_W,
                      "i2c{} timeout addr={:#04x} {}",
                      base::instance,
                      Core::currentRequest_.address,
                      Kvasir::Register::Flags<typename base::AbrtSrc>{base::abortCause()});
                }

                static Snapshot snapshot() {
                    return Snapshot{
                      .status       = get<0>(apply(read(Regs::IC_STATUS::FULLREGISTER))),
                      .rawIntr      = get<0>(apply(read(Regs::IC_RAW_INTR_STAT::FULLREGISTER))),
                      .intrMask     = get<0>(apply(read(Regs::IC_INTR_MASK::FULLREGISTER))),
                      .enableStatus = get<0>(apply(read(Regs::IC_ENABLE_STATUS::FULLREGISTER))),
                      .abortSource  = get<0>(apply(read(Regs::IC_TX_ABRT_SOURCE::FULLREGISTER))),
                      .txLevel      = get<0>(apply(read(Regs::IC_TXFLR::FULLREGISTER))),
                      .rxLevel      = get<0>(apply(read(Regs::IC_RXFLR::FULLREGISTER))),
                      .tar          = get<0>(apply(read(Regs::IC_TAR::FULLREGISTER))),
                      .enable       = get<0>(apply(read(Regs::IC_ENABLE::FULLREGISTER))),
                    };
                }

                /// lastTimeout() as a log line. `hw` is this block's registers, so the line is the
                /// driver's: the SAM SERCOM driver has the same call and says what its block looked
                /// like (state 1 is sending, 2 receiving; sent/received are bytes done of bytes asked).
                static void logLastTimeout([[maybe_unused]] TimeoutSnapshot<Snapshot> const& t) {
                    UC_LOG_W(
                      "i2c{} last timeout: addr {:#04x}, state {}, sent {}/{}, received {}/{}, "
                      "IC_STATUS {:#06x}, IC_RAW_INTR_STAT {:#06x}, IC_INTR_MASK {:#06x}, "
                      "IC_ENABLE_STATUS {:#x}, IC_TX_ABRT_SOURCE {:#010x}, TX FIFO {}, RX FIFO {}, "
                      "SDA {}, SCL {}, IC_TAR {:#x}, IC_ENABLE {:#x}, {} interrupt(s) in the "
                      "request, last {} us before, request {} us old",
                      base::instance,
                      t.address,
                      t.state,
                      t.sent,
                      t.toSend,
                      t.received,
                      t.toReceive,
                      t.hw.status,
                      t.hw.rawIntr,
                      t.hw.intrMask,
                      t.hw.enableStatus,
                      t.hw.abortSource,
                      t.hw.txLevel,
                      t.hw.rxLevel,
                      std::string_view{t.sdaHigh ? "high" : "LOW"},
                      std::string_view{t.sclHigh ? "high" : "LOW"},
                      t.hw.tar,
                      t.hw.enable,
                      t.isrEntries,
                      t.usSinceIsr,
                      t.usAge);
                }

                // Every entry is checked for the event its state waits on before anything is done with
                // it. A request completed here starts the next one here (Core::complete ->
                // startNext), so an entry the NVIC latched for the old request can arrive once the
                // new one is already sending or receiving: taken at face value, a receive would store
                // whatever IC_DATA_CMD holds with the FIFO empty and queue one read command too many,
                // and a send would move on before its byte was out. Such an entry is counted and dropped.
                static void onIsr() {
                    auto&      req   = Core::currentRequest_;
                    bool const error = fieldEquals(Regs::IC_INTR_STAT::R_TX_ABRTValC::active);

                    // our own ABORT (cancel, deadline): ABRT_USER_ABRT alone, not a fault; a NAK or a real fault that
                    // won the race keeps its own result (the request was over first)
                    if constexpr(Shape::Tracked) {
                        if(error && Core::cancelPending_ && Core::state_ != QueueState::idle
                           && (base::abortCause() & base::masterAbortCauses) == 0)
                        {
                            base::clearAbortSource();
                            apply(base::abort);
                            Core::complete(Core::stopAs_);
                            return;
                        }
                    }

                    if(Core::state_ == QueueState::sending) {
                        if(error) {
                            auto const cause = base::abortCause();
                            bool const isNak
                              = (cause & base::AbrtSrc::abrt_7b_addr_noack.mask) != 0;
                            // An address NAK alone is an answer - nobody at that address, every probe
                            // of a scan - not a bus fault: debug. A known part that stops answering is
                            // Presence's warning ("not responding"); a data NAK stays a warning.
                            if(cause == base::AbrtSrc::abrt_7b_addr_noack.mask) {
                                KVASIR_LOG_LIMITED(Core::nakLog_.allow(
                                                     Core::faultKey(Core::Fault::abortSend, cause)),
                                                   UC_LOG_D,
                                                   "i2c{} send addr={:#04x}: no ACK, nobody there",
                                                   base::instance,
                                                   req.address);
                            } else {
                                KVASIR_LOG_LIMITED(
                                  Core::faultLog_.note(
                                    Core::Fault::abortSend,
                                    req.address,
                                    static_cast<typename base::AbrtSrc::Addr::RegType>(cause)),
                                  UC_LOG_W,
                                  "i2c{} abort send addr={:#04x} {}",
                                  base::instance,
                                  req.address,
                                  Kvasir::Register::Flags<typename base::AbrtSrc>{cause});
                            }
                            base::clearAbortSource();
                            apply(base::abort);
                            Core::complete(isNak ? Core::Result::nack : Core::Result::failed);
                            return;
                        }

                        if(!fieldEquals(Regs::IC_RAW_INTR_STAT::TX_EMPTYValC::active)) {
                            ++Core::spuriousIsr_;
                            return;
                        }

                        // The prefix and sendData go into the FIFO in one pass: without STOP on a byte
                        // the master goes on with the next FIFO entry, or holds SCL low until there is
                        // one (RP2350 data sheet 12.2.7.1, IC_DATA_CMD.STOP in Table 1059).
                        auto const total = req.sendBytes();
                        if(Core::sendIndex_ < total) {
                            // As many bytes as the FIFO has room for: TX_EMPTY comes back once they
                            // are all out, where it used to come back for each one.
                            auto room = fifoDepth - get<0>(apply(read(Regs::IC_TXFLR::txflr)));
                            while(room != 0 && Core::sendIndex_ < total) {
                                auto const byte = req.sendByte(Core::sendIndex_++);
                                if(Core::sendIndex_ == total && stop_) {
                                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                                      write(Regs::IC_DATA_CMD::STOPValC::enable),
                                      write(Regs::IC_DATA_CMD::dat,
                                            static_cast<std::uint8_t>(byte))));
                                } else {
                                    apply(Regs::IC_DATA_CMD::overrideDefaults(
                                      write(Regs::IC_DATA_CMD::dat,
                                            static_cast<std::uint8_t>(byte))));
                                }
                                --room;
                            }
                        } else {
                            if(stop_) {
                                Core::complete(Core::Result::ok);
                            } else {
                                Core::state_ = QueueState::receiving;
                                queueReads_(true);
                                apply(base::rxInterrupts);
                            }
                        }
                    } else if(Core::state_ == QueueState::receiving) {
                        if(error) {
                            auto const cause = base::abortCause();
                            bool const isNak
                              = (cause & base::AbrtSrc::abrt_7b_addr_noack.mask) != 0;
                            // an address NAK alone is an answer, not a fault (see the send side)
                            if(cause == base::AbrtSrc::abrt_7b_addr_noack.mask) {
                                KVASIR_LOG_LIMITED(Core::nakLog_.allow(
                                                     Core::faultKey(Core::Fault::abortRecv, cause)),
                                                   UC_LOG_D,
                                                   "i2c{} recv addr={:#04x}: no ACK, nobody there",
                                                   base::instance,
                                                   req.address);
                            } else {
                                KVASIR_LOG_LIMITED(
                                  Core::faultLog_.note(
                                    Core::Fault::abortRecv,
                                    req.address,
                                    static_cast<typename base::AbrtSrc::Addr::RegType>(cause)),
                                  UC_LOG_W,
                                  "i2c{} abort recv addr={:#04x} {}",
                                  base::instance,
                                  req.address,
                                  Kvasir::Register::Flags<typename base::AbrtSrc>{cause});
                            }
                            base::clearAbortSource();
                            apply(base::abort);
                            Core::complete(isNak ? Core::Result::nack : Core::Result::failed);
                            return;
                        }

                        if(get<0>(apply(read(Regs::IC_RXFLR::rxflr))) == 0) {
                            ++Core::spuriousIsr_;
                            return;
                        }

                        // Everything that has come in, never more than was asked for: a byte past the
                        // read commands queued is not this request's.
                        auto available = get<0>(apply(read(Regs::IC_RXFLR::rxflr)));
                        while(available != 0 && Core::receivedCount_ < queuedReads_) {
                            auto const data = apply(read(Regs::IC_DATA_CMD::dat));
                            req.receiveData[Core::receivedCount_++]
                              = static_cast<std::byte>(Kvasir::Register::get<0>(data));
                            --available;
                        }
                        if(Core::receivedCount_ == req.receiveData.size()) {
                            Core::complete(Core::Result::ok);
                        } else if(Core::receivedCount_ == queuedReads_) {
                            queueReads_(false);
                        } else {
                            // Fewer than the threshold asked for (an entry that came early): wait for
                            // the rest of this batch.
                            apply(write(
                              Regs::IC_RX_TL::rx_tl,
                              static_cast<std::uint32_t>(queuedReads_ - Core::receivedCount_ - 1)));
                        }
                    } else {
                        // Idle: a stale entry (see Core::complete). Whatever it carried is
                        // over; clear the abort source so the next request reports its own.
                        base::clearAbortSource();
                    }
                }

                // -- the register work's own state and helpers ------------------------------------------

                /// The DW_apb_i2c FIFOs hold 16 entries each way (IC_TX_BUFFER_DEPTH, IC_RX_BUFFER_DEPTH).
                static constexpr std::size_t fifoDepth = 16;

                inline static bool stop_{false};   ///< a STOP on the last byte sent (no read phase)
                inline static std::size_t
                  queuedReads_{};   ///< receivedCount_ once the queued reads are in
                inline static std::uint32_t waitsExhausted_{};

                // perDeviceClock: the counts in the block, and whether they are known (not after a
                // reset). A member of a class template: never instantiated on a bus without it.
                inline static TimingCache<ClockTiming> timingCache_{};

                /// Times waitDisabled_() ran out of spins with the block still enabled, so the next
                /// IC_TAR write may have been dropped (the bus's disableWaitsExhausted()).
                static std::uint32_t waitsExhausted() { return waitsExhausted_; }

                /// Times a start rewrote the SCL counts for a device at another clock (perDeviceClock).
                static std::uint32_t clockSwitches() { return timingCache_.switches(); }

                /// The SCL counts of the request about to start, written only when they differ from
                /// the last ones. HCNT, LCNT and SPKLEN take a write only while the block is disabled
                /// ("Writes at other times have no effect", RP2350 datasheet 12.2, Tables 1062, 1063,
                /// 1093; RP2040 4.3): called after waitDisabled_(). IC_SDA_HOLD has no such rule
                /// (Table 1084). IC_CON.SPEED stays fast for every rate (Config::getSpeedModeRegister).
                /// The pads keep the drive chosen for baudRate (Config::i2cDrive), which is at least
                /// as strong as a slower device's. Interrupts disabled or in the ISR.
                static void applyTiming_(ClockTiming const& t) {
                    if(!timingCache_.needs(t)) { return; }
                    apply(write(Regs::IC_FS_SCL_HCNT::ic_fs_scl_hcnt, std::uint32_t{t.hcnt}),
                          write(Regs::IC_FS_SCL_LCNT::ic_fs_scl_lcnt, std::uint32_t{t.lcnt}),
                          write(Regs::IC_FS_SPKLEN::ic_fs_spklen, std::uint32_t{t.spklen}));
                    apply(write(Regs::IC_SDA_HOLD::ic_sda_tx_hold, std::uint32_t{t.sdaHold}));
                    timingCache_.took(t);
                }

                /// Spin until IC_ENABLE_STATUS says the block is inactive. Bounded: this runs in
                /// the ISR, and a wedged block is left to the dead-bus watchdog.
                static void waitDisabled_() {
                    for(std::uint32_t spins = 0; spins < 100'000U; ++spins) {
                        if(!fieldEquals(Regs::IC_ENABLE_STATUS::IC_ENValC::enabled)) { return; }
                    }
                    ++waitsExhausted_;
                }

                /// Queues the request's next read commands -- as many as are left, up to a FIFO's worth
                /// -- the first with RESTART after a send, the request's last with STOP, and sets
                /// IC_RX_TL so RX_FULL fires once all of them have come in: one interrupt for up to 16
                /// bytes where there was one a byte. Interrupts disabled or in the ISR.
                static void queueReads_(bool restart) {
                    auto const left
                      = Core::currentRequest_.receiveData.size() - Core::receivedCount_;
                    auto const n = std::min<std::size_t>(left, fifoDepth);
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
                            apply(Regs::IC_DATA_CMD::overrideDefaults(
                              write(Regs::IC_DATA_CMD::CMDValC::read)));
                        }
                    }
                    queuedReads_ = Core::receivedCount_ + n;
                    apply(write(Regs::IC_RX_TL::rx_tl, static_cast<std::uint32_t>(n - 1)));
                }
            };

            using Core = QueueCore<Hw, Clock, QueueDepth, CallbackSize>;
        };
    }   // namespace Detail

    /// The queued I2C master on the DW_apb_i2c: its hardware half and the front (kvasir_devices
    /// I2C/Controller/QueuedFront.hpp), whose members are one line each into the core.
    template<typename I2CConfig, typename Clock, std::size_t QueueDepth_, std::size_t CallbackSize_>
    struct I2CBehaviorQueued
      : Detail::I2CQueuedHalf<I2CConfig, Clock, QueueDepth_, CallbackSize_>
      , QueuedFront<
          typename Detail::I2CQueuedHalf<I2CConfig, Clock, QueueDepth_, CallbackSize_>::Core> {
        using Isr = Nvic::IsrsFor<std::addressof(I2CBehaviorQueued::onIsr),
                                  typename I2CBehaviorQueued::InterruptIndexs>;
    };
}}   // namespace Kvasir::I2C
