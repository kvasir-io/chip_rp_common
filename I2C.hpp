#pragma once

#include "Io.hpp"
#include "PinConfig.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/I2C.hpp"

#include <array>
#include <cassert>
#include <span>

namespace Kvasir { namespace I2C {

    namespace Detail {

        template<unsigned Instance>
        struct Config {
            using Regs = Kvasir::Peripheral::I2C::Registers<Instance>;

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationSDA(Kvasir::Register::PinLocation<Port,
                                                                                      Pin>) {
                return Instance == 0 ? PinConfig::isValidI2cPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::I2cPinType::Sda0)
                                     : PinConfig::isValidI2cPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::I2cPinType::Sda1);
            }

            template<int Port,
                     int Pin>
            static constexpr bool isValidPinLocationSCL(Kvasir::Register::PinLocation<Port,
                                                                                      Pin>) {
                return Instance == 0 ? PinConfig::isValidI2cPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::I2cPinType::Scl0)
                                     : PinConfig::isValidI2cPin<PinConfig::CurrentChip, Instance>(
                                         Pin,
                                         PinConfig::I2cPinType::Scl1);
            }

            template<typename SDAPIN>
            struct GetSDAPinConfig;

            template<int Port, int Pin>
            struct GetSDAPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(Kvasir::Io::Action::PinFunction<3>{},
                                                  Register::PinLocation<Port, Pin>{}));
            };
            template<typename SCLPIN>
            struct GetSCLPinConfig;

            template<int Port, int Pin>
            struct GetSCLPinConfig<Kvasir::Register::PinLocation<Port, Pin>> {
                using pinConfig = decltype(action(Kvasir::Io::Action::PinFunction<3>{},
                                                  Register::PinLocation<Port, Pin>{}));
            };

            // I2C baud rate calculation helpers
            static constexpr std::uint32_t calcPeriod(std::uint32_t f_clockSpeed,
                                                      std::uint32_t f_baud) {
                return (f_clockSpeed + f_baud / 2) / f_baud;
            }

            static constexpr std::uint32_t calcSpkLen(std::uint32_t lcnt) {
                return lcnt < 16 ? 1 : lcnt / 16;
            }

            static constexpr std::uint32_t calcSdaTxHold(std::uint32_t f_clockSpeed,
                                                         std::uint32_t f_baud) {
                // Per I2C spec: 300ns hold time for <1MHz, 120ns for >=1MHz
                if(f_baud < 1000000) {
                    // 300ns: freq * 3 / 10000000 + 1
                    return ((f_clockSpeed * 3) / 10000000) + 1;
                } else {
                    // 120ns: freq * 3 / 25000000 + 1
                    return ((f_clockSpeed * 3) / 25000000) + 1;
                }
            }

            struct BaudRegs {
                std::uint32_t hcnt;
                std::uint32_t lcnt;
                std::uint32_t spklen;
                std::uint32_t sda_hold;
            };

            static constexpr BaudRegs calcBaudRegs(std::uint32_t f_clockSpeed,
                                                   std::uint32_t f_baud) {
                BaudRegs regs{};

                // Calculate period in ic_clk cycles
                std::uint32_t period = calcPeriod(f_clockSpeed, f_baud);

                // Split period: 60% low, 40% high (per pico-sdk)
                regs.lcnt = period * 3 / 5;
                regs.hcnt = period - regs.lcnt;

                // Calculate spike length
                regs.spklen = calcSpkLen(regs.lcnt);

                // Calculate SDA hold time
                regs.sda_hold = calcSdaTxHold(f_clockSpeed, f_baud);

                return regs;
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud,
                     std::intmax_t Num,
                     std::intmax_t Denom>
            static constexpr bool isValidBaudConfig(std::ratio<Num,
                                                               Denom>) {
                constexpr auto regs = calcBaudRegs(f_clockSpeed, f_baud);

                // Check datasheet minimums
                // LCNT must be > SPKLEN + 7
                // HCNT must be > SPKLEN + 5
                if(regs.lcnt <= regs.spklen + 7) { return false; }
                if(regs.hcnt <= regs.spklen + 5) { return false; }

                // Verify actual baud rate is within tolerance
                constexpr auto period       = regs.hcnt + regs.lcnt;
                constexpr auto f_baudActual = f_clockSpeed / period;
                constexpr auto err
                  = f_baudActual > f_baud ? f_baudActual - f_baud : f_baud - f_baudActual;
                constexpr auto maxErr = (f_baud * Num) / Denom;

                return err <= maxErr;
            }

            template<std::uint32_t f_baud>
            static constexpr auto getSpeedModeRegister() {
                if constexpr(f_baud <= 100'000) {
                    //return write(Regs::IC_CON::SPEEDValC::standard);
                    // standard mode is buggy always use fast
                    return write(Regs::IC_CON::SPEEDValC::fast);
                } else if constexpr(f_baud <= 1'000'000) {
                    return write(Regs::IC_CON::SPEEDValC::fast);
                } else {
                    return write(Regs::IC_CON::SPEEDValC::high);
                }
            }

            template<std::uint32_t f_clockSpeed, std::uint32_t f_baud>
            struct GetBaudConfig {
                static constexpr auto config_ = []() {
                    constexpr auto regs = calcBaudRegs(f_clockSpeed, f_baud);

                    return list(
                      write(Regs::IC_FS_SCL_HCNT::ic_fs_scl_hcnt, Register::value<regs.hcnt>()),
                      write(Regs::IC_FS_SCL_LCNT::ic_fs_scl_lcnt, Register::value<regs.lcnt>()),
                      write(Regs::IC_FS_SPKLEN::ic_fs_spklen, Register::value<regs.spklen>()),
                      Regs::IC_SDA_HOLD::overrideDefaults(write(Regs::IC_SDA_HOLD::ic_sda_tx_hold,
                                                                Register::value<regs.sda_hold>())));
                }();
                using config = decltype(config_);
            };
        };
    }   // namespace Detail

    namespace Traits { namespace I2C {
        template<unsigned Instance>
        static constexpr auto getIsrIndexs() {
            static_assert(Instance < 2, "I2C Instance must be 0 or 1");
            if constexpr(Instance == 0) {
                return brigand::list<decltype(Kvasir::Interrupt::i2c0)>{};
            } else {
                return brigand::list<decltype(Kvasir::Interrupt::i2c1)>{};
            }
        }

        template<unsigned Instance>
        static constexpr auto getEnable() {
            static_assert(Instance < 2, "I2C Instance must be 0 or 1");
            if constexpr(Instance == 0) {
                return clear(Peripheral::RESETS::Registers<>::RESET::i2c0);
            } else {
                return clear(Peripheral::RESETS::Registers<>::RESET::i2c1);
            }
        }

        template<unsigned Instance>
        static constexpr auto getDisable() {
            static_assert(Instance < 2, "I2C Instance must be 0 or 1");
            if constexpr(Instance == 0) {
                return set(Peripheral::RESETS::Registers<>::RESET::i2c0);
            } else {
                return set(Peripheral::RESETS::Registers<>::RESET::i2c1);
            }
        }
    }}   // namespace Traits::I2C

    namespace Detail {

        template<typename I2CConfig_>
        struct I2CBase {
            struct I2CConfig : I2CConfig_ {
                static constexpr auto userConfigOverride = [] {
                    if constexpr(requires { I2CConfig_::userConfigOverride; }) {
                        return I2CConfig_::userConfigOverride;
                    } else {
                        return brigand::list<>{};
                    }
                }();

                static constexpr auto maxBaudRateError = [] {
                    if constexpr(requires { I2CConfig_::maxBaudRateError; }) {
                        return I2CConfig_::maxBaudRateError;
                    } else {
                        return std::ratio<1, 100>{};
                    }
                }();
            };

            // needed config
            // clockSpeed
            // baudRate
            // maxBaudRateError
            // i2cInstance
            // SdaPinLocation
            // SclPinLocation
            // userConfigOverride
            static constexpr auto Instance = I2CConfig::instance;
            using Regs                     = Kvasir::Peripheral::I2C::Registers<Instance>;
            using Config                   = Detail::Config<Instance>;

            using InterruptIndexs = decltype(Traits::I2C::getIsrIndexs<Instance>());

            static constexpr auto NoInterrupts
              = list(Regs::IC_INTR_MASK::overrideDefaults(clear(Regs::IC_INTR_MASK::m_gen_call),
                                                          clear(Regs::IC_INTR_MASK::m_rx_done),
                                                          clear(Regs::IC_INTR_MASK::m_tx_abrt),
                                                          clear(Regs::IC_INTR_MASK::m_rd_req),
                                                          clear(Regs::IC_INTR_MASK::m_tx_empty),
                                                          clear(Regs::IC_INTR_MASK::m_tx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_full),
                                                          clear(Regs::IC_INTR_MASK::m_rx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_under)));

            static constexpr auto TxInterrupts
              = list(Regs::IC_INTR_MASK::overrideDefaults(clear(Regs::IC_INTR_MASK::m_gen_call),
                                                          clear(Regs::IC_INTR_MASK::m_rx_done),
                                                          set(Regs::IC_INTR_MASK::m_tx_abrt),
                                                          clear(Regs::IC_INTR_MASK::m_rd_req),
                                                          set(Regs::IC_INTR_MASK::m_tx_empty),
                                                          clear(Regs::IC_INTR_MASK::m_tx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_full),
                                                          clear(Regs::IC_INTR_MASK::m_rx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_under)));

            static constexpr auto RxInterrupts
              = list(Regs::IC_INTR_MASK::overrideDefaults(clear(Regs::IC_INTR_MASK::m_gen_call),
                                                          clear(Regs::IC_INTR_MASK::m_rx_done),
                                                          set(Regs::IC_INTR_MASK::m_tx_abrt),
                                                          clear(Regs::IC_INTR_MASK::m_rd_req),
                                                          clear(Regs::IC_INTR_MASK::m_tx_empty),
                                                          clear(Regs::IC_INTR_MASK::m_tx_over),
                                                          set(Regs::IC_INTR_MASK::m_rx_full),
                                                          clear(Regs::IC_INTR_MASK::m_rx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_under)));

            static_assert(
              Config::template isValidBaudConfig<I2CConfig::clockSpeed,
                                                 I2CConfig::baudRate>(I2CConfig::maxBaudRateError),
              "invalid baud configuration baudRate error too big");
            static_assert(Config::isValidPinLocationSDA(I2CConfig::sdaPinLocation),
                          "invalid SDAPin");
            static_assert(Config::isValidPinLocationSCL(I2CConfig::sclPinLocation),
                          "invalid SCLPin");

            static constexpr auto powerClockEnable = list(Traits::I2C::getEnable<Instance>());

            static constexpr auto initStepPinConfig
              = list(typename Config::template GetSDAPinConfig<
                       std::decay_t<decltype(I2CConfig::sdaPinLocation)>>::pinConfig{},
                     typename Config::template GetSCLPinConfig<
                       std::decay_t<decltype(I2CConfig::sclPinLocation)>>::pinConfig{});

            static constexpr auto initStepPeripheryConfig
              = list(Regs::IC_CON::overrideDefaults(
                       write(Regs::IC_CON::MASTER_MODEValC::enabled),
                       Config::template getSpeedModeRegister<I2CConfig::baudRate>()),
                     typename Config::template GetBaudConfig<I2CConfig::clockSpeed,
                                                             I2CConfig::baudRate>::config{},
                     NoInterrupts);

            static constexpr auto initStepInterruptConfig
              = list(Nvic::makeSetPriority<I2CConfig::isrPriority>(InterruptIndexs{}),
                     Nvic::makeClearPending(InterruptIndexs{}));

            static constexpr auto initStepPeripheryEnable
              = list(Nvic::makeEnable(InterruptIndexs{}));

            static constexpr auto calcTransferTimeout(std::size_t numBytes) {
                using namespace std::chrono_literals;
                constexpr std::uint32_t bitsPerDataByte = 9;
                constexpr std::uint32_t safetyFactor    = 4;
                constexpr auto          baseTimeout     = 10ms;
                constexpr std::uint32_t microsecondsPerDataByte
                  = (bitsPerDataByte * 1'000'000 * safetyFactor) / I2CConfig::baudRate;

                std::uint32_t const timeoutUs = (numBytes + 1) * microsecondsPerDataByte;

                return std::chrono::microseconds(timeoutUs) + baseTimeout;
            }

            static constexpr auto abort
              = list(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::disabled),
                                                       set(Regs::IC_ENABLE::abort)),
                     NoInterrupts);
        };
    }   // namespace Detail

    template<typename I2CConfig, typename Clock, std::size_t BufferSize_>
    struct I2CBehavior : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t BufferSize = BufferSize_;
        using base                              = Detail::I2CBase<I2CConfig>;
        using Regs                              = typename base::Regs;
        using tp                                = typename Clock::time_point;

        enum class State { idle, blocked, sending, receiving };
        enum class OperationState { succeeded, failed, ongoing };
        inline static std::atomic<State>          state_{State::idle};
        inline static std::atomic<std::uint8_t>   receiveSize_{0};
        inline static std::atomic<OperationState> operationState_{OperationState::succeeded};
        inline static Kvasir::Atomic::Queue<std::byte, BufferSize> buffer_{};
        inline static bool                                         stop{false};
        inline static tp                                           timeoutTime{};

        static void reset() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            state_.store(State::idle, std::memory_order_relaxed);
            operationState_.store(OperationState::succeeded, std::memory_order_relaxed);
            apply(Traits::I2C::getDisable<base::Instance>());
            apply(base::powerClockEnable);
            apply(base::initStepPeripheryConfig);
            apply(base::initStepInterruptConfig);
            apply(base::initStepPeripheryEnable);
        }

        template<typename C>
        static void getReceivedBytes(C& c) {
            assert(c.size() <= buffer_.size());
            buffer_.pop_into(c);
        }

        template<typename OIT>
        static void getReceivedBytes(OIT first,
                                     OIT last) {
            while(first != last) {
                assert(!buffer_.empty());
                *first = buffer_.front();
                buffer_.pop();
                ++first;
            }
        }

        static OperationState operationState(tp const& currentTime) {
            auto op = operationState_.load(std::memory_order_relaxed);
            if(op == OperationState::ongoing) {
                if(currentTime > timeoutTime) {
                    apply(base::abort);
                    buffer_.clear();
                    operationState_.store(OperationState::failed, std::memory_order_relaxed);
                    state_.store(State::blocked, std::memory_order_relaxed);
                    return OperationState::failed;
                }
            }
            return op;
        }

        static bool acquire() {
            if(state_.load(std::memory_order_relaxed) == State::idle) {
                state_.store(State::blocked, std::memory_order_relaxed);
                return true;
            }
            return false;
        }

        static void release() {
            assert(state_.load(std::memory_order_relaxed) != State::idle);
            state_.store(State::idle, std::memory_order_relaxed);
        }

        template<typename C>
        static void send(tp const&    currentTime,
                         std::uint8_t address,
                         C const&     c) {
            assert(state_.load(std::memory_order_relaxed) != State::sending);
            assert(state_.load(std::memory_order_relaxed) != State::receiving);
            assert(!c.empty());
            assert(c.size() <= buffer_.max_size());

            buffer_.clear();
            buffer_.push(c);
            state_.store(State::sending, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = true;
            timeoutTime = currentTime + base::calcTransferTimeout(c.size());
            apply(write(Regs::IC_TAR::ic_tar, address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));
            apply(base::TxInterrupts);
        }

        static void receive(tp const&    currentTime,
                            std::uint8_t address,
                            std::uint8_t size) {
            assert(state_.load(std::memory_order_relaxed) != State::sending);
            assert(state_.load(std::memory_order_relaxed) != State::receiving);
            assert(size <= buffer_.max_size());
            assert(size != 0);

            buffer_.clear();
            receiveSize_.store(size, std::memory_order_relaxed);
            state_.store(State::receiving, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = true;
            timeoutTime = currentTime + base::calcTransferTimeout(size);

            apply(write(Regs::IC_TAR::ic_tar, address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));

            if(size == 1) {
                apply(
                  Regs::IC_DATA_CMD::overrideDefaults(write(Regs::IC_DATA_CMD::STOPValC::enable),
                                                      write(Regs::IC_DATA_CMD::CMDValC::read)));
            } else {
                apply(Regs::IC_DATA_CMD::overrideDefaults(write(Regs::IC_DATA_CMD::CMDValC::read)));
            }

            apply(base::RxInterrupts);
        }

        template<typename C>
        static void send_receive(tp const&    currentTime,
                                 std::uint8_t address,
                                 C const&     c,
                                 std::uint8_t size) {
            assert(state_.load(std::memory_order_relaxed) != State::sending);
            assert(state_.load(std::memory_order_relaxed) != State::receiving);
            assert(size <= buffer_.max_size());
            assert(size != 0);

            buffer_.clear();
            std::size_t sendSize;
            if constexpr(std::is_same_v<std::decay_t<C>, std::byte>) {
                sendSize = 1;
                buffer_.push(c);
            } else {
                assert(c.size() <= buffer_.max_size());
                assert(!c.empty());
                sendSize = c.size();
                buffer_.push(c);
            }
            receiveSize_.store(size, std::memory_order_relaxed);
            state_.store(State::sending, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = false;
            timeoutTime = currentTime + base::calcTransferTimeout(sendSize + size);

            apply(write(Regs::IC_TAR::ic_tar, address));
            apply(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled)));
            apply(base::TxInterrupts);
        }

        // ISR
        static void onIsr() {
            bool const error = get<0>(apply(read(Regs::IC_INTR_STAT::r_tx_abrt)))
                            == Regs::IC_INTR_STAT::R_TX_ABRTVal::active;

            auto lstate  = state_.load(std::memory_order_relaxed);
            auto lostate = operationState_.load(std::memory_order_relaxed);
            if(lstate == State::sending) {
                if(error) {
                    lstate  = State::blocked;
                    lostate = OperationState::failed;
                    UC_LOG_C("abort send");
                    apply(base::abort);
                } else {
                    if(!buffer_.empty()) {
                        auto const data = buffer_.front();
                        buffer_.pop();
                        if(buffer_.empty() && stop) {
                            Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                              write(Regs::IC_DATA_CMD::STOPValC::enable),
                              write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(data)));
                        } else {
                            Regs::IC_DATA_CMD::overrideDefaultsRuntime(
                              write(Regs::IC_DATA_CMD::dat, static_cast<std::uint8_t>(data)));
                        }
                    } else {
                        if(stop) {
                            lstate  = State::blocked;
                            lostate = OperationState::succeeded;
                            apply(Regs::IC_ENABLE::overrideDefaults(
                              write(Regs::IC_ENABLE::ENABLEValC::disabled)));
                            apply(base::NoInterrupts);
                        } else {
                            auto const lreceiveSize = receiveSize_.load(std::memory_order_relaxed);
                            lstate                  = State::receiving;
                            if(lreceiveSize == 1) {
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
                }
            } else if(lstate == State::receiving) {
                if(error) {
                    UC_LOG_C("abort recv");
                    lstate  = State::blocked;
                    lostate = OperationState::failed;
                    apply(base::abort);
                } else {
                    auto const lreceiveSize = receiveSize_.load(std::memory_order_relaxed);
                    auto const data         = apply(read(Regs::IC_DATA_CMD::dat));
                    buffer_.push(static_cast<std::byte>(Kvasir::Register::get<0>(data)));
                    auto const bytesLeft = lreceiveSize - buffer_.size();
                    if(bytesLeft > 1) {
                        apply(Regs::IC_DATA_CMD::overrideDefaults(
                          write(Regs::IC_DATA_CMD::CMDValC::read)));
                    } else if(bytesLeft == 1) {
                        apply(Regs::IC_DATA_CMD::overrideDefaults(
                          write(Regs::IC_DATA_CMD::STOPValC::enable),
                          write(Regs::IC_DATA_CMD::CMDValC::read)));
                    } else {
                        lstate  = State::blocked;
                        lostate = OperationState::succeeded;
                        apply(Regs::IC_ENABLE::overrideDefaults(
                          write(Regs::IC_ENABLE::ENABLEValC::disabled)));
                        apply(base::NoInterrupts);
                    }
                }
            } else {
                UC_LOG_C("bad bad");
            }

            operationState_.store(lostate, std::memory_order_relaxed);
            state_.store(lstate, std::memory_order_relaxed);
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };
}}   // namespace Kvasir::I2C
