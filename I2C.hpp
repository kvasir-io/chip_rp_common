#pragma once

#include "Clocks.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/Register/RegisterFmt.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/I2C.hpp"

namespace Kvasir { namespace I2C {

    // Startup resource: the I2C block itself (kvasir/StartUp/Resources.hpp).
    struct InstanceTag {};

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

            // I2C is open-drain: the pad only ever drives low. Keep the driver weak
            // and slew slow so falling edges stay within the I2C spec t_f window
            // (>= ~12 ns for fast mode) — a 12 mA pad produces ~1.5 ns edges with
            // heavy ringing. Standard mode (<= 100 kHz) gets 2 mA, faster modes 4 mA.
            static constexpr Io::DriveStrength i2cDrive(std::uint32_t f_baud) {
                return f_baud <= 100'000 ? Io::DriveStrength::mA_2 : Io::DriveStrength::mA_4;
            }

            template<typename SDAPIN, std::uint32_t f_baud>
            struct GetSDAPinConfig;

            template<int Port, int Pin, std::uint32_t f_baud>
            struct GetSDAPinConfig<Kvasir::Register::PinLocation<Port, Pin>, f_baud> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunctionDrive<3, i2cDrive(f_baud), false>{},
                  Register::PinLocation<Port, Pin>{}));
            };
            template<typename SCLPIN, std::uint32_t f_baud>
            struct GetSCLPinConfig;

            template<int Port, int Pin, std::uint32_t f_baud>
            struct GetSCLPinConfig<Kvasir::Register::PinLocation<Port, Pin>, f_baud> {
                using pinConfig = decltype(action(
                  Kvasir::Io::Action::PinFunctionDrive<3, i2cDrive(f_baud), false>{},
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
                std::uint32_t high_cycles = period - (period * 3 / 5);
                std::uint32_t low_cycles  = period * 3 / 5;

                // Calculate spike length based on raw cycle counts
                regs.spklen = calcSpkLen(low_cycles);

                // Program values per RP2350 datasheet Table 1053 / Section 12.2.14:
                //   actual t_HIGH = (HCNT + SPKLEN + 7) cycles → write HCNT = high_cycles − SPKLEN − 7
                //   actual t_LOW  = (LCNT + 1) cycles          → write LCNT = low_cycles − 1
                regs.hcnt = high_cycles - regs.spklen - 7;
                regs.lcnt = low_cycles - 1;

                // Calculate SDA hold time
                regs.sda_hold = calcSdaTxHold(f_clockSpeed, f_baud);

                return regs;
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud>
            static constexpr bool isValidSpkLen() {
                constexpr auto regs = calcBaudRegs(f_clockSpeed, f_baud);
                return regs.spklen <= 0xFF;   // IC_FS_SPKLEN is 8-bit
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud>
            static constexpr bool isValidHcnt() {
                constexpr auto regs = calcBaudRegs(f_clockSpeed, f_baud);
                return regs.hcnt <= 0xFFFF   // IC_FS_SCL_HCNT is 16-bit
                    && regs.hcnt > regs.spklen + 5;
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud>
            static constexpr bool isValidLcnt() {
                constexpr auto regs = calcBaudRegs(f_clockSpeed, f_baud);
                return regs.lcnt <= 0xFFFF   // IC_FS_SCL_LCNT is 16-bit
                    && regs.lcnt > regs.spklen + 7;
            }

            template<std::uint32_t f_clockSpeed,
                     std::uint32_t f_baud,
                     std::intmax_t Num,
                     std::intmax_t Denom>
            static constexpr bool isValidBaudConfig(std::ratio<Num,
                                                               Denom>) {
                constexpr auto regs         = calcBaudRegs(f_clockSpeed, f_baud);
                constexpr auto period       = (regs.hcnt + regs.spklen + 7) + (regs.lcnt + 1);
                constexpr auto f_baudActual = f_clockSpeed / period;
                constexpr auto err
                  = f_baudActual > f_baud ? f_baudActual - f_baud : f_baud - f_baudActual;
                constexpr auto maxErr = (f_baud * Num) / Denom;
                return err <= maxErr;
            }

            /// Fast mode for every rate: standard mode (<= 100 kHz) is buggy on this block,
            /// and high-speed mode (> 1 MHz) is not something it does -- the IC_HS_* count
            /// registers are never programmed here and the SCL counts in GetBaudConfig are
            /// the fast-mode ones, so selecting SPEED = high would clock the bus with
            /// numbers meant for another register set. I2CBase static_asserts the ceiling.
            template<std::uint32_t f_baud>
            static constexpr auto getSpeedModeRegister() {
                static_assert(f_baud <= 1'000'000, "fast mode plus (1 MHz) is the ceiling");
                return write(Regs::IC_CON::SPEEDValC::fast);
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

        template<unsigned Instance>
        static constexpr auto getResetDoneBit() {
            static_assert(Instance < 2, "I2C Instance must be 0 or 1");
            if constexpr(Instance == 0) {
                return Peripheral::RESETS::Registers<>::RESET_DONE::i2c0;
            } else {
                return Peripheral::RESETS::Registers<>::RESET_DONE::i2c1;
            }
        }

        /// Registers written before RESET_DONE is set are silently dropped, so wait for it.
        template<unsigned Instance>
        inline void waitResetDone() {
            while(get<0>(apply(read(getResetDoneBit<Instance>()))) == 0) {}
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

            // Startup: this block, and the clock the SCL counts are computed from (the DW
            // I2C counts clk_sys).
            using Provides = brigand::list<Startup::Resource<InstanceTag, Instance>>;
            using Claims   = Clocks::Claim<Clocks::ClkSys, I2CConfig::clockSpeed>;

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

            static_assert(I2CConfig::baudRate <= 1'000'000,
                          "the RP2040/RP2350 I2C block does fast mode plus (1 MHz) at most: "
                          "high-speed mode's own SCL count registers are not programmed by "
                          "this driver");
            static_assert(Config::template isValidSpkLen<I2CConfig::clockSpeed,
                                                         I2CConfig::baudRate>(),
                          "I2C SPKLEN overflows 8-bit register (max 255) — baud rate too low or "
                          "clock speed too high");
            static_assert(Config::template isValidLcnt<I2CConfig::clockSpeed,
                                                       I2CConfig::baudRate>(),
                          "I2C LCNT invalid: must fit in 16 bits and be > SPKLEN+7");
            static_assert(Config::template isValidHcnt<I2CConfig::clockSpeed,
                                                       I2CConfig::baudRate>(),
                          "I2C HCNT invalid: must fit in 16 bits and be > SPKLEN+5");
            static_assert(
              Config::template isValidBaudConfig<I2CConfig::clockSpeed,
                                                 I2CConfig::baudRate>(I2CConfig::maxBaudRateError),
              "I2C baud rate error too large — adjust clockSpeed, baudRate, or maxBaudRateError");
            static_assert(Config::isValidPinLocationSDA(I2CConfig::sdaPinLocation),
                          "invalid SDAPin");
            static_assert(Config::isValidPinLocationSCL(I2CConfig::sclPinLocation),
                          "invalid SCLPin");

            static constexpr auto powerClockEnable = list(Traits::I2C::getEnable<Instance>());

            static constexpr auto initStepPinConfig
              = list(typename Config::template GetSDAPinConfig<
                       std::decay_t<decltype(I2CConfig::sdaPinLocation)>,
                       I2CConfig::baudRate>::pinConfig{},
                     typename Config::template GetSCLPinConfig<
                       std::decay_t<decltype(I2CConfig::sclPinLocation)>,
                       I2CConfig::baudRate>::pinConfig{});

            // TX_EMPTY_CTRL: TX_EMPTY only once the last popped command has actually been
            // shifted out, not as soon as the FIFO drains. The queued driver completes a
            // write on that interrupt and then disables the block and writes the next
            // request's IC_TAR; with the default behaviour that happened while the last
            // byte and its STOP were still on the wire, where a TAR write is ignored.
            static constexpr auto initStepPeripheryConfig
              = list(Regs::IC_CON::overrideDefaults(
                       write(Regs::IC_CON::MASTER_MODEValC::enabled),
                       write(Regs::IC_CON::TX_EMPTY_CTRLValC::enabled),
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

            // abort: use after a TX_ABRT ISR — bus already in clean state (NAK released it),
            // this just flushes the TX FIFO and disables the peripheral.
            static constexpr auto abort
              = list(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::disabled),
                                                       set(Regs::IC_ENABLE::abort)),
                     NoInterrupts);

            // softAbortRequest: use when a transaction must be terminated mid-stream (timeout,
            // recovery). ENABLE=1 is required for the ABORT bit to issue a STOP on the bus
            // (RP2350 datasheet §12.2.11). The hardware clears the ABORT bit when done.
            // The caller must disable the peripheral afterwards (completeCurrentRequest does this).
            static constexpr auto softAbortRequest
              = list(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled),
                                                       set(Regs::IC_ENABLE::abort)),
                     NoInterrupts);

            using AbrtSrc = typename Regs::IC_TX_ABRT_SOURCE;

            // The abort causes worth reporting: everything except the slave-side bits, the
            // flush counter, and abrt_user_abrt -- the last is raised by our own ENABLE.ABORT
            // once the abort has gone through and would make one NACK look like two faults.
            static constexpr auto MasterAbortCauses = static_cast<typename AbrtSrc::Addr::RegType>(
              ~(AbrtSrc::abrt_user_abrt.Mask | AbrtSrc::abrt_slvrd_intx.Mask
                | AbrtSrc::abrt_slv_arblost.Mask | AbrtSrc::abrt_slvflush_txfifo.Mask
                | AbrtSrc::tx_flush_cnt.Mask));

            // Log it as Kvasir::Register::Flags<AbrtSrc>{abortCause()}: the field names of the
            // bits that are set, on one line.
            static auto abortCause() {
                return static_cast<typename AbrtSrc::Addr::RegType>(
                  get<0>(apply(read(AbrtSrc::FULLREGISTER))) & MasterAbortCauses);
            }

            // Reading IC_CLR_TX_ABRT clears IC_TX_ABRT_SOURCE (and the TX_ABRT interrupt), so
            // the next abort reports its own cause rather than a stale one.
            static void clearAbortSource() {
                [[maybe_unused]] auto const cleared
                  = apply(read(Regs::IC_CLR_TX_ABRT::clr_tx_abrt));
            }
        };
    }   // namespace Detail
}}   // namespace Kvasir::I2C
