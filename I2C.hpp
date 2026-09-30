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

            // The I2C spec's 50 ns spike filter in ic_clk cycles, rounded up, at least 1 (RP2350
            // datasheet 12.2.11, RP2040 datasheet 4.3.11). pico-sdk's lcnt / 16 filters far more.
            static constexpr std::uint32_t calcSpkLen(std::uint32_t f_clockSpeed) {
                auto const cycles = static_cast<std::uint32_t>(
                  (std::uint64_t{f_clockSpeed} * 50 + 999'999'999) / 1'000'000'000);
                return cycles == 0 ? 1U : cycles;
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

                regs.spklen = calcSpkLen(f_clockSpeed);

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

            /// One device's SCL counts on a bus that runs each device at its own clock
            /// (I2CConfig::perDeviceClock): what GetBaudConfig writes, as values a request
            /// carries, plus its microseconds per byte for the transfer timeout.
            struct ClockTiming {
                std::uint16_t hcnt{};
                std::uint16_t lcnt{};
                std::uint16_t sdaHold{};
                std::uint16_t usPerByte{};
                std::uint8_t  spklen{};

                constexpr bool operator==(ClockTiming const&) const = default;
            };

            /// The same numbers and the same checks as GetBaudConfig and I2CBase's
            /// static_asserts, for a rate known only per device. A failed check is a call to
            /// one of the functions below it, which are not constexpr: the compile error
            /// names what is wrong.
            template<std::intmax_t Num,
                     std::intmax_t Denom>
            static consteval ClockTiming clockTiming(std::uint32_t f_clockSpeed,
                                                     std::uint32_t f_baud,
                                                     std::ratio<Num,
                                                                Denom>) {
                if(f_baud == 0 || f_baud > 1'000'000) { i2cDeviceClockAboveFastModePlus(); }
                auto const regs = calcBaudRegs(f_clockSpeed, f_baud);
                if(regs.hcnt > 0xFFFF || regs.hcnt <= regs.spklen + 5) {
                    i2cDeviceClockGivesAnInvalidHcnt();
                }
                if(regs.lcnt > 0xFFFF || regs.lcnt <= regs.spklen + 7) {
                    i2cDeviceClockGivesAnInvalidLcnt();
                }
                auto const period = (regs.hcnt + regs.spklen + 7) + (regs.lcnt + 1);
                auto const actual = f_clockSpeed / period;
                auto const err    = actual > f_baud ? actual - f_baud : f_baud - actual;
                if(err > (f_baud * Num) / Denom) { i2cDeviceClockErrorAboveMaxBaudRateError(); }
                return ClockTiming{.hcnt      = static_cast<std::uint16_t>(regs.hcnt),
                                   .lcnt      = static_cast<std::uint16_t>(regs.lcnt),
                                   .sdaHold   = static_cast<std::uint16_t>(regs.sda_hold),
                                   .usPerByte = static_cast<std::uint16_t>(usPerDataByte(f_baud)),
                                   .spklen    = static_cast<std::uint8_t>(regs.spklen)};
            }

            // Not constexpr: reaching one in clockTiming() is the compile error that says why.
            static void i2cDeviceClockAboveFastModePlus() {}

            static void i2cDeviceClockGivesAnInvalidHcnt() {}

            static void i2cDeviceClockGivesAnInvalidLcnt() {}

            static void i2cDeviceClockErrorAboveMaxBaudRateError() {}

            /// The transfer timeout's time per byte: 9 bits, 4 times over.
            static constexpr std::uint32_t usPerDataByte(std::uint32_t f_baud) {
                constexpr std::uint32_t bitsPerDataByte = 9;
                constexpr std::uint32_t safetyFactor    = 4;
                return (bitsPerDataByte * 1'000'000 * safetyFactor) / f_baud;
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

                /// Each device at its own clock, switched between transfers (I2CQueued's
                /// timing()); `baudRate` is then the fastest any device gets and the rate
                /// the block starts with.
                static constexpr bool perDeviceClock = [] {
                    if constexpr(requires { I2CConfig_::perDeviceClock; }) {
                        return static_cast<bool>(I2CConfig_::perDeviceClock);
                    } else {
                        return false;
                    }
                }();

                /// The slowest rate a device on this bus runs at: what the idle watchdog of
                /// LineRecovery scales its threshold with. Only perDeviceClock makes it differ.
                static constexpr std::uint32_t minBaudRate = [] {
                    if constexpr(requires { I2CConfig_::minBaudRate; }) {
                        return static_cast<std::uint32_t>(I2CConfig_::minBaudRate);
                    } else {
                        return static_cast<std::uint32_t>(I2CConfig_::baudRate);
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
            static_assert(I2CConfig::minBaudRate <= I2CConfig::baudRate
                            && (I2CConfig::perDeviceClock
                                || I2CConfig::minBaudRate == I2CConfig::baudRate),
                          "minBaudRate is the slowest device on a perDeviceClock bus, at most "
                          "baudRate");
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

            static constexpr auto
            calcTransferTimeout(std::size_t   numBytes,
                                std::uint32_t microsecondsPerDataByte
                                = Config::usPerDataByte(I2CConfig::baudRate)) {
                using namespace std::chrono_literals;
                constexpr auto baseTimeout = 10ms;

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
