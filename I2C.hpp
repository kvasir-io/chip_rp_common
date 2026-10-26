#pragma once

#include "Clocks.hpp"
#include "I2CMath.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "WaitBounds.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "core/Nvic.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/I2C.hpp"
#include "peripherals/RESETS.hpp"

namespace Kvasir { namespace I2C {

    namespace Detail {

        template<unsigned Instance>
        struct Config : I2CMath {   // the SCL counts' arithmetic: I2CMath.hpp
            using Regs = Kvasir::Peripheral::I2C::Registers<Instance>;

            // I2C is open-drain: the pad only ever drives low. Keep the driver weak
            // and slew slow so falling edges stay within the I2C spec t_f window
            // (>= ~12 ns for fast mode) — a 12 mA pad produces ~1.5 ns edges with
            // heavy ringing. Standard mode (<= 100 kHz) gets 2 mA, faster modes 4 mA.
            static constexpr Io::DriveStrength i2cDrive(std::uint32_t fBaud) {
                return fBaud <= 100'000 ? Io::DriveStrength::mA2 : Io::DriveStrength::mA4;
            }

            /// Fast mode for every rate: standard mode (<= 100 kHz) is buggy on this block,
            /// and high-speed mode (> 1 MHz) is not something it does -- the IC_HS_* count
            /// registers are never programmed here and the SCL counts in GetBaudConfig are
            /// the fast-mode ones, so selecting SPEED = high would clock the bus with
            /// numbers meant for another register set. I2CBase static_asserts the ceiling.
            template<std::uint32_t FBaud>
            static constexpr auto getSpeedModeRegister() {
                static_assert(FBaud <= 1'000'000, "fast mode plus (1 MHz) is the ceiling");
                return write(Regs::IC_CON::SPEEDValC::fast);
            }

            template<std::uint32_t FClockSpeed, std::uint32_t FBaud>
            struct GetBaudConfig {
                static constexpr auto config_ = []() {
                    constexpr auto regs = calcBaudRegs(FClockSpeed, FBaud);

                    return list(
                      write(Regs::IC_FS_SCL_HCNT::ic_fs_scl_hcnt, Register::value<regs.hcnt>()),
                      write(Regs::IC_FS_SCL_LCNT::ic_fs_scl_lcnt, Register::value<regs.lcnt>()),
                      write(Regs::IC_FS_SPKLEN::ic_fs_spklen, Register::value<regs.spklen>()),
                      Regs::IC_SDA_HOLD::overrideDefaults(
                        write(Regs::IC_SDA_HOLD::ic_sda_tx_hold, Register::value<regs.sdaHold>())));
                }();
                using config = decltype(config_);
            };
        };
    }   // namespace Detail

    namespace Detail {

        template<typename I2CConfig_>
        struct I2CBase {
            struct I2CConfig : I2CConfig_ {
                static constexpr auto userConfigOverride
                  = KVASIR_CONFIG_OR(I2CConfig_, userConfigOverride, brigand::list<>{});

                static constexpr auto maxBaudRateError
                  = KVASIR_CONFIG_OR(I2CConfig_, maxBaudRateError, std::ratio<1, 100>{});

                /// Each device at its own clock, switched between transfers (I2CQueued's
                /// timing()); `baudRate` is then the fastest any device gets and the rate
                /// the block starts with.
                static constexpr bool perDeviceClock
                  = static_cast<bool>(KVASIR_CONFIG_OR(I2CConfig_, perDeviceClock, false));

                /// The slowest rate a device on this bus runs at: what the idle watchdog of
                /// LineRecovery scales its threshold with. Only perDeviceClock makes it differ.
                static constexpr std::uint32_t minBaudRate = static_cast<std::uint32_t>(
                  KVASIR_CONFIG_OR(I2CConfig_,
                                   minBaudRate,
                                   static_cast<std::uint32_t>(I2CConfig_::baudRate)));
            };

            // needed config
            // clockSpeed
            // baudRate
            // maxBaudRateError
            // i2cInstance
            // SdaPinLocation
            // SclPinLocation
            // userConfigOverride
            static constexpr auto instance = I2CConfig::instance;
            using Regs                     = Kvasir::Peripheral::I2C::Registers<instance>;
            using Config                   = Detail::Config<instance>;

            using InterruptIndexs = typename Rp::Block<Regs>::Irqs;

            // Startup: this block, and the clock the SCL counts are computed from (the DW
            // I2C counts clk_sys).
            using Provides = brigand::list<Hal::Owns<Regs>>;
            using Claims   = Hal::Claim<Clocks::ClkSys, I2CConfig::clockSpeed>;

            static constexpr auto noInterrupts
              = list(Regs::IC_INTR_MASK::overrideDefaults(clear(Regs::IC_INTR_MASK::m_gen_call),
                                                          clear(Regs::IC_INTR_MASK::m_rx_done),
                                                          clear(Regs::IC_INTR_MASK::m_tx_abrt),
                                                          clear(Regs::IC_INTR_MASK::m_rd_req),
                                                          clear(Regs::IC_INTR_MASK::m_tx_empty),
                                                          clear(Regs::IC_INTR_MASK::m_tx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_full),
                                                          clear(Regs::IC_INTR_MASK::m_rx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_under)));

            static constexpr auto txInterrupts
              = list(Regs::IC_INTR_MASK::overrideDefaults(clear(Regs::IC_INTR_MASK::m_gen_call),
                                                          clear(Regs::IC_INTR_MASK::m_rx_done),
                                                          set(Regs::IC_INTR_MASK::m_tx_abrt),
                                                          clear(Regs::IC_INTR_MASK::m_rd_req),
                                                          set(Regs::IC_INTR_MASK::m_tx_empty),
                                                          clear(Regs::IC_INTR_MASK::m_tx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_full),
                                                          clear(Regs::IC_INTR_MASK::m_rx_over),
                                                          clear(Regs::IC_INTR_MASK::m_rx_under)));

            static constexpr auto rxInterrupts
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
            // the achieved SCL against maxBaudRateError; a failure prints wanted, got and ppm
            static constexpr bool baudInTolerance = [] {
                Prescaler::assertInTolerance<Config::achievedRate(
                                               I2CConfig::clockSpeed,
                                               Config::calcBaudRegs(I2CConfig::clockSpeed,
                                                                    I2CConfig::baudRate)),
                                             I2CConfig::baudRate,
                                             Prescaler::Tolerance{I2CConfig::maxBaudRateError},
                                             "I2C SCL">();
                return true;
            }();
            // a static data member of a class template is initialised only when used: this use
            // is what runs the check
            static_assert(baudInTolerance);
            static_assert(I2CConfig::minBaudRate <= I2CConfig::baudRate
                            && (I2CConfig::perDeviceClock
                                || I2CConfig::minBaudRate == I2CConfig::baudRate),
                          "minBaudRate is the slowest device on a perDeviceClock bus, at most "
                          "baudRate");
            static_assert(Io::pinServes<PinConfig::Function::i2cSda,
                                        instance>(I2CConfig::sdaPinLocation));
            static_assert(Io::pinServes<PinConfig::Function::i2cScl,
                                        instance>(I2CConfig::sclPinLocation));

            static constexpr auto powerClockEnable = list(clear(Rp::Block<Regs>::reset));

            static constexpr auto initStepPinConfig = list(
              typename Io::Detail::PinConfigFor<std::decay_t<decltype(I2CConfig::sdaPinLocation)>,
                                                PinConfig::Function::i2cSda,
                                                instance,
                                                Config::i2cDrive(I2CConfig::baudRate)>::type{},
              typename Io::Detail::PinConfigFor<std::decay_t<decltype(I2CConfig::sclPinLocation)>,
                                                PinConfig::Function::i2cScl,
                                                instance,
                                                Config::i2cDrive(I2CConfig::baudRate)>::type{});

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
                     noInterrupts);

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
                     noInterrupts);

            // softAbortRequest: use when a transaction must be terminated mid-stream (timeout,
            // recovery). ENABLE=1 is required for the ABORT bit to issue a STOP on the bus
            // (RP2350 datasheet §12.2.11). The hardware clears the ABORT bit when done.
            // The caller must disable the peripheral afterwards (completeCurrentRequest does this).
            static constexpr auto softAbortRequest
              = list(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled),
                                                       set(Regs::IC_ENABLE::abort)),
                     noInterrupts);

            // cancelAbortRequest: a cancel of the request on the wire. ABORT is accepted only while ENABLE is set;
            // the controller "issues a STOP and flushes the Tx FIFO after completing the current transfer, then
            // sets the TX_ABORT interrupt" (IC_ENABLE.ABORT, RP2350 md l.50588, RP2040 md l.22699). Unlike
            // softAbortRequest the interrupts stay unmasked: TX_ABRT (ABRT_USER_ABRT) completes the request.
            static constexpr auto cancelAbortRequest
              = list(Regs::IC_ENABLE::overrideDefaults(write(Regs::IC_ENABLE::ENABLEValC::enabled),
                                                       set(Regs::IC_ENABLE::abort)));

            using AbrtSrc = typename Regs::IC_TX_ABRT_SOURCE;

            // The abort causes worth reporting: everything except the slave-side bits, the
            // flush counter, and abrt_user_abrt -- the last is raised by our own ENABLE.ABORT
            // once the abort has gone through and would make one NACK look like two faults.
            static constexpr auto masterAbortCauses = static_cast<typename AbrtSrc::Addr::RegType>(
              ~(AbrtSrc::abrt_user_abrt.mask | AbrtSrc::abrt_slvrd_intx.mask
                | AbrtSrc::abrt_slv_arblost.mask | AbrtSrc::abrt_slvflush_txfifo.mask
                | AbrtSrc::tx_flush_cnt.mask));

            // Log it as Kvasir::Register::Flags<AbrtSrc>{abortCause()}: the field names of the
            // bits that are set, on one line.
            static auto abortCause() {
                return static_cast<typename AbrtSrc::Addr::RegType>(
                  get<0>(apply(read(AbrtSrc::FULLREGISTER))) & masterAbortCauses);
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
