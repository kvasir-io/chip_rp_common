#pragma once

#include "chip/rp_common/Io.hpp"
#include "chip/rp_common/PIO.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/PioQspiProgram.hpp"
#include "kvasir/Io/Types.hpp"
#include "peripherals/PIO.hpp"

#include <cassert>
#include <chip/rp_common/Clocks.hpp>
#include <chip/rp_common/PIO.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Display {

    /// QSPI transport for a DCS panel controller (CO5300, ST77916, ST77922) on one RP2350 PIO
    /// state machine plus one DMA channel; the PL022 is single-lane only. The panel driver is
    /// kvasir_devices' Display::DcsPanel. The PIO program is QspiPanelProgram
    /// (PioQspiProgram.hpp), built by the compiler: no build step.
    ///
    /// Framing: an instruction byte, a 24 bit address {00h, CMD, 00h}, then parameters
    /// single-lane (02h) or pixels on four lanes (32h). Reads differ per chip, so
    /// readRegister() takes the instruction and the dummy byte count from the caller.
    ///
    /// Required Config members:
    ///   clockSpeed          system clock feeding the PIO
    ///   csPinLocation       chip select, driven here as a plain GPIO
    ///   sclkPinLocation     SCK, the state machine's side-set pin
    ///   d0PinLocation .. d3PinLocation   the four data lanes, which must be consecutive
    ///   pioInstance         0 or 1
    ///   smInstance          0..3
    /// Optional:
    ///   baudRate      (default 37.5 MHz) write clock; the panel allows up to 50 MHz
    ///   readBaudRate  (default 5 MHz)    read clock; the panel allows up to 10 MHz
    ///   timeout       (default 250 ms)   deadline for one transfer
    ///   driveStrength / slewFast         pad overrides for SCK and the data lanes
    ///
    /// writePixels() and fillPixels() are asynchronous: writePixels()' span must stay valid
    /// until idle(). handler() must be polled from the main loop, it detects completion.
    template<typename Clock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             typename Config_>
    struct PioQspi {
        struct Config : Config_ {
            static constexpr auto baudRate = [] {
                if constexpr(requires { Config_::baudRate; }) {
                    return Config_::baudRate;
                } else {
                    return 37'500'000;
                }
            }();

            static constexpr auto readBaudRate = [] {
                if constexpr(requires { Config_::readBaudRate; }) {
                    return Config_::readBaudRate;
                } else {
                    return 5'000'000;
                }
            }();

            static constexpr auto timeout = [] {
                if constexpr(requires { Config_::timeout; }) {
                    return Config_::timeout;
                } else {
                    return std::chrono::milliseconds{250};
                }
            }();

            static constexpr auto driveStrength = [] {
                if constexpr(requires { Config_::driveStrength; }) {
                    return Config_::driveStrength;
                } else {
                    return Kvasir::Io::DriveStrength::mA_8;
                }
            }();

            static constexpr auto slewFast = [] {
                if constexpr(requires { Config_::slewFast; }) {
                    return Config_::slewFast;
                } else {
                    return true;
                }
            }();
        };

        using Programm = QspiPanelProgram;

        /// What the bus runs at; DcsPanel checks it against the chip's ceilings.
        static constexpr auto WriteHz = Config::baudRate;
        static constexpr auto ReadHz  = Config::readBaudRate;

        static constexpr unsigned PioInstance = Config::pioInstance;
        static constexpr unsigned SmInstance  = Config::smInstance;

        static_assert(PioInstance < 3,
                      "the RP2350 has PIO0..PIO2");
        static_assert(SmInstance < 4,
                      "invalid state machine index");

        static constexpr auto pinNumber(auto pin) {
            return []<int Port, int PinN>(Kvasir::Register::PinLocation<Port, PinN>) {
                return PinN;
            }(pin);
        }

        static constexpr auto SclkPin = pinNumber(Config::sclkPinLocation);
        static constexpr auto D0Pin   = pinNumber(Config::d0PinLocation);

        // `out pins, 4` drives four consecutive GPIOs from out_base.
        static_assert(pinNumber(Config::d1PinLocation) == D0Pin + 1
                        && pinNumber(Config::d2PinLocation) == D0Pin + 2
                        && pinNumber(Config::d3PinLocation) == D0Pin + 3,
                      "D0..D3 must be four consecutive GPIOs in ascending order");

        // Two PIO cycles per bit: clock low while driving, high while the panel samples.
        static constexpr double WriteDiv{static_cast<double>(Config::clockSpeed)
                                         / (2.0 * static_cast<double>(Config::baudRate))};
        static constexpr double ReadDiv{static_cast<double>(Config::clockSpeed)
                                        / (2.0 * static_cast<double>(Config::readBaudRate))};

        // Kvasir::Pio::getDiv silently truncates the integer part to 8 bits.
        static_assert(WriteDiv >= 1.0 && WriteDiv < 256.0,
                      "baudRate unreachable from clockSpeed with an 8 bit PIO divider");
        static_assert(ReadDiv >= 1.0 && ReadDiv < 256.0,
                      "readBaudRate unreachable from clockSpeed with an 8 bit PIO divider");

        // jmp targets are absolute: the program only runs at offset 0.
        static constexpr unsigned ProgrammOffset = 0;

        template<auto Location>
        using PinOf = std::remove_cvref_t<decltype(Location)>;

        // The state machine: StateMachine (PioStateMachine.hpp) loads the program, maps the
        // pins, gives SCK and the lanes the PIO function with this bus's pad drive and slew,
        // makes them outputs, sets the write clock, and checks it all against the program's
        // .side_set. It stays stopped after Startup: every transfer starts it at an entry point.
        // The pin groups stay the same in every phase; SET covers all four lanes so the read
        // entry's `set pindirs` can release them, and `in pins, 1` samples D0.
        struct SmConfig {
            static constexpr auto     ClockSpeed    = Config::clockSpeed;
            static constexpr auto     PioInstance   = Config::pioInstance;
            static constexpr auto     SmInstance    = Config::smInstance;
            static constexpr unsigned ProgramOffset = ProgrammOffset;
            static constexpr double   clockDiv      = WriteDiv;
            static constexpr auto     sidesetPins = brigand::list<PinOf<Config::sclkPinLocation>>{};
            using Lanes                           = brigand::list<PinOf<Config::d0PinLocation>,
                                                                  PinOf<Config::d1PinLocation>,
                                                                  PinOf<Config::d2PinLocation>,
                                                                  PinOf<Config::d3PinLocation>>;
            static constexpr auto outPins         = Lanes{};
            static constexpr auto setPins         = Lanes{};
            static constexpr auto inPins          = brigand::list<PinOf<Config::d0PinLocation>>{};
            static constexpr auto driveStrength   = Config::driveStrength;
            static constexpr bool slewFast        = Config::slewFast;
            static constexpr bool startEnabled    = false;
        };

        using Sm = Kvasir::Pio::StateMachine<Programm, SmConfig>;

        /// A second program at offset 0 of this PIO is a build error.
        using Provides = typename Sm::Provides;
        using Claims   = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>, typename Sm::Claims>;

        enum class State : std::uint8_t { idle, payload, drain, failed };

        static constexpr auto powerClockEnable = Sm::powerClockEnable;

        // the chip select is a plain GPIO, driven here
        static constexpr auto initStepPinConfig
          = brigand::append<std::remove_cvref_t<decltype(Sm::initStepPinConfig)>,
                            decltype(list(makeOutputInitHigh(Config::csPinLocation)))>{};

        static constexpr auto initStepPeripheryConfig = Sm::initStepPeripheryConfig;
        static constexpr auto initStepPeripheryEnable = Sm::initStepPeripheryEnable;

        static void runtimeInit() { Sm::runtimeInit(); }

        static void preEnableRuntimeInit() { Sm::preEnableRuntimeInit(); }

    private:
        static inline State                      state_{State::idle};
        static inline typename Clock::time_point deadline_{};
        static inline std::uint16_t              fillWord_{};

        using SmRegs = typename Sm::SmRegs;
        using Fifo   = typename Sm::Fifo;

        // `set pindirs, 0b1111`: the four lanes outputs again after a read (SET 0xE000,
        // destination 4 = pindirs in [7:5], data [4:0])
        static constexpr std::uint16_t SetPinDirsAllOutputs = 0xE000U | (4U << 5U) | 0x0FU;

        static void setEnabled(bool on) { Sm::setEnabled(on); }

        static void clearTxStall() { Sm::clearTxStall(); }

        static bool txStalled() { return Sm::txStalled(); }

        static bool txFull() { return Sm::txFull(); }

        static bool rxEmpty() { return Sm::rxEmpty(); }

        // The write phases: TX FIFO joined (eight entries), autopull, shift left - MSB first,
        // on four lanes high nibble first with D3 as its MSB; PullThresh 8 for bytes, 16 for the
        // fill pattern.
        template<unsigned PullThresh>
        static void configureWriteShift() {
            static_assert(PullThresh == 8 || PullThresh == 16,
                          "only the byte-wise payload and the two-byte fill pattern exist");
            Sm::template setShift<Kvasir::Pio::Shift{.autopull      = true,
                                                     .pullThreshold = PullThresh,
                                                     .outShiftRight = false,
                                                     .joinTx        = true}>();
        }

        // The read phase: no join (the bit count arrives through the TX FIFO), autopull off (the
        // read entry `pull`s its bit count itself), autopush every 8 bits, shift left.
        static void configureReadShift() {
            Sm::template setShift<
              Kvasir::Pio::Shift{.autopush = true, .pushThreshold = 8, .inShiftRight = false}>();
        }

        static void clearFifos() { Sm::clearFifos(); }

        /// Point the stopped state machine at an entry point. SM_RESTART leaves the PC alone,
        /// hence the forced jmp after it.
        static void enterPhase(unsigned entry) {
            setEnabled(false);
            Sm::restartAt(entry);
        }

        static void pushByte(std::uint8_t b) {
            // Frames are preloaded into a stopped state machine: full means too long.
            assert(!txFull());
            // Shift left with pull_thresh 8: the byte goes to OSR[31:24].
            apply(write(Fifo::TXF::fifo, static_cast<std::uint32_t>(b) << 24U));
        }

        static void pushWord(std::uint32_t w) { apply(write(Fifo::TXF::fifo, w)); }

        static void csAssert() { apply(clear(Config::csPinLocation)); }

        static void csRelease() { apply(set(Config::csPinLocation)); }

        /// Clock a short single-lane frame out and wait for it. The frame is in the FIFO
        /// before enable: a mid-frame stall would latch the sticky FDEBUG_TXSTALL early.
        static bool sendHeader(std::uint8_t               instruction,
                               std::uint8_t               command,
                               std::span<std::byte const> params) {
            assert(params.size() <= 4);

            enterPhase(Programm::offset("single"));
            clearFifos();
            configureWriteShift<8>();

            // CO5300 datasheet 5.2.1.
            pushByte(instruction);
            pushByte(0x00);
            pushByte(command);
            pushByte(0x00);
            for(auto const p : params) { pushByte(static_cast<std::uint8_t>(p)); }

            clearTxStall();
            setEnabled(true);

            auto const deadline = Clock::now() + Config::timeout;
            while(!txStalled()) {
                if(Clock::now() > deadline) {
                    setEnabled(false);
                    return false;
                }
            }
            setEnabled(false);
            return true;
        }

        template<typename Dma::TransferSize Size,
                 bool                       IncrementSource>
        static void startPayloadDma(std::uint32_t source,
                                    std::size_t   count) {
            // A narrow write is replicated across the 32 bit word, so an 8 bit transfer
            // lands in OSR[31:24]: no alignment or byte swapping (as ws2812.hpp does).
            Dma::template start<DmaChannel,
                                DmaPriority,
                                Sm::template txDmaTrigger<Dma>(),
                                Size,
                                false,
                                IncrementSource>(Sm::txFifoAddress, source, count);
        }

    public:
        // SecondaryCore rendezvous hook, on core 0 before every launch of the owning core:
        // quiets a transfer the previous run died in. Only when one is in flight - before
        // the first launch PIO and DMA are still in reset, and touching them would fault.
        static void primaryPrepare() {
            if(state_ == State::payload || state_ == State::drain) {
                static_cast<void>(Dma::template abort<DmaChannel>());
                setEnabled(false);
                csRelease();
            }
            state_ = State::idle;
        }

        static bool idle() { return state_ == State::idle; }

        static bool failed() { return state_ == State::failed; }

        static State state() { return state_; }

        /// Clear a latched failure.
        static void clearError() {
            if(state_ == State::failed) { state_ = State::idle; }
        }

        /// Write one command with at most four parameters, synchronously (< 2 us).
        static bool writeCommand(std::uint8_t               cmd,
                                 std::span<std::byte const> params = {}) {
            assert(idle());
            csAssert();
            auto const ok = sendHeader(0x02, cmd, params);
            csRelease();
            if(!ok) { state_ = State::failed; }
            return ok;
        }

        /// Stream pixels into panel RAM, asynchronously: `data` must stay valid until idle().
        /// `continuation` selects RAMWRC (3Ch) over RAMWR (2Ch).
        static void writePixels(std::span<std::byte const> data,
                                bool                       continuation = false) {
            assert(idle());
            assert(!data.empty());

            csAssert();
            // A 32h frame's header is single-lane, only the payload is quad.
            if(!sendHeader(0x32, continuation ? 0x3C : 0x2C, {})) {
                csRelease();
                state_ = State::failed;
                return;
            }

            enterPhase(Programm::offset("quad"));
            configureWriteShift<8>();
            startPayloadDma<Dma::TransferSize::_8, true>(
              reinterpret_cast<std::uint32_t>(data.data()),
              data.size());
            setEnabled(true);

            deadline_ = Clock::now() + Config::timeout;
            state_    = State::payload;
        }

        /// Repeat one colour over `pixels` pixels: the DMA rereads one 16 bit word.
        static void fillPixels(std::uint8_t colourHigh,
                               std::uint8_t colourLow,
                               std::size_t  pixels,
                               bool         continuation = false) {
            assert(idle());
            assert(pixels != 0);

            fillWord_ = static_cast<std::uint16_t>((static_cast<std::uint16_t>(colourHigh) << 8U)
                                                   | static_cast<std::uint16_t>(colourLow));

            csAssert();
            if(!sendHeader(0x32, continuation ? 0x3C : 0x2C, {})) {
                csRelease();
                state_ = State::failed;
                return;
            }

            enterPhase(Programm::offset("quad"));
            configureWriteShift<16>();
            startPayloadDma<Dma::TransferSize::_16, false>(
              reinterpret_cast<std::uint32_t>(&fillWord_),
              pixels);
            setEnabled(true);

            deadline_ = Clock::now() + Config::timeout;
            state_    = State::payload;
        }

        /// Read a register back, synchronously; the header runs at readBaudRate too.
        /// `instruction` 03h / `dummyBytes` 0 on the CO5300, 0Bh / 1 on the Sitronix parts.
        static bool readRegister(std::uint8_t         instruction,
                                 std::uint8_t         cmd,
                                 std::size_t          dummyBytes,
                                 std::span<std::byte> out) {
            assert(idle());
            assert(!out.empty());

            Sm::template setClockDiv<ReadDiv>();
            csAssert();

            auto const restore = [] {
                setEnabled(false);
                // The read left the lanes as inputs.
                Sm::exec(SetPinDirsAllOutputs);
                csRelease();
                Sm::template setClockDiv<WriteDiv>();
            };

            // CO5300 datasheet 5.2.3.
            if(!sendHeader(instruction, cmd, {})) {
                restore();
                state_ = State::failed;
                return false;
            }

            enterPhase(Programm::offset("read"));
            clearFifos();
            configureReadShift();
            auto const total = out.size() + dummyBytes;
            pushWord(static_cast<std::uint32_t>(total) * 8U - 1U);
            setEnabled(true);

            auto const deadline = Clock::now() + Config::timeout;
            for(std::size_t i = 0; i < total; ++i) {
                while(rxEmpty()) {
                    if(Clock::now() > deadline) {
                        restore();
                        state_ = State::failed;
                        return false;
                    }
                }
                // Right-justified, MSB first.
                auto const b = static_cast<std::byte>(get<0>(apply(read(Fifo::RXF::fifo))) & 0xFFU);
                if(i >= dummyBytes) { out[i - dummyBytes] = b; }
            }

            restore();
            return true;
        }

        static void handler() {
            switch(state_) {
            case State::payload:
                if(Dma::template ready<DmaChannel>()) {
                    // TXSTALL is sticky and may be set by an earlier starvation: clear it
                    // and wait for it to come back.
                    clearTxStall();
                    state_ = State::drain;
                } else if(Clock::now() > deadline_) {
                    static_cast<void>(Dma::template abort<DmaChannel>());
                    setEnabled(false);
                    csRelease();
                    state_ = State::failed;
                }
                break;

            case State::drain:
                if(txStalled()) {
                    setEnabled(false);
                    csRelease();
                    state_ = State::idle;
                } else if(Clock::now() > deadline_) {
                    setEnabled(false);
                    csRelease();
                    state_ = State::failed;
                }
                break;

            case State::idle:
            case State::failed: break;
            }
        }
    };

}}   // namespace Kvasir::Display
