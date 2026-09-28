#pragma once

#include "chip/rp_common/Io.hpp"
#include "chip/rp_common/PIO.hpp"
#include "displayPio/PioQspi.hpp"
#include "kvasir/Io/Types.hpp"
#include "peripherals/PIO.hpp"
#include "peripherals/RESETS.hpp"

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
    /// kvasir_devices' Display::DcsPanel. The firmware generates `displayPio/PioQspi.hpp`:
    /// pioasm_generate(displayPio INPUT_FILE ${CHIP_ROOT}/src/chip/rp_common/pio/PioQspi.pio).
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
        using Claims
          = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>,
                            Kvasir::Clocks::Claim<Kvasir::Clocks::ClkSys, Config_::clockSpeed>>;

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

        using Programm = Kvasir::Pio::qspiPanelProgramm;

        /// What the bus runs at; DcsPanel checks it against the chip's ceilings.
        static constexpr auto WriteHz = Config::baudRate;
        static constexpr auto ReadHz  = Config::readBaudRate;

        static constexpr unsigned PioInstance = Config::pioInstance;
        static constexpr unsigned SmInstance  = Config::smInstance;

        static_assert(PioInstance < 3,
                      "the RP2350 has PIO0..PIO2");
        static_assert(SmInstance < 4,
                      "invalid state machine index");

        using PioRegs = Kvasir::Peripheral::PIO::Registers<PioInstance>;
        using SmRegs  = typename PioRegs::template SM<SmInstance>;
        using Fifo    = typename PioRegs::template FIFO<SmInstance>;

        static constexpr std::uint32_t SmMask = 1U << SmInstance;

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

        static_assert(Programm::Instructions.size() <= 32,
                      "PIO instruction memory is 32 words");

        // jmp targets are absolute: the program only runs at offset 0.
        static constexpr unsigned ProgrammOffset = 0;

        /// A second program at offset 0 of this PIO is a build error.
        using Provides = Kvasir::Pio::Provides<PioInstance, SmInstance, ProgrammOffset, Programm>;

        enum class State : std::uint8_t { idle, payload, drain, failed };

        static constexpr int PioFunction = Kvasir::Pio::pinFunction<PioInstance>;

        using LanePinConfig = Kvasir::Io::Action::
          PinFunctionDrive<PioFunction, Config::driveStrength, Config::slewFast>;

        static constexpr auto powerClockEnable = list(Kvasir::Pio::getEnable<PioInstance>());

        static constexpr auto initStepPinConfig
          = list(action(LanePinConfig{}, Config::sclkPinLocation),
                 action(LanePinConfig{}, Config::d0PinLocation),
                 action(LanePinConfig{}, Config::d1PinLocation),
                 action(LanePinConfig{}, Config::d2PinLocation),
                 action(LanePinConfig{}, Config::d3PinLocation),
                 makeOutputInitHigh(Config::csPinLocation));

        // The pin groups stay the same in every phase. SET covers all four lanes so the read
        // program's `set pindirs` can release them.
        static constexpr auto PinCtrlConfig = SmRegs::PINCTRL::overrideDefaults(
          write(SmRegs::PINCTRL::sideset_count, Kvasir::Register::value<1>()),
          write(SmRegs::PINCTRL::sideset_base, Kvasir::Register::value<SclkPin>()),
          write(SmRegs::PINCTRL::out_base, Kvasir::Register::value<D0Pin>()),
          write(SmRegs::PINCTRL::out_count, Kvasir::Register::value<4>()),
          write(SmRegs::PINCTRL::set_base, Kvasir::Register::value<D0Pin>()),
          write(SmRegs::PINCTRL::set_count, Kvasir::Register::value<4>()),
          write(SmRegs::PINCTRL::in_base, Kvasir::Register::value<D0Pin>()));

        static constexpr auto initStepPeripheryConfig
          = list(PinCtrlConfig,

                 // Every entry point loops by jmp; the wrap spans all of instruction memory.
                 SmRegs::EXECCTRL::overrideDefaults(
                   write(SmRegs::EXECCTRL::wrap_bottom, Kvasir::Register::value<0>()),
                   write(SmRegs::EXECCTRL::wrap_top, Kvasir::Register::value<31>()),
                   clear(SmRegs::EXECCTRL::side_en),
                   clear(SmRegs::EXECCTRL::side_pindir)));

        /// The reset release is asynchronous: no PIO access before RESET_DONE (the CYW43
        /// transport failed intermittently without this wait).
        static bool resetDone() {
            using Resets = Kvasir::Peripheral::RESETS::Registers<>;
            if constexpr(PioInstance == 0) {
                return get<0>(apply(read(Resets::RESET_DONE::pio0))) != 0;
            } else {
                return get<0>(apply(read(Resets::RESET_DONE::pio1))) != 0;
            }
        }

        static void preEnableRuntimeInit() {
            while(!resetDone()) {}

            for(std::uint32_t volatile* addr = reinterpret_cast<std::uint32_t volatile*>(
                  PioRegs::template INSTR_MEM<ProgrammOffset>::Addr::value);
                auto const v : Programm::Instructions)
            {
                *addr = v;
                ++addr;
            }

            setEnabled(false);

            // Side-set writes values, not directions: SCK's output enable has to be set
            // through the SET group once, or the clock stays an input and the panel is dead.
            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Kvasir::Register::value<SclkPin>()),
              write(SmRegs::PINCTRL::set_count, Kvasir::Register::value<1>())));
            forceInstruction(SetPinDirsOneOutput);

            apply(PinCtrlConfig);
            forceInstruction(SetPinDirsAllOutputs);

            setDivider(WriteDiv);
        }

    private:
        static inline State                      state_{State::idle};
        static inline typename Clock::time_point deadline_{};
        static inline std::uint16_t              fillWord_{};

        // `set pindirs, 0b1111 side 0`: SET 0xE000, destination 4 = pindirs in [7:5], data [4:0].
        static constexpr std::uint32_t SetPinDirsAllOutputs = 0xE000U | (4U << 5U) | 0x0FU;
        static constexpr std::uint32_t SetPinDirsOneOutput  = 0xE000U | (4U << 5U) | 0x01U;

        static void setEnabled(bool on) {
            // Read-modify-write: CTRL.SM_ENABLE holds the other state machines' bits too.
            auto const cur = get<0>(apply(read(PioRegs::CTRL::sm_enable)));
            apply(write(PioRegs::CTRL::sm_enable, on ? (cur | SmMask) : (cur & ~SmMask)));
        }

        static void forceInstruction(std::uint32_t instr) {
            apply(write(SmRegs::INSTR::instr, instr));
        }

        static void setDivider(double div) {
            auto const d = Kvasir::Pio::getDiv(div);
            apply(write(SmRegs::CLKDIV::_int, static_cast<std::uint32_t>(std::get<0>(d))),
                  write(SmRegs::CLKDIV::frac, static_cast<std::uint32_t>(std::get<1>(d))));
        }

        static void clearTxStall() { apply(write(PioRegs::FDEBUG::txstall, SmMask)); }

        static bool txStalled() {
            return (get<0>(apply(read(PioRegs::FDEBUG::txstall))) & SmMask) != 0;
        }

        static bool txFull() { return (get<0>(apply(read(PioRegs::FSTAT::txfull))) & SmMask) != 0; }

        static bool rxEmpty() {
            return (get<0>(apply(read(PioRegs::FSTAT::rxempty))) & SmMask) != 0;
        }

        /// Toggling FJOIN_RX flushes both FIFOs.
        static void clearFifos() {
            apply(write(SmRegs::SHIFTCTRL::fjoin_rx, 1U));
            apply(write(SmRegs::SHIFTCTRL::fjoin_rx, 0U));
        }

        /// Write-phase shift setup: PullThresh 8 for bytes, 16 for the fill pattern.
        template<unsigned PullThresh>
        static void configureWriteShift() {
            static_assert(PullThresh == 8 || PullThresh == 16,
                          "only the byte-wise payload and the two-byte fill pattern exist");
            apply(SmRegs::SHIFTCTRL::overrideDefaults(
              // TX FIFO joined: eight entries.
              write(SmRegs::SHIFTCTRL::fjoin_tx, Kvasir::Register::value<1>()),
              write(SmRegs::SHIFTCTRL::fjoin_rx, Kvasir::Register::value<0>()),
              // Shift left: MSB first, on four lanes high nibble first with D3 as its MSB.
              write(SmRegs::SHIFTCTRL::out_shiftdir, Kvasir::Register::value<0>()),
              write(SmRegs::SHIFTCTRL::autopull, Kvasir::Register::value<1>()),
              write(SmRegs::SHIFTCTRL::autopush, Kvasir::Register::value<0>()),
              write(SmRegs::SHIFTCTRL::push_thresh, Kvasir::Register::value<0>()),
              write(SmRegs::SHIFTCTRL::pull_thresh, Kvasir::Register::value<PullThresh>())));
        }

        static void configureReadShift() {
            apply(SmRegs::SHIFTCTRL::overrideDefaults(
              // No join: the bit count arrives through the TX FIFO.
              write(SmRegs::SHIFTCTRL::fjoin_rx, Kvasir::Register::value<0>()),
              write(SmRegs::SHIFTCTRL::fjoin_tx, Kvasir::Register::value<0>()),
              // Autopull off: the read entry `pull`s its bit count itself.
              write(SmRegs::SHIFTCTRL::autopull, Kvasir::Register::value<0>()),
              write(SmRegs::SHIFTCTRL::in_shiftdir, Kvasir::Register::value<0>()),
              write(SmRegs::SHIFTCTRL::autopush, Kvasir::Register::value<1>()),
              write(SmRegs::SHIFTCTRL::push_thresh, Kvasir::Register::value<8>())));
        }

        /// Point the stopped state machine at an entry point. SM_RESTART leaves the PC alone,
        /// hence the forced jmp after it.
        static void enterPhase(unsigned entry) {
            setEnabled(false);
            apply(write(PioRegs::CTRL::sm_restart, SmMask),
                  write(PioRegs::CTRL::clkdiv_restart, SmMask));
            forceInstruction(entry + ProgrammOffset);
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

            enterPhase(Programm::offset_single);
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
                                Kvasir::Pio::getTxDmaTrigger<Dma, PioInstance, SmInstance>(),
                                Size,
                                false,
                                IncrementSource>(Fifo::TXF::Addr::value, source, count);
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

            enterPhase(Programm::offset_quad);
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

            enterPhase(Programm::offset_quad);
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

            setDivider(ReadDiv);
            csAssert();

            auto const restore = [] {
                setEnabled(false);
                // The read left the lanes as inputs.
                forceInstruction(SetPinDirsAllOutputs);
                csRelease();
                setDivider(WriteDiv);
            };

            // CO5300 datasheet 5.2.3.
            if(!sendHeader(instruction, cmd, {})) {
                restore();
                state_ = State::failed;
                return false;
            }

            enterPhase(Programm::offset_read);
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
