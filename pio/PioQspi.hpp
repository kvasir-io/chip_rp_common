#pragma once

#include "chip/rp_common/DMA.hpp"
#include "chip/rp_common/Io.hpp"
#include "chip/rp_common/PIO.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/PioQspiProgram.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/Link.hpp"
#include "peripherals/PIO.hpp"

#include <cassert>
#include <chip/rp_common/Clocks.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Display {

    namespace detail {
        // PioQspi's configuration, divider checks and state machine, outside the class so that the
        // state machine can be PioQspi's base (Pio::StartupOf); PioQspi derives from it too.
        template<typename Config_>
        struct PioQspiParts {
            struct Config : Config_ {
                static constexpr auto baudRate = KVASIR_CONFIG_OR(Config_, baudRate, 37'500'000);

                static constexpr auto readBaudRate
                  = KVASIR_CONFIG_OR(Config_, readBaudRate, 5'000'000);

                static constexpr auto timeout
                  = KVASIR_CONFIG_OR(Config_, timeout, std::chrono::milliseconds{250});

                static constexpr auto driveStrength
                  = KVASIR_CONFIG_OR(Config_, driveStrength, Kvasir::Io::DriveStrength::mA8);

                static constexpr auto slewFast = KVASIR_CONFIG_OR(Config_, slewFast, true);
            };

            using Program = QspiPanelProgram;

            /// What the bus runs at; DcsPanel checks it against the chip's ceilings.
            static constexpr auto writeHz = Config::baudRate;
            static constexpr auto readHz  = Config::readBaudRate;

            static constexpr unsigned pioInstance = Config::pioInstance;
            static constexpr unsigned smInstance  = Config::smInstance;

            static_assert(pioInstance < 3,
                          "the RP2350 has PIO0..PIO2");
            static_assert(smInstance < 4,
                          "invalid state machine index");

            static constexpr auto sclkPin = Kvasir::Io::pinNumber(Config::sclkPinLocation);
            static constexpr auto d0Pin   = Kvasir::Io::pinNumber(Config::d0PinLocation);

            // `out pins, 4` drives four consecutive GPIOs from out_base.
            static_assert(Kvasir::Io::pinNumber(Config::d1PinLocation) == d0Pin + 1
                            && Kvasir::Io::pinNumber(Config::d2PinLocation) == d0Pin + 2
                            && Kvasir::Io::pinNumber(Config::d3PinLocation) == d0Pin + 3,
                          "D0..D3 must be four consecutive GPIOs in ascending order");

            // Two PIO cycles per bit: clock low while driving, high while the panel samples.
            static constexpr double writeDiv{static_cast<double>(Config::clockSpeed)
                                             / (2.0 * static_cast<double>(Config::baudRate))};
            static constexpr double readDiv{static_cast<double>(Config::clockSpeed)
                                            / (2.0 * static_cast<double>(Config::readBaudRate))};

            // SMx_CLKDIV is a 16.8 divider, INT 31:16 and FRAC 15:8 (RP2350 datasheet Table 995).
            static_assert(writeDiv >= 1.0 && writeDiv < 65536.0,
                          "baudRate unreachable from clockSpeed with a 16.8 PIO divider");
            static_assert(readDiv >= 1.0 && readDiv < 65536.0,
                          "readBaudRate unreachable from clockSpeed with a 16.8 PIO divider");

            // jmp targets are absolute: the program only runs at offset 0.
            static constexpr unsigned programOffsetValue = 0;

            template<auto Location>
            using PinOf = std::remove_cvref_t<decltype(Location)>;

            // The state machine: StateMachine (PioStateMachine.hpp) loads the program, maps the
            // pins, gives SCK and the lanes the PIO function with this bus's pad drive and slew,
            // makes them outputs, sets the write clock, and checks it all against the program's
            // .side_set. It stays stopped after Startup: every transfer starts it at an entry point.
            // The pin groups stay the same in every phase; SET covers all four lanes so the read
            // entry's `set pindirs` can release them, and `in pins, 1` samples D0.
            struct SmConfig {
                static constexpr auto     clockSpeed    = Config::clockSpeed;
                static constexpr auto     pioInstance   = Config::pioInstance;
                static constexpr auto     smInstance    = Config::smInstance;
                static constexpr unsigned programOffset = programOffsetValue;
                static constexpr double   clockDiv      = writeDiv;
                static constexpr auto sidesetPins = brigand::list<PinOf<Config::sclkPinLocation>>{};
                using Lanes                       = brigand::list<PinOf<Config::d0PinLocation>,
                                                                  PinOf<Config::d1PinLocation>,
                                                                  PinOf<Config::d2PinLocation>,
                                                                  PinOf<Config::d3PinLocation>>;
                static constexpr auto outPins     = Lanes{};
                static constexpr auto setPins     = Lanes{};
                static constexpr auto inPins      = brigand::list<PinOf<Config::d0PinLocation>>{};
                static constexpr auto driveStrength = Config::driveStrength;
                static constexpr bool slewFast      = Config::slewFast;
                static constexpr bool startEnabled  = false;
            };

            using Sm = Kvasir::Pio::StateMachine<Program, SmConfig>;
        };
    }   // namespace detail

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
    /// until idle(). turn() must be polled from the main loop, it detects completion.
    template<typename Clock,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             typename Config_>
    struct PioQspi
      : detail::PioQspiParts<Config_>
      , Kvasir::Pio::StartupOf<typename detail::PioQspiParts<Config_>::Sm> {
        using Parts = detail::PioQspiParts<Config_>;
        using Parts::readDiv;
        using Parts::writeDiv;
        using typename Parts::Config;
        using typename Parts::Program;
        using typename Parts::Sm;

        // A second program at offset 0 of this PIO is a build error (the base's Provides).
        using Claims = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>, typename Sm::Claims>;

        enum class State : std::uint8_t { idle, payload, drain, failed };

        // the chip select is a plain GPIO, driven here
        static constexpr auto initStepPinConfig
          = brigand::append<std::remove_cvref_t<decltype(Sm::initStepPinConfig)>,
                            decltype(list(makeOutputInitHigh(Config::csPinLocation)))>{};

    private:
        static inline State                      state_{State::idle};
        static inline typename Clock::time_point deadline_{};
        static inline std::uint16_t              fillWord_{};

        using SmRegs = typename Sm::SmRegs;
        using Fifo   = typename Sm::Fifo;

        // `set pindirs, 0b1111`: the four lanes outputs again after a read (SET, destination 4 = pindirs)
        static constexpr std::uint16_t setPinDirsAllOutputs = Kvasir::Pio::Enc::set(4, 0x0F);
        static_assert(setPinDirsAllOutputs == 0xE08FU);

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

        /// Point the stopped state machine at an entry point, its FIFOs empty. SM_RESTART leaves the
        /// PC alone, hence the forced jmp after it. The FIFOs are emptied first: a failed or aborted
        /// payload leaves words in the TX FIFO, and with autopull on every non-OUT cycle - the forced
        /// jmp too - refills an exhausted OSR from it (RP2040 datasheet 3.5.4.2, md l.16507; RP2350
        /// 11.5.4.2, md l.45705), so the next frame shifted a stale byte out ahead of its own.
        static void enterPhase(unsigned entry) {
            setEnabled(false);
            clearFifos();
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

            enterPhase(Program::offset("single"));
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
                                ::Kvasir::DMA::ChannelConfig{
                                  .priority       = DmaPriority,
                                  .trigger        = Sm::template txDmaTrigger<Dma>(),
                                  .size           = Size,
                                  .incrementRead  = IncrementSource,
                                  .incrementWrite = false}>(Sm::txFifoAddress, source, count);
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

        /// Absent while a failure is latched, answering otherwise. No lever of its own: the failure stays until the
        /// caller's clearError() (DcsPanel's retry does it), so a PioQspi used alone does not come back by itself -
        /// never a Health::LinkCheck on it (a latched link turns one fault into a reset loop): check the DcsPanel.
        [[nodiscard]] static Kvasir::Link link() {
            return failed() ? Kvasir::Link::absent : Kvasir::Link::answering;
        }

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

            enterPhase(Program::offset("quad"));
            configureWriteShift<8>();
            startPayloadDma<Dma::TransferSize::size8, true>(
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

            enterPhase(Program::offset("quad"));
            configureWriteShift<16>();
            startPayloadDma<Dma::TransferSize::size16, false>(
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

            Sm::template setClockDiv<readDiv>();
            csAssert();

            auto const restore = [] {
                setEnabled(false);
                // The read left the lanes as inputs.
                Sm::exec(setPinDirsAllOutputs);
                csRelease();
                Sm::template setClockDiv<writeDiv>();
            };

            // CO5300 datasheet 5.2.3.
            if(!sendHeader(instruction, cmd, {})) {
                restore();
                state_ = State::failed;
                return false;
            }

            enterPhase(Program::offset("read"));
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

        static void turn() {
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
