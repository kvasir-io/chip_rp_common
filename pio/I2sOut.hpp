#pragma once
#include "chip/rp_common/DMA.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#include "i2sPio/i2s.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <type_traits>

namespace Kvasir { namespace Pio {

    /// Which side of the serial port makes the clocks.
    enum class I2sRole : std::uint8_t {
        /// This chip drives BCLK and LRCK, as a plain I2S DAC wants.
        master,
        /// A codec drives them and this follows.
        slave
    };

    /// I2S transmit from a PIO state machine, fed by DMA from two buffers in turn.
    ///
    /// One frame is one 32-bit word, 16 bits a slot, left slot in the high half: the format
    /// `pico_audio_i2s` uses, which a DMA of 32-bit words puts on the wire unshuffled.
    ///
    /// Config:
    ///   ClockSpeed   (required) clk_sys
    ///   PioInstance  (required) 0..2 -- but not 2, see below
    ///   SmInstance   (required) 0..3
    ///   SampleRate   (required) frames a second. Sets the divider in master mode; in slave
    ///                           mode only the buffer maths uses it.
    ///   Role         (default master)
    ///   BufferFrames (default 512) frames in each of the two buffers (21 ms at 48 kHz).
    ///   ProgramOffset, GpioBase   as PioStateMachine.hpp
    ///
    /// `BclkPin` and `LrckPin` must be consecutive GPIOs, LRCK the higher: master mode side-sets
    /// both with one 2-bit field and slave mode waits on both from one IN base. `DataPin` is
    /// separate and may be anywhere in the same GPIO window.
    ///
    /// Not PIO2: `Kvasir::Pio::getTxDmaTrigger` (PIO.hpp) maps every instance other than 0 to
    /// PIO1's TX DREQ, so a state machine there would be fed from the wrong request.
    template<typename Clock,
             typename BclkPin,
             typename LrckPin,
             typename DataPin,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             typename Config_>
    struct I2sOut {
        /// One frame: the left slot in the high half, the right in the low.
        using Frame = std::uint32_t;

        static constexpr unsigned pinNumberOf(auto pin) {
            return []<int Port, int PinN>(Kvasir::Register::PinLocation<Port, PinN>) {
                return static_cast<unsigned>(PinN);
            }(pin);
        }

        static constexpr unsigned BclkNumber = pinNumberOf(BclkPin{});
        static constexpr unsigned LrckNumber = pinNumberOf(LrckPin{});
        static constexpr unsigned DataNumber = pinNumberOf(DataPin{});

        static_assert(LrckNumber == BclkNumber + 1,
                      "LRCK must be the GPIO immediately above BCLK: master mode side-sets both "
                      "with one two-bit field and slave mode waits on both from one IN base");

        static constexpr I2sRole Role = [] {
            if constexpr(requires { Config_::Role; }) {
                return Config_::Role;
            } else {
                return I2sRole::master;
            }
        }();

        static constexpr std::size_t BufferFrames = [] {
            if constexpr(requires { Config_::BufferFrames; }) {
                return std::size_t{Config_::BufferFrames};
            } else {
                return std::size_t{512};
            }
        }();

        static constexpr bool IsMaster = Role == I2sRole::master;

        static_assert(Config_::PioInstance != 2,
                      "PIO2's TX DREQ is mis-mapped to PIO1 by Kvasir::Pio::getTxDmaTrigger, so a "
                      "state machine there would be fed from the wrong data request. Use PIO0 or "
                      "PIO1, or fix PIO.hpp first");

        // -- the state machine ------------------------------------------------------------------

        /// Two cycles a bit, 32 bits a frame: 64 * the sample rate. Slave mode takes its edges
        /// from outside and only has to keep up, so it runs undivided.
        struct SmConfig : Config_ {
            static constexpr auto ProgramOffset = [] {
                if constexpr(requires { Config_::ProgramOffset; }) {
                    return Config_::ProgramOffset;
                } else {
                    return 0;
                }
            }();

            static constexpr double clockDiv = [] {
                if constexpr(IsMaster) {
                    return double{Config_::ClockSpeed} / (64.0 * double{Config_::SampleRate});
                } else {
                    return 1.0;
                }
            }();

            static constexpr auto outPins = brigand::list<DataPin>{};

            static constexpr auto sidesetPins = [] {
                if constexpr(IsMaster) {
                    return brigand::list<BclkPin, LrckPin>{};
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr auto inPins = [] {
                if constexpr(IsMaster) {
                    return brigand::list<>{};
                } else {
                    return brigand::list<BclkPin, LrckPin>{};
                }
            }();

            // The MSB of a slot leaves first, and a frame is one 32-bit word.
            static constexpr bool        outShiftRight = false;
            static constexpr bool        autopull      = IsMaster;
            static constexpr std::size_t pullThreshold = 32;
            // Nothing is received, so the whole FIFO goes to the transmit side.
            static constexpr bool joinTx = true;
        };

        using Program = std::conditional_t<IsMaster,
                                           Kvasir::Pio::i2s_out_masterProgramm,
                                           Kvasir::Pio::i2s_out_slaveProgramm>;

        using Sm = Kvasir::Pio::StateMachine<Program, SmConfig>;

        /// The frame rate the divider really produces; 0 in slave mode, where the codec decides.
        static constexpr double AchievedSampleRate = [] {
            if constexpr(IsMaster) {
                auto const scaled = static_cast<std::uint32_t>(SmConfig::clockDiv * 256.0 + 0.5);
                return double{Config_::ClockSpeed} / (64.0 * (double(scaled) / 256.0));
            } else {
                return 0.0;
            }
        }();

        static_assert(!IsMaster || (SmConfig::clockDiv >= 1.0 && SmConfig::clockDiv < 65536.0),
                      "SampleRate is not reachable from ClockSpeed with a 16.8 PIO divider");

        // -- startup ----------------------------------------------------------------------------

        using Provides = typename Sm::Provides;
        using Claims   = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>, typename Sm::Claims>;

        static constexpr auto powerClockEnable        = Sm::powerClockEnable;
        static constexpr auto initStepPinConfig       = Sm::initStepPinConfig;
        static constexpr auto initStepPeripheryConfig = Sm::initStepPeripheryConfig;
        static constexpr auto initStepPeripheryEnable = Sm::initStepPeripheryEnable;

        static void preEnableRuntimeInit() { Sm::preEnableRuntimeInit(); }

        static void runtimeInit() {
            Sm::runtimeInit();
            // Left disabled until start() queues the first buffer.
            Sm::setEnabled(false);
        }

        // -- the sample stream ------------------------------------------------------------------

        /// Called from the DMA completion interrupt to refill the buffer that just finished.
        /// Silence if nothing is installed.
        using Fill = void (*)(std::span<Frame>);

        static inline Fill fill{nullptr};

        static inline std::array<std::array<Frame, BufferFrames>, 2> buffers{};

        static inline std::uint32_t underruns{};

        static inline std::uint32_t buffersSent{};

        static inline bool running{false};

        static inline std::uint8_t next{};

        /// Start the clocks and the stream. `f` is called for each buffer as it frees up; it runs
        /// in the DMA interrupt, so it should be a waveform generator and not much else.
        static void start(Fill f) {
            fill        = f;
            underruns   = 0;
            buffersSent = 0;
            for(auto& b : buffers) { b.fill(0); }
            if(fill != nullptr) {
                fill(std::span<Frame>{buffers[0]});
                fill(std::span<Frame>{buffers[1]});
            }
            next    = 0;
            running = true;
            Sm::setEnabled(true);
            send_();
        }

        static void stop() {
            running = false;
            Dma::template abort<DmaChannel>();
            Sm::setEnabled(false);
        }

        [[nodiscard]] static bool isRunning() { return running; }

        /// Frames sent so far.
        [[nodiscard]] static std::uint32_t frames() { return buffersSent * BufferFrames; }

        /// Buffers handed to the DMA before their refill finished, i.e. audible glitches.
        [[nodiscard]] static std::uint32_t underrunCount() { return underruns; }

        /// TX FIFO level: full with the DMA part-way through means the machine is not consuming,
        /// empty with the DMA untouched means it never started.
        [[nodiscard]] static std::uint32_t txLevel() { return Sm::txLevel(); }

        [[nodiscard]] static std::size_t dmaRemaining() {
            return Dma::template remaining<DmaChannel>();
        }

        /// The instruction the state machine sits on; subtract the program offset for the line
        /// in the .pio.
        [[nodiscard]] static std::uint32_t programCounter() {
            using PioRegs = Kvasir::Peripheral::PIO::Registers<Config_::PioInstance>;
            using SmRegs  = typename PioRegs::template SM<Config_::SmInstance>;
            return *reinterpret_cast<std::uint32_t volatile*>(SmRegs::ADDR::Addr::value);
        }

        /// Which state machines of this instance are enabled (CTRL bits 3:0).
        [[nodiscard]] static std::uint32_t enabledMask() {
            using PioRegs = Kvasir::Peripheral::PIO::Registers<Config_::PioInstance>;
            return *reinterpret_cast<std::uint32_t volatile*>(PioRegs::CTRL::Addr::value) & 0xFU;
        }

        static constexpr std::uint32_t Offset = SmConfig::ProgramOffset;

        [[nodiscard]] static std::uint32_t pinCtrl() {
            using PioRegs = Kvasir::Peripheral::PIO::Registers<Config_::PioInstance>;
            using SmRegs  = typename PioRegs::template SM<Config_::SmInstance>;
            return *reinterpret_cast<std::uint32_t volatile*>(SmRegs::PINCTRL::Addr::value);
        }

    private:
        static void send_() {
            auto const which = next;
            next             = static_cast<std::uint8_t>(1U - next);
            Dma::template start<
              DmaChannel,
              DmaPriority,
              Kvasir::Pio::getTxDmaTrigger<Dma, Config_::PioInstance, Config_::SmInstance>(),
              Dma::TransferSize::_32,
              false,   // the FIFO address does not move
              true>    // but the buffer does
              (Sm::txFifoAddress,
               reinterpret_cast<std::uint32_t>(buffers[which].data()),
               BufferFrames,
               [] { onComplete_(); });
        }

        /// Refill the buffer that just finished while the other is on its way. `retrigger` and
        /// not `start`, which would reassign the callback slot this runs out of (see ADC.hpp).
        static void onComplete_() {
            if(!running) { return; }
            ++buffersSent;
            auto const justFinished = static_cast<std::uint8_t>(1U - next);

            Dma::template retrigger<
              DmaChannel,
              DmaPriority,
              Kvasir::Pio::getTxDmaTrigger<Dma, Config_::PioInstance, Config_::SmInstance>(),
              Dma::TransferSize::_32,
              false,
              true>(Sm::txFifoAddress,
                    reinterpret_cast<std::uint32_t>(buffers[next].data()),
                    BufferFrames);
            next = static_cast<std::uint8_t>(1U - next);

            if(fill != nullptr) {
                fill(std::span<Frame>{buffers[justFinished]});
            } else {
                ++underruns;
            }
        }
    };

}}   // namespace Kvasir::Pio
