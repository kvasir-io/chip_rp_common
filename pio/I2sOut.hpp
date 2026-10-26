#pragma once
#include "chip/rp_common/DMA.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/I2sOutProgram.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/StaticFunction.hpp"

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

    namespace detail {
        // I2sOut's configuration, program choice, divider check and state machine, outside the
        // class so that the state machine can be I2sOut's base (StartupOf); I2sOut derives from it
        // too.
        template<typename BclkPin, typename LrckPin, typename DataPin, typename Config_>
        struct I2sOutParts {
            /// One frame: the left slot in the high half, the right in the low.
            using Frame = std::uint32_t;

            static constexpr unsigned bclkNumber
              = static_cast<unsigned>(Kvasir::Io::pinNumber(BclkPin{}));
            static constexpr unsigned lrckNumber
              = static_cast<unsigned>(Kvasir::Io::pinNumber(LrckPin{}));
            static constexpr unsigned dataNumber
              = static_cast<unsigned>(Kvasir::Io::pinNumber(DataPin{}));

            static_assert(
              lrckNumber == bclkNumber + 1,
              "LRCK must be the GPIO immediately above BCLK: master mode side-sets both "
              "with one two-bit field and slave mode waits on both from one IN base");

            static constexpr I2sRole role = KVASIR_CONFIG_OR(Config_, role, I2sRole::master);

            static constexpr std::size_t bufferFrames
              = std::size_t{KVASIR_CONFIG_OR(Config_, bufferFrames, std::size_t{512})};

            static constexpr bool isMaster = role == I2sRole::master;

            // -- the state machine ------------------------------------------------------------------

            /// Two cycles a bit, 32 bits a frame: 64 * the sample rate. Slave mode takes its edges
            /// from outside and only has to keep up, so it runs undivided.
            struct SmConfig : Config_ {
                static constexpr auto programOffset = KVASIR_CONFIG_OR(Config_, programOffset, 0);

                static constexpr double clockDiv = [] {
                    if constexpr(isMaster) {
                        return double{Config_::clockSpeed} / (64.0 * double{Config_::sampleRate});
                    } else {
                        return 1.0;
                    }
                }();

                static constexpr auto outPins = brigand::list<DataPin>{};

                static constexpr auto sidesetPins = [] {
                    if constexpr(isMaster) {
                        return brigand::list<BclkPin, LrckPin>{};
                    } else {
                        return brigand::list<>{};
                    }
                }();

                static constexpr auto inPins = [] {
                    if constexpr(isMaster) {
                        return brigand::list<>{};
                    } else {
                        return brigand::list<BclkPin, LrckPin>{};
                    }
                }();

                // The MSB of a slot leaves first, and a frame is one 32-bit word.
                static constexpr bool        outShiftRight = false;
                static constexpr bool        autopull      = isMaster;
                static constexpr std::size_t pullThreshold = 32;
                // Nothing is received, so the whole FIFO goes to the transmit side.
                static constexpr bool joinTx = true;
            };

            using Program = std::conditional_t<isMaster, I2sOutMasterProgram, I2sOutSlaveProgram>;

            using Sm = Kvasir::Pio::StateMachine<Program, SmConfig>;

            /// The frame rate the divider really produces; 0 in slave mode, where the codec decides.
            /// The divider as the machine is given it (getDiv truncates the fraction, as ClockOut's
            /// AchievedHz): clockSpeed / (64 x (INT + FRAC / 256)), RP2040 datasheet Table 381 (md
            /// l.17560), RP2350 Table 995 (md l.46824). Rounding it instead claimed 47998.46 Hz for
            /// 48 kHz at 125 MHz, where LRCK runs at 48003.07.
            static constexpr double achievedSampleRate = [] {
                if constexpr(isMaster) {
                    auto const [integer, fraction] = Kvasir::Pio::getDiv(SmConfig::clockDiv);
                    return double{Config_::clockSpeed}
                         / (64.0 * (double(integer) + double(fraction) / 256.0));
                } else {
                    return 0.0;
                }
            }();

            static_assert(!isMaster || (SmConfig::clockDiv >= 1.0 && SmConfig::clockDiv < 65536.0),
                          "sampleRate is not reachable from clockSpeed with a 16.8 PIO divider");
        };
    }   // namespace detail

    /// I2S transmit from a PIO state machine, fed by DMA from two buffers in turn.
    ///
    /// One frame is one 32-bit word, 16 bits a slot, left slot in the high half: the format
    /// `pico_audio_i2s` uses, which a DMA of 32-bit words puts on the wire unshuffled.
    ///
    /// Config:
    ///   clockSpeed   (required) clk_sys
    ///   pioInstance  (required) 0..2 (0..1 on the RP2040)
    ///   smInstance   (required) 0..3
    ///   sampleRate   (required) frames a second. Sets the divider in master mode; in slave
    ///                           mode only the buffer maths uses it.
    ///   Role         (default master)
    ///   bufferFrames (default 512) frames in each of the two buffers (21 ms at 48 kHz).
    ///   statistics   (default true) count the buffers sent and the underruns (frames(),
    ///                           underrunCount()); false: no counters in RAM, no counting (and no
    ///                           FDEBUG read) in the DMA interrupt
    ///   programOffset, gpioBase   as PioStateMachine.hpp
    ///
    /// `BclkPin` and `LrckPin` must be consecutive GPIOs, LRCK the higher: master mode side-sets
    /// both with one 2-bit field and slave mode waits on both from one IN base. `DataPin` is
    /// separate and may be anywhere in the same GPIO window.
    template<typename Clock,
             typename BclkPin,
             typename LrckPin,
             typename DataPin,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             typename Config_>
    struct I2sOut
      : detail::I2sOutParts<BclkPin, LrckPin, DataPin, Config_>
      , StartupOf<typename detail::I2sOutParts<BclkPin, LrckPin, DataPin, Config_>::Sm> {
        using Parts = detail::I2sOutParts<BclkPin, LrckPin, DataPin, Config_>;
        using Parts::bufferFrames;
        using typename Parts::Frame;
        using typename Parts::Sm;
        using typename Parts::SmConfig;

        // -- startup ----------------------------------------------------------------------------

        using Claims = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>, typename Sm::Claims>;

        static void runtimeInit() {
            Sm::runtimeInit();
            // Left disabled until start() queues the first buffer.
            Sm::setEnabled(false);
        }

        // -- the sample stream ------------------------------------------------------------------

        /// Called from the DMA completion interrupt to refill the buffer that just finished.
        /// Silence if nothing is installed.
        ///
        /// A StaticFunction, like the I2C and DMA callbacks: it stores a small callable inline,
        /// so the generator may be a lambda capturing the object that owns the waveform; a plain
        /// function pointer converts to one. Sized for a capture of one or two pointers.
        static constexpr std::size_t fillSize = 2 * sizeof(void*);

        using Fill = Kvasir::StaticFunction<void(std::span<Frame>), fillSize>;

        static inline Fill fill{};

        static inline std::array<std::array<Frame, bufferFrames>, 2> buffers{};

        static constexpr bool statistics
          = static_cast<bool>(KVASIR_CONFIG_OR(Config_, statistics, true));

        // The counters exist only with statistics: without, the struct is empty and no
        // variable is emitted at all.
        template<bool On, typename = void>
        struct Counters {
            static inline std::uint32_t underruns{};
            static inline std::uint32_t buffersSent{};
        };

        template<typename Dummy>
        struct Counters<false, Dummy> {};

        using Stats = Counters<statistics>;

        static inline bool running{false};

        static inline std::uint8_t next{};

        /// Start the clocks and the stream, from the first frame of the first buffer, whatever ran before (a stream in
        /// progress is stopped first). `f` is called for each buffer as it frees up; it runs in the DMA interrupt, so
        /// it should be a waveform generator and not much else.
        ///
        /// A clean restart: stop() leaves up to 8 frames in the joined TX FIFO and a part-sent one in the OSR. The
        /// FIFOs are emptied, the machine's state cleared - SM_RESTART counts the OSR as exhausted (RP2040 datasheet
        /// 3.2.3.3 md l.15519, RP2350 11.2.4.2 md l.44512) but keeps X (CTRL, RP2040 Table 368 md l.17316, which keeps
        /// the PC too; RP2350 Table 982 md l.46572) - and the program begun at its first instruction (the master's
        /// seeds X) by a forced jump, which with the FIFO empty cannot autopull a stale word (PioQspi's enterPhase,
        /// RP2040 3.5.4.2 md l.16507). Then the DMA, which fills the FIFO before the machine runs, the stall flag
        /// cleared, the machine enabled: the first frame does not wait, and underrunCount() starts at 0.
        ///
        /// A master begins a new frame at once; a slave begins by finding the frame at the next LRCK high. A slave
        /// that keeps running while its master restarts stays where it was in the old frame and can be out of step
        /// for a while (its per-slot LRCK waits are levels): start() it again after its master.
        static void start(Fill const& f) {
            stop();
            fill.publish(f);
            if constexpr(statistics) {
                Stats::underruns   = 0;
                Stats::buffersSent = 0;
            }
            for(auto& b : buffers) { b.fill(0); }
            if(fill) {
                fill(std::span<Frame>{buffers[0]});
                fill(std::span<Frame>{buffers[1]});
            }
            next = 0;
            Sm::clearFifos();
            Sm::restartAt(0);
            running = true;
            send_();
            Sm::clearTxStall();
            Sm::setEnabled(true);
        }

        static void stop() {
            running = false;
            Dma::template abort<DmaChannel>();
            Sm::setEnabled(false);
        }

        [[nodiscard]] static bool isRunning() { return running; }

        /// Frames sent so far.
        [[nodiscard]] static std::uint32_t frames()
            requires statistics
        {
            return Stats::buffersSent * bufferFrames;
        }

        /// Underruns since start(): buffer ends at which the machine had found its TX FIFO empty since the last one -
        /// a refill (the DMA interrupt held off longer than the 8 frames the FIFO holds) came too late and the line
        /// held a bit, an audible glitch. The DMA interrupt reads and clears FDEBUG.TXSTALL ("stalled on empty TX FIFO
        /// during a blocking PULL, or an OUT with autopull enabled", RP2040 datasheet Table 370 md l.17368, RP2350
        /// Table 984 md l.46615) for it: at most one a buffer, however long the stall. With no Fill installed the
        /// buffers hold silence, which is no underrun.
        [[nodiscard]] static std::uint32_t underrunCount()
            requires statistics
        {
            return Stats::underruns;
        }

        /// TX FIFO level: full with the DMA part-way through means the machine is not consuming,
        /// empty with the DMA untouched means it never started.
        [[nodiscard]] static std::uint32_t txLevel() { return Sm::txLevel(); }

        [[nodiscard]] static std::size_t dmaRemaining() {
            return Dma::template remaining<DmaChannel>();
        }

        /// Sm::pc(), Sm::enabledMask() and Sm::pinCtrl() (PioStateMachine.hpp) are the state
        /// machine's debug reads; subtract Offset from pc() for the index in I2sOutProgram.hpp.
        static constexpr std::uint32_t offset = Sm::Offset;

    private:
        static void send_() {
            auto const which = next;
            next             = static_cast<std::uint8_t>(1U - next);
            Dma::template start<
              DmaChannel,
              ::Kvasir::DMA::ChannelConfig{
                .priority = DmaPriority,
                .trigger  = Rp::Block<Kvasir::Peripheral::PIO::Registers<Config_::pioInstance>>::
                  template dreqTxOf<Config_::smInstance>(),
                .size           = Dma::TransferSize::size32,
                .incrementRead  = true,   // the buffer moves, the FIFO does not
                .incrementWrite = false}>(Sm::txFifoAddress,
                                          reinterpret_cast<std::uint32_t>(buffers[which].data()),
                                          bufferFrames,
                                          [] { onComplete_(); });
        }

        /// Refill the buffer that just finished while the other is on its way. `retrigger` and
        /// not `start`, which would reassign the callback slot this runs out of (see ADC.hpp).
        static void onComplete_() {
            if(!running) { return; }
            auto const justFinished = static_cast<std::uint8_t>(1U - next);

            Dma::template retrigger<
              DmaChannel,
              ::Kvasir::DMA::ChannelConfig{
                .priority = DmaPriority,
                .trigger  = Rp::Block<Kvasir::Peripheral::PIO::Registers<Config_::pioInstance>>::
                  template dreqTxOf<Config_::smInstance>(),
                .size           = Dma::TransferSize::size32,
                .incrementRead  = true,
                .incrementWrite = false}>(Sm::txFifoAddress,
                                          reinterpret_cast<std::uint32_t>(buffers[next].data()),
                                          bufferFrames);
            next = static_cast<std::uint8_t>(1U - next);

            if constexpr(statistics) {
                ++Stats::buffersSent;
                if(Sm::txStalled()) {   // underrunCount()
                    Sm::clearTxStall();
                    ++Stats::underruns;
                }
            }
            if(fill) { fill(std::span<Frame>{buffers[justFinished]}); }
        }
    };

}}   // namespace Kvasir::Pio
