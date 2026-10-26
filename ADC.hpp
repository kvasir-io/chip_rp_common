#pragma once

#include "AdcMath.hpp"
#include "Clocks.hpp"
#include "DMA.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "core/Nvic.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/StaticFunction.hpp"
#include "peripherals/ADC.hpp"

#include <array>
#include <atomic>
#include <bit>
#include <cassert>
#include <cstdint>
#include <optional>
#include <ratio>
#include <span>

namespace Kvasir { namespace ADC {

    namespace Detail {
        template<int Pin>
        constexpr int pinToAdcChannel() {
            if constexpr(Chip::part == Chip::Part::rp2040 || Chip::part == Chip::Part::rp2350a) {
                if constexpr(Pin >= 26 && Pin <= 29) {
                    return Pin - 26;
                } else {
                    return -1;
                }
            } else if constexpr(Chip::part == Chip::Part::rp2350b) {
                if constexpr(Pin >= 40 && Pin <= 47) {
                    return Pin - 40;
                } else {
                    return -1;
                }
            } else {
                return -1;
            }
        }

        constexpr int tempSensorChannel() {
            if constexpr(Chip::part == Chip::Part::rp2040 || Chip::part == Chip::Part::rp2350a) {
                return 4;
            } else {
                return 8;
            }
        }

        template<int Port,
                 int Pin>
        constexpr bool isValidAdcPin(Io::PinLocation<Port,
                                                     Pin>) {
            return Port == 0 && pinToAdcChannel<Pin>() >= 0;
        }

        template<typename... PinLocs>
        constexpr bool allValidAdcPins(brigand::list<PinLocs...>) {
            return (isValidAdcPin(PinLocs{}) && ...);
        }

        constexpr bool allValidAdcPins(brigand::list<>) { return true; }

        template<int Port,
                 int Pin>
        constexpr auto makeAdcPinConfig(Io::PinLocation<Port,
                                                        Pin>) {
            using Pad  = typename Kvasir::Peripheral::PADS_BANK0::Registers<>::GPIO<Pin>::PAD;
            using Ctrl = typename Kvasir::Peripheral::IO_BANK0::Registers<>::GPIO<Pin>::CTRL;
            return list(clear(Pad::ie),
#ifdef KVASIR_CHIP_RP2350
                        clear(Pad::iso),
#endif
                        set(Pad::od),
                        clear(Pad::pue),
                        clear(Pad::pde),
                        write(Ctrl::FUNCSELValC::null));
        }

        template<typename... PinLocs>
        constexpr auto makeAllPinConfigs(brigand::list<PinLocs...>) {
            return list(makeAdcPinConfig(PinLocs{})...);
        }

        constexpr auto makeAllPinConfigs(brigand::list<>) { return brigand::list<>{}; }

        template<int Port,
                 int Pin>
        constexpr std::uint32_t pinToChannelBit(Io::PinLocation<Port,
                                                                Pin>) {
            return 1U << pinToAdcChannel<Pin>();
        }

        template<bool EnableTemp,
                 typename... PinLocs>
        constexpr std::uint32_t computeChannelMask(brigand::list<PinLocs...>) {
            std::uint32_t mask = (0U | ... | pinToChannelBit(PinLocs{}));
            if constexpr(EnableTemp) { mask |= (1U << tempSensorChannel()); }
            return mask;
        }

        template<bool EnableTemp,
                 typename... PinLocs>
        constexpr std::size_t totalChannelCount(brigand::list<PinLocs...>) {
            return sizeof...(PinLocs) + (EnableTemp ? 1 : 0);
        }

        constexpr std::uint8_t firstSetBit(std::uint32_t mask) {
            for(std::uint8_t i = 0; i < 32; ++i) {
                if(mask & (1U << i)) { return i; }
            }
            return 0;
        }

    }   // namespace Detail

    /// The AINSEL channel of an ADC-capable pin (`channelOf(HW::Pin::a0{})`); a pin that
    /// has no channel on this chip variant is a build error.
    template<int Port,
             int Pin>
    constexpr std::uint8_t channelOf(Io::PinLocation<Port,
                                                     Pin>) {
        static_assert(Port == 0 && Detail::pinToAdcChannel<Pin>() >= 0,
                      "not an ADC pin on this chip variant");
        return static_cast<std::uint8_t>(Detail::pinToAdcChannel<Pin>());
    }

    /// The on-die temperature sensor's channel: 4 on the RP2040 and the RP2350A, 8 on the
    /// RP2350B (whose GPIO40..47 take channels 0..7).
    inline constexpr std::uint8_t tempSensorChannel
      = static_cast<std::uint8_t>(Detail::tempSensorChannel());

    template<typename ADCConfig_>
    struct ADCBase {
        struct ADCConfig : ADCConfig_ {
            static constexpr auto userConfigOverride
              = KVASIR_CONFIG_OR(ADCConfig_, userConfigOverride, brigand::list<>{});

            static constexpr bool enableTempSensor
              = KVASIR_CONFIG_OR(ADCConfig_, enableTempSensor, false);

            static constexpr auto callbackFunctionSize
              = KVASIR_CONFIG_OR(ADCConfig_, callbackFunctionSize, 0);

            static constexpr auto maxSampleRateError
              = KVASIR_CONFIG_OR(ADCConfig_, maxSampleRateError, std::ratio<5, 1000>{});

            static constexpr auto pins = ADCConfig_::pins;
        };

        // needed config:
        // pins                 - brigand::list<Io::PinLocation<0, Pin>...>
        // clockSpeed           - ADC clock frequency in Hz (typically 48000000)
        // sampleRate           - desired sample rate in Hz
        // enableTempSensor     - bool (optional, default false)
        // isrPriority          - interrupt priority
        // callbackFunctionSize - StaticFunction capture size (optional, default 0)
        // maxSampleRateError   - std::ratio<Num, Denom> (optional, default 0.5%)
        // userConfigOverride   - (optional)

        using Regs            = Kvasir::Peripheral::ADC::Registers<0>;
        using InterruptIndexs = typename Rp::Block<Regs>::Irqs;

        // Startup: the ADC, and the clock the sample divider is computed from (clk_adc,
        // 48 MHz from the USB PLL).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Clocks::ClkAdc, ADCConfig::clockSpeed>;

        static constexpr auto dmaTrigger = Rp::Block<Regs>::dreqRx;

        static_assert(Detail::allValidAdcPins(ADCConfig::pins),
                      "one or more pins are not valid ADC pins for this chip variant");

        static constexpr std::uint32_t channelMask_
          = Detail::computeChannelMask<ADCConfig::enableTempSensor>(ADCConfig::pins);
        static constexpr std::size_t numChannels_
          = Detail::totalChannelCount<ADCConfig::enableTempSensor>(ADCConfig::pins);
        static constexpr std::uint8_t firstChannel_  = Detail::firstSetBit(channelMask_);
        static constexpr bool         useRoundRobin_ = numChannels_ > 1;

        static_assert(numChannels_ > 0,
                      "at least one ADC channel (pin or temp sensor) must be enabled");

        static constexpr auto divRegs_
          = Detail::calcDivRegs(ADCConfig::clockSpeed, ADCConfig::sampleRate);

        static_assert(
          Detail::isValidSampleRateConfig<ADCConfig::clockSpeed,
                                          ADCConfig::sampleRate>(ADCConfig::maxSampleRateError),
          "sample rate error exceeds maxSampleRateError");

        static constexpr auto channelTable_ = Detail::buildChannelTable<numChannels_>(channelMask_);

        /// What a free-running mode's channel-attribution check found (AdcMath.hpp).
        using Alignment = Detail::Alignment;

        /// The AINSEL channel of sample `i` of a stream whose sample 0 is in channel-table slot
        /// `firstSlot`.
        [[nodiscard]] static constexpr std::uint8_t channelAt(std::size_t firstSlot,
                                                              std::size_t i) {
            return channelTable_.at(firstSlot, i);
        }

        /// Drops since boot - a buffer (DMA mode) or a drained batch (ISR mode) - because the FIFO
        /// over- or underran (FCS.OVER / FCS.UNDER): the reader was held off for longer than the FIFO's 8
        /// entries (RP2040 datasheet 4.9.2.4, RP2350 12.4.3.4).
        [[nodiscard]] static std::uint32_t overruns() {
            return overruns_.load(std::memory_order_relaxed);
        }

        /// Dropped since boot because AINSEL disagreed with the sample count: a sample gained or
        /// lost that no FIFO flag reported.
        [[nodiscard]] static std::uint32_t misalignments() {
            return misalignments_.load(std::memory_order_relaxed);
        }

    protected:
        // Written only by the one interrupt that reads the stream (load + store: no
        // read-modify-write call on the M0+). One ADC, one owner (Hal::Owns): one pair.
        inline static std::atomic<std::uint32_t> overruns_{0};
        inline static std::atomic<std::uint32_t> misalignments_{0};

        static void countDrop(Alignment a) {
            auto& counter = a == Alignment::overrun ? overruns_ : misalignments_;
            counter.store(counter.load(std::memory_order_relaxed) + 1U, std::memory_order_relaxed);
        }

        /// Clear START_MANY and wait out a conversion in flight (waitConversionsDone()).
        static void haltConversions() {
            apply(clear(Regs::CS::start_many));
            static_cast<void>(waitConversionsDone());
        }

    public:
        static constexpr auto powerClockEnable  = list(clear(Rp::Block<Regs>::reset));
        static constexpr auto initStepPinConfig = Detail::makeAllPinConfigs(ADCConfig::pins);

        static constexpr auto initStepPeripheryConfig = list(
          Regs::CS::overrideDefaults(
            write(Regs::CS::en, Register::value<1U>()),
            write(Regs::CS::ts_en, Register::value<ADCConfig::enableTempSensor ? 1U : 0U>()),
            write(Regs::CS::ainsel, Register::value<static_cast<std::uint32_t>(firstChannel_)>()),
            write(Regs::CS::rrobin, Register::value<useRoundRobin_ ? channelMask_ : 0U>())),
          Regs::DIV::overrideDefaults(
            write(Regs::DIV::_int, Register::value<static_cast<std::uint32_t>(divRegs_.int_)>()),
            write(Regs::DIV::frac, Register::value<static_cast<std::uint32_t>(divRegs_.frac)>())),
          ADCConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<ADCConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        // Bounds for the hardware-flag waits below. A conversion takes 96 ADC clocks, 2 us at
        // 48 MHz (RP2040 datasheet 4.9.2, RP2350 12.4.3), and the FIFO holds 8 results
        // (RP2040 4.9.2.4, RP2350 12.4.3.4), so hitting either limit means the ADC is
        // disabled or wedged - give up rather than hang the caller.
        static constexpr int readyPollLimit = 100000;
        static constexpr int fifoDrainLimit = 64;

        /// ADC clocks per conversion (RP2040 datasheet 4.9.2, RP2350 12.4.3).
        static constexpr std::uint32_t conversionClocks = 96;

        /// READY reads in a row that span one whole conversion: every read takes at least one
        /// system clock, and the system clock is at most the chip's ceiling.
        static constexpr int readyStableReads = static_cast<int>(
          conversionClocks
          * ((Rp::ChipTraits::cpuHzCeiling + ADCConfig::clockSpeed - 1) / ADCConfig::clockSpeed));

        static_assert(readyStableReads < readyPollLimit / 4,
                      "the stable-READY wait must fit the poll bound several times over");

        /// After START_MANY was cleared: wait until no conversion is in flight. CS.READY is 0
        /// while one runs (RP2040 datasheet Table 568, RP2350 Table 1121), and both datasheets
        /// say to poll it before draining the FIFO (RP2040 4.9.2.5, RP2350 12.4.3.5). One read
        /// of 1 is not proof: a conversion the pacer started in the very cycles START_MANY was
        /// cleared may not have dropped READY yet. It would complete after the FIFO was
        /// drained and AINSEL reset - an extra first sample, and AINSEL one past the first
        /// channel. So READY must read 1 for ReadyStableReads reads in a row, one conversion's
        /// worth. Bounded by ReadyPollLimit; false when it never settled.
        static bool waitConversionsDone() {
            int stable = 0;
            for(int i = 0; i < readyPollLimit; ++i) {
                stable = get<0>(apply(read(Regs::CS::ready))) ? stable + 1 : 0;
                if(stable >= readyStableReads) { return true; }
            }
            return false;
        }

        static void drainFifo() {
            for(int i = 0; i < fifoDrainLimit; ++i) {
                if(get<0>(apply(read(Regs::FCS::empty)))) { break; }
                apply(read(Regs::FIFO::val));
            }
        }

        /// Start a single conversion of `channel` and poll CS.READY for it, at most
        /// ReadyPollLimit times. False means the poll was exhausted: RESULT is stale.
        [[nodiscard]] static bool startOnceAndWait(std::uint8_t channel) {
            apply(write(Regs::CS::ainsel, static_cast<std::uint32_t>(channel)));
            apply(set(Regs::CS::start_once));
            for(int i = 0; i < readyPollLimit; ++i) {
                if(get<0>(apply(read(Regs::CS::ready)))) { return true; }
            }
            return false;
        }

        /// One conversion of `channel`. Returns whatever RESULT holds after the bounded
        /// READY poll, stale if the poll was exhausted; tryReadOnce() tells the two apart.
        static std::uint16_t readOnce(std::uint8_t channel) {
            static_cast<void>(startOnceAndWait(channel));
            return static_cast<std::uint16_t>(get<0>(apply(read(Regs::RESULT::result))));
        }

        /// readOnce() that reports an exhausted READY poll (ADC disabled or wedged) as
        /// nullopt instead of handing back the stale RESULT.
        [[nodiscard]] static std::optional<std::uint16_t> tryReadOnce(std::uint8_t channel) {
            if(!startOnceAndWait(channel)) { return std::nullopt; }
            return static_cast<std::uint16_t>(get<0>(apply(read(Regs::RESULT::result))));
        }

        /// readOnce() of a channel the config enables (a pin in `pins`, or the sensor with
        /// `enableTempSensor`): any other channel is a build error.
        ///     Adc::readOnce<Kvasir::ADC::channelOf(HW::Pin::a0{})>()
        ///     Adc::readOnce<Kvasir::ADC::TempSensorChannel>()
        template<std::uint8_t Channel>
        static std::uint16_t readOnce() {
            static_assert((channelMask_ & (1U << Channel)) != 0,
                          "channel is not in this ADC's config (pins / enableTempSensor)");
            return readOnce(Channel);
        }

        /// tryReadOnce() of a channel the config enables; see readOnce<Channel>().
        template<std::uint8_t Channel>
        [[nodiscard]] static std::optional<std::uint16_t> tryReadOnce() {
            static_assert((channelMask_ & (1U << Channel)) != 0,
                          "channel is not in this ADC's config (pins / enableTempSensor)");
            return tryReadOnce(Channel);
        }
    };

    /// Free-running sampling into two ping-pong DMA buffers. When one is full the callback
    /// gets it, with the channel-table slot of its first sample, while the DMA fills the other:
    /// sample i is on channel `channelAt(firstSlot, i)` (channelTable_.channels[(firstSlot + i)
    /// % numChannels_]). A buffer does not start at the first channel unless BufferSize is a
    /// multiple of the channel count.
    ///
    /// The FIFO carries no channel number (RP2040 datasheet Table 571, RP2350 Table 1124), so
    /// the attribution is a count, and one sample lost or gained moves every channel onto its
    /// neighbour for good - a die temperature read from a 2.5 V input. So the count is checked
    /// against the hardware at the end of every buffer, right after the DMA was re-armed:
    /// AINSEL must name the channel the count says comes next (checkAlignment()). A buffer that fails - or that saw a FIFO overrun or underrun - is
    /// never handed to the callback: the stream is halted, realigned on the first channel and
    /// re-armed, and the event counted (overruns(), misalignments()). The callback only ever
    /// sees buffers whose attribution the hardware confirmed.
    template<typename ADCConfig_,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize>
    struct ADCDmaBehavior : ADCBase<ADCConfig_> {
        using base = ADCBase<ADCConfig_>;
        using Regs = typename base::Regs;

        static_assert(BufferSize > 0,
                      "a DMA buffer holds at least one sample");

        using Claims = brigand::append<typename base::Claims, Kvasir::DMA::Claims<Dma, DmaChannel>>;

        /// (completed buffer, channel-table slot of its first sample)
        using DmaCallback_t
          = Kvasir::StaticFunction<void(std::span<std::uint16_t>, std::size_t firstSlot),
                                   base::ADCConfig::callbackFunctionSize>;

        inline static std::array<std::uint16_t, BufferSize> bufferA_{};
        inline static std::array<std::uint16_t, BufferSize> bufferB_{};
        inline static bool                                  activeIsA_{true};
        inline static std::atomic<bool>                     running_{false};
        inline static DmaCallback_t                         userCallback_{};
        // Channel-table slot of the first sample of the buffer being filled.
        inline static std::size_t slot_{0};
        using base::channelAt;
        using typename base::Alignment;

        template<typename F>
        static void start(F&& callback) {
            assert(!running_.load(std::memory_order_relaxed));
            userCallback_.publish(std::forward<F>(callback));
            arm();
        }

        static void stop() {
            halt();
            userCallback_.reset();
        }

        /// Realign on the first channel without reassigning the callback: halts and re-arms.
        /// The driver does this by itself for every buffer it does not hand over; a caller
        /// needs it only to start the count over at a moment of its own choosing.
        static void restart() {
            halt();
            arm();
        }

    private:
        static void arm() {
            activeIsA_ = true;
            slot_      = 0;
            running_.store(true, std::memory_order_relaxed);

            // Configure FIFO for DMA
            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<1U>()),
                                              write(Regs::FCS::dreq_en, Register::value<1U>()),
                                              write(Regs::FCS::thresh, Register::value<1U>())));

            base::drainFifo();

            // write-1-to-clear and survive halt(): left set, every buffer would be an overrun.
            apply(set(Regs::FCS::over));
            apply(set(Regs::FCS::under));

            // Reset AINSEL to first channel
            apply(write(Regs::CS::ainsel, static_cast<std::uint32_t>(base::firstChannel_)));

            // Start first DMA transfer to buffer A
            Dma::template start<DmaChannel,
                                ::Kvasir::DMA::ChannelConfig{.priority = DmaPriority,
                                                             .trigger  = base::dmaTrigger,
                                                             .size     = Dma::TransferSize::size16,
                                                             .incrementRead  = false,
                                                             .incrementWrite = true}>(
              reinterpret_cast<std::uint32_t>(bufferA_.data()),
              Regs::FIFO::Addr::value,
              BufferSize,
              []() { dmaComplete(); });

            // Start free-running ADC
            apply(set(Regs::CS::start_many));
        }

        static void halt() {
            running_.store(false, std::memory_order_relaxed);
            // Stop the DMA before the ADC: clearing running_ alone leaves the
            // channel armed for the rest of its transfers, so the next arm()
            // would re-arm a live channel. See DMA::abort.
            Dma::template abort<DmaChannel>();
            base::haltConversions();

            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<0U>()),
                                              write(Regs::FCS::dreq_en, Register::value<1U>())));

            base::drainFifo();
        }

        /// At the end of a buffer, right after the DMA was re-armed on the other one
        /// (Detail::checkAttribution, AdcMath.hpp, has the datasheet reasoning). Since the last
        /// realign the DMA has moved slot_ + BufferSize samples (modulo the channel count) into
        /// the buffer just completed, BufferSize - TRANS_COUNT more into the new one (TRANS_COUNT
        /// reads the transfers left, live: RP2040 datasheet 2.5.1.2, RP2350 12.6.2.2), and the
        /// FIFO holds LEVEL more. A snapshot holds still when TRANS_COUNT and LEVEL read the same
        /// before and after AINSEL. Re-arming first keeps the FIFO's 8 entries (16 us at
        /// 500 kS/s) for the interrupt's entry alone: checked before the re-arm, the first buffer
        /// after a start overran on an RP2040 (a cold XIP cache in the handler).
        [[gnu::always_inline]] static Detail::AttributionSnapshot snapshot() {
            auto const left1 = Dma::template remaining<DmaChannel>();
            auto const fcs
              = apply(read(Regs::FCS::level), read(Regs::FCS::over), read(Regs::FCS::under));
            auto const ainsel = get<0>(apply(read(Regs::CS::ainsel)));
            auto const level2 = get<0>(apply(read(Regs::FCS::level)));
            auto const left2  = Dma::template remaining<DmaChannel>();
            auto const level  = get<0>(fcs);
            return Detail::AttributionSnapshot{.flagged = get<1>(fcs) != 0 || get<2>(fcs) != 0,
                                               .stable  = level == level2 && left1 == left2
                                                       && left1 <= BufferSize,
                                               .moved   = BufferSize + (BufferSize - left1) + level,
                                               .ainsel  = static_cast<std::uint32_t>(ainsel)};
        }

        /// The rest of checkAlignment() after a first snapshot that did not settle it (judged
        /// `first`). True: aligned. False: the buffer is dropped, counted, the stream halted and
        /// re-armed.
        [[gnu::noinline,
          gnu::cold]] static bool
        confirmOrRealign(Detail::Look first) {
            auto const check = Detail::checkAttributionAfter(base::channelTable_, slot_, first, [] {
                return snapshot();
            });
            if(check == Alignment::aligned) { return true; }
            base::countDrop(check);
            // halt() aborts the channel, arm() starts it with this same callback: the slot
            // being invoked is re-published, which StaticFunction allows (its invoker runs
            // on a copy of the callable).
            halt();
            arm();
            return false;
        }

        /// The first snapshot inline (it decides nearly every buffer), the rest out of line.
        [[nodiscard]] static bool checkAlignment() {
            auto const s = snapshot();
            auto const look
              = Detail::judge<base::numChannels_>(s, base::channelTable_.at(slot_, s.moved));
            if(look == Detail::Look::aligned) [[likely]] { return true; }
            return confirmOrRealign(look);
        }

        static void dmaComplete() {
            if(!running_.load(std::memory_order_relaxed)) { return; }

            auto const completed = activeIsA_ ? std::span{bufferA_} : std::span{bufferB_};
            activeIsA_           = !activeIsA_;
            auto& next           = activeIsA_ ? bufferA_ : bufferB_;

            // First, while the FIFO still has room. retrigger(), not start(): start() would
            // assign the slot onIsr() is invoking.
            Dma::template retrigger<DmaChannel,
                                    ::Kvasir::DMA::ChannelConfig{.priority = DmaPriority,
                                                                 .trigger  = base::dmaTrigger,
                                                                 .size = Dma::TransferSize::size16,
                                                                 .incrementRead  = false,
                                                                 .incrementWrite = true}>(
              reinterpret_cast<std::uint32_t>(next.data()),
              Regs::FIFO::Addr::value,
              BufferSize);

            if(!checkAlignment()) { return; }

            auto const firstSlot = slot_;
            slot_ = Detail::ChannelTable<base::numChannels_>::template advance<BufferSize>(slot_);
            if(userCallback_) { userCallback_(completed, firstSlot); }
        }
    };

    /// Free-running sampling, read by the ADC interrupt: the callback gets every sample with its
    /// channel, `(channel, value)`.
    ///
    /// The attribution is a count, as in ADCDmaBehavior (the FIFO carries no channel number), so
    /// it is checked the same way (Detail::checkAttribution, AdcMath.hpp): each interrupt drains
    /// at most the FIFO's 8 entries into a local batch, then AINSEL must name the channel of
    /// "delivered + drained + FIFO LEVEL" samples. Only a batch that passed is delivered - drained
    /// samples are held until then, so a misattributed sample never reaches the callback. A batch
    /// that fails, or a FIFO overrun or underrun (FCS.OVER / UNDER), drops the batch, halts,
    /// realigns on the first channel and restarts, and is counted (overruns(),
    /// misalignments()). The interrupt is level-triggered - asserted while LEVEL >= THRESH
    /// (RP2040 datasheet Table 570, 4.9.2.6; RP2350 Table 1123, 12.4.3.6) - so samples left in
    /// the FIFO after a batch raise it again at once.
    template<typename ADCConfig_>
    struct ADCIsrBehavior : ADCBase<ADCConfig_> {
        using base = ADCBase<ADCConfig_>;
        using Regs = typename base::Regs;
        using base::channelAt;
        using typename base::Alignment;

        using IsrCallback_t = Kvasir::StaticFunction<void(std::uint8_t, std::uint16_t),
                                                     base::ADCConfig::callbackFunctionSize>;

        /// The FIFO's depth (RP2040 datasheet 4.9.2.4, RP2350 12.4.3.4): one interrupt's batch.
        static constexpr std::size_t batchSize = 8;

        // Channel-table slot of the next sample the FIFO hands out.
        inline static std::size_t   slot_{0};
        inline static IsrCallback_t userCallback_{};

        template<typename F>
        static void start(F&& callback) {
            userCallback_.publish(std::forward<F>(callback));

            // A conversion still in flight (a START_ONCE, a stream not stopped) would complete
            // after AINSEL was written and step it past the first channel.
            base::haltConversions();

            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<1U>()),
                                              write(Regs::FCS::thresh, Register::value<1U>())));
            realignStopped();

            // slot_ and userCallback_ are plain memory the ISR reads: keep their stores before
            // the volatile writes that let it run
            std::atomic_signal_fence(std::memory_order_seq_cst);

            // Enable FIFO interrupt
            apply(set(Regs::INTE::fifo));
            apply(Nvic::makeEnable(typename base::InterruptIndexs{}));

            // Start free-running ADC
            apply(set(Regs::CS::start_many));
        }

        static void stop() {
            base::haltConversions();

            apply(clear(Regs::INTE::fifo));

            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<0U>())));

            base::drainFifo();
            userCallback_.reset();
        }

        static void onIsr() {
            // At most one FIFO's worth a run: LEVEL only grows while it is read. A built-in array:
            // std::array's operator[] is a call in the sanitize images.
            std::uint16_t batch[batchSize];
            auto const    level = static_cast<std::size_t>(get<0>(apply(read(Regs::FCS::level))));
            std::size_t const drained = level < batchSize ? level : batchSize;
            for(std::size_t i = 0; i < drained; ++i) {
                batch[i] = static_cast<std::uint16_t>(get<0>(apply(read(Regs::FIFO::val))));
            }

            // The check's first snapshot inline; it decides nearly every run. Anything else - a
            // flag, a count that moved, a mismatch to confirm - is the out-of-line rest.
            auto const first = slot_;
            auto const s     = snapshot(drained);
            if(auto const look = Detail::judge<n>(s, base::channelTable_.near(first, s.moved));
               look != Detail::Look::aligned) [[unlikely]]
            {
                if(!confirmOrRealign(look, drained)) {
                    return;
                }   // dropped: the channels are unknown
            }

            if constexpr(n > 1) { slot_ = base::channelTable_.slotAfter(first, drained); }
            for(std::size_t i = 0; i < drained; ++i) {
                if(userCallback_) { userCallback_(base::channelTable_.near(first, i), batch[i]); }
            }
        }

        using Isr = Nvic::IsrsFor<std::addressof(onIsr), typename base::InterruptIndexs>;

    private:
        static constexpr std::size_t n = base::numChannels_;

        static_assert(batchSize == Detail::fifoDepth);
        static_assert((Regs::FCS::level.mask >> std::countr_zero(Regs::FCS::level.mask))
                        == Detail::levelFieldMax,
                      "near()/slotAfter() reach a batch plus the largest LEVEL");

        /// One look for Detail::checkAttribution after `drained` samples were drained: FCS (LEVEL, OVER,
        /// UNDER: one load), AINSEL, LEVEL again - stable when both LEVELs agree. One channel
        /// needs only the flags (no round robin: checkAttribution asks nothing else).
        [[gnu::always_inline]] static Detail::AttributionSnapshot snapshot(std::size_t drained) {
            auto const fcs
              = apply(read(Regs::FCS::level), read(Regs::FCS::over), read(Regs::FCS::under));
            bool const flagged = get<1>(fcs) != 0 || get<2>(fcs) != 0;
            if constexpr(n == 1) {
                return {.flagged = flagged, .stable = true, .moved = drained, .ainsel = 0};
            } else {
                auto const ainsel = get<0>(apply(read(Regs::CS::ainsel)));
                auto const level2 = get<0>(apply(read(Regs::FCS::level)));
                return {.flagged = flagged,
                        .stable  = get<0>(fcs) == level2,
                        .moved   = drained + static_cast<std::size_t>(get<0>(fcs)),
                        .ainsel  = static_cast<std::uint32_t>(ainsel)};
            }
        }

        /// The rest of the check after a first snapshot that did not settle it, judged `first`
        /// (up to 16 snapshots in all, a mismatch confirmed twice, a flag ends it). True: aligned,
        /// deliver. False: the batch is dropped, counted, and the stream halted, realigned and
        /// restarted.
        [[gnu::noinline,
          gnu::cold]] static bool
        confirmOrRealign(Detail::Look first,
                         std::size_t  drained) {
            auto const check
              = Detail::checkAttributionAfter(base::channelTable_, slot_, first, [drained] {
                    return snapshot(drained);
                });
            if(check == Alignment::aligned) { return true; }
            base::countDrop(check);
            base::haltConversions();
            realignStopped();
            apply(set(Regs::CS::start_many));
            return false;
        }

        /// With no conversion in flight: empty the FIFO, clear its sticky flags, AINSEL to the
        /// first channel, the count to slot 0.
        static void realignStopped() {
            base::drainFifo();
            // write-1-to-clear (Table 570 / 1123); left set, every batch would be an overrun.
            apply(set(Regs::FCS::over), set(Regs::FCS::under));
            apply(write(Regs::CS::ainsel, static_cast<std::uint32_t>(base::firstChannel_)));
            slot_ = 0;
        }
    };
}}   // namespace Kvasir::ADC
