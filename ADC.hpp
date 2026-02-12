#pragma once

#include "DMA.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Util/StaticFunction.hpp"
#include "peripherals/ADC.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <ratio>
#include <span>

namespace Kvasir { namespace ADC {

    namespace Detail {
        template<int Pin>
        constexpr int pinToAdcChannel() {
            if constexpr(PinConfig::CurrentChip == PinConfig::ChipVariant::RP2040
                         || PinConfig::CurrentChip == PinConfig::ChipVariant::RP2350A)
            {
                if constexpr(Pin >= 26 && Pin <= 29) {
                    return Pin - 26;
                } else {
                    return -1;
                }
            } else if constexpr(PinConfig::CurrentChip == PinConfig::ChipVariant::RP2350B) {
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
            if constexpr(PinConfig::CurrentChip == PinConfig::ChipVariant::RP2040
                         || PinConfig::CurrentChip == PinConfig::ChipVariant::RP2350A)
            {
                return 4;
            } else {
                return 8;
            }
        }

        template<int Port,
                 int Pin>
        constexpr bool isValidAdcPin(Register::PinLocation<Port,
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
        constexpr auto makeAdcPinConfig(Register::PinLocation<Port,
                                                              Pin>) {
            using Pad  = typename Kvasir::Peripheral::PADS_BANK0::Registers<>::GPIO<Pin>::PAD;
            using Ctrl = typename Kvasir::Peripheral::IO_BANK0::Registers<>::GPIO<Pin>::CTRL;
            return list(clear(Pad::ie),
#if __has_include("chip/rp2350.hpp")
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
        constexpr std::uint32_t pinToChannelBit(Register::PinLocation<Port,
                                                                      Pin>) {
            return 1u << pinToAdcChannel<Pin>();
        }

        template<bool EnableTemp,
                 typename... PinLocs>
        constexpr std::uint32_t computeChannelMask(brigand::list<PinLocs...>) {
            std::uint32_t mask = (0u | ... | pinToChannelBit(PinLocs{}));
            if constexpr(EnableTemp) { mask |= (1u << tempSensorChannel()); }
            return mask;
        }

        template<bool EnableTemp,
                 typename... PinLocs>
        constexpr std::size_t totalChannelCount(brigand::list<PinLocs...>) {
            return sizeof...(PinLocs) + (EnableTemp ? 1 : 0);
        }

        constexpr std::uint8_t firstSetBit(std::uint32_t mask) {
            for(std::uint8_t i = 0; i < 32; ++i) {
                if(mask & (1u << i)) { return i; }
            }
            return 0;
        }

        template<std::size_t N>
        struct ChannelTable {
            std::array<std::uint8_t, N> channels{};
        };

        template<std::size_t N>
        constexpr ChannelTable<N> buildChannelTable(std::uint32_t mask) {
            ChannelTable<N> t{};
            std::size_t     idx = 0;
            for(std::uint8_t i = 0; i < 9 && idx < N; ++i) {
                if(mask & (1u << i)) { t.channels[idx++] = i; }
            }
            return t;
        }

        // ADC sample rate divider calculation
        // Total period in ADC clocks = max(96, 1 + INT + FRAC/256)
        // sample_rate = clockSpeed / period
        // When DIV register is 0 the ADC runs at its maximum rate (96 cycles per conversion).

        struct DivRegs {
            std::uint16_t int_;
            std::uint8_t  frac;
        };

        static constexpr DivRegs calcDivRegs(std::uint32_t clockSpeed,
                                             std::uint32_t sampleRate) {
            // target period in ADC clocks (fixed-point: 8 fractional bits)
            // period_fp = clockSpeed * 256 / sampleRate
            std::uint64_t period_fp = (static_cast<std::uint64_t>(clockSpeed) * 256u) / sampleRate;

            // subtract 1 (in fixed-point: 256) to get the divider value
            // DIV register encodes: period = 1 + INT + FRAC/256
            if(period_fp <= 256u) {
                // sampleRate >= clockSpeed, use DIV=0 (max rate)
                return {0, 0};
            }
            std::uint64_t div_fp = period_fp - 256u;

            std::uint32_t int_part  = static_cast<std::uint32_t>(div_fp >> 8);
            std::uint8_t  frac_part = static_cast<std::uint8_t>(div_fp & 0xFFu);

            // clamp INT to 16-bit max
            if(int_part > 0xFFFFu) {
                int_part  = 0xFFFFu;
                frac_part = 0xFFu;
            }

            return {static_cast<std::uint16_t>(int_part), frac_part};
        }

        static constexpr double calcActualSampleRate(std::uint32_t clockSpeed,
                                                     DivRegs       div) {
            if(div.int_ == 0 && div.frac == 0) {
                // DIV=0: 96 cycles per conversion
                return double(clockSpeed) / 96.0;
            }
            double period = 1.0 + double(div.int_) + double(div.frac) / 256.0;
            // actual period is max(96, period)
            if(period < 96.0) { period = 96.0; }
            return double(clockSpeed) / period;
        }

        template<std::uint32_t ClockSpeed,
                 std::uint32_t SampleRate,
                 std::intmax_t Num,
                 std::intmax_t Denom>
        static constexpr bool isValidSampleRateConfig(std::ratio<Num,
                                                                 Denom>) {
            constexpr auto div    = calcDivRegs(ClockSpeed, SampleRate);
            constexpr auto actual = calcActualSampleRate(ClockSpeed, div);
            constexpr auto err    = actual - double(SampleRate);
            constexpr auto absErr = err > 0.0 ? err : -err;
            constexpr auto maxErr = double(SampleRate) * (double(Num) / double(Denom));
            return absErr <= maxErr;
        }

    }   // namespace Detail

    namespace Traits { namespace ADC {
        static constexpr auto getIsrIndexs() {
            return brigand::list<decltype(Kvasir::Interrupt::adc_fifo)>{};
        }

        static constexpr auto getEnable() {
            return clear(Peripheral::RESETS::Registers<>::RESET::adc);
        }

        static constexpr auto DmaTrigger() { return DMA::TriggerSource::adc; }
    }}   // namespace Traits::ADC

    template<typename ADCConfig_>
    struct ADCBase {
        struct ADCConfig : ADCConfig_ {
            static constexpr auto userConfigOverride = [] {
                if constexpr(requires { ADCConfig_::userConfigOverride; }) {
                    return ADCConfig_::userConfigOverride;
                } else {
                    return brigand::list<>{};
                }
            }();

            static constexpr bool enableTempSensor = [] {
                if constexpr(requires { ADCConfig_::enableTempSensor; }) {
                    return ADCConfig_::enableTempSensor;
                } else {
                    return false;
                }
            }();

            static constexpr auto callbackFunctionSize = [] {
                if constexpr(requires { ADCConfig_::callbackFunctionSize; }) {
                    return ADCConfig_::callbackFunctionSize;
                } else {
                    return 0;
                }
            }();

            static constexpr auto maxSampleRateError = [] {
                if constexpr(requires { ADCConfig_::maxSampleRateError; }) {
                    return ADCConfig_::maxSampleRateError;
                } else {
                    return std::ratio<5, 1000>{};
                }
            }();

            static constexpr auto pins = ADCConfig_::pins;
        };

        // needed config:
        // pins                 - brigand::list<Register::PinLocation<0, Pin>...>
        // clockSpeed           - ADC clock frequency in Hz (typically 48000000)
        // sampleRate           - desired sample rate in Hz
        // enableTempSensor     - bool (optional, default false)
        // isrPriority          - interrupt priority
        // callbackFunctionSize - StaticFunction capture size (optional, default 0)
        // maxSampleRateError   - std::ratio<Num, Denom> (optional, default 0.5%)
        // userConfigOverride   - (optional)

        using Regs            = Kvasir::Peripheral::ADC::Registers<0>;
        using InterruptIndexs = decltype(Traits::ADC::getIsrIndexs());

        static constexpr auto DmaTrigger = Traits::ADC::DmaTrigger();

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

        static constexpr auto powerClockEnable  = list(Traits::ADC::getEnable());
        static constexpr auto initStepPinConfig = Detail::makeAllPinConfigs(ADCConfig::pins);

        static constexpr auto initStepPeripheryConfig = list(
          Regs::CS::overrideDefaults(
            write(Regs::CS::en, Register::value<1u>()),
            write(Regs::CS::ts_en, Register::value<ADCConfig::enableTempSensor ? 1u : 0u>()),
            write(Regs::CS::ainsel, Register::value<static_cast<std::uint32_t>(firstChannel_)>()),
            write(Regs::CS::rrobin, Register::value<useRoundRobin_ ? channelMask_ : 0u>())),
          Regs::DIV::overrideDefaults(
            write(Regs::DIV::_int, Register::value<static_cast<std::uint32_t>(divRegs_.int_)>()),
            write(Regs::DIV::frac, Register::value<static_cast<std::uint32_t>(divRegs_.frac)>())),
          ADCConfig::userConfigOverride);

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<ADCConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static void drainFifo() {
            while(!get<0>(apply(read(Regs::FCS::empty)))) { apply(read(Regs::FIFO::val)); }
        }

        static std::uint16_t readOnce(std::uint8_t channel) {
            apply(write(Regs::CS::ainsel, static_cast<std::uint32_t>(channel)));
            apply(set(Regs::CS::start_once));
            while(!get<0>(apply(read(Regs::CS::ready)))) {}
            return static_cast<std::uint16_t>(get<0>(apply(read(Regs::RESULT::result))));
        }
    };

    // DMA double-buffer behavior
    // Continuously samples ADC into two ping-pong buffers.
    // When one buffer fills, the callback is called with a span to the completed buffer
    // while DMA fills the other.
    template<typename ADCConfig_,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             std::size_t            BufferSize>
    struct ADCDmaBehavior : ADCBase<ADCConfig_> {
        using base = ADCBase<ADCConfig_>;
        using Regs = typename base::Regs;

        static_assert(Dma::numberOfChannels > std::size_t(DmaChannel));

        using DmaCallback_t = Kvasir::StaticFunction<void(std::span<std::uint16_t>),
                                                     base::ADCConfig::callbackFunctionSize>;

        inline static std::array<std::uint16_t, BufferSize> bufferA_{};
        inline static std::array<std::uint16_t, BufferSize> bufferB_{};
        inline static bool                                  activeIsA_{true};
        inline static std::atomic<bool>                     running_{false};
        inline static DmaCallback_t                         userCallback_{};

        template<typename F>
        static void start(F&& callback) {
            assert(!running_.load(std::memory_order_relaxed));
            userCallback_ = std::forward<F>(callback);
            activeIsA_    = true;
            running_.store(true, std::memory_order_relaxed);

            // Configure FIFO for DMA
            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<1u>()),
                                              write(Regs::FCS::dreq_en, Register::value<1u>()),
                                              write(Regs::FCS::thresh, Register::value<1u>())));

            base::drainFifo();

            // Reset AINSEL to first channel
            apply(write(Regs::CS::ainsel, static_cast<std::uint32_t>(base::firstChannel_)));

            // Start first DMA transfer to buffer A
            Dma::template start<DmaChannel,
                                DmaPriority,
                                base::DmaTrigger,
                                Dma::TransferSize::_16,
                                true,
                                false>(reinterpret_cast<std::uint32_t>(bufferA_.data()),
                                       Regs::FIFO::Addr::value,
                                       BufferSize,
                                       []() { dmaComplete(); });

            // Start free-running ADC
            apply(set(Regs::CS::start_many));
        }

        static void stop() {
            running_.store(false, std::memory_order_relaxed);
            apply(clear(Regs::CS::start_many));
            while(!get<0>(apply(read(Regs::CS::ready)))) {}

            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<0u>()),
                                              write(Regs::FCS::dreq_en, Register::value<1u>())));

            base::drainFifo();
            userCallback_.reset();
        }

    private:
        static void dmaComplete() {
            if(!running_.load(std::memory_order_relaxed)) { return; }

            auto completed = activeIsA_ ? std::span{bufferA_} : std::span{bufferB_};
            activeIsA_     = !activeIsA_;
            auto& next     = activeIsA_ ? bufferA_ : bufferB_;

            // Start DMA to the other buffer
            Dma::template start<DmaChannel,
                                DmaPriority,
                                base::DmaTrigger,
                                Dma::TransferSize::_16,
                                true,
                                false>(reinterpret_cast<std::uint32_t>(next.data()),
                                       Regs::FIFO::Addr::value,
                                       BufferSize,
                                       []() { dmaComplete(); });

            if(userCallback_) { userCallback_(completed); }
        }
    };

    // ISR behavior
    // Each ADC FIFO sample triggers the user callback with (channel, value).
    template<typename ADCConfig_>
    struct ADCIsrBehavior : ADCBase<ADCConfig_> {
        using base = ADCBase<ADCConfig_>;
        using Regs = typename base::Regs;

        using IsrCallback_t = Kvasir::StaticFunction<void(std::uint8_t, std::uint16_t),
                                                     base::ADCConfig::callbackFunctionSize>;

        inline static std::size_t   channelIndex_{0};
        inline static IsrCallback_t userCallback_{};

        template<typename F>
        static void start(F&& callback) {
            channelIndex_ = 0;
            userCallback_ = std::forward<F>(callback);

            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<1u>()),
                                              write(Regs::FCS::thresh, Register::value<1u>())));

            base::drainFifo();

            apply(write(Regs::CS::ainsel, static_cast<std::uint32_t>(base::firstChannel_)));

            // Enable FIFO interrupt
            apply(set(Regs::INTE::fifo));
            apply(Nvic::makeEnable(typename base::InterruptIndexs{}));

            // Start free-running ADC
            apply(set(Regs::CS::start_many));
        }

        static void stop() {
            apply(clear(Regs::CS::start_many));
            while(!get<0>(apply(read(Regs::CS::ready)))) {}

            apply(clear(Regs::INTE::fifo));

            apply(Regs::FCS::overrideDefaults(write(Regs::FCS::en, Register::value<0u>())));

            base::drainFifo();
            userCallback_.reset();
        }

        static void onIsr() {
            while(!get<0>(apply(read(Regs::FCS::empty)))) {
                auto val = static_cast<std::uint16_t>(get<0>(apply(read(Regs::FIFO::val))));
                auto ch  = base::channelTable_.channels[channelIndex_];
                if constexpr(base::numChannels_ > 1) {
                    channelIndex_ = (channelIndex_ + 1) % base::numChannels_;
                }
                if(userCallback_) { userCallback_(ch, val); }
            }
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };
}}   // namespace Kvasir::ADC
