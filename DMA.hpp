#pragma once

#include "PinConfig.hpp"
#include "chip/Interrupt.hpp"
#include "core/core.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/StaticFunction.hpp"
#include "peripherals/DMA.hpp"
#include "peripherals/RESETS.hpp"

#include <array>
#include <bit>
#include <optional>
#include <utility>

namespace Kvasir { namespace DMA {

    enum class DMAChannel {
        ch0  = 0,
        ch1  = 1,
        ch2  = 2,
        ch3  = 3,
        ch4  = 4,
        ch5  = 5,
        ch6  = 6,
        ch7  = 7,
        ch8  = 8,
        ch9  = 9,
        ch10 = 10,
        ch11 = 11,
        ch12 = 12,   // RP2350 only
        ch13 = 13,   // RP2350 only
        ch14 = 14,   // RP2350 only
        ch15 = 15,   // RP2350 only
    };

    enum class DMAPriority { high = 0, low };

    using TriggerSource = Kvasir::Peripheral::DMA::Registers<>::CH<0>::CTRL_TRIG::TREQ_SELVal;

    enum class DMATransferSize { _8 = 0, _16 = 1, _32 = 2 };

    // An explicit channel set for a DmaBase instance: `static constexpr auto channels =
    // Channels<DMAChannel::ch0, DMAChannel::ch5>{};`. With it, the interrupt-enable mask is
    // exactly these channels rather than "the first N", which is what lets two instances,
    // one per core, share the controller without sharing an interrupt line.
    template<DMAChannel... Cs>
    struct Channels {
        static constexpr std::size_t   count = sizeof...(Cs);
        static constexpr std::uint32_t mask  = ((1U << static_cast<unsigned>(Cs)) | ...);
    };

    // The controller's channels and interrupt lines as Startup resources
    // (kvasir/StartUp/Resources.hpp). A DmaBase provides the channels it owns and the
    // line(s) it answers; a driver claims the channels it starts transfers on. Startup then
    // refuses two instances that overlap, two drivers on one channel, a driver whose
    // instance is in no list, and a driver whose instance is in the other core's list.
    struct ChannelResourceTag {};

    struct LineResourceTag {};

    template<unsigned Channel>
    using ChannelResource = Kvasir::Startup::Resource<ChannelResourceTag, Channel>;

    template<unsigned Line>
    using LineResource = Kvasir::Startup::Resource<LineResourceTag, Line>;

    namespace Detail {
        template<std::uint32_t Mask, typename Seq>
        struct ChannelsFromMask;

        template<std::uint32_t Mask, std::size_t... Is>
        struct ChannelsFromMask<Mask, std::index_sequence<Is...>> {
            using type = brigand::flatten<brigand::list<
              std::conditional_t<((Mask >> Is) & 1U) != 0,
                                 brigand::list<ChannelResource<static_cast<unsigned>(Is)>>,
                                 brigand::list<>>...>>;
        };

        template<typename Seq>
        struct AllLines;

        template<std::size_t... Is>
        struct AllLines<std::index_sequence<Is...>> {
            using type = brigand::list<LineResource<static_cast<unsigned>(Is)>...>;
        };

        template<std::size_t N>
        constexpr bool allDistinct(std::array<unsigned,
                                              N> const& v) {
            for(std::size_t i = 0; i < N; ++i) {
                for(std::size_t j = i + 1; j < N; ++j) {
                    if(v[i] == v[j]) { return false; }
                }
            }
            return true;
        }
    }   // namespace Detail

    // What a driver puts in its `Claims`: `using Claims = Kvasir::DMA::Claims<Dma, ChA, ChB>;`.
    // The two checks a driver can make on its own are made here, with the driver's name in
    // the error; the cross-peripheral ones are Startup's.
    template<typename Dma, typename Dma::Channel... Cs>
    struct ClaimsFor {
        static_assert((Dma::ownsChannel(Cs) && ...),
                      "a driver's DMA channel is not one of its DmaBase's channels: the "
                      "transfer would run but its completion interrupt is never enabled");
        static_assert(Detail::allDistinct(std::array<unsigned,
                                                     sizeof...(Cs)>{static_cast<unsigned>(Cs)...}),
                      "a driver names the same DMA channel twice");
        using type = brigand::list<ChannelResource<static_cast<unsigned>(Cs)>...>;
    };

    template<typename Dma, typename Dma::Channel... Cs>
    using Claims = typename ClaimsFor<Dma, Cs...>::type;

    // Everything a channel's CTRL register says, as one value: what start<>() takes as
    // template parameters plus the parts it fixes - which channel to chain to when done
    // (`chainTo`, -1 = none), whether completion raises the interrupt (`irqQuiet` inverts
    // that: the interrupt comes from a null trigger instead, the end of a control-block
    // chain), an address wrap (`ringBits` = log2 of the ring in bytes, on the read or the
    // write address), the sniffer, a byte swap. A structural type, so it can be a template
    // argument and a control block's CTRL word can be computed at compile time.
    struct ChannelConfig {
        DMAPriority     priority       = DMAPriority::low;
        TriggerSource   trigger        = TriggerSource::permanent;
        DMATransferSize size           = DMATransferSize::_32;
        bool            incrementRead  = true;
        bool            incrementWrite = true;
        int             chainTo        = -1;
        bool            irqQuiet       = false;
        unsigned        ringBits       = 0;
        bool            ringOnWrite    = false;
        bool            sniff          = false;
        bool            byteSwap       = false;
    };

    // A control block for the two-register form (DMA::ControlBlockList): transfer count and
    // read address, written to a data channel's AL3_TRANS_COUNT and AL3_READ_ADDR_TRIG by a
    // control channel with a two-word ring on its write address. An all-zero block (a null
    // read address into the trigger register) ends the chain: nothing starts, and with
    // `irqQuiet` on the data channel raises its interrupt.
    struct ControlBlock {
        std::uint32_t count;
        std::uint32_t readAddress;

        static constexpr ControlBlock end() { return {0, 0}; }

        template<typename T>
        static ControlBlock of(std::span<T const> data) {
            return {static_cast<std::uint32_t>(data.size()),
                    reinterpret_cast<std::uint32_t>(data.data())};
        }
    };

    template<typename DMAConfig_>
    struct DmaBase {
        // needed config
        // numberOfChannels

        struct DMAConfig : DMAConfig_ {
            static constexpr auto callbackFunctionSize = [] {
                if constexpr(requires { DMAConfig_::callbackFunctionSize; }) {
                    return DMAConfig_::callbackFunctionSize;
                } else {
                    return 0;
                }
            }();

            static constexpr auto isrPriority = [] {
                if constexpr(requires { DMAConfig_::isrPriority; }) {
                    return DMAConfig_::isrPriority;
                } else {
                    return 0;
                }
            }();

            // Either an explicit `channels` set or the legacy "first numberOfChannels".
            static constexpr bool hasChannels = requires { DMAConfig_::channels; };

            static constexpr auto numberOfChannels = [] {
                if constexpr(requires { DMAConfig_::numberOfChannels; }) {
                    return DMAConfig_::numberOfChannels;
                } else {
                    static_assert(hasChannels, "DMA config needs numberOfChannels or channels");
                    return std::remove_cvref_t<decltype(DMAConfig_::channels)>::count;
                }
            }();

            // Which of the controller's interrupt lines (DMA_IRQ_0..3 on the RP2350) this
            // instance owns. Absent means the legacy behaviour: INTE0 and every line.
            static constexpr bool hasInterruptInstance
              = requires { DMAConfig_::interruptInstance; };

            static constexpr unsigned interruptInstance = [] {
                if constexpr(hasInterruptInstance) {
                    return static_cast<unsigned>(DMAConfig_::interruptInstance);
                } else {
                    return 0U;
                }
            }();

            static constexpr std::uint32_t interruptMask = [] {
                if constexpr(hasChannels) {
                    return std::remove_cvref_t<decltype(DMAConfig_::channels)>::mask;
                } else {
                    // Bits 0..N-1 (maskFromRange is inclusive at both ends).
                    return static_cast<std::uint32_t>(
                      Register::maskFromRange(DMAConfig_::numberOfChannels - 1, 0));
                }
            }();
        };

        using Traits = PinConfig::DmaTraits<PinConfig::CurrentChip>;

        static_assert(DMAConfig::numberOfChannels > 0,
                      "a DMA instance needs at least one channel");
        static_assert(Traits::channelCount >= DMAConfig::numberOfChannels,
                      "DMA numberOfChannels exceeds chip capabilities");
        static_assert(Traits::interruptCount > DMAConfig::interruptInstance,
                      "DMA interruptInstance exceeds chip capabilities");

        static constexpr std::size_t numberOfChannels{DMAConfig::numberOfChannels};

        // Whether this instance owns a channel: one of its explicit `channels`, or below
        // numberOfChannels in the legacy layout. What a driver should assert about the
        // channel it is handed, since with an explicit set the channel numbers and the
        // count no longer say anything about each other.
        static constexpr bool ownsChannel(DMAChannel c) {
            return ((DMAConfig::interruptMask >> static_cast<unsigned>(c)) & 1U) != 0;
        }

        // Callback slots are indexed by channel number. The legacy layout ("first N
        // channels") needs exactly N; an explicit set can name any channel, so it gets one
        // slot per hardware channel.
        static constexpr std::size_t callbackSlots
          = DMAConfig::hasChannels ? Traits::channelCount : DMAConfig::numberOfChannels;

        using FunctionArray_t = std::conditional_t<
          DMAConfig::callbackFunctionSize != 0,
          std::array<Kvasir::StaticFunction<void(), DMAConfig::callbackFunctionSize>,
                     callbackSlots>,
          std::array<std::byte, 0>>;

        using Regs = Kvasir::Peripheral::DMA::Registers<>;

    private:
        // The interrupt-enable and status registers of one line. `R` keeps the lookup
        // dependent so a chip with fewer lines (RP2040: two) never sees INTE2/INTE3.
        template<unsigned N,
                 typename R = Regs>
        static constexpr auto inteField() {
            if constexpr(N == 0) {
                return R::INTE0::inte0;
            } else if constexpr(N == 1) {
                return R::INTE1::inte1;
            } else if constexpr(N == 2) {
                return R::INTE2::inte2;
            } else {
                return R::INTE3::inte3;
            }
        }

        template<unsigned N,
                 typename R = Regs>
        static constexpr auto intsField() {
            if constexpr(N == 0) {
                return R::INTS0::ints0;
            } else if constexpr(N == 1) {
                return R::INTS1::ints1;
            } else if constexpr(N == 2) {
                return R::INTS2::ints2;
            } else {
                return R::INTS3::ints3;
            }
        }

    public:
        using TriggerSource = ::Kvasir::DMA::TriggerSource;
        using Priority      = ::Kvasir::DMA::DMAPriority;
        using Channel       = ::Kvasir::DMA::DMAChannel;
        using TransferSize  = ::Kvasir::DMA::DMATransferSize;

        // Helper to get chip-specific DMA interrupt list
        template<PinConfig::ChipVariant Chip>
        static constexpr auto getDmaInterrupts() {
            return PinConfig::DmaTraits<Chip>::Interrupts;
        }

        using AllInterruptIndexs = decltype(getDmaInterrupts<PinConfig::CurrentChip>());

        // Legacy: this instance answers every line. With interruptInstance: just its own,
        // so the line can be enabled in one core's NVIC and nothing else's.
        using InterruptIndexs = std::conditional_t<
          DMAConfig::hasInterruptInstance,
          brigand::list<brigand::at_c<AllInterruptIndexs, DMAConfig::interruptInstance>>,
          AllInterruptIndexs>;

        // Startup resources: the owned channels and the answered line(s). A legacy instance
        // provides every line, so a second instance next to it is a build error - the
        // intended one, since its ISR would never reach the vector table.
        using Provides = brigand::append<
          typename Detail::ChannelsFromMask<DMAConfig::interruptMask,
                                            std::make_index_sequence<Traits::channelCount>>::type,
          std::conditional_t<
            DMAConfig::hasInterruptInstance,
            brigand::list<LineResource<DMAConfig::interruptInstance>>,
            typename Detail::AllLines<std::make_index_sequence<Traits::interruptCount>>::type>>;

        static inline FunctionArray_t callbackFunctions{};

        // Unconditional on purpose: FirstInitStep asserts the controller reset, and the
        // instance that clears it may well be the only one, on whichever core.
        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::dma));

        // A full write, not a set of bits: Startup guarantees one instance per line
        // (Provides above), so nothing else's mask is in this register.
        static constexpr auto initStepPeripheryConfig
          = list(write(inteField<DMAConfig::interruptInstance>(),
                       Register::value<DMAConfig::interruptMask>()));

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<DMAConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(InterruptIndexs{}));

        template<typename T>
        struct Fail {
            static_assert(!std::is_void_v<T>,
                          "DMA callback not configured");
        };

    private:
        // Register writes only, no callback state: shared tail of start() and retrigger().
        template<DMAChannel      Channel,
                 DMAPriority     Priority,
                 TriggerSource   Trigger,
                 DMATransferSize Size,
                 bool            IncDest,
                 bool            IncSource>
        static void configureAndTrigger(std::uint32_t dest,
                                        std::uint32_t source,
                                        std::size_t   count) {
            using CHRegs = Regs::CH<static_cast<int>(Channel)>;

            apply(write(CHRegs::READ_ADDR::read_addr, source));
            apply(write(CHRegs::WRITE_ADDR::write_addr, dest));
            apply(write(CHRegs::TRANS_COUNT::trans_count, count));

            using ctrl = typename CHRegs::CTRL_TRIG;

            apply(ctrl::overrideDefaults(
              write(ctrl::treq_sel,
                    Register::value<typename ctrl::TREQ_SELVal,
                                    static_cast<typename ctrl::TREQ_SELVal>(Trigger)>()),
              write(ctrl::chain_to, Register::value<static_cast<int>(Channel)>()),
              set(ctrl::sniff_en),
              write(ctrl::incr_write, Register::value<IncDest ? 1 : 0>()),
              write(ctrl::incr_read, Register::value<IncSource ? 1 : 0>()),
              write(ctrl::data_size,
                    Register::value<typename ctrl::DATA_SIZEVal,
                                    static_cast<typename ctrl::DATA_SIZEVal>(Size)>()),
              write(ctrl::high_priority, Register::value<Priority == DMAPriority::high ? 1 : 0>()),
              set(ctrl::en)));
        }

    public:
        template<DMAChannel      Channel,
                 DMAPriority     Priority,
                 TriggerSource   Trigger,
                 DMATransferSize Size,
                 bool            IncDest,
                 bool            IncSource,
                 typename F>
        static void start(std::uint32_t dest,
                          std::uint32_t source,
                          std::size_t   count,
                          F&&           f) {
            if constexpr(!std::is_same_v<std::remove_cvref_t<F>, std::nullopt_t>) {
                if constexpr(DMAConfig::callbackFunctionSize > 0) {
                    callbackFunctions[static_cast<std::size_t>(Channel)] = std::forward<F>(f);
                } else {
                    Fail<void>{};
                }
            } else {
                if constexpr(DMAConfig::callbackFunctionSize > 0) {
                    callbackFunctions[static_cast<std::size_t>(Channel)].reset();
                }
            }

            configureAndTrigger<Channel, Priority, Trigger, Size, IncDest, IncSource>(dest,
                                                                                      source,
                                                                                      count);
        }

        template<DMAChannel      Channel,
                 DMAPriority     Priority,
                 TriggerSource   Trigger,
                 DMATransferSize Size,
                 bool            IncDest,
                 bool            IncSource>
        static void start(std::uint32_t dest,
                          std::uint32_t source,
                          std::size_t   count) {
            start<Channel, Priority, Trigger, Size, IncDest, IncSource>(dest,
                                                                        source,
                                                                        count,
                                                                        std::nullopt);
        }

        // Re-trigger without reinstalling the callback, for re-arming from inside
        // it. Needs a prior start(); abort() clears the callback.
        template<DMAChannel      Channel,
                 DMAPriority     Priority,
                 TriggerSource   Trigger,
                 DMATransferSize Size,
                 bool            IncDest,
                 bool            IncSource>
        static void retrigger(std::uint32_t dest,
                              std::uint32_t source,
                              std::size_t   count) {
            configureAndTrigger<Channel, Priority, Trigger, Size, IncDest, IncSource>(dest,
                                                                                      source,
                                                                                      count);
        }

        // ---- channels as descriptors: chains, control blocks, ad hoc transfers ------------

    private:
        template<typename Field>
        static constexpr std::uint32_t encode(std::uint32_t v) {
            constexpr std::uint32_t mask
              = Register::Detail::GetMask<std::remove_cvref_t<Field>>::value;
            return (v << std::countr_zero(mask)) & mask;
        }

    public:
        /// The CTRL word of `Channel` for `Cfg`, with the enable bit set: what a control
        /// block that reprograms a channel's CTRL carries, and what configure() writes.
        template<DMAChannel    Channel,
                 ChannelConfig Cfg>
        static constexpr std::uint32_t controlWord() {
            using ctrl = typename Regs::template CH<static_cast<unsigned>(Channel)>::CTRL_TRIG;
            static_assert(Cfg.chainTo < static_cast<int>(Traits::channelCount),
                          "chainTo names a channel the chip does not have");
            static_assert(Cfg.ringBits <= 15, "a DMA address ring is at most 2^15 bytes");
            std::uint32_t const chain = Cfg.chainTo < 0 ? static_cast<std::uint32_t>(Channel)
                                                        : static_cast<std::uint32_t>(Cfg.chainTo);
            return encode<decltype(ctrl::en)>(1U)
                 | encode<decltype(ctrl::high_priority)>(Cfg.priority == DMAPriority::high ? 1U
                                                                                           : 0U)
                 | encode<decltype(ctrl::data_size)>(static_cast<std::uint32_t>(Cfg.size))
                 | encode<decltype(ctrl::incr_read)>(Cfg.incrementRead ? 1U : 0U)
                 | encode<decltype(ctrl::incr_write)>(Cfg.incrementWrite ? 1U : 0U)
                 | encode<decltype(ctrl::ring_size)>(Cfg.ringBits)
                 | encode<decltype(ctrl::ring_sel)>(Cfg.ringOnWrite ? 1U : 0U)
                 | encode<decltype(ctrl::chain_to)>(chain)
                 | encode<decltype(ctrl::treq_sel)>(static_cast<std::uint32_t>(Cfg.trigger))
                 | encode<decltype(ctrl::irq_quiet)>(Cfg.irqQuiet ? 1U : 0U)
                 | encode<decltype(ctrl::bswap)>(Cfg.byteSwap ? 1U : 0U)
                 | encode<decltype(ctrl::sniff_en)>(Cfg.sniff ? 1U : 0U);
        }

        /// Program `Channel` without starting it: addresses, count and CTRL through the
        /// non-triggering alias. trigger<>() or another channel's chain starts it. The
        /// callback, if any, runs on completion (or on the null trigger with irqQuiet).
        template<DMAChannel    Channel,
                 ChannelConfig Cfg,
                 typename F>
        static void configure(std::uint32_t dest,
                              std::uint32_t source,
                              std::size_t   count,
                              F&&           f) {
            using CHRegs = typename Regs::template CH<static_cast<unsigned>(Channel)>;
            if constexpr(!std::is_same_v<std::remove_cvref_t<F>, std::nullopt_t>) {
                static_assert(DMAConfig::callbackFunctionSize > 0, "DMA callback not configured");
                callbackFunctions[static_cast<std::size_t>(Channel)] = std::forward<F>(f);
            } else if constexpr(DMAConfig::callbackFunctionSize > 0) {
                callbackFunctions[static_cast<std::size_t>(Channel)].reset();
            }
            apply(write(CHRegs::READ_ADDR::read_addr, source));
            apply(write(CHRegs::WRITE_ADDR::write_addr, dest));
            apply(write(CHRegs::TRANS_COUNT::trans_count, static_cast<std::uint32_t>(count)));
            apply(write(CHRegs::AL1_CTRL::FULLREGISTER, controlWord<Channel, Cfg>()));
        }

        template<DMAChannel    Channel,
                 ChannelConfig Cfg>
        static void configure(std::uint32_t dest,
                              std::uint32_t source,
                              std::size_t   count) {
            configure<Channel, Cfg>(dest, source, count, std::nullopt);
        }

        /// Start the channels named, at once (MULTI_CHAN_TRIGGER).
        template<DMAChannel... Cs>
        static void trigger() {
            static_assert(sizeof...(Cs) > 0);
            constexpr std::uint32_t mask = ((1U << static_cast<unsigned>(Cs)) | ...);
            apply(write(Regs::MULTI_CHAN_TRIGGER::multi_chan_trigger, Register::value<mask>()));
        }

        /// Where a control channel writes to reprogram `Channel` two words at a time: the
        /// AL3 alias's TRANS_COUNT, followed by READ_ADDR_TRIG (a ControlBlock, in that order).
        template<DMAChannel Channel>
        static constexpr std::uint32_t al3TransCountAddress
          = Regs::template CH<static_cast<unsigned>(Channel)>::AL3_TRANS_COUNT::Addr::value;

        /// Data channel `Data` plays a list of ControlBlocks fed by control channel `Ctrl`:
        /// the data channel moves `count` transfers from each block's `readAddress` to `dest`
        /// (paced by `DataCfg.trigger`), then chains to the control channel, which loads the
        /// next block into the data channel's AL3 registers and so triggers it; the null
        /// block at the end raises the data channel's interrupt and the callback.
        /// pico-examples' control_blocks, as one call.
        template<DMAChannel    Ctrl,
                 DMAChannel    Data,
                 ChannelConfig DataCfg,
                 typename F>
        static void startControlBlocks(std::uint32_t                 dest,
                                       std::span<ControlBlock const> blocks,
                                       F&&                           f) {
            static_assert(ownsChannel(Ctrl) && ownsChannel(Data),
                          "the control-block channels are not this DmaBase's");
            static_assert(Ctrl != Data, "a control channel cannot feed itself");
            constexpr ChannelConfig ctrlCfg{.priority       = DMAPriority::high,
                                            .trigger        = TriggerSource::permanent,
                                            .size           = DMATransferSize::_32,
                                            .incrementRead  = true,
                                            .incrementWrite = true,
                                            .chainTo        = -1,
                                            .irqQuiet       = true,   // the chain, not the loader
                                            .ringBits       = 3,      // two words, wrapping
                                            .ringOnWrite    = true};
            constexpr ChannelConfig dataCfg = [] {
                ChannelConfig c = DataCfg;
                c.chainTo       = static_cast<int>(Ctrl);
                c.irqQuiet      = true;   // the null trigger is the interrupt
                return c;
            }();
            static_assert((al3TransCountAddress<Data> & 0x7U) == 0,
                          "the AL3 TRANS_COUNT / READ_ADDR_TRIG pair must be 8-byte aligned "
                          "for the control channel's ring");
            configure<Data, dataCfg>(dest, 0, 0, std::forward<F>(f));
            configure<Ctrl, ctrlCfg>(al3TransCountAddress<Data>,
                                     reinterpret_cast<std::uint32_t>(blocks.data()),
                                     2);
            trigger<Ctrl>();
        }

        template<DMAChannel    Ctrl,
                 DMAChannel    Data,
                 ChannelConfig DataCfg>
        static void startControlBlocks(std::uint32_t                 dest,
                                       std::span<ControlBlock const> blocks) {
            startControlBlocks<Ctrl, Data, DataCfg>(dest, blocks, std::nullopt);
        }

        template<DMAChannel Channel>
        static bool ready() {
            using CHRegs = Regs::CH<static_cast<int>(Channel)>;
            return !apply(read(CHRegs::CTRL_TRIG::busy));
        }

        // Transfers this channel still has to do. Diagnostic: busy with a
        // nonzero count means the channel is starved, which no other register
        // shows.
        template<DMAChannel Channel>
        static std::uint32_t remaining() {
            using CHRegs = Regs::CH<static_cast<int>(Channel)>;
            return get<0>(apply(read(CHRegs::TRANS_COUNT::trans_count)));
        }

        // The FIFOs drain in a few cycles; hitting this bound means the channel
        // is wedged beyond what an abort can fix.
        static constexpr int AbortPollLimit = 10000;

        // Terminate the transfer sequence in progress on `Channel` and leave it
        // safe to restart. Needed by any error, timeout or reset path: dropping
        // a transfer does not stop the hardware, so without this the sequence
        // runs on and the next start() re-arms a live channel.
        //
        // Per datasheet the abort bit must be polled until it reads back zero -
        // until then transfers are still draining through the FIFOs and
        // restarting the channel is unsafe. Returns false if that never
        // happened, i.e. restarting the channel is still not safe.
        template<DMAChannel Channel>
        static bool abort() {
            constexpr std::uint32_t bit = 1u << static_cast<int>(Channel);
            using CHRegs                = Regs::CH<static_cast<int>(Channel)>;

            apply(clear(CHRegs::CTRL_TRIG::en));   // pause the channel first
            apply(write(Regs::CHAN_ABORT::chan_abort, bit));
            bool drained = false;
            for(int i = 0; i < AbortPollLimit; ++i) {
                if((get<0>(apply(read(Regs::CHAN_ABORT::chan_abort))) & bit) == 0) {
                    drained = true;
                    break;
                }
            }
            // Drop a completion that raced the abort: left pending it would
            // fire into the callback the next start() installs.
            apply(write(Regs::INTR::intr, bit));
            if constexpr(DMAConfig::callbackFunctionSize > 0) {
                callbackFunctions[static_cast<std::size_t>(Channel)].reset();
            }
            return drained;
        }

        static void onIsr() {
            static constexpr auto ints = intsField<DMAConfig::interruptInstance>();

            auto const channels = get<0>(apply(read(ints)));
            apply(write(ints, channels));

            if constexpr(DMAConfig::callbackFunctionSize > 0) {
                for(std::size_t i{}; auto& f : callbackFunctions) {
                    if(((channels & (1U << i)) != 0) && f) { f(); }
                    ++i;
                }
            }
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(InterruptIndexs{}));
    };
}}   // namespace Kvasir::DMA
