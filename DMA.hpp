#pragma once

#include "PinConfig.hpp"
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "chip/Interrupt.hpp"
#include "chip/PinConfig.hpp"
#include "core/core.hpp"
#include "kvasir/Atomic/CriticalSection.hpp"
#include "kvasir/Hal/Dma.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/Config.hpp"
#include "kvasir/Util/StaticFunction.hpp"
#include "peripherals/DMA.hpp"
#include "peripherals/RESETS.hpp"

#include <array>
#include <atomic>
#include <bit>
#include <optional>
#include <span>
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

    enum class DMATransferSize { size8 = 0, size16 = 1, size32 = 2 };

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
    // The channels are the SDK's Hal::DmaChannel<the controller, channel> (kvasir/Hal/Dma.hpp);
    // the lines are this chip's own (a SAM DMAC has one vector).
    struct LineResourceTag {
        static constexpr char const* describeAs = "DMA interrupt line";
    };

    template<unsigned Line>
    using LineResource = Kvasir::Startup::Resource<LineResourceTag, Line>;

    namespace Detail {
        // The counted layout as a channel set: the first N channels.
        template<std::size_t... Is>
        consteval auto firstChannels(std::index_sequence<Is...>) {
            return Channels<static_cast<DMAChannel>(Is)...>{};
        }

        template<std::uint32_t Mask, typename Seq>
        struct ChannelsFromMask;

        template<std::uint32_t Mask, std::size_t... Is>
        struct ChannelsFromMask<Mask, std::index_sequence<Is...>> {
            using type = brigand::flatten<brigand::list<
              std::conditional_t<((Mask >> Is) & 1U) != 0,
                                 brigand::list<Hal::DmaChannel<Kvasir::Peripheral::DMA::Registers<>,
                                                               static_cast<unsigned>(Is)>>,
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
        using type = brigand::list<
          Hal::DmaChannel<Kvasir::Peripheral::DMA::Registers<>, static_cast<unsigned>(Cs)>...>;
    };

    template<typename Dma, typename Dma::Channel... Cs>
    using Claims = typename ClaimsFor<Dma, Cs...>::type;

    // Everything a channel's CTRL register says, as one value, for every transfer: start<>(),
    // retrigger<>(), configure<>() and control blocks take it - priority, pacing trigger, size,
    // address increments, which channel to chain to when done (`chainTo`, -1 = none),
    // whether completion raises the interrupt (`irqQuiet` inverts that: the interrupt comes
    // from a null trigger instead, the end of a control-block chain), an address wrap (`ringBits` = log2 of the ring in bytes, on the read or the
    // write address), the sniffer, a byte swap. A structural type, so it can be a template
    // argument and a control block's CTRL word can be computed at compile time.
    struct ChannelConfig {
        DMAPriority     priority       = DMAPriority::low;
        TriggerSource   trigger        = TriggerSource::permanent;
        DMATransferSize size           = DMATransferSize::size32;
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
            static constexpr auto callbackFunctionSize
              = KVASIR_CONFIG_OR(DMAConfig_, callbackFunctionSize, 0);

            // 3, like every other driver's default; 0 would be level with SysTick.
            static constexpr auto isrPriority = KVASIR_CONFIG_OR(DMAConfig_, isrPriority, 3);

            // Either an explicit `channels` set or the counted layout, the first numberOfChannels, made into that set
            // here: everything below reads `channels`.
            static constexpr bool hasChannels = requires { DMAConfig_::channels; };
            static_assert(
              hasChannels || requires { DMAConfig_::numberOfChannels; },
              "DMA config needs numberOfChannels or channels");

            static constexpr auto channels = KVASIR_CONFIG_OR(
              DMAConfig_,
              channels,
              Detail::firstChannels(
                std::make_index_sequence<std::size_t{DMAConfig_::numberOfChannels}>{}));

            static constexpr auto numberOfChannels
              = KVASIR_CONFIG_OR(DMAConfig_,
                                 numberOfChannels,
                                 std::remove_cvref_t<decltype(channels)>::count);

            // Which of the controller's interrupt lines (DMA_IRQ_0..3 on the RP2350) this
            // instance owns. Absent: INTE0 and every line (one instance for the whole controller).
            static constexpr bool hasInterruptInstance
              = requires { DMAConfig_::interruptInstance; };

            static constexpr unsigned interruptInstance
              = static_cast<unsigned>(KVASIR_CONFIG_OR(DMAConfig_, interruptInstance, 0U));

            static constexpr std::uint32_t interruptMask
              = std::remove_cvref_t<decltype(channels)>::mask;
        };

        using Traits = PinConfig::DmaTraits<Chip::part>;

        static_assert(DMAConfig::numberOfChannels > 0,
                      "a DMA instance needs at least one channel");
        static_assert(Traits::channelCount >= DMAConfig::numberOfChannels,
                      "DMA numberOfChannels exceeds chip capabilities");
        static_assert(Traits::interruptCount > DMAConfig::interruptInstance,
                      "DMA interruptInstance exceeds chip capabilities");

        static constexpr std::size_t numberOfChannels{DMAConfig::numberOfChannels};

        // Whether this instance owns a channel: one of its `channels` (explicit, or the first
        // numberOfChannels). What a driver should assert about the
        // channel it is handed, since with an explicit set the channel numbers and the
        // count no longer say anything about each other.
        static constexpr bool ownsChannel(DMAChannel c) {
            return ((DMAConfig::interruptMask >> static_cast<unsigned>(c)) & 1U) != 0;
        }

        // Callback slots are indexed by channel number. The counted layout ("first N
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
        // The interrupt-enable and status registers of one line (INTEn / INTSn, one field each: inte<n> / ints<n>),
        // as many as the chip has lines (RP2040 two, RP2350 four).
#ifdef KVASIR_CHIP_RP2040
        using LineEnables  = brigand::list<Regs::INTE0, Regs::INTE1>;
        using LineStatuses = brigand::list<Regs::INTS0, Regs::INTS1>;
#else
        using LineEnables  = brigand::list<Regs::INTE0, Regs::INTE1, Regs::INTE2, Regs::INTE3>;
        using LineStatuses = brigand::list<Regs::INTS0, Regs::INTS1, Regs::INTS2, Regs::INTS3>;
#endif

        template<unsigned N>
        static constexpr auto inteField() {
            return Register::fieldAt<0>(brigand::at_c<LineEnables, N>{});
        }

        template<unsigned N>
        static constexpr auto intsField() {
            return Register::fieldAt<0>(brigand::at_c<LineStatuses, N>{});
        }

    public:
        using TriggerSource = ::Kvasir::DMA::TriggerSource;
        using Priority      = ::Kvasir::DMA::DMAPriority;
        using Channel       = ::Kvasir::DMA::DMAChannel;
        using TransferSize  = ::Kvasir::DMA::DMATransferSize;

        // Helper to get chip-specific DMA interrupt list
        template<Chip::Part P>
        static constexpr auto getDmaInterrupts() {
            return PinConfig::DmaTraits<P>::interrupts;
        }

        using AllInterruptIndexs = decltype(getDmaInterrupts<Chip::part>());

        // Without interruptInstance this instance answers every line; with it just its own,
        // so the line can be enabled in one core's NVIC and nothing else's.
        using InterruptIndexs = std::conditional_t<
          DMAConfig::hasInterruptInstance,
          brigand::list<brigand::at_c<AllInterruptIndexs, DMAConfig::interruptInstance>>,
          AllInterruptIndexs>;

        // Startup resources: the owned channels and the answered line(s). An instance without
        // interruptInstance provides every line, so a second instance next to it is a build
        // error - the intended one, since its ISR would never reach the vector table.
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
        static constexpr auto powerClockEnable = list(clear(Rp::Block<Regs>::reset));

        // A full write, not a set of bits: Startup guarantees one instance per line
        // (Provides above), so nothing else's mask is in this register.
        static constexpr auto initStepPeripheryConfig
          = list(write(inteField<DMAConfig::interruptInstance>(),
                       Register::value<DMAConfig::interruptMask>()));

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<DMAConfig::isrPriority>(InterruptIndexs{}),
                 Nvic::makeClearPending(InterruptIndexs{}));

        static constexpr auto initStepPeripheryEnable = list(Nvic::makeEnable(InterruptIndexs{}));

        /// The controller set up again after a reset under its users: RESETS.DMA held, or pulsed, puts every channel
        /// and INTEn back to their reset values (INTE0 resets to 0, RP2040 datasheet Table 138 md l.5531, RP2350
        /// Table 1165 md l.55015: no completion interrupt any more). Out of reset (RESET_DONE
        /// waited for) and this instance's interrupt enable written as at startup - except the bits an abort() holds
        /// clear right now (RP2040-E13, abort()'s comment): enabled mid-abort, the late completion of an aborted
        /// channel would reach its callback once more. True if it had to write. The transfers that ran are gone: each
        /// user starts its own again (the UARTs' transmit watches call this; a reader's look only when the DMA is
        /// held in reset or its own channel's bit is gone, completionEnabled()). The NVIC is no part of the block and
        /// keeps its enable.
        static bool setUpAgainIfReset() {
            bool const reset = apply(read(Rp::Block<Regs>::reset)) != 0;
            if(reset) {
                apply(powerClockEnable);
                Rp::Block<Regs>::waitResetDone();
            }
            return Kvasir::criticalSection<InteTag>([reset] {
                constexpr auto      inte = inteField<DMAConfig::interruptInstance>();
                std::uint32_t const want = DMAConfig::interruptMask & ~abortHeld_;
                if(!reset && (get<0>(apply(read(inte))) & want) == want) { return false; }
                // the held bits read 0 already: abort() cleared them in this section
                apply(write(inte, want));
                return true;
            });
        }

        /// Whether `Channel`'s completion raises this instance's line: its bit in INTEn (a DMA reset clears it, so does
        /// an abort() of the channel while it runs). The DMA must be out of reset.
        template<DMAChannel Channel>
        static bool completionEnabled() {
            constexpr auto inte = inteField<DMAConfig::interruptInstance>();
            return (get<0>(apply(read(inte))) & (1U << static_cast<unsigned>(Channel))) != 0;
        }

    private:
        // INTEn's read-modify-writes - abort()'s clear and restore, setUpAgainIfReset()'s repair - exclude each other
        // under this tag, on both cores; abortHeld_ (guarded by it) names the channels an abort() holds out of INTEn.
        struct InteTag {};

        inline static std::uint32_t abortHeld_{};

        // Called right before every register write that can start a channel. The callback slot
        // and the buffers a transfer reads are plain memory, the trigger a volatile store, and
        // the compiler may move plain stores after a volatile one: clang put the invoker's store
        // after CTRL_TRIG in start(). A short transfer (a UART frame into an empty FIFO) completes
        // within those few cycles, onIsr() found the slot still empty, and the completion was
        // lost for good. One core: ordering the compiler is all it takes.
        static void beforeTrigger() { std::atomic_signal_fence(std::memory_order_seq_cst); }

        // The callback slot of `Channel`: `f` published, or emptied for std::nullopt (shared by start() and
        // configure()).
        template<DMAChannel Channel,
                 typename F>
        static void installCallback(F&& f) {
            if constexpr(!std::is_same_v<std::remove_cvref_t<F>, std::nullopt_t>) {
                static_assert(DMAConfig::callbackFunctionSize > 0, "DMA callback not configured");
                callbackFunctions[static_cast<std::size_t>(Channel)].publish(std::forward<F>(f));
            } else if constexpr(DMAConfig::callbackFunctionSize > 0) {
                callbackFunctions[static_cast<std::size_t>(Channel)].reset();
            }
        }

        // Register writes only, no callback state: shared tail of start() and retrigger(). One apply per
        // register keeps the program order; CTRL_TRIG, the trigger, is written last.
        template<DMAChannel    Channel,
                 ChannelConfig Cfg>
        static void program(std::uint32_t dest,
                            std::uint32_t source,
                            std::size_t   count) {
            using CHRegs = typename Regs::template CH<static_cast<unsigned>(Channel)>;

            beforeTrigger();
            apply(write(CHRegs::READ_ADDR::read_addr, source));
            apply(write(CHRegs::WRITE_ADDR::write_addr, dest));
            apply(write(CHRegs::TRANS_COUNT::trans_count, static_cast<std::uint32_t>(count)));
            apply(write(CHRegs::CTRL_TRIG::FULLREGISTER,
                        Register::value<controlWord<Channel, Cfg>()>()));
        }

    public:
        /// Program `Channel` for `Cfg` and start it (CTRL_TRIG): `count` transfers from `source` to `dest`.
        /// `f` runs on completion; std::nullopt clears the channel's callback.
        template<DMAChannel    Channel,
                 ChannelConfig Cfg,
                 typename F>
        static void start(std::uint32_t dest,
                          std::uint32_t source,
                          std::size_t   count,
                          F&&           f) {
            installCallback<Channel>(std::forward<F>(f));
            program<Channel, Cfg>(dest, source, count);
        }

        template<DMAChannel    Channel,
                 ChannelConfig Cfg>
        static void start(std::uint32_t dest,
                          std::uint32_t source,
                          std::size_t   count) {
            start<Channel, Cfg>(dest, source, count, std::nullopt);
        }

        // With the buffers as spans: the count is the source's size, the element width is checked against
        // Cfg.size, and the destination must hold the source. Returns false, with nothing started, when it
        // does not.
        template<DMAChannel    Channel,
                 ChannelConfig Cfg,
                 typename T,
                 typename U,
                 typename F = std::nullopt_t>
        [[nodiscard]] static bool start(std::span<T> dest,
                                        std::span<U> source,
                                        F&&          f = std::nullopt) {
            static_assert(sizeof(T) == sizeof(U), "source and destination element widths differ");
            static_assert(sizeof(T) == (1U << static_cast<unsigned>(Cfg.size)),
                          "element width does not match the transfer size");
            if(dest.size() < source.size()) { return false; }
            start<Channel, Cfg>(std::bit_cast<std::uint32_t>(dest.data()),
                                std::bit_cast<std::uint32_t>(source.data()),
                                source.size(),
                                std::forward<F>(f));
            return true;
        }

        // Re-trigger without reinstalling the callback, for re-arming from inside
        // it. Needs a prior start(); abort() clears the callback.
        template<DMAChannel    Channel,
                 ChannelConfig Cfg>
        static void retrigger(std::uint32_t dest,
                              std::uint32_t source,
                              std::size_t   count) {
            program<Channel, Cfg>(dest, source, count);
        }

        // ---- channels as descriptors: chains, control blocks, ad hoc transfers ------------

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
            return Register::deposit(ctrl::en, 1U)
                 | Register::deposit(ctrl::high_priority,
                                     Cfg.priority == DMAPriority::high ? 1U : 0U)
                 | Register::deposit(ctrl::data_size, static_cast<std::uint32_t>(Cfg.size))
                 | Register::deposit(ctrl::incr_read, Cfg.incrementRead ? 1U : 0U)
                 | Register::deposit(ctrl::incr_write, Cfg.incrementWrite ? 1U : 0U)
                 | Register::deposit(ctrl::ring_size, Cfg.ringBits)
                 | Register::deposit(ctrl::ring_sel, Cfg.ringOnWrite ? 1U : 0U)
                 | Register::deposit(ctrl::chain_to, chain)
                 | Register::deposit(ctrl::treq_sel, static_cast<std::uint32_t>(Cfg.trigger))
                 | Register::deposit(ctrl::irq_quiet, Cfg.irqQuiet ? 1U : 0U)
                 | Register::deposit(ctrl::bswap, Cfg.byteSwap ? 1U : 0U)
                 | Register::deposit(ctrl::sniff_en, Cfg.sniff ? 1U : 0U);
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
            installCallback<Channel>(std::forward<F>(f));
            beforeTrigger();   // AL1_CTRL enables it: a running channel may chain to it at once
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
            beforeTrigger();
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
                                            .size           = DMATransferSize::size32,
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

        /// Whether `Channel` is idle (CTRL.BUSY clear): its transfer done or aborted, safe to start again.
        template<DMAChannel Channel>
        static bool ready() {
            using CHRegs = typename Regs::template CH<static_cast<unsigned>(Channel)>;
            return !apply(read(CHRegs::CTRL_TRIG::busy));
        }

        /// Whether `Channel`'s completion is raised on this instance's line and not handled yet: its bit in INTSn,
        /// "active channel interrupt requests which are currently causing IRQ n to be asserted" (RP2040 datasheet
        /// Table 140 md l.5577, RP2350 Table 1167 md l.55058). With interrupts held off, a finished transfer reads so
        /// until onIsr() runs; a channel whose interrupt enable was lost never does.
        template<DMAChannel Channel>
        static bool completionPending() {
            constexpr auto ints = intsField<DMAConfig::interruptInstance>();
            return (get<0>(apply(read(ints))) & (1U << static_cast<unsigned>(Channel))) != 0;
        }

        // Transfers this channel still has to do. Diagnostic: busy with a
        // nonzero count means the channel is starved, which no other register
        // shows.
        template<DMAChannel Channel>
        static std::uint32_t remaining() {
            using CHRegs = Regs::CH<static_cast<unsigned>(Channel)>;
            return get<0>(apply(read(CHRegs::TRANS_COUNT::trans_count)));
        }

        /// The next address `Channel` reads: READ_ADDR "updates automatically each time a read completes" (RP2040
        /// datasheet Table 121 md l.5358, RP2350 Table 1148 md l.54800). After the last read of a sequence that
        /// increments it, the sequence's end; 0 after a DMA reset (its reset value).
        template<DMAChannel Channel>
        static std::uint32_t readAddress() {
            using CHRegs = Regs::CH<static_cast<unsigned>(Channel)>;
            return get<0>(apply(read(CHRegs::READ_ADDR::read_addr)));
        }

        /// One read of `Channel`'s CTRL: EN, BUSY and RING_SIZE. A DMA reset puts RING_SIZE and EN to 0 (RP2040
        /// datasheet Table 124 md l.5436 / l.5448, RP2350 Table 1151 md l.54890 / l.54915): a ring channel's RING_SIZE
        /// then reads 0, which tells a channel that was reset from one that was only paused (EN cleared) or aborted.
        struct ChannelState {
            bool     enabled;
            bool     busy;
            unsigned ringBits;
        };

        template<DMAChannel Channel>
        static ChannelState channelState() {
            using ctrl   = typename Regs::template CH<static_cast<unsigned>(Channel)>::CTRL_TRIG;
            auto const r = apply(read(ctrl::en, ctrl::busy, ctrl::ring_size));
            return {static_cast<bool>(r.template get<0>()),
                    static_cast<bool>(r.template get<1>()),
                    static_cast<unsigned>(r.template get<2>())};
        }

        /// Whether `Channel`'s callback slot holds a function. abort() empties it; a driver that keeps one callback
        /// for every transfer (retrigger) installs it again only then, so no slot is written while a completion of the
        /// same channel may be calling it.
        template<DMAChannel Channel>
        static bool hasCallback() {
            if constexpr(DMAConfig::callbackFunctionSize > 0) {
                return static_cast<bool>(callbackFunctions[static_cast<std::size_t>(Channel)]);
            } else {
                return false;
            }
        }

        /// Restart an idle `Channel` for `count` transfers from where its addresses are now (a ring channel goes on
        /// where it wrapped), CTRL and callback as they are: one write to AL1_TRANS_COUNT_TRIG, "writing a nonzero
        /// value will reload the channel counter and start the channel" (RP2040 2.5.7 l.5012, RP2350 12.6.10
        /// l.54300); un-reprogrammed READ/WRITE_ADDR continue (RP2350 12.6.2.1). On the RP2350 the write also sets
        /// TRANS_COUNT.MODE (31:28) to 0, NORMAL (12.6.2.2.1): `count` must stay below 2^28 there.
        template<DMAChannel Channel>
        static void rearm(std::uint32_t count) {
            using CHRegs = Regs::CH<static_cast<unsigned>(Channel)>;
            beforeTrigger();
            apply(write(CHRegs::AL1_TRANS_COUNT_TRIG::FULLREGISTER, count));
        }

        // The FIFOs drain in a few cycles; hitting this bound means the channel
        // is wedged beyond what an abort can fix.
        static constexpr int abortPollLimit = 10000;

        // Terminate the transfer sequences in progress on `Cs` and leave them safe to
        // restart. Needed by any error, timeout or reset path: dropping a transfer does not
        // stop the hardware, so without this the sequence runs on and the next start()
        // re-arms a live channel. Channels that chain to one another are aborted together,
        // in one call (RP2350 datasheet 12.6.8.3).
        //
        // The sequence covers both chips' errata:
        // - RP2040-E13 (RP2040 datasheet 2.5.5.3 and errata, md l.4921, l.29532-29551):
        //   CHAN_ABORT clears before in-flight transfers land, and their completion then
        //   asserts the channel's interrupt. So the interrupt enable is cleared first,
        //   CTRL.BUSY is polled (not only CHAN_ABORT), the spurious flag is cleared, and
        //   the enable restored. The clear holds those bits (abortHeld_) until the restore, so no repair of INTEn
        //   (setUpAgainIfReset(), from another context or core) enables them mid-abort, and the restore sets back the
        //   aborted channels' bits only - another abort's held bits stay clear (one abort per channel at a time: a
        //   channel's driver aborts it).
        // - RP2350-E5 (RP2350 datasheet 12.6.8.3 and errata, md l.54201, l.66506-66520): an
        //   aborted channel can fire its CHAIN_TO and be re-triggered on the last cycle of the
        //   abort. So EN is cleared and CHAIN_TO pointed at the channel itself ("prevents
        //   chaining", 12.6.3.2, md l.53837) before the abort, and CHAN_ABORT is polled until
        //   clear.
        // start() and configure() rewrite the whole CTRL, so nothing has to be put back.
        //
        // Returns false if a channel never came to rest, i.e. restarting it is still not safe.
        template<DMAChannel... Cs>
        static bool abort() {
            static_assert(sizeof...(Cs) > 0);
            constexpr std::uint32_t bits = ((1U << static_cast<unsigned>(Cs)) | ...);
            // static: the sections' lambdas name them without a capture
            static constexpr auto          inte      = inteField<DMAConfig::interruptInstance>();
            static constexpr std::uint32_t ownedBits = bits & DMAConfig::interruptMask;

            std::uint32_t enabled{};
            if constexpr(ownedBits != 0) {
                enabled = Kvasir::criticalSection<InteTag>([] {
                    abortHeld_            = abortHeld_ | ownedBits;
                    std::uint32_t const e = get<0>(apply(read(inte)));
                    apply(write(inte, e & ~ownedBits));
                    return e;
                });
            }
            (pauseUnchained<Cs>(), ...);
            apply(write(Regs::CHAN_ABORT::chan_abort, bits));
            bool drained = false;
            for(int i = 0; i < abortPollLimit; ++i) {
                if((get<0>(apply(read(Regs::CHAN_ABORT::chan_abort))) & bits) == 0
                   && (ready<Cs>() && ...))
                {
                    drained = true;
                    break;
                }
            }
            // Drop a completion that raced the abort: left pending it would fire into the
            // callback the next start() installs.
            apply(write(Regs::INTR::intr, bits));
            if constexpr(ownedBits != 0) {
                Kvasir::criticalSection<InteTag>([enabled] {
                    apply(write(inte, get<0>(apply(read(inte))) | (enabled & ownedBits)));
                    abortHeld_ = abortHeld_ & ~ownedBits;
                });
            }
            if constexpr(DMAConfig::callbackFunctionSize > 0) {
                (callbackFunctions[static_cast<std::size_t>(Cs)].reset(), ...);
            }
            return drained;
        }

    private:
        // abort()'s first step for one channel: ignore triggers, chain to nothing.
        template<DMAChannel Channel>
        static void pauseUnchained() {
            using ctrl = typename Regs::template CH<static_cast<unsigned>(Channel)>::CTRL_TRIG;
            apply(clear(ctrl::en),
                  write(ctrl::chain_to, Register::value<static_cast<unsigned>(Channel)>()));
        }

    public:
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

        using Isr = Nvic::IsrsFor<std::addressof(onIsr), InterruptIndexs>;
    };
}}   // namespace Kvasir::DMA
