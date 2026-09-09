#pragma once
#include "PinConfig.hpp"
#include "kvasir/StartUp/Resources.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <peripherals/PIO.hpp>
#include <utility>

namespace Kvasir { namespace Pio {
    // A PIO instance's state machines and instruction slots as Startup resources
    // (kvasir/StartUp/Resources.hpp). A driver that loads a program provides the state
    // machine it runs it on and every instruction slot the program occupies, each slot
    // tagged with the program's identity: two drivers on one state machine are a build
    // error, and so are two *different* programs overlapping in one instance's memory (the
    // second silently overwriting the first), while two state machines running the *same*
    // program at the same offset share the slots (mergeIdentical: same key, same payload),
    // which is how a program is shared. The state machine is driven from one core;
    // instruction memory is chip-wide.
    struct SmTag {};

    struct InstructionTag {
        static constexpr bool coreLocal      = false;
        static constexpr bool mergeIdentical = true;
    };

    template<unsigned Instance, unsigned Sm>
    using SmResource = Startup::Resource<SmTag, Instance * 4 + Sm>;

    template<unsigned Instance, unsigned Slot, unsigned long long ProgramId>
    using InstructionResource = Startup::Resource<InstructionTag, Instance * 32 + Slot, ProgramId>;

    // A program's identity for the resource check: FNV-1a over its instructions and wrap
    // points. Two programs with the same words at the same offset are the same program.
    template<typename Program>
    constexpr unsigned long long programId() {
        unsigned long long h   = 0xcbf29ce484222325ULL;
        auto const         mix = [&](unsigned long long v) {
            h ^= v;
            h *= 0x100000001b3ULL;
        };
        for(auto const w : Program::Instructions) { mix(w); }
        mix(static_cast<unsigned long long>(Program::WrapTarget));
        mix(static_cast<unsigned long long>(Program::Wrap));
        return h;
    }

    namespace Detail {
        template<unsigned Instance, unsigned Offset, unsigned long long Id, typename Seq>
        struct Instructions;

        template<unsigned Instance, unsigned Offset, unsigned long long Id, std::size_t... Is>
        struct Instructions<Instance, Offset, Id, std::index_sequence<Is...>> {
            using type = brigand::list<
              InstructionResource<Instance, Offset + static_cast<unsigned>(Is), Id>...>;
        };
    }   // namespace Detail

    template<unsigned Instance, unsigned Sm, unsigned Offset, typename Program>
    struct ProvidesFor {
        static constexpr std::size_t Length = Program::Instructions.size();
        static_assert(Instance < PinConfig::pioCount(PinConfig::CurrentChip),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        static_assert(Sm < 4,
                      "a PIO instance has four state machines");
        static_assert(Offset + Length <= 32,
                      "a PIO program must fit the instance's 32 instruction slots from its "
                      "offset");
        using type
          = brigand::append<brigand::list<SmResource<Instance, Sm>>,
                            typename Detail::Instructions<Instance,
                                                          Offset,
                                                          programId<Program>(),
                                                          std::make_index_sequence<Length>>::type>;
    };

    // `using Provides = Kvasir::Pio::Provides<Instance, Sm, Offset, Program>;`
    template<unsigned Instance, unsigned Sm, unsigned Offset, typename Program>
    using Provides = typename ProvidesFor<Instance, Sm, Offset, Program>::type;

    // One of an instance's two interrupt lines (PIO0_IRQ_0 / _1 ...), for Pio::Irq.
    struct LineTag {};

    template<unsigned Instance, unsigned Line>
    using LineResource = Startup::Resource<LineTag, Instance * 2 + Line>;

    // GPIO function select of each instance: F6, F7, F8.
    template<unsigned Instance>
    constexpr int pinFunction = Instance == 0 ? 6
                              : Instance == 1 ? 7
                                              : 8;

    template<unsigned Instance>
    static constexpr auto getEnable() {
        static_assert(Instance < PinConfig::pioCount(PinConfig::CurrentChip),
                      "the RP2350 has PIO0..PIO2, the RP2040 PIO0 and PIO1");
        using Reset = typename Peripheral::RESETS::Registers<Instance * 0>::RESET;
        if constexpr(Instance == 0) {
            return clear(Reset::pio0);
        } else if constexpr(Instance == 1) {
            return clear(Reset::pio1);
        } else {
            return clear(Reset::pio2);
        }
    }

    template<unsigned Instance,
             typename Pin>
    static constexpr auto getPinConfig(Pin) {
        return action(Kvasir::Io::Action::PinFunction<pinFunction<Instance>>{}, Pin{});
    }

    // An input with the pad's pull-up, for a line that idles high (a UART's RX, an encoder).
    template<unsigned Instance,
             typename Pin>
    static constexpr auto getPinConfigPullUp(Pin) {
        return action(Kvasir::Io::Action::PinFunctionDrive<pinFunction<Instance>,
                                                           Io::DriveStrength::mA_4,
                                                           false,
                                                           Io::PullConfiguration::PullUp>{},
                      Pin{});
    }

    // CLKDIV is 16.8 fixed point: INT in [31:16] (0 means 65536), FRAC in [15:8]. The
    // integer part is not narrowed below the field's width, so a divider of 256 or more
    // (a slow bus from a fast clk_sys) keeps its value instead of wrapping.
    static constexpr auto getDiv(double div) {
        std::uint16_t div_int  = static_cast<std::uint16_t>(div);
        std::uint8_t  div_frac = static_cast<std::uint8_t>((div - div_int) * 256.0);
        return std::pair<std::uint16_t, std::uint8_t>(div_int, div_frac);
    }

    // The range every PIO driver's divider has to be in: at least 1.0, below what the INT
    // field holds. Checked here so a driver need not repeat it (ws2812 keeps its own,
    // tighter, timing asserts).
    template<typename SM>
    static constexpr bool divInRange(double div) {
        constexpr unsigned intBits = static_cast<unsigned>(
          std::popcount(Kvasir::Register::Detail::GetMask<
                        std::remove_cvref_t<decltype(SM::CLKDIV::_int)>>::value));
        return div >= 1.0 && div < static_cast<double>(1ULL << intBits);
    }

    template<typename SM,
             typename Div>
    static constexpr auto getDivConfig(Div) {
        static_assert(divInRange<SM>(Div{}()),
                      "PIO clock divider out of range: at least 1.0 and below 65536 (16.8 "
                      "fixed point) - check clockSpeed against the bus speed asked for");
        constexpr auto divs = getDiv(Div{}());

        return list(write(SM::CLKDIV::_int, Kvasir::Register::value<std::get<0>(divs)>()),
                    write(SM::CLKDIV::frac, Kvasir::Register::value<std::get<1>(divs)>()));
    }

    template<typename Dma,
             unsigned PioInstance,
             unsigned SmInstance>
    static constexpr typename Dma::TriggerSource getTxDmaTrigger() {
        if constexpr(PioInstance == 0) {
            if constexpr(SmInstance == 0) { return Dma::TriggerSource::pio0_tx0; }
            if constexpr(SmInstance == 1) { return Dma::TriggerSource::pio0_tx1; }
            if constexpr(SmInstance == 2) { return Dma::TriggerSource::pio0_tx2; }
            if constexpr(SmInstance == 3) { return Dma::TriggerSource::pio0_tx3; }
        } else {
            if constexpr(SmInstance == 0) { return Dma::TriggerSource::pio1_tx0; }
            if constexpr(SmInstance == 1) { return Dma::TriggerSource::pio1_tx1; }
            if constexpr(SmInstance == 2) { return Dma::TriggerSource::pio1_tx2; }
            if constexpr(SmInstance == 3) { return Dma::TriggerSource::pio1_tx3; }
        }
    }

    template<typename Dma,
             unsigned PioInstance,
             unsigned SmInstance>
    static constexpr typename Dma::TriggerSource getRxDmaTrigger() {
        if constexpr(PioInstance == 0) {
            if constexpr(SmInstance == 0) { return Dma::TriggerSource::pio0_rx0; }
            if constexpr(SmInstance == 1) { return Dma::TriggerSource::pio0_rx1; }
            if constexpr(SmInstance == 2) { return Dma::TriggerSource::pio0_rx2; }
            if constexpr(SmInstance == 3) { return Dma::TriggerSource::pio0_rx3; }
        } else {
            if constexpr(SmInstance == 0) { return Dma::TriggerSource::pio1_rx0; }
            if constexpr(SmInstance == 1) { return Dma::TriggerSource::pio1_rx1; }
            if constexpr(SmInstance == 2) { return Dma::TriggerSource::pio1_rx2; }
            if constexpr(SmInstance == 3) { return Dma::TriggerSource::pio1_rx3; }
        }
    }

}}   // namespace Kvasir::Pio
