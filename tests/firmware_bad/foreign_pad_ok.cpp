// expect-ok
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/PioStateMachine.hpp>
#include <chip/rp_common/pio/Asm.hpp>
#include <type_traits>

// a foreign in pin and a foreign jmp pin next to an own input: only the own one gets a pin
// action, the foreign ones are claimed (their owner provides them), and the synchroniser bypass,
// the PIO's own register, still reaches a foreign pad
struct P : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
    a.wait(1, Kvasir::Pio::Wait::pin, 0);
})> {};

struct C {
    static constexpr auto ClockSpeed      = HW::ClockSpeed;
    static constexpr auto PioInstance     = 0;
    static constexpr auto SmInstance      = 0;
    static constexpr auto inPins          = brigand::list<HW::Pin::clk_out>{};
    static constexpr auto jmpPin          = brigand::list<HW::Pin::uart_rx>{};
    static constexpr auto setPins         = brigand::list<HW::Pin::led>{};
    static constexpr auto foreignPads     = brigand::list<HW::Pin::clk_out, HW::Pin::uart_rx>{};
    static constexpr auto inputSyncBypass = brigand::list<HW::Pin::clk_out>{};
    static constexpr bool inPullUp        = true;
};

using Sm = Kvasir::Pio::StateMachine<P, C>;

// one pin action: the set pin's
static_assert(brigand::size<std::remove_cvref_t<decltype(Sm::initStepPinConfig)>>::value == 1);
static_assert(std::is_same_v<Sm::OwnInputPins,
                             brigand::list<>>);

template<typename Pin>
using ResourceOf = brigand::front<Kvasir::Io::PinClaims<Pin>>;

template<typename Pin>
constexpr bool claimed
  = brigand::any<Sm::Claims, std::is_same<brigand::_1, ResourceOf<Pin>>>::value;

static_assert(claimed<HW::Pin::clk_out> && claimed<HW::Pin::uart_rx> && !claimed<HW::Pin::led>);
static_assert(Sm::SyncBypassMask != 0);
