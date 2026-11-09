// The SIO's shared states (RegisterAlias.hpp, Register::SharedState) of the chip this TU is built for (KVASIR_CHIP_RP2040,
// else the RP2350; CMakeLists.txt builds it for both): each GPIO register's state against the datasheet's register list,
// the plan's refusal of one pin's level or output enable through two of them in one step, the legal lists on the mock,
// and the message's words (the pins by name, the registers). The registers are mock ones at the SIO's addresses, shaped
// like the generated ones (write-only, one-to-set / -clear / -toggle, write-ignored-if-zero); the chips' generated SIO
// meets the same literals in Io.hpp's static_assert, the chips' own builds in the conformance tree's must-fail cases.
#include "RegisterAlias.hpp"
#include "kvasir/Register/Diagnostic.hpp"
#include "test_registers.hpp"

#include <string_view>

using namespace Kvasir::Register;
using namespace Kvasir::Test;   // Field, W: kvasir_test.hpp
namespace S = Kvasir::Rp::SioSeam;
namespace D = Kvasir::Register::Detail;

namespace {
constexpr unsigned sio = 0xd0000000;

#ifdef KVASIR_CHIP_RP2040
// RP2040 datasheet 2.3.1.7 Table 16
constexpr unsigned out = 0x010, outSet = 0x014, outClr = 0x018, outXor = 0x01c;
constexpr unsigned oe = 0x020, oeSet = 0x024, oeClr = 0x028, oeXor = 0x02c;
constexpr unsigned hiOut = 0x030, hiOutSet = 0x034, hiOutClr = 0x038, hiOutXor = 0x03c;
constexpr unsigned hiOe = 0x040, hiOeSet = 0x044, hiOeClr = 0x048, hiOeXor = 0x04c;
// GPIO_HI_*: QSPI SCLK in bit 0 (Table 19)
constexpr unsigned qspiSck     = 1U << 0;
constexpr auto     qspiSckName = std::string_view{"RP SIO QSPI_SCLK, its output level"};
#else
// RP2350 datasheet 3.1.11 Table 17
constexpr unsigned out = 0x010, outSet = 0x018, outClr = 0x020, outXor = 0x028;
constexpr unsigned oe = 0x030, oeSet = 0x038, oeClr = 0x040, oeXor = 0x048;
constexpr unsigned hiOut = 0x014, hiOutSet = 0x01c, hiOutClr = 0x024, hiOutXor = 0x02c;
constexpr unsigned hiOe = 0x034, hiOeSet = 0x03c, hiOeClr = 0x044, hiOeXor = 0x04c;
// GPIO_HI_*: QSPI_SCK in bit 26 (Table 22)
constexpr unsigned qspiSck     = 1U << 26;
constexpr auto     qspiSckName = std::string_view{"RP SIO QSPI_SCK, its output level"};
#endif

// every GPIO register writes its state's own register, the others none
static_assert(S::stateOf(sio + out) == sio + out && S::stateOf(sio + outSet) == sio + out
              && S::stateOf(sio + outClr) == sio + out && S::stateOf(sio + outXor) == sio + out);
static_assert(S::stateOf(sio + oe) == sio + oe && S::stateOf(sio + oeSet) == sio + oe
              && S::stateOf(sio + oeClr) == sio + oe && S::stateOf(sio + oeXor) == sio + oe);
static_assert(S::stateOf(sio + hiOut) == sio + hiOut && S::stateOf(sio + hiOutSet) == sio + hiOut
              && S::stateOf(sio + hiOutClr) == sio + hiOut
              && S::stateOf(sio + hiOutXor) == sio + hiOut);
static_assert(S::stateOf(sio + hiOe) == sio + hiOe && S::stateOf(sio + hiOeSet) == sio + hiOe
              && S::stateOf(sio + hiOeClr) == sio + hiOe
              && S::stateOf(sio + hiOeXor) == sio + hiOe);
// CPUID, GPIO_IN, GPIO_HI_IN, FIFO_ST; an unaligned address; IO_BANK0's GPIO0_CTRL (an aliased block)
static_assert(S::stateOf(sio) == 0 && S::stateOf(sio + 0x004) == 0 && S::stateOf(sio + 0x008) == 0
              && S::stateOf(sio + 0x050) == 0 && S::stateOf(sio + 0x012) == 0
              && S::stateOf(0x40014004) == 0);
// the state is the lowest address that writes it (Register/Exec.hpp's contract: apply() looks from there up)
static_assert(out < outSet && out < outClr && out < outXor && hiOe < hiOeSet && hiOe < hiOeXor);
#ifndef KVASIR_CHIP_RP2040
// the Non-secure view (SIO_NONSEC_BASE 0xd0020000, 3.1.1) writes the same GPIO registers (3.1.3)
static_assert(S::stateOf(0xd0020000 + outClr) == sio + out
              && S::stateOf(0xd0020000 + hiOeXor) == sio + hiOe
              && S::stateOf(0xd0020000 + 0x004) == 0);
#endif

using WoSet
  = Access<AccessType::writeOnly, ReadActionType::normal, ModifiedWriteValueType::oneToSet>;
using WoClr
  = Access<AccessType::writeOnly, ReadActionType::normal, ModifiedWriteValueType::oneToClear>;
using WoXor
  = Access<AccessType::writeOnly, ReadActionType::normal, ModifiedWriteValueType::oneToToggle>;

template<unsigned Offset, typename Acc>
struct SioReg {
    using Addr = Address<sio + Offset, 0xFFFFFFFF, 0x00000000, std::uint32_t>;
    static constexpr Field<Addr, 31, 0, Acc> pins{};
};

using OutSet   = SioReg<outSet, WoSet>;
using OutClr   = SioReg<outClr, WoClr>;
using OutXor   = SioReg<outXor, WoXor>;
using OeSet    = SioReg<oeSet, WoSet>;
using OeClr    = SioReg<oeClr, WoClr>;
using HiOutSet = SioReg<hiOutSet, WoSet>;
using HiOutClr = SioReg<hiOutClr, WoClr>;

// what Io.hpp's pin actions write: the field of every pin, the pin's bit set
template<typename R, unsigned Pin>
using PinWrite = decltype(write(R::pins, value<1U << Pin>()));

template<typename... Args>
constexpr D::MergeError errorOf = D::PlanOf<Args...>::plan.error;

using E = D::MergeError;

// makeOutputInitHigh + clear (GPIO_OUT_SET, GPIO_OUT_CLR) of one pin: refused, the clear (above) first
static_assert(errorOf<PinWrite<OutSet,
                               17>,
                      PinWrite<OutClr,
                               17>>
              == E::oneStateTwoRegisters);
static_assert(D::PlanOf<PinWrite<OutSet,
                                 17>,
                        PinWrite<OutClr,
                                 17>>::plan.conflict.first
                == 1
              && D::PlanOf<PinWrite<OutSet,
                                    17>,
                           PinWrite<OutClr,
                                    17>>::plan.conflict.mask
                   == 1U << 17);
// a toggle beside a level, the output enable set and cleared, a QSPI pin
static_assert(errorOf<PinWrite<OutClr,
                               3>,
                      PinWrite<OutXor,
                               3>>
              == E::oneStateTwoRegisters);
static_assert(errorOf<PinWrite<OeClr,
                               6>,
                      PinWrite<OeSet,
                               6>>
              == E::oneStateTwoRegisters);
static_assert(errorOf<decltype(write(HiOutSet::pins,
                                     value<qspiSck>())),
                      decltype(write(HiOutClr::pins,
                                     value<qspiSck>()))>
              == E::oneStateTwoRegisters);
// legal: two pins (the zeros of the field write nothing), level and enable of one pin (two states), the low and the
// high bank's bit of one number, two steps
static_assert(errorOf<PinWrite<OutSet,
                               17>,
                      PinWrite<OutClr,
                               6>,
                      PinWrite<OutXor,
                               7>>
              == E::none);
static_assert(errorOf<PinWrite<OutClr,
                               6>,
                      PinWrite<OeSet,
                               6>>
              == E::none);
static_assert(errorOf<PinWrite<OutSet,
                               0>,
                      PinWrite<HiOutClr,
                               0>>
              == E::none);
static_assert(errorOf<PinWrite<OutSet,
                               17>,
                      SequencePoint,
                      PinWrite<OutClr,
                               17>>
              == E::none);
#ifndef KVASIR_CHIP_RP2040
struct NsOutSet {
    using Addr = Address<0xd0020000 + outSet, 0xFFFFFFFF, 0x00000000, std::uint32_t>;
    static constexpr Field<Addr, 31, 0, WoSet> pins{};
};

static_assert(errorOf<PinWrite<NsOutSet,
                               17>,
                      PinWrite<OutClr,
                               17>>
              == E::oneStateTwoRegisters);
#endif

// the message (Register/Diagnostic.hpp OneStateTwoRegisters with the chip's describe)
template<typename First, typename Last, unsigned Mask>
constexpr std::string_view messageOf
  = Kvasir::Diagnostic::Message<
      Diagnostic::OneStateTwoRegisters<RegisterTraits<typename First::Addr>,
                                       RegisterTraits<typename Last::Addr>,
                                       Mask,
                                       SharedState<Kvasir::Tag::User, typename First::Addr>>>{}
      .view();

constexpr bool has(std::string_view text,
                   std::string_view part) {
    return text.find(part) != std::string_view::npos;
}

constexpr auto levelText = messageOf<OutClr, OutSet, 1U << 17>;
static_assert(has(levelText,
                  "RP SIO GPIO17, its output level: makeOutput(pin) and clear(pin) write it low "
                  "(GPIO_OUT_CLR), makeOutputInitHigh(pin) and set(pin) high (GPIO_OUT_SET)"));
static_assert(has(levelText,
                  "or drive the pin in an apply() of its own")
              && levelText.size() < 768);
static_assert(has(messageOf<OutClr,
                            OutSet,
                            (1U << 6) | (1U << 7)>,
                  "RP SIO GPIO6, GPIO7, its output level"));
static_assert(
  has(messageOf<OeClr,
                OeSet,
                1U << 6>,
      "RP SIO GPIO6, its output enable: makeOutput(pin) and a peripheral pin set it (GPIO_OE_SET), "
      "makeInput(pin) clears it (GPIO_OE_CLR)"));
static_assert(has(messageOf<HiOutClr,
                            HiOutSet,
                            qspiSck>,
                  qspiSckName)
              && has(messageOf<HiOutClr,
                               HiOutSet,
                               qspiSck>,
                     "(GPIO_HI_OUT_CLR)"));
#ifndef KVASIR_CHIP_RP2040
static_assert(has(messageOf<HiOutClr,
                            HiOutSet,
                            1U << 3>,
                  "RP SIO GPIO35, its output level"));
#endif

// the legal lists run as before: one store per register, address descending
void twoPinsOneStep() {
    test("twoPinsOneStep");
    apply(write(OutSet::pins, value<1U << 17>()), write(OutClr::pins, value<1U << 6>()));
    checkActions({
      W{OutClr::Addr::value,  1U << 6},
      W{OutSet::Addr::value, 1U << 17}
    });
}

void onePinTwoSteps() {
    test("onePinTwoSteps");
    apply(write(OutSet::pins, value<1U << 17>()),
          sequencePoint,
          write(OutClr::pins, value<1U << 17>()));
    checkActions({
      W{OutSet::Addr::value, 1U << 17},
      W{OutClr::Addr::value, 1U << 17}
    });
}
}   // namespace

int main() {
    twoPinsOneStep();
    onePinTwoSteps();
    return Kvasir::Test::report();
}
