// An RP pin's level (or output enable) written through two SIO registers in one apply() as must-fail cases
// (kvasir_conformance_must_fail, CMakeLists.txt): one step runs by register address, highest first, and GPIO_OUT_CLR sits
// above GPIO_OUT_SET on both chips, so `makeOutputInitHigh(pin)` (GPIO_OUT_SET) with `clear(pin)` (GPIO_OUT_CLR) ended
// HIGH. RegisterAlias.hpp makes the SIO's GPIO_OUT, GPIO_OE and their GPIO_HI_* twins shared states
// (Register::SharedState) and apply() refuses two of their registers on one pin in one step, naming the pin.
// PORT_FAIL=0 must compile: a Startup whose entries give the level in the configuring action (makeOutputInitHigh), a list
// with makeOutput and clear of one pin (one register), two pins' levels in one list (the pin actions write the field of
// every pin with one bit set: the others' zeros write nothing), a read beside a write, and makeOutput then set in two
// apply() calls or two steps. Each case must fail with the pin's name and what it writes (the case list in
// CMakeLists.txt): direct apply() lists, an entry's initStepPinConfig, two entries of one Startup phase, a peripheral
// pin, the output enable, a QSPI pin (GPIO_HI_*) and, on the RP2350, the Non-secure SIO view.

#include "kvasir_config.hpp"
// clang-format off: the SDK's Startup header before the chip, as main_tu.cpp
#include <kvasir/StartUp/StartUp.hpp>
// clang-format on

#include <chip/chip.hpp>
#include <kvasir/Runtime/BootReport.hpp>
#include <kvasir/Runtime/FaultHandler.hpp>

#ifndef PORT_FAIL
    #error "PORT_FAIL=<0..9> selects the case"
#endif

namespace {
template<int N>
using Gpio = decltype(Kvasir::Io::makePinLocation(Kvasir::Io::port0, Kvasir::Io::Pin<N>{}));

using LedPin = Gpio<17>;
using OutPin = Gpio<6>;
using CsPin  = Gpio<7>;
using InPin  = Gpio<2>;

using Sio = Kvasir::Peripheral::SIO::Registers<0>;
using Kvasir::Register::value;

constexpr auto clockSpeedValue = Conformance::clockSpeedValue;
constexpr auto crystalSpeed    = 12'000'000;
using ClockSettings = Kvasir::DefaultClockSettings::Settings<clockSpeedValue, crystalSpeed>;

// the GPIO blocks out of reset (every pin entry claims their gates)
struct GpioBlocks {
    static constexpr auto powerClockEnable
      = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::io_bank0),
             clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::pads_bank0));
};

// the level in the configuring action: high from the first store
struct LedPinConfig {
    static constexpr auto initStepPinConfig = list(makeOutputInitHigh(LedPin{}));
};

// low twice through GPIO_OUT_CLR (one register: legal), and another pin pulled up (GPIO_OE_CLR)
struct Pins {
    static constexpr auto initStepPinConfig
      = list(makeOutput(CsPin{}), clear(CsPin{}), action(Kvasir::Io::Action::inputPU, InPin{}));
};

// makeOutput, then the level in an apply() of its own, or after a sequencePoint; three pins' levels in one list; two
// pins in one whole-field write beside a third's clear; a read beside a write
struct Driven {
    static constexpr auto initStepPinConfig = list(makeOutput(OutPin{}));

    static void runtimeInit() {
        apply(set(OutPin{}));
        apply(makeOutput(OutPin{}), Kvasir::Register::sequencePoint, set(OutPin{}));
        apply(set(LedPin{}), clear(OutPin{}), toggle(CsPin{}));
        apply(write(Sio::GPIO_OUT_SET::gpio_out_set, value<(1U << 17) | (1U << 6)>()),
              clear(CsPin{}));
        [[maybe_unused]] auto const in = apply(read(InPin{}), set(OutPin{}));
    }
};

#if PORT_FAIL == 1
// makeOutputInitHigh then clear of one pin in one apply(): GPIO_OUT_CLR before GPIO_OUT_SET, the pin ends high
[[maybe_unused]] void refused() { apply(makeOutputInitHigh(LedPin{}), clear(LedPin{})); }

#elif PORT_FAIL == 2
// the same pair in an entry's initStepPinConfig
struct Extra {
    static constexpr auto initStepPinConfig = list(makeOutputInitHigh(Gpio<8>{}), clear(Gpio<8>{}));
};

#elif PORT_FAIL == 3
// the level of a pin another entry configures, in the same phase (one apply() of both)
struct Extra {
    static constexpr auto initStepPinConfig = list(set(OutPin{}));
};

#elif PORT_FAIL == 4
// makeOutput and set: GPIO_OUT_CLR then GPIO_OUT_SET - high, as written, but by the address order only
[[maybe_unused]] void refused() { apply(makeOutput(LedPin{}), set(LedPin{})); }

#elif PORT_FAIL == 5
// a toggle next to the configured level: GPIO_OUT_XOR runs first, the pin ends low
[[maybe_unused]] void refused() { apply(makeOutput(LedPin{}), toggle(LedPin{})); }

#elif PORT_FAIL == 6
// a peripheral pin (its OutputInit, low: GPIO_OUT_CLR) set high in the same list
[[maybe_unused]] void refused() {
    apply(
      action(Kvasir::Io::Action::PinFunctionDrive<1, Kvasir::Io::DriveStrength::mA4>{}, InPin{}),
      set(InPin{}));
}

#elif PORT_FAIL == 7
// an input (GPIO_OE_CLR) and its output enable set through the register (GPIO_OE_SET above it)
[[maybe_unused]] void refused() {
    apply(makeInput(OutPin{}), write(Sio::GPIO_OE_SET::gpio_oe_set, value<1U << 6>()));
}

#elif PORT_FAIL == 8
// a QSPI pin's level through GPIO_HI_OUT_SET and _CLR: SCLK (RP2040 bit 0, Table 19) / SCK (RP2350 bit 26, Table 22)
    #ifdef KVASIR_CHIP_RP2040
constexpr unsigned qspiSck = 1U << 0;
    #else
constexpr unsigned qspiSck = 1U << 26;
    #endif
[[maybe_unused]] void refused() {
    apply(write(Sio::GPIO_HI_OUT_SET::FULLREGISTER, value<qspiSck>()),
          write(Sio::GPIO_HI_OUT_CLR::FULLREGISTER, value<qspiSck>()));
}

#elif PORT_FAIL == 9
// RP2350: the Non-secure SIO view (0xd0020000) writes the same GPIO_OUT
[[maybe_unused]] void refused() {
    apply(
      write(Kvasir::Peripheral::SIO::Registers<1>::GPIO_OUT_SET::gpio_out_set, value<1U << 17>()),
      clear(LedPin{}));
}
#endif
}   // namespace

using Startup = Kvasir::Startup::Startup<ClockSettings,
                                         Conformance::SystickClock,
                                         uc_log::ComBackend<uc_log::Tag::User>,
                                         Kvasir::Boot::Report<>,
                                         Kvasir::Fault::HandlerFor<>,
                                         GpioBlocks,
                                         LedPinConfig,
                                         Pins,
#if PORT_FAIL == 2 || PORT_FAIL == 3
                                         Extra,
#endif
                                         Driven>;

int main() {
    while(true) { Startup::runChipEntries<Kvasir::Hook::MainLoop>(); }
}

template struct Kvasir::Startup::Start<Startup>;
