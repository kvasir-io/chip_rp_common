#pragma once

#include "PinConfig.hpp"
#include "chip/PinConfig.hpp"
#include "kvasir/Hal/Pins.hpp"
#include "kvasir/Io/Claims.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "peripherals/IO_BANK0.hpp"
#include "peripherals/PADS_BANK0.hpp"
#include "peripherals/SIO.hpp"

#include <array>
#include <concepts>
#include <type_traits>

namespace Kvasir { namespace Io {
    // Use chip-agnostic pin configuration

    template<Chip::Part P>
    struct PinLocationTraits {
        static constexpr int portBegin = 0;
        static constexpr int portEnd   = 1;
        static constexpr int pinBegin  = 0;
        static constexpr int pinEnd    = PinConfig::ChipTraits<P>::pinCount;
        // one port, every GPIO bonded: 30 on the RP2040 and RP2350A, 48 on the RP2350B
        static constexpr std::array<std::uint64_t, 1> bonded{Hal::Pins::range(pinBegin, pinEnd)};
    };

    template<int Port,
             int Pin>
    constexpr bool isValidPinLocation() {
        return Hal::Pins::isBonded<PinLocationTraits<Chip::part>>(Port, Pin);
    }

    using SIO = Kvasir::Peripheral::SIO::Registers<0>;

    template<int Pin>
    constexpr auto getClearAction() {
        if constexpr(Pin <= 31) {
            return write(SIO::GPIO_OUT_CLR::gpio_out_clr, Register::value<1U << Pin>());
        } else {
            return write(SIO::GPIO_HI_OUT_CLR::gpio, Register::value<1U << (Pin - 32)>());
        }
    }

    template<int Pin>
    constexpr auto getSetAction() {
        if constexpr(Pin <= 31) {
            return write(SIO::GPIO_OUT_SET::gpio_out_set, Register::value<1U << Pin>());
        } else {
            return write(SIO::GPIO_HI_OUT_SET::gpio, Register::value<1U << (Pin - 32)>());
        }
    }

    template<int Pin>
    constexpr auto getToggleAction() {
        if constexpr(Pin <= 31) {
            return write(SIO::GPIO_OUT_XOR::gpio_out_xor, Register::value<1U << Pin>());
        } else {
            return write(SIO::GPIO_HI_OUT_XOR::gpio, Register::value<1U << (Pin - 32)>());
        }
    }

    template<int Pin>
    constexpr auto getOeSetAction() {
        if constexpr(Pin <= 31) {
            return write(SIO::GPIO_OE_SET::gpio_oe_set, Register::value<1U << Pin>());
        } else {
            return write(SIO::GPIO_HI_OE_SET::gpio, Register::value<1U << (Pin - 32)>());
        }
    }

    template<int Pin>
    constexpr auto getOeClrAction() {
        if constexpr(Pin <= 31) {
            return write(SIO::GPIO_OE_CLR::gpio_oe_clr, Register::value<1U << Pin>());
        } else {
            return write(SIO::GPIO_HI_OE_CLR::gpio, Register::value<1U << (Pin - 32)>());
        }
    }

    // the pin's bit of the generated GPIO_IN / GPIO_HI_IN field, read as a bool
    template<int Pin>
    constexpr auto getReadAction() {
        if constexpr(Pin <= 31) {
            return Register::read(
              Register::bit<static_cast<unsigned>(Pin), bool>(SIO::GPIO_IN::gpio_in));
        } else {
#ifdef KVASIR_CHIP_RP2350
            return Register::read(
              Register::bit<static_cast<unsigned>(Pin - 32), bool>(SIO::GPIO_HI_IN::gpio));
#else
            static_assert(Pin <= 31, "no GPIO above 31 on the RP2040 (GPIO_IN holds them all)");
#endif
        }
    }

    template<int Port, int Pin>
    struct MakeAction<Action::Clear, PinLocation<Port, Pin>>
      : decltype(MPL::list(getClearAction<Pin>())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Set, PinLocation<Port, Pin>>
      : decltype(MPL::list(getSetAction<Pin>())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Toggle, PinLocation<Port, Pin>>
      : decltype(MPL::list(getToggleAction<Pin>())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Read, PinLocation<Port, Pin>>
      : decltype(MPL::list(getReadAction<Pin>())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    namespace detail {
        template<int Pin>
        using Pad = typename Kvasir::Peripheral::PADS_BANK0::Registers<>::GPIO<Pin>::PAD;
        template<int Pin>
        using Ctrl = typename Kvasir::Peripheral::IO_BANK0::Registers<>::GPIO<Pin>::CTRL;

    }   // namespace detail

    enum class DriveStrength { mA2, mA4, mA8, mA12 };

    // Pad reset default is 4 mA with slow slew — OutputSpeed::Low maps to exactly
    // that; higher speeds increase drive strength and enable fast slew.
    constexpr DriveStrength driveFromOutputSpeed(OutputSpeed os) {
        return os == OutputSpeed::low    ? DriveStrength::mA4
             : os == OutputSpeed::medium ? DriveStrength::mA4
             : os == OutputSpeed::high   ? DriveStrength::mA8
                                         : DriveStrength::mA12;
    }

    constexpr bool slewFastFromOutputSpeed(OutputSpeed os) { return os != OutputSpeed::low; }

    template<int           Pin,
             DriveStrength DS>
    constexpr auto getDriveAction() {
        if constexpr(DS == DriveStrength::mA2) {
            return write(detail::Pad<Pin>::DRIVEValC::_2ma);
        } else if constexpr(DS == DriveStrength::mA4) {
            return write(detail::Pad<Pin>::DRIVEValC::_4ma);
        } else if constexpr(DS == DriveStrength::mA8) {
            return write(detail::Pad<Pin>::DRIVEValC::_8ma);
        } else {
            return write(detail::Pad<Pin>::DRIVEValC::_12ma);
        }
    }

    // GPIOn_CTRL's OUTOVER / OEOVER for a pin a peripheral drives: the peripheral's signal as
    // it is, or inverted (pico-sdk gpio_set_outover / gpio_set_oeover with
    // GPIO_OVERRIDE_INVERT: a clock with the other idle level, an open-drain line driven
    // through its output enable).
    enum class PinOverride { normal, invert };

    namespace Action {
        template<int               Function,
                 DriveStrength     DS,
                 bool              SlewFast = false,
                 PullConfiguration PC       = PullConfiguration::pullNone,
                 OutputInit        OI       = OutputInit::low,
                 PinOverride       OutOver  = PinOverride::normal,
                 PinOverride       OeOver   = PinOverride::normal>
        struct PinFunctionDrive {
            static constexpr int value = Function;
        };
    }   // namespace Action

    template<Io::PullConfiguration PC, int Port, int Pin>
    struct MakeAction<Action::Input<PC>, PinLocation<Port, Pin>>
      : decltype(MPL::list(
          set(detail::Pad<Pin>::ie),
#ifdef KVASIR_CHIP_RP2350
          clear(detail::Pad<Pin>::iso),
#endif
          set(detail::Pad<Pin>::od),
          write(detail::Pad<Pin>::DRIVEValC::_2ma),
          write(detail::Pad<Pin>::pde,
                Register::value<PC == PullConfiguration::pullDown ? 1 : 0>()),
          write(detail::Pad<Pin>::pue, Register::value<PC == PullConfiguration::pullUp ? 1 : 0>()),
          clear(detail::Pad<Pin>::slewfast),
          set(detail::Pad<Pin>::schmitt),
          write(detail::Ctrl<Pin>::IRQOVERValC::normal),
          write(detail::Ctrl<Pin>::INOVERValC::normal),
          write(detail::Ctrl<Pin>::OEOVERValC::disable),
          write(detail::Ctrl<Pin>::OUTOVERValC::normal),
          write(detail::Ctrl<Pin>::FUNCSELValC::sio),
          getOeClrAction<Pin>())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<Io::OutputType OT, Io::OutputSpeed OS, Io::OutputInit OI, int Port, int Pin>
    struct MakeAction<Action::Output<OT, OS, OI>, PinLocation<Port, Pin>>
      : decltype(MPL::list(
          set(detail::Pad<Pin>::ie),
#ifdef KVASIR_CHIP_RP2350
          clear(detail::Pad<Pin>::iso),
#endif
          clear(detail::Pad<Pin>::od),
          getDriveAction<Pin, driveFromOutputSpeed(OS)>(),
          clear(detail::Pad<Pin>::pde),
          clear(detail::Pad<Pin>::pue),
          write(detail::Pad<Pin>::slewfast, Register::value<slewFastFromOutputSpeed(OS) ? 1 : 0>()),
          set(detail::Pad<Pin>::schmitt),
          write(detail::Ctrl<Pin>::IRQOVERValC::normal),
          write(detail::Ctrl<Pin>::INOVERValC::normal),
          write(detail::Ctrl<Pin>::OEOVERValC::normal),
          write(detail::Ctrl<Pin>::OUTOVERValC::normal),
          write(detail::Ctrl<Pin>::FUNCSELValC::sio),
          getOeSetAction<Pin>(),
          []() {
              if constexpr(OI == Io::OutputInit::low) {
                  return getClearAction<Pin>();
              } else {
                  return getSetAction<Pin>();
              }
          }())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(OT == Io::OutputType::pushPull, "only push pull supported");
    };

    template<DriveStrength     DS,
             bool              SlewFast,
             Io::OutputInit    OI,
             PullConfiguration PC,
             PinOverride       OutOver,
             PinOverride       OeOver,
             int               Port,
             int               Pin,
             int               Function>
    struct MakeAction<Action::PinFunctionDrive<Function, DS, SlewFast, PC, OI, OutOver, OeOver>,
                      PinLocation<Port, Pin>>
      : decltype(MPL::list(
          set(detail::Pad<Pin>::ie),
#ifdef KVASIR_CHIP_RP2350
          clear(detail::Pad<Pin>::iso),
#endif
          clear(detail::Pad<Pin>::od),
          getDriveAction<Pin, DS>(),
          write(detail::Pad<Pin>::pde,
                Register::value<PC == PullConfiguration::pullDown ? 1 : 0>()),
          write(detail::Pad<Pin>::pue, Register::value<PC == PullConfiguration::pullUp ? 1 : 0>()),
          write(detail::Pad<Pin>::slewfast, Register::value<SlewFast ? 1 : 0>()),
          set(detail::Pad<Pin>::schmitt),
          write(detail::Ctrl<Pin>::IRQOVERValC::normal),
          write(detail::Ctrl<Pin>::INOVERValC::normal),
          []() {
              if constexpr(OeOver == PinOverride::invert) {
                  return write(detail::Ctrl<Pin>::OEOVERValC::invert);
              } else {
                  return write(detail::Ctrl<Pin>::OEOVERValC::normal);
              }
          }(),
          []() {
              if constexpr(OutOver == PinOverride::invert) {
                  return write(detail::Ctrl<Pin>::OUTOVERValC::invert);
              } else {
                  return write(detail::Ctrl<Pin>::OUTOVERValC::normal);
              }
          }(),
          write(detail::Ctrl<Pin>::funcsel,
                Register::value<typename detail::Ctrl<Pin>::FUNCSELVal,
                                static_cast<typename detail::Ctrl<Pin>::FUNCSELVal>(Function)>()),
          getOeSetAction<Pin>(),
          []() {
              if constexpr(OI == Io::OutputInit::low) {
                  return getClearAction<Pin>();
              } else {
                  return getSetAction<Pin>();
              }
          }())) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<Io::OutputType    OT,
             Io::OutputSpeed   OS,
             Io::OutputInit    OI,
             PullConfiguration PC,
             int               Port,
             int               Pin,
             int               Function>
    struct MakeAction<Action::PinFunction<Function, OT, OS, OI, PC>, PinLocation<Port, Pin>>
      : MakeAction<Action::PinFunctionDrive<Function,
                                            driveFromOutputSpeed(OS),
                                            slewFastFromOutputSpeed(OS),
                                            PC,
                                            OI>,
                   PinLocation<Port, Pin>> {
        static_assert(OT == Io::OutputType::pushPull,
                      "only push pull supported");
    };

    // A peripheral pin of a driver config (sdaPinLocation, misoPinLocation, ...): the check that the pin can have its
    // role on this instance, and its set-up - the role's function select (PinConfig::functionSelect), pad drive,
    // slew and pull. An Io::NotUsed<> location (an optional pin left out) passes and sets nothing up.
    template<PinConfig::Function F,
             unsigned            Instance,
             typename T>
    constexpr bool pinServes(NotUsed<T>) {
        return true;
    }

    template<PinConfig::Function F,
             unsigned            Instance,
             int                 Port,
             int                 Pin>
    constexpr bool pinServes(PinLocation<Port,
                                         Pin>) {
        return PinConfig::CheckedPin<Chip::part, F, Instance, Pin>::value;
    }

    namespace Detail {
        template<typename Location,
                 PinConfig::Function F,
                 unsigned            Instance,
                 DriveStrength       Drive    = DriveStrength::mA4,
                 bool                SlewFast = false,
                 PullConfiguration   Pull     = PullConfiguration::pullNone>
        struct PinConfigFor;

        template<typename T,
                 PinConfig::Function F,
                 unsigned            Instance,
                 DriveStrength       Drive,
                 bool                SlewFast,
                 PullConfiguration   Pull>
        struct PinConfigFor<NotUsed<T>, F, Instance, Drive, SlewFast, Pull> {
            using type = brigand::list<>;
        };

        template<int                 Port,
                 int                 Pin,
                 PinConfig::Function F,
                 unsigned            Instance,
                 DriveStrength       Drive,
                 bool                SlewFast,
                 PullConfiguration   Pull>
        struct PinConfigFor<PinLocation<Port, Pin>, F, Instance, Drive, SlewFast, Pull> {
            static constexpr int funcSel = PinConfig::functionSelect(Chip::part, F, Instance, Pin);
            static_assert(PinConfig::CheckedPin<Chip::part,
                                                F,
                                                Instance,
                                                Pin>::value);
            using type = decltype(action(Action::PinFunctionDrive<funcSel, Drive, SlewFast, Pull>{},
                                         PinLocation<Port, Pin>{}));
        };
    }   // namespace Detail

    // The pins a peripheral configures, derived from what it writes: every pin path above
    // (and the ADC's) writes IO_BANK0's GPIOn_CTRL, so an action on that register is the
    // peripheral taking GPIO n. Startup collects these as the peripheral's provided
    // resources; two peripherals on one pin, or a driver claiming a pin nobody configures,
    // are then build errors (kvasir/StartUp/Resources.hpp).
    namespace Detail {
        using IoBank = Kvasir::Peripheral::IO_BANK0::Registers<>;

        constexpr int gpioOfCtrlAddress(unsigned address) {
            constexpr unsigned base   = IoBank::baseAddr;
            constexpr unsigned stride = 8;
            constexpr unsigned ctrl   = 4;
            constexpr unsigned pins   = PinConfig::ChipTraits<Chip::part>::pinCount;
            if(address < base + ctrl) { return -1; }
            unsigned const offset = address - base - ctrl;
            if(offset % stride != 0) { return -1; }
            if(offset / stride >= pins) { return -1; }
            return static_cast<int>(offset / stride);
        }

        static_assert(gpioOfCtrlAddress(IoBank::GPIO<0>::CTRL::Addr::value) == 0);
        static_assert(gpioOfCtrlAddress(IoBank::GPIO<22>::CTRL::Addr::value) == 22);
        static_assert(gpioOfCtrlAddress(IoBank::GPIO<22>::STATUS::Addr::value) == -1);
    }   // namespace Detail
}}   // namespace Kvasir::Io

namespace Kvasir { namespace Startup {
    template<unsigned Addr,
             unsigned Z,
             unsigned O,
             typename RegType,
             typename Mode,
             unsigned Mask,
             typename Access,
             typename FieldType,
             typename TAction>
        requires(Io::Detail::gpioOfCtrlAddress(Addr) >= 0)
    struct ResourceOfAction<Register::Action<
      Register::
        FieldLocation<Register::Address<Addr, Z, O, RegType, Mode>, Mask, Access, FieldType>,
      TAction>> {
        using type = brigand::list<Io::PinResource<0, Io::Detail::gpioOfCtrlAddress(Addr)>>;
    };
}}   // namespace Kvasir::Startup
