#pragma once
#include "Clocks.hpp"
#include "Io.hpp"
#include "PinConfig.hpp"
#include "chip/Interrupt.hpp"
#include "core/Nvic.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/CLOCKS.hpp"

#include <bit>
#include <cstdint>

// Two small clock-block peripherals for a Startup list.
//
// Clocks::Output<Pin, Source, Div>: one of the four GPOUT generators on its pin, a clock
// source divided by a 16.16 divider. The RP2350A's GPOUT pins are GPIO 13 / 21 (GPOUT0),
// 15 / 23 (GPOUT1), 24 (GPOUT2), 25 (GPOUT3); another pin is a build error.
//
//   using ClockOut = Kvasir::Clocks::Output<HW::Pin::clk_out, Kvasir::Clocks::Source::clkSys, 10>;
//
// Clocks::Resus<Config, &callback>: the resuscitation of clk_sys. If the system clock
// stops (a PLL powered down, a source lost) for `timeout` clk_ref cycles, the hardware
// switches clk_sys to clk_ref and raises the `clocks` interrupt; the callback repairs the
// clock tree (HW::ClockSettings::coreClockInit(), typically) and the peripheral acknowledges
// the event. Config: timeout (255; 0..255 clk_ref cycles / 2), isrPriority (2).
namespace Kvasir { namespace Clocks {

    // What a GPOUT can carry. lposc, clkPeri and clkHstx exist on the RP2350 only.
    enum class Source : std::uint32_t {
        pllSys,
        gpin0,
        gpin1,
        pllUsb,
        rosc,
        xosc,
        lposc,
        clkSys,
        clkUsb,
        clkAdc,
        clkRef,
        clkPeri,
        clkHstx,
    };

    namespace detail {
        template<int Port,
                 int Pin>
        constexpr int pinNumber(Register::PinLocation<Port,
                                                      Pin>) {
            return Pin;
        }

        constexpr int gpoutOf(int pin) { return PinConfig::gpoutOf(PinConfig::CurrentChip, pin); }

        // The chip's AUXSRC value for a Source, by name: the two chips number them
        // differently and the RP2040 lacks the LPOSC, HSTX and OTP sources (it has clk_rtc).
        template<typename Ctrl>
        constexpr auto auxsrcOf(Source s) {
            using V = typename Ctrl::AUXSRCVal;
            switch(s) {
            case Source::pllSys: return V::clksrc_pll_sys;
            case Source::gpin0:  return V::clksrc_gpin0;
            case Source::gpin1:  return V::clksrc_gpin1;
            case Source::pllUsb: return V::clksrc_pll_usb;
            case Source::rosc:   return V::rosc_clksrc;
            case Source::xosc:   return V::xosc_clksrc;
            case Source::clkSys: return V::clk_sys;
            case Source::clkUsb: return V::clk_usb;
            case Source::clkAdc: return V::clk_adc;
            case Source::clkRef: return V::clk_ref;
            default:             return V::clk_sys;   // narrowed below by the static_asserts
            }
        }

        template<typename Ctrl>
        constexpr auto auxsrcOfExtra(Source s) {
#if __has_include("chip/rp2350.hpp")
            using V = typename Ctrl::AUXSRCVal;
            switch(s) {
            case Source::lposc:   return V::lposc_clksrc;
            case Source::clkPeri: return V::clk_peri;
            case Source::clkHstx: return V::clk_hstx;
            default:              return auxsrcOf<Ctrl>(s);
            }
#else
            return auxsrcOf<Ctrl>(s);
#endif
        }
    }   // namespace detail

    template<typename Pin, Source Src, unsigned DivInt, unsigned DivFrac16 = 0>
    struct Output {
        static constexpr int PinNumber = detail::pinNumber(Pin{});
        static constexpr int Gpout     = detail::gpoutOf(PinNumber);
        static_assert(Gpout >= 0,
                      "not a GPOUT pin: GPIO 21 (GPOUT0), 23 (GPOUT1), 24 (GPOUT2), 25 (GPOUT3); "
                      "on the RP2350 also 13 (GPOUT0) and 15 (GPOUT1)");
        static_assert(!PinConfig::isRp2040(PinConfig::CurrentChip)
                        || (Src != Source::lposc && Src != Source::clkPeri
                            && Src != Source::clkHstx),
                      "the RP2040 has no LPOSC, clk_peri or clk_hstx output");

        using Clk  = Kvasir::Peripheral::CLOCKS::Registers<>;
        using Ctrl = std::conditional_t<
          Gpout == 0,
          typename Clk::CLK_GPOUT0_CTRL,
          std::conditional_t<Gpout == 1,
                             typename Clk::CLK_GPOUT1_CTRL,
                             std::conditional_t<Gpout == 2,
                                                typename Clk::CLK_GPOUT2_CTRL,
                                                typename Clk::CLK_GPOUT3_CTRL>>>;
        using Div = std::conditional_t<
          Gpout == 0,
          typename Clk::CLK_GPOUT0_DIV,
          std::conditional_t<Gpout == 1,
                             typename Clk::CLK_GPOUT1_DIV,
                             std::conditional_t<Gpout == 2,
                                                typename Clk::CLK_GPOUT2_DIV,
                                                typename Clk::CLK_GPOUT3_DIV>>>;

        // The clock-output GPIO function: F9 on the RP2350, F8 on the RP2040.
        static constexpr auto initStepPinConfig = list(action(
          Kvasir::Io::Action::PinFunctionDrive<PinConfig::gpoutFunction(PinConfig::CurrentChip),
                                               Io::DriveStrength::mA_8,
                                               true>{},
          Pin{}));

        // The divider's fields: 16.16 on the RP2350, 24.8 on the RP2040. The fraction is
        // given in 1/65536 and scaled to the field's width.
        static constexpr unsigned FracBits = static_cast<unsigned>(std::popcount(
          Register::Detail::GetMask<std::remove_cvref_t<decltype(Div::frac)>>::value));
        static constexpr unsigned IntBits  = static_cast<unsigned>(std::popcount(
          Register::Detail::GetMask<std::remove_cvref_t<decltype(Div::_int)>>::value));
        static_assert(DivInt < (1U << IntBits),
                      "the GPOUT divider's integer part does not fit the chip's field");
        static_assert(DivFrac16 < 65536,
                      "the fraction is in 1/65536");
        static_assert(DivInt > 0 || DivFrac16 == 0,
                      "an integer part of 0 means the maximum; a fraction needs an integer part");
        static constexpr std::uint32_t DivFrac = DivFrac16 >> (16U - FracBits);

        static constexpr auto initStepPeripheryConfig
          = list(write(Div::_int, Register::value<DivInt>()),
                 write(Div::frac, Register::value<DivFrac>()),
                 Ctrl::overrideDefaults(write(
                   Ctrl::auxsrc,
                   Register::value<typename Ctrl::AUXSRCVal, detail::auxsrcOfExtra<Ctrl>(Src)>())));

        static constexpr auto initStepPeripheryEnable = list(set(Ctrl::enable));

        static void setEnabled(bool on) {
            if(on) {
                apply(set(Ctrl::enable));
            } else {
                apply(clear(Ctrl::enable));
            }
        }

        /// The output for a source of `sourceHz`.
        static constexpr std::uint32_t outputHz(std::uint32_t sourceHz) {
            return static_cast<std::uint32_t>(
              (static_cast<std::uint64_t>(sourceHz) << 16)
              / ((static_cast<std::uint64_t>(DivInt) << 16) | DivFrac16));
        }
    };

    struct ResusTag {};

    template<typename Config_, void (*Callback)()>
    struct Resus {
        struct Config {
            static constexpr std::uint32_t timeout = [] {
                if constexpr(requires { Config_::timeout; }) {
                    return static_cast<std::uint32_t>(Config_::timeout);
                } else {
                    return 255U;
                }
            }();
            static constexpr int isrPriority = [] {
                if constexpr(requires { Config_::isrPriority; }) {
                    return static_cast<int>(Config_::isrPriority);
                } else {
                    return 2;
                }
            }();
        };

        static_assert(Config::timeout < 256,
                      "the resus timeout is 8 bits");

        using Clk       = Kvasir::Peripheral::CLOCKS::Registers<>;
        using NvicIndex = Nvic::Index<decltype(Kvasir::Interrupt::clocks)::value>;

        using Provides = brigand::list<Startup::Resource<ResusTag, 0>>;

        static constexpr auto initStepPeripheryConfig
          = list(Clk::CLK_SYS_RESUS_CTRL::overrideDefaults(
                   write(Clk::CLK_SYS_RESUS_CTRL::timeout, Register::value<Config::timeout>())),
                 write(Clk::INTE::clk_sys_resus, Register::value<1>()));

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Config::isrPriority>(NvicIndex{}),
                 Nvic::makeClearPending(NvicIndex{}));

        static constexpr auto initStepPeripheryEnable
          = list(set(Clk::CLK_SYS_RESUS_CTRL::enable), Nvic::makeEnable(NvicIndex{}));

        /// Whether the last event has not been acknowledged yet.
        [[nodiscard]] static bool resuscitated() {
            return apply(read(Clk::CLK_SYS_RESUS_STATUS::resussed));
        }

        static void onIsr() {
            if(apply(read(Clk::INTS::clk_sys_resus))) {
                Callback();
                // Acknowledge: a pulse on CLEAR ends the event and re-arms the detector.
                apply(set(Clk::CLK_SYS_RESUS_CTRL::clear));
                apply(clear(Clk::CLK_SYS_RESUS_CTRL::clear));
            }
        }

        static constexpr Nvic::Isr<std::addressof(onIsr), NvicIndex> isr{};
    };

}}   // namespace Kvasir::Clocks
