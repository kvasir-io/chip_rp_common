#pragma once
#include "kvasir/Register/Register.hpp"
#include "peripherals/CLOCKS.hpp"

#if __has_include("peripherals/QMI.hpp")
    #include "peripherals/QMI.hpp"
#endif

#include <cmath>
#include <cstdint>

namespace Kvasir { namespace DefaultClockSettings {
    namespace detail {
        struct PllSettings {
            std::uint32_t fbdiv;
            std::uint32_t pd1;
            std::uint32_t pd2;
            std::uint32_t refdiv;
        };

        //Use a lower VCO frequency when possible. This reduces power consumption, at the cost of increased jitter"
        template<bool LowVco = false>
        static constexpr PllSettings calcPllSettings(double clockSpeed,
                                                     double crystalSpeed) {
            // RP2350 PLL constraints from datasheet
            constexpr double vco_max = 1'600'000'000;
            constexpr double vco_min = 750'000'000;
            constexpr double ref_min = 5'000'000;

            constexpr std::uint32_t fbdiv_max   = 320;
            constexpr std::uint32_t fbdiv_min   = 16;
            constexpr std::uint32_t refdiv_min  = 1;
            constexpr std::uint32_t refdiv_max  = 63;
            constexpr std::uint32_t postdiv_max = 7;
            constexpr std::uint32_t postdiv_min = 1;

            // Calculate REFDIV range based on minimum reference frequency constraint
            auto refdiv_range_max = static_cast<std::uint32_t>(crystalSpeed / ref_min);
            refdiv_range_max      = std::min(refdiv_range_max, refdiv_max);
            refdiv_range_max      = std::max(refdiv_range_max, refdiv_min);

            PllSettings bestSettings{.fbdiv = 0, .pd1 = 0, .pd2 = 0, .refdiv = 0};
            double      bestMargin = clockSpeed;
            double      bestVco    = 0.0;

            // Search algorithm matching RP2350 vcocalc.py
            for(std::uint32_t refdiv = refdiv_min; refdiv <= refdiv_range_max; ++refdiv) {
                double const refFreq = crystalSpeed / refdiv;
                if(refFreq < ref_min) {
                    continue;   // Skip if reference frequency too low
                }

                for(std::uint32_t fbdiv = fbdiv_min; fbdiv <= fbdiv_max; ++fbdiv) {
                    double const vco = refFreq * fbdiv;
                    if(vco < vco_min || vco > vco_max) {
                        continue;
                    }

                    // pd1 is inner loop to prefer higher pd1:pd2 ratios for lower power
                    for(std::uint32_t pd2 = postdiv_min; pd2 <= postdiv_max; ++pd2) {
                        for(std::uint32_t pd1 = postdiv_min; pd1 <= postdiv_max; ++pd1) {
                            // Check for integer frequency ratios (from vcocalc.py line 50)
                            if(static_cast<std::uint64_t>(vco * 1000)
                                 % static_cast<std::uint64_t>(pd1 * pd2)
                               != 0)
                            {
                                continue;
                            }

                            double const out = vco / pd1 / pd2;
                            double const margin
                              = out > clockSpeed ? out - clockSpeed : clockSpeed - out;

                            // VCO preference logic from vcocalc.py line 49
                            bool const vcoIsBetter = LowVco ? (vco < bestVco) : (vco > bestVco);

                            // Accept if better margin, or same margin with preferred VCO
                            constexpr double tolerance   = 1e-9;
                            bool const       marginEqual = (margin - bestMargin) < tolerance
                                                  && (margin - bestMargin) > -tolerance;

                            if(margin < bestMargin || (marginEqual && vcoIsBetter)) {
                                bestSettings = PllSettings{.fbdiv  = fbdiv,
                                                           .pd1    = pd1,
                                                           .pd2    = pd2,
                                                           .refdiv = refdiv};
                                bestMargin   = margin;
                                bestVco      = vco;
                            }
                        }
                    }
                }
            }

            return bestSettings;
        }

        namespace impl {
            template<auto CrystalSpeed,
                     typename Reg>

            static constexpr auto getXoscFreqRange() {
                if constexpr(CrystalSpeed >= 1'000'000 && CrystalSpeed <= 15'000'000) {
                    return Reg::FREQ_RANGEValC::_1_15mhz;
                } else if constexpr(CrystalSpeed >= 10'000'000 && CrystalSpeed <= 30'000'000) {
                    return Reg::FREQ_RANGEValC::_10_30mhz;
                } else if constexpr(CrystalSpeed >= 25'000'000 && CrystalSpeed <= 60'000'000) {
                    return Reg::FREQ_RANGEValC::_25_60mhz;
                } else if constexpr(CrystalSpeed >= 40'000'000 && CrystalSpeed <= 100'000'000) {
                    return Reg::FREQ_RANGEValC::_40_100mhz;
                } else {
                    static_assert(CrystalSpeed >= 1'000'000 && CrystalSpeed <= 100'000'000,
                                  "Crystal frequency must be between 1 MHz and 100 MHz");
                    return Reg::FREQ_RANGEValC::_1_15mhz;
                }
            }
        }   // namespace impl

        template<auto CrystalSpeed>
        static constexpr auto getXoscFreqRange() {
            using Reg = Kvasir::Peripheral::XOSC::Registers<>::CTRL;
            return impl::getXoscFreqRange<CrystalSpeed, Reg>();
        }
    }   // namespace detail

    // Flash configuration function for W25Q32RV optimal settings
    template<auto ClockSpeed>
    void flashInit() {
#if __has_include("peripherals/QMI.hpp")
        using namespace Kvasir::Peripheral::QMI;
        using QMI = Registers<0>;

        constexpr std::uint32_t max_flash_freq = 104'000'000;
        constexpr std::uint32_t min_clkdiv     = 2;
        constexpr std::uint32_t max_clkdiv     = 255;

        constexpr std::uint32_t calculated_clkdiv
          = (ClockSpeed + max_flash_freq - 1) / max_flash_freq;

        constexpr std::uint32_t clkdiv = [&]() {
            if(calculated_clkdiv < min_clkdiv) {
                return min_clkdiv;
            }
            if(calculated_clkdiv > max_clkdiv) {
                return max_clkdiv;
            }
            return calculated_clkdiv;
        }();

        constexpr std::uint32_t flash_freq = ClockSpeed / clkdiv;
        static_assert(flash_freq <= max_flash_freq,
                      "Flash frequency exceeds W25Q32RV maximum of 104 MHz");

        apply(QMI::M0_TIMING::overrideDefaults(
          write(QMI::M0_TIMING::clkdiv, value<std::uint32_t, clkdiv>()),
          write(QMI::M0_TIMING::cooldown, value<std::uint32_t, 1>()),
          write(QMI::M0_TIMING::rxdelay, value<std::uint32_t, 2>()),
          write(QMI::M0_TIMING::PAGEBREAKValC::none),
          write(QMI::M0_TIMING::select_setup, value<std::uint32_t, 0>()),
          write(QMI::M0_TIMING::select_hold, value<std::uint32_t, 0>()),
          write(QMI::M0_TIMING::max_select, value<std::uint32_t, 0>()),
          write(QMI::M0_TIMING::min_deselect, value<std::uint32_t, 0>())));

        apply(QMI::M0_RFMT::overrideDefaults(write(QMI::M0_RFMT::dtr, value<std::uint32_t, 0>()),
                                             write(QMI::M0_RFMT::DUMMY_LENValC::_16),
                                             write(QMI::M0_RFMT::SUFFIX_LENValC::_8),
                                             write(QMI::M0_RFMT::PREFIX_LENValC::_8),
                                             write(QMI::M0_RFMT::DATA_WIDTHValC::q),
                                             write(QMI::M0_RFMT::DUMMY_WIDTHValC::q),
                                             write(QMI::M0_RFMT::SUFFIX_WIDTHValC::q),
                                             write(QMI::M0_RFMT::ADDR_WIDTHValC::q),
                                             write(QMI::M0_RFMT::PREFIX_WIDTHValC::s)));
#endif
    }

    template<auto ClockSpeed,
             auto CrystalSpeed>
    void coreClockInit() {
        using Kvasir::Register::value;

        static constexpr auto pllSettings     = detail::calcPllSettings(ClockSpeed, CrystalSpeed);
        static constexpr auto usb_pllSettings = detail::calcPllSettings(48'000'000, CrystalSpeed);

        static_assert(ClockSpeed
                        == (CrystalSpeed / pllSettings.refdiv) * pllSettings.fbdiv
                             / (pllSettings.pd1 * pllSettings.pd2),
                      "bad clock config");

        static_assert(48'000'000
                        == (CrystalSpeed / usb_pllSettings.refdiv) * usb_pllSettings.fbdiv
                             / (usb_pllSettings.pd1 * usb_pllSettings.pd2),
                      "bad clock config");

        using PERI_CLOCK = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_CTRL;
        using SYS_CLOCK  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_CTRL;
        using REF_CLOCK  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_CTRL;
        using USB_CLOCK  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        using XOSC       = Kvasir::Peripheral::XOSC::Registers<>;
        using RST        = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL        = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL     = Kvasir::Peripheral::PLL::Registers<1>;

        flashInit<ClockSpeed>();

        // disable periphery clocks
        apply(PERI_CLOCK::overrideDefaults(clear(PERI_CLOCK::enable)));

        // set ref clock to default
        apply(REF_CLOCK::overrideDefaults(write(REF_CLOCK::SRCValC::rosc_clksrc_ph)));

        // set sysclock to default
        apply(SYS_CLOCK::overrideDefaults(write(SYS_CLOCK::SRCValC::clk_ref)));

        apply(write(XOSC::CTRL::ENABLEValC::en), write(detail::getXoscFreqRange<CrystalSpeed>()));
        // wait for XOSC stable
        while(!apply(read(XOSC::STATUS::stable))) {
        }
        {   //sys pll
            // reset pll
            apply(set(RST::RESET::pll_sys));
            apply(clear(RST::RESET::pll_sys));
            while(!apply(read(RST::RESET_DONE::pll_sys))) {
            }

            apply(PLL::CS::overrideDefaults(clear(PLL::CS::bypass),
                                            write(PLL::CS::refdiv, value<pllSettings.refdiv>())));

            apply(write(PLL::FBDIV_INT::fbdiv_int, value<pllSettings.fbdiv>()));

            apply(PLL::PWR::overrideDefaults(clear(PLL::PWR::vcopd),
                                             clear(PLL::PWR::pd),
                                             set(PLL::PWR::postdivpd)));

            // wait for PLL lock
            while(!apply(read(PLL::CS::lock))) {
            }

            apply(
              PLL::PRIM::overrideDefaults(write(PLL::PRIM::postdiv1, value<pllSettings.pd1>()),
                                          write(PLL::PRIM::postdiv2, value<pllSettings.pd2>())));

            apply(PLL::PWR::overrideDefaults(clear(PLL::PWR::vcopd),
                                             clear(PLL::PWR::pd),
                                             clear(PLL::PWR::postdivpd)));

            // set sysclock to pll
            apply(SYS_CLOCK::overrideDefaults(write(SYS_CLOCK::SRCValC::clksrc_clk_sys_aux),
                                              write(SYS_CLOCK::AUXSRCValC::clksrc_pll_sys)));

            // set ref clock to xosc
            apply(REF_CLOCK::overrideDefaults(write(REF_CLOCK::SRCValC::xosc_clksrc)));

            // enable periphery clock
            apply(PERI_CLOCK::overrideDefaults(set(PERI_CLOCK::enable)));
        }

        {   //usb pll
            // reset pll
            apply(set(RST::RESET::pll_usb));
            apply(clear(RST::RESET::pll_usb));
            while(!apply(read(RST::RESET_DONE::pll_usb))) {
            }

            apply(USBPLL::CS::overrideDefaults(
              clear(USBPLL::CS::bypass),
              write(USBPLL::CS::refdiv, value<usb_pllSettings.refdiv>())));

            apply(write(USBPLL::FBDIV_INT::fbdiv_int, value<usb_pllSettings.fbdiv>()));

            apply(USBPLL::PWR::overrideDefaults(clear(USBPLL::PWR::vcopd),
                                                clear(USBPLL::PWR::pd),
                                                set(USBPLL::PWR::postdivpd)));

            // wait for PLL lock
            while(!apply(read(USBPLL::CS::lock))) {
            }

            apply(USBPLL::PRIM::overrideDefaults(
              write(USBPLL::PRIM::postdiv1, value<usb_pllSettings.pd1>()),
              write(USBPLL::PRIM::postdiv2, value<usb_pllSettings.pd2>())));

            apply(USBPLL::PWR::overrideDefaults(clear(USBPLL::PWR::vcopd),
                                                clear(USBPLL::PWR::pd),
                                                clear(USBPLL::PWR::postdivpd)));

            // enable periphery clock
            apply(USB_CLOCK::overrideDefaults(set(USB_CLOCK::enable),
                                              write(USB_CLOCK::AUXSRCValC::clksrc_pll_usb)));
        }
    }

    template<auto ClockSpeed,
             auto CrystalSpeed>
    void peripheryClockInit() {}
}}   // namespace Kvasir::DefaultClockSettings
