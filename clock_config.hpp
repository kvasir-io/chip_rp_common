#pragma once
#include "Clocks.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/CLOCKS.hpp"

#if __has_include("peripherals/QMI.hpp")
    #include "bootrom_functions.hpp"
    #include "peripherals/QMI.hpp"
#endif

#if __has_include("peripherals/XIP_CTRL.hpp")
    #include "peripherals/XIP_CTRL.hpp"
#endif

#if __has_include("peripherals/POWMAN.hpp")
    #include "peripherals/POWMAN.hpp"
#endif
#if __has_include("peripherals/VREG_AND_CHIP_RESET.hpp")
    #include "peripherals/VREG_AND_CHIP_RESET.hpp"
#endif

#include <cmath>
#include <cstdint>

namespace Kvasir { namespace Clocks {
    // The two PLLs by name: the system PLL feeds clk_sys, the USB PLL clk_usb and clk_adc.
    using PllSys = Peripheral::PLL::Registers<0>;
    using PllUsb = Peripheral::PLL::Registers<1>;
}}   // namespace Kvasir::Clocks

namespace Kvasir { namespace DefaultClockSettings {
    // The clocks coreClockInit below programs, as Startup resources (Clocks.hpp). An
    // application's ClockSettings opts into the clock check with
    //     using Provides = Kvasir::DefaultClockSettings::Provides<ClockSpeed, CrystalSpeed>;
    template<auto ClockSpeed, auto CrystalSpeed>
    using Provides = Clocks::DefaultProvides<ClockSpeed, CrystalSpeed>;

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
                    if(vco < vco_min || vco > vco_max) { continue; }

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

    // Core voltage for the requested clock. Reset value is 1.10 V, the
    // datasheet setting for 150 MHz; above that the regulator must be raised
    // before the PLL switch. 1.15 V covers up to ~266 MHz - extend the range
    // only together with hardware validation.
    //
    // POWMAN writes are ignored unless bits 31:16 carry the 0x5AFE password,
    // hence the full-register write instead of a field write.
    template<auto ClockSpeed>
    void vregInit() {
#if __has_include("peripherals/POWMAN.hpp")
        if constexpr(ClockSpeed > 150'000'000) {
            static_assert(ClockSpeed <= 266'000'000,
                          "no validated core voltage for this clock - extend "
                          "vregInit together with hardware validation");
            using VREG = Kvasir::Peripheral::POWMAN::Registers<>::VREG;
            using Kvasir::Register::value;
            constexpr std::uint32_t passwd    = 0x5AFEU << 16;
            constexpr std::uint32_t vsel_1v15 = 0b01100;
            // The regulator ignores writes while an update is in flight; the
            // update takes microseconds, so the polls are bounded.
            for(int i = 0; i < 100000; ++i) {
                if(!apply(read(VREG::update_in_progress))) { break; }
            }
            apply(write(VREG::FULLREGISTER, value<std::uint32_t, passwd | (vsel_1v15 << 4)>()));
            for(int i = 0; i < 100000; ++i) {
                if(!apply(read(VREG::update_in_progress))) { break; }
            }
        }
#elif __has_include("peripherals/VREG_AND_CHIP_RESET.hpp")
        // RP2040. Reset value is 1.10 V (vsel 0b01011), the datasheet setting for 133 MHz.
        // 200 MHz at 1.15 V (0b01100) is the one overclock pico-sdk validates
        // (SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST); nothing above it is.
        if constexpr(ClockSpeed > 133'000'000) {
            static_assert(ClockSpeed <= 200'000'000,
                          "no validated core voltage for this clock on the RP2040 - extend "
                          "vregInit together with hardware validation");
            using VREG = Kvasir::Peripheral::VREG_AND_CHIP_RESET::Registers<>::VREG;
            using Kvasir::Register::value;
            constexpr std::uint32_t vsel_1v15 = 0b01100;
            apply(write(VREG::vsel, value<std::uint32_t, vsel_1v15>()));
            // The RP2040 regulator has no "update in progress" flag; pico-sdk waits 1 ms.
            // This runs before the crystal is up, on the ring oscillator at up to ~12 MHz,
            // so the wait is a counted loop: 12 000 turns of a 1-cycle-ish loop is at
            // least 1 ms at any ROSC speed, and only a few ms at the slowest.
            std::uint32_t turns = 12'000;
            asm volatile(
              "1:\n"
              "subs %0, %0, #1\n"
              "bne 1b\n"
              : "+l"(turns)
              :
              : "cc");
        }
#else
        static_assert(ClockSpeed <= 150'000'000,
                      "no POWMAN register description - cannot raise the core "
                      "voltage this clock needs");
#endif
    }

    namespace detail {
        // The QMI's M0_TIMING for the flash at the final clock: divider, sample delay,
        // deselect time. Applied by XipReadMode::apply() in peripheryClockInit(), from RAM,
        // after the PLL switch - see flashInit() for why not earlier.
        //
        // MaxFlashFreq: what the flash is rated for, 100 MHz by default.
        template<auto ClockSpeed,
                 auto MaxFlashFreq>
        constexpr std::uint32_t flashClkdiv() {
            constexpr std::uint32_t min_clkdiv = 2;
            constexpr std::uint32_t max_clkdiv = 255;

            constexpr std::uint32_t calculated_clkdiv
              = (ClockSpeed + MaxFlashFreq - 1) / MaxFlashFreq;

            if(calculated_clkdiv < min_clkdiv) { return min_clkdiv; }
            if(calculated_clkdiv > max_clkdiv) { return max_clkdiv; }
            return calculated_clkdiv;
        }

        template<auto ClockSpeed,
                 auto MaxFlashFreq>
        constexpr std::uint32_t flashTiming() {
            constexpr std::uint32_t clkdiv = flashClkdiv<ClockSpeed, MaxFlashFreq>();

            constexpr std::uint32_t flash_freq = ClockSpeed / clkdiv;
            static_assert(flash_freq <= MaxFlashFreq, "Flash frequency exceeds MaxFlashFreq");

            // rxdelay: when the QMI samples the data lines, in half clk_sys cycles after
            // the SCK edge. The flash launches a bit ~9 ns (clock-to-output plus the pad
            // round trip) after the opposite edge, half an SCK period earlier, so the bit
            // is there from (9 ns - clkdiv/2 cycles) to (9 ns + clkdiv/2 cycles) after
            // the sample edge. Sampling 7.5 ns after the edge sits mid-window at every
            // clk_sys the divider rule allows: 2 at 150 MHz (window 1..3), 3 at 200 MHz
            // (window 2..4). Below about 33 MHz that rounds to 0, which is clamped to 1.
            constexpr std::uint32_t rxdelay_rounded = static_cast<std::uint32_t>(
              (static_cast<unsigned long long>(ClockSpeed) * 15ULL + 500'000'000ULL)
              / 1'000'000'000ULL);   // round(7.5 ns * 2 * clk_sys)
            constexpr std::uint32_t rxdelay = rxdelay_rounded < 1 ? 1 : rxdelay_rounded;
            static_assert(rxdelay <= 7, "rxdelay out of the QMI's range");

            // min_deselect: chip select high for half an SCK period plus this many clk_sys
            // cycles between transfers. The W25Q64JV wants 10 ns (tSHSL, read); 2 cycles
            // give 3 x 5 ns at 200 MHz clk_sys, 3 x 6.7 ns at 150 MHz. (The bootrom uses 7.)
            constexpr std::uint32_t min_deselect = 2;

            constexpr std::uint32_t cooldown = 1;   // hold CS, append sequential accesses
            // pagebreak none, select_setup 0, select_hold 0, max_select 0
            return (cooldown << 30) | (min_deselect << 12) | (rxdelay << 8) | clkdiv;
        }
    }   // namespace detail

    // Before the PLL switch, from flash, on the ring oscillator: only the divider.
    //
    // CLKDIV is the one M0_TIMING field the QMI lets software change while it is fetching;
    // everything else needs the QMI idle, which code running from flash cannot promise.
    // And the sample delay that is right at the final clock is wrong here: with the
    // divider at 2 the next bit lands half an SCK period after the sample edge, which at
    // ROSC speed is long before a 7.5 ns rxdelay samples. Divider 4 with the bootrom's rxdelay 2
    // reads clean at ROSC and at any final clk_sys the divider rule allows, so this is
    // what the code between here and peripheryClockInit() runs on - or the final divider,
    // when a slow flash needs a larger one.
    template<auto ClockSpeed,
             auto MaxFlashFreq = 100'000'000>
    void flashInit() {
#if __has_include("peripherals/QMI.hpp")
        using namespace Kvasir::Peripheral::QMI;
        using QMI = Registers<0>;
        (void)detail::flashTiming<ClockSpeed, MaxFlashFreq>();   // the static_asserts
        constexpr std::uint32_t finalClkdiv = detail::flashClkdiv<ClockSpeed, MaxFlashFreq>();
        constexpr std::uint32_t bootClkdiv  = finalClkdiv > 4 ? finalClkdiv : 4;
        // This divider with rxdelay 2 (XipReadMode::TimingAtClkRef, which the resus handler runs
        // coreClockInit() on) samples half an SCK period plus one clk_sys cycle after the edge the
        // flash launches on. The bit is valid from 10 ns after that edge at the latest: 2.5 ns
        // clk_sys to QSPI output + 1.5 ns QSPI input to clk_sys (RP2350 datasheet Table 1292,
        // 3.3 V) + 6 ns tCLQV (W25Q128JV AC table; the W25Q64JV family). Holds up to ~300 MHz.
        constexpr unsigned long long samplePs = (bootClkdiv / 2 + 1) * 1'000'000'000'000ULL
                                              / static_cast<unsigned long long>(ClockSpeed);
        static_assert(samplePs >= 10'000,
                      "clk_sys too fast for the boot/resus flash timing: the flash's data is not "
                      "valid yet when the QMI samples it (see flashInit)");
        apply(write(QMI::M0_TIMING::clkdiv, value<std::uint32_t, bootClkdiv>()));
        // A larger divider is only in effect after the next QMI access, and it has to be before
        // clk_sys goes up (RP2350 datasheet, Table 1297 M0_TIMING.CLKDIV): the next fetch may hit
        // the XIP cache, so read the uncached alias (XIP_NOCACHE_NOALLOC_BASE, 2.2 address map).
        asm volatile("dsb" ::: "memory");
        (void)*reinterpret_cast<std::uint32_t const volatile*>(
          Kvasir::detail::XipReadMode::XipNoCacheBase);
        asm volatile("dsb" ::: "memory");
#endif

#if __has_include("peripherals/XIP_CTRL.hpp")
        // Restore XIP cache to reset state. The debugger may disable it during flash writes;
        // a run_low reset leaves peripheral state as-is, so this ensures the cache is always
        // enabled regardless of reset cause.
        apply(Kvasir::Peripheral::XIP_CTRL::Registers<>::CTRL::overrideDefaults());
#endif
    }

    template<auto ClockSpeed,
             auto CrystalSpeed,
             auto MaxFlashFreq = 100'000'000>
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

        using PERI_CLOCK    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_CTRL;
        using SYS_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_CTRL;
        using REF_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_CTRL;
        using REF_CLOCK_DIV = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_DIV;
        using USB_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        using ADC_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_ADC_CTRL;
        using XOSC          = Kvasir::Peripheral::XOSC::Registers<>;
        using RST           = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL           = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL        = Kvasir::Peripheral::PLL::Registers<1>;

        // Voltage first, then flash timing, then the PLL switch - all three
        // still running from ROSC, so the core never executes a cycle at the
        // new frequency on the old voltage.
        vregInit<ClockSpeed>();
        flashInit<ClockSpeed, MaxFlashFreq>();

        // disable periphery clocks
        apply(PERI_CLOCK::overrideDefaults(clear(PERI_CLOCK::enable)));

        // set ref clock to default
        apply(REF_CLOCK::overrideDefaults(write(REF_CLOCK::SRCValC::rosc_clksrc_ph)));

        // set sysclock to default
        apply(SYS_CLOCK::overrideDefaults(write(SYS_CLOCK::SRCValC::clk_ref)));

        // A clk_sys resus (Clocks::Resus) survives a core reset: the clocks block is not
        // reset by SYSRESETREQ, so after an unhandled event every debugger reset boots with
        // clk_sys forced to clk_ref, the PLL switch below has no effect, and the flash
        // timing peripheryClockInit() applies for ClockSpeed reads garbage at 12 MHz - a
        // HardFault before main, on every reset, until the resus is released.
        // clk_sys is explicitly on clk_ref now, so releasing it changes nothing here.
        {
            using RESUS_CTRL   = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_RESUS_CTRL;
            using RESUS_STATUS = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_RESUS_STATUS;
            if(apply(read(RESUS_STATUS::resussed))) {
                apply(set(RESUS_CTRL::clear));
                apply(clear(RESUS_CTRL::clear));
            }
        }

        apply(write(XOSC::CTRL::ENABLEValC::en), write(detail::getXoscFreqRange<CrystalSpeed>()));
        // wait for XOSC stable
        while(!apply(read(XOSC::STATUS::stable))) {}
        {   //sys pll
            // reset pll
            apply(set(RST::RESET::pll_sys));
            apply(clear(RST::RESET::pll_sys));
            while(!apply(read(RST::RESET_DONE::pll_sys))) {}

            apply(PLL::CS::overrideDefaults(clear(PLL::CS::bypass),
                                            write(PLL::CS::refdiv, value<pllSettings.refdiv>())));

            apply(write(PLL::FBDIV_INT::fbdiv_int, value<pllSettings.fbdiv>()));

            apply(PLL::PWR::overrideDefaults(clear(PLL::PWR::vcopd),
                                             clear(PLL::PWR::pd),
                                             set(PLL::PWR::postdivpd)));

            // wait for PLL lock
            while(!apply(read(PLL::CS::lock))) {}

            apply(
              PLL::PRIM::overrideDefaults(write(PLL::PRIM::postdiv1, value<pllSettings.pd1>()),
                                          write(PLL::PRIM::postdiv2, value<pllSettings.pd2>())));

            apply(PLL::PWR::overrideDefaults(clear(PLL::PWR::vcopd),
                                             clear(PLL::PWR::pd),
                                             clear(PLL::PWR::postdivpd)));

            // set sysclock to pll
            apply(SYS_CLOCK::overrideDefaults(write(SYS_CLOCK::SRCValC::clksrc_clk_sys_aux),
                                              write(SYS_CLOCK::AUXSRCValC::clksrc_pll_sys)));

            // set ref clock to xosc, undivided (boot may leave CLK_REF_DIV != 1)
            apply(write(REF_CLOCK_DIV::_int, value<std::uint32_t{1}>()));
            apply(REF_CLOCK::overrideDefaults(write(REF_CLOCK::SRCValC::xosc_clksrc)));

            // enable periphery clock
            apply(PERI_CLOCK::overrideDefaults(set(PERI_CLOCK::enable)));
        }

        {   //usb pll
            // reset pll
            apply(set(RST::RESET::pll_usb));
            apply(clear(RST::RESET::pll_usb));
            while(!apply(read(RST::RESET_DONE::pll_usb))) {}

            apply(USBPLL::CS::overrideDefaults(
              clear(USBPLL::CS::bypass),
              write(USBPLL::CS::refdiv, value<usb_pllSettings.refdiv>())));

            apply(write(USBPLL::FBDIV_INT::fbdiv_int, value<usb_pllSettings.fbdiv>()));

            apply(USBPLL::PWR::overrideDefaults(clear(USBPLL::PWR::vcopd),
                                                clear(USBPLL::PWR::pd),
                                                set(USBPLL::PWR::postdivpd)));

            // wait for PLL lock
            while(!apply(read(USBPLL::CS::lock))) {}

            apply(USBPLL::PRIM::overrideDefaults(
              write(USBPLL::PRIM::postdiv1, value<usb_pllSettings.pd1>()),
              write(USBPLL::PRIM::postdiv2, value<usb_pllSettings.pd2>())));

            apply(USBPLL::PWR::overrideDefaults(clear(USBPLL::PWR::vcopd),
                                                clear(USBPLL::PWR::pd),
                                                clear(USBPLL::PWR::postdivpd)));

            // enable periphery clock
            apply(USB_CLOCK::overrideDefaults(set(USB_CLOCK::enable),
                                              write(USB_CLOCK::AUXSRCValC::clksrc_pll_usb)));
            apply(ADC_CLOCK::overrideDefaults(set(ADC_CLOCK::enable),
                                              write(ADC_CLOCK::AUXSRCValC::clksrc_pll_usb)));
        }
    }

    // After initMemory(): the RAM functions exist now, and interrupts are not enabled yet.
    // The flash's final timing and read mode (see flashInit() and XipReadMode). A project
    // that passes its own MaxFlashFreq must pass it here and to coreClockInit().
    template<auto ClockSpeed,
             auto CrystalSpeed,
             auto MaxFlashFreq = 100'000'000>
    void peripheryClockInit() {
#if __has_include("peripherals/QMI.hpp")
        std::uint32_t primask{};
        asm volatile("mrs %0, primask\n cpsid i" : "=r"(primask)::"memory");
        Kvasir::detail::XipReadMode::apply(detail::flashTiming<ClockSpeed, MaxFlashFreq>());
        asm volatile("msr primask, %0" ::"r"(primask) : "memory");
#endif
    }

    // An application's whole ClockSettings in one line:
    //     using ClockSettings = Kvasir::DefaultClockSettings::Settings<ClockSpeed, CrystalSpeed>;
    // Provides (the clock check), coreClockInit() and peripheryClockInit() with the same
    // three arguments, so the numbers are spelled once.
    template<auto ClockSpeed, auto CrystalSpeed, auto MaxFlashFreq = 100'000'000>
    struct Settings {
        using Provides = DefaultClockSettings::Provides<ClockSpeed, CrystalSpeed>;

        static void coreClockInit() {
            DefaultClockSettings::coreClockInit<ClockSpeed, CrystalSpeed, MaxFlashFreq>();
        }

        static void peripheryClockInit() {
            DefaultClockSettings::peripheryClockInit<ClockSpeed, CrystalSpeed, MaxFlashFreq>();
        }
    };
}}   // namespace Kvasir::DefaultClockSettings
