#pragma once
#include "Clocks.hpp"
#include "PllSearch.hpp"
#include "WaitBounds.hpp"
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
    #include "peripherals/SYSCFG.hpp"
#endif
#if __has_include("peripherals/VREG_AND_CHIP_RESET.hpp")
    #include "peripherals/VREG_AND_CHIP_RESET.hpp"
#endif

#include <cmath>
#include <cstdint>
#include <type_traits>

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
        // CLK_x_SELECTED == 1 << src, one decoded bit per CTRL SRC value (RP2040 datasheet Tables 222/225, RP2350
        // Tables 558/561), as a whole-register wait condition (the RP2040 SVD has no field); may read 0 mid-switch.
        template<typename Selected, unsigned Src>
        inline constexpr auto selectedIs = Kvasir::Register::FieldValue<
          typename std::remove_cvref_t<decltype(Selected::FULLREGISTER)>::type,
          (std::uint32_t{1} << Src)>{};

        // At least N cycles of clk_sys, for a wait no status bit reports
        template<unsigned N>
        [[gnu::always_inline]] inline void spinCycles() {
            for(unsigned i = 0; i != (N + 1) / 2; ++i) {
                asm volatile("nop" ::
                               : "memory");   // + the loop's own instructions: >= 2 cycles a pass
            }
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

    namespace detail {
        // A counted wait for code that runs before any timer is set up (vregInit): `turns` times a
        // loop of at least one cycle, registers only.
        inline void spinTurns(std::uint32_t turns) {
            // gcc assembles Thumb-1 inline asm in divided syntax, where `subs` with three operands
            // does not exist; clang is always unified. The directive emits nothing.
            asm volatile(
              ".syntax unified\n"
              "1:\n"
              "subs %0, %0, #1\n"
              "bne 1b\n"
              : "+l"(turns)
              :
              : "cc");
        }
    }   // namespace detail

    // Core voltage for the requested clock. Reset value is 1.10 V, the
    // datasheet setting for 150 MHz; above that the regulator must be raised
    // before the PLL switch. 1.15 V covers up to ~266 MHz - extend the range
    // only together with hardware validation.
    //
    // POWMAN writes are ignored unless bits 31:16 carry the 0x5AFE password: the SVD's
    // PASSWORD key field, which every write of VREG names (a compile error otherwise).
    //
    // The sequence is pico-sdk's vreg_set_voltage (hardware_vreg/vreg.c) on the RP2350, and every
    // step of it is load-bearing:
    //   - VREG_CTRL.UNLOCK first: "The regulator can be directly controlled by software, but must
    //     first be unlocked by writing a 1 to the UNLOCK field in the VREG_CTRL register" (RP2350
    //     datasheet 6.3.2). Locked, the VSEL write sets UPDATE_IN_PROGRESS and nothing else: the bit
    //     stays set for good, the voltage stays at 1.10 V and the PLL switch below runs the core at
    //     ClockSpeed on it. Seen at 200 MHz: VREG 0x80C0 -- VSEL 1.15 V, UPDATE_IN_PROGRESS -- for
    //     as long as the image ran. It cannot be relocked; pico-sdk leaves it unlocked too.
    //   - Then time for the output to get there before the PLL switch: UPDATE_IN_PROGRESS covers the
    //     register update only, and VOUT_OK cannot tell 1.10 V from 1.15 V (it asserts at 90 % of the
    //     selected voltage, 6.3.4). pico-sdk waits SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US, 1 ms
    //     (pico_runtime_init/runtime_init_clocks.c); the data sheet gives no settling time.
    //   - SYSCFG.AUXCTRL bit 0 cleared before that (for every image): it forces POWMAN onto the 32 kHz LPOSC, which
    //     the boot ROM's reboot() and Watchdog::trigger set before a reset (watchdog.hpp
    //     powmanOffClkRef) and which survives the boot ROM's reboot (found set, 1, in a running
    //     image). On the LPOSC every POWMAN access takes ~100 us: the two bounded polls below were
    //     10 s of every boot. Nothing in this image needs it set; the next reset that needs it sets
    //     it again.
    template<auto ClockSpeed>
    void vregInit() {
#if __has_include("peripherals/POWMAN.hpp")
        {   // every image, whatever its clock: POWMAN (and the AON timer) back on their own clock
            using AuxCtrl = Kvasir::Peripheral::SYSCFG::Registers<>::AUXCTRL;
            if(auto const aux = get<0>(apply(read(AuxCtrl::auxctrl))); (aux & 1U) != 0) {
                apply(write(AuxCtrl::auxctrl, aux & ~1U));
            }
        }
        if constexpr(ClockSpeed > 150'000'000) {
            static_assert(ClockSpeed <= 266'000'000,
                          "no validated core voltage for this clock - extend "
                          "vregInit together with hardware validation");
            using VREG      = Kvasir::Peripheral::POWMAN::Registers<>::VREG;
            using VREG_CTRL = Kvasir::Peripheral::POWMAN::Registers<>::VREG_CTRL;
            using Kvasir::Register::value;
            constexpr std::uint32_t vsel_1v15 = 0b01100;

            apply(write(VREG_CTRL::PASSWORDValC::password), set(VREG_CTRL::unlock));

            // The regulator ignores writes while an update is in flight; an update takes
            // microseconds, so the polls are bounded (at clk_ref speed a few ms each).
            bool idle = false;
            for(int i = 0; i < 100000; ++i) {
                if(!apply(read(VREG::update_in_progress))) {
                    idle = true;
                    break;
                }
            }
            // Only a change needs the write and the settling time. The voltage is back at 1.10 V "at
            // initial power-on or after a reset event" (6.3.2), which events the data sheet does not
            // list; a core-0-only reset keeps it, and may even find clk_sys still on the PLL here. A VSEL
            // that reads right proves nothing while an update hangs: one written locked reads 1.15 V for good.
            if(!idle || get<0>(apply(read(VREG::vsel))) != vsel_1v15) {
                apply(write(VREG::PASSWORDValC::password),
                      write(VREG::vsel, value<std::uint32_t, vsel_1v15>()));
                for(int i = 0; i < 100000; ++i) {
                    if(!apply(read(VREG::update_in_progress))) { break; }
                }
                // Still before the PLL switch: clk_sys on the ROSC, which the boot ROM of an A3 runs at
                // up to 96 MHz (8.3, md l.27989). 96 000 turns of a loop of at least one cycle are at
                // least 1 ms at that; on a slow ROSC (4.6 MHz before A3) some tens of ms, once, on the
                // boot that changes the voltage.
                detail::spinTurns(96'000);
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
            detail::spinTurns(12'000);
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

        static_assert(pllSettings.refdiv != 0,
                      "no PLL setting for ClockSpeed: none comes nearer than ClockSpeed itself "
                      "(the PLL gives 750 MHz / 49 at the least, see detail::calcPllSettings)");
        static_assert(usb_pllSettings.refdiv != 0, "no USB PLL setting for 48 MHz");
        // exactly, or the message says by how much it is missed
        Prescaler::assertInTolerance<detail::pllOutput(CrystalSpeed, pllSettings),
                                     ClockSpeed,
                                     Prescaler::Tolerance{0, 1},
                                     "clk_sys PLL">();
        Prescaler::assertInTolerance<detail::pllOutput(CrystalSpeed, usb_pllSettings),
                                     48'000'000,
                                     Prescaler::Tolerance{0, 1},
                                     "clk_usb PLL">();

        using PERI_CLOCK    = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_CTRL;
        using SYS_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_CTRL;
        using REF_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_CTRL;
        using REF_CLOCK_DIV = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_DIV;
        using USB_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        using ADC_CLOCK     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_ADC_CTRL;
        using SYS_DIV       = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_DIV;
        using SYS_SELECTED  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_SYS_SELECTED;
        using REF_SELECTED  = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_REF_SELECTED;
        using XOSC          = Kvasir::Peripheral::XOSC::Registers<>;
        using RST           = Kvasir::Peripheral::RESETS::Registers<0>;
        using PLL           = Kvasir::Peripheral::PLL::Registers<0>;
        using USBPLL        = Kvasir::Peripheral::PLL::Registers<1>;

        // Voltage first, then flash timing, then the PLL switch - all three
        // still running from ROSC, so the core never executes a cycle at the
        // new frequency on the old voltage.
        vregInit<ClockSpeed>();
        flashInit<ClockSpeed, MaxFlashFreq>();

        // After a core-only reset (RP2040 datasheet 2.4.2.9, RP2350 7.3.1) clk_sys may still run on PLL_SYS via its
        // aux mux. So: glitchless muxes to safe sources first (polling CLK_x_SELECTED), aux muxes changed only while
        // deselected, generators without a glitchless mux disabled for two source cycles first (RP2040 2.15.3.2,
        // RP2350 8.1.3.2). Field writes on purpose: overrideDefaults would also reset AUXSRC (RP2350 Table 559).
        apply(write(REF_CLOCK::SRCValC::rosc_clksrc_ph));
        Kvasir::Register::waitUntil<Kvasir::Chip::ClkSelectedBound>(
          detail::selectedIs<REF_SELECTED, 0>);
        apply(write(SYS_CLOCK::SRCValC::clk_ref));
        Kvasir::Register::waitUntil<Kvasir::Chip::ClkSelectedBound>(
          detail::selectedIs<SYS_SELECTED, 0>);
        // a previous image's divider would stay: INT 1, FRAC 0 (RP2040 Table 224, RP2350 Table 560); changing
        // it on the fly is allowed (RP2040 2.15.3.3, RP2350 8.1.3.3)
        apply(write(SYS_DIV::_int, value<std::uint32_t{1}>()),
              write(SYS_DIV::frac, value<std::uint32_t{0}>()));

        // clk_peri has no glitchless mux: disable, let it stop, aux to clk_sys; enabled again after the PLL. Its
        // destinations (UART, SPI) are held in reset by FirstInitStep as well.
        apply(clear(PERI_CLOCK::enable));
#if __has_include("peripherals/POWMAN.hpp")
        // CTRL.ENABLED follows ENABLE once the generator has stopped (RP2350 Table 562, 8.1.3.2)
        Kvasir::Register::waitUntil<Kvasir::Chip::ClkSelectedBound>(
          Kvasir::Register::isClear(PERI_CLOCK::enabled));
        // never written before: INT 1 (Table 563, reset 1)
        apply(write(Kvasir::Peripheral::CLOCKS::Registers<>::CLK_PERI_DIV::_int,
                    value<std::uint32_t{1}>()));
#else
        // the RP2040 has no ENABLED bit: two cycles of the old source (clk_sys or slower) pass in these
        detail::spinCycles<64>();
#endif
        apply(write(PERI_CLOCK::AUXSRCValC::clk_sys));

        // clk_usb / clk_adc off before PLL_USB is reset below (their destinations are in reset already)
        apply(clear(USB_CLOCK::enable), clear(ADC_CLOCK::enable));

        // A clk_sys resus (Clocks::Resus) survives a core reset: the clocks block is not
        // reset by SYSRESETREQ, so after an unhandled event every debugger reset boots with
        // clk_sys forced to clk_ref, the PLL switch below has no effect, and the flash
        // timing peripheryClockInit() applies for ClockSpeed reads garbage at 12 MHz - a
        // HardFault before main, on every reset, until the resus is released.
        // clk_sys is on clk_ref now (CLK_SYS_SELECTED said so), so releasing it changes nothing here.
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
        Kvasir::Register::waitUntil<Kvasir::Chip::XoscStableBound>(
          Kvasir::Register::isSet(XOSC::STATUS::stable));
        {   //sys pll
            // reset pll
            apply(set(RST::RESET::pll_sys));
            apply(clear(RST::RESET::pll_sys));
            Kvasir::Register::waitUntil<Kvasir::Chip::ResetDoneBound>(
              Kvasir::Register::isSet(RST::RESET_DONE::pll_sys));

            apply(PLL::CS::overrideDefaults(clear(PLL::CS::bypass),
                                            write(PLL::CS::refdiv, value<pllSettings.refdiv>())));

            apply(write(PLL::FBDIV_INT::fbdiv_int, value<pllSettings.fbdiv>()));

            apply(PLL::PWR::overrideDefaults(clear(PLL::PWR::vcopd),
                                             clear(PLL::PWR::pd),
                                             set(PLL::PWR::postdivpd)));

            // wait for PLL lock
            Kvasir::Register::waitUntil<Kvasir::Chip::PllLockBound>(
              Kvasir::Register::isSet(PLL::CS::lock));

            apply(
              PLL::PRIM::overrideDefaults(write(PLL::PRIM::postdiv1, value<pllSettings.pd1>()),
                                          write(PLL::PRIM::postdiv2, value<pllSettings.pd2>())));

            apply(PLL::PWR::overrideDefaults(clear(PLL::PWR::vcopd),
                                             clear(PLL::PWR::pd),
                                             clear(PLL::PWR::postdivpd)));

            // clk_sys to the PLL: the aux mux first, while the glitchless mux is still on clk_ref, then the switch
            apply(write(SYS_CLOCK::AUXSRCValC::clksrc_pll_sys));
            apply(write(SYS_CLOCK::SRCValC::clksrc_clk_sys_aux));
            Kvasir::Register::waitUntil<Kvasir::Chip::ClkSelectedBound>(
              detail::selectedIs<SYS_SELECTED, 1>);

            // set ref clock to xosc, undivided (boot may leave CLK_REF_DIV != 1)
            apply(write(REF_CLOCK_DIV::_int, value<std::uint32_t{1}>()));
            apply(write(REF_CLOCK::SRCValC::xosc_clksrc));
            Kvasir::Register::waitUntil<Kvasir::Chip::ClkSelectedBound>(
              detail::selectedIs<REF_SELECTED, 2>);

            // enable periphery clock (its AUXSRC is clk_sys since above)
            apply(set(PERI_CLOCK::enable));
        }

        {   //usb pll
            // reset pll
            apply(set(RST::RESET::pll_usb));
            apply(clear(RST::RESET::pll_usb));
            Kvasir::Register::waitUntil<Kvasir::Chip::ResetDoneBound>(
              Kvasir::Register::isSet(RST::RESET_DONE::pll_usb));

            apply(USBPLL::CS::overrideDefaults(
              clear(USBPLL::CS::bypass),
              write(USBPLL::CS::refdiv, value<usb_pllSettings.refdiv>())));

            apply(write(USBPLL::FBDIV_INT::fbdiv_int, value<usb_pllSettings.fbdiv>()));

            apply(USBPLL::PWR::overrideDefaults(clear(USBPLL::PWR::vcopd),
                                                clear(USBPLL::PWR::pd),
                                                set(USBPLL::PWR::postdivpd)));

            // wait for PLL lock
            Kvasir::Register::waitUntil<Kvasir::Chip::PllLockBound>(
              Kvasir::Register::isSet(USBPLL::CS::lock));

            apply(USBPLL::PRIM::overrideDefaults(
              write(USBPLL::PRIM::postdiv1, value<usb_pllSettings.pd1>()),
              write(USBPLL::PRIM::postdiv2, value<usb_pllSettings.pd2>())));

            apply(USBPLL::PWR::overrideDefaults(clear(USBPLL::PWR::vcopd),
                                                clear(USBPLL::PWR::pd),
                                                clear(USBPLL::PWR::postdivpd)));

            // clk_usb / clk_adc: no glitchless mux - the source while disabled, then the enable
            apply(write(USB_CLOCK::AUXSRCValC::clksrc_pll_usb),
                  write(ADC_CLOCK::AUXSRCValC::clksrc_pll_usb));
            apply(set(USB_CLOCK::enable), set(ADC_CLOCK::enable));
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
        Kvasir::detail::enableQuadIo();
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
