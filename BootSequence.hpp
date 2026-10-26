#pragma once
// The RP family's first Startup step (Kvasir_SDK StartUp.hpp's reset entry: cleanCore, then this), before .data and
// .bss exist: registers only, no variables, every step always_inline so nothing becomes a call into code that needs
// initialised memory. The order is fixed here; each chip gives its own steps and field lists as Rp::BootTraits
// (chip/BootTraits.hpp):
//   Core::startup()
//   Multicore::resetCore1()        before this image touches RAM: a debugger's flash-and-reset or a SYSRESETREQ
//                                  restarts core 0 only, and a core 1 the image before launched keeps running out of
//                                  the RAM this one is about to fill (PSM FRCE_OFF.PROC1: RP2040 datasheet Table 198,
//                                  RP2350 datasheet Table 531 - the one FRCE_OFF bit erratum RP2350-E19 allows to set)
//   Traits::afterCore1Parked()     RP2040: the XIP cache flush through the boot ROM, watchdog off; RP2350: watchdog
//                                  tick, CPACR, RCP salt, watchdog off unless a TBYB trial, EXTEXCLALL
//   Traits::holdBlocksInReset()    every block of the chip's table (chip/Blocks.hpp) into reset, the rest kept
//   Traits::afterResetsHeld()      RP2040: PROC0_NMI_MASK; RP2350: nothing
//   Traits::selectWatchdogDomains() PSM.WDSEL = WatchdogDomains::onTimeout, RESETS.WDSEL every block (setEveryField)
//   PM::markChipReset()            WATCHDOG SCRATCH1: whether CHIP_RESET names this reset or an earlier boot's, and
//                                  on the RP2040 whether the cores alone were reset (the CLOCKS block still on the
//                                  image's sources) - chip_reset.hpp, ResetCauseRules.hpp; before anything reads the
//                                  reset cause, and before the clock init
#include "Multicore.hpp"
#include "chip_reset.hpp"
#include "core/core.hpp"
#include "kvasir/Hal/Block.hpp"
#include "kvasir/Register/Register.hpp"

#include <cstdint>
#include <type_traits>

// Whether this image arms Kvasir::CoreResetGuard (chip/CoreResetGuard.hpp): multicore flash images. A plain 0/1 for
// `#if` - the chip's StartUp.hpp includes the guard under it, below FirstInitStep.
#if defined(KVASIR_MULTICORE) && KVASIR_MULTICORE && !(defined(KVASIR_RAM_ONLY) && KVASIR_RAM_ONLY)
    #define KVASIR_RP_CORE_RESET_GUARDED 1
#else
    #define KVASIR_RP_CORE_RESET_GUARDED 0
#endif

namespace Kvasir::Rp {
template<typename Traits>
[[gnu::always_inline]] inline void firstInit() {
    Startup::Core::startup();
    Multicore::resetCore1();
    Traits::afterCore1Parked();
    Traits::holdBlocksInReset();
    Traits::afterResetsHeld();
    Traits::selectWatchdogDomains();
    PM::markChipReset();
}

namespace Detail {
    template<typename Table>
    struct HoldInReset;

    template<typename... Rows>
    struct HoldInReset<Hal::BlockTable<Rows...>> {
        template<typename... Others>
        [[gnu::always_inline]] static void run(Others... others) {
            Register::apply(Register::set(Rows::reset)..., others...);
        }
    };
}   // namespace Detail

// One RESETS.RESET write: every row of the block table `Table` held in reset, plus the chip's `others` (the blocks
// without a row: set the ones no driver releases, clear the ones the boot keeps running).
template<typename Table,
         typename... Others>
[[gnu::always_inline]] inline void holdBlocksInReset(Others... others) {
    Detail::HoldInReset<Table>::run(others...);
}

namespace Detail {
    template<typename Fields>
    struct EveryField;

    template<typename... Fs>
    struct EveryField<Register::Fields<Fs...>> {
        [[gnu::always_inline]] static void set() {
            Register::apply(Register::set(std::remove_cvref_t<Fs>{})...);
        }
    };
}   // namespace Detail

// Every field of register R set, in one apply(): R::fields, the field list svd_converter generates
// (kvasir/Register/Fields.hpp) - RESETS.WDSEL's "every block" without a hand list that can miss one.
template<typename R>
[[gnu::always_inline]] inline void setEveryField() {
    Detail::EveryField<typename R::fields>::set();
}

// The guard of a single-core or RAM image: nothing to arm.
struct NoGuard {
    static void runtimeInit() {}
};

// What both chips' SecondaryCoreInit share (kvasir/StartUp/SecondaryCore.hpp calls them from core 0). `Guard`:
// Kvasir::CoreResetGuard in a multicore flash image, else NoGuard.
template<typename Guard>
struct SecondaryCoreInitBase {
    // The FIFO handshake with the boot ROM's holding pen, and the PSM reset that puts core 1 back into it.
    [[nodiscard]] static bool launch(std::uint32_t entry,
                                     std::uint32_t sp,
                                     std::uint32_t vtor) {
        return Multicore::launchCore1(entry, sp, vtor);
    }

    static void reset() { Multicore::resetCore1(); }

    // Called by SecondaryCore's runtimeInit before core 1 starts, unless its list holds
    // Kvasir::Startup::NoCoreResetGuard (chip/CoreResetGuard.hpp). No-op in a RAM image.
    static void armCoreResetGuard() { Guard::runtimeInit(); }

    // Also from SecondaryCore's runtimeInit: core 0's NMI routed for Port::StopOtherCore on core 1, and the
    // vector that parks it (Multicore.hpp Core0Park).
    static void armOtherCoreStop() { Multicore::Core0Park::arm(); }

    using PrimaryIsr = Multicore::Core0Park::Isr;
};
}   // namespace Kvasir::Rp
