// The RP reset-cause decision (ResetCauseRules.hpp) for both chips: a reset of the cores alone (SYSRESETREQ, the
// firmware's or a debugger's) after a RUN-pin, power-on or debugger chip reset is `core_reset`, not the chip reset
// CHIP_RESET still names; the mark each boot writes into WATCHDOG SCRATCH1 says which. Building is most of the test
// (static_assert); main() runs the same table at run time through volatiles.
#include "ResetCauseRules.hpp"

#include <cstdint>
#include <cstdio>

namespace PM = Kvasir::PM;
namespace D  = PM::Detail;
namespace M  = PM::ChipResetMark;
using C      = PM::ResetCause;

namespace {
// What the boots of a sequence of resets read: the mark each boot finds, given whether the reset cleared it.
constexpr std::uint32_t afterReset(std::uint32_t mark,
                                   bool          resetClearsMark) {
    return M::next(resetClearsMark ? 0U : mark, false);
}

// ---- the mark
static_assert(M::next(0U,
                      false)
                == M::fresh,
              "no mark (reset value 0): CHIP_RESET is this boot's");
static_assert(M::next(0xDEAD'BEEFU,
                      false)
                == M::fresh,
              "a foreign word: this boot's");
static_assert(M::next(M::fresh,
                      false)
                  == M::earlier
                && M::next(M::earlier,
                           false)
                     == M::earlier
                && M::next(M::coresOnly,
                           false)
                     == M::earlier,
              "a mark that survived: an earlier boot's");
static_assert(!M::chipResetIsEarlier(M::fresh) && M::chipResetIsEarlier(M::earlier)
              && !M::chipResetIsEarlier(0U));
static_assert(M::fresh != M::earlier && M::fresh != 0U && M::earlier != 0U);
// RP2040: the cores alone were reset (the clocks kept the image's sources)
static_assert(M::next(M::earlier,
                      true)
                  == M::coresOnly
                && M::next(M::coresOnly,
                           true)
                     == M::coresOnly
                && M::next(M::fresh,
                           true)
                     == M::coresOnly,
              "a cores-only reset over any mark");
static_assert(M::next(0U,
                      true)
                == M::fresh,
              "no mark: CHIP_RESET is this boot's, whatever the clocks say");
static_assert(M::chipResetIsEarlier(M::coresOnly) && M::isCoresOnly(M::coresOnly)
                && !M::isCoresOnly(M::earlier) && !M::isCoresOnly(M::fresh) && !M::isCoresOnly(0U),
              "CoresOnly is an Earlier that says more");
static_assert(M::coresOnly != M::fresh && M::coresOnly != M::earlier && M::coresOnly != 0U);
// the clocks' one-hot SELECTED words (RP2040 Tables 222/225): reset value 0x1 each
static_assert(!D::clocksKeptImageSources(1U,
                                         1U),
              "clk_ref on the ROSC, clk_sys on clk_ref: their reset sources");
static_assert(D::clocksKeptImageSources(1U << 2U,
                                        1U << 1U),
              "clk_ref on the XOSC, clk_sys on its aux (PLL_SYS)");
static_assert(D::clocksKeptImageSources(1U << 2U,
                                        1U),
              "clk_ref on the XOSC alone (a degraded boot on the crystal)");
static_assert(D::clocksKeptImageSources(1U,
                                        1U << 1U));
// RUN reset, then two SYSRESETREQs, then RUN again
static_assert(afterReset(0U,
                         true)
              == M::fresh);
static_assert(afterReset(afterReset(0U,
                                    true),
                         false)
              == M::earlier);
static_assert(afterReset(afterReset(afterReset(0U,
                                               true),
                                    false),
                         false)
              == M::earlier);
static_assert(afterReset(afterReset(afterReset(0U,
                                               true),
                                    false),
                         true)
              == M::fresh);

// ---- RP2350
constexpr D::Rp2350Reset rp2350(bool earlier) {
    return D::Rp2350Reset{.wdForce          = false,
                          .wdTimer          = false,
                          .chipResetEarlier = earlier,
                          .hadRescue        = false,
                          .hadDpResetReq    = false,
                          .hadGlitchDetect  = false,
                          .hadBor           = false,
                          .hadPor           = false,
                          .hadRunLow        = false,
                          .hadSwcorePd      = false};
}

constexpr D::Rp2350Reset rp2350RunLow(bool earlier) {
    auto r      = rp2350(earlier);
    r.hadRunLow = true;
    return r;
}

static_assert(D::classify(rp2350RunLow(false)) == C::runLow,
              "a RUN-pin reset: run_low");
static_assert(D::classify(rp2350RunLow(true)) == C::coreReset,
              "a SYSRESETREQ after it (HAD_RUN_LOW kept, the mark kept): core_reset, not run_low");
static_assert(
  [] {
      auto r   = rp2350(true);
      r.hadPor = true;
      return D::classify(r) == C::coreReset;
  }(),
  "a SYSRESETREQ after a power-on: core_reset, not por");
static_assert(
  [] {
      auto r          = rp2350(true);
      r.hadDpResetReq = true;
      return D::classify(r) == C::coreReset;
  }(),
  "every chip-level reset clears the mark: after it nothing of CHIP_RESET is this boot's");
static_assert(
  [] {
      auto r          = rp2350(false);
      r.hadDpResetReq = true;
      return D::classify(r) == C::dpResetReq;
  }(),
  "a debugger's chip-level reset (CDBGPWRUPREQ) itself");
static_assert(
  [] {
      auto r    = rp2350RunLow(true);
      r.wdTimer = true;
      return D::classify(r) == C::watchdogTimer;
  }(),
  "a watchdog reset (REASON set; it keeps the mark): the watchdog");
static_assert([] {
    auto r    = rp2350RunLow(false);
    r.wdForce = true;
    return D::classify(r) == C::watchdogForce;
}());
static_assert(D::classify(rp2350(false)) == C::unknown,
              "no bit at all");
static_assert(D::classify(rp2350(true)) == C::coreReset);

// ---- RP2040
constexpr D::Rp2040Reset rp2040(bool earlier) {
    return D::Rp2040Reset{.coreResetCarried = false,
                          .coresOnly        = false,
                          .wdForce          = false,
                          .wdTimer          = false,
                          .chipResetEarlier = earlier,
                          .hadPsmRestart    = false,
                          .hadPor           = false,
                          .hadRun           = false};
}

static_assert(
  [] {
      auto r   = rp2040(false);
      r.hadRun = true;
      return D::classify(r) == C::runLow;
  }(),
  "RP2040: a RUN-pin reset");
static_assert(
  [] {
      auto r   = rp2040(true);
      r.hadRun = true;
      return D::classify(r) == C::coreReset;
  }(),
  "RP2040: a reset of core 0 after it: core_reset");
static_assert(
  [] {
      auto r   = rp2040(true);
      r.hadPor = true;
      return D::classify(r) == C::coreReset;
  }(),
  "RP2040: after a power-on too");
static_assert(
  [] {
      auto r          = rp2040(true);
      r.hadPsmRestart = true;
      return D::classify(r) == C::rescue;
  }(),
  "RP2040: a rescue keeps the scratch words - HAD_PSM_RESTART is taken as it reads");
static_assert(
  [] {
      auto r    = rp2040(true);
      r.hadRun  = true;
      r.wdTimer = true;
      return D::classify(r) == C::watchdogTimer;
  }(),
  "RP2040: REASON before CHIP_RESET (REASON is this boot's unless the cores alone were reset)");
static_assert(
  [] {
      auto r      = rp2040(true);
      r.wdTimer   = true;
      r.coresOnly = true;
      return D::classify(r) == C::coreReset;
  }(),
  "RP2040: a debugger reset after a watchdog timeout - REASON still reads timer, the clocks say "
  "core_reset");
static_assert(
  [] {
      auto r      = rp2040(true);
      r.wdForce   = true;
      r.coresOnly = true;
      return D::classify(r) == C::coreReset;
  }(),
  "RP2040: the same after a forced watchdog reboot");
static_assert(
  [] {
      auto r             = rp2040(false);
      r.hadRun           = true;
      r.coreResetCarried = true;
      return D::classify(r) == C::coreReset;
  }(),
  "RP2040: CoreResetGuard's carried word first");
static_assert(D::classify(rp2040(false)) == C::unknown);
}   // namespace

int main() {
    // the same decisions at run time (a constexpr-only path cannot hide a difference)
    bool volatile earlier   = true;
    bool const volatile run = true;
    auto r                  = rp2350RunLow(earlier);
    r.hadRunLow             = run;
    auto q                  = rp2040(earlier);
    q.hadRun                = run;
    std::uint32_t volatile found{M::fresh};
    int failures = 0;
    if(D::classify(r) != C::coreReset) { ++failures; }
    if(D::classify(q) != C::coreReset) { ++failures; }
    if(M::next(found, false) != M::earlier) { ++failures; }
    bool volatile coresOnly = true;
    if(M::next(found, coresOnly) != M::coresOnly) { ++failures; }
    std::uint32_t volatile refSelected{1U};
    if(D::clocksKeptImageSources(refSelected, 1U)) { ++failures; }
    earlier = false;
    if(D::classify(rp2350RunLow(earlier)) != C::runLow) { ++failures; }
    std::printf("reset_cause_test: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
