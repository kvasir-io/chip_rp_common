// The crystal check's counts (CrystalCounts.hpp, clock_config.hpp crystalSettled): when they settle, and the record's
// statistics - the worst step split at the run that settled the check, the counts discarded before it. Building is
// most of the test (static_assert on fixed sequences: a crystal still starting up into the first count, a run broken by
// a failed count, a check that never settles). main() then runs a seeded sweep of made-up sequences at run time against
// two references: the loop crystalSettled() ran before the split (its decision and fields must not change), and the
// split worked out afterwards from the whole sequence (instead of while counting).
#include "CrystalCounts.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

namespace {
template<std::uint32_t Agree, std::uint32_t ToleranceShift, bool Record = true>
using Counts = Kvasir::Clocks::Detail::CrystalCounts<Agree, ToleranceShift, Record>;

// crystalSettled()'s loop over @p seq (a count past its end: 0, a failed count), at most @p maxCounts counts
template<std::uint32_t Agree,
         std::uint32_t ToleranceShift,
         bool          Record = true>
constexpr Counts<Agree,
                 ToleranceShift,
                 Record>
check(std::span<std::uint32_t const> seq,
      std::uint32_t                  maxCounts = 256) {
    Counts<Agree, ToleranceShift, Record> c{};
    while(c.n != maxCounts) {
        std::uint32_t const count = c.n < seq.size() ? seq[c.n] : 0U;
        if(c.add(count)) { break; }
    }
    c.finish();
    return c;
}

// The default config's (XoscMeasured: 4 in a row within 1/256)
using Default = Counts<4, 8>;

constexpr Default checkDefault(std::span<std::uint32_t const> seq) { return check<4, 8>(seq); }

// ---- a crystal still starting up into the first count (FC0 counts in 1/32 kHz): 19 % low, discarded; the four after
// it agree. The overall worst step (227 104 on 1 193 824: 190 232 ppm) is the start-up's, not the run's.
constexpr std::array<std::uint32_t, 5> startingUp{1'193'824,
                                                  1'420'928,
                                                  1'421'280,
                                                  1'421'200,
                                                  1'421'120};
constexpr Default                      su = checkDefault(startingUp);
static_assert(su.settled && su.n == 5 && su.discarded() == 1,
              "settled in 5 counts, the first discarded");
static_assert(su.worstStep() == 227'104 && su.worstStepFrom() == 1'193'824,
              "worstStep: the start-up's step");
static_assert(su.before == 227'104 && su.beforeFrom == 1'193'824,
              "the step into the run is before it, not in it");
static_assert(su.inRun == 352 && su.inRunFrom == 1'420'928,
              "the run's own worst step: 247 ppm");
static_assert(su.zeroCounts == 0 && su.bestRun == 4 && su.minCount() == 1'193'824
                && su.hi == 1'421'280 && su.previous == 1'421'120,
              "the rest of the record as before");

// ---- a run broken by a failed count: its agreeing step (5 072 on 1 420 928: 3 569 ppm) is before the run that settles
constexpr std::array<std::uint32_t, 7>
                  broken{1'420'928, 1'426'000, 0, 1'421'000, 1'421'080, 1'421'040, 1'421'000};
constexpr Default br = checkDefault(broken);
static_assert(br.settled && br.n == 7 && br.discarded() == 3,
              "the two counts and the failed one discarded");
static_assert(br.before == 5'072 && br.beforeFrom == 1'420'928,
              "the broken run's step before the settled run");
static_assert(br.inRun == 80 && br.inRunFrom == 1'421'000,
              "the settled run's worst: 56 ppm");
static_assert(br.zeroCounts == 1 && br.worstStep() == 5'072 && br.worstStepFrom() == 1'420'928);

// ---- no run settles (runs of three, each broken by a failed count, of four needed), the largest step only in the
// last run, which nothing breaks: finish() puts it before - every step of a check that did not settle is
constexpr auto neverSettles = [] {
    std::array<std::uint32_t, 256> s{};
    for(std::size_t g = 0; g != 64; ++g) {
        s[4 * g]     = 0;
        s[4 * g + 1] = 1'421'000;
        s[4 * g + 2] = 1'421'040;
        s[4 * g + 3] = g == 63 ? 1'424'040U : 1'421'000U;   // 3 000 on 1 421 040: within 1/256
    }
    return s;
}();
constexpr Default ns = checkDefault(neverSettles);
static_assert(!ns.settled && ns.n == 256 && ns.discarded() == 256,
              "not settled: every count discarded");
static_assert(ns.inRun == 0 && ns.inRunFrom == 0,
              "no settled run");
static_assert(ns.before == 3'000 && ns.beforeFrom == 1'421'040,
              "the last run's step, by the final fold");
static_assert(ns.zeroCounts == 64 && ns.bestRun == 3 && ns.worstStep() == 3'000);

// ---- settled in the minimum: nothing before the run, the earliest of equal steps
constexpr std::array<std::uint32_t, 4> minimum{800'000, 800'010, 800'020, 800'030};
constexpr Default                      mi = checkDefault(minimum);
static_assert(mi.settled && mi.n == 4 && mi.discarded() == 0 && mi.before == 0
              && mi.beforeFrom == 0);
static_assert(mi.inRun == 10 && mi.inRunFrom == 800'000 && mi.worstStepFrom() == 800'000);

// ---- a tie between a broken run's step and the settled run's: worstStep names the earlier, the broken run's
constexpr std::array<std::uint32_t, 7> tie{900'000, 900'100, 0, 950'000, 950'100, 950'200, 950'300};
constexpr Default                      ti = checkDefault(tie);
static_assert(ti.settled && ti.before == 100 && ti.beforeFrom == 900'000 && ti.inRun == 100
              && ti.inRunFrom == 950'000);
static_assert(ti.worstStep() == 100 && ti.worstStepFrom() == 900'000,
              "the earlier step wins the tie");

// ---- equal steps in two broken runs: the earlier run's stays before; two equal start-up steps: the first
constexpr std::array<std::uint32_t, 10>
  twoBroken{900'000, 900'100, 0, 910'000, 910'100, 0, 950'000, 950'010, 950'020, 950'030};
static_assert(checkDefault(twoBroken).before == 100 && checkDefault(twoBroken).beforeFrom == 900'000
                && checkDefault(twoBroken).worstStepFrom() == 900'000
                && checkDefault(twoBroken).discarded() == 6,
              "the fold keeps the earlier run's step on a tie");
constexpr std::array<std::uint32_t, 6> twoJumps{100'000,
                                                200'000,
                                                300'000,
                                                300'010,
                                                300'020,
                                                300'030};
static_assert(checkDefault(twoJumps).before == 100'000
                && checkDefault(twoJumps).beforeFrom == 100'000
                && checkDefault(twoJumps).discarded() == 2,
              "of two equal steps before the run, the first");

// ---- the tolerance's edge: a step of previous >> 8 agrees, one more does not (agree 2: one step settles)
constexpr std::array<std::uint32_t, 2> onEdge{256'000, 257'000};
constexpr std::array<std::uint32_t, 2> pastEdge{256'000, 257'001};
static_assert(check<2,
                    8>(onEdge,
                       2)
                  .settled
                && check<2,
                         8>(onEdge,
                            2)
                       .inRun
                     == 1'000,
              "1 000 on 256 000 (256 000 >> 8) agrees");
static_assert(!check<2,
                     8>(pastEdge,
                        2)
                  .settled
                && check<2,
                         8>(pastEdge,
                            2)
                       .before
                     == 1'001,
              "1 001 on 256 000 does not");

// ---- a failed count ends a run and is no step; all failed: nothing at all
constexpr std::array<std::uint32_t, 0> none{};
constexpr Default                      allZero = checkDefault(none);
static_assert(!allZero.settled && allZero.n == 256 && allZero.zeroCounts == 256
              && allZero.minCount() == 0 && allZero.hi == 0 && allZero.bestRun == 0
              && allZero.worstStep() == 0);

// ---- unrecorded (XoscStartupConfig's recordCheck off): the same decision and counts, nothing else
static_assert(check<4,
                    8,
                    false>(startingUp)
                .settled
              && check<4,
                       8,
                       false>(startingUp)
                     .n
                   == 5
              && check<4,
                       8,
                       false>(startingUp)
                     .inRun
                   == 0
              && check<4,
                       8,
                       false>(startingUp)
                     .zeroCounts
                   == 0);
static_assert(!check<4,
                     8,
                     false>(neverSettles)
                 .settled
              && check<4,
                       8,
                       false>(neverSettles)
                     .n
                   == 256);

// ---- reference 1: crystalSettled()'s loop before the split, as it was (chip_rp_common 2a1f446)
struct Old {
    bool          settled;
    std::uint32_t n, run, zeros, best, lo, hi, last, worst, worstFrom;
};

template<std::uint32_t Agree,
         std::uint32_t ToleranceShift>
Old oldLoop(std::vector<std::uint32_t> const& seq,
            std::uint32_t                     maxCounts) {
    std::uint32_t previous  = 0;
    std::uint32_t run       = 0;
    std::uint32_t n         = 0;
    std::uint32_t zeros     = 0;
    std::uint32_t best      = 0;
    std::uint32_t lo        = 0xFFFF'FFFFU;
    std::uint32_t hi        = 0;
    std::uint32_t worst     = 0;
    std::uint32_t worstFrom = 0;
    bool          settled   = false;
    while(n != maxCounts) {
        std::uint32_t const count = n < seq.size() ? seq[n] : 0U;
        ++n;
        std::uint32_t const diff = count > previous ? count - previous : previous - count;
        bool const agrees = count != 0 && previous != 0 && diff <= (previous >> ToleranceShift);
        run               = agrees ? run + 1 : 1;
        if(count == 0) {
            ++zeros;
        } else {
            lo   = count < lo ? count : lo;
            hi   = count > hi ? count : hi;
            best = run > best ? run : best;
            if(previous != 0 && diff > worst) {
                worst     = diff;
                worstFrom = previous;
            }
        }
        previous = count;
        if(count != 0 && run >= Agree) {
            settled = true;
            break;
        }
    }
    return {settled, n, run, zeros, best, hi == 0 ? 0 : lo, hi, previous, worst, worstFrom};
}

// ---- reference 2: the split from the whole sequence afterwards. The settled run is the last Agree counts; a step
// (counts i - 1 and i, both nonzero) is in it when count i - 1 is.
struct Split {
    std::uint32_t inRun, inRunFrom, before, beforeFrom, discarded;
};

Split splitAfterwards(std::vector<std::uint32_t> const& seq,
                      std::uint32_t                     n,
                      bool                              settled,
                      std::uint32_t                     agree) {
    Split               s{};
    std::uint32_t const runStart = settled ? n - agree : n;
    s.discarded                  = runStart;
    for(std::uint32_t i = 1; i < n; ++i) {
        std::uint32_t const a = i - 1 < seq.size() ? seq[i - 1] : 0U;
        std::uint32_t const b = i < seq.size() ? seq[i] : 0U;
        if(a == 0 || b == 0) { continue; }
        std::uint32_t const d = b > a ? b - a : a - b;
        if(i - 1 >= runStart) {
            if(d > s.inRun) {
                s.inRun     = d;
                s.inRunFrom = a;
            }
        } else if(d > s.before) {
            s.before     = d;
            s.beforeFrom = a;
        }
    }
    return s;
}

std::uint32_t rng = 0x2545'F491U;   // xorshift32, seeded: the same sequences on every run

std::uint32_t next() {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

// counts around a base as a crystal against a ROSC gives them: mostly within the tolerance, some failed counts, some
// far off (a crystal still starting up, a disturbed reference), a base that moves now and then
std::vector<std::uint32_t> madeUp(std::uint32_t length,
                                  std::uint32_t toleranceShift) {
    std::vector<std::uint32_t> s;
    std::uint32_t              base = 200'000 + next() % 1'500'000;
    for(std::uint32_t i = 0; i != length; ++i) {
        std::uint32_t const r = next() % 100;
        std::uint32_t const t = base >> toleranceShift;
        if(r < 8) {
            s.push_back(0);
        } else if(r < 18) {
            s.push_back(base - base / 5 + next() % (2 * (base / 5)));   // up to 20 % off
        } else {
            if(r < 22) { base = 200'000 + next() % 1'500'000; }
            s.push_back(base - t + next() % (2 * t + t / 2 + 1));   // around the tolerance's edge
        }
    }
    return s;
}

int failures = 0;

void expect(bool          good,
            char const*   what,
            std::uint32_t seqNo) {
    if(!good) {
        ++failures;
        if(failures <= 20) {
            std::printf("FAIL sequence %u: %s\n", static_cast<unsigned>(seqNo), what);
        }
    }
}

template<std::uint32_t Agree,
         std::uint32_t ToleranceShift>
void sweep(std::uint32_t sequences,
           std::uint32_t maxCounts) {
    for(std::uint32_t k = 0; k != sequences; ++k) {
        auto const seq = madeUp(1 + next() % (maxCounts + 8), ToleranceShift);
        auto const c   = check<Agree, ToleranceShift>(seq, maxCounts);
        auto const u   = check<Agree, ToleranceShift, false>(seq, maxCounts);
        Old const  o   = oldLoop<Agree, ToleranceShift>(seq, maxCounts);
        expect(c.settled == o.settled && c.n == o.n && c.run == o.run, "the decision as before", k);
        expect(u.settled == o.settled && u.n == o.n, "unrecorded: the same decision", k);
        expect(c.zeroCounts == o.zeros && c.bestRun == o.best && c.minCount() == o.lo
                 && c.hi == o.hi && c.previous == o.last,
               "zeros, best run, range and last count as before",
               k);
        expect(c.worstStep() == o.worst && c.worstStepFrom() == o.worstFrom,
               "worstStep / worstStepFrom as before (the larger half, the earlier on a tie)",
               k);
        Split const s = splitAfterwards(seq, c.n, c.settled, Agree);
        expect(c.inRun == s.inRun && c.inRunFrom == s.inRunFrom, "the settled run's worst step", k);
        expect(c.before == s.before && c.beforeFrom == s.beforeFrom,
               "the worst step before the run",
               k);
        expect(c.discarded() == s.discarded, "the counts discarded", k);
        expect(!c.settled || c.inRun <= (c.inRunFrom >> ToleranceShift),
               "the run's step within the tolerance",
               k);
    }
}
}   // namespace

int main() {
    // a fixed sequence once more at run time (a constexpr-only path cannot hide a difference)
    std::vector<std::uint32_t> const seq(startingUp.begin(), startingUp.end());
    auto const                       c = check<4, 8>(seq);
    expect(c.settled && c.discarded() == 1 && c.before == 227'104 && c.inRun == 352,
           "start-up at run time",
           0);

    sweep<4, 8>(20'000, 256);   // the default config
    sweep<4, 8>(20'000, 12);    // few counts: many checks that do not settle
    sweep<2, 8>(10'000, 256);   // the least agree
    sweep<8, 9>(10'000, 256);   // stricter: 8 in a row within 1/512
    sweep<3, 6>(10'000, 64);
    std::printf("crystal_counts_test: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
