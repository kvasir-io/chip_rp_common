#pragma once
// The crystal check's counts one by one, without chip headers (clock_config.hpp crystalSettled, XoscMeasured): whether
// they settled - Agree counts in a row, each nonzero and within 1/2^ToleranceShift of the one before - and, where the
// check is recorded (Clocks::XoscStartupConfig's recordCheck), what Clocks::CrystalCheck keeps of them. The boot runs it
// as a local before initMemory() (registers and the stack, stored into the record once at the end); the host test
// (tests/crystal_counts_test.cpp) feeds it sequences.
#include <cstdint>

namespace Kvasir { namespace Clocks { namespace Detail {
    template<std::uint32_t Agree, std::uint32_t ToleranceShift, bool Record>
    struct CrystalCounts {
        static_assert(Agree >= 2,
                      "XoscMeasured compares counts: agree is at least 2");

        std::uint32_t n{0};          // counts taken
        std::uint32_t previous{0};   // the last count; 0: none yet, or it failed
        std::uint32_t run{0};        // counts in a row, each within the tolerance of the one before
        bool          settled{false};

        // Record only. The counts of 0 (not finished, or the crystal stopped in one: DIED), the longest run, the
        // nonzero counts' range (minCount(): 0 when there was none).
        std::uint32_t zeroCounts{0};
        std::uint32_t bestRun{0};
        std::uint32_t lo{0xFFFF'FFFFU};
        std::uint32_t hi{0};
        // The largest step of the run going on (once settled: of the run that settled the check), and the largest
        // before it - the step into the run and every broken run's included; after finish() of a check that did not
        // settle, every step. A step is the difference of two nonzero counts in a row, kept with the count before it
        // (the tolerance there was that count >> ToleranceShift); the earlier step wins a tie.
        std::uint32_t inRun{0};
        std::uint32_t inRunFrom{0};
        std::uint32_t before{0};
        std::uint32_t beforeFrom{0};

        /// One more count, in 1/32 kHz (0 for one that failed): whether the counts settled with it. Not called again
        /// once they did.
        constexpr bool add(std::uint32_t count) {
            ++n;
            std::uint32_t const diff = count > previous ? count - previous : previous - count;
            bool const agrees = count != 0 && previous != 0 && diff <= (previous >> ToleranceShift);
            run               = agrees ? run + 1 : 1;
            if constexpr(Record) { note(count, diff, agrees); }
            previous = count;
            settled  = count != 0 && run >= Agree;
            return settled;
        }

        /// After the last count: the run going on, if it did not settle the check, belongs to what came before.
        constexpr void finish() {
            if constexpr(Record) {
                if(!settled) { foldRun(); }
            }
        }

        /// The counts before the run that settled the check; all of them when none did.
        [[nodiscard]] constexpr std::uint32_t discarded() const { return settled ? n - run : n; }

        [[nodiscard]] constexpr std::uint32_t minCount() const { return hi == 0 ? 0 : lo; }

        /// The largest step of all, with its count before - every step is in one of the two halves, and the steps
        /// before the run all came first.
        [[nodiscard]] constexpr std::uint32_t worstStep() const {
            return inRun > before ? inRun : before;
        }

        [[nodiscard]] constexpr std::uint32_t worstStepFrom() const {
            return inRun > before ? inRunFrom : beforeFrom;
        }

    private:
        constexpr void note(std::uint32_t count,
                            std::uint32_t diff,
                            bool          agrees) {
            if(!agrees) { foldRun(); }   // a count that does not agree ends the run going on
            if(count == 0) {
                ++zeroCounts;
                return;
            }
            lo      = count < lo ? count : lo;
            hi      = count > hi ? count : hi;
            bestRun = run > bestRun ? run : bestRun;
            // no step: the first count, or the one after a failed count
            if(previous == 0) { return; }
            if(agrees) {
                if(diff > inRun) {
                    inRun     = diff;
                    inRunFrom = previous;
                }
            } else if(diff > before) {
                before     = diff;
                beforeFrom = previous;
            }
        }

        // the run going on is broken (or ended unsettled): its steps go to what came before the run that settles
        constexpr void foldRun() {
            if(inRun > before) {
                before     = inRun;
                beforeFrom = inRunFrom;
            }
            inRun     = 0;
            inRunFrom = 0;
        }
    };
}}}   // namespace Kvasir::Clocks::Detail
