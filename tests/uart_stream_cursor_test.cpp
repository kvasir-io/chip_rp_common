// UartStream's RX cursor (UARTStreamCursor.hpp) against a simulated DMA channel: the channel writes each element's
// global position into the ring, stops at a segment's end with its completion pending, the ISR runs late and re-arms
// or stalls, the reader reads what writtenTotal() says, pauses, commits part of a run. Random interleavings, and runs
// whose totals start just below 2^32. Checked after every step:
//   - the reader never sees an element not yet written, or one already overwritten (the value is its position)
//   - the channel never overwrites an unread element
//   - the ISR stalls exactly when the next segment would overwrite unread data
//   - readCommit() resumes a stalled ring as soon as a segment fits again
#include "UARTStreamCursor.hpp"

#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

namespace D = Kvasir::UART::Detail;

namespace {
int failures = 0;

void check(bool          ok,
           char const*   what,
           std::uint32_t step) {
    if(!ok) {
        if(failures < 10) { std::printf("FAIL at step %u: %s\n", step, what); }
        ++failures;
    }
}

struct Sim {
    std::uint32_t              ring;
    std::uint32_t              seg;
    std::vector<std::uint32_t> mem;
    // the channel
    std::uint32_t wpos;   // the next element's global position
    std::uint32_t tc;     // TRANS_COUNT
    bool          active{true};
    bool          pending{};   // completion raised, ISR not yet run
    // the driver
    std::uint32_t segDone;
    bool          stalled{};
    std::uint32_t readTotal;
    std::uint32_t stalls{};

    Sim(std::uint32_t r,
        std::uint32_t s,
        std::uint32_t startSegments)
      : ring{r}
      , seg{s}
      , mem(r,
            0xFFFF'FFFFU)
      , wpos{startSegments * s}
      , tc{s}
      , segDone{startSegments}
      , readTotal{startSegments * s} {}

    void dma(std::uint32_t step) {   // one character arrives and is moved
        if(!active) { return; }
        check(wpos - readTotal < ring, "the channel overwrites an unread element", step);
        mem[wpos & (ring - 1U)] = wpos;
        ++wpos;
        if(--tc == 0) {
            active  = false;
            pending = true;
        }
    }

    void isr(std::uint32_t step) {
        if(!pending) { return; }
        pending = false;
        ++segDone;
        bool const fits = segDone * seg + seg - readTotal <= ring;   // the model's own statement
        check(fits == D::mayRearm(segDone, readTotal, seg, ring),
              "mayRearm disagrees with the model",
              step);
        if(D::mayRearm(segDone, readTotal, seg, ring)) {
            tc     = seg;
            active = true;
        } else {
            stalled = true;
            ++stalls;
        }
    }

    // read what the cursor offers, take `take` of it (0 = a paused reader)
    void read(std::uint32_t take,
              std::uint32_t step) {
        std::uint32_t const w = D::writtenTotal(segDone, stalled, tc, seg);
        check(w - readTotal <= wpos - readTotal, "writtenTotal ahead of the channel", step);
        check(w == wpos,
              "writtenTotal misses written elements",
              step);   // TRANS_COUNT is exact here
        auto const run = D::readable(w, readTotal, ring);
        check(run.at + run.len <= ring, "a run past the ring's end", step);
        check(run.len == 0 || run.len == std::min(w - readTotal, ring - run.at),
              "a run shorter than offered",
              step);
        for(std::uint32_t i = 0; i != run.len; ++i) {
            if(mem[run.at + i] != readTotal + i) {
                check(false, "the reader sees a wrong element", step);
                break;
            }
        }
        take = std::min(take, run.len);
        readTotal += take;
        if(stalled && D::mayRearm(segDone, readTotal, seg, ring)) {
            stalled = false;
            tc      = seg;
            active  = true;
        }
    }
};

void randomRun(std::uint32_t ring,
               std::uint32_t segs,
               std::uint32_t startSegments,
               std::uint32_t seed,
               std::uint32_t steps,
               bool          slowReader) {
    Sim                                s{ring, ring / segs, startSegments};
    std::mt19937                       rng{seed};
    std::uniform_int_distribution<int> op{0, 9};
    std::uint32_t                      readSteps = 0;
    for(std::uint32_t i = 0; i != steps; ++i) {
        int const o = op(rng);
        if(o < 5) {
            s.dma(i);
        } else if(o < 7) {
            s.isr(i);
        } else if(!slowReader) {
            ++readSteps;
            s.read(static_cast<std::uint32_t>(rng() % (ring + 1)), i);
        } else if((rng() % 24) == 0) {   // falls behind: rare reads of less than a segment
            ++readSteps;
            s.read(static_cast<std::uint32_t>(rng() % (s.seg + 1)), i);
        }
        // a stalled ring holds a full ring of unread elements up to the next segment
        if(s.stalled) {
            check(s.segDone * s.seg + s.seg - s.readTotal > s.ring,
                  "stalled although a segment fits",
                  i);
        }
    }
    std::printf(
      "ring %u x %u from segment %u, %s reader: %u steps, %u reads, %u elements, %u stalls\n",
      ring,
      segs,
      startSegments,
      slowReader ? "slow" : "fast",
      steps,
      readSteps,
      s.wpos - startSegments * (ring / segs),
      s.stalls);
}
}   // namespace

int main() {
    static_assert(D::writtenTotal(3, false, 10, 64) == 3 * 64 + 54);
    static_assert(D::writtenTotal(3, false, 0, 64) == 4 * 64,
                  "a pending completion counts its whole segment");
    static_assert(D::writtenTotal(4, true, 0, 64) == 4 * 64, "a stalled ring adds nothing");
    static_assert(D::mayRearm(3, 0, 64, 256) && !D::mayRearm(4, 0, 64, 256));
    static_assert(D::readable(300, 250, 256).at == 250 && D::readable(300, 250, 256).len == 6,
                  "up to the ring's end");
    static_assert(D::mayRearm(0xFFFF'FFFFU / 64U, 0xFFFF'FFFFU / 64U * 64U, 64, 256),
                  "across the 2^32 wrap");

    std::uint32_t const nearWrap64
      = (0x1'0000'0000ULL - 4 * 256) / 64;   // totals wrap a few segments in
    randomRun(256, 4, 0, 1, 2'000'000, false);
    randomRun(256, 4, 0, 2, 2'000'000, true);
    randomRun(256, 4, static_cast<std::uint32_t>(nearWrap64), 3, 2'000'000, true);
    randomRun(64, 2, static_cast<std::uint32_t>((0x1'0000'0000ULL - 64) / 32), 4, 1'000'000, true);
    randomRun(2048, 8, 0, 5, 2'000'000, true);
    if(failures != 0) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::puts("all cursor checks passed");
    return 0;
}
