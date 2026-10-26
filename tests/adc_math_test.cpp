// The ADC pacer divider (AdcMath.hpp): DIV = 0 for one conversion's time (96 clocks, RP2040 datasheet 4.9.2, RP2350
// 12.4.3) or less, never a paced period of 96 (the pacer would skip every other conversion, measured), 97 clocks the
// shortest paced one, and the usual 1 + INT + FRAC / 256 encoding above (DIV register, RP2040 datasheet 4.9.6, RP2350
// 12.4.7). Checked in constant evaluation and at run time through a volatile. And the channel-attribution check both
// free-running modes share (checkAttribution): scripted snapshots, then a model ADC (round robin into an 8-entry FIFO)
// read in batches, with a sample stolen and an overrun forced.
#include "AdcMath.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace D = Kvasir::ADC::Detail;

namespace {
struct Case {
    std::uint32_t rate;
    std::uint16_t int_;
    std::uint8_t  frac;
};

constexpr std::uint32_t clk = 48'000'000;

constexpr Case cases[] = {
  {500'000,      0,    0}, // 96 clocks: back to back
  {600'000,      0,    0}, // faster than the ADC: the fastest it has
  {498'500,      0,    0}, // 96.29 clocks: nearer 96, back to back
  {497'000,     96,    0}, // 96.58 clocks: nearer 97
  {495'000,     96,    0}, // 96.97 clocks: 97, the shortest paced period
  {494'845,     96,    0}, // 97.0 clocks
  {480'000,     99,    0}, // 100 clocks
  {100'000,    479,    0}, // 480 clocks: 24's rate
  { 10'000,   4799,    0}, // 4800 clocks
  {  1'000,  47999,    0}, // the datasheet's example: DIV.INT 47999 runs at 1 kS/s
  {      3, 0xFFFF, 0xFF}, // slower than the divider reaches: clamped
};

constexpr bool matches(Case const&   c,
                       std::uint32_t rate) {
    auto const d = D::calcDivRegs(clk, rate);
    return d.int_ == c.int_ && d.frac == c.frac;
}

constexpr bool allMatch() {
    for(auto const& c : cases) {
        if(!matches(c, c.rate)) { return false; }
    }
    return true;
}

static_assert(allMatch());
static_assert(D::calcActualSampleRate(clk,
                                      D::DivRegs{0,
                                                 0})
              == 500'000.0);
static_assert(D::calcActualSampleRate(clk,
                                      D::DivRegs{96,
                                                 0})
              == 48'000'000.0 / 97.0);
static_assert(D::isValidSampleRateConfig<clk,
                                         500'000>(std::ratio<0,
                                                             1>{}));
static_assert(D::isValidSampleRateConfig<clk,
                                         495'000>(std::ratio<5,
                                                             1000>{}));
static_assert(!D::isValidSampleRateConfig<clk,
                                          498'000>(std::ratio<1,
                                                              1000>{}));

// ---- checkAttribution (the ISR and DMA modes' shared check) --------------------------------

constexpr auto table2 = D::buildChannelTable<2>(0b1'0001);   // A0 (0) and the sensor (4)
constexpr auto table3 = D::buildChannelTable<3>(0b1'0110);   // 1, 2, 4
static_assert(table2.channels[0] == 0 && table2.channels[1] == 4);
static_assert(table3.at(0,
                        0)
                == 1
              && table3.at(0,
                           1)
                   == 2
              && table3.at(0,
                           2)
                   == 4
              && table3.at(2,
                           1)
                   == 1);

// Snapshots played back one per call; past the end the last one repeats.
template<std::size_t K>
struct Script {
    D::AttributionSnapshot snaps[K];
    std::size_t            next  = 0;
    int                    calls = 0;

    constexpr D::AttributionSnapshot operator()() {
        ++calls;
        auto const s = snaps[next];
        if(next + 1 < K) { ++next; }
        return s;
    }
};

constexpr D::AttributionSnapshot snap(std::size_t   moved,
                                      std::uint32_t ainsel,
                                      bool          stable  = true,
                                      bool          flagged = false) {
    return {flagged, stable, moved, ainsel};
}

template<std::size_t N,
         std::size_t K>
constexpr D::Alignment run(D::ChannelTable<N> const& t,
                           std::size_t               slot,
                           Script<K>                 s,
                           int*                      calls = nullptr) {
    auto const r = D::checkAttribution(t, slot, s);
    if(calls != nullptr) { *calls = s.calls; }
    return r;
}

// Batch of 3 from slot 0 on channels 0,4: 3 moved + 1 in the FIFO = 4 -> slot 0 -> channel 0.
static_assert(run(table2,
                  0,
                  Script<1>{{snap(4,
                                  0)}})
              == D::Alignment::aligned);
static_assert(run(table2,
                  1,
                  Script<1>{{snap(4,
                                  4)}})
              == D::Alignment::aligned);
static_assert(run(table2,
                  0,
                  Script<1>{{snap(3,
                                  4)}})
              == D::Alignment::aligned);
// one mismatch, then a match: a completion seen half-way, the samples are good
static_assert(run(table2,
                  0,
                  Script<2>{
                    {snap(3,
                     0),
                     snap(3,
                     4)}
})
              == D::Alignment::aligned);
// two mismatches: confirmed
static_assert(run(table2,
                  0,
                  Script<1>{{snap(3,
                                  0)}})
              == D::Alignment::misaligned);
// a flag ends it at once, unstable or not
static_assert(run(table2,
                  0,
                  Script<1>{{snap(4,
                                  0,
                                  false,
                                  true)}})
              == D::Alignment::overrun);
// unstable snapshots are not judged; never stable -> misaligned after the attempts
static_assert(run(table2,
                  0,
                  Script<2>{
                    {snap(9,
                     9, false),
                     snap(4,
                     0)}
})
              == D::Alignment::aligned);
static_assert(run(table2,
                  0,
                  Script<1>{{snap(4,
                                  0,
                                  false)}})
              == D::Alignment::misaligned);
// one channel: no round robin, only the flags count
constexpr auto table1 = D::buildChannelTable<1>(0b100);
static_assert(run(table1,
                  0,
                  Script<1>{{snap(5,
                                  7)}})
              == D::Alignment::aligned);
static_assert(run(table1,
                  0,
                  Script<1>{{snap(5,
                                  2,
                                  true,
                                  true)}})
              == D::Alignment::overrun);
// three channels: a sample lost shifts by one, a sample gained by one the other way
static_assert(run(table3,
                  0,
                  Script<1>{{snap(5,
                                  table3.at(0,
                                            5))}})
              == D::Alignment::aligned);
static_assert(run(table3,
                  0,
                  Script<1>{{snap(5,
                                  table3.at(0,
                                            6))}})
              == D::Alignment::misaligned);
static_assert(run(table3,
                  0,
                  Script<1>{{snap(5,
                                  table3.at(0,
                                            4))}})
              == D::Alignment::misaligned);

// ---- the table lookups the interrupt uses instead of `%` ------------------------------------

template<std::size_t N>
constexpr bool lookupsAgree(std::uint32_t mask) {
    auto const t = D::buildChannelTable<N>(mask);
    for(std::size_t first = 0; first < N; ++first) {
        for(std::size_t i = 0; i <= D::nearReach; ++i) {
            if(t.near(first, i) != t.at(first, i)) { return false; }
            if(t.slotAfter(first, i) != (first + i) % N) { return false; }
        }
        if(D::ChannelTable<N>::template advance<333>(first) != (first + 333) % N) { return false; }
        if(D::ChannelTable<N>::template advance<1>(first) != (first + 1) % N) { return false; }
    }
    return true;
}

static_assert(lookupsAgree<1>(0b100));
static_assert(lookupsAgree<2>(0b1'0001));
static_assert(lookupsAgree<3>(0b1'0110));
static_assert(lookupsAgree<9>(0b1'1111'1111));
// a batch plus the largest LEVEL the field can read
static_assert(D::nearReach == D::fifoDepth + D::levelFieldMax && D::fifoDepth == 8
              && D::levelFieldMax == 15);

// ---- checkAttributionAfter(judge(first snapshot)) is checkAttribution -------------------------
// The check as it was before the interrupt took its first snapshot inline (one loop over
// snap(), verbatim), and every script of up to 7 snapshots of the four kinds (flagged, unstable,
// a match, a mismatch), the last one repeating: same verdict, same number of snapshots taken.

template<int         Attempts            = 16,
         int         MismatchesToConfirm = 2,
         std::size_t N,
         typename Snap>
constexpr D::Alignment referenceCheck(D::ChannelTable<N> const& table,
                                      std::size_t               firstSlot,
                                      Snap&&                    snap) {
    int mismatches = 0;
    for(int attempt = 0; attempt < Attempts; ++attempt) {
        D::AttributionSnapshot const s = snap();
        if(s.flagged) { return D::Alignment::overrun; }
        if constexpr(N == 1) {
            return D::Alignment::aligned;
        } else {
            if(!s.stable) { continue; }
            if(s.ainsel == table.at(firstSlot, s.moved)) { return D::Alignment::aligned; }
            if(++mismatches == MismatchesToConfirm) { return D::Alignment::misaligned; }
        }
    }
    return D::Alignment::misaligned;
}

template<std::size_t N>
constexpr D::AttributionSnapshot ofKind(D::ChannelTable<N> const& t,
                                        int                       kind,
                                        std::size_t               moved) {
    switch(kind) {
    case 0:  return snap(moved, t.at(0, moved), true, true);   // flagged
    case 1:  return snap(moved, t.at(0, moved + 1), false);    // unstable
    case 2:  return snap(moved, t.at(0, moved));               // AINSEL as counted
    default: return snap(moved, t.at(0, moved + 1));           // one off
    }
}

template<std::size_t N,
         int         Attempts,
         int         Confirm>
bool afterMatchesReference(D::ChannelTable<N> const& t) {
    constexpr int maxLen = 7;
    for(int len = 1; len <= maxLen; ++len) {
        int combos = 1;
        for(int i = 0; i < len; ++i) { combos *= 4; }
        for(int c = 0; c < combos; ++c) {
            Script<maxLen> sc{};
            int            code = c;
            for(int i = 0; i < len; ++i) {
                sc.snaps[i] = ofKind(t, code % 4, 3 + static_cast<std::size_t>(i));
                code /= 4;
            }
            for(int i = len; i < maxLen; ++i) { sc.snaps[i] = sc.snaps[len - 1]; }
            Script<maxLen> a    = sc;
            Script<maxLen> b    = sc;
            auto const     want = referenceCheck<Attempts, Confirm>(t, 0, a);
            auto const     s0   = b();
            auto const     got
              = D::checkAttributionAfter<Attempts, Confirm>(t,
                                                            0,
                                                            D::judge<N>(s0, t.near(0, s0.moved)),
                                                            b);
            auto const plain = D::checkAttribution<Attempts, Confirm>(t, 0, sc);
            if(want != got || a.calls != b.calls || plain != want) {
                std::printf("FAIL after/reference N=%u len %d code %d: %d vs %d (%d vs %d calls)\n",
                            unsigned{N},
                            len,
                            c,
                            int(got),
                            int(want),
                            b.calls,
                            a.calls);
                return false;
            }
        }
    }
    return true;
}

// A model stream: an ADC converting round robin into an 8-entry FIFO, a reader that drains
// batches and checks each one; `steal` takes one sample behind the reader's back once.
struct Model {
    D::ChannelTable<3> t = table3;
    std::uint32_t      ainsel{};   // channel of the next conversion
    std::size_t        hwSlot{};   // slot of that conversion
    std::uint8_t       fifo[8]{};
    std::size_t        level{};
    bool               over{};

    void convert() {
        if(level == 8) {
            over = true;
        } else {
            fifo[level++] = t.channels[hwSlot];
        }
        hwSlot = (hwSlot + 1) % 3;
        ainsel = t.channels[hwSlot];
    }

    std::uint8_t pop() {
        auto const v = fifo[0];
        for(std::size_t i = 1; i < level; ++i) { fifo[i - 1] = fifo[i]; }
        --level;
        return v;
    }
};

/// Runs `batches` drains of up to 4 samples with 1..4 conversions between them; returns how
/// many delivered samples were on a wrong channel and what the checks said.
struct ModelResult {
    int wrong, aligned, misaligned, overruns;
};

ModelResult runModel(int batches,
                     int stealAt,
                     int overrunAt) {
    Model         m{};
    std::size_t   slot = 0;
    ModelResult   r{};
    std::uint32_t seed = 12345;
    for(int b = 0; b < batches; ++b) {
        seed           = seed * 1103515245U + 12345U;
        int const conv = b == overrunAt ? 12 : 1 + static_cast<int>((seed >> 16) % 4);
        for(int i = 0; i < conv; ++i) { m.convert(); }
        if(b == stealAt && m.level > 0) { m.pop(); }
        std::uint8_t held[4]{};
        std::size_t  n = 0;
        while(n < 4 && m.level > 0) { held[n++] = m.pop(); }
        auto const check = D::checkAttribution(m.t, slot, [&] {
            return D::AttributionSnapshot{m.over, true, n + m.level, m.ainsel};
        });
        if(check == D::Alignment::aligned) {
            ++r.aligned;
            for(std::size_t i = 0; i < n; ++i) {
                if(held[i] != m.t.at(slot, i)) { ++r.wrong; }
            }
            slot = (slot + n) % 3;
        } else {
            ++(check == D::Alignment::overrun ? r.overruns : r.misaligned);
            // realign: drain, clear OVER, AINSEL to the first channel
            m.level  = 0;
            m.over   = false;
            m.hwSlot = 0;
            m.ainsel = m.t.channels[0];
            slot     = 0;
        }
    }
    return r;
}
}   // namespace

int main() {
    int failures = 0;
    for(auto const& c : cases) {
        std::uint32_t volatile rate = c.rate;
        if(!matches(c, rate)) {
            auto const d = D::calcDivRegs(clk, rate);
            std::printf("FAIL %u S/s: INT %u FRAC %u, want %u %u\n",
                        unsigned{c.rate},
                        unsigned{d.int_},
                        unsigned{d.frac},
                        unsigned{c.int_},
                        unsigned{c.frac});
            ++failures;
        }
    }

    struct ModelCase {
        char const* what;
        int         steal, overrun, wantMisaligned, wantOverruns;
    };

    constexpr ModelCase models[] = {
      {  "clean", -1, -1, 0, 0},
      { "stolen", 40, -1, 1, 0},
      {"overrun", -1, 60, 0, 1},
    };
    for(auto const& c : models) {
        auto const r = runModel(400, c.steal, c.overrun);
        if(r.wrong != 0 || r.misaligned != c.wantMisaligned || r.overruns != c.wantOverruns
           || r.aligned < 390)
        {
            std::printf("FAIL model %s: wrong %d aligned %d misaligned %d overruns %d\n",
                        c.what,
                        r.wrong,
                        r.aligned,
                        r.misaligned,
                        r.overruns);
            ++failures;
        }
    }
    if(!afterMatchesReference<2, 16, 2>(table2)) { ++failures; }
    if(!afterMatchesReference<3, 16, 2>(table3)) { ++failures; }
    if(!afterMatchesReference<1, 16, 2>(table1)) { ++failures; }
    if(!afterMatchesReference<3, 3, 1>(table3)) { ++failures; }   // short attempts, no confirmation
    if(!afterMatchesReference<3, 4, 3>(table3)) { ++failures; }   // confirmation past the attempts
    std::printf("%s\n", failures == 0 ? "adc_math_test: ok" : "adc_math_test: FAILED");
    return failures == 0 ? 0 : 1;
}
