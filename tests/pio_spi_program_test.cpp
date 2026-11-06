// PioSpiQueued's program and sample-cycle choice (pio/PioSpiProgram.hpp), on the host.
//
// A board's SPI timing claims are only as good as the program's cycle count, so the program is run here, cycle by
// cycle, by a small simulator of the instructions it uses (decoded from the assembled words by the encoding table of
// RP2350 datasheet 11.4: opcode in bits 15:13; with one non-optional side-set bit, bit 12 is SCK and bits 11:8 the
// delay). For frames of several lengths and both sample cycles it checks:
//   - SCK idles high, a frame of N bits has N falling and N rising edges, one bit every 4 cycles, 2 low and 2 high;
//   - MOSI changes on every falling edge (2 cycles of setup before the rising one);
//   - bit k is sampled exactly Lag cycles after its own falling edge, the last bit Lag + 1 cycles after;
//   - the machine is back at `idle` after the frame, SCK high.
// The choice of the sample cycle from a board's round trip is checked against fixed points, refusals included.
#include "pio/PioSpiProgram.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

using Kvasir::SPI::Detail::PioSpiProgram;
using Kvasir::SPI::Detail::pioSpiSampleLag;

// ---- the sample-cycle choice ---------------------------------------------------------------------------------------
// window: [longest round trip + 2 ns, shortest + one bit - 2 ns]; the cycle inside it nearest its middle
static_assert(pioSpiSampleLag(250'000'000,
                              14,
                              18)
                == 6,
              "250 MHz: 24 ns, the middle of [20, 28]");
static_assert(pioSpiSampleLag(150'000'000,
                              14,
                              18)
                == 5,
              "150 MHz: 33.3 ns in [20, 38.7]; 40 ns is past the bit");
static_assert(pioSpiSampleLag(125'000'000,
                              14,
                              18)
                == 5,
              "125 MHz: 40 ns in [20, 44]; 48 ns is past it");
static_assert(pioSpiSampleLag(200'000'000,
                              14,
                              18)
                == 5,
              "200 MHz: 25 ns nearer the middle of [20, 32] than 30");
static_assert(pioSpiSampleLag(250'000'000,
                              4,
                              8)
                == 0,
              "a round trip this short: the window [10, 18] ns ends before cycle 5 (20 ns)");
static_assert(pioSpiSampleLag(250'000'000,
                              20,
                              26)
                == 0,
              "a round trip this long: the window [28, 34] ns starts after cycle 6 (24 ns)");

static_assert(PioSpiProgram<5>::sidesetCount == 1 && !PioSpiProgram<5>::sidesetOptional,
              "one side-set bit, SCK");
static_assert(PioSpiProgram<6>::sidesetCount == 1 && !PioSpiProgram<6>::sidesetOptional,
              "one side-set bit, SCK");

int failures = 0;

void check(bool        ok,
           char const* what,
           unsigned    lag,
           unsigned    bits) {
    if(!ok) {
        std::printf("  [FAIL] lag %u, %u bits: %s\n", lag, bits, what);
        ++failures;
    }
}

struct Trace {
    std::vector<unsigned> falls;
    std::vector<unsigned> rises;
    std::vector<unsigned> mosi;      // cycles of `out pins`
    std::vector<unsigned> samples;   // cycles of `in pins`
    bool                  sckAtEnd{};
    unsigned              pcAtEnd{};
    bool                  finished{};
};

// One frame of `bits` bits: the go already given, the count in the OSR, data always there (no stall).
template<typename P>
Trace run(unsigned bits) {
    Trace         t;
    auto const&   code     = P::instructions;
    unsigned      pc       = P::offset("idle");
    unsigned      cycle    = 0;
    bool          sck      = true;   // idles high (pinsInitialHigh)
    std::uint32_t x        = 0;
    bool          leftIdle = false;
    for(unsigned steps = 0; steps < 100'000; ++steps) {
        if(leftIdle && pc == P::offset("idle")) {
            t.finished = true;
            break;
        }
        auto const w     = code[pc];
        auto const op    = (w >> 13U) & 7U;
        bool const side  = ((w >> 12U) & 1U) != 0;
        auto const delay = (w >> 8U) & 0xFU;
        if(side != sck) { (side ? t.rises : t.falls).push_back(cycle); }
        sck = side;
        unsigned next
          = pc == static_cast<unsigned>(P::wrap) ? static_cast<unsigned>(P::wrapTarget) : pc + 1U;
        auto const a
          = (w >> 5U) & 7U;   // JMP condition, IN source, OUT destination, MOV destination
        auto const idx = w & 0x1FU;
        switch(op) {
        case 0:   // JMP
            if(a == 0) {
                next = idx;
            } else if(a == 2) {   // X--: taken when X was not zero, X decremented either way
                if(x != 0) { next = idx; }
                --x;
            }
            break;
        case 1:   // WAIT: the go is there
            leftIdle = true;
            break;
        case 2:   // IN
            if(a == 0) { t.samples.push_back(cycle); }
            break;
        case 3:   // OUT
            if(a == 0) { t.mosi.push_back(cycle); }
            if(a == 1) { x = bits - 1U; }   // the bit count, autopulled into the OSR before the go
            break;
        default: break;   // MOV y, y (nop)
        }
        pc = next;
        cycle += 1U + delay;
    }
    t.sckAtEnd = sck;
    t.pcAtEnd  = pc;
    return t;
}

template<unsigned Lag>
void testFrames() {
    using P = PioSpiProgram<Lag>;
    static_assert(P::sampleCycles == Lag && P::lastSampleCycles == Lag + 1);
    for(unsigned const bits : {8U, 16U, 24U, 32U, 8U * 1514U}) {
        auto const t = run<P>(bits);
        check(t.finished, "back at idle after the frame", Lag, bits);
        check(t.sckAtEnd, "SCK high at the end", Lag, bits);
        check(t.falls.size() == bits && t.rises.size() == bits,
              "one falling and one rising edge a bit",
              Lag,
              bits);
        check(t.mosi.size() == bits && t.samples.size() == bits,
              "one MOSI change and one sample a bit",
              Lag,
              bits);
        if(t.falls.size() != bits || t.rises.size() != bits || t.mosi.size() != bits
           || t.samples.size() != bits)
        {
            continue;
        }
        bool period      = true;
        bool duty        = true;
        bool mosiOnFall  = true;
        bool samplePoint = true;
        for(unsigned k = 0; k < bits; ++k) {
            if(k != 0 && t.falls[k] - t.falls[k - 1] != P::cyclesPerBit) { period = false; }
            if(t.rises[k] != t.falls[k] + 2U) { duty = false; }
            if(t.mosi[k] != t.falls[k]) { mosiOnFall = false; }
            unsigned const want = t.falls[k] + (k + 1U == bits ? Lag + 1U : Lag);
            if(t.samples[k] != want) { samplePoint = false; }
        }
        check(period, "a bit every 4 cycles", Lag, bits);
        check(duty, "2 cycles low, 2 high", Lag, bits);
        check(mosiOnFall, "MOSI changes on the falling edge", Lag, bits);
        check(samplePoint,
              "bit k sampled Lag cycles after its falling edge, the last Lag + 1",
              Lag,
              bits);
    }
}

}   // namespace

int main() {
    testFrames<5>();
    testFrames<6>();
    if(failures != 0) {
        std::printf("pio_spi_program_test: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("pio_spi_program_test: ok\n");
    return 0;
}
