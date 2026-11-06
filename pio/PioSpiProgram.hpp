#pragma once
// The program of chip_rp_common PioSpiQueued.hpp and the choice of its sample cycle, apart from the driver so the host
// tests (tests/pio_spi_program_test.cpp) build them without a chip: SPI mode 3 (SCK idles high; out on the falling
// edge, latched on the rising one), MSB first, four machine cycles a bit, each bit sampled during the next one.
#include "Asm.hpp"

#include <cstdint>

namespace Kvasir { namespace SPI { namespace Detail {

    /// SPI mode 3, MSB first, four cycles a bit. X counts the bits; bit 0 has no sample, bit k samples bit k - 1
    /// `Lag` cycles after that bit's falling edge (5: the cycle after bit k's falling edge, 6: bit k's rising
    /// edge), the tail the last bit at Lag + 1.
    template<unsigned Lag>
    struct PioSpiProgram : Kvasir::Pio::Program<Kvasir::Pio::assemble([](Kvasir::Pio::Asm& a) {
        using namespace Kvasir::Pio;
        a.sideSet(1);   // SCK
        a.wrapTarget();
        a.label("idle");
        // The OSR is empty here (the last frame shifted out, or a restart), so autopull takes the bit count as
        // soon as start() pushes it - an explicit `pull` would be a no-op on the full OSR (with autopull on).
        a.wait(1, Wait::irq, 0, IrqMode::rel).side(1);   // start()'s go
        a.out(Out::x, 32).side(1);   // X = the bit count - 1; the next `out` autopulls a frame
        a.out(Out::pins, 1).side(0).delay(1);   // bit 0: falling edge, MOSI
        a.jmp(Cond::xDec, "loop")
          .side(1)
          .delay(1);   // rising edge (a transfer is >= 8 bits: taken)
        a.label("loop");
        if constexpr(Lag == 6) {
            a.out(Out::pins, 1).side(0).delay(1);   // bit k: falling edge, MOSI
            a.in(In::pins, 1).side(1);              // rising edge; MISO of bit k - 1
        } else {
            a.out(Out::pins, 1).side(0);   // bit k: falling edge, MOSI
            a.in(In::pins, 1).side(0);     // MISO of bit k - 1
            a.nop().side(1);               // rising edge
        }
        a.jmp(Cond::xDec, "loop").side(1);
        a.nop().side(1).delay(Lag - 4);   // after the last bit SCK stays high
        a.in(In::pins, 1).side(1);   // MISO of the last bit, Lag + 1 cycles after its falling edge
        a.wrap();
    })> {
        static_assert(Lag == 5 || Lag == 6,
                      "the loop samples 5 or 6 cycles after a bit's falling edge");
        static constexpr unsigned cyclesPerBit     = 4;
        static constexpr unsigned sampleCycles     = Lag;
        static constexpr unsigned lastSampleCycles = Lag + 1;
    };

    /// The sample cycle for a board (5 or 6 after a bit's falling edge, at `clockHz`) inside the window its
    /// round trip leaves - from the longest round trip to the next falling edge plus the shortest, 2 ns kept free
    /// each side - nearest the window's middle; 0 when neither is inside.
    consteval unsigned pioSpiSampleLag(std::uint64_t clockHz,
                                       std::uint64_t roundTripMinNs,
                                       std::uint64_t roundTripMaxNs) {
        std::uint64_t const psCycle  = 1'000'000'000'000ULL / clockHz;
        std::uint64_t const psMargin = 2'000;
        std::uint64_t const lo       = roundTripMaxNs * 1000U + psMargin;
        std::uint64_t const hiPlus   = roundTripMinNs * 1000U + 4U * psCycle;   // + one bit
        if(hiPlus < psMargin) { return 0; }
        std::uint64_t const hi       = hiPlus - psMargin;
        unsigned            best     = 0;
        std::uint64_t       bestDist = ~std::uint64_t{0};
        for(unsigned lag = 5; lag <= 6; ++lag) {
            std::uint64_t const t = lag * psCycle;
            if(t < lo || t > hi) { continue; }
            std::uint64_t const twice = 2U * t;
            std::uint64_t const dist  = twice > lo + hi ? twice - (lo + hi) : (lo + hi) - twice;
            if(dist < bestDist) {
                best     = lag;
                bestDist = dist;
            }
        }
        return best;
    }

}}}   // namespace Kvasir::SPI::Detail
