// The RP I2C SCL counts (I2CMath.hpp) against the datasheets (RP2350 12.2.14.1, md l.49439-49451 and Figure 90 l.49482;
// RP2040 4.3.14.1, md l.21550-21562): HCNT > SPKLEN + 5, LCNT > SPKLEN + 7, 16-bit counts, a period of
// (HCNT + SPKLEN + 7) + (LCNT + 1) ic_clk cycles; SPKLEN the 50 ns spike length rounded up (12.2.11, md l.49326). Values
// at the clocks the firmwares use, every rate a grid of clocks allows, the per-device clockTiming(); the refusals are
// bad/i2c_*.cpp. Building is the test (static_assert).
#include "I2CMath.hpp"

#include <cstdint>
#include <ratio>

namespace {
using M = Kvasir::I2C::Detail::I2CMath;

// 150 MHz, 400 kHz: 375 cycles, 60 % low (225 = LCNT + 1), 40 % high (150 = HCNT + SPKLEN + 7), SPKLEN ceil(7.5) = 8,
// SDA hold 300 ns = 46 cycles (45 + 1)
constexpr auto fm150 = M::calcBaudRegs(150'000'000, 400'000);
static_assert(fm150.spklen == 8 && fm150.hcnt == 135 && fm150.lcnt == 224 && fm150.sdaHold == 46);
// 125 MHz, 100 kHz and 1 MHz (fast mode plus: SDA hold 120 ns)
constexpr auto sm125 = M::calcBaudRegs(125'000'000, 100'000);
static_assert(sm125.spklen == 7 && sm125.hcnt == 486 && sm125.lcnt == 749 && sm125.sdaHold == 38);
constexpr auto fp125 = M::calcBaudRegs(125'000'000, 1'000'000);
static_assert(fp125.spklen == 7 && fp125.hcnt == 36 && fp125.lcnt == 74 && fp125.sdaHold == 16);

// the period the counts give is the one asked for (to the rounding of calcPeriod)
constexpr std::uint64_t periodOf(M::BaudRegs const& r) {
    return std::uint64_t{r.hcnt} + r.spklen + 7U + r.lcnt + 1U;
}

static_assert(periodOf(fm150) == 375 && periodOf(sm125) == 1250 && periodOf(fp125) == 125);
static_assert(M::achievedRate(150'000'000,
                              fm150)
                  .num
                == 150'000'000
              && M::achievedRate(150'000'000,
                                 fm150)
                     .den
                   == 375);

// SPKLEN: the smallest count of ic_clk cycles that covers 50 ns, at least 1
constexpr bool spikeCovered(std::uint32_t clk) {
    auto const n = M::calcSpkLen(clk);
    return n >= 1 && std::uint64_t{n} * 1'000'000'000 >= std::uint64_t{clk} * 50
        && (n == 1 || std::uint64_t{n - 1} * 1'000'000'000 < std::uint64_t{clk} * 50);
}

static_assert(spikeCovered(12'000'000) && spikeCovered(48'000'000) && spikeCovered(125'000'000)
              && spikeCovered(150'000'000) && spikeCovered(200'000'000)
              && spikeCovered(266'000'000));
static_assert(M::calcSpkLen(12'000'000) == 1);   // 0.6 cycles: one

// the datasheet minimums, as the driver checks them, on a grid of clocks and rates
template<std::uint32_t Clk,
         std::uint32_t Baud>
constexpr bool valid() {
    constexpr auto r = M::calcBaudRegs(Clk, Baud);
    return M::isValidHcnt<Clk, Baud>() == (r.hcnt <= 0xFFFF && r.hcnt > r.spklen + 5)
        && M::isValidLcnt<Clk, Baud>() == (r.lcnt <= 0xFFFF && r.lcnt > r.spklen + 7)
        && M::isValidHcnt<Clk, Baud>() && M::isValidLcnt<Clk, Baud>();
}

static_assert(valid<125'000'000,
                    100'000>()
              && valid<125'000'000,
                       400'000>()
              && valid<125'000'000,
                       1'000'000>());
static_assert(valid<150'000'000,
                    100'000>()
              && valid<150'000'000,
                       400'000>()
              && valid<150'000'000,
                       1'000'000>());
static_assert(valid<48'000'000,
                    400'000>());
// 12 MHz cannot make 1 MHz: 5 high cycles leave HCNT below SPKLEN + 7 (the subtraction wraps: refused)
static_assert(!M::isValidHcnt<12'000'000,
                              1'000'000>());
// 125 MHz at 1 kHz: LCNT 74999 does not fit the 16-bit register
static_assert(!M::isValidLcnt<125'000'000,
                              1'000>());

// the per-device timing: the same counts, and 9 bits four times over per byte
constexpr auto timing = M::clockTiming(150'000'000, 400'000, std::ratio<1, 100>{});
static_assert(timing.hcnt == fm150.hcnt && timing.lcnt == fm150.lcnt
              && timing.spklen == fm150.spklen && timing.sdaHold == fm150.sdaHold
              && timing.usPerByte == 90);
static_assert(M::usPerDataByte(100'000) == 360 && M::usPerDataByte(1'000'000) == 36);
}   // namespace

int main() { return 0; }
