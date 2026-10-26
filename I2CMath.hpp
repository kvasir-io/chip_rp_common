#pragma once
// The RP I2C block's SCL counts (I2C.hpp's Config derives from it): register-free - the standard library and the SDK's
// Prescaler only - so the same functions the driver writes its IC_FS_* registers from run in a host test
// (tests/i2c_math_test.cpp). The rules (RP2350 datasheet 12.2.14.1, md l.49439-49451 and Figure 90 l.49482; RP2040
// datasheet 4.3.14.1, md l.21550-21562 and Figure 86 l.21593): IC_FS_SCL_HCNT must be larger than IC_FS_SPKLEN + 5, LCNT
// larger than SPKLEN + 7, the high period is HCNT + SPKLEN + 7 ic_clk cycles and the low period LCNT + 1; SPKLEN filters
// the I2C specification's 50 ns spikes (RP2350 12.2.11, md l.49326).
#include "kvasir/Util/Prescaler.hpp"

#include <cstdint>
#include <ratio>

namespace Kvasir { namespace I2C { namespace Detail {
    struct I2CMath {
        // I2C baud rate calculation helpers
        static constexpr std::uint32_t calcPeriod(std::uint32_t fClockSpeed,
                                                  std::uint32_t fBaud) {
            return (fClockSpeed + fBaud / 2) / fBaud;
        }

        // The I2C spec's 50 ns spike filter in ic_clk cycles, rounded up, at least 1 (RP2350
        // datasheet 12.2.11, RP2040 datasheet 4.3.11). pico-sdk's lcnt / 16 filters far more.
        static constexpr std::uint32_t calcSpkLen(std::uint32_t fClockSpeed) {
            auto const cycles = static_cast<std::uint32_t>(
              (std::uint64_t{fClockSpeed} * 50 + 999'999'999) / 1'000'000'000);
            return cycles == 0 ? 1U : cycles;
        }

        static constexpr std::uint32_t calcSdaTxHold(std::uint32_t fClockSpeed,
                                                     std::uint32_t fBaud) {
            // Per I2C spec: 300ns hold time for <1MHz, 120ns for >=1MHz
            if(fBaud < 1000000) {
                // 300ns: freq * 3 / 10000000 + 1
                return ((fClockSpeed * 3) / 10000000) + 1;
            }
            // 120ns: freq * 3 / 25000000 + 1
            return ((fClockSpeed * 3) / 25000000) + 1;
        }

        struct BaudRegs {
            std::uint32_t hcnt;
            std::uint32_t lcnt;
            std::uint32_t spklen;
            std::uint32_t sdaHold;
        };

        static constexpr BaudRegs calcBaudRegs(std::uint32_t fClockSpeed,
                                               std::uint32_t fBaud) {
            BaudRegs regs{};

            // Calculate period in ic_clk cycles
            std::uint32_t period = calcPeriod(fClockSpeed, fBaud);

            // Split period: 60% low, 40% high (per pico-sdk)
            std::uint32_t highCycles = period - (period * 3 / 5);
            std::uint32_t lowCycles  = period * 3 / 5;

            regs.spklen = calcSpkLen(fClockSpeed);

            // Program values per RP2350 datasheet Table 1053 / Section 12.2.14:
            //   actual t_HIGH = (HCNT + SPKLEN + 7) cycles → write HCNT = high_cycles − SPKLEN − 7
            //   actual t_LOW  = (LCNT + 1) cycles          → write LCNT = low_cycles − 1
            regs.hcnt = highCycles - regs.spklen - 7;
            regs.lcnt = lowCycles - 1;

            // Calculate SDA hold time
            regs.sdaHold = calcSdaTxHold(fClockSpeed, fBaud);

            return regs;
        }

        template<std::uint32_t FClockSpeed,
                 std::uint32_t FBaud>
        static constexpr bool isValidHcnt() {
            constexpr auto regs = calcBaudRegs(FClockSpeed, FBaud);
            return regs.hcnt <= 0xFFFF   // IC_FS_SCL_HCNT is 16-bit
                && regs.hcnt > regs.spklen + 5;
        }

        template<std::uint32_t FClockSpeed,
                 std::uint32_t FBaud>
        static constexpr bool isValidLcnt() {
            constexpr auto regs = calcBaudRegs(FClockSpeed, FBaud);
            return regs.lcnt <= 0xFFFF   // IC_FS_SCL_LCNT is 16-bit
                && regs.lcnt > regs.spklen + 7;
        }

        /// The SCL rate the counts give, rise and fall times left out: a period of
        /// (HCNT + SPKLEN + 7) + (LCNT + 1) ic_clk cycles (RP2350 datasheet 12.2.14.1,
        /// Figure 90; RP2040 datasheet 4.3.14.1, Figure 86).
        static constexpr Prescaler::Rational achievedRate(std::uint32_t   fClockSpeed,
                                                          BaudRegs const& regs) {
            return {fClockSpeed, std::uint64_t{regs.hcnt} + regs.spklen + 7U + regs.lcnt + 1U};
        }

        /// One device's SCL counts on a bus that runs each device at its own clock
        /// (I2CConfig::perDeviceClock): what GetBaudConfig writes, as values a request
        /// carries, plus its microseconds per byte for the transfer timeout.
        struct ClockTiming {
            std::uint16_t hcnt{};
            std::uint16_t lcnt{};
            std::uint16_t sdaHold{};
            std::uint16_t usPerByte{};
            std::uint8_t  spklen{};

            constexpr bool operator==(ClockTiming const&) const = default;
        };

        /// The same numbers and the same checks as GetBaudConfig and I2CBase's
        /// static_asserts, for a rate known only per device. A failed check is a call to
        /// one of the functions below it, which are not constexpr: the compile error
        /// names what is wrong.
        template<std::intmax_t Num,
                 std::intmax_t Denom>
        static consteval ClockTiming clockTiming(std::uint32_t fClockSpeed,
                                                 std::uint32_t fBaud,
                                                 std::ratio<Num,
                                                            Denom>) {
            if(fBaud == 0 || fBaud > 1'000'000) { i2cDeviceClockAboveFastModePlus(); }
            auto const regs = calcBaudRegs(fClockSpeed, fBaud);
            if(regs.hcnt > 0xFFFF || regs.hcnt <= regs.spklen + 5) {
                i2cDeviceClockGivesAnInvalidHcnt();
            }
            if(regs.lcnt > 0xFFFF || regs.lcnt <= regs.spklen + 7) {
                i2cDeviceClockGivesAnInvalidLcnt();
            }
            // out of tolerance: the note "in call to 'rateOutOfTolerance(...)'" has the numbers
            Prescaler::requireInTolerance(achievedRate(fClockSpeed, regs),
                                          fBaud,
                                          Prescaler::Tolerance{std::ratio<Num, Denom>{}});
            return ClockTiming{.hcnt      = static_cast<std::uint16_t>(regs.hcnt),
                               .lcnt      = static_cast<std::uint16_t>(regs.lcnt),
                               .sdaHold   = static_cast<std::uint16_t>(regs.sdaHold),
                               .usPerByte = static_cast<std::uint16_t>(usPerDataByte(fBaud)),
                               .spklen    = static_cast<std::uint8_t>(regs.spklen)};
        }

        // Not constexpr: reaching one in clockTiming() is the compile error that says why.
        static void i2cDeviceClockAboveFastModePlus() {}

        static void i2cDeviceClockGivesAnInvalidHcnt() {}

        static void i2cDeviceClockGivesAnInvalidLcnt() {}

        /// The transfer timeout's time per byte: 9 bits, 4 times over.
        static constexpr std::uint32_t usPerDataByte(std::uint32_t fBaud) {
            constexpr std::uint32_t bitsPerDataByte = 9;
            constexpr std::uint32_t safetyFactor    = 4;
            return (bitsPerDataByte * 1'000'000 * safetyFactor) / fBaud;
        }
    };
}}}   // namespace Kvasir::I2C::Detail
