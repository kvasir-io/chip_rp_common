// expect: i2cDeviceClockGivesAnInvalidHcnt
// 12 MHz cannot make 1 MHz SCL: HCNT below SPKLEN + 5 (RP2350 12.2.14.1)
#include "I2CMath.hpp"

constexpr auto t
  = Kvasir::I2C::Detail::I2CMath::clockTiming(12'000'000, 1'000'000, std::ratio<1, 100>{});
