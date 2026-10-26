// expect: pin_refused_i2c\.cpp:[0-9]+:.*GPIO 30 is not I2C1 SCL on the RP2350A \(I2C1 SCL: GPIO 3 7 11 15 19 23 27\)
#include "PinConfig.hpp"

// past the QFN-60 package's 30 GPIOs: no role at all
static_assert(Kvasir::PinConfig::CheckedPin<Kvasir::Chip::Part::rp2350a,
                                            Kvasir::PinConfig::Function::i2cScl,
                                            1,
                                            30>::value);
