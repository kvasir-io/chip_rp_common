// expect: pin_refused_spi\.cpp:[0-9]+:.*GPIO 34 is not SPI1 SCK on the RP2350B \(SPI1 SCK: GPIO 10 14 26 30 42 46\)
#include "PinConfig.hpp"

// SPI1's SCK on GPIO 34 (that is SPI0's SCK, Table 645 F1)
static_assert(Kvasir::PinConfig::CheckedPin<Kvasir::Chip::Part::rp2350b,
                                            Kvasir::PinConfig::Function::spiSck,
                                            1,
                                            34>::value);
