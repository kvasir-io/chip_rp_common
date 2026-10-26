// expect: pin_refused_uart\.cpp:[0-9]+:.*GPIO 5 is not UART0 TX on the RP2040 \(UART0 TX: GPIO 0 12 16 28\)
#include "PinConfig.hpp"

// A UART driver's txPinLocation on a pin without that role: the driver's Io::pinServes asks CheckedPin, whose message
// names the pin, the role, the instance and the pins that have it.
static_assert(Kvasir::PinConfig::CheckedPin<Kvasir::Chip::Part::rp2040,
                                            Kvasir::PinConfig::Function::uartTx,
                                            0,
                                            5>::value);
