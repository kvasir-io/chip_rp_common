// expect: pwm_unreachable\.cpp:[0-9]+:.*PWM frequency unreachable
#include "PwmMath.hpp"

// The driver's static_assert on a frequency the search refuses (130 kHz needs a TOP below 1024 at 133 MHz) fails with
// its own message: the search no longer logs, so the refusal path is a constant expression.
static_assert(Kvasir::PWM::detail::calcDivAndTop<133'000'000>(130'000,
                                                              1024)
                .has_value(),
              "PWM frequency unreachable");
