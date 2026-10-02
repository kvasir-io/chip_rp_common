// expect-ok
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/DMA.hpp>
#include <chip/rp_common/pio/I2sOut.hpp>
#include <type_traits>

// I2sOut's counters exist only with Statistics (the default): without, no RAM and no counting
struct DmaConfig {
    static constexpr auto channels          = Kvasir::DMA::Channels<Kvasir::DMA::DMAChannel::ch1>{};
    static constexpr auto interruptInstance = 0;
    static constexpr auto callbackFunctionSize = 16;
};

using Dma = Kvasir::DMA::DmaBase<DmaConfig>;

template<bool Stats>
struct AudioConfig {
    static constexpr auto ClockSpeed   = HW::ClockSpeed;
    static constexpr auto PioInstance  = 1;
    static constexpr auto SmInstance   = 1;
    static constexpr auto SampleRate   = 24'000;
    static constexpr auto BufferFrames = 64;
    static constexpr bool Statistics   = Stats;
};

template<bool Stats>
using Audio = Kvasir::Pio::I2sOut<HW::SystickClock,
                                  Kvasir::Register::PinLocation<0, 2>,
                                  Kvasir::Register::PinLocation<0, 3>,
                                  Kvasir::Register::PinLocation<0, 4>,
                                  Dma,
                                  Dma::Channel::ch1,
                                  Dma::Priority::high,
                                  AudioConfig<Stats>>;

static_assert(std::is_empty_v<typename Audio<false>::Stats>);
template<typename A>
constexpr bool counts = requires {
    A::frames();
    A::underrunCount();
};
static_assert(!counts<Audio<false>> && counts<Audio<true>>);
