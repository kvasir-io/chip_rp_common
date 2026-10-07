// expect-ok
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/CRC.hpp>
#include <type_traits>

// a SnifferCalc in the Startup list next to its DmaBase: the channel is claimed once and provided
struct DmaConfig {
    static constexpr auto numberOfChannels     = 2;
    static constexpr auto callbackFunctionSize = 0;
    static constexpr auto isrPriority          = 3;
};

using Dma = Kvasir::DMA::DmaBase<DmaConfig>;

using Sniffer
  = Kvasir::CRC::SnifferCalc<Kvasir::Crc::Presets::crc32IsoHdlc, Dma, Dma::Channel::ch0>;

// an unlisted preset on the same channel claims nothing; another channel is another resource
using Other = Kvasir::CRC::SnifferCalc<Kvasir::Crc::Presets::crc16Ibm3740, Dma, Dma::Channel::ch1>;

using Startup = Kvasir::Startup::Startup<HW::ClockSettings, HW::SystickClock, Dma, Sniffer, Other>;

static_assert(sizeof(Startup) > 0);
static_assert(std::is_same_v<Sniffer::Claims,
                             Kvasir::DMA::Claims<Dma,
                                                 Dma::Channel::ch0>>);
