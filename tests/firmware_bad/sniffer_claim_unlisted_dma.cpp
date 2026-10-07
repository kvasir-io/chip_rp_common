// expect: claims a hardware resource nothing in any Startup list provides: DMA channel 0
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/CRC.hpp>

struct DmaConfig {
    static constexpr auto numberOfChannels     = 1;
    static constexpr auto callbackFunctionSize = 0;
    static constexpr auto isrPriority          = 3;
};

using Dma = Kvasir::DMA::DmaBase<DmaConfig>;

using Sniffer
  = Kvasir::CRC::SnifferCalc<Kvasir::Crc::Presets::crc32IsoHdlc, Dma, Dma::Channel::ch0>;

// the DmaBase is in no list: the block stays in reset and the completion flag is never cleared
using Startup = Kvasir::Startup::Startup<HW::ClockSettings, HW::SystickClock, Sniffer>;

static_assert(sizeof(Startup) > 0);
