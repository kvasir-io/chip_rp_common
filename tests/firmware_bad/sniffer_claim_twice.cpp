// expect: a hardware resource is claimed by two peripherals.*DMA channel 0
// chips: rp2040 rp2350
#include "Board.hpp"

#include <chip/rp_common/CRC.hpp>

struct DmaConfig {
    static constexpr auto numberOfChannels     = 2;
    static constexpr auto callbackFunctionSize = 0;
    static constexpr auto isrPriority          = 3;
};

using Dma = Kvasir::DMA::DmaBase<DmaConfig>;

using Sniffer
  = Kvasir::CRC::SnifferCalc<Kvasir::Crc::Presets::crc32IsoHdlc, Dma, Dma::Channel::ch0>;

// a driver on the sniffer's channel: its transfers and the sniffer's would overwrite each other
struct Driver {
    using Claims = Kvasir::DMA::Claims<Dma, Dma::Channel::ch0>;
};

using Startup = Kvasir::Startup::Startup<HW::ClockSettings, HW::SystickClock, Dma, Sniffer, Driver>;

static_assert(sizeof(Startup) > 0);
