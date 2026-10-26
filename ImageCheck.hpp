#pragma once
// The RP backend of Kvasir::ImageCheck (Kvasir_SDK kvasir/Runtime/ImageCheck.hpp): the XIP streaming FIFO read by a DMA
// channel into a dummy word with the DMA's CRC sniffer on that channel, so no CPU cycle goes per byte and the reads
// neither use nor pollute the XIP cache.
//
//     using ImageCheck = Kvasir::ImageCheck::Checker<Kvasir::ImageCheck::RpStream<HW::Dma, HW::Dma::Channel::ch3>,
//                                                    Kvasir::ImageCheck::EveryTurn<4096>>;
//
// The stream (Xip.hpp; RP2040 datasheet 2.6.3.4 md l.5986, RP2350 4.4.3 md l.18043): STREAM_ADDR takes the XIP
// address of the first word, STREAM_CTR the word count (bits 21:0), the flash reads run in the background at lower
// priority than the core's XIP fetches; the FIFO raises the xip_stream DREQ and the DMA reads it through the
// auxiliary port. The sniffer's accumulator (SNIFF_DATA) persists between transfers, so a pass is one armSniffer and
// one stream per chunk; the result transforms (OUT_REV, OUT_INV) apply on read only (RP2040 2.5.5.2, RP2350 12.6.8.2).
// Bytes before the first aligned word and after the last whole one go through the same channel as 8-bit memory
// transfers from the uncached window (Flash::uncached), sniffed alike.
//
// Synchronous: chunk() waits for the channel (polled BUSY, so it works with interrupts masked and with a DmaBase
// without callback slots), so no stream is in flight when it returns and no flash writer on this core can meet one.
// A channel that does not finish in TimeoutUs is aborted (RP2040-E8's workaround after clearing STREAM_CTR: one
// dummy read of the uncached window, RP2040 datasheet errata md l.29710-29722) and the chunk fails: the checker
// drops the pass instead of judging it.
//
// A pass keeps its sum in the sniffer across turns, so nothing else may use the sniffer while a pass runs: listed
// (the Checker forwards these), the backend claims its channel and provides the sniffer as held (DmaCrc.hpp's
// SnifferUse), which any other sniffer user in the Startup lists - a SnifferCalc, an image's DmaCrc::Sniffer -
// collides with.
#include "DMA.hpp"
#include "DmaCrc.hpp"
#include "WaitBounds.hpp"
#include "Xip.hpp"
#include "flash.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Runtime/ImageCheck.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace Kvasir::ImageCheck {

template<typename Dma,
         typename Dma::Channel Channel,
         bool                  ByteSwap  = false,
         std::uint64_t         TimeoutUs = 100'000>
struct RpStream {
    static_assert(Dma::ownsChannel(Channel),
                  "the image check's DMA channel is not one of this DmaBase's channels");

    using Claims   = Kvasir::DMA::Claims<Dma, Channel>;
    using Provides = brigand::list<Kvasir::DmaCrc::SnifferUse<Kvasir::DmaCrc::heldBy(Channel)>>;

    static constexpr auto sniff = [] {
        auto s     = Kvasir::DmaCrc::Sniff::crc32();
        s.byteSwap = ByteSwap;
        return s;
    }();

    static void begin() { Kvasir::DmaCrc::armSniffer<sniff, Dma, Channel>(); }

    [[nodiscard]] static bool chunk(std::uintptr_t address,
                                    std::size_t    length) {
        auto       a    = static_cast<std::uint32_t>(address);
        auto const end  = a + static_cast<std::uint32_t>(length);
        auto const head = std::min<std::uint32_t>((4U - (a & 3U)) & 3U, end - a);
        if(head != 0 && !bytes(a, head)) { return false; }
        a += head;
        std::uint32_t const words = (end - a) / 4U;
        if(words != 0) {
            Kvasir::Xip::startStream(a, words);
            Dma::template start<Channel,
                                ::Kvasir::DMA::ChannelConfig{
                                  .priority       = Dma::Priority::high,
                                  .trigger        = Dma::TriggerSource::xip_stream,
                                  .size           = Dma::TransferSize::size32,
                                  .incrementRead  = false,
                                  .incrementWrite = false,
                                  .sniff          = true}>(reinterpret_cast<std::uint32_t>(&sink_),
                                                  Kvasir::Xip::streamFifoAux,
                                                  words);
            if(!finished()) { return false; }
            a += words * 4U;
        }
        return a == end || bytes(a, end - a);
    }

    [[nodiscard]] static std::uint32_t result() {
        return Kvasir::DmaCrc::sniffResult<sniff, Dma>();
    }

private:
    using Timeout = Kvasir::Register::MicrosecondsBound<TimeoutUs>;

    static bool bytes(std::uint32_t a,
                      std::uint32_t n) {
        Dma::template start<Channel,
                            ::Kvasir::DMA::ChannelConfig{.priority = Dma::Priority::high,
                                                         .trigger  = Dma::TriggerSource::permanent,
                                                         .size     = Dma::TransferSize::size8,
                                                         .incrementRead  = true,
                                                         .incrementWrite = false,
                                                         .sniff          = true}>(
          reinterpret_cast<std::uint32_t>(&sink_),
          Kvasir::Flash::uncached(a),
          n);
        return finished();
    }

    static bool finished() {
        for(std::uint32_t i = 0; i != Timeout::polls; ++i) {
            if(Dma::template ready<Channel>()) { return true; }
        }
        quiesce();
        return false;
    }

    // stop the stream (RP2040-E8: then one dummy read of the uncached window) and the channel
    static void quiesce() {
        apply(write(Kvasir::Xip::Ctrl::STREAM_CTR::stream_ctr, 0U));
        (void)Kvasir::Memory::load<std::uint32_t>(Kvasir::Flash::xipUncachedBase);
        (void)Dma::template abort<Channel>();
        while(!Kvasir::Xip::streamFifoEmpty()) {
            (void)apply(read(Kvasir::Xip::Ctrl::STREAM_FIFO::FULLREGISTER));
        }
    }

    static inline std::uint32_t volatile sink_{};
};
}   // namespace Kvasir::ImageCheck
