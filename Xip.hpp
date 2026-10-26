#pragma once
#include "DMA.hpp"
#include "chip/RpTraits.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/XIP_CTRL.hpp"

#include <cstddef>
#include <cstdint>

// The XIP cache's two counters and its streaming FIFO (RP2350 datasheet 4.4.3, 4.4.4).
//
// Counters: every access to cached flash bumps CTR_ACC, every hit CTR_HIT; both saturate
// and clear on any write. hitRate() over a loop says how much of it ran from the cache.
//
// Stream: STREAM_ADDR / STREAM_CTR make the cache controller read a run of words from
// flash in the background into a FIFO, without polluting the cache; a DMA channel with
// the xip_stream TREQ drains the FIFO through the XIP_AUX window (so the DMA's reads
// never contend with the core's own XIP fetches). Kvasir::Xip::stream<Dma, Channel>(from,
// into, words, callback) sets both up.
namespace Kvasir { namespace Xip {

    using Ctrl = Kvasir::Peripheral::XIP_CTRL::Registers<>;

    struct Counters {
        std::uint32_t hits{};
        std::uint32_t accesses{};

        [[nodiscard]] constexpr std::uint32_t hitPermille() const {
            return accesses == 0 ? 0U
                                 : static_cast<std::uint32_t>(
                                     (static_cast<std::uint64_t>(hits) * 1000U) / accesses);
        }
    };

    [[nodiscard]] inline Counters counters() {
        return {get<0>(apply(read(Ctrl::CTR_HIT::FULLREGISTER))),
                get<0>(apply(read(Ctrl::CTR_ACC::FULLREGISTER)))};
    }

    inline void resetCounters() {
        apply(write(Ctrl::CTR_HIT::FULLREGISTER, Register::value<0>()));
        apply(write(Ctrl::CTR_ACC::FULLREGISTER, Register::value<0>()));
    }

    /// The streaming FIFO's address through the auxiliary window: what a DMA channel reads (chip/RpTraits.hpp).
    static constexpr std::uint32_t streamFifoAux = Rp::ChipTraits::xipStreamFifoAux;

    [[nodiscard]] inline bool streamFifoEmpty() { return apply(read(Ctrl::STAT::fifo_empty)) != 0; }

    /// Drain whatever a previous stream left, then start `words` from `flashAddress`
    /// (a 4-byte aligned XIP address). STREAM_ADDR's field is bits 31:2 (RP2040 Table 160, RP2350
    /// Table 444): the field value is the word address.
    inline void startStream(std::uint32_t flashAddress,
                            std::uint32_t words) {
        while(!streamFifoEmpty()) { (void)apply(read(Ctrl::STREAM_FIFO::FULLREGISTER)); }
        apply(write(Ctrl::STREAM_ADDR::stream_addr, flashAddress >> 2U));
        apply(write(Ctrl::STREAM_CTR::stream_ctr, words));
    }

    /// Words the stream still has to fetch (0: done, the FIFO may still hold some).
    [[nodiscard]] inline std::uint32_t streamRemaining() {
        return get<0>(apply(read(Ctrl::STREAM_CTR::stream_ctr)));
    }

    /// A stream into RAM by DMA: `words` 32-bit words from `flashAddress` into `into`,
    /// `f` when the channel is done.
    template<typename Dma,
             typename Dma::Channel Channel,
             typename F>
    inline void stream(std::uint32_t  flashAddress,
                       std::uint32_t* into,
                       std::uint32_t  words,
                       F&&            f) {
        static_assert(Dma::ownsChannel(Channel),
                      "the XIP stream's DMA channel is not this DmaBase's");
        startStream(flashAddress, words);
        Dma::template start<Channel,
                            ::Kvasir::DMA::ChannelConfig{.priority = Dma::Priority::high,
                                                         .trigger  = Dma::TriggerSource::xip_stream,
                                                         .size     = Dma::TransferSize::size32,
                                                         .incrementRead  = false,
                                                         .incrementWrite = true}>(
          reinterpret_cast<std::uint32_t>(into),
          streamFifoAux,
          words,
          std::forward<F>(f));
    }

}}   // namespace Kvasir::Xip
