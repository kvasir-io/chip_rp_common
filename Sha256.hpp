#pragma once
#if !__has_include("chip/rp2350.hpp")
    #error "the SHA-256 block exists on the RP2350 only"
#endif
#include "kvasir/Register/Register.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/SHA256.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// The RP2350's SHA-256 accelerator as a Startup-list peripheral. The block takes the message
// as 32-bit words (sixteen per 64-byte block, WDATA_RDY between them) and holds the digest
// in SUM0..7; it does not pad, so the driver keeps the partial block and appends the FIPS
// 180-4 padding at the end. BSWAP is on, so a byte stream hashes as a byte stream.
//
//   using Sha = Kvasir::Sha256::Sha256<>;             // in the Startup list
//   auto const d = Sha::hash(bytes);                  // one call, blocking
//   Sha::start(); Sha::update(a); Sha::update(b); auto const d = Sha::finish();
//
// updateDma<Dma, Channel>(bytes, callback) feeds the whole 64-byte blocks by DMA (paced by
// the block's DREQ) and the remainder by the CPU once the callback runs; finish() as before.
namespace Kvasir { namespace Sha256 {

    using Digest = std::array<std::uint8_t, 32>;

    template<typename Config_ = void>
    struct Sha256 {
        using Regs = Kvasir::Peripheral::SHA256::Registers<>;

        static constexpr std::size_t BlockBytes = 64;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::sha256));

        static constexpr auto initStepPeripheryConfig
          = list(Regs::CSR::overrideDefaults(write(Regs::CSR::bswap, Register::value<1>()),
                                             write(Regs::CSR::DMA_SIZEValC::_32bit)));

        /// A new message.
        static void start() {
            apply(
              write(Regs::CSR::FULLREGISTER,
                    Register::value<(1U << 12) | (2U << 8) | 1U>()));   // bswap, 32-bit DMA, START
            pending_ = 0;
            total_   = 0;
        }

        /// More of the message, any length. Whole 64-byte blocks go straight to the
        /// block, the rest waits in the driver for the next update() or finish().
        static void update(std::span<std::byte const> data) {
            total_ += data.size();
            for(auto const b : data) {
                block_[pending_++] = b;
                if(pending_ == BlockBytes) {
                    feedBlock_();
                    pending_ = 0;
                }
            }
        }

        /// The padding, then the digest. Blocks until the block has it (a few hundred
        /// cycles after the last word).
        [[nodiscard]] static Digest finish() {
            auto const bits    = static_cast<std::uint64_t>(total_) * 8U;
            block_[pending_++] = std::byte{0x80};
            if(pending_ > BlockBytes - 8) {
                while(pending_ < BlockBytes) { block_[pending_++] = std::byte{0}; }
                feedBlock_();
                pending_ = 0;
            }
            while(pending_ < BlockBytes - 8) { block_[pending_++] = std::byte{0}; }
            for(int i = 7; i >= 0; --i) {
                block_[pending_++] = static_cast<std::byte>(bits >> (8 * i));
            }
            feedBlock_();
            pending_ = 0;

            while(!apply(read(Regs::CSR::sum_vld))) {}
            Digest d{};
            storeWord_(d, 0, get<0>(apply(read(Regs::SUM0::sum0))));
            storeWord_(d, 4, get<0>(apply(read(Regs::SUM1::sum1))));
            storeWord_(d, 8, get<0>(apply(read(Regs::SUM2::sum2))));
            storeWord_(d, 12, get<0>(apply(read(Regs::SUM3::sum3))));
            storeWord_(d, 16, get<0>(apply(read(Regs::SUM4::sum4))));
            storeWord_(d, 20, get<0>(apply(read(Regs::SUM5::sum5))));
            storeWord_(d, 24, get<0>(apply(read(Regs::SUM6::sum6))));
            storeWord_(d, 28, get<0>(apply(read(Regs::SUM7::sum7))));
            return d;
        }

        [[nodiscard]] static Digest hash(std::span<std::byte const> data) {
            start();
            update(data);
            return finish();
        }

        /// The whole-block prefix of `data` by DMA (32-bit words, paced by the block's
        /// request line), the tail by the CPU once `f` has run. `data` must be 4-byte
        /// aligned and stay put; call finish() after the callback.
        template<typename Dma,
                 typename Dma::Channel Channel,
                 typename F>
        static void updateDma(std::span<std::byte const> data,
                              F&&                        f) {
            static_assert(Dma::ownsChannel(Channel),
                          "the SHA-256 DMA channel is not this DmaBase's");
            auto const blocks = (data.size() / BlockBytes) * BlockBytes;
            auto const tail   = data.subspan(blocks);
            if(blocks == 0) {
                update(tail);
                f();
                return;
            }
            total_ += blocks;
            tail_ = tail;
            Dma::template start<Channel,
                                Dma::Priority::high,
                                Dma::TriggerSource::sha256,
                                Dma::TransferSize::_32,
                                false,
                                true>(Regs::WDATA::Addr::value,
                                      reinterpret_cast<std::uint32_t>(data.data()),
                                      blocks / 4,
                                      [cb = std::forward<F>(f)]() {
                                          update(tail_);
                                          cb();
                                      });
        }

        /// The block's error flag: a word was written while it was not ready (sticky).
        [[nodiscard]] static bool overrun() { return apply(read(Regs::CSR::err_wdata_not_rdy)); }

        static constexpr std::uint32_t wdataAddress = Regs::WDATA::Addr::value;

    private:
        static void feedBlock_() {
            for(std::size_t i = 0; i < BlockBytes; i += 4) {
                std::uint32_t const w = static_cast<std::uint32_t>(block_[i])
                                      | (static_cast<std::uint32_t>(block_[i + 1]) << 8)
                                      | (static_cast<std::uint32_t>(block_[i + 2]) << 16)
                                      | (static_cast<std::uint32_t>(block_[i + 3]) << 24);
                while(!apply(read(Regs::CSR::wdata_rdy))) {}
                apply(write(Regs::WDATA::wdata, w));
            }
        }

        static void storeWord_(Digest&       d,
                               std::size_t   at,
                               std::uint32_t w) {
            d[at]     = static_cast<std::uint8_t>(w >> 24);
            d[at + 1] = static_cast<std::uint8_t>(w >> 16);
            d[at + 2] = static_cast<std::uint8_t>(w >> 8);
            d[at + 3] = static_cast<std::uint8_t>(w);
        }

        static inline std::array<std::byte, BlockBytes> block_{};
        static inline std::size_t                       pending_{};
        static inline std::uint64_t                     total_{};
        static inline std::span<std::byte const>        tail_{};
    };

}}   // namespace Kvasir::Sha256
