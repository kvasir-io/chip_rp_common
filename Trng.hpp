#pragma once
#ifndef KVASIR_CHIP_RP2350
    #error "the TRNG exists on the RP2350 only"
#endif
#include "chip/Blocks.hpp"   // the chip's block table (Rp::Block)
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Config.hpp"
#include "peripherals/RESETS.hpp"
#include "peripherals/TRNG.hpp"

#include <array>
#include <cstdint>

// The RP2350's true random number generator as a Startup-list peripheral: a ring oscillator
// sampled into a 192-bit entropy holding register (EHR), six 32-bit words per fill; reading
// the last word starts the next fill.
//
//   using Trng = Kvasir::Trng::Trng<>;                 // in the Startup list
//   std::uint32_t const r = Trng::next32();           // blocks for a fill when the cache is empty
//   std::uint64_t const s = Trng::next64();
//
// Config (all optional):
//   rawSamples (false)  true: bypass the block's statistical checks (von Neumann, continuous,
//                       autocorrelation) as pico_rand does: fast, but the bits are raw ring-
//                       oscillator samples - neighbouring bits correlate, a word is not uniform.
//                       Only for a cryptographic hash. false (the default) keeps the checks:
//                       uniform words, a fill in about 1.3 ms; a rejected run is restarted (it
//                       stops the generator).
//   sampleCount         rng_clk (= clk_sys) cycles between two ring-oscillator samples (SAMPLE_CNT1).
//                       Checked (rawSamples false): 250. The data sheet suggests 20-25 for about 2 ms a
//                       generation (12.12.2), but measured on an RP2350A at 150 MHz the words that pass
//                       the checks are not uniform there: the low 5 bits of 65536 words fell 36..208 % into
//                       32 bins at 25 (and nearly every run failed a check), 87..122 % at 100, 94..106 % at
//                       250 (a fill ~1.3 ms, 1 failed run) - as even as at 500 and 1000. Raw: 25; the
//                       data sheet forbids less than 17 while the von Neumann decorrelator is bypassed
//                       (12.12.5, SAMPLE_CNT1), which rawSamples does.
//   chainLength (0)     which of the four ring-oscillator lengths (CONFIG.RND_SRC_SEL).
namespace Kvasir { namespace Trng {

    template<typename Config_ = void>
    struct Trng {
        struct Config {
            static constexpr bool rawSamples
              = static_cast<bool>(KVASIR_CONFIG_OR(Config_, rawSamples, false));
            static constexpr std::uint32_t sampleCount = static_cast<std::uint32_t>(
              KVASIR_CONFIG_OR(Config_, sampleCount, rawSamples ? 25U : 250U));
            static constexpr std::uint32_t chainLength
              = static_cast<std::uint32_t>(KVASIR_CONFIG_OR(Config_, chainLength, 0U));
        };

        static_assert(Config::chainLength < 4,
                      "the TRNG has four ring-oscillator chain lengths");
        static_assert(!Config::rawSamples || Config::sampleCount >= 17,
                      "with the von Neumann decorrelator bypassed (rawSamples) the sample count "
                      "must not be less "
                      "than 17 (RP2350 datasheet 12.12.5, SAMPLE_CNT1)");

        using Regs = Kvasir::Peripheral::TRNG::Registers<>;

        static constexpr auto powerClockEnable = list(clear(Rp::Block<Regs>::reset));

        static constexpr auto initStepPeripheryConfig = list(
          write(Regs::SAMPLE_CNT1::sample_cntr1, Register::value<Config::sampleCount>()),
          write(Regs::CONFIG::rnd_src_sel, Register::value<Config::chainLength>()),
          write(Regs::DEBUG_CONTROL::vnc_bypass, Register::value<Config::rawSamples ? 1 : 0>()),
          write(Regs::DEBUG_CONTROL::crngt_bypass, Register::value<Config::rawSamples ? 1 : 0>()),
          write(Regs::DEBUG_CONTROL::auto_correlate_bypass,
                Register::value<Config::rawSamples ? 1 : 0>()));

        static constexpr auto initStepPeripheryEnable
          = list(set(Regs::RND_SOURCE_ENABLE::rnd_src_en));

        static constexpr std::size_t wordsPerFill = 6;

        static inline std::uint32_t restarts_{};

        /// Whether the EHR holds a complete fill.
        [[nodiscard]] static bool ready() { return apply(read(Regs::VALID::ehr_valid)) != 0; }

        /// RNG_ICR: a 1 clears the bit in RNG_ISR (RP2350 datasheet 12.12.5, Table 1264;
        /// AUTOCORR_ERR only by an RNG reset). Every field named, the rest reserved: one store.
        static void clearFindings() {
            apply(set(Regs::RNG_ICR::vn_err,
                      Regs::RNG_ICR::crngt_err,
                      Regs::RNG_ICR::autocorr_err,
                      Regs::RNG_ICR::ehr_valid));
        }

        /// A run that failed an entropy check stops the generator, and AUTOCORR_ERR clears only with
        /// an RNG reset (RP2350 datasheet 12.12.3, 12.12.5 RNG_ICR): soft reset - the two reads after
        /// it are the bootrom's fixed delay (12.12.4.1) - then the configuration again and a new run.
        static void restart() {
            ++restarts_;
            apply(set(Regs::SW_RESET::sw_reset));
            static_cast<void>(apply(read(Regs::SW_RESET::sw_reset)));
            static_cast<void>(apply(read(Regs::SW_RESET::sw_reset)));
            apply(initStepPeripheryConfig);
            clearFindings();
            apply(set(Regs::RND_SOURCE_ENABLE::rnd_src_en));
        }

        /// How many runs failed an entropy check and were restarted.
        [[nodiscard]] static std::uint32_t restarts() { return restarts_; }

        /// One fill of the EHR: six words, 192 bits. Blocks until the fill is complete; with
        /// the checks on, a fill the block rejected is discarded and the wait starts over.
        [[nodiscard]] static std::array<std::uint32_t,
                                        wordsPerFill>
        fill() {
            while(true) {
                while(!ready()) {
                    if constexpr(!Config::rawSamples) {
                        auto const errors = apply(read(Regs::RNG_ISR::autocorr_err,
                                                       Regs::RNG_ISR::crngt_err,
                                                       Regs::RNG_ISR::vn_err));
                        if(get<0>(errors) || get<1>(errors) || get<2>(errors)) { restart(); }
                    }
                }
                std::array<std::uint32_t, wordsPerFill> words{
                  get<0>(apply(read(Regs::EHR_DATA0::ehr_data0))),
                  get<0>(apply(read(Regs::EHR_DATA1::ehr_data1))),
                  get<0>(apply(read(Regs::EHR_DATA2::ehr_data2))),
                  get<0>(apply(read(Regs::EHR_DATA3::ehr_data3))),
                  get<0>(apply(read(Regs::EHR_DATA4::ehr_data4))),
                  get<0>(apply(read(Regs::EHR_DATA5::ehr_data5)))};   // this read clears the EHR
                clearFindings();
                if constexpr(!Config::rawSamples) {
                    // with the checks on, a completed run stops the generator (12.12.3): start the next
                    apply(set(Regs::RND_SOURCE_ENABLE::rnd_src_en));
                }
                return words;
            }
        }

        /// A word from the cached fill, refilling when it is used up.
        [[nodiscard]] static std::uint32_t next32() {
            static std::array<std::uint32_t, wordsPerFill> cache{};
            static std::size_t                             left = 0;
            if(left == 0) {
                cache = fill();
                left  = wordsPerFill;
            }
            return cache[--left];
        }

        [[nodiscard]] static std::uint64_t next64() {
            return (static_cast<std::uint64_t>(next32()) << 32) | next32();
        }
    };

}}   // namespace Kvasir::Trng
