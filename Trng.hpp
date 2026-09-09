#pragma once
#if !__has_include("chip/rp2350.hpp")
    #error "the TRNG exists on the RP2350 only"
#endif
#include "kvasir/Register/Register.hpp"
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
//   rawSamples (true)   bypass the block's statistical checks (von Neumann, continuous,
//                       autocorrelation) as pico_rand does: fast, meant to be conditioned by
//                       a hash or a PRNG seed. false keeps the checks; a rejected fill is
//                       discarded and next32() waits for the next.
//   sampleCount (0)     oscillator cycles between samples (SAMPLE_CNT1); 0 is every cycle.
//   chainLength (0)     which of the four ring-oscillator lengths (CONFIG.RND_SRC_SEL).
namespace Kvasir { namespace Trng {

    template<typename Config_ = void>
    struct Trng {
        struct Config {
            static constexpr bool rawSamples = [] {
                if constexpr(requires { Config_::rawSamples; }) {
                    return static_cast<bool>(Config_::rawSamples);
                } else {
                    return true;
                }
            }();
            static constexpr std::uint32_t sampleCount = [] {
                if constexpr(requires { Config_::sampleCount; }) {
                    return static_cast<std::uint32_t>(Config_::sampleCount);
                } else {
                    return 0U;
                }
            }();
            static constexpr std::uint32_t chainLength = [] {
                if constexpr(requires { Config_::chainLength; }) {
                    return static_cast<std::uint32_t>(Config_::chainLength);
                } else {
                    return 0U;
                }
            }();
        };

        static_assert(Config::chainLength < 4,
                      "the TRNG has four ring-oscillator chain lengths");

        using Regs = Kvasir::Peripheral::TRNG::Registers<>;

        static constexpr auto powerClockEnable
          = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::trng));

        static constexpr auto initStepPeripheryConfig = list(
          write(Regs::SAMPLE_CNT1::sample_cntr1, Register::value<Config::sampleCount>()),
          write(Regs::CONFIG::rnd_src_sel, Register::value<Config::chainLength>()),
          write(Regs::DEBUG_CONTROL::vnc_bypass, Register::value<Config::rawSamples ? 1 : 0>()),
          write(Regs::DEBUG_CONTROL::crngt_bypass, Register::value<Config::rawSamples ? 1 : 0>()),
          write(Regs::DEBUG_CONTROL::auto_correlate_bypass,
                Register::value<Config::rawSamples ? 1 : 0>()));

        static constexpr auto initStepPeripheryEnable
          = list(set(Regs::RND_SOURCE_ENABLE::rnd_src_en));

        static constexpr std::size_t WordsPerFill = 6;

        /// Whether the EHR holds a complete fill.
        [[nodiscard]] static bool ready() { return apply(read(Regs::VALID::ehr_valid)); }

        /// One fill of the EHR: six words, 192 bits. Blocks until the fill is complete; with
        /// the checks on, a fill the block rejected is discarded and the wait starts over.
        [[nodiscard]] static std::array<std::uint32_t,
                                        WordsPerFill>
        fill() {
            while(true) {
                while(!ready()) {
                    if constexpr(!Config::rawSamples) {
                        auto const errors = apply(read(Regs::RNG_ISR::autocorr_err,
                                                       Regs::RNG_ISR::crngt_err,
                                                       Regs::RNG_ISR::vn_err));
                        if(get<0>(errors) || get<1>(errors) || get<2>(errors)) {
                            // Clear the finding and restart the bit counter; the source keeps
                            // running.
                            apply(write(Regs::RNG_ICR::FULLREGISTER, Register::value<0xFU>()));
                            apply(set(Regs::RST_BITS_COUNTER::rst_bits_counter));
                        }
                    }
                }
                std::array<std::uint32_t, WordsPerFill> words{
                  get<0>(apply(read(Regs::EHR_DATA0::ehr_data0))),
                  get<0>(apply(read(Regs::EHR_DATA1::ehr_data1))),
                  get<0>(apply(read(Regs::EHR_DATA2::ehr_data2))),
                  get<0>(apply(read(Regs::EHR_DATA3::ehr_data3))),
                  get<0>(apply(read(Regs::EHR_DATA4::ehr_data4))),
                  get<0>(
                    apply(read(Regs::EHR_DATA5::ehr_data5)))};   // this read starts the next fill
                apply(write(Regs::RNG_ICR::FULLREGISTER, Register::value<0xFU>()));
                return words;
            }
        }

        /// A word from the cached fill, refilling when it is used up.
        [[nodiscard]] static std::uint32_t next32() {
            static std::array<std::uint32_t, WordsPerFill> cache{};
            static std::size_t                             left = 0;
            if(left == 0) {
                cache = fill();
                left  = WordsPerFill;
            }
            return cache[--left];
        }

        [[nodiscard]] static std::uint64_t next64() {
            return (static_cast<std::uint64_t>(next32()) << 32) | next32();
        }
    };

}}   // namespace Kvasir::Trng
