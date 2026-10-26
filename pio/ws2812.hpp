#pragma once
#include "chip/rp_common/Clocks.hpp"
#include "chip/rp_common/DMA.hpp"
#include "chip/rp_common/PIO.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/Ws2812Program.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Util/Config.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Pio {
    /// A WS2812 bit's waveform, in state-machine cycles. The SM runs at ledClockSpeed *
    /// (t1 + t2 + t3), i.e. 9.6 MHz (104.17 ns/cycle) for the usual 800 kHz LED clock and {3, 4, 5}.
    /// A bit is
    ///   0: high for t1,      low for t2 + t3
    ///   1: high for t1 + t2, low for t3
    ///
    /// {3, 4, 5}, 12 cycles per bit rather than the minimum that works: the extra granularity is
    /// what lets all four pulse widths sit in the interior of the acceptance window of every part
    /// in this family (WS2812, WS2812B, SK6812, Wuerth WL-ICLED) instead of on a bound. At 10
    /// cycles there is exactly one set that fits at all, and its T1H lands on SK6812's ceiling.
    /// WS2812's static_asserts check the widths against those windows, so a set (or an
    /// ledClockSpeed) that does not suit the parts is a compile error, not a misbehaving strip.
    struct Ws2812Cycles {
        unsigned t1;
        unsigned t2;
        unsigned t3;
    };

    namespace detail {
        // WS2812's configuration, program, timing checks and state machine, outside the class so
        // that the state machine can be WS2812's base (StartupOf). WS2812 derives from it too:
        // its names stay WS2812's (NeoPixel::Program, ::DivFactor, ::PinIndex ...).
        template<typename Pin, typename Config_>
        struct Ws2812Parts {
            static constexpr unsigned pinNumber
              = static_cast<unsigned>(Kvasir::Io::pinNumber(Pin{}));

            struct Config : Config_ {
                static constexpr Ws2812Cycles cycles
                  = Ws2812Cycles{KVASIR_CONFIG_OR(Config_, cycles, Ws2812Cycles{3, 4, 5})};
                static constexpr auto ledClockSpeed
                  = KVASIR_CONFIG_OR(Config_, ledClockSpeed, 800000);
                static constexpr auto programOffset = KVASIR_CONFIG_OR(Config_, programOffset, 0);

                static constexpr auto resetTime
                  = KVASIR_CONFIG_OR(Config_, resetTime, std::chrono::microseconds{60});
                static constexpr unsigned gpioBase = static_cast<unsigned>(
                  KVASIR_CONFIG_OR(Config_,
                                   gpioBase,
                                   Kvasir::Pio::gpioBaseFor(std::array{pinNumber})));
            };

            using Program = Ws2812Program<Config::cycles.t1, Config::cycles.t2, Config::cycles.t3>;

            static constexpr double divFactor{
              double{Config::clockSpeed}
              / (double{Config::ledClockSpeed} * double{Program::cyclesPerBit})};

            // SMx_CLKDIV is a 16.8 divider, INT 31:16 and FRAC 15:8 (RP2040 datasheet Table 381,
            // RP2350 datasheet Table 995); the pulse-width asserts below bound it further.
            static_assert(divFactor >= 1.0 && divFactor < 65536.0,
                          "ledClockSpeed unreachable from clockSpeed with a 16.8 PIO divider");

            // The four pulse widths the program produces, in nanoseconds. The SM runs at
            // ledClockSpeed * CyclesPerBit, so ledClockSpeed scales all four *proportionally*; a
            // part that wants different ratios, such as WS2811 in its 400kHz mode, needs other
            // cycles. The asserts below are what says so at compile time instead of leaving it to
            // be discovered on a strip.
            static constexpr double cycleTimeNs{
              1.0e9 / (double{Config::ledClockSpeed} * double{Program::cyclesPerBit})};

            static constexpr double t0hNs{double{Program::t1} * cycleTimeNs};
            static constexpr double t1hNs{double{Program::t1 + Program::t2} * cycleTimeNs};
            static constexpr double t0lNs{double{Program::t2 + Program::t3} * cycleTimeNs};
            static constexpr double t1lNs{double{Program::t3} * cycleTimeNs};

            // Intersection of the acceptance windows of the parts this driver targets. Each bound
            // is the tightest one across WS2812, WS2812B, SK6812 and the Wuerth WL-ICLED, so a
            // failure here does not necessarily mean the width is wrong for the part on *your*
            // board -- check which part sets the bound before widening it.
            static_assert(t0hNs >= 250.0,
                          "T0H below WS2812B minimum");
            static_assert(t0hNs <= 400.0,
                          "T0H above WL-ICLED maximum");
            static_assert(t1hNs >= 650.0,
                          "T1H below WS2812B minimum");
            static_assert(t1hNs <= 750.0,
                          "T1H above SK6812 maximum");
            static_assert(t0lNs >= 800.0,
                          "T0L below WL-ICLED minimum");
            static_assert(t0lNs <= 1000.0,
                          "T0L above WS2812B maximum");
            static_assert(t1lNs >= 450.0,
                          "T1L below SK6812 minimum");
            static_assert(t1lNs <= 600.0,
                          "T1L above WS2812B maximum");

            // The state machine: StateMachine (PioStateMachine.hpp) loads the program, maps the pin
            // as its side-set pin, sets the divider, and takes the FIFO join, autopull, threshold
            // and shift direction from the program (Ws2812Program's .fifo / .out) - and checks all
            // of it against the program.
            struct SmConfig {
                static constexpr auto     clockSpeed    = Config::clockSpeed;
                static constexpr auto     pioInstance   = Config::pioInstance;
                static constexpr auto     smInstance    = Config::smInstance;
                static constexpr unsigned programOffset = Config::programOffset;
                static constexpr unsigned gpioBase      = Config::gpioBase;
                static constexpr double   clockDiv      = divFactor;
                static constexpr auto     sidesetPins   = brigand::list<Pin>{};
            };

            using Sm = StateMachine<Program, SmConfig>;

            // The pin as the state machine names it: PINCTRL's bases are five bits wide and count
            // from the instance's GPIOBASE, not from GPIO 0 (PIO.hpp); StateMachine checks the window.
            static constexpr unsigned pinIndex = Kvasir::Pio::pinIndex(pinNumber, Config::gpioBase);
        };
    }   // namespace detail

    /// Drives a chain of WS2812-style LEDs from one PIO state machine, fed by DMA.
    ///
    /// Config:
    ///   clockSpeed     (required) system clock feeding the PIO
    ///   pioInstance    (required) 0 or 1
    ///   smInstance     (required) 0..3
    ///   ledClockSpeed  (default 800000) bit rate on the wire
    ///   cycles         (default {3, 4, 5}) the bit's waveform in SM cycles (Ws2812Cycles);
    ///                                   another set reaches parts with other ratios, such
    ///                                   as WS2811 in its 400kHz mode
    ///   programOffset  (default 0)      where the program is loaded in instruction memory
    ///   gpioBase       (derived)        the instance's GPIO window, 0 or 16 (PIO.hpp).
    ///                                   Derived from Pin; name it only to agree with
    ///                                   another driver on the same PIO instance.
    ///   resetTime      (default 60us)   line-low time that latches a frame.
    ///                                   WS2812/WS2812B want >=50us, but several parts in
    ///                                   this family specify more -- the Wuerth WL-ICLED
    ///                                   needs >=200us, and sending the next frame sooner
    ///                                   makes it a continuation of the previous one
    ///                                   instead of a new one. Override per board.
    ///
    /// send() is asynchronous and the driver holds no pixel buffer of its own: the span
    /// handed to send() must stay alive and unmodified until ready() is true again.
    /// turn() must be polled from the main loop -- completion is detected there, and a
    /// driver that is never polled stays busy forever.
    template<typename Clock,
             typename Pin,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             typename Config_>
    struct WS2812
      : detail::Ws2812Parts<Pin, Config_>
      , StartupOf<typename detail::Ws2812Parts<Pin, Config_>::Sm> {
        using Parts = detail::Ws2812Parts<Pin, Config_>;
        using typename Parts::Config;
        using typename Parts::Sm;

        // Startup: the state machine and instruction slots the program occupies, the GPIO
        // window it needs of its instance, the DMA channel, and the clock the divider is
        // computed from.
        using Claims = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>, typename Sm::Claims>;

        static inline bool                       running{false};
        static inline typename Clock::time_point whenRdy{};

        /// Starts a DMA transfer of `leds` to the state machine.
        /// Returns false and does nothing when a previous frame is still in flight or the
        /// reset window has not elapsed -- dropping that result silently drops frames, so
        /// it is [[nodiscard]]. The caller keeps ownership of the buffer either way, and
        /// must not touch it until ready() is true again.
        template<typename RGB>
        [[nodiscard]] static bool send(std::span<RGB> leds) {
            static_assert(sizeof(RGB) == 3, "only rgb");
            if(!ready()) { return false; }

            // Clear the stall flag before the transfer starts, never after: turn() takes
            // the flag as the end-of-frame marker, so a clear that lands after the first
            // data is on its way could wipe a stall belonging to this frame.
            Sm::clearTxStall();

            Dma::template start<DmaChannel,
                                ::Kvasir::DMA::ChannelConfig{.priority = DmaPriority,
                                                             .trigger
                                                             = Sm::template txDmaTrigger<Dma>(),
                                                             .size = Dma::TransferSize::size8,
                                                             .incrementRead  = true,
                                                             .incrementWrite = false}>(
              Sm::txFifoAddress,
              reinterpret_cast<std::uint32_t>(leds.data()),
              leds.size() * sizeof(RGB));

            running = true;
            return true;
        }

        static bool ready() {
            if(running) { return false; }
            return Clock::now() >= whenRdy || whenRdy == typename Clock::time_point{};
        }

        static void turn() {
            if(Sm::txStalled() && running) {
                running = false;
                whenRdy = Clock::now() + Config::resetTime;
            }
        }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (Port/Extends.hpp);
        // a firmware that runs the hook must not also call turn() by hand
        using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &WS2812::turn>;
    };
}}   // namespace Kvasir::Pio
