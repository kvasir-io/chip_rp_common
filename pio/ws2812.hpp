#pragma once
#include "chip/rp_common/Clocks.hpp"
#include "chip/rp_common/PIO.hpp"
#include "chip/rp_common/PioStateMachine.hpp"
#include "chip/rp_common/pio/Ws2812Program.hpp"
#include "kvasir/StartUp/Hooks.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Pio {
    /// A WS2812 bit's waveform, in state-machine cycles. The SM runs at LedClockSpeed *
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
    /// LedClockSpeed) that does not suit the parts is a compile error, not a misbehaving strip.
    struct Ws2812Cycles {
        unsigned t1;
        unsigned t2;
        unsigned t3;
    };

    /// Drives a chain of WS2812-style LEDs from one PIO state machine, fed by DMA.
    ///
    /// Config:
    ///   ClockSpeed     (required) system clock feeding the PIO
    ///   PioInstance    (required) 0 or 1
    ///   SmInstance     (required) 0..3
    ///   LedClockSpeed  (default 800000) bit rate on the wire
    ///   Cycles         (default {3, 4, 5}) the bit's waveform in SM cycles (Ws2812Cycles);
    ///                                   another set reaches parts with other ratios, such
    ///                                   as WS2811 in its 400kHz mode
    ///   ProgramOffset  (default 0)      where the program is loaded in instruction memory
    ///   GpioBase       (derived)        the instance's GPIO window, 0 or 16 (PIO.hpp).
    ///                                   Derived from Pin; name it only to agree with
    ///                                   another driver on the same PIO instance.
    ///   ResetTime      (default 60us)   line-low time that latches a frame.
    ///                                   WS2812/WS2812B want >=50us, but several parts in
    ///                                   this family specify more -- the Wuerth WL-ICLED
    ///                                   needs >=200us, and sending the next frame sooner
    ///                                   makes it a continuation of the previous one
    ///                                   instead of a new one. Override per board.
    ///
    /// send() is asynchronous and the driver holds no pixel buffer of its own: the span
    /// handed to send() must stay alive and unmodified until ready() is true again.
    /// handler() must be polled from the main loop -- completion is detected there, and a
    /// driver that is never polled stays busy forever.
    template<typename Clock,
             typename Pin,
             typename Dma,
             typename Dma::Channel  DmaChannel,
             typename Dma::Priority DmaPriority,
             typename Config_>
    struct WS2812 {
        static constexpr unsigned PinNumber
          = []<int Port, int PinN>(Kvasir::Register::PinLocation<Port, PinN>) {
                return static_cast<unsigned>(PinN);
            }(Pin{});

        struct Config : Config_ {
            static constexpr Ws2812Cycles Cycles = [] {
                if constexpr(requires { Config_::Cycles; }) {
                    return Ws2812Cycles{Config_::Cycles};
                } else {
                    return Ws2812Cycles{3, 4, 5};
                }
            }();
            static constexpr auto LedClockSpeed = [] {
                if constexpr(requires { Config_::LedClockSpeed; }) {
                    return Config_::LedClockSpeed;
                } else {
                    return 800000;
                }
            }();
            static constexpr auto ProgramOffset = [] {
                if constexpr(requires { Config_::ProgramOffset; }) {
                    return Config_::ProgramOffset;
                } else if constexpr(requires { Config_::ProgrammOffset; }) {   // the old spelling
                    return Config_::ProgrammOffset;
                } else {
                    return 0;
                }
            }();
            static constexpr auto ResetTime = [] {
                if constexpr(requires { Config_::ResetTime; }) {
                    return Config_::ResetTime;
                } else {
                    return std::chrono::microseconds{60};
                }
            }();
            static constexpr unsigned GpioBase = [] {
                if constexpr(requires { Config_::GpioBase; }) {
                    return static_cast<unsigned>(Config_::GpioBase);
                } else {
                    return Kvasir::Pio::gpioBaseFor(std::array{PinNumber});
                }
            }();
        };

        using Programm = Ws2812Program<Config::Cycles.t1, Config::Cycles.t2, Config::Cycles.t3>;

        static constexpr double DivFactor{
          double{Config::ClockSpeed}
          / (double{Config::LedClockSpeed} * double{Programm::CyclesPerBit})};

        // Kvasir::Pio::getDiv narrows div_int to std::uint8_t without complaining, so a
        // divider that does not fit would silently alias to a wrong SM clock and put every
        // pulse width out of spec.
        static_assert(DivFactor >= 1.0 && DivFactor < 256.0,
                      "LedClockSpeed unreachable from ClockSpeed with an 8-bit PIO divider");

        // The four pulse widths the program produces, in nanoseconds. The SM runs at
        // LedClockSpeed * CyclesPerBit, so LedClockSpeed scales all four *proportionally*; a
        // part that wants different ratios, such as WS2811 in its 400kHz mode, needs other
        // Cycles. The asserts below are what says so at compile time instead of leaving it to
        // be discovered on a strip.
        static constexpr double CycleTimeNs{
          1.0e9 / (double{Config::LedClockSpeed} * double{Programm::CyclesPerBit})};

        static constexpr double T0HNs{double{Programm::T1} * CycleTimeNs};
        static constexpr double T1HNs{double{Programm::T1 + Programm::T2} * CycleTimeNs};
        static constexpr double T0LNs{double{Programm::T2 + Programm::T3} * CycleTimeNs};
        static constexpr double T1LNs{double{Programm::T3} * CycleTimeNs};

        // Intersection of the acceptance windows of the parts this driver targets. Each bound
        // is the tightest one across WS2812, WS2812B, SK6812 and the Wuerth WL-ICLED, so a
        // failure here does not necessarily mean the width is wrong for the part on *your*
        // board -- check which part sets the bound before widening it.
        static_assert(T0HNs >= 250.0,
                      "T0H below WS2812B minimum");
        static_assert(T0HNs <= 400.0,
                      "T0H above WL-ICLED maximum");
        static_assert(T1HNs >= 650.0,
                      "T1H below WS2812B minimum");
        static_assert(T1HNs <= 750.0,
                      "T1H above SK6812 maximum");
        static_assert(T0LNs >= 800.0,
                      "T0L below WL-ICLED minimum");
        static_assert(T0LNs <= 1000.0,
                      "T0L above WS2812B maximum");
        static_assert(T1LNs >= 450.0,
                      "T1L below SK6812 minimum");
        static_assert(T1LNs <= 600.0,
                      "T1L above WS2812B maximum");

        // The state machine: StateMachine (PioStateMachine.hpp) loads the program, maps the pin
        // as its side-set pin, sets the divider, and takes the FIFO join, autopull, threshold
        // and shift direction from the program (Ws2812Program's .fifo / .out) - and checks all
        // of it against the program.
        struct SmConfig {
            static constexpr auto     ClockSpeed    = Config::ClockSpeed;
            static constexpr auto     PioInstance   = Config::PioInstance;
            static constexpr auto     SmInstance    = Config::SmInstance;
            static constexpr unsigned ProgramOffset = Config::ProgramOffset;
            static constexpr unsigned GpioBase      = Config::GpioBase;
            static constexpr double   clockDiv      = DivFactor;
            static constexpr auto     sidesetPins   = brigand::list<Pin>{};
        };

        using Sm = StateMachine<Programm, SmConfig>;

        // The pin as the state machine names it: PINCTRL's bases are five bits wide and count
        // from the instance's GPIOBASE, not from GPIO 0 (PIO.hpp); StateMachine checks the window.
        static constexpr unsigned PinIndex = Kvasir::Pio::pinIndex(PinNumber, Config::GpioBase);

        // Startup: the state machine and instruction slots the program occupies, the GPIO
        // window it needs of its instance, the DMA channel, and the clock the divider is
        // computed from.
        using Provides = typename Sm::Provides;
        using Claims   = brigand::append<Kvasir::DMA::Claims<Dma, DmaChannel>, typename Sm::Claims>;

        static constexpr auto powerClockEnable        = Sm::powerClockEnable;
        static constexpr auto initStepPinConfig       = Sm::initStepPinConfig;
        static constexpr auto initStepPeripheryConfig = Sm::initStepPeripheryConfig;
        static constexpr auto initStepPeripheryEnable = Sm::initStepPeripheryEnable;

        static void preEnableRuntimeInit() { Sm::preEnableRuntimeInit(); }

        static void runtimeInit() { Sm::runtimeInit(); }

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

            // Clear the stall flag before the transfer starts, never after: handler() takes
            // the flag as the end-of-frame marker, so a clear that lands after the first
            // data is on its way could wipe a stall belonging to this frame.
            Sm::clearTxStall();

            Dma::template start<DmaChannel,
                                DmaPriority,
                                Sm::template txDmaTrigger<Dma>(),
                                Dma::TransferSize::_8,
                                false,
                                true>(Sm::txFifoAddress,
                                      reinterpret_cast<std::uint32_t>(leds.data()),
                                      leds.size() * sizeof(RGB));

            running = true;
            return true;
        }

        static bool ready() {
            if(running) { return false; }
            return Clock::now() >= whenRdy || whenRdy == typename Clock::time_point{};
        }

        static void handler() {
            if(Sm::txStalled() && running) {
                running = false;
                whenRdy = Clock::now() + Config::ResetTime;
            }
        }

        // once per main-loop turn: Startup::run<Kvasir::Hook::MainLoop>() calls it (StartUp/Hooks.hpp);
        // a firmware that runs the hook must not also call handler() by hand
        using Extends = Kvasir::Startup::Extend<Kvasir::Hook::MainLoop, &WS2812::handler>;
    };
}}   // namespace Kvasir::Pio
