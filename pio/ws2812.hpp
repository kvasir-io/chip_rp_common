#pragma once
#include "chip/rp_common/PIO.hpp"
#include "ws2812Pio/ws2812.hpp"

#include <chrono>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Pio {
    /// Drives a chain of WS2812-style LEDs from one PIO state machine, fed by DMA.
    ///
    /// Config:
    ///   ClockSpeed     (required) system clock feeding the PIO
    ///   PioInstance    (required) 0 or 1
    ///   SmInstance     (required) 0..3
    ///   LedClockSpeed  (default 800000) bit rate on the wire
    ///   ProgrammOffset (default 0)      where the program is loaded in instruction memory
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
        struct Config : Config_ {
            static constexpr auto LedClockSpeed = [] {
                if constexpr(requires { Config_::LedClockSpeed; }) {
                    return Config_::LedClockSpeed;
                } else {
                    return 800000;
                }
            }();
            static constexpr auto ProgrammOffset = [] {
                if constexpr(requires { Config_::ProgrammOffset; }) {
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
        };

        using Programm = Pio::ws2812Programm;

        static constexpr double DivFactor{
          double{Config::ClockSpeed}
          / (double{Config::LedClockSpeed} * double{Programm::CylcesPerBit})};

        // Kvasir::Pio::getDiv narrows div_int to std::uint8_t without complaining, so a
        // divider that does not fit would silently alias to a wrong SM clock and put every
        // pulse width out of spec.
        static_assert(DivFactor >= 1.0 && DivFactor < 256.0,
                      "LedClockSpeed unreachable from ClockSpeed with an 8-bit PIO divider");

        // The four pulse widths the program produces, in nanoseconds. The SM runs at
        // LedClockSpeed * CylcesPerBit, so LedClockSpeed scales all four *proportionally* --
        // which is why it cannot on its own reach a part that wants different ratios, such as
        // WS2811 in its 400kHz mode. The asserts below are what says so at compile time
        // instead of leaving it to be discovered on a strip.
        static constexpr double CycleTimeNs{
          1.0e9 / (double{Config::LedClockSpeed} * double{Programm::CylcesPerBit})};

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

        using PioRegs = Kvasir::Peripheral::PIO::Registers<Config::PioInstance>;
        using SmRegs  = typename PioRegs::template SM<Config::SmInstance>;

        static_assert(Config::SmInstance < 4,
                      "a PIO instance has four state machines");

        static constexpr std::uint32_t SmMask = 1U << Config::SmInstance;

        static constexpr auto powerClockEnable
          = list(Kvasir::Pio::getEnable<Config::PioInstance>());

        static constexpr auto initStepPinConfig
          = list(Kvasir::Pio::getPinConfig<Config::PioInstance>(Pin{}));

        static constexpr auto initStepPeripheryConfig
          = list(Kvasir::Pio::getDivConfig<SmRegs>([]() { return DivFactor; }),

                 SmRegs::EXECCTRL::overrideDefaults(
                   write(SmRegs::EXECCTRL::wrap_bottom,
                         Kvasir::Register::value<Programm::WrapTarget + Config::ProgrammOffset>()),
                   write(SmRegs::EXECCTRL::wrap_top,
                         Kvasir::Register::value<Programm::Wrap + Config::ProgrammOffset>())),

                 clear(SmRegs::SHIFTCTRL::fjoin_rx),
                 write(SmRegs::SHIFTCTRL::fjoin_tx, Kvasir::Register::value<1>()),
                 write(SmRegs::SHIFTCTRL::pull_thresh, Kvasir::Register::value<8>()),
                 write(SmRegs::SHIFTCTRL::push_thresh, Kvasir::Register::value<0>()),
                 write(SmRegs::SHIFTCTRL::out_shiftdir, Kvasir::Register::value<0>()),
                 write(SmRegs::SHIFTCTRL::in_shiftdir, Kvasir::Register::value<1>()),
                 write(SmRegs::SHIFTCTRL::autopull, Kvasir::Register::value<1>()),
                 write(SmRegs::SHIFTCTRL::autopush, Kvasir::Register::value<0>()));

        static void preEnableRuntimeInit() {
            static_assert(32 >= Config::ProgrammOffset + Programm::Instructions.size(),
                          "to many Instructions");
            for(std::uint16_t volatile* addr = reinterpret_cast<std::uint16_t volatile*>(
                  PioRegs::template INSTR_MEM<Config::ProgrammOffset>::Addr::value);
                auto v : Programm::Instructions)
            {
                *addr = v;
                ++addr;
                ++addr;
            }
            static constexpr auto PinNumber
              = []<int Port, int PinN>(Kvasir::Register::PinLocation<Port, PinN>) {
                    return PinN;
                }(Pin{});

            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::set_base, Kvasir::Register::value<PinNumber>()),
              write(SmRegs::PINCTRL::set_count, Kvasir::Register::value<1>())));

            apply(write(SmRegs::INSTR::instr, Kvasir::Register::value<0xe000 | (4 << 5) | 0x1f>()));

            apply(SmRegs::PINCTRL::overrideDefaults(
              write(SmRegs::PINCTRL::sideset_count, Kvasir::Register::value<1>()),
              write(SmRegs::PINCTRL::set_count, Kvasir::Register::value<0>()),
              write(SmRegs::PINCTRL::sideset_base, Kvasir::Register::value<PinNumber>())));
        }

        // sm_restart and clkdiv_restart are self-clearing one-shot fields, so writing a
        // mask that names only this SM restarts only this SM. sm_enable is *not*: it is a
        // latched 4-bit field, and writing it here would clear whichever sibling SMs the
        // other peripherals on this PIO instance had just enabled. It is therefore done as
        // a read-modify-write in runtimeInit(), which the startup sequence calls after
        // every peripheral's initStepPeripheryEnable has been applied.
        static constexpr auto initStepPeripheryEnable
          = list(write(PioRegs::CTRL::sm_restart, Kvasir::Register::value<SmMask>()),
                 write(PioRegs::CTRL::clkdiv_restart, Kvasir::Register::value<SmMask>()),
                 //JUMP to programm
                 write(SmRegs::INSTR::instr, Kvasir::Register::value<Config::ProgrammOffset>()));

        static void runtimeInit() {
            auto const enabled = get<0>(apply(read(PioRegs::CTRL::sm_enable)));
            apply(write(PioRegs::CTRL::sm_enable, enabled | SmMask));
        }

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
            apply(write(PioRegs::FDEBUG::txstall, Kvasir::Register::value<SmMask>()));

            Dma::template start<
              DmaChannel,
              DmaPriority,
              Kvasir::Pio::getTxDmaTrigger<Dma, Config::PioInstance, Config::SmInstance>(),
              Dma::TransferSize::_8,
              false,
              true>(PioRegs::template FIFO<Config::SmInstance>::TXF::Addr::value,
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
            bool const stall = get<0>(apply(read(PioRegs::FDEBUG::txstall))) & SmMask;
            if(stall && running) {
                running = false;
                whenRdy = Clock::now() + Config::ResetTime;
            }
        }
    };
}}   // namespace Kvasir::Pio
