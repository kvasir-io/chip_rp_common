#pragma once

// The RP2040 / RP2350 USB controller as a backend of kvasir_devices' USB device
// (kvasir/Devices/USB/Backend.hpp has the contract):
//
//     namespace HW {
//     template<typename Clock, typename Config>
//     using UsbBackend = Kvasir::USB::Rp::Backend<Clock, Config>;
//     }
//     using Usb = Kvasir::USB::CdcAcm<HW::UsbBackend, Clock, UsbConfig>;
//
// Besides what the device reads, the config struct may carry
//
//     isrPriority      1
//     DoubleBuffered   true: two packets armed per IN endpoint (endpointOps.hpp)

#include "../Clocks.hpp"
#include "detail.hpp"
#include "endpointOps.hpp"
#include "kvasir/Register/RegisterFmt.hpp"
#include "kvasir/Util/RateLimiter.hpp"

#include <bit>
#include <chip/chip.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <kvasir/Devices/USB/Backend.hpp>
#include <kvasir/Devices/USB/Config.hpp>
#include <kvasir/Devices/USB/Descriptors.hpp>
#include <memory>
#include <string_view>

namespace Kvasir::USB::Rp {
// Startup resource: the one USB controller (kvasir/StartUp/Resources.hpp).
struct InstanceTag {};

template<typename Clock, typename ConfigT>
struct Backend {
private:
    template<typename, std::size_t, EndpointDirection, EndpointTransferType>
    friend struct Kvasir::USB::detail::EndpointOps;

    using Regs       = Kvasir::Peripheral::USB::Registers<0>;
    using BufferRegs = Kvasir::Peripheral::USB_DPRAM::Registers<0>;
    using ClockType  = Clock;
    using Traits     = Kvasir::USB::ConfigTraits<ConfigT>;

    static constexpr auto isrPriority = [] {
        if constexpr(requires { ConfigT::isrPriority; }) {
            return ConfigT::isrPriority;
        } else {
            return 1;
        }
    }();
    static constexpr auto DoubleBuffered = [] {
        if constexpr(requires { ConfigT::DoubleBuffered; }) {
            return ConfigT::DoubleBuffered;
        } else {
            return true;
        }
    }();

    enum class Fault : std::uint8_t {
        epTxError = 1,
        epRxError,
        sieError,
        watchdog,
    };
    // Fault logging goes through this: a bad cable repeats these per packet, from inside the
    // ISR.
    static inline Kvasir::RateLimiter<Clock, Kvasir::RateLimiterConfig{.burst = 8}> faultLog_{};

    // RP2040/RP2350 DPSRAM buffer size
    static constexpr std::size_t DPSRAMSize = 4096;

    static constexpr auto InterruptIndexes = brigand::list<decltype(Kvasir::Interrupt::usbctrl)>{};

    static SetupPacket getSetupPacket() {
        SetupPacket ret;
        detail::device_memory_memcpy(
          std::addressof(ret),
          reinterpret_cast<void const*>(BufferRegs::SETUP_PACKET_LOW::Addr::value),
          sizeof(SetupPacket));
        return ret;
    }

    // IN transfer for an even bit index, OUT transfer for an odd one. Each bit is cleared before
    // its endpoint is told.
    template<typename Reg,
             typename F>
    static void forEachEndpointBit(F&& f) {
        std::uint32_t bits = apply(read(Reg::FULLREGISTER));
        while(bits) {
            auto const endpointBitIndex = static_cast<std::uint32_t>(std::countr_zero(bits));
            auto const bit              = 1U << endpointBitIndex;
            apply(write(Reg::FULLREGISTER, bit));
            f(endpointBitIndex >> 1U, (endpointBitIndex & 1U) == 0);
            bits &= ~bit;
        }
    }

    static constexpr auto getSofEnable() {
        if constexpr(Traits::UseSof) {
            return set(Regs::INTE::dev_sof);
        } else {
            return clear(Regs::INTE::dev_sof);
        }
    }

    static constexpr auto getWatchdogEnable() {
#if __has_include("chip/rp2350.hpp")
        return Kvasir::MPL::list(
          Regs::DEV_SM_WATCHDOG::overrideDefaults(
            clear(Regs::DEV_SM_WATCHDOG::enable),
            write(Regs::DEV_SM_WATCHDOG::limit, Kvasir::Register::value<1024 * 8>())),
          Kvasir::Register::SequencePoint{},
          Regs::DEV_SM_WATCHDOG::overrideDefaults(
            set(Regs::DEV_SM_WATCHDOG::enable),
            write(Regs::DEV_SM_WATCHDOG::limit, Kvasir::Register::value<1024 * 8>())));
#else
        return Kvasir::MPL::list();
#endif
    }

    static constexpr auto getIstEnable() {
        return Kvasir::MPL::list(
          Regs::INTE::overrideDefaults(set(Regs::INTE::buff_status),
                                       set(Regs::INTE::bus_reset),
                                       set(Regs::INTE::setup_req),
                                       set(Regs::INTE::abort_done),
                                       set(Regs::INTE::dev_suspend),
                                       set(Regs::INTE::dev_resume_from_host),
#if __has_include("chip/rp2350.hpp")
                                       set(Regs::INTE::dev_sm_watchdog_fired),
#endif
                                       getSofEnable()));
    }

public:
    static constexpr std::size_t MaxPacketSize = 64;
    // EP0 plus one DPRAM group per further endpoint number. EndpointOps always addresses
    // buffers through the DOUBLEBUFFER groups (2 x 2 x 64 bytes each), of which the buffer map
    // has 8, whether or not double buffering is enabled.
    static constexpr std::size_t EndpointCount = 8;
    // EP_ABORT is a request; the controller answers with the abort_done interrupt.
    static constexpr bool AsyncCancel = true;

    template<std::size_t N, EndpointDirection Dir, EndpointTransferType Type>
    using Endpoint = Kvasir::USB::detail::EndpointOps<Backend, N, Dir, Type>;

private:
    using EP0_IN  = Endpoint<0, EndpointDirection::In, EndpointTransferType::Control>;
    using EP0_OUT = Endpoint<0, EndpointDirection::Out, EndpointTransferType::Control>;

public:
    template<typename Sink>
    static void dispatchEvents() {
        static constexpr auto CommonIsrList
          = Kvasir::MPL::list(read(Regs::INTS::setup_req),
                              read(Regs::INTS::buff_status),
                              read(Regs::INTS::bus_reset),
#if __has_include("chip/rp2350.hpp")
                              read(Regs::INTS::dev_sm_watchdog_fired),
#endif
                              read(Regs::INTS::dev_suspend),
                              read(Regs::INTS::dev_resume_from_host),
                              read(Regs::INTS::abort_done));
        static constexpr auto IsrList = []() {
            if constexpr(Traits::UseSof) {
                return Kvasir::MPL::list(CommonIsrList, read(Regs::INTS::dev_sof));
            } else {
                return CommonIsrList;
            }
        }();

        auto const          status    = apply(IsrList);
        std::uint32_t const sieStatus = apply(read(Regs::SIE_STATUS::FULLREGISTER));

        constexpr std::uint32_t errorMask
          = Regs::SIE_STATUS::data_seq_error.Mask | Regs::SIE_STATUS::rx_timeout.Mask
          | Regs::SIE_STATUS::rx_overflow.Mask | Regs::SIE_STATUS::bit_stuff_error.Mask
          | Regs::SIE_STATUS::crc_error.Mask
#if __has_include("chip/rp2350.hpp")
          | Regs::SIE_STATUS::endpoint_error.Mask
#endif
          ;

#if __has_include("chip/rp2350.hpp")
        if(std::uint32_t const txError = apply(read(Regs::EP_TX_ERROR::FULLREGISTER))) {
            KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::epTxError, txError)),
                               UC_LOG_E,
                               "USB: EP_TX_ERROR: {}",
                               Kvasir::Register::Flags<typename Regs::EP_TX_ERROR>{txError});
            apply(write(Regs::EP_TX_ERROR::FULLREGISTER, Kvasir::Register::value<0xffffffff>()));
        }
        if(std::uint32_t const rxError = apply(read(Regs::EP_RX_ERROR::FULLREGISTER))) {
            KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::epRxError, rxError)),
                               UC_LOG_E,
                               "USB: EP_RX_ERROR: {}",
                               Kvasir::Register::Flags<typename Regs::EP_RX_ERROR>{rxError});
            apply(write(Regs::EP_RX_ERROR::FULLREGISTER, Kvasir::Register::value<0xffffffff>()));
        }
#endif
        if(sieStatus & errorMask) {
            KVASIR_LOG_LIMITED(
              faultLog_.allow(Kvasir::rateLimitKey(Fault::sieError, sieStatus & errorMask)),
              UC_LOG_E,
              "USB: SIE_STATUS error: {}",
              Kvasir::Register::Flags<typename Regs::SIE_STATUS>{sieStatus & errorMask});
            // Clear error bits (WC - write 1 to clear)
            apply(write(Regs::SIE_STATUS::FULLREGISTER, sieStatus & errorMask));
        }

        if constexpr(Traits::UseSof == true) {
            if(status[Regs::INTS::dev_sof]) {
                auto const sof = get<0>(apply(read(Regs::SOF_RD::count)));
                Sink::startOfFrame(static_cast<std::uint16_t>(sof));
            }
        }

        if(status[Regs::INTS::bus_reset]) {
            apply(Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::bus_reset)));
            Sink::busReset();
            return;
        }

        // DEV_SUSPEND: "set when the device suspend state changes. Cleared by writing to
        // SIE_STATUS.SUSPENDED"; DEV_RESUME_FROM_HOST: "cleared by writing to SIE_STATUS.RESUME"
        // (RP2040 datasheet 4.1.4, RP2350 datasheet 12.6.10: the INTR register). Which way it changed is what
        // SUSPENDED said when the interrupt was taken. Written through the whole register: the
        // RP2350's SVD has SUSPENDED as read-only.
        if(status[Regs::INTS::dev_suspend]) {
            apply(write(Regs::SIE_STATUS::FULLREGISTER, Regs::SIE_STATUS::suspended.Mask));
            if(sieStatus & Regs::SIE_STATUS::suspended.Mask) {
                Sink::suspend();
            } else {
                Sink::resume();
            }
        }
        if(status[Regs::INTS::dev_resume_from_host]) {
            apply(write(Regs::SIE_STATUS::FULLREGISTER, Regs::SIE_STATUS::resume.Mask));
            Sink::resume();
        }

#if __has_include("chip/rp2350.hpp")
        if(status[Regs::INTS::dev_sm_watchdog_fired]) {
            KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::watchdog)),
                               UC_LOG_E,
                               "USB: watchdog");
            apply(set(Regs::DEV_SM_WATCHDOG::fired));
        }
#endif

        if(status[Regs::INTS::abort_done]) {
            forEachEndpointBit<typename Regs::EP_ABORT_DONE>(
              [](std::size_t ep, bool in) { Sink::cancelComplete(ep, in); });
        }

        if(status[Regs::INTS::buff_status]) {
            forEachEndpointBit<typename Regs::BUFF_STATUS>(
              [](std::size_t ep, bool in) { Sink::transferComplete(ep, in); });
        }

        if(status[Regs::INTS::setup_req]) {
            apply(Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::setup_rec)));
            Sink::setup(getSetupPacket());
        }
    }

    static void setAddress(std::uint8_t address) {
        apply(write(Regs::EP<0>::ADDR_ENDP::address, address));
    }

    // Whatever EP0 has armed is dropped, and both directions go on with DATA1.
    //
    // A buffer the controller already holds is not taken back by rewriting its buffer control:
    // "EP_ABORT ... can be set to ignore the buffer control register for this endpoint in case you
    // would like to revoke a buffer ... a corresponding bit in EP_ABORT_DONE is set when it is safe
    // to modify the buffer control register" (RP2040 datasheet 4.1.4, the same in the RP2350's).
    // Without it a host that abandons a control read after its first packet gets the second one
    // as the answer to its next request: the old packet is still the controller's when that
    // request's IN token comes, whatever the register says by then (seen on an RP2040 as a babble
    // or a timeout on the host, usb_playground RESULTS.md). The wait is for the controller to
    // say the endpoint is idle, which it is: it has just taken the SETUP. Bounded all the same.
    // (EP_ABORT needs an RP2040 B2 or later, erratum RP2040-E2 - as the bulk endpoints' abort does.)
    static void beginControlTransfer() {
        std::uint32_t armed = 0;
        if(EP0_IN::armedBuffers() != 0) { armed |= 1U << 0U; }
        if(EP0_OUT::armedBuffers() != 0) { armed |= 1U << 1U; }
        if(armed != 0) {
            std::uint32_t const aborts = apply(read(Regs::EP_ABORT::FULLREGISTER));
            apply(write(Regs::EP_ABORT::FULLREGISTER, aborts | armed));
            for(std::uint32_t i = 0; i != 100'000U; ++i) {
                std::uint32_t const done = apply(read(Regs::EP_ABORT_DONE::FULLREGISTER));
                if((done & armed) == armed) { break; }
            }
        }
        EP0_IN::cancelTransfer();
        EP0_OUT::cancelTransfer();
        if(armed != 0) {
            std::uint32_t const aborts = apply(read(Regs::EP_ABORT::FULLREGISTER));
            apply(write(Regs::EP_ABORT::FULLREGISTER, aborts & ~armed));
            // Acknowledged here: nobody waits for an abort_done of endpoint 0.
            apply(write(Regs::EP_ABORT_DONE::FULLREGISTER, armed));
        }
        EP0_IN::state.setDataPhase();
        EP0_OUT::state.setDataPhase();
    }

    static void maskInterrupt() { apply(Kvasir::Nvic::makeDisable(InterruptIndexes)); }

    static void unmaskInterrupt() { apply(Kvasir::Nvic::makeEnable(InterruptIndexes)); }

    static void prepare() {
        //can use std::memset here since it should always make alligned access.
        std::memset(reinterpret_cast<void*>(BufferRegs::baseAddr), 0, DPSRAMSize);
    }

    static void connect() {
        apply(Kvasir::Nvic::makeEnable(InterruptIndexes));

        apply(Regs::SIE_CTRL::overrideDefaults(
          set(Regs::SIE_CTRL::pullup_en),
          set(Regs::SIE_CTRL::ep0_int_1buf),
          clear(Regs::SIE_CTRL::pulldown_en),
          // EP0 single buffered, see EndpointOps::DoubleBuffered.
          write(Regs::SIE_CTRL::ep0_double_buf, Kvasir::Register::value<0>())));
    }

    /// What this controller can check about itself, one log line each, for the first contact
    /// with a new board: true = nothing here explains a device that does not enumerate. Call it
    /// from main(), after Startup. (Data sheets, register descriptions: "RESETS: RESET_DONE",
    /// "CLOCKS: CLK_USB_CTRL", "PLL: CS", "USB: MAIN_CTRL", "USB: SIE_CTRL", "USB: SIE_STATUS".)
    static bool selfTest() {
        using Resets     = Kvasir::Peripheral::RESETS::Registers<>;
        using ClkUsb     = Kvasir::Peripheral::CLOCKS::Registers<>::CLK_USB_CTRL;
        using PllUsb     = Kvasir::Peripheral::PLL::Registers<1>;
        bool       ok    = true;
        auto const check = [&ok](bool pass, std::string_view what, std::string_view hint) {
            if(pass) {
                UC_LOG_I("USB self-test: ok   {}", what);
            } else {
                UC_LOG_E("USB self-test: FAIL {} - {}", what, hint);
                ok = false;
            }
        };
        std::uint32_t const resetDone = apply(read(Resets::RESET_DONE::usbctrl));
        check(resetDone != 0, "the controller is out of reset", "RESETS.RESET.usbctrl still set");
        std::uint32_t const clkEnabled = apply(read(ClkUsb::enable));
        check(clkEnabled != 0, "clk_usb is enabled", "the clock settings never started clk_usb");
        std::uint32_t const pllLock = apply(read(PllUsb::CS::lock));
        check(pllLock != 0, "pll_usb is locked", "fine only if clk_usb comes from another source");
        if(resetDone == 0) { return false; }   // its registers do not answer while in reset
        std::uint32_t const controllerEn = apply(read(Regs::MAIN_CTRL::controller_en));
        check(controllerEn != 0, "the controller is enabled", "Usb is not in the Startup list?");
        std::uint32_t const pullup = apply(read(Regs::SIE_CTRL::pullup_en));
        check(pullup != 0, "the D+ pull-up is on", "connect() did not run, or disconnect() did");
        std::uint32_t const vbus = apply(read(Regs::SIE_STATUS::vbus_detected));
        check(vbus != 0, "VBUS is seen", "no cable, or VBUS detection without its override");
        std::uint32_t const connected = apply(read(Regs::SIE_STATUS::connected));
        std::uint32_t const suspended = apply(read(Regs::SIE_STATUS::suspended));
        UC_LOG_I("USB self-test: bus connected={} suspended={} (a host that enumerates shows 1/0)",
                 connected,
                 suspended);
        return ok;
    }

    // Off the bus: the pull-up released. connect() puts it back.
    static void disconnect() { apply(clear(Regs::SIE_CTRL::pullup_en)); }

    //Kvasir Callbacks
    template<auto Handler>
    using Isr = brigand::list<
      Kvasir::Nvic::Isr<Handler, Kvasir::Nvic::Index<decltype(Kvasir::Interrupt::usbctrl)::value>>>;

    using Provides = brigand::list<Kvasir::Startup::Resource<InstanceTag, 0>>;
    // The PHY runs from clk_usb at 48 MHz; a ClockSettings that programs it otherwise is
    // caught here (optional tag: unchecked until the settings declare their clocks).
    using Claims = Clocks::Claim<Clocks::ClkUsb, 48'000'000>;

    static constexpr auto powerClockEnable
      = list(clear(Kvasir::Peripheral::RESETS::Registers<>::RESET::usbctrl));

    static constexpr auto initStepPeripheryConfig
      = list(Regs::MUXING::overrideDefaults(set(Regs::MUXING::to_phy)),
             Regs::PWR::overrideDefaults(set(Regs::PWR::vbus_detect),
                                         set(Regs::PWR::vbus_detect_override_en)),
             Regs::MAIN_CTRL::overrideDefaults(set(Regs::MAIN_CTRL::controller_en),
                                               clear(Regs::MAIN_CTRL::phy_iso)),
             getIstEnable(),
             getWatchdogEnable());

    static constexpr auto initStepInterruptConfig
      = list(Kvasir::Nvic::makeSetPriority<isrPriority>(InterruptIndexes),
             Kvasir::Nvic::makeClearPending(InterruptIndexes));
};
}   // namespace Kvasir::USB::Rp
