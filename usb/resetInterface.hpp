#pragma once

// The picotool reset interface (kvasir/Devices/USB/VendorReset.hpp) on an RP2040 / RP2350.

#include "../bootrom_functions.hpp"

#include <cstddef>
#include <kvasir/Devices/USB/VendorReset.hpp>

namespace Kvasir::USB::Rp {

struct ResetActions {
    [[noreturn]] static void bootloader() { Kvasir::Bootrom::resetToUsbBoot(); }

    // A chip reboot (pico-sdk's reset interface does the same through watchdog_reboot):
    // SYSRESETREQ would only warm-reset this core, see reboot().
    [[noreturn]] static void reboot() { Kvasir::Bootrom::reboot(); }
};

// In a device's mixin list as it is; with callbacks, through an alias of the application's:
//
//     template<typename C, typename Cfg, typename D, std::size_t I, std::size_t E>
//     using ResetInterface = Kvasir::USB::Rp::ResetInterfaceWith<C, Cfg, D, I, E, Drain, Drain>;
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         typename BeforeBootselCallback = void,
         typename BeforeFlashCallback   = void>
using ResetInterfaceWith = Kvasir::USB::VendorReset::Mixin<Clock,
                                                           Config,
                                                           Derived,
                                                           FirstInterfaceNumber,
                                                           FirstEndpointNumber,
                                                           ResetActions,
                                                           BeforeBootselCallback,
                                                           BeforeFlashCallback>;

template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber>
using ResetInterface
  = ResetInterfaceWith<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber>;
}   // namespace Kvasir::USB::Rp
