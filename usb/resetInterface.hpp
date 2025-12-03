#pragma once

#include "descriptors.hpp"
#include "mixins.hpp"

#include <cstddef>

namespace Kvasir::USB::ResetInterface {

template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber>
struct Mixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;   // Grants access to helper functions

    static constexpr std::size_t InterfaceCount = 1;
    static constexpr std::size_t EndpointCount  = 0;

    static constexpr auto InterfaceDescriptor
      = Kvasir::USB::Descriptors::Interface{.bInterfaceNumber   = FirstInterfaceNumber,
                                            .bAlternateSetting  = 0,
                                            .bNumEndpoints      = 0,
                                            .bInterfaceClass    = 255,
                                            .bInterfaceSubClass = 0,
                                            .bInterfaceProtocol = 1};

    // Callbacks
    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        if(pkt.wIndex != FirstInterfaceNumber
           || pkt.recipient() != SetupPacket::Recipient::interface)
        {
            return false;
        }

        static constexpr SetupPacket::Request REQUEST_BOOTSEL{0x01};
        static constexpr SetupPacket::Request REQUEST_FLASH{0x02};

        if(pkt.bRequest == REQUEST_BOOTSEL) {
            UC_LOG_I("USB: Rebooting to BOOTSEL mode");
            Kvasir::resetToUsbBoot();
            return true;
        } else if(pkt.bRequest == REQUEST_FLASH) {
            UC_LOG_I("USB: Rebooting to flash");
            apply(Kvasir::SystemControl::SystemReset{});
            return true;
        }
        return false;
    }

    static bool EndpointHandlerCallback(std::size_t,
                                        bool) {
        return false;
    }

    static void ResetCallback() {}

    static void ConfiguredCallback() {}

    static void SetupEndpointsCallback() {}
};
}   // namespace Kvasir::USB::ResetInterface
