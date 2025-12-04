#pragma once

#include "adapter.hpp"
#include "descriptors.hpp"
#include "mixins.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir::USB::SimpleBulk {
namespace Descriptors {
    template<std::uint16_t DeviceVersion,
             std::uint16_t VendorID,
             std::uint16_t ProductID,
             std::uint8_t  ManufacturerStringID,
             std::uint8_t  ProductStringID,
             std::uint8_t  SerialNumberStringID>
    consteval auto makeDeviceDescriptorArray() {
        return USB::Descriptors::makeDeviceDescriptorArray<DeviceVersion,
                                                           VendorID,
                                                           ProductID,
                                                           ManufacturerStringID,
                                                           ProductStringID,
                                                           SerialNumberStringID,
                                                           DeviceClass::Miscellaneous,
                                                           DeviceClass::Miscellaneous>();
    }

    template<std::uint8_t InterfaceID,
             std::uint8_t DataEndpointID>
    consteval auto makeInterfaceDescriptorArrays() {
        constexpr USB::Descriptors::Interface InterfaceDescriptor{.bInterfaceNumber{InterfaceID},
                                                                  .bAlternateSetting{0},
                                                                  .bNumEndpoints{2},
                                                                  .bInterfaceClass{255},
                                                                  .bInterfaceSubClass{0},
                                                                  .bInterfaceProtocol{0}};

        constexpr USB::Descriptors::Endpoint DataInEndpointDescriptor{
          .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, DataEndpointID)},
          .bmAttributes{EndpointTransferType::Bulk},
          .wMaxPacketSize{detail::MaxPacketSize}};

        constexpr USB::Descriptors::Endpoint DataOutEndpointDescriptor{
          .bEndpointAddress{makeEndpointAddress(EndpointDirection::Out, DataEndpointID)},
          .bmAttributes{EndpointTransferType::Bulk},
          .wMaxPacketSize{detail::MaxPacketSize}};

        constexpr auto Interface
          = USB::Descriptors::detail::generateArray(InterfaceDescriptor,
                                                    DataOutEndpointDescriptor,
                                                    DataInEndpointDescriptor);

        return Interface;
    }
}   // namespace Descriptors

// Simple Bulk Mixin - provides simple bulk functionality
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         template<typename, typename, typename, typename, std::size_t> class SendRecvImpl
         = Kvasir::USB::detail::SendRecvAdapter>
struct Mixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;   // Grants access to helper functions

    using Self
      = Mixin<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber, SendRecvImpl>;

    static constexpr std::size_t DataEndpointNumber = FirstEndpointNumber;
    static constexpr std::size_t InterfaceCount     = 1;
    static constexpr std::size_t EndpointCount      = 1;

    static constexpr auto InterfaceDescriptor
      = USB::SimpleBulk::Descriptors::makeInterfaceDescriptorArrays<FirstInterfaceNumber,
                                                                    DataEndpointNumber>();

    using DataEndpointHandler = SendRecvImpl<Clock, Config, Derived, Self, DataEndpointNumber>;

    // Callbacks
    static void SetupEndpointsCallback() { DataEndpointHandler::SetupEndpointsCallback(); }

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        return DataEndpointHandler::SetupPacketRequestCallback(pkt);
    }

    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        return DataEndpointHandler::EndpointHandlerCallback(epNum, in);
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        return DataEndpointHandler::AbortDoneCallback(epNum, in);
    }

    static void ResetCallback() { DataEndpointHandler::ResetCallback(); }

    static void ConfiguredCallback(std::uint8_t configuration) {
        DataEndpointHandler::ConfiguredCallback(configuration);
    }

public:
    // Expose SendRecvAdapter Public API
    static bool isSendReady() { return DataEndpointHandler::isSendReady(); }

    static auto& getRecvBuffer() { return DataEndpointHandler::getRecvBuffer(); }

    static bool send(std::span<std::byte const> data) { return DataEndpointHandler::send(data); }

    static void send_nocopy(std::span<std::byte const> data) {
        DataEndpointHandler::send_nocopy(data);
    }
};

}   // namespace Kvasir::USB::SimpleBulk
