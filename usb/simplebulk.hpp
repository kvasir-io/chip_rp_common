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

    // Device-to-host only interface. Protocol 2 tells a host apart from the command
    // interface (protocol 0) and the picotool reset interface (protocol 1) without
    // relying on interface order.
    template<std::uint8_t InterfaceID,
             std::uint8_t DataEndpointID>
    consteval auto makeInInterfaceDescriptorArrays() {
        constexpr USB::Descriptors::Interface InterfaceDescriptor{.bInterfaceNumber{InterfaceID},
                                                                  .bAlternateSetting{0},
                                                                  .bNumEndpoints{1},
                                                                  .bInterfaceClass{255},
                                                                  .bInterfaceSubClass{0},
                                                                  .bInterfaceProtocol{2}};

        constexpr USB::Descriptors::Endpoint DataInEndpointDescriptor{
          .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, DataEndpointID)},
          .bmAttributes{EndpointTransferType::Bulk},
          .wMaxPacketSize{detail::MaxPacketSize}};

        return USB::Descriptors::detail::generateArray(InterfaceDescriptor,
                                                       DataInEndpointDescriptor);
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
    // A vendor bulk interface has no notion of a host "opening" it the way CDC-ACM has
    // DTR. Connected means the bus is configured: that is when the host may talk, and
    // a bus reset or unplug is the only disconnect there is. Whether an application on
    // the host is alive is the protocol's business (a heartbeat), not the transport's.
    static bool isConnected() { return Derived::isConfigured(); }

    // Expose SendRecvAdapter Public API
    static bool isSendReady() { return DataEndpointHandler::isSendReady() && isConnected(); }

    static auto& getRecvBuffer() { return DataEndpointHandler::getRecvBuffer(); }

    static bool send(std::span<std::byte const> data) { return DataEndpointHandler::send(data); }

    static void send_nocopy(std::span<std::byte const> data) {
        DataEndpointHandler::send_nocopy(data);
    }
};

// IN-only bulk interface: a one-way stream from the device (logs, events). Its public
// names differ from Mixin's on purpose -- both end up as sibling public bases of the
// device type, and identical names would make Derived::send / getRecvBuffer ambiguous.
template<typename Clock,
         typename Config,
         typename Derived,
         std::size_t FirstInterfaceNumber,
         std::size_t FirstEndpointNumber,
         template<typename, typename, typename, typename, std::size_t> class SendImpl
         = Kvasir::USB::detail::SendOnlyAdapter>
struct InMixin {
private:
    friend Derived;
    friend struct Kvasir::USB::detail::MixinTraits;

    using Self
      = InMixin<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber, SendImpl>;

    static constexpr std::size_t DataEndpointNumber = FirstEndpointNumber;
    static constexpr std::size_t InterfaceCount     = 1;
    static constexpr std::size_t EndpointCount      = 1;

    static constexpr auto InterfaceDescriptor
      = USB::SimpleBulk::Descriptors::makeInInterfaceDescriptorArrays<FirstInterfaceNumber,
                                                                      DataEndpointNumber>();

    using DataEndpointHandler = SendImpl<Clock, Config, Derived, Self, DataEndpointNumber>;

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
    static constexpr std::size_t StreamSendBufferSize = DataEndpointHandler::SendBufferSize;

    static bool isStreamSendReady() {
        return DataEndpointHandler::isSendReady() && Derived::isConfigured();
    }

    // Copies; the transfer runs from the adapter's own buffer. False when a previous
    // transfer is still in flight or the data does not fit.
    static bool sendStream(std::span<std::byte const> data) {
        if(!Derived::isConfigured()) { return false; }
        return DataEndpointHandler::send(data);
    }

    static bool sendStream_nocopy(std::span<std::byte const> data) {
        if(!Derived::isConfigured()) { return false; }
        return DataEndpointHandler::send_nocopy(data);
    }
};

}   // namespace Kvasir::USB::SimpleBulk
