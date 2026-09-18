#pragma once

#include "adapter.hpp"
#include "descriptors.hpp"
#include "endpointOps.hpp"
#include "mixins.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <string_view>
#include <utility>

namespace Kvasir::USB::CDC {

// Class-specific requests (CDC PSTN subclass).
enum class Request : std::uint8_t {
    setLineCoding       = 0x20,
    getLineCoding       = 0x21,
    setControlLineState = 0x22,
};

struct [[gnu::packed]] LineCoding {
    std::uint32_t dwDTERate{9600};
    std::uint8_t  bCharFormat{};
    std::uint8_t  bParityType{};
    std::uint8_t  bDataBits{8};
};

namespace Descriptors {
    struct [[gnu::packed]] Header
      : detail::InterfaceDescriptorBase<Header, DescriptorSubType::CDC_Header> {
        std::uint16_t bcdCDC{0x0120};
    };

    struct [[gnu::packed]] Union
      : detail::InterfaceDescriptorBase<Union, DescriptorSubType::CDC_Union> {
        std::uint8_t bMasterInterface0;
        std::uint8_t bSlaveInterface0;
    };

    struct [[gnu::packed]] CallManagement
      : detail::InterfaceDescriptorBase<CallManagement, DescriptorSubType::CDC_CallManagement> {
        std::uint8_t bmCapabilities;
        std::uint8_t bDataInterface;
    };
}   // namespace Descriptors

namespace ACM {
    namespace Descriptors {
        struct [[gnu::packed]] Descriptor
          : detail::InterfaceDescriptorBase<Descriptor, DescriptorSubType::CDC_ACM_Descriptor> {
            // SET_LINE_CODING, GET_LINE_CODING and SET_CONTROL_LINE_STATE are supported
            std::uint8_t bmCapabilities{0x02};
        };

        template<std::uint8_t ManagementInterfaceId,
                 std::uint8_t DataInterfaceId,
                 std::uint8_t ManagementEndpointId,
                 std::uint8_t DataEndpointID>
        consteval auto makeInterfaceDescriptorArrays() {
            constexpr USB::Descriptors::InterfaceAssociation CDC_ACM_InterfaceAssociation{
              .bFirstInterface{ManagementInterfaceId},
              .bInterfaceCount{2},
              .bFunctionClass{2},
              .bFunctionSubClass{2},
              .bFunctionProtocol{0}};

            constexpr USB::Descriptors::Interface CDC_ManagementInterfaceDescriptor{
              .bInterfaceNumber{ManagementInterfaceId},
              .bAlternateSetting{0},
              .bNumEndpoints{1},
              .bInterfaceClass{std::to_underlying(DeviceClass::Communication)},
              .bInterfaceSubClass{2},
              .bInterfaceProtocol{0}};

            constexpr USB::Descriptors::Interface ACM_InterfaceDescriptor{
              .bInterfaceNumber{DataInterfaceId},
              .bAlternateSetting{0},
              .bNumEndpoints{2},
              .bInterfaceClass{std::to_underlying(DeviceClass::CDC_Data)},
              .bInterfaceSubClass{0},
              .bInterfaceProtocol{0}};

            constexpr USB::CDC::Descriptors::Header CDC_HeaderDescriptor{};

            constexpr USB::CDC::ACM::Descriptors::Descriptor CDC_ACM_Descriptor{};

            constexpr USB::CDC::Descriptors::Union CDC_UnionDescriptor{
              .bMasterInterface0{ManagementInterfaceId},
              .bSlaveInterface0{DataInterfaceId}};

            constexpr USB::CDC::Descriptors::CallManagement CDC_CallManagementDescriptor{
              .bmCapabilities{0x0},
              .bDataInterface{DataInterfaceId}};

            // CDC Management endpoint packet size
            static constexpr std::uint16_t ManagementEndpointPacketSize = 8;
            // Interrupt endpoint polling interval (in ms, max 255 to minimize polling)
            static constexpr std::uint8_t InterruptEndpointInterval = 255;

            constexpr USB::Descriptors::Endpoint CDC_ManagementEndpointDescriptor{
              .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, ManagementEndpointId)},
              .bmAttributes{EndpointTransferType::Interrupt},
              .wMaxPacketSize{ManagementEndpointPacketSize},
              .bInterval{InterruptEndpointInterval}};

            constexpr USB::Descriptors::Endpoint ACM_DataInEndpointDescriptor{
              .bEndpointAddress{makeEndpointAddress(EndpointDirection::In, DataEndpointID)},
              .bmAttributes{EndpointTransferType::Bulk},
              .wMaxPacketSize{detail::MaxPacketSize}};

            constexpr USB::Descriptors::Endpoint ACM_DataOutEndpointDescriptor{
              .bEndpointAddress{makeEndpointAddress(EndpointDirection::Out, DataEndpointID)},
              .bmAttributes{EndpointTransferType::Bulk},
              .wMaxPacketSize{detail::MaxPacketSize}};

            constexpr auto CDC_Interface
              = USB::Descriptors::detail::generateArray(CDC_ManagementInterfaceDescriptor,
                                                        CDC_HeaderDescriptor,
                                                        CDC_CallManagementDescriptor,
                                                        CDC_ACM_Descriptor,
                                                        CDC_UnionDescriptor,
                                                        CDC_ManagementEndpointDescriptor);
            constexpr auto ACM_Interface
              = USB::Descriptors::detail::generateArray(ACM_InterfaceDescriptor,
                                                        ACM_DataOutEndpointDescriptor,
                                                        ACM_DataInEndpointDescriptor);

            constexpr auto Interface
              = USB::Descriptors::detail::generateArray(CDC_ACM_InterfaceAssociation,
                                                        CDC_Interface,
                                                        ACM_Interface);

            return Interface;
        }
    }   // namespace Descriptors

    // CDC-ACM Mixin - provides CDC/ACM functionality
    template<typename Clock,
             typename Config,
             typename Derived,
             std::size_t FirstInterfaceNumber,
             std::size_t FirstEndpointNumber,
             typename Unused = void>
    struct Mixin {
    private:
        friend Derived;
        friend struct Kvasir::USB::detail::MixinTraits;   // Grants access to helper functions

        using Self
          = Mixin<Clock, Config, Derived, FirstInterfaceNumber, FirstEndpointNumber, Unused>;

        // CDC-ACM uses 2 interfaces (management + data)
        static constexpr std::size_t InterfaceCount = 2;
        static constexpr std::size_t EndpointCount  = 2;

        static constexpr std::size_t ManagementInterface      = FirstInterfaceNumber;
        static constexpr std::size_t DataInterface            = FirstInterfaceNumber + 1;
        static constexpr std::size_t ManagementEndpointNumber = FirstEndpointNumber;
        static constexpr std::size_t DataEndpointNumber       = FirstEndpointNumber + 1;

        // Interface descriptors for both CDC management and ACM data interfaces
        static constexpr auto InterfaceDescriptor
          = USB::CDC::ACM::Descriptors::makeInterfaceDescriptorArrays<ManagementInterface,
                                                                      DataInterface,
                                                                      ManagementEndpointNumber,
                                                                      DataEndpointNumber>();

        using ManagementEP        = detail::EndpointOps<Derived,
                                                        ManagementEndpointNumber,
                                                        EndpointDirection::In,
                                                        EndpointTransferType::Interrupt>;
        using DataEndpointHandler = Kvasir::USB::detail::
          BulkDataAdapter<Clock, Config, Derived, Self, DataEndpointNumber, true, true>;

        // The notification endpoint: set up, never sent on, but its halt state is the host's to set.
        struct Notification {
            static constexpr std::uint8_t Address
              = makeEndpointAddress(EndpointDirection::In,
                                    static_cast<std::uint8_t>(ManagementEndpointNumber));

            static inline bool halted{false};

            static void halt() {
                halted = true;
                ManagementEP::stall();
            }

            static void clearHalt() {
                halted = false;
                ManagementEP::clearStall();
                ManagementEP::resetPid();
            }
        };

        // Callbacks
        static void SetupEndpointsCallback() {
            ManagementEP::setupEndpoint();
            DataEndpointHandler::SetupEndpointsCallback();
        }

        static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
            using Direction = SetupPacket::Direction;
            using Recipient = SetupPacket::Recipient;
            using Type      = SetupPacket::Type;

            // A new request ends a SET_LINE_CODING whose data stage never came.
            lineCodingExpected = false;

            if(pkt.type() == Type::classT && pkt.recipient() == Recipient::interface) {
                if(pkt.wIndex == ManagementInterface) {
                    switch(static_cast<Request>(std::to_underlying(pkt.bRequest))) {
                    case Request::setControlLineState:
                        {
                            if(pkt.direction() == Direction::hostToDevice) {
                                using namespace std::string_view_literals;
                                Derived::acknowledgeSetupRequest();
                                connected = (pkt.wValue & 0x01) != 0;
                                UC_LOG_I("CDC-ACM: Control line state - DTR={}, RTS={}",
                                         (pkt.wValue & 0x01) ? "on"sv : "off"sv,
                                         (pkt.wValue & 0x02) ? "on"sv : "off"sv);
                                return true;
                            }
                        }
                        break;
                    case Request::getLineCoding:
                        {
                            if(pkt.direction() == Direction::deviceToHost) {
                                UC_LOG_I("CDC-ACM: Get line coding request");
                                return Derived::ep0INDataPhase(
                                  std::as_bytes(std::span{std::addressof(lineCoding), 1}),
                                  pkt.wLength);
                            }
                        }
                        break;
                    case Request::setLineCoding:
                        {
                            if(pkt.direction() == Direction::hostToDevice
                               && pkt.wLength == sizeof(USB::CDC::LineCoding))
                            {
                                lineCodingExpected = true;
                                Derived::ep0OUTDataPhase(sizeof(USB::CDC::LineCoding));
                                UC_LOG_I("CDC-ACM: Set line coding request");
                                return true;
                            }
                        }
                        break;
                    default: break;
                    }
                }
                return false;
            }
            return detail::handleEndpointRequest<Derived, Notification>(pkt)
                || detail::handleSetInterface<Derived>(pkt,
                                                       ManagementInterface,
                                                       ManagementEP::resetPid)
                || detail::handleSetInterface<Derived>(pkt,
                                                       DataInterface,
                                                       DataEndpointHandler::restart)
                || DataEndpointHandler::SetupPacketRequestCallback(pkt);
        }

        static bool EndpointHandlerCallback(std::size_t epNum,
                                            bool        in) {
            // Handle line coding data stage on EP0
            if(epNum == 0 && !in && lineCodingExpected) {
                std::array<std::byte, sizeof(lineCoding)> tempBuffer{};
                lineCodingExpected = false;
                if(!Derived::ep0OUTGetData(tempBuffer)) {
                    Derived::stallControlRequest();
                    return true;
                }
                std::memcpy(std::addressof(lineCoding), tempBuffer.data(), tempBuffer.size());
                Derived::acknowledgeSetupRequest();
                UC_LOG_I(
                  "CDC-ACM: Line coding set - {} baud, {} data bits, parity={}, stop "
                  "bits={}",
                  lineCoding.dwDTERate,
                  lineCoding.bDataBits,
                  lineCoding.bParityType,
                  lineCoding.bCharFormat);
                return true;
            }
            return DataEndpointHandler::EndpointHandlerCallback(epNum, in);
        }

        static bool AbortDoneCallback(std::size_t epNum,
                                      bool        in) {
            return DataEndpointHandler::AbortDoneCallback(epNum, in);
        }

        static void ResetCallback() {
            lineCodingExpected   = false;
            connected            = false;
            Notification::halted = false;
            ManagementEP::reset();
            DataEndpointHandler::ResetCallback();
        }

        static void ConfiguredCallback(std::uint8_t configuration) {
            if(configuration == 0) { connected = false; }
            ManagementEP::resetPid();
            DataEndpointHandler::ConfiguredCallback(configuration);
        }

        // State
        static inline std::atomic<bool>    connected          = false;
        static inline bool                 lineCodingExpected = false;
        static inline USB::CDC::LineCoding lineCoding{};

    public:
        // Public API
        static bool isConnected() { return connected; }

        // The data endpoint
        static bool isSendReady() { return DataEndpointHandler::isSendReady() && isConnected(); }

        static auto& getRecvBuffer() { return DataEndpointHandler::getRecvBuffer(); }

        static bool send(std::span<std::byte const> data) {
            return DataEndpointHandler::send(data);
        }

        static std::size_t writeAvailable() { return DataEndpointHandler::writeAvailable(); }
    };

}   // namespace ACM
}   // namespace Kvasir::USB::CDC
