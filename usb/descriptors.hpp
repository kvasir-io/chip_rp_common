#pragma once

#include <array>
#include <cassert>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <kvasir/Util/StaticVector.hpp>
#include <kvasir/Util/using_literals.hpp>
#include <memory>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>

namespace Kvasir::USB {

enum class DescriptorType : std::uint8_t {
    device                    = 1,
    configuration             = 2,
    string                    = 3,
    interface                 = 4,
    endpoint                  = 5,
    deviceQualifier           = 6,
    otherSpeedConfiguration   = 7,
    interfacePower            = 8,
    otg                       = 9,
    debug                     = 10,
    interfaceAssociation      = 11,
    security                  = 12,
    key                       = 13,
    encryptionType            = 14,
    bos                       = 15,
    deviceCapability          = 16,
    wirelessEndpointCompanion = 17,
    csInterface               = 36
};

enum class DescriptorSubType : std::uint8_t {
    CDC_Header         = 0x00,
    CDC_CallManagement = 0x01,
    CDC_Union          = 0x06,
    CDC_ACM_Descriptor = 0x02
};

enum class DeviceClass : std::uint8_t {
    Communication = 0x02,
    CDC_Data      = 0x0A,
    Miscellaneous = 0xEF,
};

enum class ConfigurationAttributes : std::uint8_t {
    RemoteWakeup = 0x20,
    SelfPowered  = 0x40,
    BusPowered   = 0x80
};

enum class EndpointTransferType : std::uint8_t {
    Control     = 0x0,
    Isochronous = 0x1,
    Bulk        = 0x2,
    Interrupt   = 0x3,
};

enum class EndpointDirection : std::uint8_t { Out = 0x00, In = 0x80 };

// Control transfer stages (USB 2.0 Specification Chapter 5.5.5)
enum class ControlStage : std::uint8_t {
    Idle = 0,
    Setup,
    Data,
    Status,
    Stall,
};

constexpr std::uint8_t makeEndpointAddress(EndpointDirection direction,
                                           std::uint8_t      endpointID) {
    return static_cast<std::uint8_t>(direction) | endpointID;
}

struct SetupPacket {
    enum class Direction : std::uint8_t { hostToDevice, deviceToHost };

    enum class Type : std::uint8_t { standard, classT, vendor, reserved };

    enum class Recipient : std::uint8_t { device, interface, endpoint, other };

    enum class Request : std::uint8_t {
        getStatus           = 0,
        clearFeature        = 1,
        setFeature          = 3,
        setAddress          = 5,
        getDescriptor       = 6,
        setDescriptor       = 7,
        getConfiguration    = 8,
        setConfiguration    = 9,
        getInterface        = 10,
        setInterface        = 11,
        setLineCoding       = 0x20,
        getLineCoding       = 0x21,
        setControlLineState = 0x22
    };

    Direction direction() const { return static_cast<Direction>((bmRequestType & 0x80) >> 7); }

    Type type() const { return static_cast<Type>((bmRequestType & 0x60) >> 5); }

    Recipient recipient() const { return static_cast<Recipient>(bmRequestType & 0x1f); }

    DescriptorType descriptorType() const { return static_cast<DescriptorType>(wValue >> 8); }

    uint8_t  bmRequestType;
    Request  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

namespace detail {
    // USB Full-Speed maximum packet size (64 bytes)
    static constexpr std::size_t MaxPacketSize = 64;
    // Maximum number of USB endpoints (RP2040/RP2350 supports 16 endpoints)
    static constexpr std::size_t MaxEndpoints = 16;

    static constexpr std::uint16_t bcdUSB{0x0200};

    template<typename Derived, DescriptorType DT>
    struct DescriptorBase {
        std::uint8_t   bLength{sizeof(Derived)};
        DescriptorType bDescriptorType{DT};
    };

    template<typename Derived, DescriptorSubType DST>
    struct InterfaceDescriptorBase : DescriptorBase<Derived, DescriptorType::csInterface> {
        DescriptorSubType bDescriptorSubType{DST};
    };
}   // namespace detail

namespace Descriptors {

    struct [[gnu::packed]] Device : detail::DescriptorBase<Device, DescriptorType::device> {
        std::uint16_t bcdUSB{detail::bcdUSB};
        DeviceClass   bDeviceClass;
        DeviceClass   bDeviceSubClass;
        std::uint8_t  bDeviceProtocol;
        std::uint8_t  bMaxPacketSize0;
        std::uint16_t idVendor;
        std::uint16_t idProduct;
        std::uint16_t bcdDevice;
        std::uint8_t  iManufacturer;
        std::uint8_t  iProduct;
        std::uint8_t  iSerialNumber;
        std::uint8_t  bNumConfigurations;
    };

    struct [[gnu::packed]] Configuration
      : detail::DescriptorBase<Configuration, DescriptorType::configuration> {
        std::uint16_t           wTotalLength;
        std::uint8_t            bNumInterfaces;
        std::uint8_t            bConfigurationValue;
        std::uint8_t            iConfiguration{};
        ConfigurationAttributes bmAttributes{ConfigurationAttributes::BusPowered};
        std::uint8_t            bMaxPower;
    };

    struct [[gnu::packed]] Interface
      : detail::DescriptorBase<Interface, DescriptorType::interface> {
        std::uint8_t bInterfaceNumber;
        std::uint8_t bAlternateSetting;
        std::uint8_t bNumEndpoints;
        std::uint8_t bInterfaceClass;
        std::uint8_t bInterfaceSubClass;
        std::uint8_t bInterfaceProtocol;
        std::uint8_t iInterface{};
    };

    struct [[gnu::packed]] InterfaceAssociation
      : detail::DescriptorBase<InterfaceAssociation, DescriptorType::interfaceAssociation> {
        std::uint8_t bFirstInterface;
        std::uint8_t bInterfaceCount;
        std::uint8_t bFunctionClass;
        std::uint8_t bFunctionSubClass;
        std::uint8_t bFunctionProtocol;
        std::uint8_t iFunction{};
    };

    struct [[gnu::packed]] Endpoint : detail::DescriptorBase<Endpoint, DescriptorType::endpoint> {
        std::uint8_t         bEndpointAddress;
        EndpointTransferType bmAttributes;
        std::uint16_t        wMaxPacketSize;
        std::uint8_t         bInterval{};
    };

    struct [[gnu::packed]] DeviceQualifier
      : detail::DescriptorBase<DeviceQualifier, DescriptorType::deviceQualifier> {
        std::uint16_t bcdUSB{detail::bcdUSB};
        DeviceClass   bDeviceClass;
        DeviceClass   bDeviceSubClass;
        std::uint8_t  bDeviceProtocol;
        std::uint8_t  bMaxPacketSize0;
        std::uint8_t  bNumConfigurations;
        std::uint8_t  bReserved{};
    };

    namespace detail {
        template<typename T>
        constexpr std::size_t getActualSize() {
            // For std::array, use tuple_size to get the actual element count
            if constexpr(requires { std::tuple_size<T>::value; }) {
                return std::tuple_size<T>::value;
            } else {
                return sizeof(T);
            }
        }

        template<typename... Ts>
        constexpr auto generateArray(Ts const&... args) {
            std::array<std::byte, (getActualSize<Ts>() + ...)> buffer{};
            std::size_t                                        offset = 0;

            auto copyOne = [&]<typename T>(T const& arg) {
                constexpr std::size_t actual_size = getActualSize<T>();
                if constexpr(actual_size > 0) {
                    auto byte_array = std::bit_cast<std::array<std::byte, sizeof(T)>>(arg);
                    for(std::size_t i = 0; i < actual_size; ++i) {
                        buffer[offset + i] = byte_array[i];
                    }
                    offset += actual_size;
                }
            };

            (copyOne(args), ...);
            return buffer;
        }

    }   // namespace detail

    template<std::uint16_t DeviceVersion,
             std::uint16_t VendorID,
             std::uint16_t ProductID,
             std::uint8_t  ManufacturerStringID,
             std::uint8_t  ProductStringID,
             std::uint8_t  SerialNumberStringID,
             DeviceClass   Class,
             DeviceClass   SubClass>
    consteval auto makeDeviceDescriptorArray() {
        constexpr USB::Descriptors::Device DeviceDescriptor{
          .bDeviceClass{Class},
          .bDeviceSubClass{SubClass},
          .bDeviceProtocol{1},
          .bMaxPacketSize0{Kvasir::USB::detail::MaxPacketSize},
          .idVendor{VendorID},
          .idProduct{ProductID},
          .bcdDevice{DeviceVersion},
          .iManufacturer{ManufacturerStringID},
          .iProduct{ProductStringID},
          .iSerialNumber{SerialNumberStringID},
          .bNumConfigurations{1}};
        return detail::generateArray(DeviceDescriptor);
    }

    template<typename... Interfaces>
    consteval auto makeConfigDescriptorArray(std::uint16_t BusPower,
                                             bool          BusPowered,
                                             std::uint8_t  NumInterfaces,
                                             Interfaces const&... interfaces) {
        assert(500 >= BusPower);
        auto remainingConfig = detail::generateArray(interfaces...);

        USB::Descriptors::Configuration ConfigDescriptor{
          .wTotalLength{sizeof(remainingConfig) + sizeof(USB::Descriptors::Configuration)},
          .bNumInterfaces{NumInterfaces},
          .bConfigurationValue{1},
          .bmAttributes{BusPowered ? ConfigurationAttributes::BusPowered
                                   : ConfigurationAttributes::SelfPowered},
          .bMaxPower{static_cast<std::uint8_t>(BusPower / 2)}};

        return detail::generateArray(ConfigDescriptor, remainingConfig);
    }

}   // namespace Descriptors

template<typename Buffer>
struct DescriptorString {
    Buffer buffer{};

    constexpr DescriptorString(std::string_view s) {
        assert(buffer.max_size() >= s.size() * 2);

        if constexpr(requires { buffer.resize(0); }) {
            buffer.resize(s.size() * 2);
        } else {
            assert(buffer.max_size() == s.size() * 2);
        }

        assert(buffer.size() == s.size() * 2);
        auto pos = buffer.begin();
        for(auto c : s) {
            *pos = std::byte{c};
            ++pos;
            *pos = std::byte{0};
            ++pos;
        }
    }

    constexpr std::size_t size() const { return buffer.size(); }

    constexpr std::byte const* begin() const { return buffer.begin(); }

    constexpr std::byte const* end() const { return buffer.end(); }

    constexpr std::byte const* data() const { return buffer.data(); }

    constexpr DescriptorString const& get() const { return *this; }
};

template<std::size_t ByteCount>
consteval std::size_t checkedDescriptorStringSize() {
    static_assert(ByteCount <= 252, "String too long for USB string descriptor (max 126 chars)");
    return ByteCount;
}

template<std::size_t N>
DescriptorString(char const (&)[N])
  -> DescriptorString<std::array<std::byte,
                                 checkedDescriptorStringSize<(N - 1) * 2>()>>;
DescriptorString(std::string_view s) -> DescriptorString<Kvasir::StaticVector<std::byte,
                                                                              252>>;
template<char... chars>
DescriptorString(sc::StringConstant<chars...> sc)
  -> DescriptorString<std::array<std::byte,
                                 checkedDescriptorStringSize<sizeof...(chars) * 2>()>>;

template<typename F>
struct RuntimeDescriptorString {
    F f{};

    constexpr RuntimeDescriptorString(F f_) : f{f_} {}

    using DT = DescriptorString<Kvasir::StaticVector<std::byte, 252>>;

    struct RuntimeDescriptorStringImpl : DT {
        template<typename T>
        RuntimeDescriptorStringImpl(T const& v)
          : DT{[&]() {
              if constexpr(requires { std::string_view{v}; }) {
                  return DT{std::string_view{v}};
              } else {
                  std::array<char, 126> b;
                  auto const            ret = std::to_chars(b.begin(), b.end(), v);
                  assert(ret.ec == std::errc{});
                  return DT{
                    std::string_view{b.begin(), ret.ptr}
                  };
              }
          }()} {}
    };

    RuntimeDescriptorStringImpl get() const { return RuntimeDescriptorStringImpl{f()}; }
};

template<typename F>
RuntimeDescriptorString(F f) -> RuntimeDescriptorString<F>;

template<typename I,
         typename T>
I insertStringDescriptor(T const& descriptor,
                         I        first,
                         I        last) {
    // Check if buffer has enough space
    if(static_cast<std::size_t>(std::distance(first, last)) < descriptor.size()) {
        return first;   // Return unchanged position on error
    }
    std::memcpy(std::addressof(*first), descriptor.data(), descriptor.size());
    return std::next(first, std::ssize(descriptor));
}

// Returns iterator to end of inserted data, or 'first' unchanged if index is invalid
template<typename IT,
         typename T>
IT insertStringDescriptor(std::size_t index,
                          T const&    descriptors,
                          IT          first,
                          IT          last) {
    static constexpr auto Size = std::tuple_size_v<T>;
    if(index >= Size) {
        return first;   // index is out of range
    }

    IT   result = first;
    bool found  = false;

    auto action = [&](auto i) {
        static constexpr std::size_t I = decltype(i)::value;
        if(I == index) {
            result = insertStringDescriptor(std::get<I>(descriptors).get(), first, last);
            found  = true;
            return true;
        }
        return Size > I;
    };

    [&]<std::size_t... Ns>(std::index_sequence<Ns...>) {
        (action(std::integral_constant<std::size_t, Ns>{}) && ...);
    }(std::make_index_sequence<Size>{});

    return found ? result : first;
}
}   // namespace Kvasir::USB

namespace remote_fmt {
template<>
struct formatter<Kvasir::USB::SetupPacket> {
    template<typename Printer>
    constexpr auto format(Kvasir::USB::SetupPacket const& pkt,
                          Printer&                        printer) const {
        return format_to(
          printer,
          SC_LIFT("direction:{} type:{} recipient:{}  descriptorType:{}  request:{} "
                  "bmRequestType:{:#x} bRequest:{:#x} wValue:{:#x} wIndex:{:#x} wLength:{:#x}"),
          pkt.direction(),
          pkt.type(),
          pkt.recipient(),
          pkt.descriptorType(),
          pkt.bRequest,
          pkt.bmRequestType,
          std::to_underlying(pkt.bRequest),
          pkt.wValue,
          pkt.wIndex,
          pkt.wLength);
    }
};
}   // namespace remote_fmt
