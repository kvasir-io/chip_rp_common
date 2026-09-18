#pragma once

#include "../Clocks.hpp"
#include "cdcacm.hpp"
#include "descriptors.hpp"
#include "detail.hpp"
#include "endpointOps.hpp"
#include "kvasir/Register/RegisterFmt.hpp"
#include "kvasir/Util/RateLimiter.hpp"
#include "mixins.hpp"
#include "resetInterface.hpp"
#include "simplebulk.hpp"
#include "winUsb.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chip/chip.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <kvasir/Atomic/Queue.hpp>
#include <kvasir/Util/StaticFunction.hpp>
#include <memory>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

namespace Kvasir::USB {
// Startup resource: the one USB controller (kvasir/StartUp/Resources.hpp).
struct InstanceTag {};

namespace detail {

    template<typename Clock,
             typename ConfigT,
             DeviceClass Class,
             DeviceClass SubClass,
             std::size_t FirstInterfaceNumber,
             std::size_t FirstEndpointNumber,
             template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
    struct USBBase
      : detail::MixinTraits::MixinBases<Clock,
                                        ConfigT,
                                        USBBase<Clock,
                                                ConfigT,
                                                Class,
                                                SubClass,
                                                FirstInterfaceNumber,
                                                FirstEndpointNumber,
                                                Mixins...>,
                                        FirstInterfaceNumber,
                                        FirstEndpointNumber,
                                        std::make_index_sequence<sizeof...(Mixins)>,
                                        Mixins...> {
    private:
        using Self = USBBase<Clock,
                             ConfigT,
                             Class,
                             SubClass,
                             FirstInterfaceNumber,
                             FirstEndpointNumber,
                             Mixins...>;

        using MixinsBase
          = detail::MixinTraits::MixinBases<Clock,
                                            ConfigT,
                                            Self,
                                            FirstInterfaceNumber,
                                            FirstEndpointNumber,
                                            std::make_index_sequence<sizeof...(Mixins)>,
                                            Mixins...>;

        template<typename, std::size_t, EndpointDirection, EndpointTransferType>
        friend struct EndpointOps;

        using Regs        = Kvasir::Peripheral::USB::Registers<0>;
        using BufferRegs  = Kvasir::Peripheral::USB_DPRAM::Registers<0>;
        using SetupPacket = Kvasir::USB::SetupPacket;
        using ClockType   = Clock;

        struct Config : ConfigT {
            static constexpr bool UseSof = [] {
                if constexpr(requires { ConfigT::StartOfFrameCallback(std::uint16_t{}); }) {
                    return true;
                } else {
                    return false;
                }
            }();

            static constexpr auto BusPower = [] {
                if constexpr(requires { ConfigT::BusPower; }) {
                    return ConfigT::BusPower;
                } else {
                    return 500;
                }
            }();

            static constexpr auto BusPowered = [] {
                if constexpr(requires { ConfigT::BusPowered; }) {
                    return ConfigT::BusPowered;
                } else {
                    return true;
                }
            }();

            static constexpr auto isrPriority = [] {
                if constexpr(requires { ConfigT::isrPriority; }) {
                    return ConfigT::isrPriority;
                } else {
                    return 1;
                }
            }();
            static_assert(
              !requires { ConfigT::DoubleBufferd; },
              "Config::DoubleBufferd is now spelt DoubleBuffered");
            static constexpr auto DoubleBuffered = [] {
                if constexpr(requires { ConfigT::DoubleBuffered; }) {
                    return ConfigT::DoubleBuffered;
                } else {
                    return true;
                }
            }();
        };

        static constexpr bool DoubleBuffered = Config::DoubleBuffered;

        // EP0 plus one DPRAM group per mixin endpoint number. EndpointOps always
        // addresses buffers through the DOUBLEBUFFER groups (2 x 2 x 64 bytes each), of
        // which the buffer map has 8, whether or not double buffering is enabled.
        static_assert(1
                          + MixinTraits::countMixinEndpoints<Clock,
                                                             ConfigT,
                                                             USBBase,
                                                             Mixins...>()
                        <= 8,
                      "too many endpoint numbers for the DPRAM buffer map");

        using EP0_IN
          = EndpointOps<USBBase, 0, EndpointDirection::In, EndpointTransferType::Control>;
        using EP0_OUT
          = EndpointOps<USBBase, 0, EndpointDirection::Out, EndpointTransferType::Control>;

        static constexpr std::tuple DescriptorStrings{
          []() {
              if constexpr(std::is_invocable_v<decltype(Config::ManufacturerString)>) {
                  return Kvasir::USB::RuntimeDescriptorString{Config::ManufacturerString};
              } else {
                  return Kvasir::USB::DescriptorString{SC_LIFT(Config::ManufacturerString)};
              }
          }(),
          []() {
              if constexpr(std::is_invocable_v<decltype(Config::ProductString)>) {
                  return Kvasir::USB::RuntimeDescriptorString{Config::ProductString};
              } else {
                  return Kvasir::USB::DescriptorString{SC_LIFT(Config::ProductString)};
              }
          }(),
          []() {
              if constexpr(std::is_invocable_v<decltype(Config::SerialNumberString)>) {
                  return Kvasir::USB::RuntimeDescriptorString{Config::SerialNumberString};
              } else {
                  return Kvasir::USB::DescriptorString{SC_LIFT(Config::SerialNumberString)};
              }
          }()};

        static constexpr auto DeviceDescriptor{USB::Descriptors::makeDeviceDescriptorArray<
          Config::ProductVersionBCD,
          Config::VendorID,
          Config::ProductID,
          1,
          2,
          3,
          Class,
          SubClass,
          detail::MixinTraits::maxBcdUSB<Clock, ConfigT, Self, Mixins...>()>()};

        static constexpr auto ConfigDescriptor = []() {
            constexpr std::size_t MixinInterfaceCount
              = detail::MixinTraits::countMixinInterfaces<Clock, ConfigT, Self, Mixins...>();

            constexpr auto mixinDescriptors
              = detail::MixinTraits::assembleMixinDescriptors<Clock,
                                                              ConfigT,
                                                              Self,
                                                              FirstInterfaceNumber,
                                                              FirstEndpointNumber,
                                                              Mixins...>();

            return Descriptors::makeConfigDescriptorArray(
              Config::BusPower,
              Config::BusPowered,
              static_cast<std::uint8_t>(MixinInterfaceCount),
              mixinDescriptors);
        }();

        // State
        static inline std::atomic<std::uint8_t>  configuration{};
        static inline std::uint8_t               deviceBusAddr{};
        static inline bool                       pendingAddressSet{false};
        static inline std::span<std::byte const> remainingControlData{};
        static inline bool                       endOfTransferPending{false};
        static inline std::array<std::byte, 256> stringDescriptorBuffer{};
        static inline detail::EP0ControlState    ep0_ctrl{};
        static inline std::uint8_t               isrMaskDepth{};

        enum class Fault : std::uint8_t {
            epTxError = 1,
            epRxError,
            sieError,
            watchdog,
            unhandledBufferDone,
            unhandledAbortDone,
            unhandledSetup,
            invalidOutData,
            outReadNotInDataPhase,
        };
        // Fault logging goes through this: a bad cable or a misbehaving host repeats
        // these per packet, from inside the ISR.
        static inline Kvasir::RateLimiter<Clock, Kvasir::RateLimiterConfig{.burst = 8}> faultLog_{};

        static void onIsr() {
            static constexpr auto CommonIsrList
              = Kvasir::MPL::list(read(Regs::INTS::setup_req),
                                  read(Regs::INTS::buff_status),
                                  read(Regs::INTS::bus_reset),
#if __has_include("chip/rp2350.hpp")
                                  read(Regs::INTS::dev_sm_watchdog_fired),
#endif
                                  read(Regs::INTS::abort_done));
            static constexpr auto IsrList = []() {
                if constexpr(Config::UseSof) {
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
                apply(
                  write(Regs::EP_TX_ERROR::FULLREGISTER, Kvasir::Register::value<0xffffffff>()));
            }
            if(std::uint32_t const rxError = apply(read(Regs::EP_RX_ERROR::FULLREGISTER))) {
                KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::epRxError, rxError)),
                                   UC_LOG_E,
                                   "USB: EP_RX_ERROR: {}",
                                   Kvasir::Register::Flags<typename Regs::EP_RX_ERROR>{rxError});
                apply(
                  write(Regs::EP_RX_ERROR::FULLREGISTER, Kvasir::Register::value<0xffffffff>()));
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

            if constexpr(Config::UseSof == true) {
                if(status[Regs::INTS::dev_sof]) {
                    auto const sof = get<0>(apply(read(Regs::SOF_RD::count)));
                    ConfigT::StartOfFrameCallback(static_cast<std::uint16_t>(sof));
                }
            }

            if(status[Regs::INTS::bus_reset]) {
                apply(Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::bus_reset)));
                handleBusReset();
                return;
            }

#if __has_include("chip/rp2350.hpp")
            if(status[Regs::INTS::dev_sm_watchdog_fired]) {
                KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::watchdog)),
                                   UC_LOG_E,
                                   "USB: watchdog");
                apply(set(Regs::DEV_SM_WATCHDOG::fired));
            }
#endif

            if(status[Regs::INTS::abort_done]) { handleAbort(); }

            if(status[Regs::INTS::buff_status]) { handleBufferStatus(); }

            if(status[Regs::INTS::setup_req]) {
                apply(Regs::SIE_STATUS::overrideDefaults(set(Regs::SIE_STATUS::setup_rec)));
                handleSetupPacket(getSetupPacket());
            }
        }

        static void endpointConfig() {
            EP0_IN::setupEndpoint();
            EP0_OUT::setupEndpoint();
            MixinsBase::callSetupEndpoints();
        }

        static SetupPacket getSetupPacket() {
            SetupPacket ret;
            detail::device_memory_memcpy(
              std::addressof(ret),
              reinterpret_cast<void const*>(BufferRegs::SETUP_PACKET_LOW::Addr::value),
              sizeof(SetupPacket));
            return ret;
        }

        static void handleBusReset() {
            ep0_ctrl.reset();
            deviceBusAddr        = 0;
            pendingAddressSet    = false;
            configuration        = 0;
            remainingControlData = {};
            endOfTransferPending = false;
            EP0_IN::reset();
            EP0_OUT::reset();

            apply(write(Regs::EP<0>::ADDR_ENDP::address, Kvasir::Register::value<0>()));

            MixinsBase::callReset();
            UC_LOG_I("USB: Bus reset detected");
        }

        static bool EndpointHandler(std::size_t ep_num,
                                    bool        in) {
            if(ep_num == 0 && in) {
                if(pendingAddressSet) {
                    UC_LOG_I("USB: Setting device address to {}", deviceBusAddr);
                    // Set actual device address in hardware
                    apply(write(Regs::EP<0>::ADDR_ENDP::address, deviceBusAddr));
                    pendingAddressSet = false;
                    EP0_IN::bufferFinished();
                    ep0_ctrl.transition(ControlStage::Idle);
                    return true;
                }
                if(!remainingControlData.empty() || endOfTransferPending) {
                    EP0_IN::bufferFinished();
                    auto const chunk = remainingControlData.first(
                      std::min(remainingControlData.size(), MaxPacketSize));
                    remainingControlData = remainingControlData.subspan(chunk.size());
                    if(chunk.empty()) { endOfTransferPending = false; }
                    if(remainingControlData.empty() && !endOfTransferPending) {
                        ep0IN<true>(chunk);
                    } else {
                        ep0IN<false>(chunk);
                    }
                    return true;
                }
                if(ep0_ctrl.stage() == ControlStage::Data) {
                    EP0_IN::bufferFinished();
                    ep0_ctrl.transition(ControlStage::Status);
                    ep0OUT<true>(0);
                    return true;
                }
                if(ep0_ctrl.stage() == ControlStage::Status) {
                    EP0_IN::bufferFinished();
                    ep0_ctrl.transition(ControlStage::Idle);
                    return true;
                }
                return false;
            }
            if(ep_num == 0 && !in) {
                if(ep0_ctrl.stage() == ControlStage::Status) {
                    EP0_OUT::bufferFinished();
                    ep0_ctrl.transition(ControlStage::Idle);
                    return true;
                }
            }
            return false;
        }

        static void handleBufferDone(std::size_t ep_num,
                                     bool        in) {
            using namespace std::string_view_literals;
            if(EndpointHandler(ep_num, in)) { return; }
            if(!MixinsBase::callEndpointHandler(ep_num, in)) {
                KVASIR_LOG_LIMITED(
                  faultLog_.allow(Kvasir::rateLimitKey(Fault::unhandledBufferDone, ep_num, in)),
                  UC_LOG_W,
                  "USB: Unhandled endpoint buffer done (EP{} {})",
                  ep_num,
                  in ? "IN"sv : "OUT"sv);
            }
        }

        static void handleBufferStatus() {
            std::uint32_t buffers = apply(read(Regs::BUFF_STATUS::FULLREGISTER));
            while(buffers) {
                auto const endpointBitIndex = static_cast<std::uint32_t>(std::countr_zero(buffers));
                auto const bit              = 1U << endpointBitIndex;
                // clear this in advance
                apply(write(Regs::BUFF_STATUS::FULLREGISTER, bit));
                // IN transfer for even endpointBitIndex, OUT transfer for odd endpointBitIndex
                handleBufferDone(endpointBitIndex >> 1U, (endpointBitIndex & 1U) == 0);
                buffers &= ~bit;
            }
        }

        static void handleAbortDone(std::size_t ep_num,
                                    bool        in) {
            using namespace std::string_view_literals;
            if(!MixinsBase::callAbortDone(ep_num, in)) {
                KVASIR_LOG_LIMITED(
                  faultLog_.allow(Kvasir::rateLimitKey(Fault::unhandledAbortDone, ep_num, in)),
                  UC_LOG_W,
                  "USB: Unhandled endpoint abort done (EP{} {})",
                  ep_num,
                  in ? "IN"sv : "OUT"sv);
            }
        }

        static void handleAbort() {
            std::uint32_t aborts = apply(read(Regs::EP_ABORT_DONE::FULLREGISTER));
            while(aborts) {
                auto const endpointBitIndex = static_cast<std::uint32_t>(std::countr_zero(aborts));
                auto const bit              = 1U << endpointBitIndex;
                // clear this in advance
                apply(write(Regs::EP_ABORT_DONE::FULLREGISTER, bit));
                // IN transfer for even endpointBitIndex, OUT transfer for odd endpointBitIndex
                handleAbortDone(endpointBitIndex >> 1U, (endpointBitIndex & 1U) == 0);
                aborts &= ~bit;
            }
        }

        static bool handleDeviceDescriptor(SetupPacket const& pkt) {
            return ep0INDataPhase(Self::DeviceDescriptor, pkt.wLength);
        }

        static bool handleConfigDescriptor(SetupPacket const& pkt) {
            // The only configuration has index 0.
            if((pkt.wValue & 0xff) != 0) { return false; }
            return ep0INDataPhase(Self::ConfigDescriptor, pkt.wLength);
        }

        static bool handleStringDescriptor(SetupPacket const& pkt) {
            // USB Language descriptor ID (English US)
            static constexpr std::array<std::byte, 2> USBLanguageDescriptor{std::byte{0x09},
                                                                            std::byte{0x04}};

            auto const  index  = static_cast<std::size_t>(pkt.wValue & 0xff);
            auto&       buffer = stringDescriptorBuffer;
            std::size_t len{};

            // Windows probes for a Microsoft OS 1.0 descriptor on every enumeration.
            static constexpr std::size_t MsOs10StringIndex = 0xEE;
            if(index == MsOs10StringIndex) { return stallQuietly(); }

            if(index == 0) {
                buffer[2] = USBLanguageDescriptor[0];
                buffer[3] = USBLanguageDescriptor[1];
                len       = 4;
            } else {
                auto const start = buffer.begin() + 2;
                auto const end   = Kvasir::USB::insertStringDescriptor(index - 1,
                                                                       DescriptorStrings,
                                                                       start,
                                                                       buffer.end());
                // Check if insertion failed (iterator unchanged means invalid index)
                if(end == start) { return false; }
                len = static_cast<std::size_t>(std::distance(buffer.begin(), end));
            }

            buffer[0] = std::byte(len);
            buffer[1] = std::byte(DescriptorType::string);

            return ep0INDataPhase(std::span{buffer.data(), len}, pkt.wLength);
        }

        // Descriptors hosts routinely ask for that a full-speed device does not have; the
        // STALL is the expected answer, so it is not logged.
        static bool stallQuietly() {
            stallControlRequest();
            return true;
        }

        static bool handleGetDescriptor(SetupPacket const& pkt) {
            switch(pkt.descriptorType()) {
            case DescriptorType::device:                  return handleDeviceDescriptor(pkt);
            case DescriptorType::configuration:           return handleConfigDescriptor(pkt);
            case DescriptorType::string:                  return handleStringDescriptor(pkt);
            case DescriptorType::deviceQualifier:
            case DescriptorType::otherSpeedConfiguration:
            case DescriptorType::debug:                   return stallQuietly();
            default:                                      return false;
            }
        }

        static bool handleSetupPacketDeviceIn(SetupPacket const& pkt) {
            switch(pkt.bRequest) {
            case SetupPacket::Request::getDescriptor: return handleGetDescriptor(pkt);

            case SetupPacket::Request::getStatus:
                {
                    std::array<std::byte, 2> const status{std::byte{Config::BusPowered ? 0 : 1}};
                    return ep0INDataPhase(status, pkt.wLength);
                }

            case SetupPacket::Request::getConfiguration:
                {
                    std::array<std::byte, 1> const value{std::byte{configuration.load()}};
                    return ep0INDataPhase(value, pkt.wLength);
                }

            default: return false;
            }
        }

        static bool handleSetupPacketDeviceOut(SetupPacket const& pkt) {
            switch(pkt.bRequest) {
            case SetupPacket::Request::setAddress:
                acknowledgeSetupRequest();
                // Set address is special: send 0-length status packet first with address 0
                deviceBusAddr     = pkt.wValue & 0x7f;
                pendingAddressSet = true;
                return true;

            case SetupPacket::Request::setConfiguration:
                using namespace std::string_view_literals;
                // The only configuration is 1.
                if((pkt.wValue & 0xff) > 1) { return false; }
                acknowledgeSetupRequest();
                configuration = (pkt.wValue & 0xff);

                MixinsBase::callConfigured(configuration);
                UC_LOG_I("USB: Device {} (config={})",
                         configuration == 0 ? "unconfigured"sv : "configured"sv,
                         configuration.load());
                return true;

            default: return false;
            }
        }

        static constexpr std::size_t TotalInterfaceCount
          = MixinTraits::countMixinInterfaces<Clock, ConfigT, Self, Mixins...>();

        // Standard requests no mixin claimed: interfaces without endpoints of their own always
        // run alternate setting 0, and EP0 is never halted.
        static bool handleStandardFallback(SetupPacket const& pkt) {
            using Request   = SetupPacket::Request;
            using Recipient = SetupPacket::Recipient;
            bool const in   = pkt.direction() == SetupPacket::Direction::deviceToHost;

            if(pkt.recipient() == Recipient::interface) {
                if(configuration == 0 || pkt.wIndex >= TotalInterfaceCount) { return false; }
                switch(pkt.bRequest) {
                case Request::getStatus:
                    {
                        if(!in) { return false; }
                        static constexpr std::array<std::byte, 2> Status{};
                        return ep0INDataPhase(Status, pkt.wLength);
                    }
                case Request::getInterface:
                    {
                        if(!in) { return false; }
                        static constexpr std::array<std::byte, 1> AlternateSetting{};
                        return ep0INDataPhase(AlternateSetting, pkt.wLength);
                    }
                case Request::setInterface:
                    if(in || pkt.wValue != 0) { return false; }
                    acknowledgeSetupRequest();
                    return true;
                default: return false;
                }
            }
            if(pkt.recipient() == Recipient::endpoint && (pkt.wIndex & 0x7f) == 0
               && pkt.bRequest == Request::getStatus && in)
            {
                static constexpr std::array<std::byte, 2> Status{};
                return ep0INDataPhase(Status, pkt.wLength);
            }
            return false;
        }

        static void handleSetupPacket(SetupPacket const& pkt) {
            remainingControlData = {};
            endOfTransferPending = false;
            ep0_ctrl.transition(ControlStage::Setup);

            EP0_IN::cancelTransfer();
            EP0_OUT::cancelTransfer();
            EP0_IN::state.setDataPhase();
            EP0_OUT::state.setDataPhase();

            bool handled = false;

            if(pkt.type() == SetupPacket::Type::standard
               && pkt.recipient() == SetupPacket::Recipient::device)
            {
                if(pkt.direction() == SetupPacket::Direction::hostToDevice) {
                    handled = handleSetupPacketDeviceOut(pkt);
                } else {
                    handled = handleSetupPacketDeviceIn(pkt);
                }
            }
            if(!handled) { handled = MixinsBase::callSetupPacketRequest(pkt); }
            if(!handled && pkt.type() == SetupPacket::Type::standard) {
                handled = handleStandardFallback(pkt);
            }

            // Centralized error handling - USB 2.0 spec requires STALL for unsupported requests
            if(!handled) {
                KVASIR_LOG_LIMITED(
                  faultLog_.allow(
                    Kvasir::rateLimitKey(Fault::unhandledSetup, pkt.bmRequestType, pkt.bRequest)),
                  UC_LOG_W,
                  "USB: STALL - Unhandled setup packet: {}",
                  pkt);
                stallControlRequest();
            }
        }

        template<bool Last>
        static void ep0IN(std::span<std::byte const> data) {
            EP0_IN::template tryTransfer<Last>(data);
        }

        template<bool Last>
        static void ep0OUT(std::size_t size) {
            EP0_OUT::template tryTransfer<Last>(size);
        }

        // RP2040/RP2350 DPSRAM buffer size
        static constexpr std::size_t DPSRAMSize = 4096;

        static constexpr auto getSofEnable() {
            if constexpr(Config::UseSof) {
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
#if __has_include("chip/rp2350.hpp")
                                           set(Regs::INTE::dev_sm_watchdog_fired),
#endif
                                           getSofEnable()));
        }

        static constexpr auto InterruptIndexes
          = brigand::list<decltype(Kvasir::Interrupt::usbctrl)>{};

    public:
        // We can make them private with c++26 friend pack indexing
        //Mixin API
        // Multi-packet IN data stage, truncated to wLength. data must outlive the transfer.
        static bool ep0INDataPhase(std::span<std::byte const> data,
                                   std::uint16_t              wLength) {
            // wLength 0: no data stage, the status stage follows directly.
            if(wLength == 0) {
                acknowledgeSetupRequest();
                return true;
            }
            data = data.first(std::min<std::size_t>(data.size(), wLength));
            // A short reply that ends on a packet boundary needs a zero-length packet.
            bool const endsOnPacketBoundary
              = !data.empty() && data.size() < wLength && data.size() % MaxPacketSize == 0;
            auto const first = data.first(std::min(data.size(), MaxPacketSize));

            ep0_ctrl.transition(ControlStage::Data);
            remainingControlData = data.subspan(first.size());
            endOfTransferPending = endsOnPacketBoundary;
            if(remainingControlData.empty() && !endOfTransferPending) {
                ep0IN<true>(first);
            } else {
                ep0IN<false>(first);
            }
            return true;
        }

        // Whole configuration descriptor, for mixins describing other mixins' interfaces.
        // Only usable where the device type is complete, i.e. inside a callback body.
        static constexpr std::span<std::byte const> configDescriptor() { return ConfigDescriptor; }

        static void ep0OUTDataPhase(std::size_t size) {
            assert(MaxPacketSize >= size);
            ep0_ctrl.transition(ControlStage::Data);
            ep0OUT<true>(size);
        }

        static bool ep0OUTGetData(std::span<std::byte> data) {
            if(ep0_ctrl.stage() == ControlStage::Data) {
                std::size_t const len = EP0_OUT::readCurrentBuffer(data);
                if(len != data.size()) {
                    KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::invalidOutData)),
                                       UC_LOG_E,
                                       "USB: Invalid out data (received={}, expected={})",
                                       len,
                                       data.size());
                    return false;
                }
                EP0_OUT::bufferFinished();
                return true;
            } else {
                KVASIR_LOG_LIMITED(
                  faultLog_.allow(Kvasir::rateLimitKey(Fault::outReadNotInDataPhase)),
                  UC_LOG_E,
                  "USB: out read attempted while not in data phase");
                return false;
            }
        }

        static void acknowledgeSetupRequest() {
            ep0_ctrl.transition(ControlStage::Status);
            ep0IN<true>(std::span<std::byte const>{});
        }

        // Ends the current control request with a STALL, from the setup or the data stage.
        static void stallControlRequest() {
            EP0_IN::stall();
            EP0_OUT::stall();
            ep0_ctrl.transition(ControlStage::Stall);
        }

        // Runs f with the USB interrupt masked, for thread code touching endpoint state the
        // interrupt also changes. Nests; safe from inside the interrupt too.
        template<typename F>
        static decltype(auto) withIsrMasked(F&& f) {
            struct Guard {
                Guard() {
                    if(isrMaskDepth++ == 0) { apply(Kvasir::Nvic::makeDisable(InterruptIndexes)); }
                }

                ~Guard() {
                    if(--isrMaskDepth == 0) { apply(Kvasir::Nvic::makeEnable(InterruptIndexes)); }
                }

                Guard(Guard const&)            = delete;
                Guard& operator=(Guard const&) = delete;
            };

            Guard const guard{};
            return std::forward<F>(f)();
        }

    public:
        //Kvasir Callbacks
        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Kvasir::Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(InterruptIndexes));

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
          = list(Kvasir::Nvic::makeSetPriority<Config::isrPriority>(InterruptIndexes),
                 Kvasir::Nvic::makeClearPending(InterruptIndexes));

        static constexpr auto runtimeInit = []() {
            //can use std::memset here since it should always make alligned access.
            std::memset(reinterpret_cast<void*>(BufferRegs::baseAddr), 0, DPSRAMSize);

            endpointConfig();
            apply(Kvasir::Nvic::makeEnable(InterruptIndexes));

            apply(Regs::SIE_CTRL::overrideDefaults(
              set(Regs::SIE_CTRL::pullup_en),
              set(Regs::SIE_CTRL::ep0_int_1buf),
              clear(Regs::SIE_CTRL::pulldown_en),
              // EP0 single buffered, see EndpointOps::DoubleBuffered.
              write(Regs::SIE_CTRL::ep0_double_buf, Kvasir::Register::value<0>())));
        };

    public:
        //Public API
        static bool isConfigured() { return configuration != 0; }
    };
}   // namespace detail

// Convenience type aliases for common USB configurations

// Standard CDC-ACM device with SendRecv and Reset capabilities
template<typename Clock,
         typename Config,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
using CDC_ACM_Standard = detail::USBBase<Clock,
                                         Config,
                                         DeviceClass::Miscellaneous,
                                         DeviceClass::Communication,
                                         0,   // FirstInterfaceNumber
                                         1,   // FirstEndpointNumber
                                         CDC::ACM::Mixin,
                                         Mixins...>;

// Standard simple bulk device. Device class EF/02/01: a composite device.
template<typename Clock,
         typename Config,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
using Bulk_Standard = detail::USBBase<Clock,
                                      Config,
                                      DeviceClass::Miscellaneous,
                                      DeviceClass::CommonClass,
                                      0,   // FirstInterfaceNumber
                                      1,   // FirstEndpointNumber
                                      SimpleBulk::Mixin,
                                      Mixins...>;
}   // namespace Kvasir::USB
