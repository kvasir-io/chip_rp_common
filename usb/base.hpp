#pragma once

#include "cdcacm.hpp"
#include "descriptors.hpp"
#include "detail.hpp"
#include "endpointOps.hpp"
#include "mixins.hpp"
#include "resetInterface.hpp"
#include "simplebulk.hpp"

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
            static constexpr auto DoubleBufferd = [] {
                if constexpr(requires { ConfigT::DoubleBufferd; }) {
                    return ConfigT::DoubleBufferd;
                } else {
                    return true;
                }
            }();
        };

        static constexpr bool DoubleBufferd = Config::DoubleBufferd;

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

        static constexpr auto DeviceDescriptor{
          USB::Descriptors::makeDeviceDescriptorArray<Config::ProductVersionBCD,
                                                      Config::VendorID,
                                                      Config::ProductID,
                                                      1,
                                                      2,
                                                      3,
                                                      Class,
                                                      SubClass>()};

        static constexpr auto ConfigDescriptor = []() {
            constexpr std::size_t MixinInterfaceCount
              = detail::MixinTraits::countMixinInterfaces<Clock, Config, Self, Mixins...>();

            constexpr auto mixinDescriptors
              = detail::MixinTraits::assembleMixinDescriptors<Clock,
                                                              Config,
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
        static inline std::span<std::byte const> remainingDescriptor{};
        static inline std::array<std::byte, 256> stringDescriptorBuffer{};
        static inline detail::EP0ControlState    ep0_ctrl{};

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

            auto const              status    = apply(IsrList);
            std::uint32_t const     sieStatus = apply(read(Regs::SIE_STATUS::FULLREGISTER));
            constexpr std::uint32_t errorMask = (1U << 31)    // DATA_SEQ_ERROR
                                              | (1U << 27)    // RX_TIMEOUT
                                              | (1U << 26)    // RX_OVERFLOW
                                              | (1U << 25)    // BIT_STUFF_ERROR
                                              | (1U << 24)    // CRC_ERROR
                                              | (1U << 23);   // ENDPOINT_ERROR

#if __has_include("chip/rp2350.hpp")
            if(apply(read(Regs::EP_TX_ERROR::FULLREGISTER))) {
                UC_LOG_E("USB: EP_TX_ERROR: {}", Regs::EP_TX_ERROR{});
                apply(
                  write(Regs::EP_TX_ERROR::FULLREGISTER, Kvasir::Register::value<0xffffffff>()));
            }
            if(apply(read(Regs::EP_RX_ERROR::FULLREGISTER))) {
                UC_LOG_E("USB: EP_RX_ERROR: {}", Regs::EP_RX_ERROR{});
                apply(
                  write(Regs::EP_RX_ERROR::FULLREGISTER, Kvasir::Register::value<0xffffffff>()));
            }
#endif
            if(sieStatus & errorMask) {
                UC_LOG_E("USB: SIE_STATUS error: {:#010x} {}",
                         sieStatus & errorMask,
                         Regs::SIE_STATUS{});
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
                UC_LOG_E("USB: watchdog");
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
            deviceBusAddr       = 0;
            pendingAddressSet   = false;
            configuration       = 0;
            remainingDescriptor = {};
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
                    return true;
                }
                if(!remainingDescriptor.empty()) {
                    auto const chunkSize = std::min(remainingDescriptor.size(), MaxPacketSize);
                    bool const isLast    = (remainingDescriptor.size() <= MaxPacketSize);
                    EP0_IN::bufferFinished();
                    if(isLast) {
                        ep0IN<true>(remainingDescriptor.first(chunkSize));
                        remainingDescriptor = {};
                    } else {
                        ep0IN<false>(remainingDescriptor.first(chunkSize));
                        remainingDescriptor = remainingDescriptor.subspan(chunkSize);
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
                UC_LOG_W("USB: Unhandled endpoint buffer done (EP{} {})",
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
                UC_LOG_W("USB: Unhandled endpoint abort done (EP{} {})",
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

        static bool handleDeviceDescriptor() {
            ep0_ctrl.transition(ControlStage::Data);
            ep0IN<true>(Self::DeviceDescriptor);
            return true;
        }

        static bool handleConfigDescriptor(SetupPacket const& pkt) {
            if(pkt.wLength >= Self::ConfigDescriptor.size()) {
                if constexpr(Self::ConfigDescriptor.size() > MaxPacketSize) {
                    ep0_ctrl.transition(ControlStage::Data);
                    remainingDescriptor = std::span{Self::ConfigDescriptor.begin() + MaxPacketSize,
                                                    Self::ConfigDescriptor.end()};
                    ep0IN<false>(std::span{Self::ConfigDescriptor.begin(),
                                           Self::ConfigDescriptor.begin() + MaxPacketSize});
                } else {
                    ep0_ctrl.transition(ControlStage::Data);
                    ep0IN<true>(Self::ConfigDescriptor);
                }
                return true;
            } else if(pkt.wLength == sizeof(Kvasir::USB::Descriptors::Configuration)) {
                ep0_ctrl.transition(ControlStage::Data);
                // Host queries only config part
                ep0IN<true>(std::span{Self::ConfigDescriptor.begin(),
                                      Self::ConfigDescriptor.begin()
                                        + sizeof(Kvasir::USB::Descriptors::Configuration)});
                return true;
            }
            return false;
        }

        static bool handleStringDescriptor(SetupPacket const& pkt) {
            // USB Language descriptor ID (English US)
            static constexpr std::array<std::byte, 2> USBLanguageDescriptor{std::byte{0x09},
                                                                            std::byte{0x04}};

            auto const  index  = static_cast<std::size_t>(pkt.wValue & 0xff);
            auto&       buffer = stringDescriptorBuffer;
            std::size_t len{};

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

            auto const totalLen = std::min<std::size_t>(pkt.wLength, len);
            ep0_ctrl.transition(ControlStage::Data);

            if(totalLen <= MaxPacketSize) {
                ep0IN<true>(std::span{buffer.data(), totalLen});
            } else {
                ep0IN<false>(std::span{buffer.data(), MaxPacketSize});
                remainingDescriptor
                  = std::span{buffer.data() + MaxPacketSize, totalLen - MaxPacketSize};
            }
            return true;
        }

        static bool handleDeviceQualifier() {
            // Full-Speed devices STALL deviceQualifier requests per USB 2.0 spec
            // Handle this STALL differently to prevent generic error message since it is default behavior and no error
            EP0_IN::stall();
            EP0_OUT::stall();
            ep0_ctrl.transition(ControlStage::Stall);
            return true;
        }

        static bool handleGetDescriptor(SetupPacket const& pkt) {
            switch(pkt.descriptorType()) {
            case DescriptorType::device:          return handleDeviceDescriptor();
            case DescriptorType::configuration:   return handleConfigDescriptor(pkt);
            case DescriptorType::string:          return handleStringDescriptor(pkt);
            case DescriptorType::deviceQualifier: return handleDeviceQualifier();
            default:                              return false;
            }
        }

        static bool handleSetupPacketDeviceIn(SetupPacket const& pkt) {
            switch(pkt.bRequest) {
            case SetupPacket::Request::getDescriptor: return handleGetDescriptor(pkt);

            case SetupPacket::Request::getStatus:
                {
                    ep0_ctrl.transition(ControlStage::Data);
                    std::array<std::byte, 2> buffer{};
                    ep0IN<true>(buffer);
                    return true;
                }

            default: return false;
            }
            return false;
        }

        static bool handleSetupPacketDeviceOut(SetupPacket const& pkt) {
            switch(pkt.bRequest) {
            case SetupPacket::Request::setAddress:
                acknowledgeSetupRequest();
                // Set address is special: send 0-length status packet first with address 0
                deviceBusAddr     = pkt.wValue & 0xff;
                pendingAddressSet = true;
                return true;

            case SetupPacket::Request::setConfiguration:
                using namespace std::string_view_literals;
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

        static void handleSetupPacket(SetupPacket const& pkt) {
            remainingDescriptor = {};
            ep0_ctrl.transition(ControlStage::Setup);

            EP0_IN::state.setDataPhase();
            EP0_OUT::state.setDataPhase();

            bool handled = false;

            if(pkt.recipient() == SetupPacket::Recipient::device) {
                if(pkt.direction() == SetupPacket::Direction::hostToDevice) {
                    handled = handleSetupPacketDeviceOut(pkt);
                } else {
                    handled = handleSetupPacketDeviceIn(pkt);
                }
            }
            if(!handled) { handled = MixinsBase::callSetupPacketRequest(pkt); }

            // Centralized error handling - USB 2.0 spec requires STALL for unsupported requests
            if(!handled) {
                UC_LOG_W("USB: STALL - Unhandled setup packet: {}", pkt);
                EP0_IN::stall();
                EP0_OUT::stall();
                ep0_ctrl.transition(ControlStage::Stall);
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
        static void ep0INDataPhase(std::span<std::byte const> data) {
            assert(MaxPacketSize >= data.size());
            ep0_ctrl.transition(ControlStage::Data);
            ep0IN<true>(data);
        }

        static void ep0OUTDataPhase(std::size_t size) {
            assert(MaxPacketSize >= size);
            ep0_ctrl.transition(ControlStage::Data);
            ep0OUT<true>(size);
        }

        static bool ep0OUTGetData(std::span<std::byte> data) {
            if(ep0_ctrl.stage() == ControlStage::Data) {
                std::size_t const len = EP0_OUT::readCurrentBuffer(data);
                if(len != data.size()) {
                    UC_LOG_E("USB: Invalid out data (received={}, expected={})", len, data.size());
                    return false;
                }
                EP0_OUT::bufferFinished();
                return true;
            } else {
                UC_LOG_E("USB: out read attempted while not in data phase");
                return false;
            }
        }

        static void acknowledgeSetupRequest() {
            ep0_ctrl.transition(ControlStage::Status);
            ep0IN<true>(std::span<std::byte const>{});
        }

    public:
        //Kvasir Callbacks
        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Kvasir::Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(InterruptIndexes));

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
              write(Regs::SIE_CTRL::ep0_double_buf,
                    Kvasir::Register::value<DoubleBufferd ? 1 : 0>())));
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

// Standard simple bulk device with SendRecv capability
template<typename Clock,
         typename Config,
         template<typename, typename, typename, std::size_t, std::size_t> class... Mixins>
using Bulk_Standard = detail::USBBase<Clock,
                                      Config,
                                      DeviceClass::Miscellaneous,
                                      DeviceClass::Miscellaneous,
                                      0,   // FirstInterfaceNumber
                                      1,   // FirstEndpointNumber
                                      SimpleBulk::Mixin,
                                      Mixins...>;
}   // namespace Kvasir::USB
