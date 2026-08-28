#pragma once

#include "descriptors.hpp"
#include "endpointOps.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <span>
#include <string_view>

namespace Kvasir::USB::detail {
// Adapter for send/recv pattern used by mixins
template<typename Clock,
         typename Config,
         typename Derived,
         typename Mixin,
         std::size_t EndpointNumber>
struct SendRecvAdapter {
private:
    friend Mixin;
    static constexpr auto RecvBufferSize = [] {
        if constexpr(requires { Config::RecvBufferSize; }) {
            return Config::RecvBufferSize;
        } else {
            return 4096;
        }
    }();

    using EP_IN
      = EndpointOps<Derived, EndpointNumber, EndpointDirection::In, EndpointTransferType::Bulk>;
    using EP_OUT
      = EndpointOps<Derived, EndpointNumber, EndpointDirection::Out, EndpointTransferType::Bulk>;

    static constexpr auto SendBufferSize = [] {
        if constexpr(requires { Config::SendBufferSize; }) {
            return Config::SendBufferSize;
        } else {
            return 4096;
        }
    }();

    // State
    static inline std::atomic<bool>                                sendRdy{false};
    static inline Kvasir::Atomic::Queue<std::byte, RecvBufferSize> recvBuffer{};
    static inline std::span<std::byte const>                       currentSendData{};
    static inline std::array<std::byte, SendBufferSize>            sendBuffer{};
    // A message whose length is an exact multiple of the packet size must be closed
    // with a zero-length packet, otherwise a host reading with a large buffer cannot
    // tell where it ends and waits for more. Set when the final full packet goes out.
    static inline bool needZeroLengthPacket{false};

    // Callbacks (called by MixinBases)
    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        if(epNum == EP_IN::ep_num && in) {
            onEndpointIn();
            return true;
        } else if(epNum == EP_OUT::ep_num && !in) {
            onEndpointOut();
            return true;
        }
        return false;
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        // abortDone() clears the buffer control, so whatever was armed is gone and the
        // endpoint has to be brought back into service here if the device is still
        // configured (a halt clear); after a bus reset or SET_CONFIGURATION(0) the
        // ConfiguredCallback does that.
        if(epNum == EP_IN::ep_num && in) {
            EP_IN::abortDone();
            if(Derived::isConfigured()) { sendRdy = true; }
            return true;
        } else if(epNum == EP_OUT::ep_num && !in) {
            EP_OUT::abortDone();
            if(Derived::isConfigured()) { recv(); }
            return true;
        }
        return false;
    }

    // CLEAR_FEATURE(ENDPOINT_HALT) on one of the data endpoints. A host that has just
    // opened the device issues it on both endpoints to start from a known state; it is
    // answered here by discarding whatever was queued in that direction and resetting
    // the data toggle, which is what the request means. Without this the base class
    // would STALL the request and the host would treat the device as broken.
    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        using Direction = SetupPacket::Direction;
        using Recipient = SetupPacket::Recipient;
        using Request   = SetupPacket::Request;
        using Type      = SetupPacket::Type;

        if(pkt.type() != Type::standard || pkt.recipient() != Recipient::endpoint
           || pkt.direction() != Direction::hostToDevice || pkt.bRequest != Request::clearFeature
           || pkt.wValue != 0 /* ENDPOINT_HALT */)
        {
            return false;
        }
        std::size_t const epNum = pkt.wIndex & 0x7F;
        bool const        in    = (pkt.wIndex & 0x80) != 0;
        if(epNum != EndpointNumber) { return false; }

        // The endpoint is re-armed (OUT) or released for sending (IN) from
        // AbortDoneCallback: abortDone() wipes the buffer control, so anything armed
        // between abort() and the abort-done interrupt would be lost.
        if(in) {
            currentSendData      = std::span<std::byte const>{};
            needZeroLengthPacket = false;
            sendRdy              = false;
            EP_IN::abort();
            EP_IN::resetPid();
        } else {
            EP_OUT::abort();
            EP_OUT::resetPid();
            recvBuffer.clear();
        }
        Derived::acknowledgeSetupRequest();
        using namespace std::string_view_literals;
        UC_LOG_I("Bulk: EP{} {} halt cleared, queue flushed", epNum, in ? "IN"sv : "OUT"sv);
        return true;
    }

    static void ResetCallback() {
        currentSendData      = std::span<std::byte const>{};
        needZeroLengthPacket = false;
        sendRdy              = false;
        recvBuffer.clear();
        EP_IN::abort();
        EP_OUT::abort();
        EP_IN::reset();
        EP_OUT::reset();
    }

    static void ConfiguredCallback(std::uint8_t configuration) {
        if(configuration == 0) {
            currentSendData      = std::span<std::byte const>{};
            needZeroLengthPacket = false;
            sendRdy              = false;
            recvBuffer.clear();
            EP_OUT::abort();
            EP_OUT::resetPid();
        } else {
            sendRdy = true;
            recv();
        }
    }

    static void SetupEndpointsCallback() {
        EP_IN::setupEndpoint();
        EP_OUT::setupEndpoint();
    }

    // Internal implementation
    static bool recv() { return EP_OUT::armReceive(MaxPacketSize); }

    static void onEndpointOut() {
        std::array<std::byte, MaxPacketSize> tempBuffer{};
        std::size_t const                    len = EP_OUT::readCurrentBuffer(tempBuffer);
        recvBuffer.push(std::span{tempBuffer.data(), len});

        EP_OUT::bufferFinished();
        recv();
    }

    static void onEndpointIn() {
        EP_IN::bufferFinished();
        sendNext();
    }

    static bool sendNext() {
        if(currentSendData.size() > MaxPacketSize) {
            auto const sub  = currentSendData.subspan(0, MaxPacketSize);
            currentSendData = currentSendData.subspan(MaxPacketSize);
            return send_impl<false>(sub);
        } else if(!currentSendData.empty()) {
            auto const sub  = currentSendData;
            currentSendData = std::span<std::byte const>{};
            if(sub.size() == MaxPacketSize) {
                // A full final packet is not a message end on the wire; the ZLP that
                // follows is.
                needZeroLengthPacket = true;
                return send_impl<false>(sub);
            }
            return send_impl<true>(sub);
        } else if(needZeroLengthPacket) {
            needZeroLengthPacket = false;
            return send_impl<true>({});
        } else {
            sendRdy = true;
            return true;
        }
    }

    template<bool Last>
    static bool send_impl(std::span<std::byte const> data) {
        assert(MaxPacketSize >= data.size());
        return EP_IN::template tryTransfer<Last>(data);
    }

public:
    // Public API
    static bool isSendReady() { return sendRdy; }

    static auto& getRecvBuffer() { return recvBuffer; }

    static bool send_nocopy(std::span<std::byte const> data) {
        //TODO think about locking isr
        if(!sendRdy) { return false; }
        currentSendData = data;
        sendRdy         = false;
        return sendNext();
    }

    static bool send(std::span<std::byte const> data) {
        if(!sendRdy) { return false; }
        if(data.size() > sendBuffer.size()) { return false; }
        std::copy(data.begin(), data.end(), sendBuffer.begin());
        return send_nocopy(std::span<std::byte const>{sendBuffer.data(), data.size()});
    }
};

// The IN half of SendRecvAdapter for a device-to-host only endpoint (a log or event
// stream). No OUT endpoint is set up, so the OUT half of its DPRAM group stays unused
// and no receive queue exists. The send buffer is sized by Config::StreamSendBufferSize
// so a secondary channel can be smaller than the command channel, which uses
// Config::SendBufferSize.
template<typename Clock,
         typename Config,
         typename Derived,
         typename Mixin,
         std::size_t EndpointNumber>
struct SendOnlyAdapter {
private:
    friend Mixin;

    using EP_IN
      = EndpointOps<Derived, EndpointNumber, EndpointDirection::In, EndpointTransferType::Bulk>;

    static constexpr auto SendBufferSize = [] {
        if constexpr(requires { Config::StreamSendBufferSize; }) {
            return Config::StreamSendBufferSize;
        } else {
            return 1024;
        }
    }();

    static inline std::atomic<bool>                     sendRdy{false};
    static inline std::span<std::byte const>            currentSendData{};
    static inline std::array<std::byte, SendBufferSize> sendBuffer{};
    static inline bool                                  needZeroLengthPacket{false};

    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        if(epNum == EP_IN::ep_num && in) {
            EP_IN::bufferFinished();
            sendNext();
            return true;
        }
        return false;
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        if(epNum == EP_IN::ep_num && in) {
            EP_IN::abortDone();
            if(Derived::isConfigured()) { sendRdy = true; }
            return true;
        }
        return false;
    }

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        using Direction = SetupPacket::Direction;
        using Recipient = SetupPacket::Recipient;
        using Request   = SetupPacket::Request;
        using Type      = SetupPacket::Type;

        if(pkt.type() != Type::standard || pkt.recipient() != Recipient::endpoint
           || pkt.direction() != Direction::hostToDevice || pkt.bRequest != Request::clearFeature
           || pkt.wValue != 0 /* ENDPOINT_HALT */)
        {
            return false;
        }
        std::size_t const epNum = pkt.wIndex & 0x7F;
        bool const        in    = (pkt.wIndex & 0x80) != 0;
        if(epNum != EndpointNumber || !in) { return false; }

        // Released for sending again from AbortDoneCallback, see SendRecvAdapter.
        currentSendData      = std::span<std::byte const>{};
        needZeroLengthPacket = false;
        sendRdy              = false;
        EP_IN::abort();
        EP_IN::resetPid();
        Derived::acknowledgeSetupRequest();
        UC_LOG_I("Bulk: EP{} IN halt cleared", epNum);
        return true;
    }

    static void ResetCallback() {
        currentSendData      = std::span<std::byte const>{};
        needZeroLengthPacket = false;
        sendRdy              = false;
        EP_IN::abort();
        EP_IN::reset();
    }

    static void ConfiguredCallback(std::uint8_t configuration) {
        if(configuration == 0) {
            currentSendData      = std::span<std::byte const>{};
            needZeroLengthPacket = false;
            sendRdy              = false;
        } else {
            sendRdy = true;
        }
    }

    static void SetupEndpointsCallback() { EP_IN::setupEndpoint(); }

    static bool sendNext() {
        if(currentSendData.size() > MaxPacketSize) {
            auto const sub  = currentSendData.subspan(0, MaxPacketSize);
            currentSendData = currentSendData.subspan(MaxPacketSize);
            return send_impl<false>(sub);
        } else if(!currentSendData.empty()) {
            auto const sub  = currentSendData;
            currentSendData = std::span<std::byte const>{};
            if(sub.size() == MaxPacketSize) {
                needZeroLengthPacket = true;
                return send_impl<false>(sub);
            }
            return send_impl<true>(sub);
        } else if(needZeroLengthPacket) {
            needZeroLengthPacket = false;
            return send_impl<true>({});
        } else {
            sendRdy = true;
            return true;
        }
    }

    template<bool Last>
    static bool send_impl(std::span<std::byte const> data) {
        assert(MaxPacketSize >= data.size());
        return EP_IN::template tryTransfer<Last>(data);
    }

public:
    static bool isSendReady() { return sendRdy; }

    static bool send_nocopy(std::span<std::byte const> data) {
        if(!sendRdy) { return false; }
        currentSendData = data;
        sendRdy         = false;
        return sendNext();
    }

    static bool send(std::span<std::byte const> data) {
        if(!sendRdy) { return false; }
        if(data.size() > sendBuffer.size()) { return false; }
        std::copy(data.begin(), data.end(), sendBuffer.begin());
        return send_nocopy(std::span<std::byte const>{sendBuffer.data(), data.size()});
    }
};
}   // namespace Kvasir::USB::detail
