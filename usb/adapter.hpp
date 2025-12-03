#pragma once

#include "descriptors.hpp"
#include "endpointOps.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <span>

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

    // State
    static inline std::atomic<bool>                                sendRdy{false};
    static inline Kvasir::Atomic::Queue<std::byte, RecvBufferSize> recvBuffer{};
    static inline std::span<std::byte const>                       currentSendData{};

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

    static bool SetupPacketRequestCallback([[maybe_unused]] SetupPacket const& pkt) {
        return false;
    }

    static void ResetCallback() {
        currentSendData = std::span<std::byte const>{};
        sendRdy         = false;
        recvBuffer.clear();
        EP_IN::reset();
        EP_OUT::reset();
    }

    static void ConfiguredCallback() {
        sendRdy = true;
        recv();
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
        recv();
    }

    static void onEndpointIn() { sendNext(); }

    static void sendNext() {
        if(currentSendData.size() > MaxPacketSize) {
            auto const sub  = currentSendData.subspan(0, MaxPacketSize);
            currentSendData = currentSendData.subspan(MaxPacketSize);
            send_impl<false>(sub);
        } else if(!currentSendData.empty()) {
            auto const sub  = currentSendData;
            currentSendData = std::span<std::byte const>{};
            send_impl<true>(sub);
        } else {
            sendRdy = true;
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

    static bool send(std::span<std::byte const> data) {
        assert(sendRdy == true);
        return send_impl<true>(data);
    }

    static void send_nocopy(std::span<std::byte const> data) {
        assert(sendRdy == true);
        currentSendData = data;
        sendRdy         = false;
        sendNext();
    }
};
}   // namespace Kvasir::USB::detail
