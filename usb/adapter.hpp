#pragma once

#include "descriptors.hpp"
#include "endpointOps.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <functional>
#include <kvasir/Atomic/Queue.hpp>
#include <span>
#include <string_view>
#include <utility>

namespace Kvasir::USB::detail {

// Standard requests addressed to one endpoint: GET_STATUS and SET/CLEAR_FEATURE(ENDPOINT_HALT).
// Endpoint provides Address, halted, halt() and clearHalt().
template<typename Derived,
         typename Endpoint>
bool handleEndpointRequest(SetupPacket const& pkt) {
    using Direction = SetupPacket::Direction;
    using Request   = SetupPacket::Request;

    if(pkt.type() != SetupPacket::Type::standard
       || pkt.recipient() != SetupPacket::Recipient::endpoint || pkt.wIndex != Endpoint::Address)
    {
        return false;
    }
    switch(pkt.bRequest) {
    case Request::getStatus:
        {
            if(pkt.direction() != Direction::deviceToHost) { return false; }
            std::array<std::byte, 2> const status{static_cast<std::byte>(Endpoint::halted)};
            return Derived::ep0INDataPhase(status, pkt.wLength);
        }
    case Request::setFeature:
    case Request::clearFeature:
        {
            if(pkt.direction() != Direction::hostToDevice || pkt.wValue != 0 /* ENDPOINT_HALT */) {
                return false;
            }
            if(pkt.bRequest == Request::setFeature) {
                Endpoint::halt();
            } else {
                Endpoint::clearHalt();
            }
            Derived::acknowledgeSetupRequest();
            using namespace std::string_view_literals;
            UC_LOG_I("USB: EP{:#04x} halt {}",
                     Endpoint::Address,
                     pkt.bRequest == Request::setFeature ? "set"sv : "cleared"sv);
            return true;
        }
    default: return false;
    }
}

// SET_INTERFACE on one interface: alternate setting 0 is the only one, and selecting it resets the
// interface's endpoints (restart).
template<typename Derived,
         typename Restart>
bool handleSetInterface(SetupPacket const& pkt,
                        std::size_t        interfaceNumber,
                        Restart            restart) {
    bool const toInterface = pkt.recipient() == SetupPacket::Recipient::interface;
    if(pkt.type() != SetupPacket::Type::standard || !toInterface
       || pkt.direction() != SetupPacket::Direction::hostToDevice
       || pkt.bRequest != SetupPacket::Request::setInterface || pkt.wIndex != interfaceNumber
       || pkt.wValue != 0)
    {
        return false;
    }
    restart();
    Derived::acknowledgeSetupRequest();
    return true;
}

// The IN half of a bulk function.
//
//   Framed = true    send() is one message. A message that is empty or fills its last packet is
//                    followed by a zero-length packet, so the host's read returns exactly that
//                    message. Message lengths are queued next to the ring, so the next message can
//                    be written while the current one is going out.
//   Framed = false   a byte stream; flush() ends the host's transfer.
//
// The application fills the ring, the interrupt empties it (single producer, single consumer).
// Both DPRAM buffers are kept armed, so the host never has to wait for the interrupt between two
// packets.
//
// Stopping aborts the endpoint and resets its data toggle; sending resumes in abortDone(), which
// wipes the buffer control. A halt clear hands the unsent packets over again and keeps the ring:
// nothing lost, nothing repeated. SET_CONFIGURATION, SET_INTERFACE and a bus reset drop everything.
template<typename Derived, std::size_t EndpointNumber, std::size_t MaxMessageSize, bool Framed>
struct BulkInEndpoint {
    using EP
      = EndpointOps<Derived, EndpointNumber, EndpointDirection::In, EndpointTransferType::Bulk>;

    static constexpr std::uint8_t Address
      = makeEndpointAddress(EndpointDirection::In, static_cast<std::uint8_t>(EndpointNumber));

    // Framed: room for one message being sent and one being written.
    static constexpr std::size_t RingSize = Framed ? 2 * MaxMessageSize + 1 : MaxMessageSize;

    // How many messages may be queued at once.
    static constexpr std::size_t MaxQueuedMessages = 4;

    // How many packets may be with the controller: one per DPRAM buffer.
    static constexpr std::uint8_t BuffersPerEndpoint = EP::AdvancesOnArm ? 2 : 1;

    using RingType  = Kvasir::Atomic::Queue<std::byte, RingSize>;
    using SizeQueue = Kvasir::Atomic::
      Queue<std::uint32_t, MaxQueuedMessages + 1, Kvasir::Atomic::OverFlowPolicyIgnore>;

    static inline RingType ring{};

    // One staging buffer per DPRAM buffer: a packet stays there until it is sent, for a replay.
    static inline std::array<std::array<std::byte, MaxPacketSize>, BuffersPerEndpoint> packets{};
    static inline std::array<std::uint16_t, BuffersPerEndpoint> packetSizes{};
    static inline std::uint8_t nextSlot{};   // where the next packet is put together

    // The packet put together in packets[nextSlot] and not yet handed over.
    static inline std::size_t stagedSize{};
    static inline bool        stagedEndsMessage{false};
    static inline bool        halted{false};
    static inline bool        restartPending{false};
    static inline bool        replayUnsent{false};   // for abortDone(): hand the unsent over again

    // Byte stream only: flush() was called, and whether the last packet was full - only then
    // does the host need a zero-length packet to see the end.
    static inline bool endOfTransferPending{false};
    static inline bool lastPacketFull{false};

    // Framed only: the length of each queued message, and what is left of the one going out.
    // The message stays open until the packet that ends the host's transfer is handed over.
    static inline std::conditional_t<Framed, SizeQueue, std::monostate> messageSizes{};
    static inline std::uint32_t                                         messageBytesLeft{};
    static inline bool                                                  messageOpen{false};

    // Hands the next packet to the controller. Runs in the interrupt, or from the application with
    // the interrupt masked.
    static void armNext() {
        if(halted || restartPending || !Derived::isConfigured()) { return; }
        // Asked of the controller, not counted: two completions can raise a single interrupt.
        auto const free = EP::freeBuffers();
        if(std::ranges::none_of(free, std::identity{})) { return; }
        if(stagedSize == 0) {
            if constexpr(Framed) {
                if(!messageOpen) {
                    if(!messageSizes.pop_into(messageBytesLeft)) { return; }
                    messageOpen = true;
                }
            }
            std::size_t size = std::min(ring.size(), MaxPacketSize);
            if constexpr(Framed) { size = std::min<std::size_t>(size, messageBytesLeft); }
            if(size == 0) {
                endTransfer(free);
                return;
            }
            std::span<std::byte> chunk{packets[nextSlot].data(), size};
            if(!ring.pop_into(chunk)) { return; }
            stagedSize = size;
            if constexpr(Framed) {
                messageBytesLeft -= static_cast<std::uint32_t>(size);
                stagedEndsMessage = messageBytesLeft == 0;
            }
        }
        if(handOver(stagedSize, free)) {
            if constexpr(Framed) {
                // A full last packet leaves the message open until a zero-length packet follows.
                if(stagedEndsMessage && stagedSize != MaxPacketSize) { messageOpen = false; }
            } else {
                lastPacketFull = stagedSize == MaxPacketSize;
            }
            stagedSize = 0;
            // The bytes are in DPRAM now, so the slot is free for the second packet.
            armNext();
        }
    }

    // With nothing left to send: the host sees the end of a transfer in a short packet, so an
    // empty message, or a transfer whose last packet was full, takes a zero-length packet.
    static void endTransfer(typename EP::FreeBuffers const& free) {
        if constexpr(Framed) {
            if(messageOpen && handOver(0, free)) {
                messageOpen = false;
                armNext();
            }
        } else {
            if(!endOfTransferPending) { return; }
            if(lastPacketFull && !handOver(0, free)) { return; }
            endOfTransferPending = false;
            lastPacketFull       = false;
        }
    }

    // Hands what is in the current slot to the controller and moves on to the other slot.
    static bool handOver(std::size_t                     size,
                         typename EP::FreeBuffers const& free) {
        if(!EP::template tryTransfer<true>(std::span{packets[nextSlot]}.first(size), free)) {
            return false;
        }
        packetSizes[nextSlot] = static_cast<std::uint16_t>(size);
        nextSlot              = static_cast<std::uint8_t>((nextSlot + 1) % BuffersPerEndpoint);
        return true;
    }

    // The packets an abort took back, in the order they were armed.
    static void handOverAgain(std::uint8_t count) {
        for(std::uint8_t i = 0; i != count; ++i) {
            std::uint8_t const slot = static_cast<std::uint8_t>(
              (nextSlot + BuffersPerEndpoint - count + i) % BuffersPerEndpoint);
            if(!EP::template tryTransfer<true>(std::span{packets[slot]}.first(packetSizes[slot]))) {
                return;
            }
        }
    }

    static void bufferDone() {
        EP::bufferFinished();
        armNext();
    }

    // Consumer-side flush: safe against the application filling the ring.
    static void dropQueued() {
        std::byte discard{};
        while(ring.pop_into(discard)) {}
        if constexpr(Framed) {
            std::uint32_t size{};
            while(messageSizes.pop_into(size)) {}
            messageOpen      = false;
            messageBytesLeft = 0;
        }
        stagedSize           = 0;
        endOfTransferPending = false;
        lastPacketFull       = false;
    }

    // Takes the endpoint's packets back from the controller.
    static void abortTransfers(bool keepQueued) {
        restartPending = true;
        replayUnsent   = keepQueued;
        EP::abort();
        if(!keepQueued) { dropQueued(); }
    }

    static void stop() { abortTransfers(false); }

    static void busReset() {
        halted         = false;
        restartPending = false;
        replayUnsent   = false;
        dropQueued();
        EP::abort();
        EP::reset();
    }

    static void abortDone() {
        // Only what the controller still holds was never sent.
        auto const unsent = static_cast<std::uint8_t>(EP::armedBuffers());
        EP::abortDone();
        EP::rewindArmed(unsent);
        EP::resetPid();
        std::uint8_t const replay = std::exchange(replayUnsent, false) ? unsent : 0;
        if(std::exchange(restartPending, false) && Derived::isConfigured() && !halted) {
            handOverAgain(replay);
            armNext();
        }
    }

    static void halt() {
        halted = true;
        EP::stall();
    }

    static void clearHalt() {
        halted = false;
        EP::clearStall();
        abortTransfers(true);
    }

    // ---- what the application calls ----------------------------------------------------------

    static std::size_t writeAvailable() { return ring.max_size() - ring.size(); }

    // A byte stream: takes as much as the ring has room for and returns how much that was.
    static std::size_t write(std::span<std::byte const> data)
        requires(!Framed)
    {
        auto const chunk = data.first(std::min(writeAvailable(), data.size()));
        if(!chunk.empty()) { ring.push(chunk); }
        Derived::withIsrMasked([] { armNext(); });
        return chunk.size();
    }

    // Ends the host's current transfer once everything written so far has gone out.
    static void flush()
        requires(!Framed)
    {
        Derived::withIsrMasked([] {
            endOfTransferPending = true;
            armNext();
        });
    }

    // Whether a whole message fits right now.
    static bool isSendReady() {
        if(halted || !Derived::isConfigured()) { return false; }
        if constexpr(Framed) {
            return writeAvailable() >= MaxMessageSize && messageSizes.size() < MaxQueuedMessages;
        } else {
            return writeAvailable() != 0;
        }
    }

    // One message, all of it or none of it.
    static bool send(std::span<std::byte const> data) {
        if(data.size() > MaxMessageSize) { return false; }
        return Derived::withIsrMasked([&] {
            if constexpr(Framed) {
                if(writeAvailable() < data.size() || messageSizes.size() >= MaxQueuedMessages) {
                    return false;
                }
                if(!data.empty()) { ring.push(data); }
                // The bytes are in the ring before the interrupt learns of the message.
                messageSizes.push(static_cast<std::uint32_t>(data.size()));
            } else {
                if(writeAvailable() < data.size()) { return false; }
                if(!data.empty()) { ring.push(data); }
            }
            armNext();
            return true;
        });
    }
};

// The OUT half of a bulk function: a receive queue with back-pressure. The endpoint is armed only
// while the queue has room for a packet; otherwise the host is NAKed until the application reads.
//
// A halt clear keeps the queue: those bytes were acknowledged. A reconfiguration or a bus reset
// empties it, but the interrupt (the producer) only requests that; the reading side does it, and
// the endpoint is not armed again before.
template<typename Derived, std::size_t EndpointNumber, std::size_t RecvBufferSize>
struct BulkOutEndpoint {
    using EP
      = EndpointOps<Derived, EndpointNumber, EndpointDirection::Out, EndpointTransferType::Bulk>;

    static constexpr std::uint8_t Address
      = makeEndpointAddress(EndpointDirection::Out, static_cast<std::uint8_t>(EndpointNumber));

    using QueueType = Kvasir::Atomic::Queue<std::byte, RecvBufferSize>;

    static inline QueueType         queue{};
    static inline std::atomic<bool> paused{false};
    static inline std::atomic<bool> flushPending{false};
    static inline bool              halted{false};
    static inline bool              restartPending{false};

    static bool hasRoom() { return queue.max_size() - queue.size() >= MaxPacketSize; }

    static void arm() {
        if(!flushPending && hasRoom()) {
            paused = false;
            EP::armReceive(MaxPacketSize);
        } else {
            paused = true;
        }
    }

    static void abortTransfers(bool keepQueued) {
        restartPending = true;
        paused         = false;
        if(!keepQueued) { flushPending = true; }
        EP::abort();
        EP::resetPid();
    }

    static void stop() { abortTransfers(false); }

    static void busReset() {
        halted         = false;
        paused         = false;
        restartPending = false;
        flushPending   = true;
        EP::abort();
        EP::reset();
    }

    static void abortDone() {
        EP::abortDone();
        if(std::exchange(restartPending, false) && Derived::isConfigured() && !halted) { arm(); }
    }

    static void halt() {
        halted = true;
        EP::stall();
    }

    static void clearHalt() {
        halted = false;
        EP::clearStall();
        abortTransfers(true);
    }

    static void bufferDone() {
        std::array<std::byte, MaxPacketSize> tempBuffer{};
        std::size_t const                    len = EP::readCurrentBuffer(tempBuffer);
        queue.push(std::span{tempBuffer.data(), len});
        EP::bufferFinished();
        arm();
    }

    // The reading side, before every access: performs a requested flush, and arms a paused
    // endpoint again once there is room.
    static void service() {
        if(flushPending.load()) {
            std::byte discard{};
            while(queue.pop_into(discard)) {}
            flushPending = false;
        }
        if(!paused.load()) { return; }
        Derived::withIsrMasked([] {
            if(paused && !halted && !restartPending && !flushPending && Derived::isConfigured()
               && hasRoom())
            {
                arm();
            }
        });
    }
};

// What getRecvBuffer() hands out: the receive queue's reading side (a single consumer), which
// flushes the queue when the endpoint was stopped and lets a paused endpoint take data again.
template<typename Out>
struct RecvQueue {
    bool pop_into(std::byte& out) {
        Out::service();
        bool const got = Out::queue.pop_into(out);
        if(got) { Out::service(); }
        return got;
    }

    template<typename Range>
    bool pop_into(Range& range) {
        Out::service();
        bool const got = Out::queue.pop_into(range);
        if(got) { Out::service(); }
        return got;
    }

    std::size_t size() const {
        Out::service();
        return Out::queue.size();
    }

    bool empty() const { return size() == 0; }

    constexpr std::size_t max_size() const { return Out::queue.max_size(); }
};

// A bulk function: BulkInEndpoint, plus BulkOutEndpoint when Bidirectional.
// Config::SendBufferSize sizes the send ring (framed: the largest message),
// Config::RecvBufferSize the receive queue.
template<typename Clock,
         typename Config,
         typename Derived,
         typename Mixin,
         std::size_t EndpointNumber,
         bool        Framed,
         bool        Bidirectional>
struct BulkDataAdapter {
private:
    friend Mixin;

    static constexpr auto SendBufferSize = [] {
        if constexpr(requires { Config::SendBufferSize; }) {
            return Config::SendBufferSize;
        } else {
            return 4096;
        }
    }();

    static constexpr auto RecvBufferSize = [] {
        if constexpr(requires { Config::RecvBufferSize; }) {
            return Config::RecvBufferSize;
        } else {
            return 4096;
        }
    }();

    using In = BulkInEndpoint<Derived, EndpointNumber, SendBufferSize, Framed>;
    using Out
      = BulkOutEndpoint<Derived, EndpointNumber, Bidirectional ? RecvBufferSize : std::size_t{2}>;

    static inline RecvQueue<Out> recvQueue{};

    // Callbacks (called by MixinBases)
    static bool EndpointHandlerCallback(std::size_t epNum,
                                        bool        in) {
        if(epNum != EndpointNumber) { return false; }
        if(in) {
            In::bufferDone();
            return true;
        }
        if constexpr(Bidirectional) {
            Out::bufferDone();
            return true;
        }
        return false;
    }

    static bool AbortDoneCallback(std::size_t epNum,
                                  bool        in) {
        if(epNum != EndpointNumber) { return false; }
        if(in) {
            In::abortDone();
            return true;
        }
        if constexpr(Bidirectional) {
            Out::abortDone();
            return true;
        }
        return false;
    }

    static bool SetupPacketRequestCallback(SetupPacket const& pkt) {
        if(handleEndpointRequest<Derived, In>(pkt)) { return true; }
        if constexpr(Bidirectional) { return handleEndpointRequest<Derived, Out>(pkt); }
        return false;
    }

    static void ResetCallback() {
        In::busReset();
        if constexpr(Bidirectional) { Out::busReset(); }
    }

    // SET_CONFIGURATION resets the data toggles whatever the value.
    static void ConfiguredCallback(std::uint8_t) { restart(); }

    static void SetupEndpointsCallback() {
        In::EP::setupEndpoint();
        if constexpr(Bidirectional) { Out::EP::setupEndpoint(); }
    }

public:
    // SET_INTERFACE on the owning interface: the same as a reconfiguration.
    static void restart() {
        In::stop();
        if constexpr(Bidirectional) { Out::stop(); }
    }

    // ---- what the application calls ----------------------------------------------------------

    /// Whether send() would take a whole message (framed) or anything at all (stream).
    static bool isSendReady() { return In::isSendReady(); }

    /// One message (framed) or a block of the stream: all of it or none of it.
    static bool send(std::span<std::byte const> data) { return In::send(data); }

    /// Byte stream only: takes as much as there is room for and says how much that was.
    static std::size_t write(std::span<std::byte const> data)
        requires(!Framed)
    {
        return In::write(data);
    }

    static std::size_t writeAvailable() { return In::writeAvailable(); }

    /// Byte stream only: ends the host's current transfer once everything written so far has gone
    /// out. A framed endpoint does that at the end of every message by itself.
    static void flush()
        requires(!Framed)
    {
        In::flush();
    }

    static auto& getRecvBuffer()
        requires Bidirectional
    {
        return recvQueue;
    }

    /// Where the IN side stands, for diagnostics.
    struct SendDiagnostics {
        std::uint32_t queuedBytes;   // written, still waiting in the ring
        std::uint16_t heldBytes;     // taken out of the ring, waiting for a free buffer
        bool          armed;         // a packet is with the controller
        bool          restarting;    // waiting for an abort to finish
        bool          halted;
    };

    static SendDiagnostics sendDiagnostics() {
        return SendDiagnostics{.queuedBytes = static_cast<std::uint32_t>(In::ring.size()),
                               .heldBytes   = static_cast<std::uint16_t>(In::stagedSize),
                               .armed       = In::EP::armedBuffers() != 0,
                               .restarting  = In::restartPending,
                               .halted      = In::halted};
    }
};

}   // namespace Kvasir::USB::detail
