#pragma once

#include "descriptors.hpp"
#include "detail.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <type_traits>

namespace Kvasir::USB::detail {
template<typename Base, std::size_t EP, EndpointDirection Dir, EndpointTransferType Type>
struct EndpointOps {
private:
    using BufferRegs = typename Base::BufferRegs;
    using Regs       = typename Base::Regs;

    struct EPState {
        bool next_pid_data1{false};   // false=DATA0, true=DATA1
        bool next_buffer_b{false};    // false=buffer_0, true=buffer_1

        constexpr void togglePid() { next_pid_data1 = !next_pid_data1; }

        constexpr void toggleBuffer() { next_buffer_b = !next_buffer_b; }

        constexpr void reset() {
            next_pid_data1 = false;
            next_buffer_b  = false;
        }

        constexpr void setSetupStage() { next_pid_data1 = false; }

        constexpr void setDataPhase() { next_pid_data1 = true; }

        constexpr bool pid() const { return next_pid_data1; }

        constexpr std::size_t buffer() const { return next_buffer_b ? 1 : 0; }
    };

public:
    // State
    static inline EPState        state{};
    static constexpr std::size_t ep_num = EP;
    static constexpr bool        IsIn   = (Dir == EndpointDirection::In);

private:
    template<std::size_t Buffer>
    using BufferControlReg = std::conditional_t<
      IsIn,
      std::conditional_t<Buffer == 0,
                         typename BufferRegs::template EP<EP>::IN_BUFFER_CONTROL_0,
                         typename BufferRegs::template EP<EP>::IN_BUFFER_CONTROL_1>,
      std::conditional_t<Buffer == 0,
                         typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL_0,
                         typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL_1>>;

    using BothBuffersControlReg
      = std::conditional_t<IsIn,
                           typename BufferRegs::template EP<EP>::IN_BUFFER_CONTROL,
                           typename BufferRegs::template EP<EP>::OUT_BUFFER_CONTROL>;

    template<std::size_t Buffer,
             bool        Last>
    static void writeBufferControl(bool          pid,
                                   std::uint16_t length) {
        using BC = BufferControlReg<Buffer>;
        apply(write(BC::last, Kvasir::Register::value<std::uint16_t, Last>()),
              write(BC::full, Kvasir::Register::value<std::uint16_t, IsIn>()),
              write(BC::pid, pid ? 1 : 0),
              write(BC::length, length),
              set(BC::available));
    }

    template<std::size_t Buffer,
             bool        Last>
    static void startTransfer(std::span<std::byte const> data,
                              bool                       pid) {
        static_assert(Buffer == 0 || Buffer == 1, "only dual buffered");
        static_assert(IsIn, "span data transfer only for IN endpoints");
        if(!data.empty()) {
            device_memory_memcpy(reinterpret_cast<void*>(getBufferAddress<Buffer>()),
                                 data.data(),
                                 data.size());
        }
        writeBufferControl<Buffer, Last>(pid, static_cast<std::uint16_t>(data.size()));
    }

    template<std::size_t Buffer,
             bool        Last>
    static void startTransfer(std::size_t size,
                              bool        pid) {
        static_assert(Buffer == 0 || Buffer == 1, "only dual buffered");
        static_assert(!IsIn, "size-only transfer only for OUT endpoints");
        writeBufferControl<Buffer, Last>(pid, static_cast<std::uint16_t>(size));
    }

    // Read data from a specific buffer (0 or 1) into destination span
    // Returns the number of bytes read
    template<std::size_t Buffer>
    static std::size_t readBuffer(std::span<std::byte> dest) {
        using namespace std::string_view_literals;
        static_assert(Buffer == 0 || Buffer == 1, "Buffer must be 0 or 1");
        static_assert(!IsIn, "read only on OUT endpoint");

        auto const bufferState = apply(read(BufferControlReg<Buffer>::length),
                                       read(BufferControlReg<Buffer>::full),
                                       read(BufferControlReg<Buffer>::available));

        std::uint16_t transferLength = bufferState[BufferControlReg<Buffer>::length];
        bool const    full           = bufferState[BufferControlReg<Buffer>::full];
        bool const available = !static_cast<bool>(bufferState[BufferControlReg<Buffer>::available]);

        if(!full || !available) {
            UC_LOG_C("Attempted read on empty buffer (EP{} {} buffer{}): full={} available={}",
                     EP,
                     IsIn ? "IN"sv : "OUT"sv,
                     Buffer,
                     full,
                     available);
            return 0;
        }

        if(transferLength != dest.size()) {
            bool report = true;
            if constexpr(EP != 0) { report = transferLength > dest.size(); }

            if(report) {
                UC_LOG_E("Buffer size mismatch (EP{} {} buffer{}): received={} expected={}",
                         EP,
                         IsIn ? "IN"sv : "OUT"sv,
                         Buffer,
                         transferLength,
                         dest.size());
            }

            transferLength
              = std::min<std::uint16_t>(transferLength, static_cast<std::uint16_t>(dest.size()));
        }

        device_memory_memcpy(dest.data(),
                             reinterpret_cast<void const*>(getBufferAddress<Buffer>()),
                             transferLength);

        return transferLength;
    }

    template<std::size_t Buffer>
    static constexpr std::uintptr_t getBufferAddress() {
        static_assert(Buffer == 0 || Buffer == 1, "Buffer must be 0 or 1");
        using B = typename BufferRegs::template DOUBLEBUFFER<EP>;
        // EP0 IN and OUT share the same buffer
        if constexpr(IsIn || EP == 0) {
            return B::template IN<Buffer>::Addr::value;
        } else {
            return B::template OUT<Buffer>::Addr::value;
        }
    }

    static std::array<bool,
                      2>
    getBufferAvailable() {
        auto const av = apply(read(BothBuffersControlReg::available_0),
                              read(BothBuffersControlReg::available_1));
        return {!static_cast<bool>(get<0>(av)), !static_cast<bool>(get<1>(av))};
    }

    static void resetBufferSelect() { apply(set(BothBuffersControlReg::reset)); }

public:
    template<bool Last,
             typename TransferType>
    static bool tryTransfer(TransferType const& data) {
        using namespace std::string_view_literals;
        auto const buffersAvailable = getBufferAvailable();

        // Both buffers busy?
        if(!buffersAvailable[0] && !buffersAvailable[1]) {
            UC_LOG_W("EP{} {}: Both buffers busy, cannot transfer", EP, IsIn ? "IN"sv : "OUT"sv);
            return false;
        }

        std::size_t const expectedBuffer = state.buffer();

        // Try expected
        if(buffersAvailable[expectedBuffer]) {
            if(expectedBuffer == 0) {
                startTransfer<0, Last>(data, state.pid());
            } else {
                startTransfer<1, Last>(data, state.pid());
            }
            state.toggleBuffer();
            state.togglePid();
            return true;
        }

        // Expected buffer busy
        UC_LOG_W("EP{} {}: Buffer {} not available (buffer0={}, buffer1={})",
                 EP,
                 IsIn ? "IN"sv : "OUT"sv,
                 expectedBuffer,
                 buffersAvailable[0] ? "free"sv : "busy"sv,
                 buffersAvailable[1] ? "free"sv : "busy"sv);

        return false;
    }

    // Read data from the current buffer based on BUFF_CPU_SHOULD_HANDLE register
    // Returns the number of bytes read
    static std::size_t readCurrentBuffer(std::span<std::byte> dest) {
        static_assert(!IsIn, "read only on OUT endpoint");

        std::uint32_t const buffers = apply(read(Regs::BUFF_CPU_SHOULD_HANDLE::FULLREGISTER));

        static constexpr std::uint32_t EPBitMask = (1 << ((EP * 2) + (IsIn ? 0 : 1)));

        bool const buffer0 = (buffers & EPBitMask) == 0;

        if(buffer0) {
            if(!state.next_buffer_b) {
                UC_LOG_W(
                  "EP{} OUT: Buffer mismatch - HW indicates buffer0 but state expects "
                  "buffer1",
                  EP);
            }
            return readBuffer<0>(dest);
        } else {
            if(state.next_buffer_b) {
                UC_LOG_W(
                  "EP{} OUT: Buffer mismatch - HW indicates buffer1 but state expects "
                  "buffer0",
                  EP);
            }
            return readBuffer<1>(dest);
        }
    }

    template<std::size_t Buffer = 0>
    static constexpr std::uint16_t getBufferOffset() {
        return static_cast<std::uint16_t>(getBufferAddress<Buffer>() - BufferRegs::baseAddr);
    }

    template<bool Last = false>
    static bool armReceive(std::size_t max_size) {
        static_assert(!IsIn, "armReceive only valid for OUT endpoints");
        return tryTransfer<Last>(max_size);
    }

    static void stall() {
        apply(set(BothBuffersControlReg::stall));

        if constexpr(EP == 0) {
            if constexpr(IsIn) {
                apply(set(Regs::EP_STALL_ARM::ep0_in));
            } else {
                apply(set(Regs::EP_STALL_ARM::ep0_out));
            }
        }
    }

    static void reset() { state.reset(); }

    static void clearStall() {
        apply(clear(BothBuffersControlReg::stall));

        if constexpr(EP == 0) {
            if constexpr(IsIn) {
                apply(clear(Regs::EP_STALL_ARM::ep0_in));
            } else {
                apply(clear(Regs::EP_STALL_ARM::ep0_out));
            }
        }
    }

    static void setupEndpoint() {
        using EPReg             = std::conditional_t<IsIn,
                                                     typename BufferRegs::template EP<EP>::IN_CONTROL,
                                                     typename BufferRegs::template EP<EP>::OUT_CONTROL>;
        using RegType           = typename EPReg::ENDPOINT_TYPEValC;
        constexpr auto regValue = []() {
            if constexpr(Type == EndpointTransferType::Bulk) {
                return RegType::bulk;
            } else if constexpr(Type == EndpointTransferType::Interrupt) {
                return RegType::interrupt;
            } else if constexpr(Type == EndpointTransferType::Isochronous) {
                return RegType::isochronous;
            } else {
                return RegType::control;
            }
        }();
        if constexpr(EP != 0) {
            apply(EPReg::overrideDefaults(
              set(EPReg::enable),
              write(regValue),
              set(EPReg::double_buffered),
              set(EPReg::interrupt_per_buff),
              write(EPReg::buffer_address, Kvasir::Register::value<getBufferOffset()>())));
        }
    }
};

// EP0 Control Transfer State (shared between EP0_IN and EP0_OUT)
// USB 2.0 Specification Chapter 8.5.3
struct EP0ControlState {
private:
    ControlStage current_stage{ControlStage::Idle};

    // Transition table: for each destination state, bitmask of valid source states
    // valid_from[to_state] = bitmask where bit N = 1 if transition from state N is valid
    // Bit 0=Idle, Bit 1=Setup, Bit 2=Data, Bit 3=Status, Bit 4=Stall
    static constexpr std::array<std::uint8_t, 5> valid_from = {
      0b01000,   // [0] to Idle:   valid from Status (bit 3)
      0b11111,   // [1] to Setup:  valid from any state (bits 0-4)
      0b00010,   // [2] to Data:   valid from Setup (bit 1)
      0b00110,   // [3] to Status: valid from Setup or Data (bits 1-2)
      0b00010,   // [4] to Stall:  valid from Setup (bit 1)
    };

public:
    constexpr void transition(ControlStage new_stage) {
        auto const valid_mask  = valid_from[std::to_underlying(new_stage)];
        auto const current_bit = 1 << std::to_underlying(current_stage);

        if(!(valid_mask & current_bit)) {
            UC_LOG_E("Invalid EP0 stage transition {} -> {}", current_stage, new_stage);
        }

        current_stage = new_stage;
    }

    constexpr ControlStage stage() const { return current_stage; }

    constexpr void reset() { current_stage = ControlStage::Idle; }
};
}   // namespace Kvasir::USB::detail
