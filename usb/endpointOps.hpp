#pragma once

#include "detail.hpp"
#include "kvasir/Util/RateLimiter.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <kvasir/Devices/USB/Descriptors.hpp>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>

namespace Kvasir::USB::detail {
// One endpoint of the RP2040 / RP2350 controller: its buffer control, its DPRAM buffers and the
// data toggle, which this controller leaves to software. What kvasir/Devices/USB/Backend.hpp asks
// of an endpoint, plus what only this controller can do (abort(), abortDone(), rewindArmed(),
// cancelTransfer(), state) - public for an application that drives the DPRAM itself.
template<typename Base, std::size_t EP, EndpointDirection Dir, EndpointTransferType Type>
struct EndpointOps {
private:
    using BufferRegs = typename Base::BufferRegs;
    using Regs       = typename Base::Regs;

    // EP0 is always single buffered: a transfer the host abandons leaves the controller's buffer
    // selection where software cannot put it back.
    static constexpr bool DoubleBuffered = Base::DoubleBuffered && EP != 0;

    struct EPState {
        bool next_pid_data1{};   // false=DATA0, true=DATA1

        using bufferType = std::conditional_t<DoubleBuffered, bool, std::monostate>;

        [[no_unique_address]] bufferType next_buffer_b{};   // false=buffer_0, true=buffer_1

        constexpr void togglePid() { next_pid_data1 = !next_pid_data1; }

        constexpr void toggleBuffer() {
            if constexpr(DoubleBuffered) { next_buffer_b = !next_buffer_b; }
        }

        constexpr void resetBuffer() {
            if constexpr(DoubleBuffered) { next_buffer_b = false; }
        }

        constexpr void resetPid() { next_pid_data1 = false; }

        constexpr void reset() {
            resetBuffer();
            resetPid();
        }

        constexpr void setSetupStage() { next_pid_data1 = false; }

        constexpr void setDataPhase() { next_pid_data1 = true; }

        constexpr bool pid() const { return next_pid_data1; }

        constexpr std::size_t buffer() const {
            if constexpr(DoubleBuffered) {
                return next_buffer_b ? 1 : 0;
            } else {
                return 0;
            }
        }
    };

public:
    // State
    static inline EPState        state{};
    static constexpr std::size_t ep_num = EP;
    static constexpr bool        IsIn   = (Dir == EndpointDirection::In);

    // The buffer and PID cursor advance when a packet is armed, on every endpoint: a
    // double-buffered IN endpoint has to (its second packet is armed behind the first), and with
    // all of them alike nobody has to be told when a packet is over. A packet that is taken back
    // unsent moves the cursor back with it (rewindArmed).
    static constexpr bool AdvancesOnArm = true;

    // How many packets may be with the controller at once.
    static constexpr std::size_t QueueDepth = IsIn && DoubleBuffered ? 2 : 1;

    static constexpr bool AsyncCancel = Base::AsyncCancel;

    enum class Fault : std::uint8_t {
        readEmptyBuffer = 1,
        sizeMismatch,
        allBuffersBusy,
        bufferNotAvailable,
        bufferMismatch,
    };
    // Fault logging goes through this: a stalled host or a pulled cable repeats
    // these per packet.
    static inline Kvasir::RateLimiter<typename Base::ClockType> faultLog_{};

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

    static constexpr std::uint32_t EPBitMask = (1U << ((EP * 2) + (IsIn ? 0 : 1)));

    template<std::size_t Buffer,
             bool        Last>
    static void writeBufferControl(bool          pid,
                                   std::uint16_t length) {
        using BC = BufferControlReg<Buffer>;

        static constexpr bool delayAvailable =
#if __has_include("chip/rp2350.hpp")
          false;
#else
          true;
#endif
        BC::overrideDefaultsRuntime(
          write(BC::last, Kvasir::Register::value<std::uint16_t, Last>()),
          write(BC::full, Kvasir::Register::value<std::uint16_t, IsIn>()),
          write(BC::pid, pid ? 1 : 0),
          write(BC::length, length),
          write(BC::available, Kvasir::Register::value<std::uint16_t, delayAvailable ? 0 : 1>()));

        if constexpr(delayAvailable) {
            BC::overrideDefaultsRuntime(
              write(BC::last, Kvasir::Register::value<std::uint16_t, Last>()),
              write(BC::full, Kvasir::Register::value<std::uint16_t, IsIn>()),
              write(BC::pid, pid ? 1 : 0),
              write(BC::length, length),
              set(BC::available));
        }
    }

    static void clearBufferControl() {
        if constexpr(DoubleBuffered) {
            apply(BothBuffersControlReg::overrideDefaults());
        } else {
            apply(BufferControlReg<0>::overrideDefaults());
        }
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
            KVASIR_LOG_LIMITED(
              faultLog_.allow(Kvasir::rateLimitKey(Fault::readEmptyBuffer, Buffer)),
              UC_LOG_C,
              "Attempted read on empty buffer (EP{} {} buffer{}): full={} available={}",
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
                KVASIR_LOG_LIMITED(
                  faultLog_.allow(Kvasir::rateLimitKey(Fault::sizeMismatch, Buffer)),
                  UC_LOG_E,
                  "Buffer size mismatch (EP{} {} buffer{}): received={} expected={}",
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

public:
    // Per buffer, whether it is free: not with the controller.
    using FreeBuffers = std::array<bool, DoubleBuffered ? 2 : 1>;

    static FreeBuffers freeBuffers() {
        if constexpr(DoubleBuffered) {
            auto const av = apply(read(BothBuffersControlReg::available_0),
                                  read(BothBuffersControlReg::available_1));
            return {!static_cast<bool>(get<0>(av)), !static_cast<bool>(get<1>(av))};
        } else {
            auto const av = apply(read(BufferControlReg<0>::available));
            return {!static_cast<bool>(get<0>(av))};
        }
    }

    template<bool Last,
             typename TransferType>
    static bool tryTransfer(TransferType const& data) {
        return tryTransfer<Last>(data, freeBuffers());
    }

    // For a caller that already has freeBuffers(): saves a register read per packet.
    template<bool Last,
             typename TransferType>
    static bool tryTransfer(TransferType const& data,
                            FreeBuffers const&  buffersAvailable) {
        using namespace std::string_view_literals;

        // All buffers busy?
        if(std::ranges::none_of(buffersAvailable, [](bool av) { return av; })) {
            KVASIR_LOG_LIMITED(faultLog_.allow(Kvasir::rateLimitKey(Fault::allBuffersBusy)),
                               UC_LOG_W,
                               "EP{} {}: All buffers busy, cannot transfer (buffers_av: {}) {}",
                               EP,
                               IsIn ? "IN"sv : "OUT"sv,
                               buffersAvailable,
                               BothBuffersControlReg{});
            return false;
        }

        std::size_t const expectedBuffer = state.buffer();

        // Try expected
        if(buffersAvailable[expectedBuffer]) {
            bool const pidToUse = state.pid();

            if constexpr(DoubleBuffered) {
                if(expectedBuffer == 0) {
                    startTransfer<0, Last>(data, pidToUse);
                } else {
                    startTransfer<1, Last>(data, pidToUse);
                }
            } else {
                startTransfer<0, Last>(data, pidToUse);
            }
            state.toggleBuffer();
            state.togglePid();
            return true;
        }

        // Expected buffer busy
        KVASIR_LOG_LIMITED(
          faultLog_.allow(Kvasir::rateLimitKey(Fault::bufferNotAvailable, expectedBuffer)),
          UC_LOG_W,
          "EP{} {}: Buffer {} not available (buffers_av: {})",
          EP,
          IsIn ? "IN"sv : "OUT"sv,
          expectedBuffer,
          buffersAvailable);

        return false;
    }

    // How many buffers the controller still holds. A software count drifts: two completions can
    // raise a single interrupt.
    static std::size_t armedBuffers() {
        return static_cast<std::size_t>(std::ranges::count(freeBuffers(), false));
    }

    // Read data from the current buffer based on BUFF_CPU_SHOULD_HANDLE register
    // Returns the number of bytes read
    static std::size_t readCurrentBuffer(std::span<std::byte> dest) {
        static_assert(!IsIn, "read only on OUT endpoint");

        if constexpr(DoubleBuffered) {
            std::uint32_t const buffers = apply(read(Regs::BUFF_CPU_SHOULD_HANDLE::FULLREGISTER));

            bool const buffer0 = (buffers & EPBitMask) == 0;

            // The cursor has moved on to the other buffer when this one was armed.
            if(buffer0) {
                if(state.buffer() != 1) {
                    KVASIR_LOG_LIMITED(
                      faultLog_.allow(Kvasir::rateLimitKey(Fault::bufferMismatch, 0)),
                      UC_LOG_W,
                      "EP{} OUT: Buffer mismatch - HW indicates buffer0 but state expects "
                      "buffer1",
                      EP);
                    return 0;
                }
                return readBuffer<0>(dest);
            } else {
                if(state.buffer() != 0) {
                    KVASIR_LOG_LIMITED(
                      faultLog_.allow(Kvasir::rateLimitKey(Fault::bufferMismatch, 1)),
                      UC_LOG_W,
                      "EP{} OUT: Buffer mismatch - HW indicates buffer1 but state expects "
                      "buffer0",
                      EP);
                    return 0;
                }
                return readBuffer<1>(dest);
            }
        } else {
            return readBuffer<0>(dest);
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

    static void abort() {
        std::uint32_t const aborts = apply(read(Regs::EP_ABORT::FULLREGISTER));
        apply(write(Regs::EP_ABORT::FULLREGISTER, aborts | EPBitMask));
    }

    static void abortDone() {
        std::uint32_t const aborts = apply(read(Regs::EP_ABORT::FULLREGISTER));
        apply(write(Regs::EP_ABORT::FULLREGISTER, aborts & ~EPBitMask));
        clearBufferControl();
    }

    // Drops whatever is armed (EP0, when a SETUP arrives). The buffer selection stays: an
    // abandoned buffer never completed, so nothing toggled it.
    static void cancelTransfer() { clearBufferControl(); }

    // Takes the armed packets back. The controller answers with abort_done, and there
    // cancelComplete() finishes it.
    static void cancel() { abort(); }

    // How many armed packets were never sent (IN), with the cursor that advanced on arming moved
    // back over them.
    static std::size_t cancelComplete() {
        // Only what the controller still holds was never sent (or, OUT, never filled).
        std::size_t const unsent = armedBuffers();
        abortDone();
        rewindArmed(unsent);
        return unsent;
    }

    static void reset() { state.reset(); }

    static void resetDataToggle() { state.resetPid(); }

    // Aborted packets were never sent: a cursor that advanced on arming moves back with them.
    static void rewindArmed(std::size_t count) {
        for(std::size_t i = 0; i != count; ++i) {
            state.toggleBuffer();
            state.togglePid();
        }
    }

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
        using EPReg   = std::conditional_t<IsIn,
                                           typename BufferRegs::template EP<EP>::IN_CONTROL,
                                           typename BufferRegs::template EP<EP>::OUT_CONTROL>;
        using RegType = typename EPReg::ENDPOINT_TYPEValC;
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
              write(EPReg::double_buffered, Kvasir::Register::value<DoubleBuffered ? 1 : 0>()),
              set(EPReg::interrupt_per_buff),
              write(EPReg::buffer_address, Kvasir::Register::value<getBufferOffset()>())));
        }
    }
};
}   // namespace Kvasir::USB::detail
