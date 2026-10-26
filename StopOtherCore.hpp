#pragma once
// Kvasir::Port::StopOtherCore<Tag::User> on the RP2040 and RP2350 (kvasir/Port/Hooks.hpp), in an image that runs code
// on core 1 (KVASIR_MULTICORE; otherwise core 1 sleeps in the boot ROM already):
//
// - on core 0, core 1 is reset: PSM FRCE_OFF.PROC1 set, read back, cleared (RP2040 DS 2.13.5 Table 198, bit 16; RP2350
//   DS 7.4.4 Table 531, bit 24), Multicore::resetCore1(). Core 1 restarts in the boot ROM and sleeps there until a
//   launch. Not left forced off: an RP2350's boot ROM reboot with FRCE_OFF.PROC1 still set never came back (measured
//   on an RP2350A).
// - on core 1, core 0 is parked: its NMI, routed from a forced IO_QSPI interrupt, spins until the reboot
//   (Multicore::Core0Park, with why the PSM cannot stop core 0). Only where the SecondaryCore armed it (listed in core
//   0's Startup list); otherwise core 0 runs on.
//
// Then the atomic shim's cross-core lock is released (Atomic::onOtherCoreStopped): the stopped core may have held it -
// an NMI is not masked by the interrupt mask the lock is taken under - and the fatal path's own atomics need it.
#include "Multicore.hpp"
#include "Sio.hpp"
#include "kvasir/Atomic/Atomic.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"

#ifdef KVASIR_MULTICORE
template<typename... Ts>
struct Kvasir::Port::StopOtherCore<Kvasir::Tag::User, Ts...> {
    void operator()() const {
        if(Kvasir::Sio::cpuId() == 0U) {
            Kvasir::Multicore::resetCore1();
        } else if(!Kvasir::Multicore::Core0Park::park()) {
            return;
        }
        Kvasir::Atomic::onOtherCoreStopped();
    }
};
#endif
