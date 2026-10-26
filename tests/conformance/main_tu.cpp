// The conformance tree's main TU (Kvasir_SDK tests/conformance/conformance.cmake MAIN_TU): an image's main TU in the
// order a main may choose - kvasir/StartUp/StartUp.hpp FIRST, the chip after it - with a Startup instantiated, so every
// late hook (FirstInitStep, the fault handler's interrupt traits, the NVIC clear-all's traits, the chip's entries) is
// compiled behind the SDK's header. Linked with a second TU (chip/chip.hpp + chip/drivers.hpp) into one object, and
// with a copy of itself, which must fail on the one-TU rule's guard. Never run.
// main() does a read-modify-write of a std::atomic. RP2040: with MAIN_TU_DEFINITIONS KVASIR_MULTICORE=1 (CMakeLists.txt)
// the atomic shim takes the chip's Port::CrossCoreLock, which chip/StartUp.hpp defines only below this TU's
// StartUp.hpp: the hook is read late (Shim::LockT), and that read-modify-write instantiates it. RP2350: an inline
// exclusive on this core, the shim's 8-byte and generic entry points defined all the same
// (kvasir/StartUp/Runtime/LibAtomic.hpp).
// clang-format off: the SDK's Startup header first, on purpose
#include <kvasir/StartUp/StartUp.hpp>
// clang-format on

#include <atomic>
#include <chip/chip.hpp>
#include <cstdint>
#include <kvasir/Runtime/BootReport.hpp>   // after the chip: its reset cause by name (until step 140)
#include <kvasir/Runtime/FaultHandler.hpp>

namespace {
constexpr auto clockSpeedValue = Conformance::clockSpeedValue;
constexpr auto crystalSpeed    = 12'000'000;
using ClockSettings = Kvasir::DefaultClockSettings::Settings<clockSpeedValue, crystalSpeed>;
std::atomic<std::uint32_t> counter{0};
}   // namespace

using Startup = Kvasir::Startup::Startup<ClockSettings,
                                         Conformance::SystickClock,
                                         uc_log::ComBackend<uc_log::Tag::User>,
                                         Kvasir::Boot::Report<>,
                                         Kvasir::Fault::HandlerFor<>>;

int main() {
    while(true) {
        counter.fetch_add(1, std::memory_order_relaxed);
        Startup::runChipEntries<Kvasir::Hook::MainLoop>();
    }
}

template struct Kvasir::Startup::Start<Startup>;
