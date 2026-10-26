#pragma once
// The image config header as a test fixture: what a firmware's kvasir_config.hpp holds - the chip variant, the log
// backend and a complete log clock. uc_log/uc_log.hpp reads it first (UC_LOG_CONFIG_HEADER, set by
// kvasir_conformance_project), and every conformance TU and every header-alone TU starts with it. Like every config
// header it never includes uc_log/uc_log.hpp. No board: nothing here names pins or a probe.

// the chip variant first: the chip headers (core/Systick.hpp, through SystickLogClock.hpp below) read it
#include <chip/rp_common/PinConfig.hpp>

// Kvasir::Chip::part comes from the chip package (CONFORMANCE_CHIP, CMakeLists.txt): chip_rp2040 passes
// KVASIR_CHIP_RP2040; chip_rp2350 KVASIR_CHIP_RP2350A or KVASIR_CHIP_RP2350B, from its KVASIR_RP2350_MPU, which
// CONFORMANCE_VARIANT sets

#include <chip/rp_common/WaitBounds.hpp>   // Register::CpuHzCeiling<Tag::User>: the loss policy's CPU ceiling
#include <uc_log/AlwaysAttached.hpp>
#include <uc_log/LogClock.hpp>
#include <uc_log/RttLog.hpp>

namespace uc_log {
template<>
struct ComBackend<uc_log::Tag::User>
  : public uc_log::RttLog<
      uc_log::AlwaysAttached,
      uc_log::Rings<1024>,
      uc_log::IsrPolicy::MaskedRecord,
      uc_log::LossPolicy::Counted<Kvasir::Register::CpuHzCeiling<Kvasir::Tag::User>::value>> {};
}   // namespace uc_log

#include <kvasir/Log/SystickLogClock.hpp>

namespace Conformance {
#ifdef KVASIR_CHIP_RP2040
inline constexpr auto clockSpeedValue = 125'000'000;
#else
inline constexpr auto clockSpeedValue = 150'000'000;
#endif

using SystickClock = Kvasir::Log::SystickLogClock<clockSpeedValue>;
}   // namespace Conformance

namespace uc_log {
template<>
struct LogClock<uc_log::Tag::User> : public Conformance::SystickClock {};
}   // namespace uc_log
