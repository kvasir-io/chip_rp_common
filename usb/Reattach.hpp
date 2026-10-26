#pragma once
// The RP2040 / RP2350 USB device off the bus and on again, for a host that gave up enumerating it while clk_usb did not
// run at 48 MHz: a degraded boot runs clk_usb from the ring oscillator or the crystal (DefaultClockSettings::
// degradedClockInit), and a host that met the device then does not try again by itself. Clocks::DegradedBoot
// (DegradedBoot.hpp) runs it after its switch-up, when clk_usb runs from PLL_USB again, through its hook
// Clocks::AfterSwitchUp, which this header specialises (included with the USB backend, usb/backend.hpp: an image
// without the backend carries none of it); the device's own state needs nothing - the host's enumeration starts with a
// bus reset (Device::busReset).
//
// The device announces itself by its D+ pull-up ("the DP pin is pulled up to indicate to the host that a Full Speed
// device has been connected", RP2040 datasheet 4.1.2 md l.17983; SIE_CTRL.PULLUP_EN, RP2040 md l.18781, RP2350 md
// l.56582): released, the host sees the device go; set again after `pause`, it sees a new device and enumerates it.
// Registers only (SIE_CTRL through the field API; the controller's interrupt handler does not write PULLUP_EN), from
// thread mode.
#include "../DegradedBoot.hpp"   // the hook Clocks::AfterSwitchUp
#include "chip/Blocks.hpp"       // the chip's block table (Rp::Block)
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/USB.hpp"
#include "peripherals/USB_DPRAM.hpp"

#include <chrono>

namespace Kvasir::USB::Rp {
struct Reattach {
    using Regs = Kvasir::Peripheral::USB::Registers<0>;

    /// How long the pull-up stays released. The USB 2.0 specification's disconnect and attach timings are not in the
    /// datasheets this package cites: chosen, not cited - long against a disconnect a hub sees in microseconds, short
    /// against a host's attach debounce, so the device is back within the debounce it waits anyway.
    static constexpr std::chrono::milliseconds pause{200};

    /// The pull-up released if the device had it on (the controller out of reset, Device::connect() ran): true then,
    /// and end() puts it back after `pause`. False, touching nothing, for an image without a USB device.
    [[nodiscard]] static bool begin() {
        // a block held in reset answers nothing (RESETS before the block's registers)
        if(!apply(read(::Kvasir::Rp::Block<Kvasir::Peripheral::USB_DPRAM::Registers<>>::resetDone)))
        {
            return false;
        }
        if(!apply(read(Regs::SIE_CTRL::pullup_en))) { return false; }
        apply(clear(Regs::SIE_CTRL::pullup_en));
        return true;
    }

    /// The pull-up on again: the host enumerates the device afresh.
    static void end() { apply(set(Regs::SIE_CTRL::pullup_en)); }
};
}   // namespace Kvasir::USB::Rp

/// The RP USB driver's part of a degraded boot's switch-up (Clocks::AfterSwitchUp, DegradedBoot.hpp).
template<typename... Ts>
struct Kvasir::Clocks::AfterSwitchUp<Kvasir::Tag::User, Ts...> {
    using type = Kvasir::USB::Rp::Reattach;
};
