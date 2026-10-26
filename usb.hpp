#pragma once

// The USB controller as a backend of kvasir_devices' USB device (kvasir/Devices/USB): only for an
// image that links kvasir::bus or kvasir::devices. A shared header that serves images without it asks
// `#if defined(KVASIR_DEVICES)` (kvasir::bus's definition) before it includes this one.
#include "usb/backend.hpp"
#include "usb/resetInterface.hpp"
