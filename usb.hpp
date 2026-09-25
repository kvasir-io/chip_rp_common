#pragma once

// The USB controller as a backend of kvasir_devices' USB device (kvasir/Devices/USB). Only for a
// firmware that has kvasir_devices on its include path (LIBRARIES kvasir::devices).
#if __has_include(<kvasir/Devices/USB/Device.hpp>)
    #include "usb/backend.hpp"
    #include "usb/resetInterface.hpp"
#endif
