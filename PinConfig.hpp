#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace Kvasir { namespace PinConfig {
    enum class ChipVariant { RP2040, RP2350A, RP2350B };

    template<ChipVariant Chip>
    struct ChipTraits;

    template<ChipVariant Chip>
    struct DmaTraits;

    template<ChipVariant Chip>
    struct PwmTraits;

    enum class UartPinType { Tx0, Rx0, Cts0, Rts0, Tx1, Rx1, Cts1, Rts1, Invalid };
    enum class I2cPinType { Sda0, Scl0, Sda1, Scl1, Invalid };
    enum class SpiPinType { Rx0, Tx0, Cs0, Sck0, Rx1, Tx1, Cs1, Sck1, Invalid };

    template<ChipVariant Chip>
    struct UartPinMap {
        static constexpr std::array<UartPinType, ChipTraits<Chip>::pinCount> pins = {};
    };

    template<ChipVariant Chip>
    struct I2cPinMap {
        static constexpr std::array<I2cPinType, ChipTraits<Chip>::pinCount> pins = {};
    };

    template<ChipVariant Chip>
    struct SpiPinMap {
        static constexpr std::array<SpiPinType, ChipTraits<Chip>::pinCount> pins = {};
    };

    struct PwmChannelInfo {
        std::uint32_t channel;
        bool          isChannelA;
    };

    template<ChipVariant Chip,
             int         Pin>
    constexpr PwmChannelInfo getPwmChannelInfo() {
        if constexpr(Chip == ChipVariant::RP2040 || Chip == ChipVariant::RP2350A) {
            static_assert(Pin >= 0 && Pin < 30, "Pin number out of range for RP2040/RP2350A");
        } else if constexpr(Chip == ChipVariant::RP2350B) {
            static_assert(Pin >= 0 && Pin < 48, "Pin number out of range for RP2350B");
        }

        // GPIO 0-31 map to PWM channels 0-7 (repeating every 16 pins)
        // GPIO 32-47 map to PWM channels 8-11 (repeating every 8 pins)
        constexpr auto channel = [] {
            if constexpr(Pin < 32) {
                return static_cast<std::uint32_t>((Pin % 16) / 2);
            } else {
                return static_cast<std::uint32_t>(8 + ((Pin % 8) / 2));
            }
        }();
        constexpr bool isChannelA = (Pin % 2) == 0;
        return {.channel = channel, .isChannelA = isChannelA};
    }

    template<ChipVariant Chip,
             unsigned    Instance>
    constexpr bool isValidUartPin(int         pin,
                                  UartPinType expectedType) {
        if(pin < 0 || pin >= static_cast<int>(ChipTraits<Chip>::pinCount)) { return false; }

        using PinElement
          = std::remove_cv_t<std::remove_reference_t<decltype(UartPinMap<Chip>::pins[0])>>;

        bool matches = false;
        if constexpr(std::is_same_v<PinElement, UartPinType>) {
            matches = (UartPinMap<Chip>::pins[static_cast<std::size_t>(pin)] == expectedType);
        } else {
            auto [first, second] = UartPinMap<Chip>::pins[static_cast<std::size_t>(pin)];
            matches              = (first == expectedType || second == expectedType);
        }

        if(Instance == 0) {
            return matches
                && (expectedType == UartPinType::Tx0 || expectedType == UartPinType::Rx0
                    || expectedType == UartPinType::Cts0 || expectedType == UartPinType::Rts0);
        }
        if(Instance == 1) {
            return matches
                && (expectedType == UartPinType::Tx1 || expectedType == UartPinType::Rx1
                    || expectedType == UartPinType::Cts1 || expectedType == UartPinType::Rts1);
        }
        return false;
    }

    template<ChipVariant Chip,
             unsigned    Instance>
    constexpr bool isValidI2cPin(int        pin,
                                 I2cPinType expectedType) {
        if(pin < 0 || pin >= static_cast<int>(ChipTraits<Chip>::pinCount)) { return false; }

        auto pinType = I2cPinMap<Chip>::pins[static_cast<std::size_t>(pin)];
        if(Instance == 0) {
            return pinType == expectedType
                && (expectedType == I2cPinType::Sda0 || expectedType == I2cPinType::Scl0);
        }
        if(Instance == 1) {
            return pinType == expectedType
                && (expectedType == I2cPinType::Sda1 || expectedType == I2cPinType::Scl1);
        }
        return false;
    }

    template<ChipVariant Chip,
             unsigned    Instance>
    constexpr bool isValidSpiPin(int        pin,
                                 SpiPinType expectedType) {
        if(pin < 0 || pin >= static_cast<int>(ChipTraits<Chip>::pinCount)) { return false; }

        auto pinType = SpiPinMap<Chip>::pins[static_cast<std::size_t>(pin)];
        if(Instance == 0) {
            return pinType == expectedType
                && (expectedType == SpiPinType::Rx0 || expectedType == SpiPinType::Tx0
                    || expectedType == SpiPinType::Cs0 || expectedType == SpiPinType::Sck0);
        }
        if(Instance == 1) {
            return pinType == expectedType
                && (expectedType == SpiPinType::Rx1 || expectedType == SpiPinType::Tx1
                    || expectedType == SpiPinType::Cs1 || expectedType == SpiPinType::Sck1);
        }
        return false;
    }

}}   // namespace Kvasir::PinConfig
