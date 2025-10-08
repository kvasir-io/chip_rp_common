#pragma once

#include <array>
#include <cstdint>

namespace Kvasir { namespace PinConfig {

    enum class ChipVariant { RP2040, RP2350A, RP2350B };

    template<ChipVariant Chip>
    struct ChipTraits;

    template<>
    struct ChipTraits<ChipVariant::RP2040> {
        static constexpr std::size_t pinCount = 30;
    };

    template<>
    struct ChipTraits<ChipVariant::RP2350A> {
        static constexpr std::size_t pinCount = 30;
    };

    template<>
    struct ChipTraits<ChipVariant::RP2350B> {
        static constexpr std::size_t pinCount = 48;
    };

    template<ChipVariant Chip>
    struct DmaTraits;

    template<>
    struct DmaTraits<ChipVariant::RP2040> {
        static constexpr std::size_t channelCount   = 12;
        static constexpr std::size_t interruptCount = 2;
    };

    template<>
    struct DmaTraits<ChipVariant::RP2350A> {
        static constexpr std::size_t channelCount   = 16;
        static constexpr std::size_t interruptCount = 4;
    };

    template<>
    struct DmaTraits<ChipVariant::RP2350B> {
        static constexpr std::size_t channelCount   = 16;
        static constexpr std::size_t interruptCount = 4;
    };

    enum class UartPinType { Tx0, Rx0, Cts0, Rts0, Tx1, Rx1, Cts1, Rts1, Invalid };
    enum class I2cPinType { Sda0, Scl0, Sda1, Scl1, Invalid };
    enum class SpiPinType { Rx0, Tx0, Cs0, Sck0, Rx1, Tx1, Cs1, Sck1, Invalid };

    template<ChipVariant Chip>
    struct UartPinMap {
        static constexpr std::array<UartPinType, ChipTraits<Chip>::pinCount> pins = {};
    };

    template<>
    struct UartPinMap<ChipVariant::RP2040> {
        static constexpr std::array<UartPinType, 30> pins
          = {UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0};
    };

    template<>
    struct UartPinMap<ChipVariant::RP2350A> {
        static constexpr std::array<UartPinType, 30> pins
          = {UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0};
    };

    template<>
    struct UartPinMap<ChipVariant::RP2350B> {
        static constexpr std::array<UartPinType, 48> pins
          = {UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx1, UartPinType::Rx1, UartPinType::Cts1, UartPinType::Rts1,
             UartPinType::Tx0, UartPinType::Rx0, UartPinType::Cts0, UartPinType::Rts0};
    };

    template<ChipVariant Chip>
    struct I2cPinMap {
        static constexpr std::array<I2cPinType, ChipTraits<Chip>::pinCount> pins = {};
    };

    template<>
    struct I2cPinMap<ChipVariant::RP2040> {
        static constexpr std::array<I2cPinType, 30> pins = {
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0,
          I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1,
          I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1,
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0};
    };

    template<>
    struct I2cPinMap<ChipVariant::RP2350A> {
        static constexpr std::array<I2cPinType, 30> pins = {
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0,
          I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1,
          I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1,
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0};
    };

    template<>
    struct I2cPinMap<ChipVariant::RP2350B> {
        static constexpr std::array<I2cPinType, 48> pins = {
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0,
          I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1,
          I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1,
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0,
          I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1,
          I2cPinType::Scl1, I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1,
          I2cPinType::Sda0, I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1, I2cPinType::Sda0,
          I2cPinType::Scl0, I2cPinType::Sda1, I2cPinType::Scl1};
    };

    template<ChipVariant Chip>
    struct SpiPinMap {
        static constexpr std::array<SpiPinType, ChipTraits<Chip>::pinCount> pins = {};
    };

    template<>
    struct SpiPinMap<ChipVariant::RP2040> {
        static constexpr std::array<SpiPinType, 30> pins = {
          SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx0,
          SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx1,  SpiPinType::Cs1,
          SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,  SpiPinType::Cs1,  SpiPinType::Sck1,
          SpiPinType::Tx1,  SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,
          SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx1,
          SpiPinType::Cs1,  SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,  SpiPinType::Cs1};
    };

    template<>
    struct SpiPinMap<ChipVariant::RP2350A> {
        static constexpr std::array<SpiPinType, 30> pins = {
          SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx0,
          SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx1,  SpiPinType::Cs1,
          SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,  SpiPinType::Cs1,  SpiPinType::Sck1,
          SpiPinType::Tx1,  SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,
          SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx1,
          SpiPinType::Cs1,  SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,  SpiPinType::Cs1};
    };

    template<>
    struct SpiPinMap<ChipVariant::RP2350B> {
        static constexpr std::array<SpiPinType, 48> pins = {
          SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx0,
          SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx1,  SpiPinType::Cs1,
          SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,  SpiPinType::Cs1,  SpiPinType::Sck1,
          SpiPinType::Tx1,  SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,
          SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,  SpiPinType::Rx1,
          SpiPinType::Cs1,  SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,  SpiPinType::Cs1,
          SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0,
          SpiPinType::Tx0,  SpiPinType::Rx0,  SpiPinType::Cs0,  SpiPinType::Sck0, SpiPinType::Tx0,
          SpiPinType::Rx1,  SpiPinType::Cs1,  SpiPinType::Sck1, SpiPinType::Tx1,  SpiPinType::Rx1,
          SpiPinType::Cs1,  SpiPinType::Sck1, SpiPinType::Tx1};
    };

    struct PwmChannelInfo {
        std::uint32_t channel;
        bool          isChannelA;
    };

    template<ChipVariant Chip>
    constexpr PwmChannelInfo getPwmChannelInfo(int pin) {
        if constexpr(Chip == ChipVariant::RP2040 || Chip == ChipVariant::RP2350A) {
            if(pin >= 30) {
                return {.channel = 0, .isChannelA = false};
            }
        } else if constexpr(Chip == ChipVariant::RP2350B) {
            if(pin >= 48) {
                return {.channel = 0, .isChannelA = false};
            }
        }

        auto channel    = static_cast<std::uint32_t>((pin % 16) / 2);
        bool isChannelA = (pin % 2) == 0;
        return {.channel = channel, .isChannelA = isChannelA};
    }

    template<ChipVariant Chip,
             unsigned    Instance>
    constexpr bool isValidUartPin(int         pin,
                                  UartPinType expectedType) {
        if(pin < 0 || pin >= static_cast<int>(ChipTraits<Chip>::pinCount)) {
            return false;
        }

        auto pinType = UartPinMap<Chip>::pins[static_cast<std::size_t>(pin)];
        if(Instance == 0) {
            return pinType == expectedType
                && (expectedType == UartPinType::Tx0 || expectedType == UartPinType::Rx0
                    || expectedType == UartPinType::Cts0 || expectedType == UartPinType::Rts0);
        }
        if(Instance == 1) {
            return pinType == expectedType
                && (expectedType == UartPinType::Tx1 || expectedType == UartPinType::Rx1
                    || expectedType == UartPinType::Cts1 || expectedType == UartPinType::Rts1);
        }
        return false;
    }

    template<ChipVariant Chip,
             unsigned    Instance>
    constexpr bool isValidI2cPin(int        pin,
                                 I2cPinType expectedType) {
        if(pin < 0 || pin >= static_cast<int>(ChipTraits<Chip>::pinCount)) {
            return false;
        }

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
        if(pin < 0 || pin >= static_cast<int>(ChipTraits<Chip>::pinCount)) {
            return false;
        }

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
