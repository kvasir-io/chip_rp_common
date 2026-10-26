#pragma once

#include <cstddef>
#include <cstdint>

namespace Kvasir::Chip {
// The RP parts, the chip contract's Chip::Part; Chip::part (below) is the one this translation unit is for.
enum class Part { rp2040, rp2350a, rp2350b };
}   // namespace Kvasir::Chip

namespace Kvasir { namespace PinConfig {

    // What differs between the two chip families beyond pins and pin functions, for the
    // shared drivers to ask instead of assuming the RP2350.
    constexpr bool isRp2040(Chip::Part c) { return c == Chip::Part::rp2040; }

    constexpr unsigned pioCount(Chip::Part c) { return isRp2040(c) ? 2U : 3U; }

    constexpr unsigned timerCount(Chip::Part c) { return isRp2040(c) ? 1U : 2U; }

    // The GPIO function select of the clock outputs (CLOCKS_GPOUTn).
    constexpr int gpoutFunction(Chip::Part c) { return isRp2040(c) ? 8 : 9; }

    // Which CLK_GPOUT generator a pin carries, -1 for none. The RP2040 has 21 / 23 / 24 / 25
    // (RP2040 datasheet 1.4.3 Table 2, F8, md l.646-661); the RP2350 adds 13 / 15 (RP2350
    // datasheet 1.2.3 Table 3, F9, md l.951-968); the B variant's GPIO 30-47 carry none
    // (md l.977-996).
    constexpr int gpoutOf(Chip::Part c,
                          int        pin) {
        switch(pin) {
        case 21: return 0;
        case 23: return 1;
        case 24: return 2;
        case 25: return 3;
        case 13: return isRp2040(c) ? -1 : 0;
        case 15: return isRp2040(c) ? -1 : 1;
        default: return -1;
        }
    }

    // User GPIOs: 30 on the RP2040 (RP2040 datasheet md l.466), 30 on the RP2350A (QFN-60) and 48 on the RP2350B
    // (QFN-80; RP2350 datasheet 9.1, md l.29215)
    constexpr int pinCountOf(Chip::Part c) { return c == Chip::Part::rp2350b ? 48 : 30; }

    template<Chip::Part P>
    struct ChipTraits;

    template<Chip::Part P>
    struct DmaTraits;

    template<Chip::Part P>
    struct PwmTraits;

    enum class UartPinType { tx0, rx0, cts0, rts0, tx1, rx1, cts1, rts1, invalid };
    enum class I2cPinType { sda0, scl0, sda1, scl1, invalid };
    enum class SpiPinType { rx0, tx0, cs0, sck0, rx1, tx1, cs1, sck1, invalid };

    // The pin functions are one pattern repeated every 8 GPIOs (RP2040 datasheet 1.4.3 Table 2, md l.620-667; RP2350
    // datasheet 9.4 Table 645, md l.29354-29440): F1 SPI, F2 UART, F3 I2C, and on the RP2350 F11 UART TX/RX on the
    // CTS/RTS pins. chip_rp_common/tests/pin_function_test.cpp checks the formulas against both tables.

    // F2: role pin%4 -> TX, RX, CTS, RTS; instance ((pin+4)/8)%2
    constexpr UartPinType uartPin(int pin) {
        constexpr UartPinType roles[2][4]{
          {UartPinType::tx0, UartPinType::rx0, UartPinType::cts0, UartPinType::rts0},
          {UartPinType::tx1, UartPinType::rx1, UartPinType::cts1, UartPinType::rts1}
        };
        return roles[((pin + 4) / 8) % 2][pin % 4];
    }

    // RP2350 F11: UART TX/RX of the same instance on the CTS/RTS pins (pin%4 in {2,3}: (pin-2+4)/8 == (pin+4)/8, the
    // 8-pin groups start at a multiple of 4), else the F2 role
    constexpr UartPinType uartAuxPin(int pin) {
        return pin % 4 >= 2 ? uartPin(pin - 2) : uartPin(pin);
    }

    // F1: role pin%4 -> RX, CSn, SCK, TX; instance (pin/8)%2
    constexpr SpiPinType spiPin(int pin) {
        constexpr SpiPinType roles[2][4]{
          {SpiPinType::rx0, SpiPinType::cs0, SpiPinType::sck0, SpiPinType::tx0},
          {SpiPinType::rx1, SpiPinType::cs1, SpiPinType::sck1, SpiPinType::tx1}
        };
        return roles[(pin / 8) % 2][pin % 4];
    }

    // F3: role pin%2 -> SDA, SCL; instance (pin/2)%2
    constexpr I2cPinType i2cPin(int pin) {
        constexpr I2cPinType roles[2][2]{
          {I2cPinType::sda0, I2cPinType::scl0},
          {I2cPinType::sda1, I2cPinType::scl1}
        };
        return roles[(pin / 2) % 2][pin % 2];
    }

    // The RP2350's UART has the F11 function (Table 645); the RP2040's does not (Table 2).
    constexpr bool hasUartAuxFunction(Chip::Part c) { return !isRp2040(c); }

    struct PwmChannelInfo {
        std::uint32_t channel;
        bool          isChannelA;
    };

    template<Chip::Part P,
             int        Pin>
    constexpr PwmChannelInfo getPwmChannelInfo() {
        if constexpr(P == Chip::Part::rp2040 || P == Chip::Part::rp2350a) {
            static_assert(Pin >= 0 && Pin < 30, "Pin number out of range for RP2040/RP2350A");
        } else if constexpr(P == Chip::Part::rp2350b) {
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

    // A peripheral pin's role, the one name the drivers' pin checks and pin set-ups use: the UART's TX/RX/CTS/RTS
    // (F2, and F11 on the RP2350), the SPI's RX/TX/CSn/SCK (F1), the I2C's SDA/SCL (F3).
    enum class Function {
        uartTx,
        uartRx,
        uartCts,
        uartRts,
        spiRx,
        spiTx,
        spiCs,
        spiSck,
        i2cSda,
        i2cScl
    };

    namespace detail {
        constexpr UartPinType uartType(Function f,
                                       unsigned instance) {
            constexpr UartPinType t[2][4]{
              {UartPinType::tx0, UartPinType::rx0, UartPinType::cts0, UartPinType::rts0},
              {UartPinType::tx1, UartPinType::rx1, UartPinType::cts1, UartPinType::rts1}
            };
            return t[instance][static_cast<unsigned>(f) - static_cast<unsigned>(Function::uartTx)];
        }

        constexpr SpiPinType spiType(Function f,
                                     unsigned instance) {
            constexpr SpiPinType t[2][4]{
              {SpiPinType::rx0, SpiPinType::tx0, SpiPinType::cs0, SpiPinType::sck0},
              {SpiPinType::rx1, SpiPinType::tx1, SpiPinType::cs1, SpiPinType::sck1}
            };
            return t[instance][static_cast<unsigned>(f) - static_cast<unsigned>(Function::spiRx)];
        }

        constexpr I2cPinType i2cType(Function f,
                                     unsigned instance) {
            constexpr I2cPinType t[2][2]{
              {I2cPinType::sda0, I2cPinType::scl0},
              {I2cPinType::sda1, I2cPinType::scl1}
            };
            return t[instance][static_cast<unsigned>(f) - static_cast<unsigned>(Function::i2cSda)];
        }

        constexpr bool isUart(Function f) { return f <= Function::uartRts; }

        constexpr bool isSpi(Function f) { return f >= Function::spiRx && f <= Function::spiSck; }
    }   // namespace detail

    // The GPIO function select that gives `pin` role `f` of the instance, -1 for none: F1 SPI, F2 UART, F3 I2C, and on
    // the RP2350 F11 for UART TX/RX on the CTS/RTS pins (the formulas above, from Table 2 / Table 645).
    constexpr int functionSelect(Chip::Part part,
                                 Function   f,
                                 unsigned   instance,
                                 int        pin) {
        if(instance > 1 || pin < 0 || pin >= pinCountOf(part)) { return -1; }
        if(detail::isUart(f)) {
            auto const want = detail::uartType(f, instance);
            if(uartPin(pin) == want) { return 2; }
            return hasUartAuxFunction(part) && uartAuxPin(pin) == want ? 11 : -1;
        }
        if(detail::isSpi(f)) { return spiPin(pin) == detail::spiType(f, instance) ? 1 : -1; }
        return i2cPin(pin) == detail::i2cType(f, instance) ? 3 : -1;
    }

    constexpr bool isValidPin(Chip::Part part,
                              Function   f,
                              unsigned   instance,
                              int        pin) {
        return functionSelect(part, f, instance, pin) >= 0;
    }

    namespace detail {
        // A compile-time message without the library: "GPIO 5 is not UART0 TX on the RP2040 (UART0 TX: GPIO 0 12 16
        // 28)" - an object with size() and data(), a C++26 static_assert message
        struct PinText {
            char        buffer[200]{};
            std::size_t length{};

            constexpr PinText& operator<<(char const* t) {
                for(; *t != '\0' && length + 1 < sizeof(buffer); ++t) { buffer[length++] = *t; }
                return *this;
            }

            constexpr PinText& operator<<(int v) {
                if(v < 0) {   // a negative pin number in a refused PinLocation
                    *this << "-";
                    v = -v;
                }
                char        digits[12]{};
                std::size_t n = 0;
                do {
                    digits[n++] = static_cast<char>('0' + v % 10);
                    v /= 10;
                } while(v != 0);
                while(n != 0 && length + 1 < sizeof(buffer)) { buffer[length++] = digits[--n]; }
                return *this;
            }

            [[nodiscard]] constexpr std::size_t size() const { return length; }

            [[nodiscard]] constexpr char const* data() const { return buffer; }
        };

        constexpr char const* functionName(Function f) {
            constexpr char const*
              names[]{"TX", "RX", "CTS", "RTS", "RX", "TX", "CSn", "SCK", "SDA", "SCL"};
            return names[static_cast<unsigned>(f)];
        }

        constexpr PinText pinRefused(Chip::Part part,
                                     Function   f,
                                     unsigned   instance,
                                     int        pin) {
            char const* block = isUart(f) ? "UART" : isSpi(f) ? "SPI" : "I2C";
            PinText     t;
            t << "GPIO " << pin << " is not " << block << static_cast<int>(instance) << " "
              << functionName(f) << " on the "
              << (isRp2040(part)                ? "RP2040"
                  : part == Chip::Part::rp2350a ? "RP2350A"
                                                : "RP2350B")
              << " (" << block << static_cast<int>(instance) << " " << functionName(f) << ": GPIO";
            for(int p = 0; p != pinCountOf(part); ++p) {
                if(isValidPin(part, f, instance, p)) { t << " " << p; }
            }
            return t << ")";
        }

        template<int>
        inline constexpr bool alwaysFalse = false;
    }   // namespace detail

    /// `value` true; for a pin that cannot have the role a static_assert names pin, role, instance and the pins that
    /// can (instantiated only then: the message is built for a refused pin alone).
    template<Chip::Part P,
             Function   F,
             unsigned   Instance,
             int        Pin,
             bool       Ok = isValidPin(P, F, Instance, Pin)>
    struct CheckedPin {
        static constexpr bool value = true;
    };

    template<Chip::Part P, Function F, unsigned Instance, int Pin>
    struct CheckedPin<P, F, Instance, Pin, false> {
        static constexpr bool check = detail::alwaysFalse<Pin>;
        static_assert(check,
                      detail::pinRefused(P,
                                         F,
                                         Instance,
                                         Pin));
        static constexpr bool value = true;
    };

}}   // namespace Kvasir::PinConfig

// Which RP chip this translation unit is for. The chip package's cmake/chip.cmake passes the macros to every TU of an
// image (kvasir_chip DEFINITIONS): chip_rp2040 KVASIR_CHIP_RP2040, chip_rp2350 KVASIR_CHIP_RP2350 and
// KVASIR_CHIP_RP2350A or KVASIR_CHIP_RP2350B (its KVASIR_RP2350_MPU), both KVASIR_CHIP_FAMILY_RP. Test them with `#if
// defined(...)` only (gcc -Wundef).
#if !defined(KVASIR_CHIP_RP2040) && !defined(KVASIR_CHIP_RP2350)
    #error \
      "chip/rp_common without KVASIR_CHIP_RP2040 or KVASIR_CHIP_RP2350: not compiled with the chip package's kvasir_chip DEFINITIONS"
#elif defined(KVASIR_CHIP_RP2040) && defined(KVASIR_CHIP_RP2350)
    #error \
      "chip/rp_common with both KVASIR_CHIP_RP2040 and KVASIR_CHIP_RP2350: kvasir_chip DEFINITIONS of two chip packages"
#elif defined(KVASIR_CHIP_RP2350) && defined(KVASIR_CHIP_RP2350A) == defined(KVASIR_CHIP_RP2350B)
    #error \
      "chip_rp2350 needs exactly one of KVASIR_CHIP_RP2350A, KVASIR_CHIP_RP2350B (cmake: KVASIR_RP2350_MPU)"
#endif

namespace Kvasir::Chip {
// The part this translation unit is for, from the package's KVASIR_CHIP_* macros.
#ifdef KVASIR_CHIP_RP2040
inline constexpr Part part = Part::rp2040;
#elifdef KVASIR_CHIP_RP2350A
inline constexpr Part part = Part::rp2350a;
#else
inline constexpr Part part = Part::rp2350b;
#endif
}   // namespace Kvasir::Chip
