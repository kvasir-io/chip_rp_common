// The RP pin-function formulas (PinConfig.hpp uartPin/uartAuxPin/spiPin/i2cPin and functionSelect built on them) against
// the datasheets' tables, row by row: RP2040 datasheet 1.4.3 Table 2 (md l.620-667), RP2350 datasheet 9.4 Table 645
// (md l.29354-29440), columns F1 (SPI), F2 (UART), F3 (I2C), F11 (RP2350 UART on the CTS/RTS pins). The rows below were
// copied out of the converted datasheets by a script and are kept as literal data. Building is the test (static_assert);
// the program prints what it compared.
#include "PinConfig.hpp"

#include <cstdio>
#include <string_view>

namespace {
using namespace Kvasir::PinConfig;

struct Row {
    std::string_view f1, f2, f3, f11;
};

// rp2040Table2: GPIO -> {F1, F2, F3, F11}
inline constexpr Row rp2040Table2[] = {
  { "SPI0 RX",  "UART0 TX", "I2C0 SDA", ""}, // 0
  {"SPI0 CSn",  "UART0 RX", "I2C0 SCL", ""}, // 1
  {"SPI0 SCK", "UART0 CTS", "I2C1 SDA", ""}, // 2
  { "SPI0 TX", "UART0 RTS", "I2C1 SCL", ""}, // 3
  { "SPI0 RX",  "UART1 TX", "I2C0 SDA", ""}, // 4
  {"SPI0 CSn",  "UART1 RX", "I2C0 SCL", ""}, // 5
  {"SPI0 SCK", "UART1 CTS", "I2C1 SDA", ""}, // 6
  { "SPI0 TX", "UART1 RTS", "I2C1 SCL", ""}, // 7
  { "SPI1 RX",  "UART1 TX", "I2C0 SDA", ""}, // 8
  {"SPI1 CSn",  "UART1 RX", "I2C0 SCL", ""}, // 9
  {"SPI1 SCK", "UART1 CTS", "I2C1 SDA", ""}, // 10
  { "SPI1 TX", "UART1 RTS", "I2C1 SCL", ""}, // 11
  { "SPI1 RX",  "UART0 TX", "I2C0 SDA", ""}, // 12
  {"SPI1 CSn",  "UART0 RX", "I2C0 SCL", ""}, // 13
  {"SPI1 SCK", "UART0 CTS", "I2C1 SDA", ""}, // 14
  { "SPI1 TX", "UART0 RTS", "I2C1 SCL", ""}, // 15
  { "SPI0 RX",  "UART0 TX", "I2C0 SDA", ""}, // 16
  {"SPI0 CSn",  "UART0 RX", "I2C0 SCL", ""}, // 17
  {"SPI0 SCK", "UART0 CTS", "I2C1 SDA", ""}, // 18
  { "SPI0 TX", "UART0 RTS", "I2C1 SCL", ""}, // 19
  { "SPI0 RX",  "UART1 TX", "I2C0 SDA", ""}, // 20
  {"SPI0 CSn",  "UART1 RX", "I2C0 SCL", ""}, // 21
  {"SPI0 SCK", "UART1 CTS", "I2C1 SDA", ""}, // 22
  { "SPI0 TX", "UART1 RTS", "I2C1 SCL", ""}, // 23
  { "SPI1 RX",  "UART1 TX", "I2C0 SDA", ""}, // 24
  {"SPI1 CSn",  "UART1 RX", "I2C0 SCL", ""}, // 25
  {"SPI1 SCK", "UART1 CTS", "I2C1 SDA", ""}, // 26
  { "SPI1 TX", "UART1 RTS", "I2C1 SCL", ""}, // 27
  { "SPI1 RX",  "UART0 TX", "I2C0 SDA", ""}, // 28
  {"SPI1 CSn",  "UART0 RX", "I2C0 SCL", ""}, // 29
};

// rp2350Table645: GPIO -> {F1, F2, F3, F11}
inline constexpr Row rp2350Table645[] = {
  { "SPI0 RX",  "UART0 TX", "I2C0 SDA",         ""}, // 0
  {"SPI0 CSn",  "UART0 RX", "I2C0 SCL",         ""}, // 1
  {"SPI0 SCK", "UART0 CTS", "I2C1 SDA", "UART0 TX"}, // 2
  { "SPI0 TX", "UART0 RTS", "I2C1 SCL", "UART0 RX"}, // 3
  { "SPI0 RX",  "UART1 TX", "I2C0 SDA",         ""}, // 4
  {"SPI0 CSn",  "UART1 RX", "I2C0 SCL",         ""}, // 5
  {"SPI0 SCK", "UART1 CTS", "I2C1 SDA", "UART1 TX"}, // 6
  { "SPI0 TX", "UART1 RTS", "I2C1 SCL", "UART1 RX"}, // 7
  { "SPI1 RX",  "UART1 TX", "I2C0 SDA",         ""}, // 8
  {"SPI1 CSn",  "UART1 RX", "I2C0 SCL",         ""}, // 9
  {"SPI1 SCK", "UART1 CTS", "I2C1 SDA", "UART1 TX"}, // 10
  { "SPI1 TX", "UART1 RTS", "I2C1 SCL", "UART1 RX"}, // 11
  { "SPI1 RX",  "UART0 TX", "I2C0 SDA",         ""}, // 12
  {"SPI1 CSn",  "UART0 RX", "I2C0 SCL",         ""}, // 13
  {"SPI1 SCK", "UART0 CTS", "I2C1 SDA", "UART0 TX"}, // 14
  { "SPI1 TX", "UART0 RTS", "I2C1 SCL", "UART0 RX"}, // 15
  { "SPI0 RX",  "UART0 TX", "I2C0 SDA",         ""}, // 16
  {"SPI0 CSn",  "UART0 RX", "I2C0 SCL",         ""}, // 17
  {"SPI0 SCK", "UART0 CTS", "I2C1 SDA", "UART0 TX"}, // 18
  { "SPI0 TX", "UART0 RTS", "I2C1 SCL", "UART0 RX"}, // 19
  { "SPI0 RX",  "UART1 TX", "I2C0 SDA",         ""}, // 20
  {"SPI0 CSn",  "UART1 RX", "I2C0 SCL",         ""}, // 21
  {"SPI0 SCK", "UART1 CTS", "I2C1 SDA", "UART1 TX"}, // 22
  { "SPI0 TX", "UART1 RTS", "I2C1 SCL", "UART1 RX"}, // 23
  { "SPI1 RX",  "UART1 TX", "I2C0 SDA",         ""}, // 24
  {"SPI1 CSn",  "UART1 RX", "I2C0 SCL",         ""}, // 25
  {"SPI1 SCK", "UART1 CTS", "I2C1 SDA", "UART1 TX"}, // 26
  { "SPI1 TX", "UART1 RTS", "I2C1 SCL", "UART1 RX"}, // 27
  { "SPI1 RX",  "UART0 TX", "I2C0 SDA",         ""}, // 28
  {"SPI1 CSn",  "UART0 RX", "I2C0 SCL",         ""}, // 29
  {"SPI1 SCK", "UART0 CTS", "I2C1 SDA", "UART0 TX"}, // 30
  { "SPI1 TX", "UART0 RTS", "I2C1 SCL", "UART0 RX"}, // 31
  { "SPI0 RX",  "UART0 TX", "I2C0 SDA",         ""}, // 32
  {"SPI0 CSn",  "UART0 RX", "I2C0 SCL",         ""}, // 33
  {"SPI0 SCK", "UART0 CTS", "I2C1 SDA", "UART0 TX"}, // 34
  { "SPI0 TX", "UART0 RTS", "I2C1 SCL", "UART0 RX"}, // 35
  { "SPI0 RX",  "UART1 TX", "I2C0 SDA",         ""}, // 36
  {"SPI0 CSn",  "UART1 RX", "I2C0 SCL",         ""}, // 37
  {"SPI0 SCK", "UART1 CTS", "I2C1 SDA", "UART1 TX"}, // 38
  { "SPI0 TX", "UART1 RTS", "I2C1 SCL", "UART1 RX"}, // 39
  { "SPI1 RX",  "UART1 TX", "I2C0 SDA",         ""}, // 40
  {"SPI1 CSn",  "UART1 RX", "I2C0 SCL",         ""}, // 41
  {"SPI1 SCK", "UART1 CTS", "I2C1 SDA", "UART1 TX"}, // 42
  { "SPI1 TX", "UART1 RTS", "I2C1 SCL", "UART1 RX"}, // 43
  { "SPI1 RX",  "UART0 TX", "I2C0 SDA",         ""}, // 44
  {"SPI1 CSn",  "UART0 RX", "I2C0 SCL",         ""}, // 45
  {"SPI1 SCK", "UART0 CTS", "I2C1 SDA", "UART0 TX"}, // 46
  { "SPI1 TX", "UART0 RTS", "I2C1 SCL", "UART0 RX"}, // 47
};

constexpr UartPinType uartOf(std::string_view s) {
    constexpr std::string_view names[]{"UART0 TX",
                                       "UART0 RX",
                                       "UART0 CTS",
                                       "UART0 RTS",
                                       "UART1 TX",
                                       "UART1 RX",
                                       "UART1 CTS",
                                       "UART1 RTS"};
    for(int i = 0; i != 8; ++i) {
        if(names[i] == s) { return static_cast<UartPinType>(i); }
    }
    return UartPinType::invalid;
}

constexpr SpiPinType spiOf(std::string_view s) {
    constexpr std::pair<std::string_view, SpiPinType> names[]{
      { "SPI0 RX",  SpiPinType::rx0},
      { "SPI0 TX",  SpiPinType::tx0},
      {"SPI0 CSn",  SpiPinType::cs0},
      {"SPI0 SCK", SpiPinType::sck0},
      { "SPI1 RX",  SpiPinType::rx1},
      { "SPI1 TX",  SpiPinType::tx1},
      {"SPI1 CSn",  SpiPinType::cs1},
      {"SPI1 SCK", SpiPinType::sck1}
    };
    for(auto const& [n, t] : names) {
        if(n == s) { return t; }
    }
    return SpiPinType::invalid;
}

constexpr I2cPinType i2cOf(std::string_view s) {
    constexpr std::pair<std::string_view, I2cPinType> names[]{
      {"I2C0 SDA", I2cPinType::sda0},
      {"I2C0 SCL", I2cPinType::scl0},
      {"I2C1 SDA", I2cPinType::sda1},
      {"I2C1 SCL", I2cPinType::scl1}
    };
    for(auto const& [n, t] : names) {
        if(n == s) { return t; }
    }
    return I2cPinType::invalid;
}

// the formulas, every pin of the table
template<std::size_t N>
constexpr bool formulasMatch(Row const (&table)[N],
                             bool aux) {
    for(std::size_t p = 0; p != N; ++p) {
        auto const pin = static_cast<int>(p);
        if(spiPin(pin) != spiOf(table[p].f1) || uartPin(pin) != uartOf(table[p].f2)
           || i2cPin(pin) != i2cOf(table[p].f3))
        {
            return false;
        }
        // F11: present exactly on the CTS/RTS pins of the RP2350, and then TX/RX of the same instance
        auto const expectedAux = table[p].f11.empty() ? uartOf(table[p].f2) : uartOf(table[p].f11);
        if(aux && uartAuxPin(pin) != expectedAux) { return false; }
        if(!aux && !table[p].f11.empty()) { return false; }
    }
    return true;
}

static_assert(formulasMatch(rp2040Table2,
                            false),
              "RP2040 Table 2 differs from the pin formulas");
static_assert(formulasMatch(rp2350Table645,
                            true),
              "RP2350 Table 645 differs from the pin formulas");

// the one check the drivers use (functionSelect / isValidPin): every pin, role and instance against the table's
// columns - F1 SPI, F2 UART, F3 I2C, F11 (RP2350) UART TX/RX - and nothing else
constexpr std::string_view roleName(Function f,
                                    unsigned instance) {
    constexpr std::string_view names[10][2]{
      { "UART0 TX",  "UART1 TX"},
      { "UART0 RX",  "UART1 RX"},
      {"UART0 CTS", "UART1 CTS"},
      {"UART0 RTS", "UART1 RTS"},
      {  "SPI0 RX",   "SPI1 RX"},
      {  "SPI0 TX",   "SPI1 TX"},
      { "SPI0 CSn",  "SPI1 CSn"},
      { "SPI0 SCK",  "SPI1 SCK"},
      { "I2C0 SDA",  "I2C1 SDA"},
      { "I2C0 SCL",  "I2C1 SCL"}
    };
    return names[static_cast<unsigned>(f)][instance];
}

template<std::size_t N>
constexpr bool selectsMatch(Kvasir::Chip::Part part,
                            Row const (&table)[N]) {
    if(pinCountOf(part) > static_cast<int>(N)) { return false; }
    for(int pin = 0; pin != pinCountOf(part); ++pin) {
        auto const& row = table[static_cast<std::size_t>(pin)];
        for(unsigned f = 0; f != 10; ++f) {
            for(unsigned instance = 0; instance != 2; ++instance) {
                auto const fn   = static_cast<Function>(f);
                auto const name = roleName(fn, instance);
                int const  want = row.f1 == name  ? 1
                                : row.f2 == name  ? 2
                                : row.f3 == name  ? 3
                                : row.f11 == name ? 11
                                                  : -1;
                if(functionSelect(part, fn, instance, pin) != want) { return false; }
                if(isValidPin(part, fn, instance, pin) != (want >= 0)) { return false; }
            }
        }
    }
    return true;
}

static_assert(selectsMatch(Kvasir::Chip::Part::rp2040,
                           rp2040Table2));
static_assert(selectsMatch(Kvasir::Chip::Part::rp2350a,
                           rp2350Table645));
static_assert(selectsMatch(Kvasir::Chip::Part::rp2350b,
                           rp2350Table645));
static_assert(pinCountOf(Kvasir::Chip::Part::rp2350b) == 48
              && pinCountOf(Kvasir::Chip::Part::rp2040) == 30);

// spot checks, and the edges the table loop does not reach
static_assert(functionSelect(Kvasir::Chip::Part::rp2350a,
                             Function::uartTx,
                             0,
                             2)
              == 11);   // F11
static_assert(functionSelect(Kvasir::Chip::Part::rp2350a,
                             Function::uartCts,
                             0,
                             2)
              == 2);   // F2
static_assert(!isValidPin(Kvasir::Chip::Part::rp2040,
                          Function::uartTx,
                          0,
                          2));
static_assert(!isValidPin(Kvasir::Chip::Part::rp2350a,
                          Function::uartTx,
                          0,
                          30));   // past the A package
static_assert(isValidPin(Kvasir::Chip::Part::rp2350b,
                         Function::uartTx,
                         1,
                         38));
static_assert(isValidPin(Kvasir::Chip::Part::rp2040,
                         Function::i2cSda,
                         1,
                         26));
static_assert(isValidPin(Kvasir::Chip::Part::rp2350b,
                         Function::spiSck,
                         0,
                         34));
static_assert(!isValidPin(Kvasir::Chip::Part::rp2350b,
                          Function::spiSck,
                          1,
                          34));
static_assert(!isValidPin(Kvasir::Chip::Part::rp2040,
                          Function::spiSck,
                          2,
                          2));   // no SPI2
static_assert(!isValidPin(Kvasir::Chip::Part::rp2040,
                          Function::i2cSda,
                          0,
                          -1));
static_assert(CheckedPin<Kvasir::Chip::Part::rp2040,
                         Function::i2cSda,
                         0,
                         4>::value);

// the refusal's text: pin, role, instance, chip and the pins that have the role
constexpr bool refusalNames() {
    auto const t = detail::pinRefused(Kvasir::Chip::Part::rp2040, Function::uartTx, 0, 5);
    return std::string_view{t.data(), t.size()}
        == "GPIO 5 is not UART0 TX on the RP2040 (UART0 TX: GPIO 0 12 16 28)";
}

static_assert(refusalNames());
}   // namespace

int main() {
    std::printf(
      "pin functions: RP2040 Table 2 %zu pins, RP2350 Table 645 %zu pins (F1/F2/F3/F11) match the "
      "formulas "
      "and the function selects of RP2040, RP2350A, RP2350B\n",
      std::size(rp2040Table2),
      std::size(rp2350Table645));
    return 0;
}
