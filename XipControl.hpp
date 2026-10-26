#pragma once
// XIP off and on again around a flash access, and the flash's direct-mode commands: the RAM functions of the flash
// path (the RP2350's QMI direct mode, XipReadMode, FlashQuadEnable; the RP2040's flash_do_cmd over XIP_SSI).
#include "PinConfig.hpp"
#include "Rom.hpp"
#include "core/Barrier.hpp"
#include "core/Delay.hpp"
#include "core/Special.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <kvasir/Register/Memory.hpp>
#include <kvasir/Register/Register.hpp>
#include <kvasir/Util/attributes.hpp>
#include <span>
#ifdef KVASIR_CHIP_RP2350
    #include "chip/RpTraits.hpp"
    #include "peripherals/BOOTRAM.hpp"
    #include "peripherals/QMI.hpp"
#endif
// the RP2040: its boot2 (Startup::second_stage_bootloader), XIP_SSI and IO_QSPI
#ifdef KVASIR_CHIP_RP2040
    #include "chip/StartUp.hpp"
    #include "peripherals/IO_QSPI.hpp"
    #include "peripherals/XIP_SSI.hpp"
#endif

namespace Kvasir { namespace detail {
#ifdef KVASIR_CHIP_RP2350
    // One transaction on chip select 0 in QMI direct mode (RP2350 datasheet 12.14.5) at
    // clk_sys / clkdiv: byte i of `rx` arrives while byte i of `tx` goes out, the first `rxSize`
    // are kept. XIP must be off. Plain pointers: hardened span accessors live in flash.
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] inline void directTransaction(std::byte const* tx,
                                                                 std::size_t      n,
                                                                 std::byte*       rx,
                                                                 std::size_t      rxSize,
                                                                 std::uint32_t    clkdiv) {
        KVASIR_RAM_FUNC_MARK();
        using QMI = Kvasir::Peripheral::QMI::Registers<0>;
        using Csr = QMI::DIRECT_CSR;
        using Tx  = QMI::DIRECT_TX;
        using namespace Kvasir::
          Register;   // always inlined, fields included: nothing here calls into flash
        // Every read-write field of DIRECT_CSR named (the rest is read-only or reserved): one store, no read.
        apply(write(Csr::rxdelay, value<std::uint32_t, 0U>()),
              write(Csr::clkdiv, clkdiv),
              clear(Csr::auto_cs1n),
              clear(Csr::auto_cs0n),
              clear(Csr::assert_cs1n),
              set(Csr::assert_cs0n),
              set(Csr::en));
        std::size_t sent = 0;
        std::size_t got  = 0;
        while(got < n) {
            auto const s = apply(read(Csr::txfull, Csr::rxempty));   // one load
            if(sent < n && s[Csr::txfull] == 0 && sent - got < 4) {
                // 8 bits (DWIDTH 0), single width, output enabled; DIRECT_TX is write-only: unnamed bits go out 0
                apply(set(Tx::oe), write(Tx::data, static_cast<std::uint32_t>(tx[sent])));
                ++sent;
            }
            if(s[Csr::rxempty] == 0) {
                auto const v = readWhole(QMI::DIRECT_RX::FULLREGISTER);   // the FIFO's data word
                if(got < rxSize) { rx[got] = static_cast<std::byte>(v & 0xFFU); }
                ++got;
            }
        }
        while(std::uint32_t{apply(read(Csr::busy))} != 0) {}
        // CS up, direct mode off: every read-write field 0
        apply(write(Csr::rxdelay, value<std::uint32_t, 0U>()),
              write(Csr::clkdiv, value<std::uint32_t, 0U>()),
              clear(Csr::auto_cs1n),
              clear(Csr::auto_cs0n),
              clear(Csr::assert_cs1n),
              clear(Csr::assert_cs0n),
              clear(Csr::en));
    }

    // A command of N bytes packed into a word, first byte lowest; returns the reply packed the
    // same way. The bytes are immediates: a const array would be placed in flash.
    template<std::size_t N>
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] inline std::uint32_t directCommand(std::uint32_t tx,
                                                                      std::uint32_t clkdiv) {
        KVASIR_RAM_FUNC_MARK();
        static_assert(N >= 1 && N <= 4);
        std::uint32_t rx = 0;
        directTransaction(reinterpret_cast<std::byte const*>(&tx),
                          N,
                          reinterpret_cast<std::byte*>(&rx),
                          N,
                          clkdiv);
        return rx;
    }

    // 25 MHz at 150 MHz clk_sys, below the W25Q16JV's 104 MHz (datasheet rev H, AC
    // characteristics).
    inline constexpr std::uint32_t directCommandClkdiv = 6;

    // a field's value (an enumerator too) at its bits of a whole-register word
    template<typename F,
             typename V>
    consteval std::uint32_t qmiAt(F field,
                                  V value) {
        return Kvasir::Register::deposit(field, static_cast<std::uint32_t>(value));
    }

    // The XIP read mode every RP2350 project runs in.
    //
    // The bootrom brings the flash up in EBh quad I/O (command on one line, address, mode
    // bits, dummy and data on four) and leaves the mode bits at 0x00, so every fetch that
    // misses the cache spends 8 SCK cycles on the command byte. With mode bits 0xA0 the
    // flash stays in "continuous read": it keeps answering EBh-shaped transfers without
    // the command byte, 28 -> 20 SCK cycles per random word, as pico-sdk's boot2 does.
    // apply() makes the switch when the QMI is in the bootrom's EBh mode and leaves any
    // other mode alone (a flash the bootrom would not run in quad mode is not forced into
    // it). It also writes the timing, because the bootrom's flash_exit_xip resets M0_TIMING
    // to clkdiv 12.
    //
    // Exception: with quadEnableSr2 set (FlashQuadEnable), the bootrom's BBh is upgraded too.
    //
    // Runs from RAM with interrupts off and the QMI idle: while the format changes,
    // nothing may fetch from flash. peripheryClockInit() calls it once at boot,
    // FlashXipDisabler::enable() after every direct-mode access (the BOOTRAM setup
    // function that re-enables XIP restores the bootrom's mode, mode bits 0x00).
    struct XipReadMode {
        using QMI  = Kvasir::Peripheral::QMI::Registers<0>;
        using Rfmt = QMI::M0_RFMT;
        using Rcmd = QMI::M0_RCMD;

        static constexpr std::uint32_t xipNoCacheBase
          = Rp::ChipTraits::xipUncachedBase;   // chip/RpTraits.hpp
        static constexpr std::uint32_t prefixLenBit = Rfmt::prefix_len.mask;
        // prefix 1-bit 8 bits, address/suffix/dummy/data 4-bit, suffix 8 bits, dummy 16
        // bits (4 quad clocks): what the bootrom programs for EBh
        static constexpr std::uint32_t rfmtQuadEBh
          = qmiAt(Rfmt::dummy_len, Rfmt::DUMMY_LENVal::_16)
          | qmiAt(Rfmt::suffix_len, Rfmt::SUFFIX_LENVal::_8)
          | qmiAt(Rfmt::prefix_len, Rfmt::PREFIX_LENVal::_8)
          | qmiAt(Rfmt::data_width, Rfmt::DATA_WIDTHVal::q)
          | qmiAt(Rfmt::dummy_width, Rfmt::DUMMY_WIDTHVal::q)
          | qmiAt(Rfmt::suffix_width, Rfmt::SUFFIX_WIDTHVal::q)
          | qmiAt(Rfmt::addr_width, Rfmt::ADDR_WIDTHVal::q)
          | qmiAt(Rfmt::prefix_width, Rfmt::PREFIX_WIDTHVal::s);
        static_assert(rfmtQuadEBh == 0x0004'92A8U,
                      "the bootrom's EBh format word");
        static constexpr std::uint32_t commandEBh         = 0xEBU;
        static constexpr std::uint32_t commandBBh         = 0xBBU;
        static constexpr std::uint32_t continuousModeBits = 0xA0U;
        // M0_RCMD's PREFIX (the command byte) and SUFFIX (the mode bits) in a snapshot of the register: the masks of the
        // fields, the shape of the compares as before the field names (extract() let the optimiser lay the RAM
        // function's branches out otherwise: +4 B)
        static constexpr std::uint32_t prefixMask = Rcmd::prefix.mask;
        static constexpr unsigned      suffixShift
          = static_cast<unsigned>(std::countr_zero(Rcmd::suffix.mask));
        static constexpr std::uint32_t suffixBits = Rcmd::suffix.mask >> suffixShift;
        static_assert(prefixMask == 0xFFU && suffixShift == 8 && suffixBits == 0xFFU,
                      "M0_RCMD: PREFIX 7:0, SUFFIX 15:8");

        // Status register 2 with QE that FlashQuadEnable set at boot, 0 if it did not.
        // FlashXipDisabler::enable() writes it again before every return to XIP.
        static inline std::uint8_t quadEnableSr2 = 0;

        // M0_TIMING that reads at clk_ref speed and at the full clock: clkdiv 4, rxdelay 2,
        // min_deselect 2, cooldown 1 - the divider flashInit() uses before the PLL switch
        // with the bootrom's sample delay, i.e. what every boot runs on through the switch
        // until peripheryClockInit(). What a clk_sys resus handler writes before it touches
        // flash: at 12 MHz the full-clock timing's rxdelay samples after the bit is gone. The
        // handler then runs coreClockInit() from flash on it through the PLL switch, like a boot:
        // flashInit() there raises the divider to the boot one first, and its static_assert
        // checks that rxdelay 2 still samples a valid bit at the project's clk_sys.
        static constexpr std::uint32_t timingAtClkRef
          = qmiAt(QMI::M0_TIMING::cooldown, 1U) | qmiAt(QMI::M0_TIMING::min_deselect, 2U)
          | qmiAt(QMI::M0_TIMING::rxdelay, 2U) | qmiAt(QMI::M0_TIMING::clkdiv, 4U);
        static_assert(timingAtClkRef == ((1U << 30) | (2U << 12) | (2U << 8) | 4U));

        // The resus handler reads it before the flash is readable: never an out-of-line call.
        [[nodiscard,
          gnu::always_inline]] static std::uint32_t
        currentTiming() {
            return Kvasir::Register::readWhole(QMI::M0_TIMING::FULLREGISTER);
        }

        // Both are called from apply(), a RAM function that runs while the flash is not readable: never out of
        // line (gcc's debug variant left isContinuous in flash, found by check_ram_funcs.py).
        [[nodiscard,
          KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] static bool
        isQuadEBh(std::uint32_t rfmt,
                  std::uint32_t rcmd) {
            return (rfmt | prefixLenBit) == rfmtQuadEBh && (rcmd & prefixMask) == commandEBh;
        }

        [[nodiscard,
          KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] static bool
        isContinuous(std::uint32_t rfmt,
                     std::uint32_t rcmd) {
            return isQuadEBh(rfmt, rcmd) && (rfmt & prefixLenBit) == 0
                && ((rcmd >> suffixShift) & suffixBits) == continuousModeBits;
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void apply(std::uint32_t timing) {
            KVASIR_RAM_FUNC_MARK();
            using Kvasir::Register::readWhole;
            using Kvasir::Register::writeWhole;

            // The fetch that brought us here may still hold the chip select (cooldown: 64 x COOLDOWN clk_sys cycles
            // plus half an SCK period, COOLDOWN at most 3, RP2350 datasheet M0_TIMING, md l.60453): let it expire, then
            // the QMI is idle. 1024 cycles: what the 256 turns of the loop here before took at least.
            Kvasir::Core::spinCycles<1024>();
            writeWhole(QMI::M0_TIMING::FULLREGISTER, timing);
            Kvasir::Core::dsb();

            auto const rfmt = readWhole(QMI::M0_RFMT::FULLREGISTER);
            auto const rcmd = readWhole(QMI::M0_RCMD::FULLREGISTER);
            bool const upgrade
              = isQuadEBh(rfmt, rcmd) || (quadEnableSr2 != 0 && (rcmd & prefixMask) == commandBBh);
            if(!upgrade || isContinuous(rfmt, rcmd)) { return; }

            // The one transfer with the command byte and the continuous mode bits: the
            // flash is in continuous mode after it.
            // both fields of M0_RCMD named (the upper half is reserved): one store
            Kvasir::Register::apply(
              Kvasir::Register::write(Rcmd::prefix, Kvasir::Register::value<commandEBh>()),
              Kvasir::Register::write(Rcmd::suffix, Kvasir::Register::value<continuousModeBits>()));
            writeWhole(QMI::M0_RFMT::FULLREGISTER, rfmtQuadEBh);
            Kvasir::Core::dsb();
            (void)Kvasir::Memory::load<std::uint32_t>(xipNoCacheBase);
            Kvasir::Core::dsb();
            // Let the chip select cooldown (as above) run out so the format is changed with the QMI idle.
            Kvasir::Core::spinCycles<1024>();
            writeWhole(QMI::M0_RFMT::FULLREGISTER,
                       rfmtQuadEBh & ~prefixLenBit);   // no command byte from now on
            Kvasir::Core::dsb();
            Kvasir::Core::isb();
        }
    };
#endif

    struct FlashXipDisabler {
        using RomConnectInternalFlash = void (*)();
        using RomFlashExitXip         = void (*)();
        using RomFlashFlushCache      = void (*)();
        using XipEnableFunction       = void (*)();

        RomConnectInternalFlash const connectInternalFlash;
        RomFlashExitXip const         flashExitXip;
        RomFlashFlushCache const      flushCache;
        XipEnableFunction const       xipEnable;

#ifdef KVASIR_CHIP_RP2350
        // Where the RP2350's boot ROM leaves the XIP setup function (boot2's successor) on a flash boot: the base of
        // boot RAM, at most 256 bytes, position-independent, never executable there (RP2350 datasheet 5.2.7
        // "Flash boot", md l.19373-19379; BOOTRAM_BASE 0x400e0000, l.1551 - the SVD's BOOTRAM block). Memory, not a
        // register: read as a 64-word array.
        static constexpr std::uintptr_t xipSetupFunctionAddress
          = Kvasir::Peripheral::BOOTRAM::Registers<>::baseAddr;
        static_assert(sizeof(std::array<std::uint32_t,
                                        64>)
                        == 256,
                      "the XIP setup function's 256 bytes");
#endif

        // The XIP setup function, copied out of BOOTRAM (never executable) into .ramExec, the
        // image's writable and executable range (chip Mpu.hpp; the stack is never executed).
        [[KVASIR_SECTION_MEMBER(".ramExec")]] alignas(
          4) static inline std::array<std::uint32_t,
                                      64> xipEnableRamCopy{};

#ifdef KVASIR_CHIP_RP2350
        std::uint32_t flashTiming;
#endif

        static std::array<std::uint32_t,
                          64> getXipEnable() {
            if constexpr(Chip::part == Kvasir::Chip::Part::rp2040) {
#ifdef KVASIR_CHIP_RP2040
                return Kvasir::Startup::secondStageBootloader;
#endif
            } else {
#ifdef KVASIR_CHIP_RP2350
                std::array<std::uint32_t, 64> words{};
                Kvasir::Memory::loadBytes(xipSetupFunctionAddress,
                                          std::as_writable_bytes(std::span{words}).data(),
                                          sizeof(words));
                return words;
#endif
            }
        }

        FlashXipDisabler()
          : connectInternalFlash{
              Kvasir::RomFunctions::getRomFunctionPointer<'I',
                                                          'F',
                                                          RomConnectInternalFlash>()}
          , flashExitXip{Kvasir::RomFunctions::getRomFunctionPointer<'E',
                                                                     'X',
                                                                     RomFlashExitXip>()}
          , flushCache{Kvasir::RomFunctions::getRomFunctionPointer<'F',
                                                                   'C',
                                                                   RomFlashFlushCache>()}
          , xipEnable{reinterpret_cast<XipEnableFunction>(
              std::as_writable_bytes(std::span{xipEnableRamCopy}).data() + 1)} {
            xipEnableRamCopy = getXipEnable();
            Kvasir::Core::dsb();   // the copy is code from here on
            Kvasir::Core::isb();
#ifdef KVASIR_CHIP_RP2350
            using namespace Kvasir::Peripheral::QMI;
            using QMI   = Registers<0>;
            flashTiming = apply(read(QMI::M0_TIMING::FULLREGISTER));
#endif
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] void disable() {
            KVASIR_RAM_FUNC_MARK();
            connectInternalFlash();
            flashExitXip();
        }

        [[KVASIR_RAM_FUNC_ATTRIBUTES]] void enable() {
            KVASIR_RAM_FUNC_MARK();
            flushCache();
#ifdef KVASIR_CHIP_RP2350
            // Volatile QE again: a flash reset (66h 99h) restores the stored QE = 0.
            if(std::uint32_t const sr2 = XipReadMode::quadEnableSr2; sr2 != 0) {
                directCommand<1>(0x50U, directCommandClkdiv);   // write enable, volatile
                directCommand<2>(0x31U | (sr2 << 8), directCommandClkdiv);   // status register 2
            }
#endif
            xipEnable();
#ifdef KVASIR_CHIP_RP2350
            // xipEnable() ran the BOOTRAM setup function: bootrom mode, bootrom timing.
            XipReadMode::apply(flashTiming);
#endif
        }
    };

    struct XipGuard {
        FlashXipDisabler& disabler;

        [[KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] explicit XipGuard(FlashXipDisabler& d) : disabler(d) {
            disabler.disable();
        }

        [[KVASIR_RAM_FUNC_INLINE_ATTRIBUTES]] ~XipGuard() { disabler.enable(); }

        XipGuard(XipGuard const&)            = delete;
        XipGuard& operator=(XipGuard const&) = delete;
    };

#ifdef KVASIR_CHIP_RP2350
    // Quad I/O for a flash the bootrom left in BBh dual I/O because QE is clear, e.g. the
    // RP2354's W25Q16JV "IM" (JEDEC EF 70 15, QE = 0 from the factory; W25Q16JV datasheet rev H,
    // "Quad Enable (QE)"). QE is set as a volatile bit (50h, 31h): no wear. Winbond 40h / 70h
    // parts with QE = 0 only. With QE = 1 the flash drives IO2/IO3 (/WP, /HOLD).
    struct FlashQuadEnable {
        static constexpr std::uint32_t clkdiv = directCommandClkdiv;

        // Leaving the guard rewrites QE and switches to continuous EBh (FlashXipDisabler::enable).
        [[KVASIR_RAM_FUNC_ATTRIBUTES]] static void run(FlashXipDisabler& xip) {
            KVASIR_RAM_FUNC_MARK();
            XipGuard const guard{xip};

            auto const id = directCommand<4>(0x9FU, clkdiv);   // JEDEC id: EF, type, capacity
            auto const manufacturer = (id >> 8) & 0xFFU;
            auto const type         = (id >> 16) & 0xFFU;
            if(manufacturer != 0xEFU || (type != 0x40U && type != 0x70U)) { return; }

            auto const before = (directCommand<2>(0x35U, clkdiv) >> 8) & 0xFFU;   // SR2
            if((before & 0x02U) != 0) { return; }

            auto const wanted = before | 0x02U;
            directCommand<1>(0x50U, clkdiv);                   // write enable, volatile
            directCommand<2>(0x31U | (wanted << 8), clkdiv);   // status register 2
            // tSHSL2, /CS deselect after a write: 50 ns (W25Q16JV AC characteristics, md l.1848), 8 cycles at 150 MHz
            Kvasir::Core::spinCycles<256>();

            auto const after = (directCommand<2>(0x35U, clkdiv) >> 8) & 0xFFU;
            if((after & 0x02U) != 0) {
                XipReadMode::quadEnableSr2 = static_cast<std::uint8_t>(wanted);
            }
        }
    };

    // At boot, after initMemory(): FlashQuadEnable if the bootrom settled on BBh.
    inline void enableQuadIo() {
        using QMI = Kvasir::Peripheral::QMI::Registers<0>;
        if(get<0>(apply(read(QMI::M0_RCMD::prefix))) != XipReadMode::commandBBh) { return; }
        FlashXipDisabler xip{};
        auto const       irqs = Kvasir::Core::IrqState::saveAndMask();
        FlashQuadEnable::run(xip);
        irqs.restore();
    }
#endif

#ifdef KVASIR_CHIP_RP2040
    // Plain pointers and counts, taken apart by flash_do_cmd() in flash: a RAM function calls nothing in flash, and a
    // span's members are not inline by themselves - with libc++'s hardening (the sanitize variant) span::operator[],
    // empty() and subspan() are real functions (the sanitize image died in the first read of the serial number). The
    // register layer is always inlined (the whole apply() chain, value(), the field results; Kvasir_SDK tests/ram_funcs
    // CASE 8 builds it with inlining off). A call into flash with XIP off is a fault whose handler is in flash too:
    // the core locks up.
    //
    // SSI SR: TFNF, RFNE; DR0 the FIFO's data word (RP2040 datasheet "SSI: SR Register" / "SSI: DR0 Register", md
    // l.28208-28232, l.28490). GPIO_QSPI_SS_CTRL.OUTOVER: 2 drives the pad low, 3 high (md l.14158-14163).
    [[KVASIR_RAM_FUNC_ATTRIBUTES]] static inline void flashDoCmdImpl(FlashXipDisabler& xipDisabler,
                                                                     std::byte const*  tx,
                                                                     std::size_t       txLeft,
                                                                     std::byte*        rx,
                                                                     std::size_t       rxLeft) {
        KVASIR_RAM_FUNC_MARK();
        using SsiRegs = Kvasir::Peripheral::XIP_SSI::Registers<>;
        using CsCtrl  = Kvasir::Peripheral::IO_QSPI::Registers<>::GPIO_QSPI_SS_CTRL;
        using namespace Kvasir::
          Register;   // always inlined, fields included: nothing here calls into flash
        XipGuard const guard{xipDisabler};

        // no lambda here: gcc at -Og would leave it out of line, in flash
        apply(write(CsCtrl::OUTOVERValC::low));

        static constexpr std::size_t maxInFlight = 16 - 2;

        std::size_t inFlight{};
        while(txLeft != 0 || rxLeft != 0) {
            auto const flags = apply(read(SsiRegs::SR::tfnf, SsiRegs::SR::rfne));   // one load
            if(flags[SsiRegs::SR::tfnf] != 0 && txLeft != 0 && inFlight < maxInFlight) {
                writeWhole(SsiRegs::DR0::FULLREGISTER, static_cast<std::uint32_t>(*tx));
                ++tx;
                --txLeft;
                ++inFlight;
            }
            if(flags[SsiRegs::SR::rfne] != 0 && rxLeft != 0) {
                *rx = static_cast<std::byte>(readWhole(SsiRegs::DR0::FULLREGISTER));
                ++rx;
                --rxLeft;
                --inFlight;
            }
        }

        apply(write(CsCtrl::OUTOVERValC::high));
    }

    // Runs from flash like flash_erase(): the FlashXipDisabler's ROM lookups and boot2 copy
    // need XIP, only the _impl runs with it off.
    static inline void flashDoCmd(std::span<std::byte const> txBuffer,
                                  std::span<std::byte>       rxBuffer) {
        FlashXipDisabler xipDisabler{};
        flashDoCmdImpl(xipDisabler,
                       txBuffer.data(),
                       txBuffer.size(),
                       rxBuffer.data(),
                       rxBuffer.size());
    }

#endif
}}   // namespace Kvasir::detail
