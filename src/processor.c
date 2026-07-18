#include "processor.h"
#include "processors/dummy.h"
#include "processors/i4004.h"
#include "processors/rv32i.h"
#include "processors/i8008.h"
#include "processors/i8080.h"
#include "processors/atmega328p.h"
#include "processors/mos6502.h"
#include "processors/z80.h"
#include "processors/i8085.h"
#include "processors/i8086.h"
#include "processors/i4040.h"
#include "processors/mc6800.h"
#include "processors/mc6809.h"
#include "processors/m68000.h"
#include "processors/cdp1802.h"
#include "processors/sm83.h"
#include "processors/w65c02.h"
#include "processors/chip8.h"
#include "processors/lc3.h"
#include "processors/pdp8.h"
#include "processors/mips32.h"
#include "processors/tms9900.h"
#include "processors/i80186.h"
#include "processors/i80286.h"
#include "processors/necv20.h"
#include "processors/z180.h"
#include "processors/z8.h"
#include "processors/z8000.h"
#include "processors/m68010.h"
#include "processors/mc6805.h"
#include "processors/mc68hc11.h"
#include "processors/hd6309.h"
#include "processors/mos6510.h"
#include "processors/w65816.h"
#include "processors/huc6280.h"
#include "processors/spc700.h"
#include "processors/i8051.h"
#include "processors/i8048.h"
#include "processors/pic16.h"
#include "processors/msp430.h"
#include "processors/arm7tdmi.h"
#include "processors/sh2.h"
#include "processors/tms1000.h"
#include "processors/scmp.h"
#include "processors/f8.h"
#include "processors/s2650.h"
#include "processors/cp1600.h"
#include "processors/rv64i.h"
#include "processors/pdp11.h"
#include "processors/nova.h"
#include "processors/picoblaze.h"
#include "processors/j1.h"
#include <string.h>
#include <stdio.h>

// Static registry of processors
static const ProcessorInfo g_processors[] = {
    {
        "dummy",
        "A skeleton/mock processor for bare-bones testing",
        10,
        dummy_create,
        dummy_destroy,
        dummy_init,
        dummy_load,
        dummy_step,
        dummy_print_state,
        dummy_get_disassembly
    },
    {
        "i4004",
        "Intel 4004 4-bit microprocessor emulator",
        74000,
        i4004_create,
        i4004_destroy,
        i4004_init,
        i4004_load,
        i4004_step,
        i4004_print_state,
        i4004_get_disassembly
    },
    {
        "rv32i",
        "RISC-V RV32I 32-bit Base Integer CPU Core",
        10000000,
        rv32i_create,
        rv32i_destroy,
        rv32i_init,
        rv32i_load,
        rv32i_step,
        rv32i_print_state,
        rv32i_get_disassembly
    },
    {
        "i8008",
        "Intel 8008 8-bit microprocessor emulator",
        62500,
        i8008_create,
        i8008_destroy,
        i8008_init,
        i8008_load,
        i8008_step,
        i8008_print_state,
        i8008_get_disassembly
    },
    {
        "i8080",
        "Intel 8080 8-bit microprocessor emulator",
        2000000,
        i8080_create,
        i8080_destroy,
        i8080_init,
        i8080_load,
        i8080_step,
        i8080_print_state,
        i8080_get_disassembly
    },
    {
        "atmega328p",
        "AVR ATmega328P 8-bit microcontroller (Arduino Nano)",
        16000000,
        atmega328p_create,
        atmega328p_destroy,
        atmega328p_init,
        atmega328p_load,
        atmega328p_step,
        atmega328p_print_state,
        atmega328p_get_disassembly
    },
    {
        "mos6502",
        "MOS Technology 6502 8-bit microprocessor",
        1000000,
        mos6502_create,
        mos6502_destroy,
        mos6502_init,
        mos6502_load,
        mos6502_step,
        mos6502_print_state,
        mos6502_get_disassembly
    },
    {
        "z80",
        "Zilog Z80 8-bit microprocessor emulator",
        4000000,
        z80_create, z80_destroy, z80_init, z80_load, z80_step, z80_print_state, z80_get_disassembly
    },
    {
        "i8085",
        "Intel 8085 8-bit microprocessor emulator",
        3000000,
        i8085_create, i8085_destroy, i8085_init, i8085_load, i8085_step, i8085_print_state, i8085_get_disassembly
    },
    {
        "i8086",
        "Intel 8086 16-bit microprocessor emulator",
        5000000,
        i8086_create, i8086_destroy, i8086_init, i8086_load, i8086_step, i8086_print_state, i8086_get_disassembly
    },
    {
        "i4040",
        "Intel 4040 4-bit microprocessor emulator",
        740000,
        i4040_create, i4040_destroy, i4040_init, i4040_load, i4040_step, i4040_print_state, i4040_get_disassembly
    },
    {
        "mc6800",
        "Motorola 6800 8-bit microprocessor emulator",
        1000000,
        mc6800_create, mc6800_destroy, mc6800_init, mc6800_load, mc6800_step, mc6800_print_state, mc6800_get_disassembly
    },
    {
        "mc6809",
        "Motorola 6809 8-bit microprocessor emulator",
        1000000,
        mc6809_create, mc6809_destroy, mc6809_init, mc6809_load, mc6809_step, mc6809_print_state, mc6809_get_disassembly
    },
    {
        "m68000",
        "Motorola 68000 16/32-bit microprocessor emulator",
        8000000,
        m68000_create, m68000_destroy, m68000_init, m68000_load, m68000_step, m68000_print_state, m68000_get_disassembly
    },
    {
        "cdp1802",
        "RCA CDP1802 COSMAC 8-bit microprocessor emulator",
        1760000,
        cdp1802_create, cdp1802_destroy, cdp1802_init, cdp1802_load, cdp1802_step, cdp1802_print_state, cdp1802_get_disassembly
    },
    {
        "sm83",
        "Sharp SM83 (Game Boy CPU) 8-bit emulator",
        4194304,
        sm83_create, sm83_destroy, sm83_init, sm83_load, sm83_step, sm83_print_state, sm83_get_disassembly
    },
    {
        "w65c02",
        "WDC 65C02 8-bit microprocessor emulator",
        1000000,
        w65c02_create, w65c02_destroy, w65c02_init, w65c02_load, w65c02_step, w65c02_print_state, w65c02_get_disassembly
    },
    {
        "chip8",
        "CHIP-8 virtual machine interpreter",
        500,
        chip8_create, chip8_destroy, chip8_init, chip8_load, chip8_step, chip8_print_state, chip8_get_disassembly
    },
    {
        "lc3",
        "LC-3 educational 16-bit CPU emulator",
        1000000,
        lc3_create, lc3_destroy, lc3_init, lc3_load, lc3_step, lc3_print_state, lc3_get_disassembly
    },
    {
        "pdp8",
        "DEC PDP-8 12-bit minicomputer CPU emulator",
        333333,
        pdp8_create, pdp8_destroy, pdp8_init, pdp8_load, pdp8_step, pdp8_print_state, pdp8_get_disassembly
    },
    {
        "mips32",
        "MIPS32 (R2000-class, MIPS I) 32-bit CPU emulator",
        8000000,
        mips32_create, mips32_destroy, mips32_init, mips32_load, mips32_step, mips32_print_state, mips32_get_disassembly
    },
    {
        "tms9900",
        "Texas Instruments TMS9900 16-bit microprocessor emulator",
        3000000,
        tms9900_create, tms9900_destroy, tms9900_init, tms9900_load, tms9900_step, tms9900_print_state, tms9900_get_disassembly
    },
    {
        "i80186",
        "Intel 80186 16-bit microprocessor emulator",
        8000000,
        i80186_create, i80186_destroy, i80186_init, i80186_load, i80186_step, i80186_print_state, i80186_get_disassembly
    },
    {
        "i80286",
        "Intel 80286 16-bit microprocessor emulator (real mode)",
        8000000,
        i80286_create, i80286_destroy, i80286_init, i80286_load, i80286_step, i80286_print_state, i80286_get_disassembly
    },
    {
        "necv20",
        "NEC V20 8086-compatible microprocessor emulator",
        8000000,
        necv20_create, necv20_destroy, necv20_init, necv20_load, necv20_step, necv20_print_state, necv20_get_disassembly
    },
    {
        "z180",
        "Zilog Z180 8-bit microprocessor emulator (Z80 + MMU)",
        6000000,
        z180_create, z180_destroy, z180_init, z180_load, z180_step, z180_print_state, z180_get_disassembly
    },
    {
        "z8",
        "Zilog Z8 8-bit microcontroller emulator",
        8000000,
        z8_create, z8_destroy, z8_init, z8_load, z8_step, z8_print_state, z8_get_disassembly
    },
    {
        "z8000",
        "Zilog Z8002 16-bit microprocessor emulator",
        4000000,
        z8000_create, z8000_destroy, z8000_init, z8000_load, z8000_step, z8000_print_state, z8000_get_disassembly
    },
    {
        "m68010",
        "Motorola 68010 16/32-bit microprocessor emulator",
        8000000,
        m68010_create, m68010_destroy, m68010_init, m68010_load, m68010_step, m68010_print_state, m68010_get_disassembly
    },
    {
        "mc6805",
        "Motorola 6805 8-bit microcontroller emulator",
        1000000,
        mc6805_create, mc6805_destroy, mc6805_init, mc6805_load, mc6805_step, mc6805_print_state, mc6805_get_disassembly
    },
    {
        "mc68hc11",
        "Motorola 68HC11 8-bit microcontroller emulator",
        2000000,
        mc68hc11_create, mc68hc11_destroy, mc68hc11_init, mc68hc11_load, mc68hc11_step, mc68hc11_print_state, mc68hc11_get_disassembly
    },
    {
        "hd6309",
        "Hitachi HD6309 8-bit microprocessor emulator (native mode)",
        2000000,
        hd6309_create, hd6309_destroy, hd6309_init, hd6309_load, hd6309_step, hd6309_print_state, hd6309_get_disassembly
    },
    {
        "mos6510",
        "MOS 6510 (Commodore 64 CPU) with illegal opcodes",
        1000000,
        mos6510_create, mos6510_destroy, mos6510_init, mos6510_load, mos6510_step, mos6510_print_state, mos6510_get_disassembly
    },
    {
        "w65816",
        "WDC 65C816 8/16-bit microprocessor emulator",
        2680000,
        w65816_create, w65816_destroy, w65816_init, w65816_load, w65816_step, w65816_print_state, w65816_get_disassembly
    },
    {
        "huc6280",
        "Hudson HuC6280 (PC Engine CPU) emulator",
        7160000,
        huc6280_create, huc6280_destroy, huc6280_init, huc6280_load, huc6280_step, huc6280_print_state, huc6280_get_disassembly
    },
    {
        "spc700",
        "Sony SPC700 (SNES audio CPU) emulator",
        1024000,
        spc700_create, spc700_destroy, spc700_init, spc700_load, spc700_step, spc700_print_state, spc700_get_disassembly
    },
    {
        "i8051",
        "Intel 8051 (MCS-51) microcontroller emulator",
        1000000,
        i8051_create, i8051_destroy, i8051_init, i8051_load, i8051_step, i8051_print_state, i8051_get_disassembly
    },
    {
        "i8048",
        "Intel 8048 (MCS-48) microcontroller emulator",
        400000,
        i8048_create, i8048_destroy, i8048_init, i8048_load, i8048_step, i8048_print_state, i8048_get_disassembly
    },
    {
        "pic16",
        "Microchip PIC16 midrange microcontroller emulator",
        1000000,
        pic16_create, pic16_destroy, pic16_init, pic16_load, pic16_step, pic16_print_state, pic16_get_disassembly
    },
    {
        "msp430",
        "Texas Instruments MSP430 16-bit MCU emulator",
        1000000,
        msp430_create, msp430_destroy, msp430_init, msp430_load, msp430_step, msp430_print_state, msp430_get_disassembly
    },
    {
        "arm7tdmi",
        "ARM7TDMI (ARMv4T) 32-bit CPU emulator with Thumb",
        16780000,
        arm7tdmi_create, arm7tdmi_destroy, arm7tdmi_init, arm7tdmi_load, arm7tdmi_step, arm7tdmi_print_state, arm7tdmi_get_disassembly
    },
    {
        "sh2",
        "Hitachi SH-2 32-bit RISC CPU emulator",
        28000000,
        sh2_create, sh2_destroy, sh2_init, sh2_load, sh2_step, sh2_print_state, sh2_get_disassembly
    },
    {
        "tms1000",
        "Texas Instruments TMS1000 4-bit microcontroller emulator",
        100000,
        tms1000_create, tms1000_destroy, tms1000_init, tms1000_load, tms1000_step, tms1000_print_state, tms1000_get_disassembly
    },
    {
        "scmp",
        "National Semiconductor SC/MP (INS8060) emulator",
        500000,
        scmp_create, scmp_destroy, scmp_init, scmp_load, scmp_step, scmp_print_state, scmp_get_disassembly
    },
    {
        "f8",
        "Fairchild F8 (3850) 8-bit microprocessor emulator",
        500000,
        f8_create, f8_destroy, f8_init, f8_load, f8_step, f8_print_state, f8_get_disassembly
    },
    {
        "s2650",
        "Signetics 2650 8-bit microprocessor emulator",
        1000000,
        s2650_create, s2650_destroy, s2650_init, s2650_load, s2650_step, s2650_print_state, s2650_get_disassembly
    },
    {
        "cp1600",
        "General Instrument CP1600 (Intellivision CPU) emulator",
        894886,
        cp1600_create, cp1600_destroy, cp1600_init, cp1600_load, cp1600_step, cp1600_print_state, cp1600_get_disassembly
    },
    {
        "rv64i",
        "RISC-V RV64I 64-bit Base Integer CPU Core",
        10000000,
        rv64i_create, rv64i_destroy, rv64i_init, rv64i_load, rv64i_step, rv64i_print_state, rv64i_get_disassembly
    },
    {
        "pdp11",
        "DEC PDP-11 16-bit minicomputer CPU emulator",
        1000000,
        pdp11_create, pdp11_destroy, pdp11_init, pdp11_load, pdp11_step, pdp11_print_state, pdp11_get_disassembly
    },
    {
        "nova",
        "Data General Nova 16-bit minicomputer CPU emulator",
        500000,
        nova_create, nova_destroy, nova_init, nova_load, nova_step, nova_print_state, nova_get_disassembly
    },
    {
        "picoblaze",
        "Xilinx PicoBlaze KCPSM6 soft-core emulator",
        25000000,
        picoblaze_create, picoblaze_destroy, picoblaze_init, picoblaze_load, picoblaze_step, picoblaze_print_state, picoblaze_get_disassembly
    },
    {
        "j1",
        "J1 Forth stack-machine CPU emulator",
        50000000,
        j1_create, j1_destroy, j1_init, j1_load, j1_step, j1_print_state, j1_get_disassembly
    }
};

static const size_t g_num_processors = sizeof(g_processors) / sizeof(g_processors[0]);

const ProcessorInfo* processor_find(const char *name) {
    for (size_t i = 0; i < g_num_processors; ++i) {
        if (strcmp(g_processors[i].name, name) == 0) {
            return &g_processors[i];
        }
    }
    return NULL;
}

void processor_list_all(void) {
    printf("Available processors:\n");
    for (size_t i = 0; i < g_num_processors; ++i) {
        printf("  - %-10s : %s\n", g_processors[i].name, g_processors[i].description);
    }
}
