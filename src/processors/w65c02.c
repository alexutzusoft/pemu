// WDC 65C02 CPU core: NMOS 6502 instruction set plus CMOS additions
// (PHX/PLX/PHY/PLY, STZ, TRB/TSB, BRA, (zp) addressing, INC A/DEC A,
// BBR/BBS, RMB/SMB, WAI/STP) and CMOS behavioral fixes (JMP (ind)
// page-boundary bug fixed, decimal-mode N/Z/C/V flags valid).
#include "w65c02.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_D 0x08
#define FLAG_B 0x10
#define FLAG_U 0x20
#define FLAG_V 0x40
#define FLAG_N 0x80

typedef struct W65C02_CPU {
    uint8_t ram[65536];
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint8_t p;
    uint16_t pc;
    uint32_t ticks;
    int halted;
} W65C02_CPU;

#define SET_FLAG_C(cond) do { if (cond) cpu->p |= FLAG_C; else cpu->p &= ~FLAG_C; } while(0)
#define SET_FLAG_Z(val) do { if ((val) == 0) cpu->p |= FLAG_Z; else cpu->p &= ~FLAG_Z; } while(0)
#define SET_FLAG_N(val) do { if ((val) & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N; } while(0)
#define SET_FLAG_V(cond) do { if (cond) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V; } while(0)
#define GET_FLAG(flag) ((cpu->p & (flag)) ? 1 : 0)

// Addressing modes (drives operand decode and disassembly)
enum {
    M_IMP,  // implied
    M_ACC,  // accumulator
    M_IMM,  // #imm
    M_ZP,   // zp
    M_ZPX,  // zp,X
    M_ZPY,  // zp,Y
    M_ABS,  // abs
    M_ABX,  // abs,X
    M_ABY,  // abs,Y
    M_IND,  // (abs)      -- JMP only, page-bug fixed
    M_IAX,  // (abs,X)    -- JMP only
    M_INX,  // (zp,X)
    M_INY,  // (zp),Y
    M_IZP,  // (zp)       -- CMOS
    M_REL,  // relative branch
    M_ZPR   // zp, rel    -- BBRn/BBSn
};

static const char *g_mnem[256] = {
/*        x0     x1     x2     x3     x4     x5     x6     x7     x8     x9     xA     xB     xC     xD     xE     xF   */
/* 0x */ "BRK", "ORA", "NOP", "NOP", "TSB", "ORA", "ASL", "RMB0","PHP", "ORA", "ASL", "NOP", "TSB", "ORA", "ASL", "BBR0",
/* 1x */ "BPL", "ORA", "ORA", "NOP", "TRB", "ORA", "ASL", "RMB1","CLC", "ORA", "INC", "NOP", "TRB", "ORA", "ASL", "BBR1",
/* 2x */ "JSR", "AND", "NOP", "NOP", "BIT", "AND", "ROL", "RMB2","PLP", "AND", "ROL", "NOP", "BIT", "AND", "ROL", "BBR2",
/* 3x */ "BMI", "AND", "AND", "NOP", "BIT", "AND", "ROL", "RMB3","SEC", "AND", "DEC", "NOP", "BIT", "AND", "ROL", "BBR3",
/* 4x */ "RTI", "EOR", "NOP", "NOP", "NOP", "EOR", "LSR", "RMB4","PHA", "EOR", "LSR", "NOP", "JMP", "EOR", "LSR", "BBR4",
/* 5x */ "BVC", "EOR", "EOR", "NOP", "NOP", "EOR", "LSR", "RMB5","CLI", "EOR", "PHY", "NOP", "NOP", "EOR", "LSR", "BBR5",
/* 6x */ "RTS", "ADC", "NOP", "NOP", "STZ", "ADC", "ROR", "RMB6","PLA", "ADC", "ROR", "NOP", "JMP", "ADC", "ROR", "BBR6",
/* 7x */ "BVS", "ADC", "ADC", "NOP", "STZ", "ADC", "ROR", "RMB7","SEI", "ADC", "PLY", "NOP", "JMP", "ADC", "ROR", "BBR7",
/* 8x */ "BRA", "STA", "NOP", "NOP", "STY", "STA", "STX", "SMB0","DEY", "BIT", "TXA", "NOP", "STY", "STA", "STX", "BBS0",
/* 9x */ "BCC", "STA", "STA", "NOP", "STY", "STA", "STX", "SMB1","TYA", "STA", "TXS", "NOP", "STZ", "STA", "STZ", "BBS1",
/* Ax */ "LDY", "LDA", "LDX", "NOP", "LDY", "LDA", "LDX", "SMB2","TAY", "LDA", "TAX", "NOP", "LDY", "LDA", "LDX", "BBS2",
/* Bx */ "BCS", "LDA", "LDA", "NOP", "LDY", "LDA", "LDX", "SMB3","CLV", "LDA", "TSX", "NOP", "LDY", "LDA", "LDX", "BBS3",
/* Cx */ "CPY", "CMP", "NOP", "NOP", "CPY", "CMP", "DEC", "SMB4","INY", "CMP", "DEX", "WAI", "CPY", "CMP", "DEC", "BBS4",
/* Dx */ "BNE", "CMP", "CMP", "NOP", "NOP", "CMP", "DEC", "SMB5","CLD", "CMP", "PHX", "STP", "NOP", "CMP", "DEC", "BBS5",
/* Ex */ "CPX", "SBC", "NOP", "NOP", "CPX", "SBC", "INC", "SMB6","INX", "SBC", "NOP", "NOP", "CPX", "SBC", "INC", "BBS6",
/* Fx */ "BEQ", "SBC", "SBC", "NOP", "NOP", "SBC", "INC", "SMB7","SED", "SBC", "PLX", "NOP", "NOP", "SBC", "INC", "BBS7"
};

static const uint8_t g_mode[256] = {
/*        x0     x1     x2     x3     x4     x5     x6     x7     x8     x9     xA     xB     xC     xD     xE     xF  */
/* 0x */ M_IMP, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_ACC, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* 1x */ M_REL, M_INY, M_IZP, M_IMP, M_ZP,  M_ZPX, M_ZPX, M_ZP,  M_IMP, M_ABY, M_ACC, M_IMP, M_ABS, M_ABX, M_ABX, M_ZPR,
/* 2x */ M_ABS, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_ACC, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* 3x */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPX, M_ZP,  M_IMP, M_ABY, M_ACC, M_IMP, M_ABX, M_ABX, M_ABX, M_ZPR,
/* 4x */ M_IMP, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_ACC, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* 5x */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPX, M_ZP,  M_IMP, M_ABY, M_IMP, M_IMP, M_ABS, M_ABX, M_ABX, M_ZPR,
/* 6x */ M_IMP, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_ACC, M_IMP, M_IND, M_ABS, M_ABS, M_ZPR,
/* 7x */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPX, M_ZP,  M_IMP, M_ABY, M_IMP, M_IMP, M_IAX, M_ABX, M_ABX, M_ZPR,
/* 8x */ M_REL, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_IMP, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* 9x */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPY, M_ZP,  M_IMP, M_ABY, M_IMP, M_IMP, M_ABS, M_ABX, M_ABX, M_ZPR,
/* Ax */ M_IMM, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_IMP, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* Bx */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPY, M_ZP,  M_IMP, M_ABY, M_IMP, M_IMP, M_ABX, M_ABX, M_ABY, M_ZPR,
/* Cx */ M_IMM, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_IMP, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* Dx */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPX, M_ZP,  M_IMP, M_ABY, M_IMP, M_IMP, M_ABS, M_ABX, M_ABX, M_ZPR,
/* Ex */ M_IMM, M_INX, M_IMM, M_IMP, M_ZP,  M_ZP,  M_ZP,  M_ZP,  M_IMP, M_IMM, M_IMP, M_IMP, M_ABS, M_ABS, M_ABS, M_ZPR,
/* Fx */ M_REL, M_INY, M_IZP, M_IMP, M_ZPX, M_ZPX, M_ZPX, M_ZP,  M_IMP, M_ABY, M_IMP, M_IMP, M_ABS, M_ABX, M_ABX, M_ZPR
};

static uint8_t mem_read(W65C02_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr];
}

static void mem_write(W65C02_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr] = val;
}

static uint8_t fetch8(W65C02_CPU *cpu) {
    return mem_read(cpu, cpu->pc++);
}

static uint16_t fetch16(W65C02_CPU *cpu) {
    uint16_t lo = fetch8(cpu);
    uint16_t hi = fetch8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

static uint16_t read16(W65C02_CPU *cpu, uint16_t addr) {
    // CMOS fix: the pointer increments across a page boundary correctly
    return (uint16_t)(mem_read(cpu, addr) | ((uint16_t)mem_read(cpu, (uint16_t)(addr + 1)) << 8));
}

static uint16_t read16_zp(W65C02_CPU *cpu, uint8_t zp) {
    return (uint16_t)(mem_read(cpu, zp) | ((uint16_t)mem_read(cpu, (uint8_t)(zp + 1)) << 8));
}

static void push_byte(W65C02_CPU *cpu, uint8_t val) {
    mem_write(cpu, (uint16_t)(0x0100 + cpu->sp), val);
    cpu->sp--;
}

static uint8_t pop_byte(W65C02_CPU *cpu) {
    cpu->sp++;
    return mem_read(cpu, (uint16_t)(0x0100 + cpu->sp));
}

static void set_zn(W65C02_CPU *cpu, uint8_t val) {
    SET_FLAG_Z(val);
    SET_FLAG_N(val);
}

static void op_adc(W65C02_CPU *cpu, uint8_t v) {
    unsigned c = GET_FLAG(FLAG_C);
    if (cpu->p & FLAG_D) {
        // 65C02 decimal mode: N, Z, C, V all valid
        unsigned lo = (cpu->a & 0x0F) + (v & 0x0F) + c;
        unsigned hi = (unsigned)(cpu->a >> 4) + (unsigned)(v >> 4);
        if (lo > 0x09) { lo += 0x06; hi++; }
        SET_FLAG_V(~(cpu->a ^ v) & ((uint8_t)(hi << 4) ^ cpu->a) & 0x80);
        if (hi > 0x09) hi += 0x06;
        SET_FLAG_C(hi > 0x0F);
        cpu->a = (uint8_t)(((hi & 0x0F) << 4) | (lo & 0x0F));
    } else {
        unsigned sum = (unsigned)cpu->a + v + c;
        SET_FLAG_C(sum > 0xFF);
        SET_FLAG_V(~(cpu->a ^ v) & (cpu->a ^ (uint8_t)sum) & 0x80);
        cpu->a = (uint8_t)sum;
    }
    set_zn(cpu, cpu->a);
}

static void op_sbc(W65C02_CPU *cpu, uint8_t v) {
    unsigned borrow = GET_FLAG(FLAG_C) ? 0 : 1;
    int diff = (int)cpu->a - (int)v - (int)borrow;
    SET_FLAG_V((cpu->a ^ v) & (cpu->a ^ (uint8_t)diff) & 0x80);
    SET_FLAG_C(diff >= 0);
    if (cpu->p & FLAG_D) {
        int lo = (int)(cpu->a & 0x0F) - (int)(v & 0x0F) - (int)borrow;
        if (diff < 0) diff -= 0x60;
        if (lo < 0) diff -= 0x06;
        cpu->a = (uint8_t)diff;
    } else {
        cpu->a = (uint8_t)diff;
    }
    set_zn(cpu, cpu->a);
}

static void op_cmp(W65C02_CPU *cpu, uint8_t reg, uint8_t v) {
    uint8_t diff = (uint8_t)(reg - v);
    SET_FLAG_C(reg >= v);
    set_zn(cpu, diff);
}

static void op_bit(W65C02_CPU *cpu, uint8_t v) {
    SET_FLAG_Z(cpu->a & v);
    if (v & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N;
    if (v & 0x40) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
}

static uint8_t op_asl(W65C02_CPU *cpu, uint8_t v) {
    SET_FLAG_C(v & 0x80);
    v <<= 1;
    set_zn(cpu, v);
    return v;
}

static uint8_t op_lsr(W65C02_CPU *cpu, uint8_t v) {
    SET_FLAG_C(v & 0x01);
    v >>= 1;
    set_zn(cpu, v);
    return v;
}

static uint8_t op_rol(W65C02_CPU *cpu, uint8_t v) {
    uint8_t old_c = (uint8_t)GET_FLAG(FLAG_C);
    SET_FLAG_C(v & 0x80);
    v = (uint8_t)((v << 1) | old_c);
    set_zn(cpu, v);
    return v;
}

static uint8_t op_ror(W65C02_CPU *cpu, uint8_t v) {
    uint8_t old_c = (uint8_t)GET_FLAG(FLAG_C);
    SET_FLAG_C(v & 0x01);
    v = (uint8_t)((v >> 1) | (old_c << 7));
    set_zn(cpu, v);
    return v;
}

static void op_branch(W65C02_CPU *cpu, int cond) {
    int8_t rel = (int8_t)fetch8(cpu);
    if (cond) {
        cpu->pc = (uint16_t)(cpu->pc + rel);
    }
}

void* w65c02_create(void) {
    W65C02_CPU *cpu = (W65C02_CPU*)calloc(1, sizeof(W65C02_CPU));
    return cpu;
}

void w65c02_destroy(void *context) {
    free(context);
}

int w65c02_init(void *context) {
    if (!context) return -1;
    W65C02_CPU *cpu = (W65C02_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->p = FLAG_U | FLAG_I;
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int w65c02_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    W65C02_CPU *cpu = (W65C02_CPU*)context;

    if (address >= 65536) return -2;
    size_t copy_len = size;
    if (address + size > 65536) {
        copy_len = 65536 - address;
    }
    memcpy(cpu->ram + address, data, copy_len);
    cpu->pc = (uint16_t)address;

    return 0;
}

int w65c02_step(void *context) {
    if (!context) return -1;
    W65C02_CPU *cpu = (W65C02_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks++;

    // Decode the operand for the common addressing modes up front.
    // Flat 64KB RAM means the speculative read for store targets is harmless.
    uint16_t addr = 0;
    uint8_t val = 0;
    switch (g_mode[op]) {
        case M_IMM:
            val = fetch8(cpu);
            break;
        case M_ZP:
            addr = fetch8(cpu);
            val = mem_read(cpu, addr);
            break;
        case M_ZPX:
            addr = (uint8_t)(fetch8(cpu) + cpu->x);
            val = mem_read(cpu, addr);
            break;
        case M_ZPY:
            addr = (uint8_t)(fetch8(cpu) + cpu->y);
            val = mem_read(cpu, addr);
            break;
        case M_ABS:
            addr = fetch16(cpu);
            val = mem_read(cpu, addr);
            break;
        case M_ABX:
            addr = (uint16_t)(fetch16(cpu) + cpu->x);
            val = mem_read(cpu, addr);
            break;
        case M_ABY:
            addr = (uint16_t)(fetch16(cpu) + cpu->y);
            val = mem_read(cpu, addr);
            break;
        case M_INX:
            addr = read16_zp(cpu, (uint8_t)(fetch8(cpu) + cpu->x));
            val = mem_read(cpu, addr);
            break;
        case M_INY:
            addr = (uint16_t)(read16_zp(cpu, fetch8(cpu)) + cpu->y);
            val = mem_read(cpu, addr);
            break;
        case M_IZP:
            addr = read16_zp(cpu, fetch8(cpu));
            val = mem_read(cpu, addr);
            break;
        case M_IND: // JMP (abs) -- CMOS: page boundary handled correctly
            addr = read16(cpu, fetch16(cpu));
            break;
        case M_IAX: // JMP (abs,X)
            addr = read16(cpu, (uint16_t)(fetch16(cpu) + cpu->x));
            break;
        case M_ACC:
            val = cpu->a;
            break;
        default: // M_IMP, M_REL, M_ZPR: operands handled per-opcode
            break;
    }

    switch (op) {
        // --- Halting instructions ---
        case 0x00: // BRK (treated as program stop, matching raw 6502 core)
            cpu->halted = 1;
            return 1;
        case 0xCB: // WAI
        case 0xDB: // STP
            cpu->halted = 1;
            return 1;

        // --- ORA / AND / EOR ---
        case 0x01: case 0x05: case 0x09: case 0x0D:
        case 0x11: case 0x12: case 0x15: case 0x19: case 0x1D:
            cpu->a |= val;
            set_zn(cpu, cpu->a);
            break;
        case 0x21: case 0x25: case 0x29: case 0x2D:
        case 0x31: case 0x32: case 0x35: case 0x39: case 0x3D:
            cpu->a &= val;
            set_zn(cpu, cpu->a);
            break;
        case 0x41: case 0x45: case 0x49: case 0x4D:
        case 0x51: case 0x52: case 0x55: case 0x59: case 0x5D:
            cpu->a ^= val;
            set_zn(cpu, cpu->a);
            break;

        // --- ADC / SBC ---
        case 0x61: case 0x65: case 0x69: case 0x6D:
        case 0x71: case 0x72: case 0x75: case 0x79: case 0x7D:
            op_adc(cpu, val);
            break;
        case 0xE1: case 0xE5: case 0xE9: case 0xED:
        case 0xF1: case 0xF2: case 0xF5: case 0xF9: case 0xFD:
            op_sbc(cpu, val);
            break;

        // --- CMP / CPX / CPY ---
        case 0xC1: case 0xC5: case 0xC9: case 0xCD:
        case 0xD1: case 0xD2: case 0xD5: case 0xD9: case 0xDD:
            op_cmp(cpu, cpu->a, val);
            break;
        case 0xE0: case 0xE4: case 0xEC:
            op_cmp(cpu, cpu->x, val);
            break;
        case 0xC0: case 0xC4: case 0xCC:
            op_cmp(cpu, cpu->y, val);
            break;

        // --- Loads ---
        case 0xA1: case 0xA5: case 0xA9: case 0xAD:
        case 0xB1: case 0xB2: case 0xB5: case 0xB9: case 0xBD:
            cpu->a = val;
            set_zn(cpu, cpu->a);
            break;
        case 0xA2: case 0xA6: case 0xAE: case 0xB6: case 0xBE:
            cpu->x = val;
            set_zn(cpu, cpu->x);
            break;
        case 0xA0: case 0xA4: case 0xAC: case 0xB4: case 0xBC:
            cpu->y = val;
            set_zn(cpu, cpu->y);
            break;

        // --- Stores ---
        case 0x81: case 0x85: case 0x8D:
        case 0x91: case 0x92: case 0x95: case 0x99: case 0x9D:
            mem_write(cpu, addr, cpu->a);
            break;
        case 0x86: case 0x8E: case 0x96:
            mem_write(cpu, addr, cpu->x);
            break;
        case 0x84: case 0x8C: case 0x94:
            mem_write(cpu, addr, cpu->y);
            break;
        case 0x64: case 0x74: case 0x9C: case 0x9E: // STZ (CMOS)
            mem_write(cpu, addr, 0);
            break;

        // --- Shifts / rotates ---
        case 0x0A:
            cpu->a = op_asl(cpu, cpu->a);
            break;
        case 0x06: case 0x0E: case 0x16: case 0x1E:
            mem_write(cpu, addr, op_asl(cpu, val));
            break;
        case 0x4A:
            cpu->a = op_lsr(cpu, cpu->a);
            break;
        case 0x46: case 0x4E: case 0x56: case 0x5E:
            mem_write(cpu, addr, op_lsr(cpu, val));
            break;
        case 0x2A:
            cpu->a = op_rol(cpu, cpu->a);
            break;
        case 0x26: case 0x2E: case 0x36: case 0x3E:
            mem_write(cpu, addr, op_rol(cpu, val));
            break;
        case 0x6A:
            cpu->a = op_ror(cpu, cpu->a);
            break;
        case 0x66: case 0x6E: case 0x76: case 0x7E:
            mem_write(cpu, addr, op_ror(cpu, val));
            break;

        // --- INC / DEC (including CMOS INC A / DEC A) ---
        case 0x1A: // INC A
            cpu->a++;
            set_zn(cpu, cpu->a);
            break;
        case 0x3A: // DEC A
            cpu->a--;
            set_zn(cpu, cpu->a);
            break;
        case 0xE6: case 0xEE: case 0xF6: case 0xFE:
            val = (uint8_t)(val + 1);
            mem_write(cpu, addr, val);
            set_zn(cpu, val);
            break;
        case 0xC6: case 0xCE: case 0xD6: case 0xDE:
            val = (uint8_t)(val - 1);
            mem_write(cpu, addr, val);
            set_zn(cpu, val);
            break;

        // --- BIT / TSB / TRB ---
        case 0x24: case 0x2C: case 0x34: case 0x3C:
            op_bit(cpu, val);
            break;
        case 0x89: // BIT #imm: only Z affected (CMOS)
            SET_FLAG_Z(cpu->a & val);
            break;
        case 0x04: case 0x0C: // TSB (CMOS)
            SET_FLAG_Z(cpu->a & val);
            mem_write(cpu, addr, (uint8_t)(val | cpu->a));
            break;
        case 0x14: case 0x1C: // TRB (CMOS)
            SET_FLAG_Z(cpu->a & val);
            mem_write(cpu, addr, (uint8_t)(val & ~cpu->a));
            break;

        // --- RMB / SMB (CMOS) ---
        case 0x07: case 0x17: case 0x27: case 0x37:
        case 0x47: case 0x57: case 0x67: case 0x77:
            mem_write(cpu, addr, (uint8_t)(val & ~(1 << ((op >> 4) & 7))));
            break;
        case 0x87: case 0x97: case 0xA7: case 0xB7:
        case 0xC7: case 0xD7: case 0xE7: case 0xF7:
            mem_write(cpu, addr, (uint8_t)(val | (1 << ((op >> 4) & 7))));
            break;

        // --- Branches (BRA is CMOS) ---
        case 0x10: op_branch(cpu, !GET_FLAG(FLAG_N)); break; // BPL
        case 0x30: op_branch(cpu, GET_FLAG(FLAG_N)); break;  // BMI
        case 0x50: op_branch(cpu, !GET_FLAG(FLAG_V)); break; // BVC
        case 0x70: op_branch(cpu, GET_FLAG(FLAG_V)); break;  // BVS
        case 0x90: op_branch(cpu, !GET_FLAG(FLAG_C)); break; // BCC
        case 0xB0: op_branch(cpu, GET_FLAG(FLAG_C)); break;  // BCS
        case 0xD0: op_branch(cpu, !GET_FLAG(FLAG_Z)); break; // BNE
        case 0xF0: op_branch(cpu, GET_FLAG(FLAG_Z)); break;  // BEQ
        case 0x80: op_branch(cpu, 1); break;                 // BRA

        // --- BBRn / BBSn (CMOS) ---
        case 0x0F: case 0x1F: case 0x2F: case 0x3F:
        case 0x4F: case 0x5F: case 0x6F: case 0x7F:
            {
                uint8_t zp_val = mem_read(cpu, fetch8(cpu));
                op_branch(cpu, !(zp_val & (1 << ((op >> 4) & 7))));
            }
            break;
        case 0x8F: case 0x9F: case 0xAF: case 0xBF:
        case 0xCF: case 0xDF: case 0xEF: case 0xFF:
            {
                uint8_t zp_val = mem_read(cpu, fetch8(cpu));
                op_branch(cpu, (zp_val & (1 << ((op >> 4) & 7))) != 0);
            }
            break;

        // --- Jumps / subroutines ---
        case 0x4C: case 0x6C: case 0x7C: // JMP abs / (abs) / (abs,X)
            cpu->pc = addr;
            break;
        case 0x20: // JSR
            push_byte(cpu, (uint8_t)((cpu->pc - 1) >> 8));
            push_byte(cpu, (uint8_t)(cpu->pc - 1));
            cpu->pc = addr;
            break;
        case 0x60: // RTS
            {
                uint16_t lo = pop_byte(cpu);
                uint16_t hi = pop_byte(cpu);
                cpu->pc = (uint16_t)((lo | (hi << 8)) + 1);
            }
            break;
        case 0x40: // RTI
            {
                cpu->p = (uint8_t)((pop_byte(cpu) & ~FLAG_B) | FLAG_U);
                uint16_t lo = pop_byte(cpu);
                uint16_t hi = pop_byte(cpu);
                cpu->pc = (uint16_t)(lo | (hi << 8));
            }
            break;

        // --- Stack (PHX/PLX/PHY/PLY are CMOS) ---
        case 0x48: push_byte(cpu, cpu->a); break;                    // PHA
        case 0x68: cpu->a = pop_byte(cpu); set_zn(cpu, cpu->a); break; // PLA
        case 0xDA: push_byte(cpu, cpu->x); break;                    // PHX
        case 0xFA: cpu->x = pop_byte(cpu); set_zn(cpu, cpu->x); break; // PLX
        case 0x5A: push_byte(cpu, cpu->y); break;                    // PHY
        case 0x7A: cpu->y = pop_byte(cpu); set_zn(cpu, cpu->y); break; // PLY
        case 0x08: push_byte(cpu, cpu->p | FLAG_B | FLAG_U); break;  // PHP
        case 0x28: cpu->p = (uint8_t)((pop_byte(cpu) & ~FLAG_B) | FLAG_U); break; // PLP

        // --- Transfers ---
        case 0xAA: cpu->x = cpu->a; set_zn(cpu, cpu->x); break; // TAX
        case 0x8A: cpu->a = cpu->x; set_zn(cpu, cpu->a); break; // TXA
        case 0xA8: cpu->y = cpu->a; set_zn(cpu, cpu->y); break; // TAY
        case 0x98: cpu->a = cpu->y; set_zn(cpu, cpu->a); break; // TYA
        case 0x9A: cpu->sp = cpu->x; break;                     // TXS
        case 0xBA: cpu->x = cpu->sp; set_zn(cpu, cpu->x); break; // TSX

        // --- Register INC/DEC ---
        case 0xE8: cpu->x++; set_zn(cpu, cpu->x); break; // INX
        case 0xCA: cpu->x--; set_zn(cpu, cpu->x); break; // DEX
        case 0xC8: cpu->y++; set_zn(cpu, cpu->y); break; // INY
        case 0x88: cpu->y--; set_zn(cpu, cpu->y); break; // DEY

        // --- Flag operations ---
        case 0x18: cpu->p &= ~FLAG_C; break; // CLC
        case 0x38: cpu->p |= FLAG_C; break;  // SEC
        case 0x58: cpu->p &= ~FLAG_I; break; // CLI
        case 0x78: cpu->p |= FLAG_I; break;  // SEI
        case 0xB8: cpu->p &= ~FLAG_V; break; // CLV
        case 0xD8: cpu->p &= ~FLAG_D; break; // CLD
        case 0xF8: cpu->p |= FLAG_D; break;  // SED

        // --- NOP (0xEA and all remaining CMOS no-op holes) ---
        default:
            break;
    }

    return 0;
}

void w65c02_print_state(void *context) {
    if (!context) return;
    W65C02_CPU *cpu = (W65C02_CPU*)context;

    printf("A:%02X X:%02X Y:%02X SP:%02X PC:%04X P:%02X [%c%c%c%c%c%c%c] Ticks:%u%s\n",
           cpu->a, cpu->x, cpu->y, cpu->sp, cpu->pc, cpu->p,
           GET_FLAG(FLAG_N) ? 'N' : '-',
           GET_FLAG(FLAG_V) ? 'V' : '-',
           GET_FLAG(FLAG_B) ? 'B' : '-',
           GET_FLAG(FLAG_D) ? 'D' : '-',
           GET_FLAG(FLAG_I) ? 'I' : '-',
           GET_FLAG(FLAG_Z) ? 'Z' : '-',
           GET_FLAG(FLAG_C) ? 'C' : '-',
           cpu->ticks,
           cpu->halted ? " (halted)" : "");
}

void w65c02_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    W65C02_CPU *cpu = (W65C02_CPU*)context;

    uint16_t pc = cpu->pc;
    uint8_t op = mem_read(cpu, pc);
    uint8_t b1 = mem_read(cpu, (uint16_t)(pc + 1));
    uint8_t b2 = mem_read(cpu, (uint16_t)(pc + 2));
    uint16_t abs_addr = (uint16_t)(b1 | ((uint16_t)b2 << 8));
    const char *mnem = g_mnem[op];

    switch (g_mode[op]) {
        case M_IMP:
            snprintf(buf, buf_len, "%04X: %s", pc, mnem);
            break;
        case M_ACC:
            snprintf(buf, buf_len, "%04X: %s A", pc, mnem);
            break;
        case M_IMM:
            snprintf(buf, buf_len, "%04X: %s #$%02X", pc, mnem, b1);
            break;
        case M_ZP:
            snprintf(buf, buf_len, "%04X: %s $%02X", pc, mnem, b1);
            break;
        case M_ZPX:
            snprintf(buf, buf_len, "%04X: %s $%02X,X", pc, mnem, b1);
            break;
        case M_ZPY:
            snprintf(buf, buf_len, "%04X: %s $%02X,Y", pc, mnem, b1);
            break;
        case M_ABS:
            snprintf(buf, buf_len, "%04X: %s $%04X", pc, mnem, abs_addr);
            break;
        case M_ABX:
            snprintf(buf, buf_len, "%04X: %s $%04X,X", pc, mnem, abs_addr);
            break;
        case M_ABY:
            snprintf(buf, buf_len, "%04X: %s $%04X,Y", pc, mnem, abs_addr);
            break;
        case M_IND:
            snprintf(buf, buf_len, "%04X: %s ($%04X)", pc, mnem, abs_addr);
            break;
        case M_IAX:
            snprintf(buf, buf_len, "%04X: %s ($%04X,X)", pc, mnem, abs_addr);
            break;
        case M_INX:
            snprintf(buf, buf_len, "%04X: %s ($%02X,X)", pc, mnem, b1);
            break;
        case M_INY:
            snprintf(buf, buf_len, "%04X: %s ($%02X),Y", pc, mnem, b1);
            break;
        case M_IZP:
            snprintf(buf, buf_len, "%04X: %s ($%02X)", pc, mnem, b1);
            break;
        case M_REL:
            snprintf(buf, buf_len, "%04X: %s $%04X", pc, mnem,
                     (uint16_t)(pc + 2 + (int8_t)b1));
            break;
        case M_ZPR:
            snprintf(buf, buf_len, "%04X: %s $%02X,$%04X", pc, mnem, b1,
                     (uint16_t)(pc + 3 + (int8_t)b2));
            break;
        default:
            snprintf(buf, buf_len, "%04X: ???", pc);
            break;
    }
}
