#include "spc700.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Sony SPC700 (SNES APU / S-SMP core)
// PSW flag layout: N V P B H I Z C
#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_H 0x08
#define FLAG_B 0x10
#define FLAG_P 0x20
#define FLAG_V 0x40
#define FLAG_N 0x80

typedef struct SPC700_CPU {
    uint8_t ram[65536];
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint8_t psw;
    uint16_t pc;
    uint32_t ticks;
    int halted;
} SPC700_CPU;

#define GET_FLAG(flag) ((cpu->psw & (flag)) ? 1 : 0)

// Base cycle counts per opcode (taken branches add 2)
static const uint8_t g_cycles[256] = {
    2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 5, 4, 5, 4, 6, 8,
    2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 6, 5, 2, 2, 4, 6,
    2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 5, 4, 5, 4, 5, 4,
    2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 6, 5, 2, 2, 3, 8,
    2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 4, 4, 5, 4, 6, 6,
    2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 4, 5, 2, 2, 4, 3,
    2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 4, 4, 5, 4, 5, 5,
    2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 5, 5, 2, 2, 3, 6,
    2, 8, 4, 5, 3, 4, 3, 6, 2, 6, 5, 4, 5, 2, 4, 5,
    2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 5, 5, 2, 2, 12, 5,
    3, 8, 4, 5, 3, 4, 3, 6, 2, 6, 4, 4, 5, 2, 4, 4,
    2, 8, 4, 5, 4, 5, 5, 6, 5, 5, 5, 5, 2, 2, 3, 4,
    3, 8, 4, 5, 4, 5, 4, 7, 2, 5, 6, 4, 5, 2, 4, 9,
    2, 8, 4, 5, 5, 6, 6, 7, 4, 5, 5, 5, 2, 2, 6, 3,
    2, 8, 4, 5, 3, 4, 3, 6, 2, 4, 5, 3, 4, 3, 4, 3,
    2, 8, 4, 5, 4, 5, 5, 6, 3, 4, 5, 4, 2, 2, 4, 3
};

// ---------- Memory / fetch helpers ----------

static uint8_t rd(SPC700_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr];
}

static void wr(SPC700_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr] = val;
}

static uint16_t dp_addr(SPC700_CPU *cpu, uint8_t offset) {
    return (uint16_t)(((cpu->psw & FLAG_P) ? 0x0100 : 0x0000) + offset);
}

static uint8_t fetch8(SPC700_CPU *cpu) {
    return rd(cpu, cpu->pc++);
}

static uint16_t fetch16(SPC700_CPU *cpu) {
    uint16_t lo = fetch8(cpu);
    uint16_t hi = fetch8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

static uint16_t rd16(SPC700_CPU *cpu, uint16_t addr) {
    return (uint16_t)(rd(cpu, addr) | ((uint16_t)rd(cpu, (uint16_t)(addr + 1)) << 8));
}

// Direct-page 16-bit access (offset wraps within the page)
static uint16_t rd16_dp(SPC700_CPU *cpu, uint8_t offset) {
    return (uint16_t)(rd(cpu, dp_addr(cpu, offset)) |
                      ((uint16_t)rd(cpu, dp_addr(cpu, (uint8_t)(offset + 1))) << 8));
}

static void wr16_dp(SPC700_CPU *cpu, uint8_t offset, uint16_t val) {
    wr(cpu, dp_addr(cpu, offset), (uint8_t)(val & 0xFF));
    wr(cpu, dp_addr(cpu, (uint8_t)(offset + 1)), (uint8_t)(val >> 8));
}

static void push8(SPC700_CPU *cpu, uint8_t val) {
    wr(cpu, (uint16_t)(0x0100 + cpu->sp), val);
    cpu->sp--;
}

static uint8_t pop8(SPC700_CPU *cpu) {
    cpu->sp++;
    return rd(cpu, (uint16_t)(0x0100 + cpu->sp));
}

static void push16(SPC700_CPU *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val >> 8));
    push8(cpu, (uint8_t)(val & 0xFF));
}

static uint16_t pop16(SPC700_CPU *cpu) {
    uint16_t lo = pop8(cpu);
    uint16_t hi = pop8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

// ---------- Flag helpers ----------

static void set_flag(SPC700_CPU *cpu, uint8_t flag, int cond) {
    if (cond) cpu->psw |= flag; else cpu->psw = (uint8_t)(cpu->psw & ~flag);
}

static void set_nz(SPC700_CPU *cpu, uint8_t val) {
    set_flag(cpu, FLAG_N, val & 0x80);
    set_flag(cpu, FLAG_Z, val == 0);
}

static void set_nz16(SPC700_CPU *cpu, uint16_t val) {
    set_flag(cpu, FLAG_N, val & 0x8000);
    set_flag(cpu, FLAG_Z, val == 0);
}

// ---------- ALU helpers ----------

static uint8_t alu_logic(SPC700_CPU *cpu, uint8_t result) {
    set_nz(cpu, result);
    return result;
}

static uint8_t alu_adc(SPC700_CPU *cpu, uint8_t a, uint8_t b) {
    unsigned c = cpu->psw & FLAG_C;
    unsigned r = (unsigned)a + (unsigned)b + c;
    set_flag(cpu, FLAG_H, ((a & 0x0F) + (b & 0x0F) + c) > 0x0F);
    set_flag(cpu, FLAG_V, (~(a ^ b) & (a ^ r) & 0x80) != 0);
    set_flag(cpu, FLAG_C, r > 0xFF);
    set_nz(cpu, (uint8_t)r);
    return (uint8_t)r;
}

static uint8_t alu_sbc(SPC700_CPU *cpu, uint8_t a, uint8_t b) {
    int c = (cpu->psw & FLAG_C) ? 0 : 1;
    int r = (int)a - (int)b - c;
    set_flag(cpu, FLAG_H, ((a & 0x0F) - (b & 0x0F) - c) >= 0);
    set_flag(cpu, FLAG_V, ((a ^ b) & (a ^ r) & 0x80) != 0);
    set_flag(cpu, FLAG_C, r >= 0);
    set_nz(cpu, (uint8_t)r);
    return (uint8_t)r;
}

static void alu_cmp(SPC700_CPU *cpu, uint8_t a, uint8_t b) {
    int r = (int)a - (int)b;
    set_flag(cpu, FLAG_C, r >= 0);
    set_nz(cpu, (uint8_t)r);
}

static uint8_t alu_asl(SPC700_CPU *cpu, uint8_t val) {
    set_flag(cpu, FLAG_C, val & 0x80);
    val = (uint8_t)(val << 1);
    set_nz(cpu, val);
    return val;
}

static uint8_t alu_lsr(SPC700_CPU *cpu, uint8_t val) {
    set_flag(cpu, FLAG_C, val & 0x01);
    val = (uint8_t)(val >> 1);
    set_nz(cpu, val);
    return val;
}

static uint8_t alu_rol(SPC700_CPU *cpu, uint8_t val) {
    uint8_t old_c = (uint8_t)(cpu->psw & FLAG_C);
    set_flag(cpu, FLAG_C, val & 0x80);
    val = (uint8_t)((val << 1) | old_c);
    set_nz(cpu, val);
    return val;
}

static uint8_t alu_ror(SPC700_CPU *cpu, uint8_t val) {
    uint8_t old_c = (uint8_t)(cpu->psw & FLAG_C);
    set_flag(cpu, FLAG_C, val & 0x01);
    val = (uint8_t)((val >> 1) | (old_c << 7));
    set_nz(cpu, val);
    return val;
}

static uint8_t alu_inc(SPC700_CPU *cpu, uint8_t val) {
    val = (uint8_t)(val + 1);
    set_nz(cpu, val);
    return val;
}

static uint8_t alu_dec(SPC700_CPU *cpu, uint8_t val) {
    val = (uint8_t)(val - 1);
    set_nz(cpu, val);
    return val;
}

// Dispatch for the regular ALU columns (sel = opcode >> 5)
static uint8_t alu_apply(SPC700_CPU *cpu, uint8_t sel, uint8_t a, uint8_t b) {
    switch (sel) {
        case 0: return alu_logic(cpu, (uint8_t)(a | b)); // OR
        case 1: return alu_logic(cpu, (uint8_t)(a & b)); // AND
        case 2: return alu_logic(cpu, (uint8_t)(a ^ b)); // EOR
        case 3: alu_cmp(cpu, a, b); return a;            // CMP
        case 4: return alu_adc(cpu, a, b);               // ADC
        default: return alu_sbc(cpu, a, b);              // SBC
    }
}

// Effective address for the regular memory operand columns
// (op & 0x1F): 0x04 dp, 0x05 !abs, 0x06 (X), 0x07 [dp+X],
//              0x14 dp+X, 0x15 !abs+X, 0x16 !abs+Y, 0x17 [dp]+Y
static uint16_t operand_ea(SPC700_CPU *cpu, uint8_t op) {
    switch (op & 0x1F) {
        case 0x04: return dp_addr(cpu, fetch8(cpu));
        case 0x05: return fetch16(cpu);
        case 0x06: return dp_addr(cpu, cpu->x);
        case 0x07: return rd16_dp(cpu, (uint8_t)(fetch8(cpu) + cpu->x));
        case 0x14: return dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x));
        case 0x15: return (uint16_t)(fetch16(cpu) + cpu->x);
        case 0x16: return (uint16_t)(fetch16(cpu) + cpu->y);
        case 0x17: return (uint16_t)(rd16_dp(cpu, fetch8(cpu)) + cpu->y);
        default:   return 0;
    }
}

static void branch(SPC700_CPU *cpu, int cond) {
    int8_t rel = (int8_t)fetch8(cpu);
    if (cond) {
        cpu->pc = (uint16_t)(cpu->pc + rel);
        cpu->ticks += 2;
    }
}

// Fetch a 13-bit address + 3-bit bit number operand (bit ops)
static uint16_t fetch_membit(SPC700_CPU *cpu, uint8_t *bit) {
    uint16_t w = fetch16(cpu);
    *bit = (uint8_t)(w >> 13);
    return (uint16_t)(w & 0x1FFF);
}

// ---------- Lifecycle ----------

void* spc700_create(void) {
    SPC700_CPU *cpu = (SPC700_CPU*)calloc(1, sizeof(SPC700_CPU));
    return cpu;
}

void spc700_destroy(void *context) {
    free(context);
}

int spc700_init(void *context) {
    if (!context) return -1;
    SPC700_CPU *cpu = (SPC700_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xEF; // IPL ROM leaves SP at 0xEF
    cpu->psw = FLAG_Z;
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int spc700_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    SPC700_CPU *cpu = (SPC700_CPU*)context;

    if (address >= 65536) return -2;
    size_t copy_len = size;
    if (address + size > 65536) {
        copy_len = 65536 - address;
    }
    memcpy(cpu->ram + address, data, copy_len);
    cpu->pc = (uint16_t)address;

    return 0;
}

// ---------- Execution ----------

int spc700_step(void *context) {
    if (!context) return -1;
    SPC700_CPU *cpu = (SPC700_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks += g_cycles[op];

    uint8_t col = (uint8_t)(op & 0x1F);
    uint8_t sel = (uint8_t)(op >> 5);

    // Regular columns: ALU rows (OR/AND/EOR/CMP/ADC/SBC) and MOV A rows
    if ((col >= 0x04 && col <= 0x09) || (col >= 0x14 && col <= 0x19)) {
        if (sel < 6) {
            int is_cmp = (sel == 3);
            switch (col) {
                case 0x08: // OP A, #imm
                    cpu->a = alu_apply(cpu, sel, cpu->a, fetch8(cpu));
                    break;
                case 0x09: { // OP dp(dst), dp(src)
                    uint8_t src = fetch8(cpu);
                    uint8_t dst = fetch8(cpu);
                    uint16_t da = dp_addr(cpu, dst);
                    uint8_t r = alu_apply(cpu, sel, rd(cpu, da), rd(cpu, dp_addr(cpu, src)));
                    if (!is_cmp) wr(cpu, da, r);
                    break;
                }
                case 0x18: { // OP dp, #imm
                    uint8_t imm = fetch8(cpu);
                    uint16_t da = dp_addr(cpu, fetch8(cpu));
                    uint8_t r = alu_apply(cpu, sel, rd(cpu, da), imm);
                    if (!is_cmp) wr(cpu, da, r);
                    break;
                }
                case 0x19: { // OP (X), (Y)
                    uint16_t xa = dp_addr(cpu, cpu->x);
                    uint8_t r = alu_apply(cpu, sel, rd(cpu, xa), rd(cpu, dp_addr(cpu, cpu->y)));
                    if (!is_cmp) wr(cpu, xa, r);
                    break;
                }
                default: // OP A, mem
                    cpu->a = alu_apply(cpu, sel, cpu->a, rd(cpu, operand_ea(cpu, op)));
                    break;
            }
            return 0;
        }
        if ((col >= 0x04 && col <= 0x07) || (col >= 0x14 && col <= 0x17)) {
            if (sel == 6) { // MOV mem, A
                wr(cpu, operand_ea(cpu, op), cpu->a);
            } else {        // MOV A, mem
                cpu->a = rd(cpu, operand_ea(cpu, op));
                set_nz(cpu, cpu->a);
            }
            return 0;
        }
        // Columns 8/9/18/19 of the MOV rows fall through to the switch below
    }

    switch (op) {
        // --- Column 0: flag ops and branches ---
        case 0x00: break; // NOP
        case 0x20: cpu->psw &= (uint8_t)~FLAG_P; break; // CLRP
        case 0x40: cpu->psw |= FLAG_P; break;           // SETP
        case 0x60: cpu->psw &= (uint8_t)~FLAG_C; break; // CLRC
        case 0x80: cpu->psw |= FLAG_C; break;           // SETC
        case 0xA0: cpu->psw |= FLAG_I; break;           // EI
        case 0xC0: cpu->psw &= (uint8_t)~FLAG_I; break; // DI
        case 0xE0: cpu->psw &= (uint8_t)~(FLAG_V | FLAG_H); break; // CLRV
        case 0xED: set_flag(cpu, FLAG_C, !GET_FLAG(FLAG_C)); break; // NOTC

        case 0x10: branch(cpu, !GET_FLAG(FLAG_N)); break; // BPL
        case 0x30: branch(cpu, GET_FLAG(FLAG_N)); break;  // BMI
        case 0x50: branch(cpu, !GET_FLAG(FLAG_V)); break; // BVC
        case 0x70: branch(cpu, GET_FLAG(FLAG_V)); break;  // BVS
        case 0x90: branch(cpu, !GET_FLAG(FLAG_C)); break; // BCC
        case 0xB0: branch(cpu, GET_FLAG(FLAG_C)); break;  // BCS
        case 0xD0: branch(cpu, !GET_FLAG(FLAG_Z)); break; // BNE
        case 0xF0: branch(cpu, GET_FLAG(FLAG_Z)); break;  // BEQ
        case 0x2F: branch(cpu, 1); break;                 // BRA

        // --- Column 1: TCALL n ---
        case 0x01: case 0x11: case 0x21: case 0x31:
        case 0x41: case 0x51: case 0x61: case 0x71:
        case 0x81: case 0x91: case 0xA1: case 0xB1:
        case 0xC1: case 0xD1: case 0xE1: case 0xF1: {
            uint16_t vec = (uint16_t)(0xFFDE - 2 * (op >> 4));
            push16(cpu, cpu->pc);
            cpu->pc = rd16(cpu, vec);
            break;
        }

        // --- Column 2: SET1 / CLR1 dp.bit ---
        case 0x02: case 0x12: case 0x22: case 0x32:
        case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x82: case 0x92: case 0xA2: case 0xB2:
        case 0xC2: case 0xD2: case 0xE2: case 0xF2: {
            uint16_t addr = dp_addr(cpu, fetch8(cpu));
            uint8_t mask = (uint8_t)(1 << (op >> 5));
            uint8_t val = rd(cpu, addr);
            if (op & 0x10) val = (uint8_t)(val & ~mask); // CLR1
            else val |= mask;                            // SET1
            wr(cpu, addr, val);
            break;
        }

        // --- Column 3: BBS / BBC dp.bit, rel ---
        case 0x03: case 0x13: case 0x23: case 0x33:
        case 0x43: case 0x53: case 0x63: case 0x73:
        case 0x83: case 0x93: case 0xA3: case 0xB3:
        case 0xC3: case 0xD3: case 0xE3: case 0xF3: {
            uint8_t val = rd(cpu, dp_addr(cpu, fetch8(cpu)));
            uint8_t mask = (uint8_t)(1 << (op >> 5));
            int bit_set = (val & mask) != 0;
            branch(cpu, (op & 0x10) ? !bit_set : bit_set);
            break;
        }

        // --- Column A: 1-bit ops and 16-bit word ops ---
        case 0x0A: { // OR1 C, m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            set_flag(cpu, FLAG_C, GET_FLAG(FLAG_C) | ((rd(cpu, addr) >> bit) & 1));
            break;
        }
        case 0x2A: { // OR1 C, /m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            set_flag(cpu, FLAG_C, GET_FLAG(FLAG_C) | (((rd(cpu, addr) >> bit) & 1) ^ 1));
            break;
        }
        case 0x4A: { // AND1 C, m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            set_flag(cpu, FLAG_C, GET_FLAG(FLAG_C) & ((rd(cpu, addr) >> bit) & 1));
            break;
        }
        case 0x6A: { // AND1 C, /m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            set_flag(cpu, FLAG_C, GET_FLAG(FLAG_C) & (((rd(cpu, addr) >> bit) & 1) ^ 1));
            break;
        }
        case 0x8A: { // EOR1 C, m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            set_flag(cpu, FLAG_C, GET_FLAG(FLAG_C) ^ ((rd(cpu, addr) >> bit) & 1));
            break;
        }
        case 0xAA: { // MOV1 C, m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            set_flag(cpu, FLAG_C, (rd(cpu, addr) >> bit) & 1);
            break;
        }
        case 0xCA: { // MOV1 m.b, C
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            uint8_t val = rd(cpu, addr);
            if (GET_FLAG(FLAG_C)) val |= (uint8_t)(1 << bit);
            else val = (uint8_t)(val & ~(1 << bit));
            wr(cpu, addr, val);
            break;
        }
        case 0xEA: { // NOT1 m.b
            uint8_t bit;
            uint16_t addr = fetch_membit(cpu, &bit);
            wr(cpu, addr, (uint8_t)(rd(cpu, addr) ^ (1 << bit)));
            break;
        }
        case 0x1A: { // DECW dp
            uint8_t off = fetch8(cpu);
            uint16_t w = (uint16_t)(rd16_dp(cpu, off) - 1);
            wr16_dp(cpu, off, w);
            set_nz16(cpu, w);
            break;
        }
        case 0x3A: { // INCW dp
            uint8_t off = fetch8(cpu);
            uint16_t w = (uint16_t)(rd16_dp(cpu, off) + 1);
            wr16_dp(cpu, off, w);
            set_nz16(cpu, w);
            break;
        }
        case 0x5A: { // CMPW YA, dp
            uint16_t ya = (uint16_t)(((uint16_t)cpu->y << 8) | cpu->a);
            uint16_t w = rd16_dp(cpu, fetch8(cpu));
            int r = (int)ya - (int)w;
            set_flag(cpu, FLAG_C, r >= 0);
            set_nz16(cpu, (uint16_t)r);
            break;
        }
        case 0x7A: { // ADDW YA, dp
            uint16_t ya = (uint16_t)(((uint16_t)cpu->y << 8) | cpu->a);
            uint16_t w = rd16_dp(cpu, fetch8(cpu));
            uint32_t sum = (uint32_t)ya + w;
            set_flag(cpu, FLAG_H, ((ya & 0x0FFF) + (w & 0x0FFF)) > 0x0FFF);
            set_flag(cpu, FLAG_V, (~(ya ^ w) & (ya ^ sum) & 0x8000) != 0);
            set_flag(cpu, FLAG_C, sum > 0xFFFF);
            cpu->a = (uint8_t)(sum & 0xFF);
            cpu->y = (uint8_t)((sum >> 8) & 0xFF);
            set_nz16(cpu, (uint16_t)sum);
            break;
        }
        case 0x9A: { // SUBW YA, dp
            uint16_t ya = (uint16_t)(((uint16_t)cpu->y << 8) | cpu->a);
            uint16_t w = rd16_dp(cpu, fetch8(cpu));
            int r = (int)ya - (int)w;
            set_flag(cpu, FLAG_H, ((ya & 0x0FFF) - (w & 0x0FFF)) >= 0);
            set_flag(cpu, FLAG_V, ((ya ^ w) & (ya ^ r) & 0x8000) != 0);
            set_flag(cpu, FLAG_C, r >= 0);
            cpu->a = (uint8_t)(r & 0xFF);
            cpu->y = (uint8_t)((r >> 8) & 0xFF);
            set_nz16(cpu, (uint16_t)r);
            break;
        }
        case 0xBA: { // MOVW YA, dp
            uint16_t w = rd16_dp(cpu, fetch8(cpu));
            cpu->a = (uint8_t)(w & 0xFF);
            cpu->y = (uint8_t)(w >> 8);
            set_nz16(cpu, w);
            break;
        }
        case 0xDA: { // MOVW dp, YA
            uint8_t off = fetch8(cpu);
            wr16_dp(cpu, off, (uint16_t)(((uint16_t)cpu->y << 8) | cpu->a));
            break;
        }
        case 0xFA: { // MOV dp(dst), dp(src)
            uint8_t src = fetch8(cpu);
            uint8_t dst = fetch8(cpu);
            wr(cpu, dp_addr(cpu, dst), rd(cpu, dp_addr(cpu, src)));
            break;
        }

        // --- Column B: shifts / INC / DEC dp, dp+X and MOV Y variants ---
        case 0x0B: { uint16_t a = dp_addr(cpu, fetch8(cpu)); wr(cpu, a, alu_asl(cpu, rd(cpu, a))); break; } // ASL dp
        case 0x1B: { uint16_t a = dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)); wr(cpu, a, alu_asl(cpu, rd(cpu, a))); break; } // ASL dp+X
        case 0x2B: { uint16_t a = dp_addr(cpu, fetch8(cpu)); wr(cpu, a, alu_rol(cpu, rd(cpu, a))); break; } // ROL dp
        case 0x3B: { uint16_t a = dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)); wr(cpu, a, alu_rol(cpu, rd(cpu, a))); break; } // ROL dp+X
        case 0x4B: { uint16_t a = dp_addr(cpu, fetch8(cpu)); wr(cpu, a, alu_lsr(cpu, rd(cpu, a))); break; } // LSR dp
        case 0x5B: { uint16_t a = dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)); wr(cpu, a, alu_lsr(cpu, rd(cpu, a))); break; } // LSR dp+X
        case 0x6B: { uint16_t a = dp_addr(cpu, fetch8(cpu)); wr(cpu, a, alu_ror(cpu, rd(cpu, a))); break; } // ROR dp
        case 0x7B: { uint16_t a = dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)); wr(cpu, a, alu_ror(cpu, rd(cpu, a))); break; } // ROR dp+X
        case 0x8B: { uint16_t a = dp_addr(cpu, fetch8(cpu)); wr(cpu, a, alu_dec(cpu, rd(cpu, a))); break; } // DEC dp
        case 0x9B: { uint16_t a = dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)); wr(cpu, a, alu_dec(cpu, rd(cpu, a))); break; } // DEC dp+X
        case 0xAB: { uint16_t a = dp_addr(cpu, fetch8(cpu)); wr(cpu, a, alu_inc(cpu, rd(cpu, a))); break; } // INC dp
        case 0xBB: { uint16_t a = dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)); wr(cpu, a, alu_inc(cpu, rd(cpu, a))); break; } // INC dp+X
        case 0xCB: wr(cpu, dp_addr(cpu, fetch8(cpu)), cpu->y); break; // MOV dp, Y
        case 0xDB: wr(cpu, dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)), cpu->y); break; // MOV dp+X, Y
        case 0xEB: cpu->y = rd(cpu, dp_addr(cpu, fetch8(cpu))); set_nz(cpu, cpu->y); break; // MOV Y, dp
        case 0xFB: cpu->y = rd(cpu, dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x))); set_nz(cpu, cpu->y); break; // MOV Y, dp+X

        // --- Column C: shifts / INC / DEC !abs and A, MOV Y abs ---
        case 0x0C: { uint16_t a = fetch16(cpu); wr(cpu, a, alu_asl(cpu, rd(cpu, a))); break; } // ASL !abs
        case 0x1C: cpu->a = alu_asl(cpu, cpu->a); break; // ASL A
        case 0x2C: { uint16_t a = fetch16(cpu); wr(cpu, a, alu_rol(cpu, rd(cpu, a))); break; } // ROL !abs
        case 0x3C: cpu->a = alu_rol(cpu, cpu->a); break; // ROL A
        case 0x4C: { uint16_t a = fetch16(cpu); wr(cpu, a, alu_lsr(cpu, rd(cpu, a))); break; } // LSR !abs
        case 0x5C: cpu->a = alu_lsr(cpu, cpu->a); break; // LSR A
        case 0x6C: { uint16_t a = fetch16(cpu); wr(cpu, a, alu_ror(cpu, rd(cpu, a))); break; } // ROR !abs
        case 0x7C: cpu->a = alu_ror(cpu, cpu->a); break; // ROR A
        case 0x8C: { uint16_t a = fetch16(cpu); wr(cpu, a, alu_dec(cpu, rd(cpu, a))); break; } // DEC !abs
        case 0x9C: cpu->a = alu_dec(cpu, cpu->a); break; // DEC A
        case 0xAC: { uint16_t a = fetch16(cpu); wr(cpu, a, alu_inc(cpu, rd(cpu, a))); break; } // INC !abs
        case 0xBC: cpu->a = alu_inc(cpu, cpu->a); break; // INC A
        case 0xCC: wr(cpu, fetch16(cpu), cpu->y); break; // MOV !abs, Y
        case 0xDC: cpu->y = alu_dec(cpu, cpu->y); break; // DEC Y
        case 0xEC: cpu->y = rd(cpu, fetch16(cpu)); set_nz(cpu, cpu->y); break; // MOV Y, !abs
        case 0xFC: cpu->y = alu_inc(cpu, cpu->y); break; // INC Y

        // --- Column D: push, register transfers, immediates ---
        case 0x0D: push8(cpu, cpu->psw); break; // PUSH PSW
        case 0x2D: push8(cpu, cpu->a); break;   // PUSH A
        case 0x4D: push8(cpu, cpu->x); break;   // PUSH X
        case 0x6D: push8(cpu, cpu->y); break;   // PUSH Y
        case 0x1D: cpu->x = alu_dec(cpu, cpu->x); break; // DEC X
        case 0x3D: cpu->x = alu_inc(cpu, cpu->x); break; // INC X
        case 0x5D: cpu->x = cpu->a; set_nz(cpu, cpu->x); break;  // MOV X, A
        case 0x7D: cpu->a = cpu->x; set_nz(cpu, cpu->a); break;  // MOV A, X
        case 0x9D: cpu->x = cpu->sp; set_nz(cpu, cpu->x); break; // MOV X, SP
        case 0xBD: cpu->sp = cpu->x; break;                      // MOV SP, X
        case 0xDD: cpu->a = cpu->y; set_nz(cpu, cpu->a); break;  // MOV A, Y
        case 0xFD: cpu->y = cpu->a; set_nz(cpu, cpu->y); break;  // MOV Y, A
        case 0x8D: cpu->y = fetch8(cpu); set_nz(cpu, cpu->y); break; // MOV Y, #imm
        case 0xCD: cpu->x = fetch8(cpu); set_nz(cpu, cpu->x); break; // MOV X, #imm
        case 0xAD: alu_cmp(cpu, cpu->y, fetch8(cpu)); break;         // CMP Y, #imm

        // --- Column E: X/Y compares, pop, decimal adjust, CBNE/DBNZ ---
        case 0x1E: alu_cmp(cpu, cpu->x, rd(cpu, fetch16(cpu))); break;           // CMP X, !abs
        case 0x3E: alu_cmp(cpu, cpu->x, rd(cpu, dp_addr(cpu, fetch8(cpu)))); break; // CMP X, dp
        case 0x5E: alu_cmp(cpu, cpu->y, rd(cpu, fetch16(cpu))); break;           // CMP Y, !abs
        case 0x7E: alu_cmp(cpu, cpu->y, rd(cpu, dp_addr(cpu, fetch8(cpu)))); break; // CMP Y, dp
        case 0x0E: { // TSET1 !abs
            uint16_t addr = fetch16(cpu);
            uint8_t val = rd(cpu, addr);
            set_nz(cpu, (uint8_t)(cpu->a - val));
            wr(cpu, addr, (uint8_t)(val | cpu->a));
            break;
        }
        case 0x4E: { // TCLR1 !abs
            uint16_t addr = fetch16(cpu);
            uint8_t val = rd(cpu, addr);
            set_nz(cpu, (uint8_t)(cpu->a - val));
            wr(cpu, addr, (uint8_t)(val & ~cpu->a));
            break;
        }
        case 0x2E: { // CBNE dp, rel
            uint8_t val = rd(cpu, dp_addr(cpu, fetch8(cpu)));
            branch(cpu, cpu->a != val);
            break;
        }
        case 0xDE: { // CBNE dp+X, rel
            uint8_t val = rd(cpu, dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->x)));
            branch(cpu, cpu->a != val);
            break;
        }
        case 0x6E: { // DBNZ dp, rel
            uint16_t addr = dp_addr(cpu, fetch8(cpu));
            uint8_t val = (uint8_t)(rd(cpu, addr) - 1);
            wr(cpu, addr, val);
            branch(cpu, val != 0);
            break;
        }
        case 0xFE: // DBNZ Y, rel
            cpu->y = (uint8_t)(cpu->y - 1);
            branch(cpu, cpu->y != 0);
            break;
        case 0x8E: cpu->psw = pop8(cpu); break; // POP PSW
        case 0xAE: cpu->a = pop8(cpu); break;   // POP A
        case 0xCE: cpu->x = pop8(cpu); break;   // POP X
        case 0xEE: cpu->y = pop8(cpu); break;   // POP Y
        case 0x9E: { // DIV YA, X
            uint16_t ya = (uint16_t)(((uint16_t)cpu->y << 8) | cpu->a);
            set_flag(cpu, FLAG_H, (cpu->y & 0x0F) >= (cpu->x & 0x0F));
            set_flag(cpu, FLAG_V, cpu->y >= cpu->x);
            if ((int)cpu->y < ((int)cpu->x << 1)) {
                cpu->a = (uint8_t)(ya / cpu->x);
                cpu->y = (uint8_t)(ya % cpu->x);
            } else {
                cpu->a = (uint8_t)(255 - (ya - ((int)cpu->x << 9)) / (256 - cpu->x));
                cpu->y = (uint8_t)(cpu->x + (ya - ((int)cpu->x << 9)) % (256 - cpu->x));
            }
            set_nz(cpu, cpu->a);
            break;
        }
        case 0xBE: { // DAS A
            if (!GET_FLAG(FLAG_C) || cpu->a > 0x99) {
                cpu->psw &= (uint8_t)~FLAG_C;
                cpu->a = (uint8_t)(cpu->a - 0x60);
            }
            if (!GET_FLAG(FLAG_H) || (cpu->a & 0x0F) > 0x09) {
                cpu->a = (uint8_t)(cpu->a - 0x06);
            }
            set_nz(cpu, cpu->a);
            break;
        }

        // --- Column F: control flow, MOV specials, MUL, DAA, halts ---
        case 0x0F: // BRK
            push16(cpu, cpu->pc);
            push8(cpu, cpu->psw);
            cpu->psw |= FLAG_B;
            cpu->psw &= (uint8_t)~FLAG_I;
            cpu->pc = rd16(cpu, 0xFFDE);
            break;
        case 0x1F: { // JMP [!abs+X]
            uint16_t ptr = (uint16_t)(fetch16(cpu) + cpu->x);
            cpu->pc = rd16(cpu, ptr);
            break;
        }
        case 0x5F: cpu->pc = fetch16(cpu); break; // JMP !abs
        case 0x3F: { // CALL !abs
            uint16_t target = fetch16(cpu);
            push16(cpu, cpu->pc);
            cpu->pc = target;
            break;
        }
        case 0x4F: { // PCALL up
            uint8_t up = fetch8(cpu);
            push16(cpu, cpu->pc);
            cpu->pc = (uint16_t)(0xFF00 | up);
            break;
        }
        case 0x6F: cpu->pc = pop16(cpu); break; // RET
        case 0x7F: // RETI
            cpu->psw = pop8(cpu);
            cpu->pc = pop16(cpu);
            break;
        case 0x8F: { // MOV dp, #imm
            uint8_t imm = fetch8(cpu);
            wr(cpu, dp_addr(cpu, fetch8(cpu)), imm);
            break;
        }
        case 0x9F: // XCN A
            cpu->a = (uint8_t)((cpu->a >> 4) | (cpu->a << 4));
            set_nz(cpu, cpu->a);
            break;
        case 0xAF: // MOV (X)+, A
            wr(cpu, dp_addr(cpu, cpu->x), cpu->a);
            cpu->x++;
            break;
        case 0xBF: // MOV A, (X)+
            cpu->a = rd(cpu, dp_addr(cpu, cpu->x));
            cpu->x++;
            set_nz(cpu, cpu->a);
            break;
        case 0xCF: { // MUL YA
            uint16_t r = (uint16_t)((uint16_t)cpu->y * cpu->a);
            cpu->a = (uint8_t)(r & 0xFF);
            cpu->y = (uint8_t)(r >> 8);
            set_nz(cpu, cpu->y);
            break;
        }
        case 0xDF: { // DAA A
            if (GET_FLAG(FLAG_C) || cpu->a > 0x99) {
                cpu->psw |= FLAG_C;
                cpu->a = (uint8_t)(cpu->a + 0x60);
            }
            if (GET_FLAG(FLAG_H) || (cpu->a & 0x0F) > 0x09) {
                cpu->a = (uint8_t)(cpu->a + 0x06);
            }
            set_nz(cpu, cpu->a);
            break;
        }
        case 0xEF: // SLEEP
        case 0xFF: // STOP
            cpu->halted = 1;
            return 1;

        // --- MOV row columns 8/9 ---
        case 0xC8: alu_cmp(cpu, cpu->x, fetch8(cpu)); break; // CMP X, #imm
        case 0xC9: wr(cpu, fetch16(cpu), cpu->x); break;     // MOV !abs, X
        case 0xD8: wr(cpu, dp_addr(cpu, fetch8(cpu)), cpu->x); break; // MOV dp, X
        case 0xD9: wr(cpu, dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->y)), cpu->x); break; // MOV dp+Y, X
        case 0xE8: cpu->a = fetch8(cpu); set_nz(cpu, cpu->a); break; // MOV A, #imm
        case 0xE9: cpu->x = rd(cpu, fetch16(cpu)); set_nz(cpu, cpu->x); break; // MOV X, !abs
        case 0xF8: cpu->x = rd(cpu, dp_addr(cpu, fetch8(cpu))); set_nz(cpu, cpu->x); break; // MOV X, dp
        case 0xF9: cpu->x = rd(cpu, dp_addr(cpu, (uint8_t)(fetch8(cpu) + cpu->y))); set_nz(cpu, cpu->x); break; // MOV X, dp+Y

        default:
            return -2; // Should be unreachable: all 256 opcodes are covered
    }

    return 0;
}

// ---------- State / disassembly ----------

void spc700_print_state(void *context) {
    if (!context) return;
    SPC700_CPU *cpu = (SPC700_CPU*)context;

    uint16_t ya = (uint16_t)(((uint16_t)cpu->y << 8) | cpu->a);
    printf("Sony SPC700 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%02X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Registers: A=0x%02X  X=0x%02X  Y=0x%02X  YA=0x%04X\n", cpu->a, cpu->x, cpu->y, ya);
    printf("  PSW: 0x%02X  N=%d  V=%d  P=%d  B=%d  H=%d  I=%d  Z=%d  C=%d\n",
           cpu->psw,
           GET_FLAG(FLAG_N), GET_FLAG(FLAG_V), GET_FLAG(FLAG_P), GET_FLAG(FLAG_B),
           GET_FLAG(FLAG_H), GET_FLAG(FLAG_I), GET_FLAG(FLAG_Z), GET_FLAG(FLAG_C));
}

// Operand shapes for the disassembly table
enum {
    SH_NONE,   // no numeric operand
    SH_B,      // one byte operand: pre $0xNN post
    SH_W,      // one word operand: pre $0xNNNN post
    SH_REL,    // relative branch (2-byte instruction): pre $0xNNNN
    SH_DPREL,  // dp + rel (3-byte instruction): pre $0xNN post $0xNNNN
    SH_MBIT,   // 13-bit address + bit: pre $0xNNNN.b post
    SH_DPDP,   // dp(dst), dp(src): pre $0xNN, $0xNN
    SH_DPIMM,  // dp, #imm: pre $0xNN, #$0xNN
    SH_UP      // upper-page call: pre $0xFFNN
};

typedef struct DisEntry {
    uint8_t shape;
    const char *pre;
    const char *post;
} DisEntry;

static const DisEntry g_dis[256] = {
    /* 00 */ {SH_NONE,  "nop", ""},
    /* 01 */ {SH_NONE,  "tcall 0", ""},
    /* 02 */ {SH_B,     "set1  ", ".0"},
    /* 03 */ {SH_DPREL, "bbs   ", ".0,"},
    /* 04 */ {SH_B,     "or    a, ", ""},
    /* 05 */ {SH_W,     "or    a, ", ""},
    /* 06 */ {SH_NONE,  "or    a, (x)", ""},
    /* 07 */ {SH_B,     "or    a, [", "+x]"},
    /* 08 */ {SH_B,     "or    a, #", ""},
    /* 09 */ {SH_DPDP,  "or    ", ""},
    /* 0A */ {SH_MBIT,  "or1   c, ", ""},
    /* 0B */ {SH_B,     "asl   ", ""},
    /* 0C */ {SH_W,     "asl   ", ""},
    /* 0D */ {SH_NONE,  "push  psw", ""},
    /* 0E */ {SH_W,     "tset1 ", ""},
    /* 0F */ {SH_NONE,  "brk", ""},
    /* 10 */ {SH_REL,   "bpl   ", ""},
    /* 11 */ {SH_NONE,  "tcall 1", ""},
    /* 12 */ {SH_B,     "clr1  ", ".0"},
    /* 13 */ {SH_DPREL, "bbc   ", ".0,"},
    /* 14 */ {SH_B,     "or    a, ", "+x"},
    /* 15 */ {SH_W,     "or    a, ", "+x"},
    /* 16 */ {SH_W,     "or    a, ", "+y"},
    /* 17 */ {SH_B,     "or    a, [", "]+y"},
    /* 18 */ {SH_DPIMM, "or    ", ""},
    /* 19 */ {SH_NONE,  "or    (x), (y)", ""},
    /* 1A */ {SH_B,     "decw  ", ""},
    /* 1B */ {SH_B,     "asl   ", "+x"},
    /* 1C */ {SH_NONE,  "asl   a", ""},
    /* 1D */ {SH_NONE,  "dec   x", ""},
    /* 1E */ {SH_W,     "cmp   x, ", ""},
    /* 1F */ {SH_W,     "jmp   [", "+x]"},
    /* 20 */ {SH_NONE,  "clrp", ""},
    /* 21 */ {SH_NONE,  "tcall 2", ""},
    /* 22 */ {SH_B,     "set1  ", ".1"},
    /* 23 */ {SH_DPREL, "bbs   ", ".1,"},
    /* 24 */ {SH_B,     "and   a, ", ""},
    /* 25 */ {SH_W,     "and   a, ", ""},
    /* 26 */ {SH_NONE,  "and   a, (x)", ""},
    /* 27 */ {SH_B,     "and   a, [", "+x]"},
    /* 28 */ {SH_B,     "and   a, #", ""},
    /* 29 */ {SH_DPDP,  "and   ", ""},
    /* 2A */ {SH_MBIT,  "or1   c, /", ""},
    /* 2B */ {SH_B,     "rol   ", ""},
    /* 2C */ {SH_W,     "rol   ", ""},
    /* 2D */ {SH_NONE,  "push  a", ""},
    /* 2E */ {SH_DPREL, "cbne  ", ","},
    /* 2F */ {SH_REL,   "bra   ", ""},
    /* 30 */ {SH_REL,   "bmi   ", ""},
    /* 31 */ {SH_NONE,  "tcall 3", ""},
    /* 32 */ {SH_B,     "clr1  ", ".1"},
    /* 33 */ {SH_DPREL, "bbc   ", ".1,"},
    /* 34 */ {SH_B,     "and   a, ", "+x"},
    /* 35 */ {SH_W,     "and   a, ", "+x"},
    /* 36 */ {SH_W,     "and   a, ", "+y"},
    /* 37 */ {SH_B,     "and   a, [", "]+y"},
    /* 38 */ {SH_DPIMM, "and   ", ""},
    /* 39 */ {SH_NONE,  "and   (x), (y)", ""},
    /* 3A */ {SH_B,     "incw  ", ""},
    /* 3B */ {SH_B,     "rol   ", "+x"},
    /* 3C */ {SH_NONE,  "rol   a", ""},
    /* 3D */ {SH_NONE,  "inc   x", ""},
    /* 3E */ {SH_B,     "cmp   x, ", ""},
    /* 3F */ {SH_W,     "call  ", ""},
    /* 40 */ {SH_NONE,  "setp", ""},
    /* 41 */ {SH_NONE,  "tcall 4", ""},
    /* 42 */ {SH_B,     "set1  ", ".2"},
    /* 43 */ {SH_DPREL, "bbs   ", ".2,"},
    /* 44 */ {SH_B,     "eor   a, ", ""},
    /* 45 */ {SH_W,     "eor   a, ", ""},
    /* 46 */ {SH_NONE,  "eor   a, (x)", ""},
    /* 47 */ {SH_B,     "eor   a, [", "+x]"},
    /* 48 */ {SH_B,     "eor   a, #", ""},
    /* 49 */ {SH_DPDP,  "eor   ", ""},
    /* 4A */ {SH_MBIT,  "and1  c, ", ""},
    /* 4B */ {SH_B,     "lsr   ", ""},
    /* 4C */ {SH_W,     "lsr   ", ""},
    /* 4D */ {SH_NONE,  "push  x", ""},
    /* 4E */ {SH_W,     "tclr1 ", ""},
    /* 4F */ {SH_UP,    "pcall ", ""},
    /* 50 */ {SH_REL,   "bvc   ", ""},
    /* 51 */ {SH_NONE,  "tcall 5", ""},
    /* 52 */ {SH_B,     "clr1  ", ".2"},
    /* 53 */ {SH_DPREL, "bbc   ", ".2,"},
    /* 54 */ {SH_B,     "eor   a, ", "+x"},
    /* 55 */ {SH_W,     "eor   a, ", "+x"},
    /* 56 */ {SH_W,     "eor   a, ", "+y"},
    /* 57 */ {SH_B,     "eor   a, [", "]+y"},
    /* 58 */ {SH_DPIMM, "eor   ", ""},
    /* 59 */ {SH_NONE,  "eor   (x), (y)", ""},
    /* 5A */ {SH_B,     "cmpw  ya, ", ""},
    /* 5B */ {SH_B,     "lsr   ", "+x"},
    /* 5C */ {SH_NONE,  "lsr   a", ""},
    /* 5D */ {SH_NONE,  "mov   x, a", ""},
    /* 5E */ {SH_W,     "cmp   y, ", ""},
    /* 5F */ {SH_W,     "jmp   ", ""},
    /* 60 */ {SH_NONE,  "clrc", ""},
    /* 61 */ {SH_NONE,  "tcall 6", ""},
    /* 62 */ {SH_B,     "set1  ", ".3"},
    /* 63 */ {SH_DPREL, "bbs   ", ".3,"},
    /* 64 */ {SH_B,     "cmp   a, ", ""},
    /* 65 */ {SH_W,     "cmp   a, ", ""},
    /* 66 */ {SH_NONE,  "cmp   a, (x)", ""},
    /* 67 */ {SH_B,     "cmp   a, [", "+x]"},
    /* 68 */ {SH_B,     "cmp   a, #", ""},
    /* 69 */ {SH_DPDP,  "cmp   ", ""},
    /* 6A */ {SH_MBIT,  "and1  c, /", ""},
    /* 6B */ {SH_B,     "ror   ", ""},
    /* 6C */ {SH_W,     "ror   ", ""},
    /* 6D */ {SH_NONE,  "push  y", ""},
    /* 6E */ {SH_DPREL, "dbnz  ", ","},
    /* 6F */ {SH_NONE,  "ret", ""},
    /* 70 */ {SH_REL,   "bvs   ", ""},
    /* 71 */ {SH_NONE,  "tcall 7", ""},
    /* 72 */ {SH_B,     "clr1  ", ".3"},
    /* 73 */ {SH_DPREL, "bbc   ", ".3,"},
    /* 74 */ {SH_B,     "cmp   a, ", "+x"},
    /* 75 */ {SH_W,     "cmp   a, ", "+x"},
    /* 76 */ {SH_W,     "cmp   a, ", "+y"},
    /* 77 */ {SH_B,     "cmp   a, [", "]+y"},
    /* 78 */ {SH_DPIMM, "cmp   ", ""},
    /* 79 */ {SH_NONE,  "cmp   (x), (y)", ""},
    /* 7A */ {SH_B,     "addw  ya, ", ""},
    /* 7B */ {SH_B,     "ror   ", "+x"},
    /* 7C */ {SH_NONE,  "ror   a", ""},
    /* 7D */ {SH_NONE,  "mov   a, x", ""},
    /* 7E */ {SH_B,     "cmp   y, ", ""},
    /* 7F */ {SH_NONE,  "reti", ""},
    /* 80 */ {SH_NONE,  "setc", ""},
    /* 81 */ {SH_NONE,  "tcall 8", ""},
    /* 82 */ {SH_B,     "set1  ", ".4"},
    /* 83 */ {SH_DPREL, "bbs   ", ".4,"},
    /* 84 */ {SH_B,     "adc   a, ", ""},
    /* 85 */ {SH_W,     "adc   a, ", ""},
    /* 86 */ {SH_NONE,  "adc   a, (x)", ""},
    /* 87 */ {SH_B,     "adc   a, [", "+x]"},
    /* 88 */ {SH_B,     "adc   a, #", ""},
    /* 89 */ {SH_DPDP,  "adc   ", ""},
    /* 8A */ {SH_MBIT,  "eor1  c, ", ""},
    /* 8B */ {SH_B,     "dec   ", ""},
    /* 8C */ {SH_W,     "dec   ", ""},
    /* 8D */ {SH_B,     "mov   y, #", ""},
    /* 8E */ {SH_NONE,  "pop   psw", ""},
    /* 8F */ {SH_DPIMM, "mov   ", ""},
    /* 90 */ {SH_REL,   "bcc   ", ""},
    /* 91 */ {SH_NONE,  "tcall 9", ""},
    /* 92 */ {SH_B,     "clr1  ", ".4"},
    /* 93 */ {SH_DPREL, "bbc   ", ".4,"},
    /* 94 */ {SH_B,     "adc   a, ", "+x"},
    /* 95 */ {SH_W,     "adc   a, ", "+x"},
    /* 96 */ {SH_W,     "adc   a, ", "+y"},
    /* 97 */ {SH_B,     "adc   a, [", "]+y"},
    /* 98 */ {SH_DPIMM, "adc   ", ""},
    /* 99 */ {SH_NONE,  "adc   (x), (y)", ""},
    /* 9A */ {SH_B,     "subw  ya, ", ""},
    /* 9B */ {SH_B,     "dec   ", "+x"},
    /* 9C */ {SH_NONE,  "dec   a", ""},
    /* 9D */ {SH_NONE,  "mov   x, sp", ""},
    /* 9E */ {SH_NONE,  "div   ya, x", ""},
    /* 9F */ {SH_NONE,  "xcn   a", ""},
    /* A0 */ {SH_NONE,  "ei", ""},
    /* A1 */ {SH_NONE,  "tcall 10", ""},
    /* A2 */ {SH_B,     "set1  ", ".5"},
    /* A3 */ {SH_DPREL, "bbs   ", ".5,"},
    /* A4 */ {SH_B,     "sbc   a, ", ""},
    /* A5 */ {SH_W,     "sbc   a, ", ""},
    /* A6 */ {SH_NONE,  "sbc   a, (x)", ""},
    /* A7 */ {SH_B,     "sbc   a, [", "+x]"},
    /* A8 */ {SH_B,     "sbc   a, #", ""},
    /* A9 */ {SH_DPDP,  "sbc   ", ""},
    /* AA */ {SH_MBIT,  "mov1  c, ", ""},
    /* AB */ {SH_B,     "inc   ", ""},
    /* AC */ {SH_W,     "inc   ", ""},
    /* AD */ {SH_B,     "cmp   y, #", ""},
    /* AE */ {SH_NONE,  "pop   a", ""},
    /* AF */ {SH_NONE,  "mov   (x)+, a", ""},
    /* B0 */ {SH_REL,   "bcs   ", ""},
    /* B1 */ {SH_NONE,  "tcall 11", ""},
    /* B2 */ {SH_B,     "clr1  ", ".5"},
    /* B3 */ {SH_DPREL, "bbc   ", ".5,"},
    /* B4 */ {SH_B,     "sbc   a, ", "+x"},
    /* B5 */ {SH_W,     "sbc   a, ", "+x"},
    /* B6 */ {SH_W,     "sbc   a, ", "+y"},
    /* B7 */ {SH_B,     "sbc   a, [", "]+y"},
    /* B8 */ {SH_DPIMM, "sbc   ", ""},
    /* B9 */ {SH_NONE,  "sbc   (x), (y)", ""},
    /* BA */ {SH_B,     "movw  ya, ", ""},
    /* BB */ {SH_B,     "inc   ", "+x"},
    /* BC */ {SH_NONE,  "inc   a", ""},
    /* BD */ {SH_NONE,  "mov   sp, x", ""},
    /* BE */ {SH_NONE,  "das   a", ""},
    /* BF */ {SH_NONE,  "mov   a, (x)+", ""},
    /* C0 */ {SH_NONE,  "di", ""},
    /* C1 */ {SH_NONE,  "tcall 12", ""},
    /* C2 */ {SH_B,     "set1  ", ".6"},
    /* C3 */ {SH_DPREL, "bbs   ", ".6,"},
    /* C4 */ {SH_B,     "mov   ", ", a"},
    /* C5 */ {SH_W,     "mov   ", ", a"},
    /* C6 */ {SH_NONE,  "mov   (x), a", ""},
    /* C7 */ {SH_B,     "mov   [", "+x], a"},
    /* C8 */ {SH_B,     "cmp   x, #", ""},
    /* C9 */ {SH_W,     "mov   ", ", x"},
    /* CA */ {SH_MBIT,  "mov1  ", ", c"},
    /* CB */ {SH_B,     "mov   ", ", y"},
    /* CC */ {SH_W,     "mov   ", ", y"},
    /* CD */ {SH_B,     "mov   x, #", ""},
    /* CE */ {SH_NONE,  "pop   x", ""},
    /* CF */ {SH_NONE,  "mul   ya", ""},
    /* D0 */ {SH_REL,   "bne   ", ""},
    /* D1 */ {SH_NONE,  "tcall 13", ""},
    /* D2 */ {SH_B,     "clr1  ", ".6"},
    /* D3 */ {SH_DPREL, "bbc   ", ".6,"},
    /* D4 */ {SH_B,     "mov   ", "+x, a"},
    /* D5 */ {SH_W,     "mov   ", "+x, a"},
    /* D6 */ {SH_W,     "mov   ", "+y, a"},
    /* D7 */ {SH_B,     "mov   [", "]+y, a"},
    /* D8 */ {SH_B,     "mov   ", ", x"},
    /* D9 */ {SH_B,     "mov   ", "+y, x"},
    /* DA */ {SH_B,     "movw  ", ", ya"},
    /* DB */ {SH_B,     "mov   ", "+x, y"},
    /* DC */ {SH_NONE,  "dec   y", ""},
    /* DD */ {SH_NONE,  "mov   a, y", ""},
    /* DE */ {SH_DPREL, "cbne  ", "+x,"},
    /* DF */ {SH_NONE,  "daa   a", ""},
    /* E0 */ {SH_NONE,  "clrv", ""},
    /* E1 */ {SH_NONE,  "tcall 14", ""},
    /* E2 */ {SH_B,     "set1  ", ".7"},
    /* E3 */ {SH_DPREL, "bbs   ", ".7,"},
    /* E4 */ {SH_B,     "mov   a, ", ""},
    /* E5 */ {SH_W,     "mov   a, ", ""},
    /* E6 */ {SH_NONE,  "mov   a, (x)", ""},
    /* E7 */ {SH_B,     "mov   a, [", "+x]"},
    /* E8 */ {SH_B,     "mov   a, #", ""},
    /* E9 */ {SH_W,     "mov   x, ", ""},
    /* EA */ {SH_MBIT,  "not1  ", ""},
    /* EB */ {SH_B,     "mov   y, ", ""},
    /* EC */ {SH_W,     "mov   y, ", ""},
    /* ED */ {SH_NONE,  "notc", ""},
    /* EE */ {SH_NONE,  "pop   y", ""},
    /* EF */ {SH_NONE,  "sleep", ""},
    /* F0 */ {SH_REL,   "beq   ", ""},
    /* F1 */ {SH_NONE,  "tcall 15", ""},
    /* F2 */ {SH_B,     "clr1  ", ".7"},
    /* F3 */ {SH_DPREL, "bbc   ", ".7,"},
    /* F4 */ {SH_B,     "mov   a, ", "+x"},
    /* F5 */ {SH_W,     "mov   a, ", "+x"},
    /* F6 */ {SH_W,     "mov   a, ", "+y"},
    /* F7 */ {SH_B,     "mov   a, [", "]+y"},
    /* F8 */ {SH_B,     "mov   x, ", ""},
    /* F9 */ {SH_B,     "mov   x, ", "+y"},
    /* FA */ {SH_DPDP,  "mov   ", ""},
    /* FB */ {SH_B,     "mov   y, ", "+x"},
    /* FC */ {SH_NONE,  "inc   y", ""},
    /* FD */ {SH_NONE,  "mov   y, a", ""},
    /* FE */ {SH_REL,   "dbnz  y, ", ""},
    /* FF */ {SH_NONE,  "stop", ""}
};

void spc700_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    SPC700_CPU *cpu = (SPC700_CPU*)context;

    uint8_t op = rd(cpu, cpu->pc);
    uint8_t b1 = rd(cpu, (uint16_t)(cpu->pc + 1));
    uint8_t b2 = rd(cpu, (uint16_t)(cpu->pc + 2));
    uint16_t w = (uint16_t)(b1 | ((uint16_t)b2 << 8));
    const DisEntry *e = &g_dis[op];

    switch (e->shape) {
        case SH_B:
            snprintf(buf, buf_len, "%s$0x%02X%s", e->pre, b1, e->post);
            break;
        case SH_W:
            snprintf(buf, buf_len, "%s$0x%04X%s", e->pre, w, e->post);
            break;
        case SH_REL:
            snprintf(buf, buf_len, "%s$0x%04X%s", e->pre,
                     (uint16_t)(cpu->pc + 2 + (int8_t)b1), e->post);
            break;
        case SH_DPREL:
            snprintf(buf, buf_len, "%s$0x%02X%s $0x%04X", e->pre, b1, e->post,
                     (uint16_t)(cpu->pc + 3 + (int8_t)b2));
            break;
        case SH_MBIT:
            snprintf(buf, buf_len, "%s$0x%04X.%d%s", e->pre,
                     (uint16_t)(w & 0x1FFF), (int)(w >> 13), e->post);
            break;
        case SH_DPDP:
            snprintf(buf, buf_len, "%s$0x%02X, $0x%02X", e->pre, b2, b1);
            break;
        case SH_DPIMM:
            snprintf(buf, buf_len, "%s$0x%02X, #$0x%02X", e->pre, b2, b1);
            break;
        case SH_UP:
            snprintf(buf, buf_len, "%s$0x%04X", e->pre, (uint16_t)(0xFF00 | b1));
            break;
        default:
            snprintf(buf, buf_len, "%s", e->pre);
            break;
    }
}
