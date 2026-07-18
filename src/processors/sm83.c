#include "sm83.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB (16-bit address space)

// Flag bit positions in F (upper nibble only; lower nibble always reads 0)
#define FLAG_Z 0x80
#define FLAG_N 0x40
#define FLAG_H 0x20
#define FLAG_C 0x10

// Register indices as encoded in SM83 opcodes: B,C,D,E,H,L,(HL),A
enum { R_B = 0, R_C, R_D, R_E, R_H, R_L, R_HL, R_A };

typedef struct SM83CPU {
    uint8_t regs[8]; // index R_HL (6) is unused storage; memory access is virtual
    uint8_t f;       // flags register (Z N H C in upper nibble)
    uint16_t pc;
    uint16_t sp;

    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
    int ime; // interrupt master enable
} SM83CPU;

static const char* r_names[] = { "B", "C", "D", "E", "H", "L", "(HL)", "A" };
static const char* rp_names[] = { "BC", "DE", "HL", "SP" };
static const char* rp2_names[] = { "BC", "DE", "HL", "AF" };
static const char* c_names[] = { "NZ", "Z", "NC", "C" };
static const char* alu_names[] = { "ADD A,", "ADC A,", "SUB", "SBC A,", "AND", "XOR", "OR", "CP" };
static const char* rot_names[] = { "RLC", "RRC", "RL", "RR", "SLA", "SRA", "SWAP", "SRL" };

static void set_flags(SM83CPU *cpu, int z, int n, int h, int c) {
    cpu->f = (uint8_t)((z ? FLAG_Z : 0) | (n ? FLAG_N : 0) |
                       (h ? FLAG_H : 0) | (c ? FLAG_C : 0));
}

static uint16_t hl_addr(SM83CPU *cpu) {
    return ((uint16_t)cpu->regs[R_H] << 8) | cpu->regs[R_L];
}

static uint8_t get_reg(SM83CPU *cpu, uint8_t idx) {
    if (idx == R_HL) return cpu->memory[hl_addr(cpu)];
    return cpu->regs[idx];
}

static void set_reg(SM83CPU *cpu, uint8_t idx, uint8_t val) {
    if (idx == R_HL) cpu->memory[hl_addr(cpu)] = val;
    else cpu->regs[idx] = val;
}

// Register pair getter: 0=BC, 1=DE, 2=HL, 3=SP
static uint16_t get_rp(SM83CPU *cpu, uint8_t rp) {
    switch (rp) {
        case 0: return ((uint16_t)cpu->regs[R_B] << 8) | cpu->regs[R_C];
        case 1: return ((uint16_t)cpu->regs[R_D] << 8) | cpu->regs[R_E];
        case 2: return hl_addr(cpu);
        case 3: return cpu->sp;
    }
    return 0;
}

static void set_rp(SM83CPU *cpu, uint8_t rp, uint16_t val) {
    switch (rp) {
        case 0: cpu->regs[R_B] = (uint8_t)(val >> 8); cpu->regs[R_C] = val & 0xFF; break;
        case 1: cpu->regs[R_D] = (uint8_t)(val >> 8); cpu->regs[R_E] = val & 0xFF; break;
        case 2: cpu->regs[R_H] = (uint8_t)(val >> 8); cpu->regs[R_L] = val & 0xFF; break;
        case 3: cpu->sp = val; break;
    }
}

// Condition codes: 0=NZ, 1=Z, 2=NC, 3=C
static int check_cond(SM83CPU *cpu, uint8_t cond) {
    switch (cond) {
        case 0: return (cpu->f & FLAG_Z) == 0;
        case 1: return (cpu->f & FLAG_Z) != 0;
        case 2: return (cpu->f & FLAG_C) == 0;
        case 3: return (cpu->f & FLAG_C) != 0;
    }
    return 0;
}

static void push16(SM83CPU *cpu, uint16_t val) {
    cpu->sp = (cpu->sp - 1) & 0xFFFF;
    cpu->memory[cpu->sp] = (uint8_t)(val >> 8);
    cpu->sp = (cpu->sp - 1) & 0xFFFF;
    cpu->memory[cpu->sp] = val & 0xFF;
}

static uint16_t pop16(SM83CPU *cpu) {
    uint8_t lo = cpu->memory[cpu->sp];
    uint8_t hi;
    cpu->sp = (cpu->sp + 1) & 0xFFFF;
    hi = cpu->memory[cpu->sp];
    cpu->sp = (cpu->sp + 1) & 0xFFFF;
    return ((uint16_t)hi << 8) | lo;
}

// Core 8-bit ALU used by both register (10aaarrr) and immediate (11aaa110) forms.
static void alu_op(SM83CPU *cpu, uint8_t alu, uint8_t val) {
    uint8_t acc = cpu->regs[R_A];
    uint8_t carry = (cpu->f & FLAG_C) ? 1 : 0;
    switch (alu) {
        case 0: { // ADD
            uint16_t sum = (uint16_t)acc + val;
            uint8_t res = sum & 0xFF;
            set_flags(cpu, res == 0, 0, ((acc & 0x0F) + (val & 0x0F)) > 0x0F, sum > 0xFF);
            cpu->regs[R_A] = res;
            break;
        }
        case 1: { // ADC
            uint16_t sum = (uint16_t)acc + val + carry;
            uint8_t res = sum & 0xFF;
            set_flags(cpu, res == 0, 0, ((acc & 0x0F) + (val & 0x0F) + carry) > 0x0F, sum > 0xFF);
            cpu->regs[R_A] = res;
            break;
        }
        case 2: { // SUB
            uint8_t res = (uint8_t)(acc - val);
            set_flags(cpu, res == 0, 1, (acc & 0x0F) < (val & 0x0F), acc < val);
            cpu->regs[R_A] = res;
            break;
        }
        case 3: { // SBC
            uint16_t sub = (uint16_t)val + carry;
            uint8_t res = (uint8_t)(acc - sub);
            set_flags(cpu, res == 0, 1, (acc & 0x0F) < ((val & 0x0F) + carry), acc < sub);
            cpu->regs[R_A] = res;
            break;
        }
        case 4: { // AND
            uint8_t res = acc & val;
            set_flags(cpu, res == 0, 0, 1, 0);
            cpu->regs[R_A] = res;
            break;
        }
        case 5: { // XOR
            uint8_t res = acc ^ val;
            set_flags(cpu, res == 0, 0, 0, 0);
            cpu->regs[R_A] = res;
            break;
        }
        case 6: { // OR
            uint8_t res = acc | val;
            set_flags(cpu, res == 0, 0, 0, 0);
            cpu->regs[R_A] = res;
            break;
        }
        case 7: { // CP
            uint8_t res = (uint8_t)(acc - val);
            set_flags(cpu, res == 0, 1, (acc & 0x0F) < (val & 0x0F), acc < val);
            break;
        }
    }
}

// CB-prefixed rotate/shift/swap group (00rrrggg with ggg selecting the op).
static void cb_rot_op(SM83CPU *cpu, uint8_t rot, uint8_t reg) {
    uint8_t val = get_reg(cpu, reg);
    uint8_t carry = (cpu->f & FLAG_C) ? 1 : 0;
    uint8_t res = 0;
    uint8_t new_c = 0;
    switch (rot) {
        case 0: // RLC
            new_c = (val >> 7) & 1;
            res = (uint8_t)((val << 1) | new_c);
            break;
        case 1: // RRC
            new_c = val & 1;
            res = (uint8_t)((val >> 1) | (new_c << 7));
            break;
        case 2: // RL
            new_c = (val >> 7) & 1;
            res = (uint8_t)((val << 1) | carry);
            break;
        case 3: // RR
            new_c = val & 1;
            res = (uint8_t)((val >> 1) | (carry << 7));
            break;
        case 4: // SLA
            new_c = (val >> 7) & 1;
            res = (uint8_t)(val << 1);
            break;
        case 5: // SRA
            new_c = val & 1;
            res = (uint8_t)((val >> 1) | (val & 0x80));
            break;
        case 6: // SWAP
            new_c = 0;
            res = (uint8_t)((val << 4) | (val >> 4));
            break;
        case 7: // SRL
            new_c = val & 1;
            res = val >> 1;
            break;
    }
    set_reg(cpu, reg, res);
    set_flags(cpu, res == 0, 0, 0, new_c);
}

static void cb_step(SM83CPU *cpu, uint8_t op) {
    uint8_t reg = op & 0x07;
    uint8_t bit = (op >> 3) & 0x07;
    if (op < 0x40) {
        cb_rot_op(cpu, bit, reg);
    } else if (op < 0x80) { // BIT b, r
        uint8_t z = (get_reg(cpu, reg) & (1 << bit)) == 0;
        cpu->f = (uint8_t)((cpu->f & FLAG_C) | FLAG_H | (z ? FLAG_Z : 0));
    } else if (op < 0xC0) { // RES b, r
        set_reg(cpu, reg, get_reg(cpu, reg) & (uint8_t)~(1 << bit));
    } else { // SET b, r
        set_reg(cpu, reg, get_reg(cpu, reg) | (uint8_t)(1 << bit));
    }
}

void* sm83_create(void) {
    return calloc(1, sizeof(SM83CPU));
}

void sm83_destroy(void *context) {
    free(context);
}

int sm83_init(void *context) {
    SM83CPU *cpu = (SM83CPU*)context;
    if (!cpu) return -1;
    memset(cpu->regs, 0, sizeof(cpu->regs));
    cpu->f = 0;
    cpu->pc = 0;
    cpu->sp = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->ime = 0;
    return 0;
}

int sm83_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    SM83CPU *cpu = (SM83CPU*)context;
    size_t copy_len = size;
    if (!cpu) return -1;
    if (address >= MEM_SIZE) return -2;
    if (address + size > MEM_SIZE) copy_len = MEM_SIZE - address;
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Instruction length lookup keyed by opcode: 1, 2, or 3 bytes.
static int instr_len(uint8_t op) {
    switch (op) {
        // 3-byte: LD rr,d16; LD (a16),SP; JP/CALL (incl. conditional); LD (a16),A / A,(a16)
        case 0x01: case 0x11: case 0x21: case 0x31:
        case 0x08:
        case 0xC2: case 0xC3: case 0xC4: case 0xCA: case 0xCC: case 0xCD:
        case 0xD2: case 0xD4: case 0xDA: case 0xDC:
        case 0xEA: case 0xFA:
            return 3;
        // 2-byte: LD r,d8; ALU d8; JR (incl. conditional); STOP; LDH; ADD SP,r8;
        //         LD HL,SP+r8; CB prefix
        case 0x06: case 0x0E: case 0x16: case 0x1E:
        case 0x26: case 0x2E: case 0x36: case 0x3E:
        case 0xC6: case 0xCE: case 0xD6: case 0xDE:
        case 0xE6: case 0xEE: case 0xF6: case 0xFE:
        case 0x10: case 0x18: case 0x20: case 0x28: case 0x30: case 0x38:
        case 0xE0: case 0xF0: case 0xE8: case 0xF8:
        case 0xCB:
            return 2;
        default:
            return 1;
    }
}

int sm83_step(void *context) {
    SM83CPU *cpu = (SM83CPU*)context;
    uint16_t instr_pc, next_pc, imm16;
    uint8_t op, byte2;
    int8_t rel;
    int len;

    if (!cpu) return -1;
    if (cpu->halted) return 1;

    instr_pc = cpu->pc;
    op = cpu->memory[cpu->pc];
    byte2 = cpu->memory[(cpu->pc + 1) & 0xFFFF];
    imm16 = (uint16_t)(((uint16_t)cpu->memory[(cpu->pc + 2) & 0xFFFF] << 8) | byte2);
    rel = (int8_t)byte2;

    len = instr_len(op);
    next_pc = (uint16_t)(cpu->pc + len);
    cpu->pc = next_pc;
    cpu->ticks++;

    // --- LD r,r' / HALT (0x40..0x7F) ---
    if (op >= 0x40 && op <= 0x7F) {
        if (op == 0x76) { // HALT
            cpu->halted = 1;
            return 1;
        }
        set_reg(cpu, (op >> 3) & 0x07, get_reg(cpu, op & 0x07));
        return 0;
    }

    // --- ALU register ops (0x80..0xBF) ---
    if (op >= 0x80 && op <= 0xBF) {
        alu_op(cpu, (op >> 3) & 0x07, get_reg(cpu, op & 0x07));
        return 0;
    }

    switch (op) {
        case 0x00: // NOP
            break;
        case 0x10: // STOP
            cpu->halted = 1;
            return 1;

        // --- LD rr, d16 ---
        case 0x01: case 0x11: case 0x21: case 0x31:
            set_rp(cpu, (op >> 4) & 0x03, imm16);
            break;

        // --- LD (a16), SP ---
        case 0x08:
            cpu->memory[imm16] = cpu->sp & 0xFF;
            cpu->memory[(imm16 + 1) & 0xFFFF] = (uint8_t)(cpu->sp >> 8);
            break;

        // --- Indirect stores/loads of A, with HL post-inc/dec ---
        case 0x02: cpu->memory[get_rp(cpu, 0)] = cpu->regs[R_A]; break; // LD (BC),A
        case 0x12: cpu->memory[get_rp(cpu, 1)] = cpu->regs[R_A]; break; // LD (DE),A
        case 0x22: // LD (HL+),A
            cpu->memory[hl_addr(cpu)] = cpu->regs[R_A];
            set_rp(cpu, 2, (uint16_t)(hl_addr(cpu) + 1));
            break;
        case 0x32: // LD (HL-),A
            cpu->memory[hl_addr(cpu)] = cpu->regs[R_A];
            set_rp(cpu, 2, (uint16_t)(hl_addr(cpu) - 1));
            break;
        case 0x0A: cpu->regs[R_A] = cpu->memory[get_rp(cpu, 0)]; break; // LD A,(BC)
        case 0x1A: cpu->regs[R_A] = cpu->memory[get_rp(cpu, 1)]; break; // LD A,(DE)
        case 0x2A: // LD A,(HL+)
            cpu->regs[R_A] = cpu->memory[hl_addr(cpu)];
            set_rp(cpu, 2, (uint16_t)(hl_addr(cpu) + 1));
            break;
        case 0x3A: // LD A,(HL-)
            cpu->regs[R_A] = cpu->memory[hl_addr(cpu)];
            set_rp(cpu, 2, (uint16_t)(hl_addr(cpu) - 1));
            break;

        // --- INC / DEC rr (no flags) ---
        case 0x03: case 0x13: case 0x23: case 0x33:
            set_rp(cpu, (op >> 4) & 3, (uint16_t)(get_rp(cpu, (op >> 4) & 3) + 1));
            break;
        case 0x0B: case 0x1B: case 0x2B: case 0x3B:
            set_rp(cpu, (op >> 4) & 3, (uint16_t)(get_rp(cpu, (op >> 4) & 3) - 1));
            break;

        // --- ADD HL, rr (Z unchanged; N=0; H from bit 11; C from bit 15) ---
        case 0x09: case 0x19: case 0x29: case 0x39: {
            uint16_t hl = hl_addr(cpu);
            uint16_t val = get_rp(cpu, (op >> 4) & 3);
            uint32_t sum = (uint32_t)hl + val;
            cpu->f = (uint8_t)((cpu->f & FLAG_Z) |
                               (((hl & 0x0FFF) + (val & 0x0FFF)) > 0x0FFF ? FLAG_H : 0) |
                               (sum > 0xFFFF ? FLAG_C : 0));
            set_rp(cpu, 2, sum & 0xFFFF);
            break;
        }

        // --- INC / DEC r (C unchanged) ---
        case 0x04: case 0x0C: case 0x14: case 0x1C:
        case 0x24: case 0x2C: case 0x34: case 0x3C: {
            uint8_t reg = (op >> 3) & 7;
            uint8_t res = (uint8_t)(get_reg(cpu, reg) + 1);
            set_reg(cpu, reg, res);
            cpu->f = (uint8_t)((cpu->f & FLAG_C) | (res == 0 ? FLAG_Z : 0) |
                               ((res & 0x0F) == 0x00 ? FLAG_H : 0));
            break;
        }
        case 0x05: case 0x0D: case 0x15: case 0x1D:
        case 0x25: case 0x2D: case 0x35: case 0x3D: {
            uint8_t reg = (op >> 3) & 7;
            uint8_t res = (uint8_t)(get_reg(cpu, reg) - 1);
            set_reg(cpu, reg, res);
            cpu->f = (uint8_t)((cpu->f & FLAG_C) | FLAG_N | (res == 0 ? FLAG_Z : 0) |
                               ((res & 0x0F) == 0x0F ? FLAG_H : 0));
            break;
        }

        // --- LD r, d8 ---
        case 0x06: case 0x0E: case 0x16: case 0x1E:
        case 0x26: case 0x2E: case 0x36: case 0x3E:
            set_reg(cpu, (op >> 3) & 7, byte2);
            break;

        // --- Accumulator rotates (Z always cleared on SM83) ---
        case 0x07: { // RLCA
            uint8_t msb = (cpu->regs[R_A] >> 7) & 1;
            cpu->regs[R_A] = (uint8_t)((cpu->regs[R_A] << 1) | msb);
            set_flags(cpu, 0, 0, 0, msb);
            break;
        }
        case 0x0F: { // RRCA
            uint8_t lsb = cpu->regs[R_A] & 1;
            cpu->regs[R_A] = (uint8_t)((cpu->regs[R_A] >> 1) | (lsb << 7));
            set_flags(cpu, 0, 0, 0, lsb);
            break;
        }
        case 0x17: { // RLA
            uint8_t msb = (cpu->regs[R_A] >> 7) & 1;
            cpu->regs[R_A] = (uint8_t)((cpu->regs[R_A] << 1) | ((cpu->f & FLAG_C) ? 1 : 0));
            set_flags(cpu, 0, 0, 0, msb);
            break;
        }
        case 0x1F: { // RRA
            uint8_t lsb = cpu->regs[R_A] & 1;
            cpu->regs[R_A] = (uint8_t)((cpu->regs[R_A] >> 1) | ((cpu->f & FLAG_C) ? 0x80 : 0));
            set_flags(cpu, 0, 0, 0, lsb);
            break;
        }

        // --- Relative jumps ---
        case 0x18: cpu->pc = (uint16_t)(next_pc + rel); break; // JR r8
        case 0x20: case 0x28: case 0x30: case 0x38:            // JR cc, r8
            if (check_cond(cpu, (op >> 3) & 3)) cpu->pc = (uint16_t)(next_pc + rel);
            break;

        // --- DAA (SM83 semantics: depends on N flag) ---
        case 0x27: {
            uint8_t a = cpu->regs[R_A];
            uint8_t carry = (cpu->f & FLAG_C) ? 1 : 0;
            if (cpu->f & FLAG_N) {
                if (carry) a = (uint8_t)(a - 0x60);
                if (cpu->f & FLAG_H) a = (uint8_t)(a - 0x06);
            } else {
                if (carry || cpu->regs[R_A] > 0x99) { a = (uint8_t)(a + 0x60); carry = 1; }
                if ((cpu->f & FLAG_H) || (cpu->regs[R_A] & 0x0F) > 0x09) a = (uint8_t)(a + 0x06);
            }
            cpu->regs[R_A] = a;
            cpu->f = (uint8_t)((cpu->f & FLAG_N) | (a == 0 ? FLAG_Z : 0) | (carry ? FLAG_C : 0));
            break;
        }

        // --- CPL / SCF / CCF ---
        case 0x2F: // CPL (sets N and H)
            cpu->regs[R_A] = (uint8_t)~cpu->regs[R_A];
            cpu->f |= FLAG_N | FLAG_H;
            break;
        case 0x37: // SCF
            cpu->f = (uint8_t)((cpu->f & FLAG_Z) | FLAG_C);
            break;
        case 0x3F: // CCF
            cpu->f = (uint8_t)((cpu->f & FLAG_Z) | ((cpu->f & FLAG_C) ? 0 : FLAG_C));
            break;

        // --- ALU immediate (11aaa110) ---
        case 0xC6: case 0xCE: case 0xD6: case 0xDE:
        case 0xE6: case 0xEE: case 0xF6: case 0xFE:
            alu_op(cpu, (op >> 3) & 7, byte2);
            break;

        // --- Absolute jumps ---
        case 0xC3: cpu->pc = imm16; break; // JP a16
        case 0xC2: case 0xCA: case 0xD2: case 0xDA:
            if (check_cond(cpu, (op >> 3) & 3)) cpu->pc = imm16;
            break;
        case 0xE9: cpu->pc = hl_addr(cpu); break; // JP HL

        // --- Calls ---
        case 0xCD:
            push16(cpu, next_pc);
            cpu->pc = imm16;
            break;
        case 0xC4: case 0xCC: case 0xD4: case 0xDC:
            if (check_cond(cpu, (op >> 3) & 3)) {
                push16(cpu, next_pc);
                cpu->pc = imm16;
            }
            break;

        // --- Returns ---
        case 0xC9: cpu->pc = pop16(cpu); break; // RET
        case 0xD9: cpu->pc = pop16(cpu); cpu->ime = 1; break; // RETI
        case 0xC0: case 0xC8: case 0xD0: case 0xD8:
            if (check_cond(cpu, (op >> 3) & 3)) cpu->pc = pop16(cpu);
            break;

        // --- RST n ---
        case 0xC7: case 0xCF: case 0xD7: case 0xDF:
        case 0xE7: case 0xEF: case 0xF7: case 0xFF:
            push16(cpu, next_pc);
            cpu->pc = (uint16_t)(((op >> 3) & 7) * 8);
            break;

        // --- Stack ops: PUSH / POP rr ---
        case 0xC5: push16(cpu, get_rp(cpu, 0)); break; // PUSH BC
        case 0xD5: push16(cpu, get_rp(cpu, 1)); break; // PUSH DE
        case 0xE5: push16(cpu, get_rp(cpu, 2)); break; // PUSH HL
        case 0xF5: push16(cpu, (uint16_t)(((uint16_t)cpu->regs[R_A] << 8) | cpu->f)); break; // PUSH AF
        case 0xC1: set_rp(cpu, 0, pop16(cpu)); break; // POP BC
        case 0xD1: set_rp(cpu, 1, pop16(cpu)); break; // POP DE
        case 0xE1: set_rp(cpu, 2, pop16(cpu)); break; // POP HL
        case 0xF1: { // POP AF (low nibble of F always reads 0)
            uint16_t v = pop16(cpu);
            cpu->f = v & 0xF0;
            cpu->regs[R_A] = (uint8_t)(v >> 8);
            break;
        }

        // --- High-page (0xFF00) loads/stores ---
        case 0xE0: cpu->memory[0xFF00 + byte2] = cpu->regs[R_A]; break;        // LDH (a8),A
        case 0xF0: cpu->regs[R_A] = cpu->memory[0xFF00 + byte2]; break;        // LDH A,(a8)
        case 0xE2: cpu->memory[0xFF00 + cpu->regs[R_C]] = cpu->regs[R_A]; break; // LD (C),A
        case 0xF2: cpu->regs[R_A] = cpu->memory[0xFF00 + cpu->regs[R_C]]; break; // LD A,(C)

        // --- Absolute addressing of A ---
        case 0xEA: cpu->memory[imm16] = cpu->regs[R_A]; break; // LD (a16),A
        case 0xFA: cpu->regs[R_A] = cpu->memory[imm16]; break; // LD A,(a16)

        // --- SP arithmetic (H/C from low byte of unsigned add) ---
        case 0xE8: { // ADD SP, r8
            uint16_t sp = cpu->sp;
            set_flags(cpu, 0, 0,
                      ((sp & 0x0F) + (byte2 & 0x0F)) > 0x0F,
                      ((sp & 0xFF) + byte2) > 0xFF);
            cpu->sp = (uint16_t)(sp + rel);
            break;
        }
        case 0xF8: { // LD HL, SP+r8
            uint16_t sp = cpu->sp;
            set_flags(cpu, 0, 0,
                      ((sp & 0x0F) + (byte2 & 0x0F)) > 0x0F,
                      ((sp & 0xFF) + byte2) > 0xFF);
            set_rp(cpu, 2, (uint16_t)(sp + rel));
            break;
        }
        case 0xF9: cpu->sp = hl_addr(cpu); break; // LD SP,HL

        // --- Interrupt control ---
        case 0xF3: cpu->ime = 0; break; // DI
        case 0xFB: cpu->ime = 1; break; // EI

        // --- CB prefix ---
        case 0xCB:
            cb_step(cpu, byte2);
            break;

        // --- Invalid opcodes: lock the CPU ---
        case 0xD3: case 0xDB: case 0xDD: case 0xE3: case 0xE4:
        case 0xEB: case 0xEC: case 0xED: case 0xF4: case 0xFC: case 0xFD:
            cpu->halted = 1;
            return 1;

        default:
            // Should be unreachable; treat as NOP.
            break;
    }

    // Self-loop (e.g. JR $) interpreted as a software halt, matching other cores.
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

void sm83_print_state(void *context) {
    SM83CPU *cpu = (SM83CPU*)context;
    if (!cpu) return;

    printf("Sharp SM83 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  Halted: %s  IME: %d\n",
           cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No", cpu->ime);
    printf("  Flags: Z=%d N=%d H=%d C=%d  (F: 0x%02X)\n",
           (cpu->f & FLAG_Z) ? 1 : 0, (cpu->f & FLAG_N) ? 1 : 0,
           (cpu->f & FLAG_H) ? 1 : 0, (cpu->f & FLAG_C) ? 1 : 0, cpu->f);
    printf("  Registers:\n");
    printf("    A: 0x%02X  F: 0x%02X   (AF: 0x%04X)\n",
           cpu->regs[R_A], cpu->f, ((uint16_t)cpu->regs[R_A] << 8) | cpu->f);
    printf("    B: 0x%02X  C: 0x%02X   (BC: 0x%04X)\n", cpu->regs[R_B], cpu->regs[R_C], get_rp(cpu, 0));
    printf("    D: 0x%02X  E: 0x%02X   (DE: 0x%04X)\n", cpu->regs[R_D], cpu->regs[R_E], get_rp(cpu, 1));
    printf("    H: 0x%02X  L: 0x%02X   (HL: 0x%04X)\n", cpu->regs[R_H], cpu->regs[R_L], get_rp(cpu, 2));
    printf("  (HL) (at 0x%04X): 0x%02X\n", hl_addr(cpu), cpu->memory[hl_addr(cpu)]);
}

void sm83_get_disassembly(void *context, char *buf, size_t buf_len) {
    SM83CPU *cpu = (SM83CPU*)context;
    uint8_t op, byte2;
    uint16_t imm16;
    int8_t rel;

    if (!cpu || !buf || buf_len == 0) return;

    op = cpu->memory[cpu->pc];
    byte2 = cpu->memory[(cpu->pc + 1) & 0xFFFF];
    imm16 = (uint16_t)(((uint16_t)cpu->memory[(cpu->pc + 2) & 0xFFFF] << 8) | byte2);
    rel = (int8_t)byte2;

    if (op == 0xCB) {
        uint8_t reg = byte2 & 0x07;
        uint8_t bit = (byte2 >> 3) & 0x07;
        if (byte2 < 0x40)      snprintf(buf, buf_len, "%-5s %s", rot_names[bit], r_names[reg]);
        else if (byte2 < 0x80) snprintf(buf, buf_len, "BIT   %d, %s", bit, r_names[reg]);
        else if (byte2 < 0xC0) snprintf(buf, buf_len, "RES   %d, %s", bit, r_names[reg]);
        else                   snprintf(buf, buf_len, "SET   %d, %s", bit, r_names[reg]);
        return;
    }
    if (op == 0x76) { snprintf(buf, buf_len, "HALT"); return; }
    if (op >= 0x40 && op <= 0x7F) {
        snprintf(buf, buf_len, "LD    %s, %s", r_names[(op >> 3) & 7], r_names[op & 7]);
        return;
    }
    if (op >= 0x80 && op <= 0xBF) {
        snprintf(buf, buf_len, "%-6s%s", alu_names[(op >> 3) & 7], r_names[op & 7]);
        return;
    }

    switch (op) {
        case 0x00: snprintf(buf, buf_len, "NOP"); break;
        case 0x10: snprintf(buf, buf_len, "STOP"); break;
        case 0x01: case 0x11: case 0x21: case 0x31:
            snprintf(buf, buf_len, "LD    %s, 0x%04X", rp_names[(op >> 4) & 3], imm16); break;
        case 0x08: snprintf(buf, buf_len, "LD    (0x%04X), SP", imm16); break;
        case 0x02: snprintf(buf, buf_len, "LD    (BC), A"); break;
        case 0x12: snprintf(buf, buf_len, "LD    (DE), A"); break;
        case 0x22: snprintf(buf, buf_len, "LD    (HL+), A"); break;
        case 0x32: snprintf(buf, buf_len, "LD    (HL-), A"); break;
        case 0x0A: snprintf(buf, buf_len, "LD    A, (BC)"); break;
        case 0x1A: snprintf(buf, buf_len, "LD    A, (DE)"); break;
        case 0x2A: snprintf(buf, buf_len, "LD    A, (HL+)"); break;
        case 0x3A: snprintf(buf, buf_len, "LD    A, (HL-)"); break;
        case 0x03: case 0x13: case 0x23: case 0x33:
            snprintf(buf, buf_len, "INC   %s", rp_names[(op >> 4) & 3]); break;
        case 0x0B: case 0x1B: case 0x2B: case 0x3B:
            snprintf(buf, buf_len, "DEC   %s", rp_names[(op >> 4) & 3]); break;
        case 0x09: case 0x19: case 0x29: case 0x39:
            snprintf(buf, buf_len, "ADD   HL, %s", rp_names[(op >> 4) & 3]); break;
        case 0x04: case 0x0C: case 0x14: case 0x1C:
        case 0x24: case 0x2C: case 0x34: case 0x3C:
            snprintf(buf, buf_len, "INC   %s", r_names[(op >> 3) & 7]); break;
        case 0x05: case 0x0D: case 0x15: case 0x1D:
        case 0x25: case 0x2D: case 0x35: case 0x3D:
            snprintf(buf, buf_len, "DEC   %s", r_names[(op >> 3) & 7]); break;
        case 0x06: case 0x0E: case 0x16: case 0x1E:
        case 0x26: case 0x2E: case 0x36: case 0x3E:
            snprintf(buf, buf_len, "LD    %s, 0x%02X", r_names[(op >> 3) & 7], byte2); break;
        case 0x07: snprintf(buf, buf_len, "RLCA"); break;
        case 0x0F: snprintf(buf, buf_len, "RRCA"); break;
        case 0x17: snprintf(buf, buf_len, "RLA"); break;
        case 0x1F: snprintf(buf, buf_len, "RRA"); break;
        case 0x18: snprintf(buf, buf_len, "JR    0x%04X", (uint16_t)(cpu->pc + 2 + rel)); break;
        case 0x20: case 0x28: case 0x30: case 0x38:
            snprintf(buf, buf_len, "JR    %s, 0x%04X", c_names[(op >> 3) & 3],
                     (uint16_t)(cpu->pc + 2 + rel));
            break;
        case 0x27: snprintf(buf, buf_len, "DAA"); break;
        case 0x2F: snprintf(buf, buf_len, "CPL"); break;
        case 0x37: snprintf(buf, buf_len, "SCF"); break;
        case 0x3F: snprintf(buf, buf_len, "CCF"); break;
        case 0xC6: case 0xCE: case 0xD6: case 0xDE:
        case 0xE6: case 0xEE: case 0xF6: case 0xFE:
            snprintf(buf, buf_len, "%-6s0x%02X", alu_names[(op >> 3) & 7], byte2); break;
        case 0xC3: snprintf(buf, buf_len, "JP    0x%04X", imm16); break;
        case 0xC2: case 0xCA: case 0xD2: case 0xDA:
            snprintf(buf, buf_len, "JP    %s, 0x%04X", c_names[(op >> 3) & 3], imm16); break;
        case 0xE9: snprintf(buf, buf_len, "JP    HL"); break;
        case 0xCD: snprintf(buf, buf_len, "CALL  0x%04X", imm16); break;
        case 0xC4: case 0xCC: case 0xD4: case 0xDC:
            snprintf(buf, buf_len, "CALL  %s, 0x%04X", c_names[(op >> 3) & 3], imm16); break;
        case 0xC9: snprintf(buf, buf_len, "RET"); break;
        case 0xD9: snprintf(buf, buf_len, "RETI"); break;
        case 0xC0: case 0xC8: case 0xD0: case 0xD8:
            snprintf(buf, buf_len, "RET   %s", c_names[(op >> 3) & 3]); break;
        case 0xC7: case 0xCF: case 0xD7: case 0xDF:
        case 0xE7: case 0xEF: case 0xF7: case 0xFF:
            snprintf(buf, buf_len, "RST   0x%02X", ((op >> 3) & 7) * 8); break;
        case 0xC5: case 0xD5: case 0xE5: case 0xF5:
            snprintf(buf, buf_len, "PUSH  %s", rp2_names[(op >> 4) & 3]); break;
        case 0xC1: case 0xD1: case 0xE1: case 0xF1:
            snprintf(buf, buf_len, "POP   %s", rp2_names[(op >> 4) & 3]); break;
        case 0xE0: snprintf(buf, buf_len, "LDH   (0x%02X), A", byte2); break;
        case 0xF0: snprintf(buf, buf_len, "LDH   A, (0x%02X)", byte2); break;
        case 0xE2: snprintf(buf, buf_len, "LD    (C), A"); break;
        case 0xF2: snprintf(buf, buf_len, "LD    A, (C)"); break;
        case 0xEA: snprintf(buf, buf_len, "LD    (0x%04X), A", imm16); break;
        case 0xFA: snprintf(buf, buf_len, "LD    A, (0x%04X)", imm16); break;
        case 0xE8: snprintf(buf, buf_len, "ADD   SP, %d", rel); break;
        case 0xF8: snprintf(buf, buf_len, "LD    HL, SP%+d", rel); break;
        case 0xF9: snprintf(buf, buf_len, "LD    SP, HL"); break;
        case 0xF3: snprintf(buf, buf_len, "DI"); break;
        case 0xFB: snprintf(buf, buf_len, "EI"); break;
        default: snprintf(buf, buf_len, "INV   0x%02X", op); break;
    }
}
