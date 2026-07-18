#include "z8.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB program memory
#define REG_FILE_SIZE 256

// Control register addresses
#define REG_FLAGS 0xFC
#define REG_RP    0xFD
#define REG_SPH   0xFE
#define REG_SPL   0xFF

typedef struct Z8CPU {
    uint8_t reg[REG_FILE_SIZE]; // Register file (FLAGS/RP/SPH/SPL are shadowed below)
    uint16_t pc;                // Program Counter
    uint16_t sp;                // Stack Pointer (SPH:SPL, internal stack in register file)
    uint8_t rp;                 // Register Pointer (working register group)

    // Flags (FLAGS register 0xFC, bits 7..2)
    uint8_t fc; // Carry
    uint8_t fz; // Zero
    uint8_t fs; // Sign
    uint8_t fv; // Overflow
    uint8_t fd; // Decimal Adjust
    uint8_t fh; // Half Carry

    uint8_t ie; // Interrupt enable (EI/DI)
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} Z8CPU;

static const char* cc_names[16] = {
    "F", "LT", "LE", "ULE", "OV", "MI", "Z", "C",
    "T", "GE", "GT", "UGT", "NOV", "PL", "NZ", "NC"
};

static const char* alu_names[12] = {
    "ADD", "ADC", "SUB", "SBC", "OR", "AND", "TCM", "TM", "?", "?", "CP", "XOR"
};

static const char* sop_names[16] = {
    "DEC", "RLC", "INC", "?", "DA", "POP", "COM", "PUSH",
    "DECW", "RL", "INCW", "CLR", "RRC", "SRA", "RR", "SWAP"
};

static uint8_t pack_flags(Z8CPU *cpu) {
    return (uint8_t)((cpu->fc << 7) | (cpu->fz << 6) | (cpu->fs << 5) |
                     (cpu->fv << 4) | (cpu->fd << 3) | (cpu->fh << 2));
}

static void unpack_flags(Z8CPU *cpu, uint8_t val) {
    cpu->fc = (uint8_t)((val >> 7) & 1);
    cpu->fz = (uint8_t)((val >> 6) & 1);
    cpu->fs = (uint8_t)((val >> 5) & 1);
    cpu->fv = (uint8_t)((val >> 4) & 1);
    cpu->fd = (uint8_t)((val >> 3) & 1);
    cpu->fh = (uint8_t)((val >> 2) & 1);
}

// Resolve the E0-EF escape (working register addressing) in a register operand
static uint8_t reg_addr(Z8CPU *cpu, uint8_t addr) {
    if ((addr & 0xF0) == 0xE0) {
        return (uint8_t)((cpu->rp & 0xF0) | (addr & 0x0F));
    }
    return addr;
}

// Address of working register n in the current group
static uint8_t wr(Z8CPU *cpu, uint8_t n) {
    return (uint8_t)((cpu->rp & 0xF0) | (n & 0x0F));
}

static uint8_t reg_read(Z8CPU *cpu, uint8_t addr) {
    switch (addr) {
        case REG_FLAGS: return pack_flags(cpu);
        case REG_RP:    return cpu->rp;
        case REG_SPH:   return (uint8_t)(cpu->sp >> 8);
        case REG_SPL:   return (uint8_t)(cpu->sp & 0xFF);
        default:        return cpu->reg[addr];
    }
}

static void reg_write(Z8CPU *cpu, uint8_t addr, uint8_t val) {
    switch (addr) {
        case REG_FLAGS: unpack_flags(cpu, val); break;
        case REG_RP:    cpu->rp = val; break;
        case REG_SPH:   cpu->sp = (uint16_t)((cpu->sp & 0x00FF) | ((uint16_t)val << 8)); break;
        case REG_SPL:   cpu->sp = (uint16_t)((cpu->sp & 0xFF00) | val); break;
        default:        cpu->reg[addr] = val; break;
    }
}

static uint16_t pair_read(Z8CPU *cpu, uint8_t addr) {
    return (uint16_t)(((uint16_t)reg_read(cpu, addr) << 8) |
                      reg_read(cpu, (uint8_t)(addr + 1)));
}

static void pair_write(Z8CPU *cpu, uint8_t addr, uint16_t val) {
    reg_write(cpu, addr, (uint8_t)(val >> 8));
    reg_write(cpu, (uint8_t)(addr + 1), (uint8_t)(val & 0xFF));
}

static void set_zs(Z8CPU *cpu, uint8_t val) {
    cpu->fz = (uint8_t)(val == 0);
    cpu->fs = (uint8_t)(val >> 7);
}

static void push8(Z8CPU *cpu, uint8_t val) {
    cpu->sp = (uint16_t)(cpu->sp - 1);
    reg_write(cpu, (uint8_t)(cpu->sp & 0xFF), val);
}

static uint8_t pop8(Z8CPU *cpu) {
    uint8_t val = reg_read(cpu, (uint8_t)(cpu->sp & 0xFF));
    cpu->sp = (uint16_t)(cpu->sp + 1);
    return val;
}

static void push16(Z8CPU *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val & 0xFF)); // low byte at higher address
    push8(cpu, (uint8_t)(val >> 8));   // high byte at @SP
}

static uint16_t pop16(Z8CPU *cpu) {
    uint8_t hi = pop8(cpu);
    uint8_t lo = pop8(cpu);
    return (uint16_t)(((uint16_t)hi << 8) | lo);
}

static int check_cond(Z8CPU *cpu, uint8_t cc) {
    switch (cc & 0x0F) {
        case 0x0: return 0;                                    // F (never)
        case 0x1: return (cpu->fs ^ cpu->fv) != 0;             // LT
        case 0x2: return (cpu->fz | (cpu->fs ^ cpu->fv)) != 0; // LE
        case 0x3: return (cpu->fc | cpu->fz) != 0;             // ULE
        case 0x4: return cpu->fv != 0;                         // OV
        case 0x5: return cpu->fs != 0;                         // MI
        case 0x6: return cpu->fz != 0;                         // Z / EQ
        case 0x7: return cpu->fc != 0;                         // C / ULT
        case 0x8: return 1;                                    // T (always)
        case 0x9: return (cpu->fs ^ cpu->fv) == 0;             // GE
        case 0xA: return (cpu->fz | (cpu->fs ^ cpu->fv)) == 0; // GT
        case 0xB: return (cpu->fc | cpu->fz) == 0;             // UGT
        case 0xC: return cpu->fv == 0;                         // NOV
        case 0xD: return cpu->fs == 0;                         // PL
        case 0xE: return cpu->fz == 0;                         // NZ / NE
        default:  return cpu->fc == 0;                         // NC / UGE
    }
}

// Two-operand ALU (op = high nibble: 0 ADD, 1 ADC, 2 SUB, 3 SBC, 4 OR,
// 5 AND, 6 TCM, 7 TM, A CP, B XOR). dst is a resolved register file address.
static void alu_op(Z8CPU *cpu, uint8_t aop, uint8_t dst, uint8_t src_val) {
    uint8_t a = reg_read(cpu, dst);
    switch (aop) {
        case 0x0:
        case 0x1: { // ADD / ADC
            uint8_t cy = (aop == 0x1) ? cpu->fc : 0;
            unsigned sum = (unsigned)a + src_val + cy;
            uint8_t res = (uint8_t)sum;
            cpu->fc = (uint8_t)(sum > 0xFF);
            cpu->fh = (uint8_t)(((a & 0x0F) + (src_val & 0x0F) + cy) > 0x0F);
            cpu->fv = (uint8_t)((((a ^ src_val) ^ 0xFF) & (a ^ res) & 0x80) != 0);
            cpu->fd = 0;
            set_zs(cpu, res);
            reg_write(cpu, dst, res);
            break;
        }
        case 0x2:
        case 0x3:
        case 0xA: { // SUB / SBC / CP
            uint8_t cy = (aop == 0x3) ? cpu->fc : 0;
            uint8_t res = (uint8_t)(a - src_val - cy);
            cpu->fc = (uint8_t)(((unsigned)src_val + cy) > a); // borrow
            cpu->fv = (uint8_t)(((a ^ src_val) & (a ^ res) & 0x80) != 0);
            set_zs(cpu, res);
            if (aop != 0xA) {
                cpu->fh = (uint8_t)(((src_val & 0x0F) + cy) > (a & 0x0F));
                cpu->fd = 1;
                reg_write(cpu, dst, res);
            }
            break;
        }
        case 0x4: { // OR
            uint8_t res = (uint8_t)(a | src_val);
            set_zs(cpu, res);
            cpu->fv = 0;
            reg_write(cpu, dst, res);
            break;
        }
        case 0x5: { // AND
            uint8_t res = (uint8_t)(a & src_val);
            set_zs(cpu, res);
            cpu->fv = 0;
            reg_write(cpu, dst, res);
            break;
        }
        case 0x6: { // TCM (test complement under mask, flags only)
            uint8_t res = (uint8_t)((uint8_t)(~a) & src_val);
            set_zs(cpu, res);
            cpu->fv = 0;
            break;
        }
        case 0x7: { // TM (test under mask, flags only)
            uint8_t res = (uint8_t)(a & src_val);
            set_zs(cpu, res);
            cpu->fv = 0;
            break;
        }
        case 0xB: { // XOR
            uint8_t res = (uint8_t)(a ^ src_val);
            set_zs(cpu, res);
            cpu->fv = 0;
            reg_write(cpu, dst, res);
            break;
        }
        default:
            break;
    }
}

// Single-operand ops (column 0/1, sop = high nibble). addr is resolved.
static void single_op(Z8CPU *cpu, uint8_t sop, uint8_t addr) {
    switch (sop) {
        case 0x0: { // DEC
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)(v - 1);
            cpu->fv = (uint8_t)(v == 0x80);
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0x2: { // INC
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)(v + 1);
            cpu->fv = (uint8_t)(v == 0x7F);
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0x1: { // RLC
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)((v << 1) | cpu->fc);
            cpu->fc = (uint8_t)(v >> 7);
            cpu->fv = (uint8_t)(((v ^ res) >> 7) & 1);
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0x9: { // RL
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)((v << 1) | (v >> 7));
            cpu->fc = (uint8_t)(v >> 7);
            cpu->fv = (uint8_t)(((v ^ res) >> 7) & 1);
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0xC: { // RRC
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)((v >> 1) | (cpu->fc << 7));
            cpu->fc = (uint8_t)(v & 1);
            cpu->fv = (uint8_t)(((v ^ res) >> 7) & 1);
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0xE: { // RR
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)((v >> 1) | ((v & 1) << 7));
            cpu->fc = (uint8_t)(v & 1);
            cpu->fv = (uint8_t)(((v ^ res) >> 7) & 1);
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0xD: { // SRA
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)((v >> 1) | (v & 0x80));
            cpu->fc = (uint8_t)(v & 1);
            cpu->fv = 0;
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0xF: { // SWAP
            uint8_t v = reg_read(cpu, addr);
            uint8_t res = (uint8_t)((v << 4) | (v >> 4));
            cpu->fv = 0;
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0x4: { // DA
            unsigned t = reg_read(cpu, addr);
            if (cpu->fd == 0) { // after addition
                if (cpu->fh || (t & 0x0F) > 9) t += 0x06;
                if (cpu->fc || t > 0x9F) { t += 0x60; cpu->fc = 1; }
            } else { // after subtraction
                if (cpu->fh) t -= 0x06;
                if (cpu->fc) t -= 0x60;
            }
            set_zs(cpu, (uint8_t)t);
            reg_write(cpu, addr, (uint8_t)t);
            break;
        }
        case 0x6: { // COM
            uint8_t res = (uint8_t)(~reg_read(cpu, addr));
            cpu->fv = 0;
            set_zs(cpu, res);
            reg_write(cpu, addr, res);
            break;
        }
        case 0xB: // CLR (no flags)
            reg_write(cpu, addr, 0);
            break;
        case 0x5: // POP
            reg_write(cpu, addr, pop8(cpu));
            break;
        case 0x7: // PUSH
            push8(cpu, reg_read(cpu, addr));
            break;
        case 0x8: { // DECW
            uint16_t v = pair_read(cpu, addr);
            uint16_t res = (uint16_t)(v - 1);
            cpu->fv = (uint8_t)(v == 0x8000);
            cpu->fz = (uint8_t)(res == 0);
            cpu->fs = (uint8_t)(res >> 15);
            pair_write(cpu, addr, res);
            break;
        }
        case 0xA: { // INCW
            uint16_t v = pair_read(cpu, addr);
            uint16_t res = (uint16_t)(v + 1);
            cpu->fv = (uint8_t)(v == 0x7FFF);
            cpu->fz = (uint8_t)(res == 0);
            cpu->fs = (uint8_t)(res >> 15);
            pair_write(cpu, addr, res);
            break;
        }
        default:
            break;
    }
}

static int instr_len(uint8_t op) {
    switch (op) {
        case 0x6F: case 0x7F: case 0x8F: case 0x9F: case 0xAF:
        case 0xBF: case 0xCF: case 0xDF: case 0xEF: case 0xFF:
            return 1; // column F specials
        case 0xC7: case 0xD7: // indexed LD
        case 0xD6:            // CALL DA
        case 0xE4: case 0xE5: case 0xE6: case 0xE7: case 0xF5: // LD R forms
            return 3;
        case 0xD4: // CALL @RR
            return 2;
        default:
            break;
    }
    switch (op & 0x0F) {
        case 0x0: case 0x1: case 0x2: case 0x3:
        case 0x8: case 0x9: case 0xA: case 0xB: case 0xC:
            return 2;
        case 0x4: case 0x5: case 0x6: case 0x7:
        case 0xD:
            return 3;
        default: // 0xE (INC r) and unassigned 0xF entries
            return 1;
    }
}

void* z8_create(void) {
    Z8CPU *cpu = (Z8CPU*)calloc(1, sizeof(Z8CPU));
    return cpu;
}

void z8_destroy(void *context) {
    free(context);
}

int z8_init(void *context) {
    if (!context) return -1;
    Z8CPU *cpu = (Z8CPU*)context;

    memset(cpu->reg, 0, sizeof(cpu->reg));
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->pc = 0;
    cpu->sp = 0;
    cpu->rp = 0;
    cpu->fc = cpu->fz = cpu->fs = cpu->fv = cpu->fd = cpu->fh = 0;
    cpu->ie = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int z8_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    Z8CPU *cpu = (Z8CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int z8_step(void *context) {
    if (!context) return -1;
    Z8CPU *cpu = (Z8CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint8_t op = cpu->memory[cpu->pc];
    uint8_t b2 = cpu->memory[(cpu->pc + 1) & 0xFFFF];
    uint8_t b3 = cpu->memory[(cpu->pc + 2) & 0xFFFF];
    uint8_t hi = (uint8_t)(op >> 4);
    uint8_t lo = (uint8_t)(op & 0x0F);

    uint16_t next_pc = (uint16_t)(cpu->pc + instr_len(op));
    cpu->pc = next_pc;
    cpu->ticks++;

    switch (op) {
        case 0x6F: // STOP
        case 0x7F: // HALT
            cpu->halted = 1;
            return 1;
        case 0x8F: cpu->ie = 0; goto done; // DI
        case 0x9F: cpu->ie = 1; goto done; // EI
        case 0xAF: // RET
            cpu->pc = pop16(cpu);
            goto done;
        case 0xBF: // IRET
            unpack_flags(cpu, pop8(cpu));
            cpu->pc = pop16(cpu);
            cpu->ie = 1;
            goto done;
        case 0xCF: cpu->fc = 0; goto done;                    // RCF
        case 0xDF: cpu->fc = 1; goto done;                    // SCF
        case 0xEF: cpu->fc = (uint8_t)(cpu->fc ^ 1); goto done; // CCF
        case 0xFF: goto done;                                 // NOP
        case 0x30: // JP @RR
            cpu->pc = pair_read(cpu, reg_addr(cpu, b2));
            goto done;
        case 0x31: // SRP #imm
            cpu->rp = b2;
            goto done;
        case 0xD6: // CALL DA
            push16(cpu, next_pc);
            cpu->pc = (uint16_t)(((uint16_t)b2 << 8) | b3);
            goto done;
        case 0xD4: // CALL @RR
            push16(cpu, next_pc);
            cpu->pc = pair_read(cpu, reg_addr(cpu, b2));
            goto done;
        case 0xE3: // LD r, @r
            reg_write(cpu, wr(cpu, (uint8_t)(b2 >> 4)),
                      reg_read(cpu, reg_read(cpu, wr(cpu, (uint8_t)(b2 & 0x0F)))));
            goto done;
        case 0xF3: // LD @r, r
            reg_write(cpu, reg_read(cpu, wr(cpu, (uint8_t)(b2 >> 4))),
                      reg_read(cpu, wr(cpu, (uint8_t)(b2 & 0x0F))));
            goto done;
        case 0xE4: // LD R, R (src, dst)
            reg_write(cpu, reg_addr(cpu, b3), reg_read(cpu, reg_addr(cpu, b2)));
            goto done;
        case 0xE5: // LD R, @R
            reg_write(cpu, reg_addr(cpu, b3),
                      reg_read(cpu, reg_read(cpu, reg_addr(cpu, b2))));
            goto done;
        case 0xE6: // LD R, #imm
            reg_write(cpu, reg_addr(cpu, b2), b3);
            goto done;
        case 0xE7: // LD @R, #imm
            reg_write(cpu, reg_read(cpu, reg_addr(cpu, b2)), b3);
            goto done;
        case 0xF5: // LD @R, R (src, dst)
            reg_write(cpu, reg_read(cpu, reg_addr(cpu, b3)),
                      reg_read(cpu, reg_addr(cpu, b2)));
            goto done;
        case 0xC7: { // LD r, x(r)
            uint8_t idx = wr(cpu, (uint8_t)(b2 & 0x0F));
            uint8_t addr = (uint8_t)(b3 + reg_read(cpu, idx));
            reg_write(cpu, wr(cpu, (uint8_t)(b2 >> 4)), reg_read(cpu, addr));
            goto done;
        }
        case 0xD7: { // LD x(r), r
            uint8_t idx = wr(cpu, (uint8_t)(b2 & 0x0F));
            uint8_t addr = (uint8_t)(b3 + reg_read(cpu, idx));
            reg_write(cpu, addr, reg_read(cpu, wr(cpu, (uint8_t)(b2 >> 4))));
            goto done;
        }
        case 0xC2:   // LDC r, @rr (program memory)
        case 0x82: { // LDE r, @rr (external data memory, mapped to same 64K here)
            uint8_t p = wr(cpu, (uint8_t)(b2 & 0x0F));
            reg_write(cpu, wr(cpu, (uint8_t)(b2 >> 4)),
                      cpu->memory[pair_read(cpu, p)]);
            goto done;
        }
        case 0xD2:   // LDC @rr, r
        case 0x92: { // LDE @rr, r
            uint8_t p = wr(cpu, (uint8_t)(b2 & 0x0F));
            cpu->memory[pair_read(cpu, p)] =
                reg_read(cpu, wr(cpu, (uint8_t)(b2 >> 4)));
            goto done;
        }
        case 0xC3:   // LDCI @r, @rr
        case 0x83: { // LDEI @r, @rr
            uint8_t pr = wr(cpu, (uint8_t)(b2 >> 4));
            uint8_t p = wr(cpu, (uint8_t)(b2 & 0x0F));
            uint16_t maddr = pair_read(cpu, p);
            reg_write(cpu, reg_read(cpu, pr), cpu->memory[maddr]);
            reg_write(cpu, pr, (uint8_t)(reg_read(cpu, pr) + 1));
            pair_write(cpu, p, (uint16_t)(maddr + 1));
            goto done;
        }
        case 0xD3:   // LDCI @rr, @r
        case 0x93: { // LDEI @rr, @r
            uint8_t pr = wr(cpu, (uint8_t)(b2 >> 4));
            uint8_t p = wr(cpu, (uint8_t)(b2 & 0x0F));
            uint16_t maddr = pair_read(cpu, p);
            cpu->memory[maddr] = reg_read(cpu, reg_read(cpu, pr));
            reg_write(cpu, pr, (uint8_t)(reg_read(cpu, pr) + 1));
            pair_write(cpu, p, (uint16_t)(maddr + 1));
            goto done;
        }
        default:
            break;
    }

    if (lo >= 0x2 && lo <= 0x7 && (hi <= 0x7 || hi == 0xA || hi == 0xB)) {
        // Two-operand ALU instructions
        uint8_t dst = 0;
        uint8_t srcv = 0;
        switch (lo) {
            case 0x2: // op r, r
                dst = wr(cpu, (uint8_t)(b2 >> 4));
                srcv = reg_read(cpu, wr(cpu, (uint8_t)(b2 & 0x0F)));
                break;
            case 0x3: // op r, @r
                dst = wr(cpu, (uint8_t)(b2 >> 4));
                srcv = reg_read(cpu, reg_read(cpu, wr(cpu, (uint8_t)(b2 & 0x0F))));
                break;
            case 0x4: // op R, R (src, dst)
                srcv = reg_read(cpu, reg_addr(cpu, b2));
                dst = reg_addr(cpu, b3);
                break;
            case 0x5: // op R, @R (src, dst)
                srcv = reg_read(cpu, reg_read(cpu, reg_addr(cpu, b2)));
                dst = reg_addr(cpu, b3);
                break;
            case 0x6: // op R, #imm
                dst = reg_addr(cpu, b2);
                srcv = b3;
                break;
            default: // 0x7: op @R, #imm
                dst = reg_read(cpu, reg_addr(cpu, b2));
                srcv = b3;
                break;
        }
        alu_op(cpu, hi, dst, srcv);
    }
    else if (lo <= 0x1) {
        // Single-operand instructions (hi 3 = JP @RR / SRP, handled above)
        uint8_t addr = reg_addr(cpu, b2);
        if (lo == 0x1) addr = reg_read(cpu, addr);
        single_op(cpu, hi, addr);
    }
    else if (lo == 0x8) { // LD r, R
        reg_write(cpu, wr(cpu, hi), reg_read(cpu, reg_addr(cpu, b2)));
    }
    else if (lo == 0x9) { // LD R, r
        reg_write(cpu, reg_addr(cpu, b2), reg_read(cpu, wr(cpu, hi)));
    }
    else if (lo == 0xA) { // DJNZ r, RA
        uint8_t r = wr(cpu, hi);
        uint8_t val = (uint8_t)(reg_read(cpu, r) - 1);
        reg_write(cpu, r, val);
        if (val != 0) {
            cpu->pc = (uint16_t)(next_pc + (int8_t)b2);
        }
    }
    else if (lo == 0xB) { // JR cc, RA
        if (check_cond(cpu, hi)) {
            cpu->pc = (uint16_t)(next_pc + (int8_t)b2);
        }
    }
    else if (lo == 0xC) { // LD r, #imm
        reg_write(cpu, wr(cpu, hi), b2);
    }
    else if (lo == 0xD) { // JP cc, DA
        if (check_cond(cpu, hi)) {
            cpu->pc = (uint16_t)(((uint16_t)b2 << 8) | b3);
        }
    }
    else if (lo == 0xE) { // INC r
        single_op(cpu, 0x2, wr(cpu, hi));
    }
    // Remaining encodings are unassigned on the Z8: treated as NOPs

done:
    // PC self-loop is interpreted as a software halt
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

void z8_print_state(void *context) {
    if (!context) return;
    Z8CPU *cpu = (Z8CPU*)context;

    printf("Zilog Z8 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  RP: 0x%02X  IE: %d  Halted: %s\n",
           cpu->pc, cpu->sp, cpu->rp, cpu->ie, cpu->halted ? "Yes" : "No");
    printf("  Flags: C=%d Z=%d S=%d V=%d D=%d H=%d (FLAGS=0x%02X)\n",
           cpu->fc, cpu->fz, cpu->fs, cpu->fv, cpu->fd, cpu->fh, pack_flags(cpu));
    printf("  Working registers (group 0x%02X):\n", cpu->rp & 0xF0);
    for (int i = 0; i < 16; ++i) {
        printf("    r%-2d: 0x%02X%s", i, reg_read(cpu, wr(cpu, (uint8_t)i)),
               (i % 4 == 3) ? "\n" : "  ");
    }
}

// Format a register operand, showing the E0-EF working register escape as rN
static void fmt_reg(char *out, size_t out_len, uint8_t addr) {
    if ((addr & 0xF0) == 0xE0) {
        snprintf(out, out_len, "r%d", addr & 0x0F);
    } else {
        snprintf(out, out_len, "0x%02X", addr);
    }
}

void z8_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    Z8CPU *cpu = (Z8CPU*)context;

    uint8_t op = cpu->memory[cpu->pc];
    uint8_t b2 = cpu->memory[(cpu->pc + 1) & 0xFFFF];
    uint8_t b3 = cpu->memory[(cpu->pc + 2) & 0xFFFF];
    uint8_t hi = (uint8_t)(op >> 4);
    uint8_t lo = (uint8_t)(op & 0x0F);
    uint16_t next_pc = (uint16_t)(cpu->pc + instr_len(op));
    char t1[16], t2[16];

    switch (op) {
        case 0x6F: snprintf(buf, buf_len, "STOP"); return;
        case 0x7F: snprintf(buf, buf_len, "HALT"); return;
        case 0x8F: snprintf(buf, buf_len, "DI"); return;
        case 0x9F: snprintf(buf, buf_len, "EI"); return;
        case 0xAF: snprintf(buf, buf_len, "RET"); return;
        case 0xBF: snprintf(buf, buf_len, "IRET"); return;
        case 0xCF: snprintf(buf, buf_len, "RCF"); return;
        case 0xDF: snprintf(buf, buf_len, "SCF"); return;
        case 0xEF: snprintf(buf, buf_len, "CCF"); return;
        case 0xFF: snprintf(buf, buf_len, "NOP"); return;
        case 0x30:
            fmt_reg(t1, sizeof(t1), b2);
            snprintf(buf, buf_len, "JP    @%s", t1);
            return;
        case 0x31: snprintf(buf, buf_len, "SRP   #0x%02X", b2); return;
        case 0xD6:
            snprintf(buf, buf_len, "CALL  0x%04X", ((uint16_t)b2 << 8) | b3);
            return;
        case 0xD4:
            fmt_reg(t1, sizeof(t1), b2);
            snprintf(buf, buf_len, "CALL  @%s", t1);
            return;
        case 0xE3:
            snprintf(buf, buf_len, "LD    r%d, @r%d", b2 >> 4, b2 & 0x0F);
            return;
        case 0xF3:
            snprintf(buf, buf_len, "LD    @r%d, r%d", b2 >> 4, b2 & 0x0F);
            return;
        case 0xE4:
            fmt_reg(t1, sizeof(t1), b3);
            fmt_reg(t2, sizeof(t2), b2);
            snprintf(buf, buf_len, "LD    %s, %s", t1, t2);
            return;
        case 0xE5:
            fmt_reg(t1, sizeof(t1), b3);
            fmt_reg(t2, sizeof(t2), b2);
            snprintf(buf, buf_len, "LD    %s, @%s", t1, t2);
            return;
        case 0xE6:
            fmt_reg(t1, sizeof(t1), b2);
            snprintf(buf, buf_len, "LD    %s, #0x%02X", t1, b3);
            return;
        case 0xE7:
            fmt_reg(t1, sizeof(t1), b2);
            snprintf(buf, buf_len, "LD    @%s, #0x%02X", t1, b3);
            return;
        case 0xF5:
            fmt_reg(t1, sizeof(t1), b3);
            fmt_reg(t2, sizeof(t2), b2);
            snprintf(buf, buf_len, "LD    @%s, %s", t1, t2);
            return;
        case 0xC7:
            snprintf(buf, buf_len, "LD    r%d, 0x%02X(r%d)", b2 >> 4, b3, b2 & 0x0F);
            return;
        case 0xD7:
            snprintf(buf, buf_len, "LD    0x%02X(r%d), r%d", b3, b2 & 0x0F, b2 >> 4);
            return;
        case 0xC2:
            snprintf(buf, buf_len, "LDC   r%d, @rr%d", b2 >> 4, b2 & 0x0F);
            return;
        case 0xD2:
            snprintf(buf, buf_len, "LDC   @rr%d, r%d", b2 & 0x0F, b2 >> 4);
            return;
        case 0x82:
            snprintf(buf, buf_len, "LDE   r%d, @rr%d", b2 >> 4, b2 & 0x0F);
            return;
        case 0x92:
            snprintf(buf, buf_len, "LDE   @rr%d, r%d", b2 & 0x0F, b2 >> 4);
            return;
        case 0xC3:
            snprintf(buf, buf_len, "LDCI  @r%d, @rr%d", b2 >> 4, b2 & 0x0F);
            return;
        case 0xD3:
            snprintf(buf, buf_len, "LDCI  @rr%d, @r%d", b2 & 0x0F, b2 >> 4);
            return;
        case 0x83:
            snprintf(buf, buf_len, "LDEI  @r%d, @rr%d", b2 >> 4, b2 & 0x0F);
            return;
        case 0x93:
            snprintf(buf, buf_len, "LDEI  @rr%d, @r%d", b2 & 0x0F, b2 >> 4);
            return;
        default:
            break;
    }

    if (lo >= 0x2 && lo <= 0x7 && (hi <= 0x7 || hi == 0xA || hi == 0xB)) {
        const char *name = alu_names[hi];
        switch (lo) {
            case 0x2:
                snprintf(buf, buf_len, "%-5s r%d, r%d", name, b2 >> 4, b2 & 0x0F);
                return;
            case 0x3:
                snprintf(buf, buf_len, "%-5s r%d, @r%d", name, b2 >> 4, b2 & 0x0F);
                return;
            case 0x4:
                fmt_reg(t1, sizeof(t1), b3);
                fmt_reg(t2, sizeof(t2), b2);
                snprintf(buf, buf_len, "%-5s %s, %s", name, t1, t2);
                return;
            case 0x5:
                fmt_reg(t1, sizeof(t1), b3);
                fmt_reg(t2, sizeof(t2), b2);
                snprintf(buf, buf_len, "%-5s %s, @%s", name, t1, t2);
                return;
            case 0x6:
                fmt_reg(t1, sizeof(t1), b2);
                snprintf(buf, buf_len, "%-5s %s, #0x%02X", name, t1, b3);
                return;
            default: // 0x7
                fmt_reg(t1, sizeof(t1), b2);
                snprintf(buf, buf_len, "%-5s @%s, #0x%02X", name, t1, b3);
                return;
        }
    }
    else if (lo <= 0x1 && hi != 0x3) {
        fmt_reg(t1, sizeof(t1), b2);
        snprintf(buf, buf_len, "%-5s %s%s", sop_names[hi], (lo == 0x1) ? "@" : "", t1);
    }
    else if (lo == 0x8) {
        fmt_reg(t1, sizeof(t1), b2);
        snprintf(buf, buf_len, "LD    r%d, %s", hi, t1);
    }
    else if (lo == 0x9) {
        fmt_reg(t1, sizeof(t1), b2);
        snprintf(buf, buf_len, "LD    %s, r%d", t1, hi);
    }
    else if (lo == 0xA) {
        snprintf(buf, buf_len, "DJNZ  r%d, 0x%04X", hi,
                 (uint16_t)(next_pc + (int8_t)b2));
    }
    else if (lo == 0xB) {
        uint16_t target = (uint16_t)(next_pc + (int8_t)b2);
        if (hi == 0x8) snprintf(buf, buf_len, "JR    0x%04X", target);
        else snprintf(buf, buf_len, "JR    %s, 0x%04X", cc_names[hi], target);
    }
    else if (lo == 0xC) {
        snprintf(buf, buf_len, "LD    r%d, #0x%02X", hi, b2);
    }
    else if (lo == 0xD) {
        uint16_t target = (uint16_t)(((uint16_t)b2 << 8) | b3);
        if (hi == 0x8) snprintf(buf, buf_len, "JP    0x%04X", target);
        else snprintf(buf, buf_len, "JP    %s, 0x%04X", cc_names[hi], target);
    }
    else if (lo == 0xE) {
        snprintf(buf, buf_len, "INC   r%d", hi);
    }
    else {
        snprintf(buf, buf_len, "INV   0x%02X", op);
    }
}
