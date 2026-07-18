#include "s2650.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 32768   // 32 KB (15-bit address space)
#define ADDR_MASK 0x7FFF // 15-bit address mask
#define PAGE_MASK 0x1FFF // 13-bit offset within an 8 KB page
#define PAGE_BITS 0x6000 // page number bits of an address

// Program Status Upper bits
#define PSU_S  0x80 // Sense input
#define PSU_F  0x40 // Flag output
#define PSU_II 0x20 // Interrupt Inhibit
#define PSU_SP 0x07 // Return address stack pointer (3 bits)

// Program Status Lower bits
#define PSL_CC  0xC0 // Condition Code field (CC1:CC0)
#define PSL_IDC 0x20 // Inter-Digit Carry
#define PSL_RS  0x10 // Register bank Select
#define PSL_WC  0x08 // With Carry
#define PSL_OVF 0x04 // Overflow
#define PSL_COM 0x02 // logical COMpare when set, arithmetic when clear
#define PSL_C   0x01 // Carry/borrow

typedef struct S2650CPU {
    uint8_t r0;          // Register 0 (shared between banks)
    uint8_t rbank[2][3]; // R1-R3, two banks selected by PSL RS bit
    uint16_t pc;         // 15-bit Program Counter (page + 13-bit IAR)
    uint8_t psu;         // Program Status Upper
    uint8_t psl;         // Program Status Lower
    uint16_t ras[8];     // 8-level internal Return Address Stack

    uint8_t io_port[2];  // Non-extended ports: [0] = control (C), [1] = data (D)
    uint8_t io_ext[256]; // Extended I/O ports (REDE/WRTE)

    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} S2650CPU;

static const char* cc_names[] = { "EQ", "GT", "LT", "UN" };

// Instruction length by opcode class (opcode >> 2)
static const uint8_t len_table[64] = {
    1, 2, 2, 3,  1, 1, 2, 3,  1, 2, 2, 3,  1, 1, 2, 3,
    1, 2, 2, 3,  1, 2, 2, 3,  1, 2, 2, 3,  1, 2, 2, 3,
    1, 2, 2, 3,  1, 1, 2, 3,  1, 2, 2, 3,  1, 2, 2, 3,
    1, 2, 2, 3,  1, 2, 2, 3,  1, 2, 2, 3,  1, 2, 2, 3
};

static inline uint8_t rd_mem(S2650CPU *cpu, uint16_t addr) {
    return cpu->memory[addr & ADDR_MASK];
}

static inline void wr_mem(S2650CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->memory[addr & ADDR_MASK] = val;
}

// Fetch a 15-bit indirect address; the second pointer byte wraps within the page
static inline uint16_t rd_indirect(S2650CPU *cpu, uint16_t addr) {
    uint8_t hi = rd_mem(cpu, addr);
    uint8_t lo = rd_mem(cpu, (uint16_t)((addr & PAGE_BITS) | ((addr + 1) & PAGE_MASK)));
    return (uint16_t)((((uint16_t)hi << 8) | lo) & ADDR_MASK);
}

static inline uint8_t get_reg(S2650CPU *cpu, uint8_t idx) {
    if (idx == 0) return cpu->r0;
    return cpu->rbank[(cpu->psl & PSL_RS) ? 1 : 0][idx - 1];
}

static inline void set_reg(S2650CPU *cpu, uint8_t idx, uint8_t val) {
    if (idx == 0) cpu->r0 = val;
    else cpu->rbank[(cpu->psl & PSL_RS) ? 1 : 0][idx - 1] = val;
}

// Set the condition code field from a result: 0 = zero, 1 = positive, 2 = negative
static inline void set_cc(S2650CPU *cpu, uint8_t val) {
    uint8_t cc = 0;
    if (val != 0) cc = (uint8_t)((val & 0x80) ? 0x80 : 0x40);
    cpu->psl = (uint8_t)((cpu->psl & ~PSL_CC) | cc);
}

// Test a branch/return condition field (3 = unconditional)
static inline int test_cc(S2650CPU *cpu, uint8_t cc) {
    return (cc == 3) || (((cpu->psl >> 6) & 3) == cc);
}

static inline void ras_push(S2650CPU *cpu, uint16_t addr) {
    uint8_t sp = (uint8_t)((cpu->psu + 1) & PSU_SP);
    cpu->psu = (uint8_t)((cpu->psu & ~PSU_SP) | sp);
    cpu->ras[sp] = (uint16_t)(addr & ADDR_MASK);
}

static inline uint16_t ras_pop(S2650CPU *cpu) {
    uint8_t sp = (uint8_t)(cpu->psu & PSU_SP);
    uint16_t addr = cpu->ras[sp];
    cpu->psu = (uint8_t)((cpu->psu & ~PSU_SP) | ((sp - 1) & PSU_SP));
    return addr;
}

// Sign-extend the 7-bit displacement of a relative operand byte
static inline int sign7(uint8_t hr) {
    int off = hr & 0x3F;
    if (hr & 0x40) off -= 64;
    return off;
}

// Relative effective address: displacement from the next instruction, wrapping
// within the current page; bit 7 of the operand byte selects indirect addressing
static uint16_t rel_ea(S2650CPU *cpu, uint8_t hr, uint16_t next_pc) {
    uint16_t ea = (uint16_t)((next_pc & PAGE_BITS) |
                             ((next_pc + sign7(hr)) & PAGE_MASK));
    if (hr & 0x80) ea = rd_indirect(cpu, ea);
    return ea;
}

// Zero-page effective address for ZBRR/ZBSR: -64..+63 around address 0 of page 0
static uint16_t zero_ea(S2650CPU *cpu, uint8_t hr) {
    uint16_t ea = (uint16_t)((sign7(hr)) & PAGE_MASK);
    if (hr & 0x80) ea = rd_indirect(cpu, ea);
    return ea;
}

// Absolute (non-branch) effective address: 13 bits within the current page,
// optional indirection, then optional indexing with auto-increment/decrement.
// When indexed, the data register becomes R0 and r is the index register.
static uint16_t abs_ea(S2650CPU *cpu, uint8_t r, uint16_t page, uint8_t hr,
                       uint8_t dr, uint8_t *data_reg) {
    uint16_t ea = (uint16_t)(page | (((uint16_t)(hr & 0x1F) << 8) | dr));
    uint8_t idx = hr & 0x60;
    *data_reg = r;
    if (hr & 0x80) ea = rd_indirect(cpu, ea);
    if (idx != 0) {
        if (idx == 0x20) set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) + 1));
        else if (idx == 0x40) set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) - 1));
        ea = (uint16_t)((ea & PAGE_BITS) | ((ea + get_reg(cpu, r)) & PAGE_MASK));
        *data_reg = 0;
    }
    return ea;
}

// Branch absolute effective address: full 15-bit target, bit 7 selects indirect
static uint16_t bra_ea(S2650CPU *cpu, uint8_t hr, uint8_t dr) {
    uint16_t ea = (uint16_t)((((uint16_t)(hr & 0x7F) << 8) | dr) & ADDR_MASK);
    if (hr & 0x80) ea = rd_indirect(cpu, ea);
    return ea;
}

static uint8_t do_add(S2650CPU *cpu, uint8_t a, uint8_t b) {
    unsigned cin = (cpu->psl & PSL_WC) ? (cpu->psl & PSL_C) : 0;
    unsigned sum = (unsigned)a + b + cin;
    uint8_t res = (uint8_t)sum;
    cpu->psl &= (uint8_t)~(PSL_C | PSL_OVF | PSL_IDC);
    if (sum > 0xFF) cpu->psl |= PSL_C;
    if (((a & 0x0F) + (b & 0x0F) + cin) > 0x0F) cpu->psl |= PSL_IDC;
    if (~(a ^ b) & (a ^ res) & 0x80) cpu->psl |= PSL_OVF;
    set_cc(cpu, res);
    return res;
}

static uint8_t do_sub(S2650CPU *cpu, uint8_t a, uint8_t b) {
    unsigned cin = (cpu->psl & PSL_WC) ? (cpu->psl & PSL_C) : 1;
    unsigned nb = (unsigned)(uint8_t)~b;
    unsigned sum = (unsigned)a + nb + cin;
    uint8_t res = (uint8_t)sum;
    cpu->psl &= (uint8_t)~(PSL_C | PSL_OVF | PSL_IDC);
    if (sum > 0xFF) cpu->psl |= PSL_C; // C = 1 means no borrow
    if (((a & 0x0F) + (nb & 0x0F) + cin) > 0x0F) cpu->psl |= PSL_IDC;
    if ((a ^ b) & (a ^ res) & 0x80) cpu->psl |= PSL_OVF;
    set_cc(cpu, res);
    return res;
}

// Compare a against b: COM bit selects logical (unsigned) vs arithmetic (signed)
static void do_com(S2650CPU *cpu, uint8_t a, uint8_t b) {
    uint8_t cc;
    if (cpu->psl & PSL_COM) {
        cc = (a == b) ? 0x00 : (uint8_t)((a > b) ? 0x40 : 0x80);
    } else {
        int8_t sa = (int8_t)a, sb = (int8_t)b;
        cc = (sa == sb) ? 0x00 : (uint8_t)((sa > sb) ? 0x40 : 0x80);
    }
    cpu->psl = (uint8_t)((cpu->psl & ~PSL_CC) | cc);
}

// Rotate left; with WC set it is a 9-bit rotate through carry, updating IDC
static uint8_t do_rrl(S2650CPU *cpu, uint8_t before) {
    uint8_t res;
    if (cpu->psl & PSL_WC) {
        uint8_t c = (uint8_t)(cpu->psl & PSL_C);
        res = (uint8_t)((before << 1) | c);
        cpu->psl &= (uint8_t)~(PSL_C | PSL_IDC);
        cpu->psl |= (uint8_t)((before >> 7) | (res & PSL_IDC));
    } else {
        res = (uint8_t)((before << 1) | (before >> 7));
    }
    set_cc(cpu, res);
    cpu->psl = (uint8_t)((cpu->psl & ~PSL_OVF) | (((res ^ before) >> 5) & PSL_OVF));
    return res;
}

// Rotate right; with WC set it is a 9-bit rotate through carry, updating IDC
static uint8_t do_rrr(S2650CPU *cpu, uint8_t before) {
    uint8_t res;
    if (cpu->psl & PSL_WC) {
        uint8_t c = (uint8_t)(cpu->psl & PSL_C);
        res = (uint8_t)((before >> 1) | (c << 7));
        cpu->psl &= (uint8_t)~(PSL_C | PSL_IDC);
        cpu->psl |= (uint8_t)((before & PSL_C) | (res & PSL_IDC));
    } else {
        res = (uint8_t)((before >> 1) | (before << 7));
    }
    set_cc(cpu, res);
    cpu->psl = (uint8_t)((cpu->psl & ~PSL_OVF) | (((res ^ before) >> 5) & PSL_OVF));
    return res;
}

void* s2650_create(void) {
    S2650CPU *cpu = (S2650CPU*)calloc(1, sizeof(S2650CPU));
    return cpu;
}

void s2650_destroy(void *context) {
    free(context);
}

int s2650_init(void *context) {
    if (!context) return -1;
    S2650CPU *cpu = (S2650CPU*)context;
    memset(cpu, 0, sizeof(S2650CPU));
    return 0;
}

int s2650_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    S2650CPU *cpu = (S2650CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int s2650_step(void *context) {
    if (!context) return -1;
    S2650CPU *cpu = (S2650CPU*)context;

    if (cpu->halted) return 1;

    uint16_t page = (uint16_t)(cpu->pc & PAGE_BITS);
    uint8_t op = rd_mem(cpu, cpu->pc);
    uint8_t r = (uint8_t)(op & 3);
    uint8_t len = len_table[op >> 2];

    // Operand bytes; the instruction address register wraps within its page
    uint8_t byte2 = rd_mem(cpu, (uint16_t)(page | ((cpu->pc + 1) & PAGE_MASK)));
    uint8_t byte3 = rd_mem(cpu, (uint16_t)(page | ((cpu->pc + 2) & PAGE_MASK)));

    uint16_t next_pc = (uint16_t)(page | ((cpu->pc + len) & PAGE_MASK));
    cpu->pc = next_pc;
    cpu->ticks++;

    uint8_t dreg;
    uint16_t ea;

    switch (op >> 2) {
        case 0x00: // LODZ r (LODZ R0 is undefined; treated as R0 -> R0)
            cpu->r0 = get_reg(cpu, r);
            set_cc(cpu, cpu->r0);
            break;
        case 0x01: // LODI,r v
            set_reg(cpu, r, byte2);
            set_cc(cpu, byte2);
            break;
        case 0x02: { // LODR,r (*)a
            uint8_t val = rd_mem(cpu, rel_ea(cpu, byte2, next_pc));
            set_reg(cpu, r, val);
            set_cc(cpu, val);
            break;
        }
        case 0x03: { // LODA,r (*)a(,X)
            uint8_t val;
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            val = rd_mem(cpu, ea);
            set_reg(cpu, dreg, val);
            set_cc(cpu, val);
            break;
        }
        case 0x04: // 0x10-0x11 illegal, 0x12 SPSU, 0x13 SPSL
            if (op == 0x12) {
                cpu->r0 = (uint8_t)(cpu->psu & ~0x18);
                set_cc(cpu, cpu->r0);
            } else if (op == 0x13) {
                cpu->r0 = cpu->psl;
                set_cc(cpu, cpu->r0);
            }
            break;
        case 0x05: // RETC,cc
            if (test_cc(cpu, r)) cpu->pc = ras_pop(cpu);
            break;
        case 0x06: // BCTR,cc (*)a
            if (test_cc(cpu, r)) cpu->pc = rel_ea(cpu, byte2, next_pc);
            break;
        case 0x07: // BCTA,cc (*)a
            if (test_cc(cpu, r)) cpu->pc = bra_ea(cpu, byte2, byte3);
            break;
        case 0x08: // EORZ r
            cpu->r0 = (uint8_t)(cpu->r0 ^ get_reg(cpu, r));
            set_cc(cpu, cpu->r0);
            break;
        case 0x09: // EORI,r v
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) ^ byte2));
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x0A: // EORR,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) ^
                                      rd_mem(cpu, rel_ea(cpu, byte2, next_pc))));
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x0B: // EORA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            set_reg(cpu, dreg, (uint8_t)(get_reg(cpu, dreg) ^ rd_mem(cpu, ea)));
            set_cc(cpu, get_reg(cpu, dreg));
            break;
        case 0x0C: // REDC,r
            set_reg(cpu, r, cpu->io_port[0]);
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x0D: // RETE,cc
            if (test_cc(cpu, r)) {
                cpu->pc = ras_pop(cpu);
                cpu->psu &= (uint8_t)~PSU_II;
            }
            break;
        case 0x0E: // BSTR,cc (*)a
            if (test_cc(cpu, r)) {
                ras_push(cpu, next_pc);
                cpu->pc = rel_ea(cpu, byte2, next_pc);
            }
            break;
        case 0x0F: // BSTA,cc (*)a
            if (test_cc(cpu, r)) {
                ras_push(cpu, next_pc);
                cpu->pc = bra_ea(cpu, byte2, byte3);
            }
            break;
        case 0x10: // 0x40 HALT, ANDZ r
            if (op == 0x40) {
                cpu->halted = 1;
                return 1;
            }
            cpu->r0 = (uint8_t)(cpu->r0 & get_reg(cpu, r));
            set_cc(cpu, cpu->r0);
            break;
        case 0x11: // ANDI,r v
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) & byte2));
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x12: // ANDR,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) &
                                      rd_mem(cpu, rel_ea(cpu, byte2, next_pc))));
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x13: // ANDA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            set_reg(cpu, dreg, (uint8_t)(get_reg(cpu, dreg) & rd_mem(cpu, ea)));
            set_cc(cpu, get_reg(cpu, dreg));
            break;
        case 0x14: // RRR,r
            set_reg(cpu, r, do_rrr(cpu, get_reg(cpu, r)));
            break;
        case 0x15: // REDE,r v
            set_reg(cpu, r, cpu->io_ext[byte2]);
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x16: // BRNR,r (*)a
            if (get_reg(cpu, r) != 0) cpu->pc = rel_ea(cpu, byte2, next_pc);
            break;
        case 0x17: // BRNA,r (*)a
            if (get_reg(cpu, r) != 0) cpu->pc = bra_ea(cpu, byte2, byte3);
            break;
        case 0x18: // IORZ r
            cpu->r0 = (uint8_t)(cpu->r0 | get_reg(cpu, r));
            set_cc(cpu, cpu->r0);
            break;
        case 0x19: // IORI,r v
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) | byte2));
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x1A: // IORR,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) |
                                      rd_mem(cpu, rel_ea(cpu, byte2, next_pc))));
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x1B: // IORA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            set_reg(cpu, dreg, (uint8_t)(get_reg(cpu, dreg) | rd_mem(cpu, ea)));
            set_cc(cpu, get_reg(cpu, dreg));
            break;
        case 0x1C: // REDD,r
            set_reg(cpu, r, cpu->io_port[1]);
            set_cc(cpu, get_reg(cpu, r));
            break;
        case 0x1D: // CPSU/CPSL/PPSU/PPSL v
            switch (op) {
                case 0x74: cpu->psu = (uint8_t)(cpu->psu & ~byte2 & ~0x18); break;
                case 0x75: cpu->psl = (uint8_t)(cpu->psl & ~byte2); break;
                case 0x76: cpu->psu = (uint8_t)((cpu->psu | byte2) & ~0x18); break;
                case 0x77: cpu->psl = (uint8_t)(cpu->psl | byte2); break;
            }
            break;
        case 0x1E: // BSNR,r (*)a
            if (get_reg(cpu, r) != 0) {
                ras_push(cpu, next_pc);
                cpu->pc = rel_ea(cpu, byte2, next_pc);
            }
            break;
        case 0x1F: // BSNA,r (*)a
            if (get_reg(cpu, r) != 0) {
                ras_push(cpu, next_pc);
                cpu->pc = bra_ea(cpu, byte2, byte3);
            }
            break;
        case 0x20: // ADDZ r
            cpu->r0 = do_add(cpu, cpu->r0, get_reg(cpu, r));
            break;
        case 0x21: // ADDI,r v
            set_reg(cpu, r, do_add(cpu, get_reg(cpu, r), byte2));
            break;
        case 0x22: // ADDR,r (*)a
            set_reg(cpu, r, do_add(cpu, get_reg(cpu, r),
                                   rd_mem(cpu, rel_ea(cpu, byte2, next_pc))));
            break;
        case 0x23: // ADDA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            set_reg(cpu, dreg, do_add(cpu, get_reg(cpu, dreg), rd_mem(cpu, ea)));
            break;
        case 0x24: // 0x90-0x91 illegal, 0x92 LPSU, 0x93 LPSL
            if (op == 0x92) cpu->psu = (uint8_t)(cpu->r0 & ~0x18);
            else if (op == 0x93) cpu->psl = cpu->r0;
            break;
        case 0x25: { // DAR,r
            uint8_t val = get_reg(cpu, r);
            if ((cpu->psl & PSL_C) == 0) val = (uint8_t)(val + 0xA0);
            if ((cpu->psl & PSL_IDC) == 0)
                val = (uint8_t)((val & 0xF0) | ((val + 0x0A) & 0x0F));
            set_reg(cpu, r, val);
            break;
        }
        case 0x26: // BCFR,cc (*)a / ZBRR (*)a
            if (op == 0x9B) cpu->pc = zero_ea(cpu, byte2);
            else if (!test_cc(cpu, r)) cpu->pc = rel_ea(cpu, byte2, next_pc);
            break;
        case 0x27: // BCFA,cc (*)a / BXA (*)a,R3
            if (op == 0x9F)
                cpu->pc = (uint16_t)((bra_ea(cpu, byte2, byte3) +
                                      get_reg(cpu, 3)) & ADDR_MASK);
            else if (!test_cc(cpu, r)) cpu->pc = bra_ea(cpu, byte2, byte3);
            break;
        case 0x28: // SUBZ r
            cpu->r0 = do_sub(cpu, cpu->r0, get_reg(cpu, r));
            break;
        case 0x29: // SUBI,r v
            set_reg(cpu, r, do_sub(cpu, get_reg(cpu, r), byte2));
            break;
        case 0x2A: // SUBR,r (*)a
            set_reg(cpu, r, do_sub(cpu, get_reg(cpu, r),
                                   rd_mem(cpu, rel_ea(cpu, byte2, next_pc))));
            break;
        case 0x2B: // SUBA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            set_reg(cpu, dreg, do_sub(cpu, get_reg(cpu, dreg), rd_mem(cpu, ea)));
            break;
        case 0x2C: // WRTC,r
            cpu->io_port[0] = get_reg(cpu, r);
            break;
        case 0x2D: // 0xB4 TPSU v, 0xB5 TPSL v, 0xB6-0xB7 illegal
            if (op == 0xB4)
                cpu->psl = (uint8_t)((cpu->psl & ~PSL_CC) |
                                     (((cpu->psu & byte2) == byte2) ? 0x00 : 0x80));
            else if (op == 0xB5)
                cpu->psl = (uint8_t)((cpu->psl & ~PSL_CC) |
                                     (((cpu->psl & byte2) == byte2) ? 0x00 : 0x80));
            break;
        case 0x2E: // BSFR,cc (*)a / ZBSR (*)a
            if (op == 0xBB) {
                ras_push(cpu, next_pc);
                cpu->pc = zero_ea(cpu, byte2);
            } else if (!test_cc(cpu, r)) {
                ras_push(cpu, next_pc);
                cpu->pc = rel_ea(cpu, byte2, next_pc);
            }
            break;
        case 0x2F: // BSFA,cc (*)a / BSXA (*)a,R3
            if (op == 0xBF) {
                ras_push(cpu, next_pc);
                cpu->pc = (uint16_t)((bra_ea(cpu, byte2, byte3) +
                                      get_reg(cpu, 3)) & ADDR_MASK);
            } else if (!test_cc(cpu, r)) {
                ras_push(cpu, next_pc);
                cpu->pc = bra_ea(cpu, byte2, byte3);
            }
            break;
        case 0x30: // 0xC0 NOP, STRZ r
            if (op != 0xC0) {
                set_reg(cpu, r, cpu->r0);
                set_cc(cpu, cpu->r0);
            }
            break;
        case 0x31: // 0xC4-0xC7 illegal
            break;
        case 0x32: // STRR,r (*)a
            wr_mem(cpu, rel_ea(cpu, byte2, next_pc), get_reg(cpu, r));
            break;
        case 0x33: // STRA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            wr_mem(cpu, ea, get_reg(cpu, dreg));
            break;
        case 0x34: // RRL,r
            set_reg(cpu, r, do_rrl(cpu, get_reg(cpu, r)));
            break;
        case 0x35: // WRTE,r v
            cpu->io_ext[byte2] = get_reg(cpu, r);
            break;
        case 0x36: // BIRR,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) + 1));
            if (get_reg(cpu, r) != 0) cpu->pc = rel_ea(cpu, byte2, next_pc);
            break;
        case 0x37: // BIRA,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) + 1));
            if (get_reg(cpu, r) != 0) cpu->pc = bra_ea(cpu, byte2, byte3);
            break;
        case 0x38: // COMZ r
            do_com(cpu, cpu->r0, get_reg(cpu, r));
            break;
        case 0x39: // COMI,r v
            do_com(cpu, get_reg(cpu, r), byte2);
            break;
        case 0x3A: // COMR,r (*)a
            do_com(cpu, get_reg(cpu, r), rd_mem(cpu, rel_ea(cpu, byte2, next_pc)));
            break;
        case 0x3B: // COMA,r (*)a(,X)
            ea = abs_ea(cpu, r, page, byte2, byte3, &dreg);
            do_com(cpu, get_reg(cpu, dreg), rd_mem(cpu, ea));
            break;
        case 0x3C: // WRTD,r
            cpu->io_port[1] = get_reg(cpu, r);
            break;
        case 0x3D: // TMI,r v
            cpu->psl = (uint8_t)((cpu->psl & ~PSL_CC) |
                                 (((get_reg(cpu, r) & byte2) == byte2) ? 0x00 : 0x80));
            break;
        case 0x3E: // BDRR,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) - 1));
            if (get_reg(cpu, r) != 0) cpu->pc = rel_ea(cpu, byte2, next_pc);
            break;
        case 0x3F: // BDRA,r (*)a
            set_reg(cpu, r, (uint8_t)(get_reg(cpu, r) - 1));
            if (get_reg(cpu, r) != 0) cpu->pc = bra_ea(cpu, byte2, byte3);
            break;
    }

    return 0;
}

void s2650_print_state(void *context) {
    if (!context) return;
    S2650CPU *cpu = (S2650CPU*)context;
    int bank = (cpu->psl & PSL_RS) ? 1 : 0;

    printf("Signetics 2650 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  Halted: %s\n", cpu->pc, cpu->halted ? "Yes" : "No");
    printf("  R0: 0x%02X  R1: 0x%02X  R2: 0x%02X  R3: 0x%02X  (bank %d)\n",
           cpu->r0, cpu->rbank[bank][0], cpu->rbank[bank][1],
           cpu->rbank[bank][2], bank);
    printf("  PSU: 0x%02X (S=%d F=%d II=%d SP=%d)\n",
           cpu->psu,
           (cpu->psu & PSU_S) ? 1 : 0, (cpu->psu & PSU_F) ? 1 : 0,
           (cpu->psu & PSU_II) ? 1 : 0, cpu->psu & PSU_SP);
    printf("  PSL: 0x%02X (CC=%d IDC=%d RS=%d WC=%d OVF=%d COM=%d C=%d)\n",
           cpu->psl,
           (cpu->psl >> 6) & 3, (cpu->psl & PSL_IDC) ? 1 : 0,
           (cpu->psl & PSL_RS) ? 1 : 0, (cpu->psl & PSL_WC) ? 1 : 0,
           (cpu->psl & PSL_OVF) ? 1 : 0, (cpu->psl & PSL_COM) ? 1 : 0,
           cpu->psl & PSL_C);
}

// Format the operand of a relative-addressed instruction (resolved target)
static void fmt_rel(char *out, size_t out_len, uint16_t next_pc, uint8_t hr) {
    uint16_t target = (uint16_t)((next_pc & PAGE_BITS) |
                                 ((next_pc + sign7(hr)) & PAGE_MASK));
    snprintf(out, out_len, "%s0x%04X", (hr & 0x80) ? "*" : "", target);
}

// Format the operand of an absolute-addressed (non-branch) instruction
static void fmt_abs(char *out, size_t out_len, uint16_t page, uint8_t r,
                    uint8_t hr, uint8_t dr) {
    uint16_t addr = (uint16_t)(page | (((uint16_t)(hr & 0x1F) << 8) | dr));
    const char *ind = (hr & 0x80) ? "*" : "";
    switch (hr & 0x60) {
        case 0x00: snprintf(out, out_len, "%s0x%04X", ind, addr); break;
        case 0x20: snprintf(out, out_len, "%s0x%04X,R%d,+", ind, addr, r); break;
        case 0x40: snprintf(out, out_len, "%s0x%04X,R%d,-", ind, addr, r); break;
        default:   snprintf(out, out_len, "%s0x%04X,R%d", ind, addr, r); break;
    }
}

void s2650_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    S2650CPU *cpu = (S2650CPU*)context;

    uint16_t page = (uint16_t)(cpu->pc & PAGE_BITS);
    uint8_t op = rd_mem(cpu, cpu->pc);
    uint8_t r = (uint8_t)(op & 3);
    uint8_t len = len_table[op >> 2];
    uint8_t byte2 = rd_mem(cpu, (uint16_t)(page | ((cpu->pc + 1) & PAGE_MASK)));
    uint8_t byte3 = rd_mem(cpu, (uint16_t)(page | ((cpu->pc + 2) & PAGE_MASK)));
    uint16_t next_pc = (uint16_t)(page | ((cpu->pc + len) & PAGE_MASK));
    uint16_t bra = (uint16_t)((((uint16_t)(byte2 & 0x7F) << 8) | byte3) & ADDR_MASK);
    const char *bind = (byte2 & 0x80) ? "*" : "";
    char opnd[32];

    // Mnemonic roots for the sixteen 4-column rows of the opcode map
    static const char *mem_ops[16] = {
        "LOD", NULL, "EOR", NULL, "AND", NULL, "IOR", NULL,
        "ADD", NULL, "SUB", NULL, "STR", NULL, "COM", NULL
    };

    uint8_t row = (uint8_t)(op >> 4);
    uint8_t mode = (uint8_t)((op >> 2) & 3); // 0=Z, 1=I, 2=R, 3=A

    if ((row & 1) == 0 && mem_ops[row] != NULL &&
        !(row == 12 && mode == 1) && op != 0x40 && op != 0xC0) {
        // Regular memory/register instruction (STRI does not exist)
        const char *root = mem_ops[row];
        switch (mode) {
            case 0:
                snprintf(buf, buf_len, "%sZ  R%d", root, r);
                break;
            case 1:
                snprintf(buf, buf_len, "%sI,R%d 0x%02X", root, r, byte2);
                break;
            case 2:
                fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
                snprintf(buf, buf_len, "%sR,R%d %s", root, r, opnd);
                break;
            default:
                fmt_abs(opnd, sizeof(opnd), page, r, byte2, byte3);
                snprintf(buf, buf_len, "%sA,R%d %s", root, r, opnd);
                break;
        }
        return;
    }

    switch (op >> 2) {
        case 0x04:
            if (op == 0x12) snprintf(buf, buf_len, "SPSU");
            else if (op == 0x13) snprintf(buf, buf_len, "SPSL");
            else snprintf(buf, buf_len, "INV   0x%02X", op);
            break;
        case 0x05: snprintf(buf, buf_len, "RETC,%s", cc_names[r]); break;
        case 0x06:
            fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
            snprintf(buf, buf_len, "BCTR,%s %s", cc_names[r], opnd);
            break;
        case 0x07:
            snprintf(buf, buf_len, "BCTA,%s %s0x%04X", cc_names[r], bind, bra);
            break;
        case 0x0C: snprintf(buf, buf_len, "REDC,R%d", r); break;
        case 0x0D: snprintf(buf, buf_len, "RETE,%s", cc_names[r]); break;
        case 0x0E:
            fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
            snprintf(buf, buf_len, "BSTR,%s %s", cc_names[r], opnd);
            break;
        case 0x0F:
            snprintf(buf, buf_len, "BSTA,%s %s0x%04X", cc_names[r], bind, bra);
            break;
        case 0x10: snprintf(buf, buf_len, "HALT"); break; // only 0x40 reaches here
        case 0x14: snprintf(buf, buf_len, "RRR,R%d", r); break;
        case 0x15: snprintf(buf, buf_len, "REDE,R%d 0x%02X", r, byte2); break;
        case 0x16:
            fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
            snprintf(buf, buf_len, "BRNR,R%d %s", r, opnd);
            break;
        case 0x17:
            snprintf(buf, buf_len, "BRNA,R%d %s0x%04X", r, bind, bra);
            break;
        case 0x1C: snprintf(buf, buf_len, "REDD,R%d", r); break;
        case 0x1D: {
            static const char *ps_ops[4] = { "CPSU", "CPSL", "PPSU", "PPSL" };
            snprintf(buf, buf_len, "%s  0x%02X", ps_ops[r], byte2);
            break;
        }
        case 0x1E:
            fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
            snprintf(buf, buf_len, "BSNR,R%d %s", r, opnd);
            break;
        case 0x1F:
            snprintf(buf, buf_len, "BSNA,R%d %s0x%04X", r, bind, bra);
            break;
        case 0x24:
            if (op == 0x92) snprintf(buf, buf_len, "LPSU");
            else if (op == 0x93) snprintf(buf, buf_len, "LPSL");
            else snprintf(buf, buf_len, "INV   0x%02X", op);
            break;
        case 0x25: snprintf(buf, buf_len, "DAR,R%d", r); break;
        case 0x26:
            if (op == 0x9B) {
                snprintf(buf, buf_len, "ZBRR  %s0x%04X", (byte2 & 0x80) ? "*" : "",
                         (uint16_t)(sign7(byte2) & PAGE_MASK));
            } else {
                fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
                snprintf(buf, buf_len, "BCFR,%s %s", cc_names[r], opnd);
            }
            break;
        case 0x27:
            if (op == 0x9F) snprintf(buf, buf_len, "BXA   %s0x%04X,R3", bind, bra);
            else snprintf(buf, buf_len, "BCFA,%s %s0x%04X", cc_names[r], bind, bra);
            break;
        case 0x2C: snprintf(buf, buf_len, "WRTC,R%d", r); break;
        case 0x2D:
            if (op == 0xB4) snprintf(buf, buf_len, "TPSU  0x%02X", byte2);
            else if (op == 0xB5) snprintf(buf, buf_len, "TPSL  0x%02X", byte2);
            else snprintf(buf, buf_len, "INV   0x%02X", op);
            break;
        case 0x2E:
            if (op == 0xBB) {
                snprintf(buf, buf_len, "ZBSR  %s0x%04X", (byte2 & 0x80) ? "*" : "",
                         (uint16_t)(sign7(byte2) & PAGE_MASK));
            } else {
                fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
                snprintf(buf, buf_len, "BSFR,%s %s", cc_names[r], opnd);
            }
            break;
        case 0x2F:
            if (op == 0xBF) snprintf(buf, buf_len, "BSXA  %s0x%04X,R3", bind, bra);
            else snprintf(buf, buf_len, "BSFA,%s %s0x%04X", cc_names[r], bind, bra);
            break;
        case 0x30: snprintf(buf, buf_len, "NOP"); break; // only 0xC0 reaches here
        case 0x34: snprintf(buf, buf_len, "RRL,R%d", r); break;
        case 0x35: snprintf(buf, buf_len, "WRTE,R%d 0x%02X", r, byte2); break;
        case 0x36:
            fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
            snprintf(buf, buf_len, "BIRR,R%d %s", r, opnd);
            break;
        case 0x37:
            snprintf(buf, buf_len, "BIRA,R%d %s0x%04X", r, bind, bra);
            break;
        case 0x3C: snprintf(buf, buf_len, "WRTD,R%d", r); break;
        case 0x3D: snprintf(buf, buf_len, "TMI,R%d 0x%02X", r, byte2); break;
        case 0x3E:
            fmt_rel(opnd, sizeof(opnd), next_pc, byte2);
            snprintf(buf, buf_len, "BDRR,R%d %s", r, opnd);
            break;
        case 0x3F:
            snprintf(buf, buf_len, "BDRA,R%d %s0x%04X", r, bind, bra);
            break;
        default:
            snprintf(buf, buf_len, "INV   0x%02X", op);
            break;
    }
}
