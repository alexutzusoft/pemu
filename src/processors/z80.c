#include "z80.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB (16-bit address space)

// Flag bit positions in F register
#define FLAG_C  0x01 // Carry
#define FLAG_N  0x02 // Add/Subtract
#define FLAG_PV 0x04 // Parity/Overflow
#define FLAG_H  0x10 // Half Carry
#define FLAG_Z  0x40 // Zero
#define FLAG_S  0x80 // Sign

// Index addressing mode for DD/FD prefixes
#define IDX_HL 0
#define IDX_IX 1
#define IDX_IY 2

typedef struct Z80CPU {
    // Main register set
    uint8_t a, f, b, c, d, e, h, l;
    // Alternate register set
    uint8_t a2, f2, b2, c2, d2, e2, h2, l2;
    // Index registers (stored as byte halves for DD/FD half-register access)
    uint8_t ixh, ixl, iyh, iyl;
    uint16_t sp;
    uint16_t pc;
    uint8_t i, r;      // Interrupt vector / refresh
    uint8_t iff1, iff2; // Interrupt flip-flops
    uint8_t im;         // Interrupt mode

    uint8_t memory[MEM_SIZE];
    uint8_t ports[256];
    uint32_t ticks;
    int halted;
} Z80CPU;

static const char* cc_names[] = { "NZ", "Z", "NC", "C", "PO", "PE", "P", "M" };
static const char* rp_names[] = { "BC", "DE", "HL", "SP" };
static const char* rp2_names[] = { "BC", "DE", "HL", "AF" };
static const char* alu_names[] = { "ADD A,", "ADC A,", "SUB ", "SBC A,", "AND ", "XOR ", "OR ", "CP " };
static const char* cb_names[] = { "RLC", "RRC", "RL", "RR", "SLA", "SRA", "SLL", "SRL" };
static const char* r_names[] = { "B", "C", "D", "E", "H", "L", "(HL)", "A" };

// ---------------------------------------------------------------------------
// Basic helpers
// ---------------------------------------------------------------------------

static uint8_t parity8(uint8_t val) {
    uint8_t count = 0;
    for (int i = 0; i < 8; ++i) {
        if ((val >> i) & 1) count++;
    }
    return (count % 2 == 0) ? 1 : 0;
}

static uint8_t mem_rd(Z80CPU *cpu, uint16_t addr) {
    return cpu->memory[addr];
}

static void mem_wr(Z80CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->memory[addr] = val;
}

static uint16_t rd16(Z80CPU *cpu, uint16_t addr) {
    return (uint16_t)(mem_rd(cpu, addr) | ((uint16_t)mem_rd(cpu, (uint16_t)(addr + 1)) << 8));
}

static void wr16(Z80CPU *cpu, uint16_t addr, uint16_t val) {
    mem_wr(cpu, addr, (uint8_t)(val & 0xFF));
    mem_wr(cpu, (uint16_t)(addr + 1), (uint8_t)(val >> 8));
}

static uint8_t fetch8(Z80CPU *cpu) {
    uint8_t val = mem_rd(cpu, cpu->pc);
    cpu->pc = (uint16_t)(cpu->pc + 1);
    return val;
}

static uint16_t fetch16(Z80CPU *cpu) {
    uint8_t lo = fetch8(cpu);
    uint8_t hi = fetch8(cpu);
    return (uint16_t)(lo | ((uint16_t)hi << 8));
}

// ---------------------------------------------------------------------------
// Register pair access
// ---------------------------------------------------------------------------

static uint16_t get_bc(Z80CPU *cpu) { return (uint16_t)(((uint16_t)cpu->b << 8) | cpu->c); }
static uint16_t get_de(Z80CPU *cpu) { return (uint16_t)(((uint16_t)cpu->d << 8) | cpu->e); }
static uint16_t get_hl(Z80CPU *cpu) { return (uint16_t)(((uint16_t)cpu->h << 8) | cpu->l); }
static uint16_t get_af(Z80CPU *cpu) { return (uint16_t)(((uint16_t)cpu->a << 8) | cpu->f); }
static uint16_t get_ix(Z80CPU *cpu) { return (uint16_t)(((uint16_t)cpu->ixh << 8) | cpu->ixl); }
static uint16_t get_iy(Z80CPU *cpu) { return (uint16_t)(((uint16_t)cpu->iyh << 8) | cpu->iyl); }

static void set_bc(Z80CPU *cpu, uint16_t v) { cpu->b = (uint8_t)(v >> 8); cpu->c = (uint8_t)(v & 0xFF); }
static void set_de(Z80CPU *cpu, uint16_t v) { cpu->d = (uint8_t)(v >> 8); cpu->e = (uint8_t)(v & 0xFF); }
static void set_hl(Z80CPU *cpu, uint16_t v) { cpu->h = (uint8_t)(v >> 8); cpu->l = (uint8_t)(v & 0xFF); }
static void set_af(Z80CPU *cpu, uint16_t v) { cpu->a = (uint8_t)(v >> 8); cpu->f = (uint8_t)(v & 0xFF); }
static void set_ix(Z80CPU *cpu, uint16_t v) { cpu->ixh = (uint8_t)(v >> 8); cpu->ixl = (uint8_t)(v & 0xFF); }
static void set_iy(Z80CPU *cpu, uint16_t v) { cpu->iyh = (uint8_t)(v >> 8); cpu->iyl = (uint8_t)(v & 0xFF); }

// HL, IX or IY depending on active index mode
static uint16_t get_hlr(Z80CPU *cpu, int mode) {
    if (mode == IDX_IX) return get_ix(cpu);
    if (mode == IDX_IY) return get_iy(cpu);
    return get_hl(cpu);
}

static void set_hlr(Z80CPU *cpu, int mode, uint16_t v) {
    if (mode == IDX_IX) set_ix(cpu, v);
    else if (mode == IDX_IY) set_iy(cpu, v);
    else set_hl(cpu, v);
}

// Register pairs BC/DE/HL/SP (HL replaced by IX/IY under prefix)
static uint16_t get_rp(Z80CPU *cpu, int idx, int mode) {
    switch (idx) {
        case 0: return get_bc(cpu);
        case 1: return get_de(cpu);
        case 2: return get_hlr(cpu, mode);
        default: return cpu->sp;
    }
}

static void set_rp(Z80CPU *cpu, int idx, int mode, uint16_t v) {
    switch (idx) {
        case 0: set_bc(cpu, v); break;
        case 1: set_de(cpu, v); break;
        case 2: set_hlr(cpu, mode, v); break;
        default: cpu->sp = v; break;
    }
}

// Pointer to an 8-bit register by index (0=B..7=A, 6 is memory -> NULL).
// H and L map to IXH/IXL or IYH/IYL when an index mode is active.
static uint8_t* reg_ptr(Z80CPU *cpu, int idx, int mode) {
    switch (idx) {
        case 0: return &cpu->b;
        case 1: return &cpu->c;
        case 2: return &cpu->d;
        case 3: return &cpu->e;
        case 4: return (mode == IDX_IX) ? &cpu->ixh : (mode == IDX_IY) ? &cpu->iyh : &cpu->h;
        case 5: return (mode == IDX_IX) ? &cpu->ixl : (mode == IDX_IY) ? &cpu->iyl : &cpu->l;
        case 7: return &cpu->a;
        default: return NULL; // 6 = (HL)/(IX+d)/(IY+d)
    }
}

// Effective address of the memory operand. For IX/IY modes this fetches the
// signed displacement byte that follows the opcode.
static uint16_t mem_addr(Z80CPU *cpu, int mode) {
    if (mode == IDX_HL) return get_hl(cpu);
    int8_t d = (int8_t)fetch8(cpu);
    return (uint16_t)(get_hlr(cpu, mode) + d);
}

// ---------------------------------------------------------------------------
// Flag / ALU helpers
// ---------------------------------------------------------------------------

static uint8_t flags_szp(uint8_t res) {
    uint8_t f = 0;
    if (res & 0x80) f |= FLAG_S;
    if (res == 0) f |= FLAG_Z;
    if (parity8(res)) f |= FLAG_PV;
    return f;
}

static void alu_add8(Z80CPU *cpu, uint8_t v, uint8_t cy) {
    int res = cpu->a + v + cy;
    uint8_t r8 = (uint8_t)res;
    uint8_t f = 0;
    if (r8 & 0x80) f |= FLAG_S;
    if (r8 == 0) f |= FLAG_Z;
    if ((cpu->a ^ v ^ r8) & 0x10) f |= FLAG_H;
    if ((~(cpu->a ^ v) & (cpu->a ^ r8)) & 0x80) f |= FLAG_PV;
    if (res > 0xFF) f |= FLAG_C;
    cpu->a = r8;
    cpu->f = f;
}

static void alu_sub8(Z80CPU *cpu, uint8_t v, uint8_t cy, int store) {
    int res = cpu->a - v - cy;
    uint8_t r8 = (uint8_t)res;
    uint8_t f = FLAG_N;
    if (r8 & 0x80) f |= FLAG_S;
    if (r8 == 0) f |= FLAG_Z;
    if ((cpu->a ^ v ^ r8) & 0x10) f |= FLAG_H;
    if (((cpu->a ^ v) & (cpu->a ^ r8)) & 0x80) f |= FLAG_PV;
    if (res < 0) f |= FLAG_C;
    if (store) cpu->a = r8;
    cpu->f = f;
}

static void alu_logic(Z80CPU *cpu, uint8_t res, uint8_t set_h) {
    cpu->a = res;
    cpu->f = (uint8_t)(flags_szp(res) | (set_h ? FLAG_H : 0));
}

static uint8_t alu_inc8(Z80CPU *cpu, uint8_t v) {
    uint8_t res = (uint8_t)(v + 1);
    uint8_t f = (uint8_t)(cpu->f & FLAG_C);
    if (res & 0x80) f |= FLAG_S;
    if (res == 0) f |= FLAG_Z;
    if ((res & 0x0F) == 0x00) f |= FLAG_H;
    if (res == 0x80) f |= FLAG_PV;
    cpu->f = f;
    return res;
}

static uint8_t alu_dec8(Z80CPU *cpu, uint8_t v) {
    uint8_t res = (uint8_t)(v - 1);
    uint8_t f = (uint8_t)((cpu->f & FLAG_C) | FLAG_N);
    if (res & 0x80) f |= FLAG_S;
    if (res == 0) f |= FLAG_Z;
    if ((res & 0x0F) == 0x0F) f |= FLAG_H;
    if (res == 0x7F) f |= FLAG_PV;
    cpu->f = f;
    return res;
}

// ADD HL,rp (also ADD IX,rp / ADD IY,rp): only H, N, C affected
static uint16_t alu_add16(Z80CPU *cpu, uint16_t lhs, uint16_t rhs) {
    uint32_t res = (uint32_t)lhs + rhs;
    uint8_t f = (uint8_t)(cpu->f & (FLAG_S | FLAG_Z | FLAG_PV));
    if ((lhs ^ rhs ^ (uint16_t)res) & 0x1000) f |= FLAG_H;
    if (res > 0xFFFF) f |= FLAG_C;
    cpu->f = f;
    return (uint16_t)res;
}

// ED-prefixed ADC HL,rp: full flags on 16-bit result
static void alu_adc16(Z80CPU *cpu, uint16_t rhs) {
    uint16_t lhs = get_hl(cpu);
    uint32_t res = (uint32_t)lhs + rhs + (cpu->f & FLAG_C);
    uint16_t r16 = (uint16_t)res;
    uint8_t f = 0;
    if (r16 & 0x8000) f |= FLAG_S;
    if (r16 == 0) f |= FLAG_Z;
    if ((lhs ^ rhs ^ r16) & 0x1000) f |= FLAG_H;
    if ((~(lhs ^ rhs) & (lhs ^ r16)) & 0x8000) f |= FLAG_PV;
    if (res > 0xFFFF) f |= FLAG_C;
    set_hl(cpu, r16);
    cpu->f = f;
}

// ED-prefixed SBC HL,rp: full flags on 16-bit result
static void alu_sbc16(Z80CPU *cpu, uint16_t rhs) {
    uint16_t lhs = get_hl(cpu);
    int32_t res = (int32_t)lhs - rhs - (cpu->f & FLAG_C);
    uint16_t r16 = (uint16_t)res;
    uint8_t f = FLAG_N;
    if (r16 & 0x8000) f |= FLAG_S;
    if (r16 == 0) f |= FLAG_Z;
    if ((lhs ^ rhs ^ r16) & 0x1000) f |= FLAG_H;
    if (((lhs ^ rhs) & (lhs ^ r16)) & 0x8000) f |= FLAG_PV;
    if (res < 0) f |= FLAG_C;
    set_hl(cpu, r16);
    cpu->f = f;
}

static void alu_daa(Z80CPU *cpu) {
    uint8_t a = cpu->a;
    uint8_t adj = 0;
    uint8_t carry = (uint8_t)(cpu->f & FLAG_C);
    if ((cpu->f & FLAG_H) || (a & 0x0F) > 9) adj |= 0x06;
    if (carry || a > 0x99) { adj |= 0x60; carry = FLAG_C; }
    uint8_t res = (cpu->f & FLAG_N) ? (uint8_t)(a - adj) : (uint8_t)(a + adj);
    cpu->f = (uint8_t)((cpu->f & FLAG_N) | carry | ((a ^ res) & FLAG_H) | flags_szp(res));
    cpu->a = res;
}

static int check_cond(Z80CPU *cpu, int cond) {
    switch (cond) {
        case 0: return (cpu->f & FLAG_Z) == 0;  // NZ
        case 1: return (cpu->f & FLAG_Z) != 0;  // Z
        case 2: return (cpu->f & FLAG_C) == 0;  // NC
        case 3: return (cpu->f & FLAG_C) != 0;  // C
        case 4: return (cpu->f & FLAG_PV) == 0; // PO
        case 5: return (cpu->f & FLAG_PV) != 0; // PE
        case 6: return (cpu->f & FLAG_S) == 0;  // P
        default: return (cpu->f & FLAG_S) != 0; // M
    }
}

static void push16(Z80CPU *cpu, uint16_t val) {
    cpu->sp = (uint16_t)(cpu->sp - 2);
    wr16(cpu, cpu->sp, val);
}

static uint16_t pop16(Z80CPU *cpu) {
    uint16_t val = rd16(cpu, cpu->sp);
    cpu->sp = (uint16_t)(cpu->sp + 2);
    return val;
}

// ---------------------------------------------------------------------------
// CB-prefixed execution (rotates/shifts, BIT, RES, SET)
// ---------------------------------------------------------------------------

static uint8_t cb_rotate(Z80CPU *cpu, int rot_op, uint8_t v) {
    uint8_t res = 0;
    uint8_t carry = 0;
    switch (rot_op) {
        case 0: carry = (uint8_t)(v >> 7); res = (uint8_t)((v << 1) | carry); break;               // RLC
        case 1: carry = (uint8_t)(v & 1);  res = (uint8_t)((v >> 1) | (carry << 7)); break;         // RRC
        case 2: carry = (uint8_t)(v >> 7); res = (uint8_t)((v << 1) | (cpu->f & FLAG_C)); break;    // RL
        case 3: carry = (uint8_t)(v & 1);  res = (uint8_t)((v >> 1) | ((cpu->f & FLAG_C) << 7)); break; // RR
        case 4: carry = (uint8_t)(v >> 7); res = (uint8_t)(v << 1); break;                          // SLA
        case 5: carry = (uint8_t)(v & 1);  res = (uint8_t)((v >> 1) | (v & 0x80)); break;           // SRA
        case 6: carry = (uint8_t)(v >> 7); res = (uint8_t)((v << 1) | 1); break;                    // SLL (undoc)
        case 7: carry = (uint8_t)(v & 1);  res = (uint8_t)(v >> 1); break;                          // SRL
    }
    cpu->f = (uint8_t)(flags_szp(res) | (carry ? FLAG_C : 0));
    return res;
}

static void exec_cb(Z80CPU *cpu, int mode) {
    uint16_t addr = 0;
    uint8_t op;
    if (mode != IDX_HL) {
        // DD CB d op: displacement precedes the sub-opcode
        int8_t d = (int8_t)fetch8(cpu);
        addr = (uint16_t)(get_hlr(cpu, mode) + d);
        op = fetch8(cpu);
    } else {
        op = fetch8(cpu);
    }

    int group = (op >> 6) & 3;
    int bit = (op >> 3) & 7;
    int reg = op & 7;
    int use_mem = (mode != IDX_HL) || (reg == 6);
    if (mode == IDX_HL && reg == 6) addr = get_hl(cpu);

    uint8_t val = use_mem ? mem_rd(cpu, addr) : *reg_ptr(cpu, reg, IDX_HL);

    switch (group) {
        case 0: { // Rotates and shifts
            uint8_t res = cb_rotate(cpu, bit, val);
            if (use_mem) mem_wr(cpu, addr, res);
            else *reg_ptr(cpu, reg, IDX_HL) = res;
            break;
        }
        case 1: { // BIT b,r
            uint8_t masked = (uint8_t)(val & (1 << bit));
            uint8_t f = (uint8_t)((cpu->f & FLAG_C) | FLAG_H);
            if (masked == 0) f |= FLAG_Z | FLAG_PV;
            if (masked & 0x80) f |= FLAG_S;
            cpu->f = f;
            break;
        }
        case 2: { // RES b,r
            uint8_t res = (uint8_t)(val & ~(1 << bit));
            if (use_mem) mem_wr(cpu, addr, res);
            else *reg_ptr(cpu, reg, IDX_HL) = res;
            break;
        }
        default: { // SET b,r
            uint8_t res = (uint8_t)(val | (1 << bit));
            if (use_mem) mem_wr(cpu, addr, res);
            else *reg_ptr(cpu, reg, IDX_HL) = res;
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// ED-prefixed execution
// ---------------------------------------------------------------------------

static void exec_ed(Z80CPU *cpu) {
    uint8_t op = fetch8(cpu);

    if ((op & 0xCF) == 0x42) { // SBC HL,rp
        alu_sbc16(cpu, get_rp(cpu, (op >> 4) & 3, IDX_HL));
    }
    else if ((op & 0xCF) == 0x4A) { // ADC HL,rp
        alu_adc16(cpu, get_rp(cpu, (op >> 4) & 3, IDX_HL));
    }
    else if ((op & 0xCF) == 0x43) { // LD (nn),rp
        wr16(cpu, fetch16(cpu), get_rp(cpu, (op >> 4) & 3, IDX_HL));
    }
    else if ((op & 0xCF) == 0x4B) { // LD rp,(nn)
        set_rp(cpu, (op >> 4) & 3, IDX_HL, rd16(cpu, fetch16(cpu)));
    }
    else if ((op & 0xC7) == 0x44) { // NEG (and mirrors)
        uint8_t a = cpu->a;
        cpu->a = 0;
        alu_sub8(cpu, a, 0, 1);
    }
    else if ((op & 0xC7) == 0x45) { // RETN / RETI (and mirrors)
        cpu->iff1 = cpu->iff2;
        cpu->pc = pop16(cpu);
    }
    else if ((op & 0xC7) == 0x46) { // IM 0/1/2 (and mirrors)
        int sel = (op >> 3) & 3;
        cpu->im = (sel < 2) ? 0 : (uint8_t)(sel - 1);
    }
    else if (op == 0x47) { // LD I,A
        cpu->i = cpu->a;
    }
    else if (op == 0x4F) { // LD R,A
        cpu->r = cpu->a;
    }
    else if (op == 0x57 || op == 0x5F) { // LD A,I / LD A,R
        cpu->a = (op == 0x57) ? cpu->i : cpu->r;
        uint8_t f = (uint8_t)(cpu->f & FLAG_C);
        if (cpu->a & 0x80) f |= FLAG_S;
        if (cpu->a == 0) f |= FLAG_Z;
        if (cpu->iff2) f |= FLAG_PV;
        cpu->f = f;
    }
    else if (op == 0xA0 || op == 0xA8 || op == 0xB0 || op == 0xB8) {
        // LDI / LDD / LDIR / LDDR
        int step = (op & 0x08) ? -1 : 1;
        uint16_t hl = get_hl(cpu);
        uint16_t de = get_de(cpu);
        uint16_t bc = get_bc(cpu);
        mem_wr(cpu, de, mem_rd(cpu, hl));
        set_hl(cpu, (uint16_t)(hl + step));
        set_de(cpu, (uint16_t)(de + step));
        bc = (uint16_t)(bc - 1);
        set_bc(cpu, bc);
        uint8_t f = (uint8_t)(cpu->f & (FLAG_S | FLAG_Z | FLAG_C));
        if (bc != 0) f |= FLAG_PV;
        cpu->f = f;
        if ((op & 0x10) && bc != 0) {
            cpu->pc = (uint16_t)(cpu->pc - 2); // Repeat (LDIR/LDDR)
        }
    }
    else if ((op & 0xC7) == 0x40) { // IN r,(C)
        int reg = (op >> 3) & 7;
        uint8_t val = cpu->ports[cpu->c];
        if (reg != 6) *reg_ptr(cpu, reg, IDX_HL) = val;
        cpu->f = (uint8_t)((cpu->f & FLAG_C) | flags_szp(val));
    }
    else if ((op & 0xC7) == 0x41) { // OUT (C),r
        int reg = (op >> 3) & 7;
        cpu->ports[cpu->c] = (reg == 6) ? 0 : *reg_ptr(cpu, reg, IDX_HL);
    }
    // All other ED opcodes: treated as NOP
}

// ---------------------------------------------------------------------------
// Main (unprefixed / DD / FD) execution. Returns 1 if HALT was executed.
// ---------------------------------------------------------------------------

static int exec_main(Z80CPU *cpu, uint8_t op, int mode) {
    if (op == 0x76) { // HALT
        cpu->halted = 1;
        return 1;
    }

    if ((op & 0xC0) == 0x40) { // LD r,r'
        int dst = (op >> 3) & 7;
        int src = op & 7;
        if (dst == 6) {
            uint16_t addr = mem_addr(cpu, mode);
            mem_wr(cpu, addr, *reg_ptr(cpu, src, IDX_HL));
        } else if (src == 6) {
            uint16_t addr = mem_addr(cpu, mode);
            *reg_ptr(cpu, dst, IDX_HL) = mem_rd(cpu, addr);
        } else {
            *reg_ptr(cpu, dst, mode) = *reg_ptr(cpu, src, mode);
        }
        return 0;
    }

    if ((op & 0xC0) == 0x80) { // ALU A,r
        int alu_op = (op >> 3) & 7;
        int src = op & 7;
        uint8_t val = (src == 6) ? mem_rd(cpu, mem_addr(cpu, mode)) : *reg_ptr(cpu, src, mode);
        switch (alu_op) {
            case 0: alu_add8(cpu, val, 0); break;
            case 1: alu_add8(cpu, val, (uint8_t)(cpu->f & FLAG_C)); break;
            case 2: alu_sub8(cpu, val, 0, 1); break;
            case 3: alu_sub8(cpu, val, (uint8_t)(cpu->f & FLAG_C), 1); break;
            case 4: alu_logic(cpu, (uint8_t)(cpu->a & val), 1); break;
            case 5: alu_logic(cpu, (uint8_t)(cpu->a ^ val), 0); break;
            case 6: alu_logic(cpu, (uint8_t)(cpu->a | val), 0); break;
            case 7: alu_sub8(cpu, val, 0, 0); break; // CP
        }
        return 0;
    }

    if ((op & 0xC0) == 0x00) { // 0x00 - 0x3F block
        switch (op & 0x07) {
            case 0x00:
                switch (op) {
                    case 0x00: break; // NOP
                    case 0x08: { // EX AF,AF'
                        uint8_t t;
                        t = cpu->a; cpu->a = cpu->a2; cpu->a2 = t;
                        t = cpu->f; cpu->f = cpu->f2; cpu->f2 = t;
                        break;
                    }
                    case 0x10: { // DJNZ e
                        int8_t off = (int8_t)fetch8(cpu);
                        cpu->b = (uint8_t)(cpu->b - 1);
                        if (cpu->b != 0) cpu->pc = (uint16_t)(cpu->pc + off);
                        break;
                    }
                    case 0x18: { // JR e
                        int8_t off = (int8_t)fetch8(cpu);
                        cpu->pc = (uint16_t)(cpu->pc + off);
                        break;
                    }
                    default: { // JR cc,e (0x20/0x28/0x30/0x38)
                        int8_t off = (int8_t)fetch8(cpu);
                        if (check_cond(cpu, ((op >> 3) & 7) - 4)) {
                            cpu->pc = (uint16_t)(cpu->pc + off);
                        }
                        break;
                    }
                }
                break;
            case 0x01:
                if (op & 0x08) { // ADD HL,rp
                    set_hlr(cpu, mode, alu_add16(cpu, get_hlr(cpu, mode), get_rp(cpu, (op >> 4) & 3, mode)));
                } else {         // LD rp,nn
                    set_rp(cpu, (op >> 4) & 3, mode, fetch16(cpu));
                }
                break;
            case 0x02:
                switch (op) {
                    case 0x02: mem_wr(cpu, get_bc(cpu), cpu->a); break;         // LD (BC),A
                    case 0x0A: cpu->a = mem_rd(cpu, get_bc(cpu)); break;        // LD A,(BC)
                    case 0x12: mem_wr(cpu, get_de(cpu), cpu->a); break;         // LD (DE),A
                    case 0x1A: cpu->a = mem_rd(cpu, get_de(cpu)); break;        // LD A,(DE)
                    case 0x22: wr16(cpu, fetch16(cpu), get_hlr(cpu, mode)); break;      // LD (nn),HL
                    case 0x2A: set_hlr(cpu, mode, rd16(cpu, fetch16(cpu))); break;      // LD HL,(nn)
                    case 0x32: mem_wr(cpu, fetch16(cpu), cpu->a); break;        // LD (nn),A
                    case 0x3A: cpu->a = mem_rd(cpu, fetch16(cpu)); break;       // LD A,(nn)
                }
                break;
            case 0x03: { // INC rp / DEC rp
                int rp = (op >> 4) & 3;
                uint16_t v = get_rp(cpu, rp, mode);
                set_rp(cpu, rp, mode, (uint16_t)((op & 0x08) ? v - 1 : v + 1));
                break;
            }
            case 0x04: { // INC r
                int reg = (op >> 3) & 7;
                if (reg == 6) {
                    uint16_t addr = mem_addr(cpu, mode);
                    mem_wr(cpu, addr, alu_inc8(cpu, mem_rd(cpu, addr)));
                } else {
                    uint8_t *p = reg_ptr(cpu, reg, mode);
                    *p = alu_inc8(cpu, *p);
                }
                break;
            }
            case 0x05: { // DEC r
                int reg = (op >> 3) & 7;
                if (reg == 6) {
                    uint16_t addr = mem_addr(cpu, mode);
                    mem_wr(cpu, addr, alu_dec8(cpu, mem_rd(cpu, addr)));
                } else {
                    uint8_t *p = reg_ptr(cpu, reg, mode);
                    *p = alu_dec8(cpu, *p);
                }
                break;
            }
            case 0x06: { // LD r,n
                int reg = (op >> 3) & 7;
                if (reg == 6) {
                    uint16_t addr = mem_addr(cpu, mode);
                    mem_wr(cpu, addr, fetch8(cpu));
                } else {
                    *reg_ptr(cpu, reg, mode) = fetch8(cpu);
                }
                break;
            }
            case 0x07:
                switch (op) {
                    case 0x07: { // RLCA
                        uint8_t carry = (uint8_t)(cpu->a >> 7);
                        cpu->a = (uint8_t)((cpu->a << 1) | carry);
                        cpu->f = (uint8_t)((cpu->f & (FLAG_S | FLAG_Z | FLAG_PV)) | carry);
                        break;
                    }
                    case 0x0F: { // RRCA
                        uint8_t carry = (uint8_t)(cpu->a & 1);
                        cpu->a = (uint8_t)((cpu->a >> 1) | (carry << 7));
                        cpu->f = (uint8_t)((cpu->f & (FLAG_S | FLAG_Z | FLAG_PV)) | carry);
                        break;
                    }
                    case 0x17: { // RLA
                        uint8_t carry = (uint8_t)(cpu->a >> 7);
                        cpu->a = (uint8_t)((cpu->a << 1) | (cpu->f & FLAG_C));
                        cpu->f = (uint8_t)((cpu->f & (FLAG_S | FLAG_Z | FLAG_PV)) | carry);
                        break;
                    }
                    case 0x1F: { // RRA
                        uint8_t carry = (uint8_t)(cpu->a & 1);
                        cpu->a = (uint8_t)((cpu->a >> 1) | ((cpu->f & FLAG_C) << 7));
                        cpu->f = (uint8_t)((cpu->f & (FLAG_S | FLAG_Z | FLAG_PV)) | carry);
                        break;
                    }
                    case 0x27: alu_daa(cpu); break; // DAA
                    case 0x2F: // CPL
                        cpu->a = (uint8_t)~cpu->a;
                        cpu->f = (uint8_t)(cpu->f | FLAG_H | FLAG_N);
                        break;
                    case 0x37: // SCF
                        cpu->f = (uint8_t)((cpu->f & (FLAG_S | FLAG_Z | FLAG_PV)) | FLAG_C);
                        break;
                    case 0x3F: { // CCF
                        uint8_t old_c = (uint8_t)(cpu->f & FLAG_C);
                        cpu->f = (uint8_t)((cpu->f & (FLAG_S | FLAG_Z | FLAG_PV)) |
                                           (old_c ? FLAG_H : FLAG_C));
                        break;
                    }
                }
                break;
        }
        return 0;
    }

    // 0xC0 - 0xFF block
    switch (op & 0x07) {
        case 0x00: // RET cc
            if (check_cond(cpu, (op >> 3) & 7)) cpu->pc = pop16(cpu);
            break;
        case 0x01:
            switch (op) {
                case 0xC9: cpu->pc = pop16(cpu); break;              // RET
                case 0xD9: { // EXX
                    uint8_t t;
                    t = cpu->b; cpu->b = cpu->b2; cpu->b2 = t;
                    t = cpu->c; cpu->c = cpu->c2; cpu->c2 = t;
                    t = cpu->d; cpu->d = cpu->d2; cpu->d2 = t;
                    t = cpu->e; cpu->e = cpu->e2; cpu->e2 = t;
                    t = cpu->h; cpu->h = cpu->h2; cpu->h2 = t;
                    t = cpu->l; cpu->l = cpu->l2; cpu->l2 = t;
                    break;
                }
                case 0xE9: cpu->pc = get_hlr(cpu, mode); break;      // JP (HL)
                case 0xF9: cpu->sp = get_hlr(cpu, mode); break;      // LD SP,HL
                default: { // POP rp2 (0xC1/0xD1/0xE1/0xF1)
                    int rp = (op >> 4) & 3;
                    uint16_t v = pop16(cpu);
                    if (rp == 3) set_af(cpu, v);
                    else set_rp(cpu, rp, mode, v);
                    break;
                }
            }
            break;
        case 0x02: { // JP cc,nn
            uint16_t addr = fetch16(cpu);
            if (check_cond(cpu, (op >> 3) & 7)) cpu->pc = addr;
            break;
        }
        case 0x03:
            switch (op) {
                case 0xC3: cpu->pc = fetch16(cpu); break;            // JP nn
                case 0xD3: cpu->ports[fetch8(cpu)] = cpu->a; break;  // OUT (n),A
                case 0xDB: cpu->a = cpu->ports[fetch8(cpu)]; break;  // IN A,(n)
                case 0xE3: { // EX (SP),HL
                    uint16_t t = rd16(cpu, cpu->sp);
                    wr16(cpu, cpu->sp, get_hlr(cpu, mode));
                    set_hlr(cpu, mode, t);
                    break;
                }
                case 0xEB: { // EX DE,HL (not affected by DD/FD)
                    uint16_t t = get_de(cpu);
                    set_de(cpu, get_hl(cpu));
                    set_hl(cpu, t);
                    break;
                }
                case 0xF3: cpu->iff1 = cpu->iff2 = 0; break;         // DI
                case 0xFB: cpu->iff1 = cpu->iff2 = 1; break;         // EI
            }
            break;
        case 0x04: { // CALL cc,nn
            uint16_t addr = fetch16(cpu);
            if (check_cond(cpu, (op >> 3) & 7)) {
                push16(cpu, cpu->pc);
                cpu->pc = addr;
            }
            break;
        }
        case 0x05:
            if (op == 0xCD) { // CALL nn
                uint16_t addr = fetch16(cpu);
                push16(cpu, cpu->pc);
                cpu->pc = addr;
            } else { // PUSH rp2 (0xC5/0xD5/0xE5/0xF5)
                int rp = (op >> 4) & 3;
                push16(cpu, (rp == 3) ? get_af(cpu) : get_rp(cpu, rp, mode));
            }
            break;
        case 0x06: { // ALU A,n
            int alu_op = (op >> 3) & 7;
            uint8_t val = fetch8(cpu);
            switch (alu_op) {
                case 0: alu_add8(cpu, val, 0); break;
                case 1: alu_add8(cpu, val, (uint8_t)(cpu->f & FLAG_C)); break;
                case 2: alu_sub8(cpu, val, 0, 1); break;
                case 3: alu_sub8(cpu, val, (uint8_t)(cpu->f & FLAG_C), 1); break;
                case 4: alu_logic(cpu, (uint8_t)(cpu->a & val), 1); break;
                case 5: alu_logic(cpu, (uint8_t)(cpu->a ^ val), 0); break;
                case 6: alu_logic(cpu, (uint8_t)(cpu->a | val), 0); break;
                case 7: alu_sub8(cpu, val, 0, 0); break; // CP
            }
            break;
        }
        case 0x07: // RST n
            push16(cpu, cpu->pc);
            cpu->pc = (uint16_t)(((op >> 3) & 7) * 8);
            break;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void* z80_create(void) {
    Z80CPU *cpu = (Z80CPU*)calloc(1, sizeof(Z80CPU));
    return cpu;
}

void z80_destroy(void *context) {
    free(context);
}

int z80_init(void *context) {
    if (!context) return -1;
    Z80CPU *cpu = (Z80CPU*)context;
    memset(cpu, 0, sizeof(Z80CPU));
    cpu->sp = 0xFFFF;
    return 0;
}

int z80_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    Z80CPU *cpu = (Z80CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int z80_step(void *context) {
    if (!context) return -1;
    Z80CPU *cpu = (Z80CPU*)context;

    if (cpu->halted) return 1;

    cpu->ticks++;
    cpu->r = (uint8_t)((cpu->r & 0x80) | ((cpu->r + 1) & 0x7F));

    // Consume DD/FD prefixes (last one wins)
    int mode = IDX_HL;
    uint8_t op = fetch8(cpu);
    while (op == 0xDD || op == 0xFD) {
        mode = (op == 0xDD) ? IDX_IX : IDX_IY;
        op = fetch8(cpu);
    }

    if (op == 0xCB) {
        exec_cb(cpu, mode);
        return 0;
    }
    if (op == 0xED) {
        exec_ed(cpu);
        return 0;
    }
    return exec_main(cpu, op, mode);
}

void z80_print_state(void *context) {
    if (!context) return;
    Z80CPU *cpu = (Z80CPU*)context;

    printf("Zilog Z80 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  A: 0x%02X  F: 0x%02X  [S=%d Z=%d H=%d P/V=%d N=%d C=%d]\n",
           cpu->a, cpu->f,
           (cpu->f & FLAG_S) ? 1 : 0, (cpu->f & FLAG_Z) ? 1 : 0,
           (cpu->f & FLAG_H) ? 1 : 0, (cpu->f & FLAG_PV) ? 1 : 0,
           (cpu->f & FLAG_N) ? 1 : 0, (cpu->f & FLAG_C) ? 1 : 0);
    printf("  B: 0x%02X  C: 0x%02X  D: 0x%02X  E: 0x%02X  H: 0x%02X  L: 0x%02X\n",
           cpu->b, cpu->c, cpu->d, cpu->e, cpu->h, cpu->l);
    printf("  IX: 0x%04X  IY: 0x%04X  I: 0x%02X  R: 0x%02X  IM: %d  IFF1: %d\n",
           get_ix(cpu), get_iy(cpu), cpu->i, cpu->r, cpu->im, cpu->iff1);
    printf("  AF': 0x%04X  BC': 0x%04X  DE': 0x%04X  HL': 0x%04X\n",
           ((uint16_t)cpu->a2 << 8) | cpu->f2, ((uint16_t)cpu->b2 << 8) | cpu->c2,
           ((uint16_t)cpu->d2 << 8) | cpu->e2, ((uint16_t)cpu->h2 << 8) | cpu->l2);
    printf("  (HL): 0x%02X  (SP): 0x%04X\n",
           cpu->memory[get_hl(cpu)], rd16(cpu, cpu->sp));
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

// Format the name of 8-bit register 'idx' honoring the index mode.
// 'd' is the displacement for (IX+d)/(IY+d); use_d indicates it applies.
static void dis_reg(char *out, size_t out_len, int idx, int mode, int8_t d) {
    if (idx == 6 && mode != IDX_HL) {
        snprintf(out, out_len, "(%s%+d)", (mode == IDX_IX) ? "IX" : "IY", d);
    } else if ((idx == 4 || idx == 5) && mode != IDX_HL) {
        snprintf(out, out_len, "%s%s", (mode == IDX_IX) ? "IX" : "IY", (idx == 4) ? "H" : "L");
    } else {
        snprintf(out, out_len, "%s", r_names[idx]);
    }
}

static const char* dis_rp(int idx, int mode) {
    if (idx == 2) {
        if (mode == IDX_IX) return "IX";
        if (mode == IDX_IY) return "IY";
    }
    return rp_names[idx];
}

void z80_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    Z80CPU *cpu = (Z80CPU*)context;

    uint16_t pc = cpu->pc;
    int mode = IDX_HL;
    uint8_t op = mem_rd(cpu, pc);
    int prefixes = 0;
    while ((op == 0xDD || op == 0xFD) && prefixes < 4) {
        mode = (op == 0xDD) ? IDX_IX : IDX_IY;
        pc = (uint16_t)(pc + 1);
        op = mem_rd(cpu, pc);
        prefixes++;
    }

    uint8_t b1 = mem_rd(cpu, (uint16_t)(pc + 1));
    uint8_t b2 = mem_rd(cpu, (uint16_t)(pc + 2));
    uint16_t nn = (uint16_t)(b1 | ((uint16_t)b2 << 8));
    const char *ir = (mode == IDX_IX) ? "IX" : (mode == IDX_IY) ? "IY" : "HL";
    char rs[16], rd[16];

    if (op == 0xCB) {
        uint8_t sub;
        int8_t d = 0;
        if (mode != IDX_HL) { d = (int8_t)b1; sub = b2; }
        else sub = b1;
        int group = (sub >> 6) & 3;
        int bit = (sub >> 3) & 7;
        dis_reg(rs, sizeof(rs), sub & 7, mode, d);
        if (group == 0) snprintf(buf, buf_len, "%-5s %s", cb_names[bit], rs);
        else snprintf(buf, buf_len, "%-5s %d, %s",
                      (group == 1) ? "BIT" : (group == 2) ? "RES" : "SET", bit, rs);
        return;
    }

    if (op == 0xED) {
        uint8_t sub = b1;
        uint16_t ed_nn = (uint16_t)(b2 | ((uint16_t)mem_rd(cpu, (uint16_t)(pc + 3)) << 8));
        if ((sub & 0xCF) == 0x42) snprintf(buf, buf_len, "SBC   HL, %s", rp_names[(sub >> 4) & 3]);
        else if ((sub & 0xCF) == 0x4A) snprintf(buf, buf_len, "ADC   HL, %s", rp_names[(sub >> 4) & 3]);
        else if ((sub & 0xCF) == 0x43) snprintf(buf, buf_len, "LD    (0x%04X), %s", ed_nn, rp_names[(sub >> 4) & 3]);
        else if ((sub & 0xCF) == 0x4B) snprintf(buf, buf_len, "LD    %s, (0x%04X)", rp_names[(sub >> 4) & 3], ed_nn);
        else if ((sub & 0xC7) == 0x44) snprintf(buf, buf_len, "NEG");
        else if (sub == 0x4D) snprintf(buf, buf_len, "RETI");
        else if ((sub & 0xC7) == 0x45) snprintf(buf, buf_len, "RETN");
        else if ((sub & 0xC7) == 0x46) {
            int sel = (sub >> 3) & 3;
            snprintf(buf, buf_len, "IM    %d", (sel < 2) ? 0 : sel - 1);
        }
        else if (sub == 0x47) snprintf(buf, buf_len, "LD    I, A");
        else if (sub == 0x4F) snprintf(buf, buf_len, "LD    R, A");
        else if (sub == 0x57) snprintf(buf, buf_len, "LD    A, I");
        else if (sub == 0x5F) snprintf(buf, buf_len, "LD    A, R");
        else if (sub == 0xA0) snprintf(buf, buf_len, "LDI");
        else if (sub == 0xA8) snprintf(buf, buf_len, "LDD");
        else if (sub == 0xB0) snprintf(buf, buf_len, "LDIR");
        else if (sub == 0xB8) snprintf(buf, buf_len, "LDDR");
        else if ((sub & 0xC7) == 0x40) snprintf(buf, buf_len, "IN    %s, (C)", r_names[(sub >> 3) & 7]);
        else if ((sub & 0xC7) == 0x41) snprintf(buf, buf_len, "OUT   (C), %s", r_names[(sub >> 3) & 7]);
        else snprintf(buf, buf_len, "ED    0x%02X (NOP)", sub);
        return;
    }

    if (op == 0x76) { snprintf(buf, buf_len, "HALT"); return; }

    if ((op & 0xC0) == 0x40) { // LD r,r'
        int dst = (op >> 3) & 7;
        int src = op & 7;
        int mem = (dst == 6 || src == 6);
        dis_reg(rd, sizeof(rd), dst, mem ? mode : mode * (dst == 4 || dst == 5), (int8_t)b1);
        dis_reg(rs, sizeof(rs), src, mem ? mode : mode * (src == 4 || src == 5), (int8_t)b1);
        if (mem && mode != IDX_HL) {
            // Non-memory operand is a plain register under an index prefix
            if (dst != 6) snprintf(rd, sizeof(rd), "%s", r_names[dst]);
            if (src != 6) snprintf(rs, sizeof(rs), "%s", r_names[src]);
        }
        snprintf(buf, buf_len, "LD    %s, %s", rd, rs);
        return;
    }

    if ((op & 0xC0) == 0x80) { // ALU A,r
        dis_reg(rs, sizeof(rs), op & 7, mode, (int8_t)b1);
        snprintf(buf, buf_len, "%-6s%s", alu_names[(op >> 3) & 7], rs);
        return;
    }

    if ((op & 0xC0) == 0x00) {
        switch (op & 0x07) {
            case 0x00:
                if (op == 0x00) snprintf(buf, buf_len, "NOP");
                else if (op == 0x08) snprintf(buf, buf_len, "EX    AF, AF'");
                else if (op == 0x10) snprintf(buf, buf_len, "DJNZ  0x%04X", (uint16_t)(pc + 2 + (int8_t)b1));
                else if (op == 0x18) snprintf(buf, buf_len, "JR    0x%04X", (uint16_t)(pc + 2 + (int8_t)b1));
                else snprintf(buf, buf_len, "JR    %s, 0x%04X", cc_names[((op >> 3) & 7) - 4],
                              (uint16_t)(pc + 2 + (int8_t)b1));
                return;
            case 0x01:
                if (op & 0x08) snprintf(buf, buf_len, "ADD   %s, %s", ir, dis_rp((op >> 4) & 3, mode));
                else snprintf(buf, buf_len, "LD    %s, 0x%04X", dis_rp((op >> 4) & 3, mode), nn);
                return;
            case 0x02:
                switch (op) {
                    case 0x02: snprintf(buf, buf_len, "LD    (BC), A"); break;
                    case 0x0A: snprintf(buf, buf_len, "LD    A, (BC)"); break;
                    case 0x12: snprintf(buf, buf_len, "LD    (DE), A"); break;
                    case 0x1A: snprintf(buf, buf_len, "LD    A, (DE)"); break;
                    case 0x22: snprintf(buf, buf_len, "LD    (0x%04X), %s", nn, ir); break;
                    case 0x2A: snprintf(buf, buf_len, "LD    %s, (0x%04X)", ir, nn); break;
                    case 0x32: snprintf(buf, buf_len, "LD    (0x%04X), A", nn); break;
                    case 0x3A: snprintf(buf, buf_len, "LD    A, (0x%04X)", nn); break;
                }
                return;
            case 0x03:
                snprintf(buf, buf_len, "%-5s %s", (op & 0x08) ? "DEC" : "INC", dis_rp((op >> 4) & 3, mode));
                return;
            case 0x04:
            case 0x05:
                dis_reg(rs, sizeof(rs), (op >> 3) & 7, mode, (int8_t)b1);
                snprintf(buf, buf_len, "%-5s %s", ((op & 0x07) == 0x04) ? "INC" : "DEC", rs);
                return;
            case 0x06: {
                int reg = (op >> 3) & 7;
                dis_reg(rs, sizeof(rs), reg, mode, (int8_t)b1);
                snprintf(buf, buf_len, "LD    %s, 0x%02X", rs,
                         (reg == 6 && mode != IDX_HL) ? b2 : b1);
                return;
            }
            default: {
                const char* misc[] = { "RLCA", "RRCA", "RLA", "RRA", "DAA", "CPL", "SCF", "CCF" };
                snprintf(buf, buf_len, "%s", misc[(op >> 3) & 7]);
                return;
            }
        }
    }

    // 0xC0 - 0xFF
    switch (op & 0x07) {
        case 0x00: snprintf(buf, buf_len, "RET   %s", cc_names[(op >> 3) & 7]); return;
        case 0x01:
            if (op == 0xC9) snprintf(buf, buf_len, "RET");
            else if (op == 0xD9) snprintf(buf, buf_len, "EXX");
            else if (op == 0xE9) snprintf(buf, buf_len, "JP    (%s)", ir);
            else if (op == 0xF9) snprintf(buf, buf_len, "LD    SP, %s", ir);
            else snprintf(buf, buf_len, "POP   %s",
                          (((op >> 4) & 3) == 2) ? ir : rp2_names[(op >> 4) & 3]);
            return;
        case 0x02: snprintf(buf, buf_len, "JP    %s, 0x%04X", cc_names[(op >> 3) & 7], nn); return;
        case 0x03:
            if (op == 0xC3) snprintf(buf, buf_len, "JP    0x%04X", nn);
            else if (op == 0xD3) snprintf(buf, buf_len, "OUT   (0x%02X), A", b1);
            else if (op == 0xDB) snprintf(buf, buf_len, "IN    A, (0x%02X)", b1);
            else if (op == 0xE3) snprintf(buf, buf_len, "EX    (SP), %s", ir);
            else if (op == 0xEB) snprintf(buf, buf_len, "EX    DE, HL");
            else if (op == 0xF3) snprintf(buf, buf_len, "DI");
            else snprintf(buf, buf_len, "EI");
            return;
        case 0x04: snprintf(buf, buf_len, "CALL  %s, 0x%04X", cc_names[(op >> 3) & 7], nn); return;
        case 0x05:
            if (op == 0xCD) snprintf(buf, buf_len, "CALL  0x%04X", nn);
            else snprintf(buf, buf_len, "PUSH  %s",
                          (((op >> 4) & 3) == 2) ? ir : rp2_names[(op >> 4) & 3]);
            return;
        case 0x06: snprintf(buf, buf_len, "%-6s0x%02X", alu_names[(op >> 3) & 7], b1); return;
        default: snprintf(buf, buf_len, "RST   0x%02X", ((op >> 3) & 7) * 8); return;
    }
}
