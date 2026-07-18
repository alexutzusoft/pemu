#include "hd6309.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Condition Code register bits (E F H I N Z V C) */
#define CC_C 0x01
#define CC_V 0x02
#define CC_Z 0x04
#define CC_N 0x08
#define CC_I 0x10
#define CC_H 0x20
#define CC_F 0x40
#define CC_E 0x80

/* Mode register bits */
#define MD_NM 0x01 /* native mode */
#define MD_FM 0x02 /* FIRQ stacks entire state */
#define MD_IL 0x40 /* illegal instruction encountered */
#define MD_DZ 0x80 /* division by zero */

typedef struct HD6309_CPU {
    uint8_t ram[65536];
    uint8_t a;
    uint8_t b;
    uint8_t e;
    uint8_t f;
    uint8_t dp;
    uint8_t cc;
    uint8_t md;
    uint16_t v;
    uint16_t x;
    uint16_t y;
    uint16_t u;
    uint16_t s;
    uint16_t pc;
    uint32_t ticks;
    int halted;
} HD6309_CPU;

#define SET_FLAG(cond, flag) do { if (cond) cpu->cc |= (flag); else cpu->cc &= ~(uint8_t)(flag); } while (0)
#define GET_FLAG(flag) ((cpu->cc & (flag)) ? 1 : 0)

/* ---------------------------------------------------------------- memory */

static uint8_t mem_read(HD6309_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr];
}

static void mem_write(HD6309_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr] = val;
}

static uint16_t mem_read16(HD6309_CPU *cpu, uint16_t addr) {
    return (uint16_t)(((uint16_t)mem_read(cpu, addr) << 8) | mem_read(cpu, (uint16_t)(addr + 1)));
}

static void mem_write16(HD6309_CPU *cpu, uint16_t addr, uint16_t val) {
    mem_write(cpu, addr, (uint8_t)(val >> 8));
    mem_write(cpu, (uint16_t)(addr + 1), (uint8_t)(val & 0xFF));
}

static uint32_t mem_read32(HD6309_CPU *cpu, uint16_t addr) {
    return ((uint32_t)mem_read16(cpu, addr) << 16) | mem_read16(cpu, (uint16_t)(addr + 2));
}

static void mem_write32(HD6309_CPU *cpu, uint16_t addr, uint32_t val) {
    mem_write16(cpu, addr, (uint16_t)(val >> 16));
    mem_write16(cpu, (uint16_t)(addr + 2), (uint16_t)(val & 0xFFFF));
}

static uint8_t fetch8(HD6309_CPU *cpu) {
    uint8_t v = mem_read(cpu, cpu->pc);
    cpu->pc++;
    return v;
}

static uint16_t fetch16(HD6309_CPU *cpu) {
    uint16_t v = mem_read16(cpu, cpu->pc);
    cpu->pc += 2;
    return v;
}

static uint16_t get_d(HD6309_CPU *cpu) {
    return (uint16_t)(((uint16_t)cpu->a << 8) | cpu->b);
}

static void set_d(HD6309_CPU *cpu, uint16_t val) {
    cpu->a = (uint8_t)(val >> 8);
    cpu->b = (uint8_t)(val & 0xFF);
}

static uint16_t get_w(HD6309_CPU *cpu) {
    return (uint16_t)(((uint16_t)cpu->e << 8) | cpu->f);
}

static void set_w(HD6309_CPU *cpu, uint16_t val) {
    cpu->e = (uint8_t)(val >> 8);
    cpu->f = (uint8_t)(val & 0xFF);
}

static uint32_t get_q(HD6309_CPU *cpu) {
    return ((uint32_t)get_d(cpu) << 16) | get_w(cpu);
}

static void set_q(HD6309_CPU *cpu, uint32_t val) {
    set_d(cpu, (uint16_t)(val >> 16));
    set_w(cpu, (uint16_t)(val & 0xFFFF));
}

/* ---------------------------------------------------------------- stacks */

static void push8_s(HD6309_CPU *cpu, uint8_t val) {
    cpu->s--;
    mem_write(cpu, cpu->s, val);
}

static uint8_t pull8_s(HD6309_CPU *cpu) {
    uint8_t v = mem_read(cpu, cpu->s);
    cpu->s++;
    return v;
}

static void push16_s(HD6309_CPU *cpu, uint16_t val) {
    push8_s(cpu, (uint8_t)(val & 0xFF));
    push8_s(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16_s(HD6309_CPU *cpu) {
    uint16_t hi = pull8_s(cpu);
    uint16_t lo = pull8_s(cpu);
    return (uint16_t)((hi << 8) | lo);
}

static void push8_u(HD6309_CPU *cpu, uint8_t val) {
    cpu->u--;
    mem_write(cpu, cpu->u, val);
}

static uint8_t pull8_u(HD6309_CPU *cpu) {
    uint8_t v = mem_read(cpu, cpu->u);
    cpu->u++;
    return v;
}

static void push16_u(HD6309_CPU *cpu, uint16_t val) {
    push8_u(cpu, (uint8_t)(val & 0xFF));
    push8_u(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16_u(HD6309_CPU *cpu) {
    uint16_t hi = pull8_u(cpu);
    uint16_t lo = pull8_u(cpu);
    return (uint16_t)((hi << 8) | lo);
}

/* ---------------------------------------------------------------- flags */

static void set_nz8(HD6309_CPU *cpu, uint8_t val) {
    SET_FLAG(val & 0x80, CC_N);
    SET_FLAG(val == 0, CC_Z);
}

static void set_nz16(HD6309_CPU *cpu, uint16_t val) {
    SET_FLAG(val & 0x8000, CC_N);
    SET_FLAG(val == 0, CC_Z);
}

static void set_nz32(HD6309_CPU *cpu, uint32_t val) {
    SET_FLAG(val & 0x80000000u, CC_N);
    SET_FLAG(val == 0, CC_Z);
}

/* ---------------------------------------------------------------- ALU */

static uint8_t alu_add8(HD6309_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t sum = (uint16_t)lhs + rhs + carry_in;
    uint8_t res = (uint8_t)sum;
    SET_FLAG(((lhs & 0x0F) + (rhs & 0x0F) + carry_in) & 0x10, CC_H);
    SET_FLAG(sum & 0x100, CC_C);
    SET_FLAG((~(lhs ^ rhs) & (lhs ^ res)) & 0x80, CC_V);
    set_nz8(cpu, res);
    return res;
}

static uint8_t alu_sub8(HD6309_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t diff = (uint16_t)lhs - rhs - carry_in;
    uint8_t res = (uint8_t)diff;
    SET_FLAG(diff & 0x100, CC_C);
    SET_FLAG(((lhs ^ rhs) & (lhs ^ res)) & 0x80, CC_V);
    set_nz8(cpu, res);
    return res;
}

static uint16_t alu_add16c(HD6309_CPU *cpu, uint16_t lhs, uint16_t rhs, uint16_t carry_in) {
    uint32_t sum = (uint32_t)lhs + rhs + carry_in;
    uint16_t res = (uint16_t)sum;
    SET_FLAG(sum & 0x10000, CC_C);
    SET_FLAG((~(lhs ^ rhs) & (lhs ^ res)) & 0x8000, CC_V);
    set_nz16(cpu, res);
    return res;
}

static uint16_t alu_sub16c(HD6309_CPU *cpu, uint16_t lhs, uint16_t rhs, uint16_t carry_in) {
    uint32_t diff = (uint32_t)lhs - rhs - carry_in;
    uint16_t res = (uint16_t)diff;
    SET_FLAG(diff & 0x10000, CC_C);
    SET_FLAG(((lhs ^ rhs) & (lhs ^ res)) & 0x8000, CC_V);
    set_nz16(cpu, res);
    return res;
}

static uint16_t alu_add16(HD6309_CPU *cpu, uint16_t lhs, uint16_t rhs) {
    return alu_add16c(cpu, lhs, rhs, 0);
}

static uint16_t alu_sub16(HD6309_CPU *cpu, uint16_t lhs, uint16_t rhs) {
    return alu_sub16c(cpu, lhs, rhs, 0);
}

static uint8_t alu_logic8(HD6309_CPU *cpu, uint8_t res) {
    set_nz8(cpu, res);
    cpu->cc &= ~(uint8_t)CC_V;
    return res;
}

static uint16_t alu_logic16(HD6309_CPU *cpu, uint16_t res) {
    set_nz16(cpu, res);
    cpu->cc &= ~(uint8_t)CC_V;
    return res;
}

static uint16_t alu_ld16(HD6309_CPU *cpu, uint16_t val) {
    set_nz16(cpu, val);
    cpu->cc &= ~(uint8_t)CC_V;
    return val;
}

static uint32_t alu_ld32(HD6309_CPU *cpu, uint32_t val) {
    set_nz32(cpu, val);
    cpu->cc &= ~(uint8_t)CC_V;
    return val;
}

/*
 * Unary (read-modify-write) group shared by opcode columns 0x0n/0x4n/0x5n/0x6n/0x7n.
 * sub is the low nibble of the opcode. *writeback is set to 1 if the result
 * must be stored, 0 for TST, and -1 for an illegal column.
 */
static uint8_t do_unary(HD6309_CPU *cpu, int sub, uint8_t val, int *writeback) {
    uint8_t res = val;
    *writeback = 1;
    switch (sub) {
        case 0x0: /* NEG */
            res = (uint8_t)(0 - val);
            SET_FLAG(val == 0x80, CC_V);
            SET_FLAG(val != 0, CC_C);
            set_nz8(cpu, res);
            break;
        case 0x3: /* COM */
            res = (uint8_t)~val;
            cpu->cc &= ~(uint8_t)CC_V;
            cpu->cc |= CC_C;
            set_nz8(cpu, res);
            break;
        case 0x4: /* LSR */
            SET_FLAG(val & 0x01, CC_C);
            res = (uint8_t)(val >> 1);
            set_nz8(cpu, res);
            break;
        case 0x6: /* ROR */
            {
                uint8_t old_c = GET_FLAG(CC_C);
                SET_FLAG(val & 0x01, CC_C);
                res = (uint8_t)((val >> 1) | (old_c << 7));
                set_nz8(cpu, res);
            }
            break;
        case 0x7: /* ASR */
            SET_FLAG(val & 0x01, CC_C);
            res = (uint8_t)((val >> 1) | (val & 0x80));
            set_nz8(cpu, res);
            break;
        case 0x8: /* ASL/LSL */
            SET_FLAG(val & 0x80, CC_C);
            SET_FLAG((val ^ (uint8_t)(val << 1)) & 0x80, CC_V);
            res = (uint8_t)(val << 1);
            set_nz8(cpu, res);
            break;
        case 0x9: /* ROL */
            {
                uint8_t old_c = GET_FLAG(CC_C);
                SET_FLAG(val & 0x80, CC_C);
                SET_FLAG((val ^ (uint8_t)(val << 1)) & 0x80, CC_V);
                res = (uint8_t)((val << 1) | old_c);
                set_nz8(cpu, res);
            }
            break;
        case 0xA: /* DEC */
            res = (uint8_t)(val - 1);
            SET_FLAG(val == 0x80, CC_V);
            set_nz8(cpu, res);
            break;
        case 0xC: /* INC */
            res = (uint8_t)(val + 1);
            SET_FLAG(val == 0x7F, CC_V);
            set_nz8(cpu, res);
            break;
        case 0xD: /* TST */
            cpu->cc &= ~(uint8_t)CC_V;
            set_nz8(cpu, val);
            *writeback = 0;
            break;
        case 0xF: /* CLR */
            res = 0;
            cpu->cc &= ~(uint8_t)(CC_N | CC_V | CC_C);
            cpu->cc |= CC_Z;
            break;
        default:
            *writeback = -1;
            break;
    }
    return res;
}

/*
 * 16-bit unary group for the D/W inherent forms on page 2 (NEGD, COMW, ...).
 * Same writeback convention as do_unary.
 */
static uint16_t do_unary16(HD6309_CPU *cpu, int sub, uint16_t val, int *writeback) {
    uint16_t res = val;
    *writeback = 1;
    switch (sub) {
        case 0x0: /* NEG */
            res = (uint16_t)(0 - val);
            SET_FLAG(val == 0x8000, CC_V);
            SET_FLAG(val != 0, CC_C);
            set_nz16(cpu, res);
            break;
        case 0x3: /* COM */
            res = (uint16_t)~val;
            cpu->cc &= ~(uint8_t)CC_V;
            cpu->cc |= CC_C;
            set_nz16(cpu, res);
            break;
        case 0x4: /* LSR */
            SET_FLAG(val & 0x0001, CC_C);
            res = (uint16_t)(val >> 1);
            set_nz16(cpu, res);
            break;
        case 0x6: /* ROR */
            {
                uint16_t old_c = (uint16_t)GET_FLAG(CC_C);
                SET_FLAG(val & 0x0001, CC_C);
                res = (uint16_t)((val >> 1) | (old_c << 15));
                set_nz16(cpu, res);
            }
            break;
        case 0x7: /* ASR */
            SET_FLAG(val & 0x0001, CC_C);
            res = (uint16_t)((val >> 1) | (val & 0x8000));
            set_nz16(cpu, res);
            break;
        case 0x8: /* ASL/LSL */
            SET_FLAG(val & 0x8000, CC_C);
            SET_FLAG((val ^ (uint16_t)(val << 1)) & 0x8000, CC_V);
            res = (uint16_t)(val << 1);
            set_nz16(cpu, res);
            break;
        case 0x9: /* ROL */
            {
                uint16_t old_c = (uint16_t)GET_FLAG(CC_C);
                SET_FLAG(val & 0x8000, CC_C);
                SET_FLAG((val ^ (uint16_t)(val << 1)) & 0x8000, CC_V);
                res = (uint16_t)((val << 1) | old_c);
                set_nz16(cpu, res);
            }
            break;
        case 0xA: /* DEC */
            res = (uint16_t)(val - 1);
            SET_FLAG(val == 0x8000, CC_V);
            set_nz16(cpu, res);
            break;
        case 0xC: /* INC */
            res = (uint16_t)(val + 1);
            SET_FLAG(val == 0x7FFF, CC_V);
            set_nz16(cpu, res);
            break;
        case 0xD: /* TST */
            cpu->cc &= ~(uint8_t)CC_V;
            set_nz16(cpu, val);
            *writeback = 0;
            break;
        case 0xF: /* CLR */
            res = 0;
            cpu->cc &= ~(uint8_t)(CC_N | CC_V | CC_C);
            cpu->cc |= CC_Z;
            break;
        default:
            *writeback = -1;
            break;
    }
    return res;
}

/* ------------------------------------------------------- effective address */

static uint16_t direct_ea(HD6309_CPU *cpu) {
    return (uint16_t)(((uint16_t)cpu->dp << 8) | fetch8(cpu));
}

static uint16_t indexed_ea(HD6309_CPU *cpu) {
    uint8_t pb = fetch8(cpu);
    uint16_t *reg;
    uint16_t ea;

    switch ((pb >> 5) & 0x03) {
        case 0: reg = &cpu->x; break;
        case 1: reg = &cpu->y; break;
        case 2: reg = &cpu->u; break;
        default: reg = &cpu->s; break;
    }

    if (!(pb & 0x80)) {
        /* 5-bit signed offset, never indirect */
        int8_t off = (int8_t)(pb & 0x1F);
        if (off & 0x10) off = (int8_t)(off | 0xE0);
        cpu->ticks += 1;
        return (uint16_t)(*reg + off);
    }

    switch (pb & 0x0F) {
        case 0x0: /* ,R+ */
            ea = *reg;
            *reg = (uint16_t)(*reg + 1);
            cpu->ticks += 2;
            break;
        case 0x1: /* ,R++ */
            ea = *reg;
            *reg = (uint16_t)(*reg + 2);
            cpu->ticks += 3;
            break;
        case 0x2: /* ,-R */
            *reg = (uint16_t)(*reg - 1);
            ea = *reg;
            cpu->ticks += 2;
            break;
        case 0x3: /* ,--R */
            *reg = (uint16_t)(*reg - 2);
            ea = *reg;
            cpu->ticks += 3;
            break;
        case 0x4: /* ,R */
            ea = *reg;
            break;
        case 0x5: /* B,R */
            ea = (uint16_t)(*reg + (int8_t)cpu->b);
            cpu->ticks += 1;
            break;
        case 0x6: /* A,R */
            ea = (uint16_t)(*reg + (int8_t)cpu->a);
            cpu->ticks += 1;
            break;
        case 0x7: /* E,R (6309) */
            ea = (uint16_t)(*reg + (int8_t)cpu->e);
            cpu->ticks += 1;
            break;
        case 0x8: /* n8,R */
            ea = (uint16_t)(*reg + (int8_t)fetch8(cpu));
            cpu->ticks += 1;
            break;
        case 0x9: /* n16,R */
            ea = (uint16_t)(*reg + fetch16(cpu));
            cpu->ticks += 4;
            break;
        case 0xA: /* F,R (6309) */
            ea = (uint16_t)(*reg + (int8_t)cpu->f);
            cpu->ticks += 1;
            break;
        case 0xB: /* D,R */
            ea = (uint16_t)(*reg + get_d(cpu));
            cpu->ticks += 4;
            break;
        case 0xC: /* n8,PCR */
            {
                int8_t off = (int8_t)fetch8(cpu);
                ea = (uint16_t)(cpu->pc + off);
                cpu->ticks += 1;
            }
            break;
        case 0xD: /* n16,PCR */
            {
                uint16_t off = fetch16(cpu);
                ea = (uint16_t)(cpu->pc + off);
                cpu->ticks += 5;
            }
            break;
        case 0xE: /* W,R (6309) */
            ea = (uint16_t)(*reg + get_w(cpu));
            cpu->ticks += 4;
            break;
        default: /* 0xF: [n16] extended indirect */
            ea = fetch16(cpu);
            cpu->ticks += 2;
            break;
    }

    if (pb & 0x10) {
        ea = mem_read16(cpu, ea);
        cpu->ticks += 3;
    }
    return ea;
}

/* mode: 0 = immediate, 1 = direct, 2 = indexed, 3 = extended */
static uint16_t operand_ea(HD6309_CPU *cpu, int mode) {
    switch (mode) {
        case 1:
            cpu->ticks += 2;
            return direct_ea(cpu);
        case 2:
            cpu->ticks += 2;
            return indexed_ea(cpu);
        default:
            cpu->ticks += 3;
            return fetch16(cpu);
    }
}

static uint8_t load_operand8(HD6309_CPU *cpu, int mode) {
    if (mode == 0) {
        cpu->ticks += 1;
        return fetch8(cpu);
    }
    return mem_read(cpu, operand_ea(cpu, mode));
}

static uint16_t load_operand16(HD6309_CPU *cpu, int mode) {
    if (mode == 0) {
        cpu->ticks += 2;
        return fetch16(cpu);
    }
    return mem_read16(cpu, operand_ea(cpu, mode));
}

/* ------------------------------------------------------------ TFR / EXG */

static uint16_t get_reg_by_code(HD6309_CPU *cpu, uint8_t code) {
    switch (code & 0x0F) {
        case 0x0: return get_d(cpu);
        case 0x1: return cpu->x;
        case 0x2: return cpu->y;
        case 0x3: return cpu->u;
        case 0x4: return cpu->s;
        case 0x5: return cpu->pc;
        case 0x6: return get_w(cpu);
        case 0x7: return cpu->v;
        case 0x8: return (uint16_t)(0xFF00 | cpu->a);
        case 0x9: return (uint16_t)(0xFF00 | cpu->b);
        case 0xA: return (uint16_t)(0xFF00 | cpu->cc);
        case 0xB: return (uint16_t)(0xFF00 | cpu->dp);
        case 0xE: return (uint16_t)(0xFF00 | cpu->e);
        case 0xF: return (uint16_t)(0xFF00 | cpu->f);
        default:  return 0x0000; /* 0xC, 0xD: zero register */
    }
}

static void set_reg_by_code(HD6309_CPU *cpu, uint8_t code, uint16_t val) {
    switch (code & 0x0F) {
        case 0x0: set_d(cpu, val); break;
        case 0x1: cpu->x = val; break;
        case 0x2: cpu->y = val; break;
        case 0x3: cpu->u = val; break;
        case 0x4: cpu->s = val; break;
        case 0x5: cpu->pc = val; break;
        case 0x6: set_w(cpu, val); break;
        case 0x7: cpu->v = val; break;
        case 0x8: cpu->a = (uint8_t)(val & 0xFF); break;
        case 0x9: cpu->b = (uint8_t)(val & 0xFF); break;
        case 0xA: cpu->cc = (uint8_t)(val & 0xFF); break;
        case 0xB: cpu->dp = (uint8_t)(val & 0xFF); break;
        case 0xE: cpu->e = (uint8_t)(val & 0xFF); break;
        case 0xF: cpu->f = (uint8_t)(val & 0xFF); break;
        default: break; /* 0xC, 0xD: writes to the zero register are lost */
    }
}

static int reg_code_is8(uint8_t code) {
    switch (code & 0x0F) {
        case 0x8: case 0x9: case 0xA: case 0xB: case 0xE: case 0xF:
            return 1;
        default:
            return 0;
    }
}

/* -------------------------------------- ADDR/SUBR/... inter-register group */

/* op_sub is the low three bits of the page-2 opcode 0x30-0x37 */
static void do_interreg(HD6309_CPU *cpu, uint8_t op_sub) {
    uint8_t pb = fetch8(cpu);
    uint8_t src = (uint8_t)((pb >> 4) & 0x0F);
    uint8_t dst = (uint8_t)(pb & 0x0F);
    cpu->ticks += 2;

    if (reg_code_is8(dst)) {
        uint8_t s = (uint8_t)(get_reg_by_code(cpu, src) & 0xFF);
        uint8_t d = (uint8_t)(get_reg_by_code(cpu, dst) & 0xFF);
        uint8_t res = d;
        int store = 1;
        switch (op_sub & 0x07) {
            case 0x0: res = alu_add8(cpu, d, s, 0); break;                        /* ADDR */
            case 0x1: res = alu_add8(cpu, d, s, (uint8_t)GET_FLAG(CC_C)); break;  /* ADCR */
            case 0x2: res = alu_sub8(cpu, d, s, 0); break;                        /* SUBR */
            case 0x3: res = alu_sub8(cpu, d, s, (uint8_t)GET_FLAG(CC_C)); break;  /* SBCR */
            case 0x4: res = alu_logic8(cpu, (uint8_t)(d & s)); break;             /* ANDR */
            case 0x5: res = alu_logic8(cpu, (uint8_t)(d | s)); break;             /* ORR */
            case 0x6: res = alu_logic8(cpu, (uint8_t)(d ^ s)); break;             /* EORR */
            default:  (void)alu_sub8(cpu, d, s, 0); store = 0; break;             /* CMPR */
        }
        if (store) set_reg_by_code(cpu, dst, res);
    } else {
        uint16_t s = get_reg_by_code(cpu, src);
        uint16_t d = get_reg_by_code(cpu, dst);
        uint16_t res = d;
        int store = 1;
        switch (op_sub & 0x07) {
            case 0x0: res = alu_add16c(cpu, d, s, 0); break;                        /* ADDR */
            case 0x1: res = alu_add16c(cpu, d, s, (uint16_t)GET_FLAG(CC_C)); break; /* ADCR */
            case 0x2: res = alu_sub16c(cpu, d, s, 0); break;                        /* SUBR */
            case 0x3: res = alu_sub16c(cpu, d, s, (uint16_t)GET_FLAG(CC_C)); break; /* SBCR */
            case 0x4: res = alu_logic16(cpu, (uint16_t)(d & s)); break;             /* ANDR */
            case 0x5: res = alu_logic16(cpu, (uint16_t)(d | s)); break;             /* ORR */
            case 0x6: res = alu_logic16(cpu, (uint16_t)(d ^ s)); break;             /* EORR */
            default:  (void)alu_sub16c(cpu, d, s, 0); store = 0; break;             /* CMPR */
        }
        if (store) set_reg_by_code(cpu, dst, res);
    }
}

/* -------------------------------------------------- TFM block transfers */

static int tfm_reg_get(HD6309_CPU *cpu, uint8_t code, uint16_t *out) {
    switch (code & 0x0F) {
        case 0x0: *out = get_d(cpu); return 0;
        case 0x1: *out = cpu->x; return 0;
        case 0x2: *out = cpu->y; return 0;
        case 0x3: *out = cpu->u; return 0;
        case 0x4: *out = cpu->s; return 0;
        default:  return -1;
    }
}

static void tfm_reg_set(HD6309_CPU *cpu, uint8_t code, uint16_t val) {
    switch (code & 0x0F) {
        case 0x0: set_d(cpu, val); break;
        case 0x1: cpu->x = val; break;
        case 0x2: cpu->y = val; break;
        case 0x3: cpu->u = val; break;
        case 0x4: cpu->s = val; break;
        default: break;
    }
}

/* variant: 0 = R0+,R1+  1 = R0-,R1-  2 = R0+,R1  3 = R0,R1+ */
static int do_tfm(HD6309_CPU *cpu, int variant) {
    uint8_t pb = fetch8(cpu);
    uint8_t sc = (uint8_t)((pb >> 4) & 0x0F);
    uint8_t dc = (uint8_t)(pb & 0x0F);
    uint16_t src, dst, count;

    if (tfm_reg_get(cpu, sc, &src) < 0 || tfm_reg_get(cpu, dc, &dst) < 0) {
        cpu->md |= MD_IL;
        cpu->halted = 1;
        return 1;
    }

    count = get_w(cpu);
    cpu->ticks += 6;
    while (count != 0) {
        mem_write(cpu, dst, mem_read(cpu, src));
        switch (variant) {
            case 0: src++; dst++; break;
            case 1: src--; dst--; break;
            case 2: src++; break;
            default: dst++; break;
        }
        count--;
        cpu->ticks += 3;
    }

    tfm_reg_set(cpu, sc, src);
    tfm_reg_set(cpu, dc, dst);
    set_w(cpu, 0);
    return 0;
}

/* ---------------------------------------------- BAND/BOR/BEOR bit ops */

/* kind: 0 = BAND, 1 = BOR, 2 = BEOR */
static int do_bitop(HD6309_CPU *cpu, int kind) {
    uint8_t pb = fetch8(cpu);
    uint16_t ea = direct_ea(cpu);
    uint8_t *reg;
    uint8_t src_bit = (uint8_t)((pb >> 3) & 0x07);
    uint8_t dst_bit = (uint8_t)(pb & 0x07);
    uint8_t mbit, rbit, res;

    switch ((pb >> 6) & 0x03) {
        case 0x0: reg = &cpu->cc; break;
        case 0x1: reg = &cpu->a; break;
        case 0x2: reg = &cpu->b; break;
        default:
            cpu->md |= MD_IL;
            cpu->halted = 1;
            return 1;
    }

    mbit = (uint8_t)((mem_read(cpu, ea) >> src_bit) & 0x01);
    rbit = (uint8_t)((*reg >> dst_bit) & 0x01);
    switch (kind) {
        case 0:  res = (uint8_t)(rbit & mbit); break;
        case 1:  res = (uint8_t)(rbit | mbit); break;
        default: res = (uint8_t)(rbit ^ mbit); break;
    }
    if (res) {
        *reg |= (uint8_t)(1 << dst_bit);
    } else {
        *reg &= (uint8_t)~(1 << dst_bit);
    }
    cpu->ticks += 5;
    return 0;
}

/* ------------------------------------------- AIM/OIM/EIM/TIM memory ops */

/* sub: low nibble of the opcode (0x1 OIM, 0x2 AIM, 0x5 EIM, 0xB TIM) */
static void do_memimm(HD6309_CPU *cpu, int sub, uint8_t imm, uint16_t ea) {
    uint8_t m = mem_read(cpu, ea);
    switch (sub) {
        case 0x1: /* OIM */
            mem_write(cpu, ea, alu_logic8(cpu, (uint8_t)(m | imm)));
            break;
        case 0x2: /* AIM */
            mem_write(cpu, ea, alu_logic8(cpu, (uint8_t)(m & imm)));
            break;
        case 0x5: /* EIM */
            mem_write(cpu, ea, alu_logic8(cpu, (uint8_t)(m ^ imm)));
            break;
        default: /* 0xB: TIM (no writeback) */
            (void)alu_logic8(cpu, (uint8_t)(m & imm));
            break;
    }
    cpu->ticks += 3;
}

/* ------------------------------------------------------- PSH / PUL groups */

static void do_pshs(HD6309_CPU *cpu, uint8_t pb) {
    if (pb & 0x80) { push16_s(cpu, cpu->pc); cpu->ticks += 2; }
    if (pb & 0x40) { push16_s(cpu, cpu->u);  cpu->ticks += 2; }
    if (pb & 0x20) { push16_s(cpu, cpu->y);  cpu->ticks += 2; }
    if (pb & 0x10) { push16_s(cpu, cpu->x);  cpu->ticks += 2; }
    if (pb & 0x08) { push8_s(cpu, cpu->dp);  cpu->ticks += 1; }
    if (pb & 0x04) { push8_s(cpu, cpu->b);   cpu->ticks += 1; }
    if (pb & 0x02) { push8_s(cpu, cpu->a);   cpu->ticks += 1; }
    if (pb & 0x01) { push8_s(cpu, cpu->cc);  cpu->ticks += 1; }
}

static void do_puls(HD6309_CPU *cpu, uint8_t pb) {
    if (pb & 0x01) { cpu->cc = pull8_s(cpu);  cpu->ticks += 1; }
    if (pb & 0x02) { cpu->a = pull8_s(cpu);   cpu->ticks += 1; }
    if (pb & 0x04) { cpu->b = pull8_s(cpu);   cpu->ticks += 1; }
    if (pb & 0x08) { cpu->dp = pull8_s(cpu);  cpu->ticks += 1; }
    if (pb & 0x10) { cpu->x = pull16_s(cpu);  cpu->ticks += 2; }
    if (pb & 0x20) { cpu->y = pull16_s(cpu);  cpu->ticks += 2; }
    if (pb & 0x40) { cpu->u = pull16_s(cpu);  cpu->ticks += 2; }
    if (pb & 0x80) { cpu->pc = pull16_s(cpu); cpu->ticks += 2; }
}

static void do_pshu(HD6309_CPU *cpu, uint8_t pb) {
    if (pb & 0x80) { push16_u(cpu, cpu->pc); cpu->ticks += 2; }
    if (pb & 0x40) { push16_u(cpu, cpu->s);  cpu->ticks += 2; }
    if (pb & 0x20) { push16_u(cpu, cpu->y);  cpu->ticks += 2; }
    if (pb & 0x10) { push16_u(cpu, cpu->x);  cpu->ticks += 2; }
    if (pb & 0x08) { push8_u(cpu, cpu->dp);  cpu->ticks += 1; }
    if (pb & 0x04) { push8_u(cpu, cpu->b);   cpu->ticks += 1; }
    if (pb & 0x02) { push8_u(cpu, cpu->a);   cpu->ticks += 1; }
    if (pb & 0x01) { push8_u(cpu, cpu->cc);  cpu->ticks += 1; }
}

static void do_pulu(HD6309_CPU *cpu, uint8_t pb) {
    if (pb & 0x01) { cpu->cc = pull8_u(cpu);  cpu->ticks += 1; }
    if (pb & 0x02) { cpu->a = pull8_u(cpu);   cpu->ticks += 1; }
    if (pb & 0x04) { cpu->b = pull8_u(cpu);   cpu->ticks += 1; }
    if (pb & 0x08) { cpu->dp = pull8_u(cpu);  cpu->ticks += 1; }
    if (pb & 0x10) { cpu->x = pull16_u(cpu);  cpu->ticks += 2; }
    if (pb & 0x20) { cpu->y = pull16_u(cpu);  cpu->ticks += 2; }
    if (pb & 0x40) { cpu->s = pull16_u(cpu);  cpu->ticks += 2; }
    if (pb & 0x80) { cpu->pc = pull16_u(cpu); cpu->ticks += 2; }
}

/* ---------------------------------------------------------- SWI machinery */

static void do_swi(HD6309_CPU *cpu, uint16_t vector, int set_mask) {
    cpu->cc |= CC_E;
    push16_s(cpu, cpu->pc);
    push16_s(cpu, cpu->u);
    push16_s(cpu, cpu->y);
    push16_s(cpu, cpu->x);
    push8_s(cpu, cpu->dp);
    if (cpu->md & MD_NM) { /* native mode also stacks W */
        push8_s(cpu, cpu->f);
        push8_s(cpu, cpu->e);
        cpu->ticks += 2;
    }
    push8_s(cpu, cpu->b);
    push8_s(cpu, cpu->a);
    push8_s(cpu, cpu->cc);
    if (set_mask) {
        cpu->cc |= (CC_I | CC_F);
    }
    cpu->pc = mem_read16(cpu, vector);
    cpu->ticks += 17;
}

/* ------------------------------------------------------------- branching */

static int branch_taken(HD6309_CPU *cpu, uint8_t cond) {
    uint8_t c = GET_FLAG(CC_C);
    uint8_t z = GET_FLAG(CC_Z);
    uint8_t n = GET_FLAG(CC_N);
    uint8_t v = GET_FLAG(CC_V);
    switch (cond & 0x0F) {
        case 0x0: return 1;              /* BRA */
        case 0x1: return 0;              /* BRN */
        case 0x2: return !(c | z);       /* BHI */
        case 0x3: return (c | z);        /* BLS */
        case 0x4: return !c;             /* BCC/BHS */
        case 0x5: return c;              /* BCS/BLO */
        case 0x6: return !z;             /* BNE */
        case 0x7: return z;              /* BEQ */
        case 0x8: return !v;             /* BVC */
        case 0x9: return v;              /* BVS */
        case 0xA: return !n;             /* BPL */
        case 0xB: return n;              /* BMI */
        case 0xC: return n == v;         /* BGE */
        case 0xD: return n != v;         /* BLT */
        case 0xE: return !z && (n == v); /* BGT */
        default:  return z || (n != v);  /* BLE */
    }
}

/* ------------------------------------------------------------- illegal op */

static int illegal(HD6309_CPU *cpu) {
    cpu->md |= MD_IL;
    cpu->halted = 1;
    return 1;
}

/* --------------------------------------------------------- page 2 (0x10) */

static int step_page2(HD6309_CPU *cpu) {
    uint8_t op = fetch8(cpu);
    cpu->ticks += 1;

    /* Long conditional branches 0x21-0x2F */
    if (op >= 0x21 && op <= 0x2F) {
        int16_t off = (int16_t)fetch16(cpu);
        cpu->ticks += 4;
        if (branch_taken(cpu, op)) {
            cpu->pc = (uint16_t)(cpu->pc + off);
            cpu->ticks += 1;
        }
        return 0;
    }

    /* Inter-register operations 0x30-0x37 */
    if (op >= 0x30 && op <= 0x37) {
        do_interreg(cpu, (uint8_t)(op & 0x07));
        return 0;
    }

    switch (op) {
        case 0x38: /* PSHSW */
            push16_s(cpu, get_w(cpu));
            cpu->ticks += 4;
            return 0;
        case 0x39: /* PULSW */
            set_w(cpu, pull16_s(cpu));
            cpu->ticks += 4;
            return 0;
        case 0x3A: /* PSHUW */
            push16_u(cpu, get_w(cpu));
            cpu->ticks += 4;
            return 0;
        case 0x3B: /* PULUW */
            set_w(cpu, pull16_u(cpu));
            cpu->ticks += 4;
            return 0;
        case 0x3F: /* SWI2 */
            do_swi(cpu, 0xFFF4, 0);
            return 0;
        default:
            break;
    }

    /* Inherent D forms 0x40-0x4F (NEGD, COMD, LSRD, ...) */
    if (op >= 0x40 && op <= 0x4F) {
        int wb;
        uint16_t res = do_unary16(cpu, op & 0x0F, get_d(cpu), &wb);
        if (wb < 0) return illegal(cpu);
        if (wb) set_d(cpu, res);
        return 0;
    }

    /* Inherent W forms 0x50-0x5F (COMW, LSRW, RORW, ROLW, DECW, INCW, TSTW, CLRW) */
    if (op >= 0x50 && op <= 0x5F) {
        int sub = op & 0x0F;
        int wb;
        uint16_t res;
        if (sub != 0x3 && sub != 0x4 && sub != 0x6 && sub != 0x9 &&
            sub != 0xA && sub != 0xC && sub != 0xD && sub != 0xF) {
            return illegal(cpu);
        }
        res = do_unary16(cpu, sub, get_w(cpu), &wb);
        if (wb < 0) return illegal(cpu);
        if (wb) set_w(cpu, res);
        return 0;
    }

    if (op >= 0x80) {
        int mode = (op >> 4) & 0x03;
        switch (op & 0xCF) {
            case 0x80: /* SUBW */
                set_w(cpu, alu_sub16(cpu, get_w(cpu), load_operand16(cpu, mode)));
                cpu->ticks += 3;
                return 0;
            case 0x81: /* CMPW */
                (void)alu_sub16(cpu, get_w(cpu), load_operand16(cpu, mode));
                cpu->ticks += 3;
                return 0;
            case 0x82: /* SBCD */
                set_d(cpu, alu_sub16c(cpu, get_d(cpu), load_operand16(cpu, mode),
                                      (uint16_t)GET_FLAG(CC_C)));
                cpu->ticks += 3;
                return 0;
            case 0x83: /* CMPD */
                (void)alu_sub16(cpu, get_d(cpu), load_operand16(cpu, mode));
                cpu->ticks += 4;
                return 0;
            case 0x84: /* ANDD */
                set_d(cpu, alu_logic16(cpu, (uint16_t)(get_d(cpu) & load_operand16(cpu, mode))));
                cpu->ticks += 3;
                return 0;
            case 0x85: /* BITD */
                (void)alu_logic16(cpu, (uint16_t)(get_d(cpu) & load_operand16(cpu, mode)));
                cpu->ticks += 3;
                return 0;
            case 0x86: /* LDW */
                set_w(cpu, alu_ld16(cpu, load_operand16(cpu, mode)));
                cpu->ticks += 3;
                return 0;
            case 0x87: /* STW (no immediate form) */
                if (mode == 0) break;
                mem_write16(cpu, operand_ea(cpu, mode), alu_ld16(cpu, get_w(cpu)));
                cpu->ticks += 3;
                return 0;
            case 0x88: /* EORD */
                set_d(cpu, alu_logic16(cpu, (uint16_t)(get_d(cpu) ^ load_operand16(cpu, mode))));
                cpu->ticks += 3;
                return 0;
            case 0x89: /* ADCD */
                set_d(cpu, alu_add16c(cpu, get_d(cpu), load_operand16(cpu, mode),
                                      (uint16_t)GET_FLAG(CC_C)));
                cpu->ticks += 3;
                return 0;
            case 0x8A: /* ORD */
                set_d(cpu, alu_logic16(cpu, (uint16_t)(get_d(cpu) | load_operand16(cpu, mode))));
                cpu->ticks += 3;
                return 0;
            case 0x8B: /* ADDW */
                set_w(cpu, alu_add16(cpu, get_w(cpu), load_operand16(cpu, mode)));
                cpu->ticks += 3;
                return 0;
            case 0x8C: /* CMPY */
                (void)alu_sub16(cpu, cpu->y, load_operand16(cpu, mode));
                cpu->ticks += 4;
                return 0;
            case 0x8E: /* LDY */
                cpu->y = alu_ld16(cpu, load_operand16(cpu, mode));
                cpu->ticks += 3;
                return 0;
            case 0x8F: /* STY */
                if (mode == 0) break;
                mem_write16(cpu, operand_ea(cpu, mode), alu_ld16(cpu, cpu->y));
                cpu->ticks += 3;
                return 0;
            case 0xCC: /* LDQ direct/indexed/extended (immediate form is 0xCD page 1) */
                if (mode == 0) break;
                set_q(cpu, alu_ld32(cpu, mem_read32(cpu, operand_ea(cpu, mode))));
                cpu->ticks += 5;
                return 0;
            case 0xCD: /* STQ (no immediate form) */
                if (mode == 0) break;
                mem_write32(cpu, operand_ea(cpu, mode), alu_ld32(cpu, get_q(cpu)));
                cpu->ticks += 5;
                return 0;
            case 0xCE: /* LDS */
                cpu->s = alu_ld16(cpu, load_operand16(cpu, mode));
                cpu->ticks += 3;
                return 0;
            case 0xCF: /* STS */
                if (mode == 0) break;
                mem_write16(cpu, operand_ea(cpu, mode), alu_ld16(cpu, cpu->s));
                cpu->ticks += 3;
                return 0;
            default:
                break;
        }
    }

    return illegal(cpu);
}

/* --------------------------------------------------------- page 3 (0x11) */

static int step_page3(HD6309_CPU *cpu) {
    uint8_t op = fetch8(cpu);
    cpu->ticks += 1;

    switch (op) {
        case 0x30: /* BAND */
            return do_bitop(cpu, 0);
        case 0x32: /* BOR */
            return do_bitop(cpu, 1);
        case 0x34: /* BEOR */
            return do_bitop(cpu, 2);
        case 0x38: /* TFM R0+,R1+ */
            return do_tfm(cpu, 0);
        case 0x39: /* TFM R0-,R1- */
            return do_tfm(cpu, 1);
        case 0x3A: /* TFM R0+,R1 */
            return do_tfm(cpu, 2);
        case 0x3B: /* TFM R0,R1+ */
            return do_tfm(cpu, 3);
        case 0x3C: /* BITMD */
            {
                uint8_t imm = fetch8(cpu);
                uint8_t hit = (uint8_t)(cpu->md & imm & (MD_IL | MD_DZ));
                SET_FLAG(hit == 0, CC_Z);
                cpu->md &= (uint8_t)~(imm & (MD_IL | MD_DZ));
                cpu->ticks += 2;
            }
            return 0;
        case 0x3D: /* LDMD */
            cpu->md = (uint8_t)((cpu->md & (MD_IL | MD_DZ)) | (fetch8(cpu) & (MD_NM | MD_FM)));
            cpu->ticks += 3;
            return 0;
        case 0x3F: /* SWI3 */
            do_swi(cpu, 0xFFF2, 0);
            return 0;
        default:
            break;
    }

    /* Inherent E forms 0x43-0x4F and F forms 0x53-0x5F (COM, DEC, INC, TST, CLR) */
    if (op >= 0x40 && op <= 0x5F) {
        uint8_t *acc = (op <= 0x4F) ? &cpu->e : &cpu->f;
        int sub = op & 0x0F;
        int wb;
        uint8_t res;
        if (sub != 0x3 && sub != 0xA && sub != 0xC && sub != 0xD && sub != 0xF) {
            return illegal(cpu);
        }
        res = do_unary(cpu, sub, *acc, &wb);
        if (wb < 0) return illegal(cpu);
        if (wb) *acc = res;
        return 0;
    }

    if (op >= 0x80) {
        int e_block = (op & 0x40) == 0;
        int mode = (op >> 4) & 0x03;
        int col = op & 0x0F;
        uint8_t *acc = e_block ? &cpu->e : &cpu->f;

        switch (col) {
            case 0x0: /* SUBE/SUBF */
                *acc = alu_sub8(cpu, *acc, load_operand8(cpu, mode), 0);
                cpu->ticks += 1;
                return 0;
            case 0x1: /* CMPE/CMPF */
                (void)alu_sub8(cpu, *acc, load_operand8(cpu, mode), 0);
                cpu->ticks += 1;
                return 0;
            case 0x3: /* CMPU */
                if (!e_block) break;
                (void)alu_sub16(cpu, cpu->u, load_operand16(cpu, mode));
                cpu->ticks += 4;
                return 0;
            case 0x6: /* LDE/LDF */
                *acc = alu_logic8(cpu, load_operand8(cpu, mode));
                cpu->ticks += 1;
                return 0;
            case 0x7: /* STE/STF (no immediate form) */
                if (mode == 0) break;
                mem_write(cpu, operand_ea(cpu, mode), alu_logic8(cpu, *acc));
                cpu->ticks += 1;
                return 0;
            case 0xB: /* ADDE/ADDF */
                *acc = alu_add8(cpu, *acc, load_operand8(cpu, mode), 0);
                cpu->ticks += 1;
                return 0;
            case 0xC: /* CMPS */
                if (!e_block) break;
                (void)alu_sub16(cpu, cpu->s, load_operand16(cpu, mode));
                cpu->ticks += 4;
                return 0;
            case 0xD: /* DIVD: D / operand8 -> quotient B, remainder A */
                if (!e_block) break;
                {
                    int16_t num = (int16_t)get_d(cpu);
                    int8_t den = (int8_t)load_operand8(cpu, mode);
                    int32_t quot, rem;
                    cpu->ticks += 24;
                    if (den == 0) {
                        cpu->md |= MD_DZ;
                        cpu->halted = 1;
                        return 1;
                    }
                    quot = (int32_t)num / den;
                    rem = (int32_t)num % den;
                    if (quot < -128 || quot > 127) {
                        cpu->cc |= CC_V; /* range overflow: registers unchanged */
                        cpu->cc &= ~(uint8_t)(CC_N | CC_Z | CC_C);
                        return 0;
                    }
                    cpu->b = (uint8_t)(quot & 0xFF);
                    cpu->a = (uint8_t)(rem & 0xFF);
                    set_nz8(cpu, cpu->b);
                    SET_FLAG(quot & 1, CC_C);
                    cpu->cc &= ~(uint8_t)CC_V;
                }
                return 0;
            case 0xE: /* DIVQ: Q / operand16 -> quotient W, remainder D */
                if (!e_block) break;
                {
                    int32_t num = (int32_t)get_q(cpu);
                    int16_t den = (int16_t)load_operand16(cpu, mode);
                    int32_t quot, rem;
                    cpu->ticks += 32;
                    if (den == 0) {
                        cpu->md |= MD_DZ;
                        cpu->halted = 1;
                        return 1;
                    }
                    quot = num / den;
                    rem = num % den;
                    if (quot < -32768 || quot > 32767) {
                        cpu->cc |= CC_V; /* range overflow: registers unchanged */
                        cpu->cc &= ~(uint8_t)(CC_N | CC_Z | CC_C);
                        return 0;
                    }
                    set_w(cpu, (uint16_t)(quot & 0xFFFF));
                    set_d(cpu, (uint16_t)(rem & 0xFFFF));
                    set_nz16(cpu, get_w(cpu));
                    SET_FLAG(quot & 1, CC_C);
                    cpu->cc &= ~(uint8_t)CC_V;
                }
                return 0;
            case 0xF: /* MULD: D * operand16 -> Q (signed) */
                if (!e_block) break;
                {
                    int32_t prod = (int32_t)(int16_t)get_d(cpu) *
                                   (int32_t)(int16_t)load_operand16(cpu, mode);
                    set_q(cpu, (uint32_t)prod);
                    set_nz32(cpu, (uint32_t)prod);
                    cpu->cc &= ~(uint8_t)(CC_V | CC_C);
                    cpu->ticks += 26;
                }
                return 0;
            default:
                break;
        }
    }

    return illegal(cpu);
}

/* ------------------------------------------------------------- lifecycle */

void* hd6309_create(void) {
    HD6309_CPU *cpu = (HD6309_CPU*)calloc(1, sizeof(HD6309_CPU));
    return cpu;
}

void hd6309_destroy(void *context) {
    free(context);
}

int hd6309_init(void *context) {
    if (!context) return -1;
    HD6309_CPU *cpu = (HD6309_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->b = 0;
    cpu->e = 0;
    cpu->f = 0;
    cpu->v = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->u = 0;
    cpu->s = 0;
    cpu->dp = 0;
    cpu->cc = CC_I | CC_F;
    cpu->md = MD_NM; /* native mode */
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int hd6309_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    HD6309_CPU *cpu = (HD6309_CPU*)context;

    if (address >= 65536) return -2;
    size_t copy_len = size;
    if (address + size > 65536) {
        copy_len = 65536 - address;
    }
    memcpy(cpu->ram + address, data, copy_len);
    cpu->pc = (uint16_t)address;

    return 0;
}

/* ------------------------------------------------------------------ step */

int hd6309_step(void *context) {
    if (!context) return -1;
    HD6309_CPU *cpu = (HD6309_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks += 2;

    /* -- 0x00-0x0F: unary read-modify-write, direct mode -- */
    if (op <= 0x0F) {
        if (op == 0x01 || op == 0x02 || op == 0x05 || op == 0x0B) {
            /* OIM/AIM/EIM/TIM #imm,direct */
            uint8_t imm = fetch8(cpu);
            uint16_t ea = direct_ea(cpu);
            cpu->ticks += 2;
            do_memimm(cpu, op & 0x0F, imm, ea);
            return 0;
        }
        uint16_t ea = direct_ea(cpu);
        cpu->ticks += 3;
        if (op == 0x0E) { /* JMP direct */
            cpu->pc = ea;
            return 0;
        }
        int wb;
        uint8_t val = (op == 0x0F) ? 0 : mem_read(cpu, ea);
        uint8_t res = do_unary(cpu, op & 0x0F, val, &wb);
        if (wb < 0) return illegal(cpu);
        if (wb) mem_write(cpu, ea, res);
        return 0;
    }

    /* -- 0x10-0x1F: pages and misc -- */
    if (op <= 0x1F) {
        switch (op) {
            case 0x10:
                return step_page2(cpu);
            case 0x11:
                return step_page3(cpu);
            case 0x12: /* NOP */
                return 0;
            case 0x13: /* SYNC (no interrupt sources: halt) */
                cpu->halted = 1;
                return 1;
            case 0x14: /* SEXW */
                set_d(cpu, (get_w(cpu) & 0x8000) ? 0xFFFF : 0x0000);
                SET_FLAG(get_d(cpu) & 0x8000, CC_N);
                SET_FLAG(get_q(cpu) == 0, CC_Z);
                cpu->ticks += 2;
                return 0;
            case 0x16: /* LBRA */
                {
                    int16_t off = (int16_t)fetch16(cpu);
                    cpu->pc = (uint16_t)(cpu->pc + off);
                    cpu->ticks += 3;
                }
                return 0;
            case 0x17: /* LBSR */
                {
                    int16_t off = (int16_t)fetch16(cpu);
                    push16_s(cpu, cpu->pc);
                    cpu->pc = (uint16_t)(cpu->pc + off);
                    cpu->ticks += 7;
                }
                return 0;
            case 0x19: /* DAA */
                {
                    uint8_t adjust = 0;
                    uint8_t hi = (uint8_t)(cpu->a >> 4);
                    uint8_t lo = (uint8_t)(cpu->a & 0x0F);
                    if (GET_FLAG(CC_H) || lo > 9) adjust |= 0x06;
                    if (GET_FLAG(CC_C) || hi > 9 || (hi > 8 && lo > 9)) adjust |= 0x60;
                    uint16_t sum = (uint16_t)cpu->a + adjust;
                    if (sum & 0x100) cpu->cc |= CC_C;
                    cpu->a = (uint8_t)sum;
                    set_nz8(cpu, cpu->a);
                }
                return 0;
            case 0x1A: /* ORCC */
                cpu->cc |= fetch8(cpu);
                cpu->ticks += 1;
                return 0;
            case 0x1C: /* ANDCC */
                cpu->cc &= fetch8(cpu);
                cpu->ticks += 1;
                return 0;
            case 0x1D: /* SEX */
                cpu->a = (cpu->b & 0x80) ? 0xFF : 0x00;
                set_nz16(cpu, get_d(cpu));
                cpu->cc &= ~(uint8_t)CC_V;
                return 0;
            case 0x1E: /* EXG */
                {
                    uint8_t pb = fetch8(cpu);
                    uint16_t tmp = get_reg_by_code(cpu, (uint8_t)(pb >> 4));
                    set_reg_by_code(cpu, (uint8_t)(pb >> 4), get_reg_by_code(cpu, pb));
                    set_reg_by_code(cpu, pb, tmp);
                    cpu->ticks += 6;
                }
                return 0;
            case 0x1F: /* TFR */
                {
                    uint8_t pb = fetch8(cpu);
                    set_reg_by_code(cpu, pb, get_reg_by_code(cpu, (uint8_t)(pb >> 4)));
                    cpu->ticks += 4;
                }
                return 0;
            default:
                return illegal(cpu);
        }
    }

    /* -- 0x20-0x2F: short branches -- */
    if (op <= 0x2F) {
        int8_t off = (int8_t)fetch8(cpu);
        cpu->ticks += 1;
        if (branch_taken(cpu, op)) {
            cpu->pc = (uint16_t)(cpu->pc + off);
        }
        return 0;
    }

    /* -- 0x30-0x3F: LEA, stack ops, misc inherent -- */
    if (op <= 0x3F) {
        switch (op) {
            case 0x30: /* LEAX */
                cpu->x = indexed_ea(cpu);
                SET_FLAG(cpu->x == 0, CC_Z);
                cpu->ticks += 2;
                return 0;
            case 0x31: /* LEAY */
                cpu->y = indexed_ea(cpu);
                SET_FLAG(cpu->y == 0, CC_Z);
                cpu->ticks += 2;
                return 0;
            case 0x32: /* LEAS */
                cpu->s = indexed_ea(cpu);
                cpu->ticks += 2;
                return 0;
            case 0x33: /* LEAU */
                cpu->u = indexed_ea(cpu);
                cpu->ticks += 2;
                return 0;
            case 0x34: /* PSHS */
                do_pshs(cpu, fetch8(cpu));
                cpu->ticks += 3;
                return 0;
            case 0x35: /* PULS */
                do_puls(cpu, fetch8(cpu));
                cpu->ticks += 3;
                return 0;
            case 0x36: /* PSHU */
                do_pshu(cpu, fetch8(cpu));
                cpu->ticks += 3;
                return 0;
            case 0x37: /* PULU */
                do_pulu(cpu, fetch8(cpu));
                cpu->ticks += 3;
                return 0;
            case 0x39: /* RTS */
                cpu->pc = pull16_s(cpu);
                cpu->ticks += 3;
                return 0;
            case 0x3A: /* ABX */
                cpu->x = (uint16_t)(cpu->x + cpu->b);
                cpu->ticks += 1;
                return 0;
            case 0x3B: /* RTI */
                cpu->cc = pull8_s(cpu);
                if (cpu->cc & CC_E) {
                    cpu->a = pull8_s(cpu);
                    cpu->b = pull8_s(cpu);
                    if (cpu->md & MD_NM) { /* native mode also stacked W */
                        cpu->e = pull8_s(cpu);
                        cpu->f = pull8_s(cpu);
                        cpu->ticks += 2;
                    }
                    cpu->dp = pull8_s(cpu);
                    cpu->x = pull16_s(cpu);
                    cpu->y = pull16_s(cpu);
                    cpu->u = pull16_s(cpu);
                    cpu->ticks += 9;
                }
                cpu->pc = pull16_s(cpu);
                cpu->ticks += 4;
                return 0;
            case 0x3C: /* CWAI (no interrupt sources: halt) */
                cpu->cc &= fetch8(cpu);
                cpu->cc |= CC_E;
                cpu->halted = 1;
                return 1;
            case 0x3D: /* MUL */
                {
                    uint16_t prod = (uint16_t)cpu->a * cpu->b;
                    set_d(cpu, prod);
                    SET_FLAG(prod == 0, CC_Z);
                    SET_FLAG(prod & 0x0080, CC_C);
                    cpu->ticks += 9;
                }
                return 0;
            case 0x3F: /* SWI */
                do_swi(cpu, 0xFFFA, 1);
                return 0;
            default:
                return illegal(cpu);
        }
    }

    /* -- 0x40-0x5F: unary on A / B -- */
    if (op <= 0x5F) {
        uint8_t *acc = (op <= 0x4F) ? &cpu->a : &cpu->b;
        int wb;
        uint8_t res = do_unary(cpu, op & 0x0F, *acc, &wb);
        if (wb < 0) return illegal(cpu);
        if (wb) *acc = res;
        return 0;
    }

    /* -- 0x60-0x7F: unary read-modify-write, indexed / extended -- */
    if (op <= 0x7F) {
        int sub = op & 0x0F;
        uint16_t ea;
        if (sub == 0x1 || sub == 0x2 || sub == 0x5 || sub == 0xB) {
            /* OIM/AIM/EIM/TIM #imm,indexed / #imm,extended */
            uint8_t imm = fetch8(cpu);
            if (op <= 0x6F) {
                ea = indexed_ea(cpu);
                cpu->ticks += 2;
            } else {
                ea = fetch16(cpu);
                cpu->ticks += 3;
            }
            do_memimm(cpu, sub, imm, ea);
            return 0;
        }
        if (op <= 0x6F) {
            ea = indexed_ea(cpu);
            cpu->ticks += 3;
        } else {
            ea = fetch16(cpu);
            cpu->ticks += 4;
        }
        if (sub == 0x0E) { /* JMP */
            cpu->pc = ea;
            return 0;
        }
        int wb;
        uint8_t val = (sub == 0x0F) ? 0 : mem_read(cpu, ea);
        uint8_t res = do_unary(cpu, sub, val, &wb);
        if (wb < 0) return illegal(cpu);
        if (wb) mem_write(cpu, ea, res);
        return 0;
    }

    /* -- 0xCD: LDQ #imm32 (6309) -- */
    if (op == 0xCD) {
        uint32_t val = ((uint32_t)fetch16(cpu) << 16) | fetch16(cpu);
        set_q(cpu, alu_ld32(cpu, val));
        cpu->ticks += 3;
        return 0;
    }

    /* -- 0x80-0xFF: accumulator / register operations -- */
    {
        int b_block = (op & 0x40) != 0;
        int mode = (op >> 4) & 0x03;
        int col = op & 0x0F;
        uint8_t *acc = b_block ? &cpu->b : &cpu->a;

        switch (col) {
            case 0x0: /* SUBA/SUBB */
                *acc = alu_sub8(cpu, *acc, load_operand8(cpu, mode), 0);
                return 0;
            case 0x1: /* CMPA/CMPB */
                (void)alu_sub8(cpu, *acc, load_operand8(cpu, mode), 0);
                return 0;
            case 0x2: /* SBCA/SBCB */
                *acc = alu_sub8(cpu, *acc, load_operand8(cpu, mode), (uint8_t)GET_FLAG(CC_C));
                return 0;
            case 0x3: /* SUBD / ADDD */
                {
                    uint16_t m = load_operand16(cpu, mode);
                    cpu->ticks += 3;
                    if (b_block) {
                        set_d(cpu, alu_add16(cpu, get_d(cpu), m));
                    } else {
                        set_d(cpu, alu_sub16(cpu, get_d(cpu), m));
                    }
                }
                return 0;
            case 0x4: /* ANDA/ANDB */
                *acc = alu_logic8(cpu, (uint8_t)(*acc & load_operand8(cpu, mode)));
                return 0;
            case 0x5: /* BITA/BITB */
                (void)alu_logic8(cpu, (uint8_t)(*acc & load_operand8(cpu, mode)));
                return 0;
            case 0x6: /* LDA/LDB */
                *acc = alu_logic8(cpu, load_operand8(cpu, mode));
                return 0;
            case 0x7: /* STA/STB (no immediate form) */
                if (mode == 0) break;
                mem_write(cpu, operand_ea(cpu, mode), alu_logic8(cpu, *acc));
                return 0;
            case 0x8: /* EORA/EORB */
                *acc = alu_logic8(cpu, (uint8_t)(*acc ^ load_operand8(cpu, mode)));
                return 0;
            case 0x9: /* ADCA/ADCB */
                *acc = alu_add8(cpu, *acc, load_operand8(cpu, mode), (uint8_t)GET_FLAG(CC_C));
                return 0;
            case 0xA: /* ORA/ORB */
                *acc = alu_logic8(cpu, (uint8_t)(*acc | load_operand8(cpu, mode)));
                return 0;
            case 0xB: /* ADDA/ADDB */
                *acc = alu_add8(cpu, *acc, load_operand8(cpu, mode), 0);
                return 0;
            case 0xC: /* CMPX / LDD */
                if (b_block) {
                    set_d(cpu, alu_ld16(cpu, load_operand16(cpu, mode)));
                } else {
                    (void)alu_sub16(cpu, cpu->x, load_operand16(cpu, mode));
                    cpu->ticks += 2;
                }
                return 0;
            case 0xD: /* BSR / JSR / STD */
                if (!b_block) {
                    if (mode == 0) { /* BSR */
                        int8_t off = (int8_t)fetch8(cpu);
                        push16_s(cpu, cpu->pc);
                        cpu->pc = (uint16_t)(cpu->pc + off);
                        cpu->ticks += 5;
                    } else { /* JSR */
                        uint16_t ea = operand_ea(cpu, mode);
                        push16_s(cpu, cpu->pc);
                        cpu->pc = ea;
                        cpu->ticks += 3;
                    }
                    return 0;
                }
                if (mode == 0) break; /* STD immediate is illegal (LDQ #imm handled above) */
                mem_write16(cpu, operand_ea(cpu, mode), alu_ld16(cpu, get_d(cpu)));
                return 0;
            case 0xE: /* LDX / LDU */
                if (b_block) {
                    cpu->u = alu_ld16(cpu, load_operand16(cpu, mode));
                } else {
                    cpu->x = alu_ld16(cpu, load_operand16(cpu, mode));
                }
                return 0;
            default: /* 0xF: STX / STU (no immediate form) */
                if (mode == 0) break;
                {
                    uint16_t ea = operand_ea(cpu, mode);
                    if (b_block) {
                        mem_write16(cpu, ea, alu_ld16(cpu, cpu->u));
                    } else {
                        mem_write16(cpu, ea, alu_ld16(cpu, cpu->x));
                    }
                }
                return 0;
        }
    }

    return illegal(cpu);
}

/* ----------------------------------------------------------- print state */

void hd6309_print_state(void *context) {
    if (!context) return;
    HD6309_CPU *cpu = (HD6309_CPU*)context;

    printf("HD6309 State (native mode):\n");
    printf("  PC: $%04X  D: $%02X%02X (A=$%02X B=$%02X)  W: $%02X%02X (E=$%02X F=$%02X)\n",
           cpu->pc, cpu->a, cpu->b, cpu->a, cpu->b, cpu->e, cpu->f, cpu->e, cpu->f);
    printf("  Q:  $%08X  V: $%04X\n", get_q(cpu), cpu->v);
    printf("  X:  $%04X  Y: $%04X  U: $%04X  S: $%04X  DP: $%02X\n",
           cpu->x, cpu->y, cpu->u, cpu->s, cpu->dp);
    printf("  CC: $%02X [%c%c%c%c%c%c%c%c]  MD: $%02X [%s%s%s%s]\n",
           cpu->cc,
           (cpu->cc & CC_E) ? 'E' : '-',
           (cpu->cc & CC_F) ? 'F' : '-',
           (cpu->cc & CC_H) ? 'H' : '-',
           (cpu->cc & CC_I) ? 'I' : '-',
           (cpu->cc & CC_N) ? 'N' : '-',
           (cpu->cc & CC_Z) ? 'Z' : '-',
           (cpu->cc & CC_V) ? 'V' : '-',
           (cpu->cc & CC_C) ? 'C' : '-',
           cpu->md,
           (cpu->md & MD_DZ) ? "DZ " : "",
           (cpu->md & MD_IL) ? "IL " : "",
           (cpu->md & MD_FM) ? "FM " : "",
           (cpu->md & MD_NM) ? "NM" : "EM");
    printf("  Ticks: %u  Halted: %s\n", cpu->ticks, cpu->halted ? "yes" : "no");
}

/* ----------------------------------------------------------- disassembly */

/* Addressing modes for the disassembler */
enum {
    M_ILL, M_INH, M_IMM8, M_IMM16, M_IMM32, M_DIR, M_IDX, M_EXT,
    M_REL8, M_REL16, M_STK, M_TFR
};

static const char *g_stack_regs_s[8] = { "CC", "A", "B", "DP", "X", "Y", "U", "PC" };
static const char *g_stack_regs_u[8] = { "CC", "A", "B", "DP", "X", "Y", "S", "PC" };
static const char *g_tfr_regs[16] = {
    "D", "X", "Y", "U", "S", "PC", "W", "V",
    "A", "B", "CC", "DP", "0", "0", "E", "F"
};
static const char *g_tfm_regs[16] = {
    "D", "X", "Y", "U", "S", "?", "?", "?",
    "?", "?", "?", "?", "?", "?", "?", "?"
};
static const char *g_bit_regs[4] = { "CC", "A", "B", "?" };

static const char *g_unary_names[16] = {
    "NEG", "OIM", "AIM", "COM", "LSR", "EIM", "ROR", "ASR",
    "ASL", "ROL", "DEC", "TIM", "INC", "TST", "JMP", "CLR"
};

static const char *g_branch_names[16] = {
    "BRA", "BRN", "BHI", "BLS", "BCC", "BCS", "BNE", "BEQ",
    "BVC", "BVS", "BPL", "BMI", "BGE", "BLT", "BGT", "BLE"
};

static const char *g_interreg_names[8] = {
    "ADDR", "ADCR", "SUBR", "SBCR", "ANDR", "ORR", "EORR", "CMPR"
};

/* Column names for opcode rows 0x80-0xFF (A block, B block) */
static const char *g_col_names_a[16] = {
    "SUBA", "CMPA", "SBCA", "SUBD", "ANDA", "BITA", "LDA", "STA",
    "EORA", "ADCA", "ORA", "ADDA", "CMPX", "JSR", "LDX", "STX"
};
static const char *g_col_names_b[16] = {
    "SUBB", "CMPB", "SBCB", "ADDD", "ANDB", "BITB", "LDB", "STB",
    "EORB", "ADCB", "ORB", "ADDB", "LDD", "STD", "LDU", "STU"
};

/* 16-bit columns per block (operand is a word in immediate mode) */
static int col_is_16bit(int b_block, int col) {
    if (col == 0x3) return 1;
    if (col >= 0xC) return (b_block || col != 0xD); /* A-block 0xD is JSR/BSR */
    return 0;
}

static size_t dis_indexed(HD6309_CPU *cpu, uint16_t addr, char *buf, size_t buf_len) {
    uint8_t pb = mem_read(cpu, addr);
    static const char *regs[4] = { "X", "Y", "U", "S" };
    const char *r = regs[(pb >> 5) & 0x03];
    int ind = (pb & 0x80) && (pb & 0x10);
    const char *lb = ind ? "[" : "";
    const char *rb = ind ? "]" : "";
    size_t used = 1;

    if (!(pb & 0x80)) {
        int8_t off = (int8_t)(pb & 0x1F);
        if (off & 0x10) off = (int8_t)(off | 0xE0);
        snprintf(buf, buf_len, "%d,%s", off, r);
        return used;
    }
    switch (pb & 0x0F) {
        case 0x0: snprintf(buf, buf_len, ",%s+", r); break;
        case 0x1: snprintf(buf, buf_len, "%s,%s++%s", lb, r, rb); break;
        case 0x2: snprintf(buf, buf_len, ",-%s", r); break;
        case 0x3: snprintf(buf, buf_len, "%s,--%s%s", lb, r, rb); break;
        case 0x4: snprintf(buf, buf_len, "%s,%s%s", lb, r, rb); break;
        case 0x5: snprintf(buf, buf_len, "%sB,%s%s", lb, r, rb); break;
        case 0x6: snprintf(buf, buf_len, "%sA,%s%s", lb, r, rb); break;
        case 0x7: snprintf(buf, buf_len, "%sE,%s%s", lb, r, rb); break;
        case 0x8:
            snprintf(buf, buf_len, "%s%d,%s%s", lb, (int8_t)mem_read(cpu, (uint16_t)(addr + 1)), r, rb);
            used = 2;
            break;
        case 0x9:
            snprintf(buf, buf_len, "%s$%04X,%s%s", lb, mem_read16(cpu, (uint16_t)(addr + 1)), r, rb);
            used = 3;
            break;
        case 0xA: snprintf(buf, buf_len, "%sF,%s%s", lb, r, rb); break;
        case 0xB: snprintf(buf, buf_len, "%sD,%s%s", lb, r, rb); break;
        case 0xC:
            snprintf(buf, buf_len, "%s%d,PCR%s", lb, (int8_t)mem_read(cpu, (uint16_t)(addr + 1)), rb);
            used = 2;
            break;
        case 0xD:
            snprintf(buf, buf_len, "%s$%04X,PCR%s", lb, mem_read16(cpu, (uint16_t)(addr + 1)), rb);
            used = 3;
            break;
        case 0xE: snprintf(buf, buf_len, "%sW,%s%s", lb, r, rb); break;
        default:
            snprintf(buf, buf_len, "[$%04X]", mem_read16(cpu, (uint16_t)(addr + 1)));
            used = 3;
            break;
    }
    return used;
}

static void dis_stack_list(uint8_t pb, const char *const *names, char *buf, size_t buf_len) {
    buf[0] = '\0';
    int first = 1;
    for (int i = 0; i < 8; ++i) {
        if (pb & (1 << i)) {
            size_t len = strlen(buf);
            snprintf(buf + len, buf_len - len, "%s%s", first ? "" : ",", names[i]);
            first = 0;
        }
    }
    if (first) {
        snprintf(buf, buf_len, "(none)");
    }
}

static void dis_format(HD6309_CPU *cpu, uint16_t addr, const char *name, int mode,
                       int stack_is_u, char *buf, size_t buf_len) {
    char opbuf[48];
    switch (mode) {
        case M_INH:
            snprintf(buf, buf_len, "%s", name);
            break;
        case M_IMM8:
            snprintf(buf, buf_len, "%s #$%02X", name, mem_read(cpu, addr));
            break;
        case M_IMM16:
            snprintf(buf, buf_len, "%s #$%04X", name, mem_read16(cpu, addr));
            break;
        case M_IMM32:
            snprintf(buf, buf_len, "%s #$%08X", name, mem_read32(cpu, addr));
            break;
        case M_DIR:
            snprintf(buf, buf_len, "%s <$%02X", name, mem_read(cpu, addr));
            break;
        case M_IDX:
            (void)dis_indexed(cpu, addr, opbuf, sizeof(opbuf));
            snprintf(buf, buf_len, "%s %s", name, opbuf);
            break;
        case M_EXT:
            snprintf(buf, buf_len, "%s $%04X", name, mem_read16(cpu, addr));
            break;
        case M_REL8:
            {
                int8_t off = (int8_t)mem_read(cpu, addr);
                snprintf(buf, buf_len, "%s $%04X", name, (uint16_t)(addr + 1 + off));
            }
            break;
        case M_REL16:
            {
                int16_t off = (int16_t)mem_read16(cpu, addr);
                snprintf(buf, buf_len, "%s $%04X", name, (uint16_t)(addr + 2 + off));
            }
            break;
        case M_STK:
            dis_stack_list(mem_read(cpu, addr),
                           stack_is_u ? g_stack_regs_u : g_stack_regs_s,
                           opbuf, sizeof(opbuf));
            snprintf(buf, buf_len, "%s %s", name, opbuf);
            break;
        case M_TFR:
            {
                uint8_t pb = mem_read(cpu, addr);
                snprintf(buf, buf_len, "%s %s,%s", name,
                         g_tfr_regs[(pb >> 4) & 0x0F], g_tfr_regs[pb & 0x0F]);
            }
            break;
        default:
            snprintf(buf, buf_len, "??? ($%02X)", mem_read(cpu, (uint16_t)(addr - 1)));
            break;
    }
}

/* OIM/AIM/EIM/TIM: "#$imm," followed by the direct/indexed/extended operand */
static void dis_memimm(HD6309_CPU *cpu, uint16_t addr, const char *name, int mode,
                       char *buf, size_t buf_len) {
    uint8_t imm = mem_read(cpu, addr);
    char opbuf[48];
    switch (mode) {
        case M_DIR:
            snprintf(buf, buf_len, "%s #$%02X,<$%02X", name, imm, mem_read(cpu, (uint16_t)(addr + 1)));
            break;
        case M_IDX:
            (void)dis_indexed(cpu, (uint16_t)(addr + 1), opbuf, sizeof(opbuf));
            snprintf(buf, buf_len, "%s #$%02X,%s", name, imm, opbuf);
            break;
        default: /* M_EXT */
            snprintf(buf, buf_len, "%s #$%02X,$%04X", name, imm, mem_read16(cpu, (uint16_t)(addr + 1)));
            break;
    }
}

static void dis_page2(HD6309_CPU *cpu, uint16_t opnd, char *body, size_t body_len) {
    uint8_t op2 = mem_read(cpu, opnd);
    int mode2 = (op2 >> 4) & 0x03;
    static const int op_modes[4] = { M_IMM16, M_DIR, M_IDX, M_EXT };
    opnd++;

    if (op2 >= 0x21 && op2 <= 0x2F) {
        char lname[8];
        snprintf(lname, sizeof(lname), "L%s", g_branch_names[op2 & 0x0F]);
        dis_format(cpu, opnd, lname, M_REL16, 0, body, body_len);
        return;
    }
    if (op2 >= 0x30 && op2 <= 0x37) {
        dis_format(cpu, opnd, g_interreg_names[op2 & 0x07], M_TFR, 0, body, body_len);
        return;
    }
    switch (op2) {
        case 0x38: snprintf(body, body_len, "PSHSW"); return;
        case 0x39: snprintf(body, body_len, "PULSW"); return;
        case 0x3A: snprintf(body, body_len, "PSHUW"); return;
        case 0x3B: snprintf(body, body_len, "PULUW"); return;
        case 0x3F: snprintf(body, body_len, "SWI2"); return;
        default: break;
    }
    if (op2 >= 0x40 && op2 <= 0x5F) {
        int sub = op2 & 0x0F;
        const char *name = g_unary_names[sub];
        int d_form = (op2 <= 0x4F);
        int ok;
        if (sub == 0x1 || sub == 0x2 || sub == 0x5 || sub == 0xB || sub == 0xE) {
            ok = 0;
        } else if (d_form) {
            ok = 1;
        } else {
            ok = (sub == 0x3 || sub == 0x4 || sub == 0x6 || sub == 0x9 ||
                  sub == 0xA || sub == 0xC || sub == 0xD || sub == 0xF);
        }
        if (ok) {
            snprintf(body, body_len, "%s%c", name, d_form ? 'D' : 'W');
            return;
        }
    }
    if (op2 >= 0x80) {
        const char *name = NULL;
        switch (op2 & 0xCF) {
            case 0x80: name = "SUBW"; break;
            case 0x81: name = "CMPW"; break;
            case 0x82: name = "SBCD"; break;
            case 0x83: name = "CMPD"; break;
            case 0x84: name = "ANDD"; break;
            case 0x85: name = "BITD"; break;
            case 0x86: name = "LDW"; break;
            case 0x87: name = (mode2 != 0) ? "STW" : NULL; break;
            case 0x88: name = "EORD"; break;
            case 0x89: name = "ADCD"; break;
            case 0x8A: name = "ORD"; break;
            case 0x8B: name = "ADDW"; break;
            case 0x8C: name = "CMPY"; break;
            case 0x8E: name = "LDY"; break;
            case 0x8F: name = (mode2 != 0) ? "STY" : NULL; break;
            case 0xCC: name = (mode2 != 0) ? "LDQ" : NULL; break;
            case 0xCD: name = (mode2 != 0) ? "STQ" : NULL; break;
            case 0xCE: name = "LDS"; break;
            case 0xCF: name = "STS"; break;
            default: break;
        }
        if (name) {
            dis_format(cpu, opnd, name, op_modes[mode2], 0, body, body_len);
            return;
        }
    }
    snprintf(body, body_len, "??? ($10 $%02X)", op2);
}

static void dis_page3(HD6309_CPU *cpu, uint16_t opnd, char *body, size_t body_len) {
    uint8_t op2 = mem_read(cpu, opnd);
    int mode2 = (op2 >> 4) & 0x03;
    static const int op_modes8[4] = { M_IMM8, M_DIR, M_IDX, M_EXT };
    static const int op_modes16[4] = { M_IMM16, M_DIR, M_IDX, M_EXT };
    opnd++;

    switch (op2) {
        case 0x30: case 0x32: case 0x34:
            {
                static const char *names[3] = { "BAND", "BOR", "BEOR" };
                uint8_t pb = mem_read(cpu, opnd);
                snprintf(body, body_len, "%s %s,%u,%u,<$%02X",
                         names[(op2 - 0x30) >> 1],
                         g_bit_regs[(pb >> 6) & 0x03],
                         (unsigned)((pb >> 3) & 0x07),
                         (unsigned)(pb & 0x07),
                         mem_read(cpu, (uint16_t)(opnd + 1)));
            }
            return;
        case 0x38: case 0x39: case 0x3A: case 0x3B:
            {
                static const char *sfx0[4] = { "+", "-", "+", "" };
                static const char *sfx1[4] = { "+", "-", "", "+" };
                uint8_t pb = mem_read(cpu, opnd);
                int var = op2 - 0x38;
                snprintf(body, body_len, "TFM %s%s,%s%s",
                         g_tfm_regs[(pb >> 4) & 0x0F], sfx0[var],
                         g_tfm_regs[pb & 0x0F], sfx1[var]);
            }
            return;
        case 0x3C:
            dis_format(cpu, opnd, "BITMD", M_IMM8, 0, body, body_len);
            return;
        case 0x3D:
            dis_format(cpu, opnd, "LDMD", M_IMM8, 0, body, body_len);
            return;
        case 0x3F:
            snprintf(body, body_len, "SWI3");
            return;
        default:
            break;
    }
    if (op2 >= 0x40 && op2 <= 0x5F) {
        int sub = op2 & 0x0F;
        if (sub == 0x3 || sub == 0xA || sub == 0xC || sub == 0xD || sub == 0xF) {
            snprintf(body, body_len, "%s%c", g_unary_names[sub], (op2 <= 0x4F) ? 'E' : 'F');
            return;
        }
    }
    if (op2 >= 0x80) {
        int e_block = (op2 & 0x40) == 0;
        int col = op2 & 0x0F;
        char acc = e_block ? 'E' : 'F';
        char namebuf[8];
        const char *name = NULL;
        int is16 = 0;

        switch (col) {
            case 0x0: snprintf(namebuf, sizeof(namebuf), "SUB%c", acc); name = namebuf; break;
            case 0x1: snprintf(namebuf, sizeof(namebuf), "CMP%c", acc); name = namebuf; break;
            case 0x3: if (e_block) { name = "CMPU"; is16 = 1; } break;
            case 0x6: snprintf(namebuf, sizeof(namebuf), "LD%c", acc); name = namebuf; break;
            case 0x7:
                if (mode2 != 0) {
                    snprintf(namebuf, sizeof(namebuf), "ST%c", acc);
                    name = namebuf;
                }
                break;
            case 0xB: snprintf(namebuf, sizeof(namebuf), "ADD%c", acc); name = namebuf; break;
            case 0xC: if (e_block) { name = "CMPS"; is16 = 1; } break;
            case 0xD: if (e_block) { name = "DIVD"; } break;
            case 0xE: if (e_block) { name = "DIVQ"; is16 = 1; } break;
            case 0xF: if (e_block) { name = "MULD"; is16 = 1; } break;
            default: break;
        }
        if (name) {
            dis_format(cpu, opnd, name, is16 ? op_modes16[mode2] : op_modes8[mode2],
                       0, body, body_len);
            return;
        }
    }
    snprintf(body, body_len, "??? ($11 $%02X)", op2);
}

void hd6309_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    HD6309_CPU *cpu = (HD6309_CPU*)context;

    uint16_t pc = cpu->pc;
    uint8_t op = mem_read(cpu, pc);
    uint16_t opnd = (uint16_t)(pc + 1);
    char body[64];
    body[0] = '\0';

    if (op == 0x10) {
        dis_page2(cpu, opnd, body, sizeof(body));
    } else if (op == 0x11) {
        dis_page3(cpu, opnd, body, sizeof(body));
    } else if (op <= 0x0F) {
        int sub = op & 0x0F;
        const char *name = g_unary_names[sub];
        if (sub == 0x1 || sub == 0x2 || sub == 0x5 || sub == 0xB) {
            dis_memimm(cpu, opnd, name, M_DIR, body, sizeof(body));
        } else {
            dis_format(cpu, opnd, name, M_DIR, 0, body, sizeof(body));
        }
    } else if (op <= 0x1F) {
        switch (op) {
            case 0x12: snprintf(body, sizeof(body), "NOP"); break;
            case 0x13: snprintf(body, sizeof(body), "SYNC"); break;
            case 0x14: snprintf(body, sizeof(body), "SEXW"); break;
            case 0x16: dis_format(cpu, opnd, "LBRA", M_REL16, 0, body, sizeof(body)); break;
            case 0x17: dis_format(cpu, opnd, "LBSR", M_REL16, 0, body, sizeof(body)); break;
            case 0x19: snprintf(body, sizeof(body), "DAA"); break;
            case 0x1A: dis_format(cpu, opnd, "ORCC", M_IMM8, 0, body, sizeof(body)); break;
            case 0x1C: dis_format(cpu, opnd, "ANDCC", M_IMM8, 0, body, sizeof(body)); break;
            case 0x1D: snprintf(body, sizeof(body), "SEX"); break;
            case 0x1E: dis_format(cpu, opnd, "EXG", M_TFR, 0, body, sizeof(body)); break;
            case 0x1F: dis_format(cpu, opnd, "TFR", M_TFR, 0, body, sizeof(body)); break;
            default: break;
        }
    } else if (op <= 0x2F) {
        dis_format(cpu, opnd, g_branch_names[op & 0x0F], M_REL8, 0, body, sizeof(body));
    } else if (op <= 0x3F) {
        switch (op) {
            case 0x30: dis_format(cpu, opnd, "LEAX", M_IDX, 0, body, sizeof(body)); break;
            case 0x31: dis_format(cpu, opnd, "LEAY", M_IDX, 0, body, sizeof(body)); break;
            case 0x32: dis_format(cpu, opnd, "LEAS", M_IDX, 0, body, sizeof(body)); break;
            case 0x33: dis_format(cpu, opnd, "LEAU", M_IDX, 0, body, sizeof(body)); break;
            case 0x34: dis_format(cpu, opnd, "PSHS", M_STK, 0, body, sizeof(body)); break;
            case 0x35: dis_format(cpu, opnd, "PULS", M_STK, 0, body, sizeof(body)); break;
            case 0x36: dis_format(cpu, opnd, "PSHU", M_STK, 1, body, sizeof(body)); break;
            case 0x37: dis_format(cpu, opnd, "PULU", M_STK, 1, body, sizeof(body)); break;
            case 0x39: snprintf(body, sizeof(body), "RTS"); break;
            case 0x3A: snprintf(body, sizeof(body), "ABX"); break;
            case 0x3B: snprintf(body, sizeof(body), "RTI"); break;
            case 0x3C: dis_format(cpu, opnd, "CWAI", M_IMM8, 0, body, sizeof(body)); break;
            case 0x3D: snprintf(body, sizeof(body), "MUL"); break;
            case 0x3F: snprintf(body, sizeof(body), "SWI"); break;
            default: break;
        }
    } else if (op <= 0x5F) {
        int sub = op & 0x0F;
        if (sub != 0x1 && sub != 0x2 && sub != 0x5 && sub != 0xB && sub != 0xE) {
            snprintf(body, sizeof(body), "%s%c", g_unary_names[sub], (op <= 0x4F) ? 'A' : 'B');
        }
    } else if (op <= 0x7F) {
        int sub = op & 0x0F;
        const char *name = g_unary_names[sub];
        int m = (op <= 0x6F) ? M_IDX : M_EXT;
        if (sub == 0x1 || sub == 0x2 || sub == 0x5 || sub == 0xB) {
            dis_memimm(cpu, opnd, name, m, body, sizeof(body));
        } else {
            dis_format(cpu, opnd, name, m, 0, body, sizeof(body));
        }
    } else if (op == 0xCD) {
        dis_format(cpu, opnd, "LDQ", M_IMM32, 0, body, sizeof(body));
    } else {
        int b_block = (op & 0x40) != 0;
        int mode = (op >> 4) & 0x03;
        int col = op & 0x0F;
        const char *name = b_block ? g_col_names_b[col] : g_col_names_a[col];
        int illegal_form = 0;

        if (mode == 0 && (col == 0x7 || col == 0xF || (b_block && col == 0xD))) {
            illegal_form = 1; /* stores have no immediate form */
        }
        if (!b_block && col == 0xD && mode == 0) {
            name = "BSR";
        }
        if (!illegal_form) {
            int fmt_mode;
            if (!b_block && col == 0xD && mode == 0) {
                fmt_mode = M_REL8;
            } else if (mode == 0) {
                fmt_mode = col_is_16bit(b_block, col) ? M_IMM16 : M_IMM8;
            } else {
                static const int op_modes[4] = { M_IMM8, M_DIR, M_IDX, M_EXT };
                fmt_mode = op_modes[mode];
            }
            dis_format(cpu, opnd, name, fmt_mode, 0, body, sizeof(body));
        }
    }

    if (body[0] == '\0') {
        snprintf(body, sizeof(body), "??? ($%02X)", op);
    }
    snprintf(buf, buf_len, "$%04X: %s", pc, body);
}
