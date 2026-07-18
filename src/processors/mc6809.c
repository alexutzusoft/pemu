#include "mc6809.h"
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

typedef struct MC6809_CPU {
    uint8_t ram[65536];
    uint8_t a;
    uint8_t b;
    uint8_t dp;
    uint8_t cc;
    uint16_t x;
    uint16_t y;
    uint16_t u;
    uint16_t s;
    uint16_t pc;
    uint32_t ticks;
    int halted;
} MC6809_CPU;

#define SET_FLAG(cond, flag) do { if (cond) cpu->cc |= (flag); else cpu->cc &= ~(uint8_t)(flag); } while (0)
#define GET_FLAG(flag) ((cpu->cc & (flag)) ? 1 : 0)

/* ---------------------------------------------------------------- memory */

static uint8_t mem_read(MC6809_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr];
}

static void mem_write(MC6809_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr] = val;
}

static uint16_t mem_read16(MC6809_CPU *cpu, uint16_t addr) {
    return (uint16_t)(((uint16_t)mem_read(cpu, addr) << 8) | mem_read(cpu, (uint16_t)(addr + 1)));
}

static void mem_write16(MC6809_CPU *cpu, uint16_t addr, uint16_t val) {
    mem_write(cpu, addr, (uint8_t)(val >> 8));
    mem_write(cpu, (uint16_t)(addr + 1), (uint8_t)(val & 0xFF));
}

static uint8_t fetch8(MC6809_CPU *cpu) {
    uint8_t v = mem_read(cpu, cpu->pc);
    cpu->pc++;
    return v;
}

static uint16_t fetch16(MC6809_CPU *cpu) {
    uint16_t v = mem_read16(cpu, cpu->pc);
    cpu->pc += 2;
    return v;
}

static uint16_t get_d(MC6809_CPU *cpu) {
    return (uint16_t)(((uint16_t)cpu->a << 8) | cpu->b);
}

static void set_d(MC6809_CPU *cpu, uint16_t val) {
    cpu->a = (uint8_t)(val >> 8);
    cpu->b = (uint8_t)(val & 0xFF);
}

/* ---------------------------------------------------------------- stacks */

static void push8_s(MC6809_CPU *cpu, uint8_t val) {
    cpu->s--;
    mem_write(cpu, cpu->s, val);
}

static uint8_t pull8_s(MC6809_CPU *cpu) {
    uint8_t v = mem_read(cpu, cpu->s);
    cpu->s++;
    return v;
}

static void push16_s(MC6809_CPU *cpu, uint16_t val) {
    push8_s(cpu, (uint8_t)(val & 0xFF));
    push8_s(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16_s(MC6809_CPU *cpu) {
    uint16_t hi = pull8_s(cpu);
    uint16_t lo = pull8_s(cpu);
    return (uint16_t)((hi << 8) | lo);
}

static void push8_u(MC6809_CPU *cpu, uint8_t val) {
    cpu->u--;
    mem_write(cpu, cpu->u, val);
}

static uint8_t pull8_u(MC6809_CPU *cpu) {
    uint8_t v = mem_read(cpu, cpu->u);
    cpu->u++;
    return v;
}

static void push16_u(MC6809_CPU *cpu, uint16_t val) {
    push8_u(cpu, (uint8_t)(val & 0xFF));
    push8_u(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16_u(MC6809_CPU *cpu) {
    uint16_t hi = pull8_u(cpu);
    uint16_t lo = pull8_u(cpu);
    return (uint16_t)((hi << 8) | lo);
}

/* ---------------------------------------------------------------- flags */

static void set_nz8(MC6809_CPU *cpu, uint8_t val) {
    SET_FLAG(val & 0x80, CC_N);
    SET_FLAG(val == 0, CC_Z);
}

static void set_nz16(MC6809_CPU *cpu, uint16_t val) {
    SET_FLAG(val & 0x8000, CC_N);
    SET_FLAG(val == 0, CC_Z);
}

/* ---------------------------------------------------------------- ALU */

static uint8_t alu_add8(MC6809_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t sum = (uint16_t)lhs + rhs + carry_in;
    uint8_t res = (uint8_t)sum;
    SET_FLAG(((lhs & 0x0F) + (rhs & 0x0F) + carry_in) & 0x10, CC_H);
    SET_FLAG(sum & 0x100, CC_C);
    SET_FLAG((~(lhs ^ rhs) & (lhs ^ res)) & 0x80, CC_V);
    set_nz8(cpu, res);
    return res;
}

static uint8_t alu_sub8(MC6809_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t diff = (uint16_t)lhs - rhs - carry_in;
    uint8_t res = (uint8_t)diff;
    SET_FLAG(diff & 0x100, CC_C);
    SET_FLAG(((lhs ^ rhs) & (lhs ^ res)) & 0x80, CC_V);
    set_nz8(cpu, res);
    return res;
}

static uint16_t alu_add16(MC6809_CPU *cpu, uint16_t lhs, uint16_t rhs) {
    uint32_t sum = (uint32_t)lhs + rhs;
    uint16_t res = (uint16_t)sum;
    SET_FLAG(sum & 0x10000, CC_C);
    SET_FLAG((~(lhs ^ rhs) & (lhs ^ res)) & 0x8000, CC_V);
    set_nz16(cpu, res);
    return res;
}

static uint16_t alu_sub16(MC6809_CPU *cpu, uint16_t lhs, uint16_t rhs) {
    uint32_t diff = (uint32_t)lhs - rhs;
    uint16_t res = (uint16_t)diff;
    SET_FLAG(diff & 0x10000, CC_C);
    SET_FLAG(((lhs ^ rhs) & (lhs ^ res)) & 0x8000, CC_V);
    set_nz16(cpu, res);
    return res;
}

static uint8_t alu_logic8(MC6809_CPU *cpu, uint8_t res) {
    set_nz8(cpu, res);
    cpu->cc &= ~(uint8_t)CC_V;
    return res;
}

static uint16_t alu_ld16(MC6809_CPU *cpu, uint16_t val) {
    set_nz16(cpu, val);
    cpu->cc &= ~(uint8_t)CC_V;
    return val;
}

/*
 * Unary (read-modify-write) group shared by opcode columns 0x0n/0x4n/0x5n/0x6n/0x7n.
 * sub is the low nibble of the opcode. *writeback is set to 1 if the result
 * must be stored, 0 for TST, and -1 for an illegal column.
 */
static uint8_t do_unary(MC6809_CPU *cpu, int sub, uint8_t val, int *writeback) {
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

/* ------------------------------------------------------- effective address */

static uint16_t direct_ea(MC6809_CPU *cpu) {
    return (uint16_t)(((uint16_t)cpu->dp << 8) | fetch8(cpu));
}

static uint16_t indexed_ea(MC6809_CPU *cpu) {
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
        case 0x8: /* n8,R */
            ea = (uint16_t)(*reg + (int8_t)fetch8(cpu));
            cpu->ticks += 1;
            break;
        case 0x9: /* n16,R */
            ea = (uint16_t)(*reg + fetch16(cpu));
            cpu->ticks += 4;
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
        case 0xF: /* [n16] extended indirect */
            ea = fetch16(cpu);
            cpu->ticks += 2;
            break;
        default:
            ea = *reg;
            break;
    }

    if (pb & 0x10) {
        ea = mem_read16(cpu, ea);
        cpu->ticks += 3;
    }
    return ea;
}

/* mode: 0 = immediate, 1 = direct, 2 = indexed, 3 = extended */
static uint16_t operand_ea(MC6809_CPU *cpu, int mode) {
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

static uint8_t load_operand8(MC6809_CPU *cpu, int mode) {
    if (mode == 0) {
        cpu->ticks += 1;
        return fetch8(cpu);
    }
    return mem_read(cpu, operand_ea(cpu, mode));
}

static uint16_t load_operand16(MC6809_CPU *cpu, int mode) {
    if (mode == 0) {
        cpu->ticks += 2;
        return fetch16(cpu);
    }
    return mem_read16(cpu, operand_ea(cpu, mode));
}

/* ------------------------------------------------------------ TFR / EXG */

static uint16_t get_reg_by_code(MC6809_CPU *cpu, uint8_t code) {
    switch (code & 0x0F) {
        case 0x0: return get_d(cpu);
        case 0x1: return cpu->x;
        case 0x2: return cpu->y;
        case 0x3: return cpu->u;
        case 0x4: return cpu->s;
        case 0x5: return cpu->pc;
        case 0x8: return (uint16_t)(0xFF00 | cpu->a);
        case 0x9: return (uint16_t)(0xFF00 | cpu->b);
        case 0xA: return (uint16_t)(0xFF00 | cpu->cc);
        case 0xB: return (uint16_t)(0xFF00 | cpu->dp);
        default:  return 0xFFFF;
    }
}

static void set_reg_by_code(MC6809_CPU *cpu, uint8_t code, uint16_t val) {
    switch (code & 0x0F) {
        case 0x0: set_d(cpu, val); break;
        case 0x1: cpu->x = val; break;
        case 0x2: cpu->y = val; break;
        case 0x3: cpu->u = val; break;
        case 0x4: cpu->s = val; break;
        case 0x5: cpu->pc = val; break;
        case 0x8: cpu->a = (uint8_t)(val & 0xFF); break;
        case 0x9: cpu->b = (uint8_t)(val & 0xFF); break;
        case 0xA: cpu->cc = (uint8_t)(val & 0xFF); break;
        case 0xB: cpu->dp = (uint8_t)(val & 0xFF); break;
        default: break;
    }
}

/* ------------------------------------------------------- PSH / PUL groups */

static void do_pshs(MC6809_CPU *cpu, uint8_t pb) {
    if (pb & 0x80) { push16_s(cpu, cpu->pc); cpu->ticks += 2; }
    if (pb & 0x40) { push16_s(cpu, cpu->u);  cpu->ticks += 2; }
    if (pb & 0x20) { push16_s(cpu, cpu->y);  cpu->ticks += 2; }
    if (pb & 0x10) { push16_s(cpu, cpu->x);  cpu->ticks += 2; }
    if (pb & 0x08) { push8_s(cpu, cpu->dp);  cpu->ticks += 1; }
    if (pb & 0x04) { push8_s(cpu, cpu->b);   cpu->ticks += 1; }
    if (pb & 0x02) { push8_s(cpu, cpu->a);   cpu->ticks += 1; }
    if (pb & 0x01) { push8_s(cpu, cpu->cc);  cpu->ticks += 1; }
}

static void do_puls(MC6809_CPU *cpu, uint8_t pb) {
    if (pb & 0x01) { cpu->cc = pull8_s(cpu);  cpu->ticks += 1; }
    if (pb & 0x02) { cpu->a = pull8_s(cpu);   cpu->ticks += 1; }
    if (pb & 0x04) { cpu->b = pull8_s(cpu);   cpu->ticks += 1; }
    if (pb & 0x08) { cpu->dp = pull8_s(cpu);  cpu->ticks += 1; }
    if (pb & 0x10) { cpu->x = pull16_s(cpu);  cpu->ticks += 2; }
    if (pb & 0x20) { cpu->y = pull16_s(cpu);  cpu->ticks += 2; }
    if (pb & 0x40) { cpu->u = pull16_s(cpu);  cpu->ticks += 2; }
    if (pb & 0x80) { cpu->pc = pull16_s(cpu); cpu->ticks += 2; }
}

static void do_pshu(MC6809_CPU *cpu, uint8_t pb) {
    if (pb & 0x80) { push16_u(cpu, cpu->pc); cpu->ticks += 2; }
    if (pb & 0x40) { push16_u(cpu, cpu->s);  cpu->ticks += 2; }
    if (pb & 0x20) { push16_u(cpu, cpu->y);  cpu->ticks += 2; }
    if (pb & 0x10) { push16_u(cpu, cpu->x);  cpu->ticks += 2; }
    if (pb & 0x08) { push8_u(cpu, cpu->dp);  cpu->ticks += 1; }
    if (pb & 0x04) { push8_u(cpu, cpu->b);   cpu->ticks += 1; }
    if (pb & 0x02) { push8_u(cpu, cpu->a);   cpu->ticks += 1; }
    if (pb & 0x01) { push8_u(cpu, cpu->cc);  cpu->ticks += 1; }
}

static void do_pulu(MC6809_CPU *cpu, uint8_t pb) {
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

static void do_swi(MC6809_CPU *cpu, uint16_t vector, int set_mask) {
    cpu->cc |= CC_E;
    push16_s(cpu, cpu->pc);
    push16_s(cpu, cpu->u);
    push16_s(cpu, cpu->y);
    push16_s(cpu, cpu->x);
    push8_s(cpu, cpu->dp);
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

static int branch_taken(MC6809_CPU *cpu, uint8_t cond) {
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

/* --------------------------------------------------------- page 2 (0x10) */

static int step_page2(MC6809_CPU *cpu) {
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

    if (op == 0x3F) { /* SWI2 */
        do_swi(cpu, 0xFFF4, 0);
        return 0;
    }

    if (op >= 0x80) {
        int mode = (op >> 4) & 0x03;
        switch (op & 0xCF) {
            case 0x83: /* CMPD */
                (void)alu_sub16(cpu, get_d(cpu), load_operand16(cpu, mode));
                cpu->ticks += 4;
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

    cpu->halted = 1;
    return 1;
}

/* --------------------------------------------------------- page 3 (0x11) */

static int step_page3(MC6809_CPU *cpu) {
    uint8_t op = fetch8(cpu);
    cpu->ticks += 1;

    if (op == 0x3F) { /* SWI3 */
        do_swi(cpu, 0xFFF2, 0);
        return 0;
    }

    if (op >= 0x80) {
        int mode = (op >> 4) & 0x03;
        switch (op & 0xCF) {
            case 0x83: /* CMPU */
                (void)alu_sub16(cpu, cpu->u, load_operand16(cpu, mode));
                cpu->ticks += 4;
                return 0;
            case 0x8C: /* CMPS */
                (void)alu_sub16(cpu, cpu->s, load_operand16(cpu, mode));
                cpu->ticks += 4;
                return 0;
            default:
                break;
        }
    }

    cpu->halted = 1;
    return 1;
}

/* ------------------------------------------------------------- lifecycle */

void* mc6809_create(void) {
    MC6809_CPU *cpu = (MC6809_CPU*)calloc(1, sizeof(MC6809_CPU));
    return cpu;
}

void mc6809_destroy(void *context) {
    free(context);
}

int mc6809_init(void *context) {
    if (!context) return -1;
    MC6809_CPU *cpu = (MC6809_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->b = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->u = 0;
    cpu->s = 0;
    cpu->dp = 0;
    cpu->cc = CC_I | CC_F;
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int mc6809_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    MC6809_CPU *cpu = (MC6809_CPU*)context;

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

int mc6809_step(void *context) {
    if (!context) return -1;
    MC6809_CPU *cpu = (MC6809_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks += 2;

    /* -- 0x00-0x0F: unary read-modify-write, direct mode -- */
    if (op <= 0x0F) {
        uint16_t ea = direct_ea(cpu);
        cpu->ticks += 3;
        if (op == 0x0E) { /* JMP direct */
            cpu->pc = ea;
            return 0;
        }
        int wb;
        uint8_t val = (op == 0x0F) ? 0 : mem_read(cpu, ea);
        uint8_t res = do_unary(cpu, op & 0x0F, val, &wb);
        if (wb < 0) {
            cpu->halted = 1;
            return 1;
        }
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
                cpu->halted = 1;
                return 1;
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
                cpu->halted = 1;
                return 1;
        }
    }

    /* -- 0x40-0x5F: unary on A / B -- */
    if (op <= 0x5F) {
        uint8_t *acc = (op <= 0x4F) ? &cpu->a : &cpu->b;
        int wb;
        uint8_t res = do_unary(cpu, op & 0x0F, *acc, &wb);
        if (wb < 0) {
            cpu->halted = 1;
            return 1;
        }
        if (wb) *acc = res;
        return 0;
    }

    /* -- 0x60-0x7F: unary read-modify-write, indexed / extended -- */
    if (op <= 0x7F) {
        uint16_t ea;
        if (op <= 0x6F) {
            ea = indexed_ea(cpu);
            cpu->ticks += 3;
        } else {
            ea = fetch16(cpu);
            cpu->ticks += 4;
        }
        if ((op & 0x0F) == 0x0E) { /* JMP */
            cpu->pc = ea;
            return 0;
        }
        int wb;
        uint8_t val = ((op & 0x0F) == 0x0F) ? 0 : mem_read(cpu, ea);
        uint8_t res = do_unary(cpu, op & 0x0F, val, &wb);
        if (wb < 0) {
            cpu->halted = 1;
            return 1;
        }
        if (wb) mem_write(cpu, ea, res);
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
                if (mode == 0) break; /* STD immediate is illegal */
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

    cpu->halted = 1;
    return 1;
}

/* ----------------------------------------------------------- print state */

void mc6809_print_state(void *context) {
    if (!context) return;
    MC6809_CPU *cpu = (MC6809_CPU*)context;

    printf("MC6809 State:\n");
    printf("  PC: $%04X  D: $%02X%02X (A=$%02X B=$%02X)\n",
           cpu->pc, cpu->a, cpu->b, cpu->a, cpu->b);
    printf("  X:  $%04X  Y: $%04X  U: $%04X  S: $%04X  DP: $%02X\n",
           cpu->x, cpu->y, cpu->u, cpu->s, cpu->dp);
    printf("  CC: $%02X [%c%c%c%c%c%c%c%c]\n",
           cpu->cc,
           (cpu->cc & CC_E) ? 'E' : '-',
           (cpu->cc & CC_F) ? 'F' : '-',
           (cpu->cc & CC_H) ? 'H' : '-',
           (cpu->cc & CC_I) ? 'I' : '-',
           (cpu->cc & CC_N) ? 'N' : '-',
           (cpu->cc & CC_Z) ? 'Z' : '-',
           (cpu->cc & CC_V) ? 'V' : '-',
           (cpu->cc & CC_C) ? 'C' : '-');
    printf("  Ticks: %u  Halted: %s\n", cpu->ticks, cpu->halted ? "yes" : "no");
}

/* ----------------------------------------------------------- disassembly */

/* Addressing modes for the disassembler */
enum {
    M_ILL, M_INH, M_IMM8, M_IMM16, M_DIR, M_IDX, M_EXT,
    M_REL8, M_REL16, M_STK, M_TFR
};

typedef struct DisInfo {
    const char *name;
    uint8_t mode;
} DisInfo;

static const char *g_stack_regs_s[8] = { "CC", "A", "B", "DP", "X", "Y", "U", "PC" };
static const char *g_stack_regs_u[8] = { "CC", "A", "B", "DP", "X", "Y", "S", "PC" };
static const char *g_tfr_regs[16] = {
    "D", "X", "Y", "U", "S", "PC", "?", "?",
    "A", "B", "CC", "DP", "?", "?", "?", "?"
};

static const char *g_unary_names[16] = {
    "NEG", NULL, NULL, "COM", "LSR", NULL, "ROR", "ASR",
    "ASL", "ROL", "DEC", NULL, "INC", "TST", "JMP", "CLR"
};

static const char *g_branch_names[16] = {
    "BRA", "BRN", "BHI", "BLS", "BCC", "BCS", "BNE", "BEQ",
    "BVC", "BVS", "BPL", "BMI", "BGE", "BLT", "BGT", "BLE"
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

static size_t dis_indexed(MC6809_CPU *cpu, uint16_t addr, char *buf, size_t buf_len) {
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
        case 0x8:
            snprintf(buf, buf_len, "%s%d,%s%s", lb, (int8_t)mem_read(cpu, (uint16_t)(addr + 1)), r, rb);
            used = 2;
            break;
        case 0x9:
            snprintf(buf, buf_len, "%s$%04X,%s%s", lb, mem_read16(cpu, (uint16_t)(addr + 1)), r, rb);
            used = 3;
            break;
        case 0xB: snprintf(buf, buf_len, "%sD,%s%s", lb, r, rb); break;
        case 0xC:
            snprintf(buf, buf_len, "%s%d,PCR%s", lb, (int8_t)mem_read(cpu, (uint16_t)(addr + 1)), rb);
            used = 2;
            break;
        case 0xD:
            snprintf(buf, buf_len, "%s$%04X,PCR%s", lb, mem_read16(cpu, (uint16_t)(addr + 1)), rb);
            used = 3;
            break;
        case 0xF:
            snprintf(buf, buf_len, "[$%04X]", mem_read16(cpu, (uint16_t)(addr + 1)));
            used = 3;
            break;
        default:
            snprintf(buf, buf_len, "?,%s", r);
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

static void dis_format(MC6809_CPU *cpu, uint16_t addr, const char *name, int mode,
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

void mc6809_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    MC6809_CPU *cpu = (MC6809_CPU*)context;

    uint16_t pc = cpu->pc;
    uint8_t op = mem_read(cpu, pc);
    uint16_t opnd = (uint16_t)(pc + 1);
    char body[64];
    body[0] = '\0';

    if (op == 0x10 || op == 0x11) {
        uint8_t op2 = mem_read(cpu, opnd);
        opnd++;
        int mode2 = (op2 >> 4) & 0x03;
        static const int op_modes[4] = { M_IMM16, M_DIR, M_IDX, M_EXT };
        if (op == 0x10) {
            if (op2 >= 0x21 && op2 <= 0x2F) {
                char lname[8];
                snprintf(lname, sizeof(lname), "L%s", g_branch_names[op2 & 0x0F]);
                dis_format(cpu, opnd, lname, M_REL16, 0, body, sizeof(body));
            } else if (op2 == 0x3F) {
                snprintf(body, sizeof(body), "SWI2");
            } else if (op2 >= 0x80) {
                const char *name = NULL;
                switch (op2 & 0xCF) {
                    case 0x83: name = "CMPD"; break;
                    case 0x8C: name = "CMPY"; break;
                    case 0x8E: name = "LDY"; break;
                    case 0x8F: name = (mode2 != 0) ? "STY" : NULL; break;
                    case 0xCE: name = "LDS"; break;
                    case 0xCF: name = (mode2 != 0) ? "STS" : NULL; break;
                    default: break;
                }
                if (name) {
                    dis_format(cpu, opnd, name, op_modes[mode2], 0, body, sizeof(body));
                }
            }
        } else {
            if (op2 == 0x3F) {
                snprintf(body, sizeof(body), "SWI3");
            } else if ((op2 & 0xCF) == 0x83) {
                dis_format(cpu, opnd, "CMPU", op_modes[mode2], 0, body, sizeof(body));
            } else if ((op2 & 0xCF) == 0x8C) {
                dis_format(cpu, opnd, "CMPS", op_modes[mode2], 0, body, sizeof(body));
            }
        }
        if (body[0] == '\0') {
            snprintf(body, sizeof(body), "??? ($%02X $%02X)", op, op2);
        }
    } else if (op <= 0x0F) {
        const char *name = g_unary_names[op & 0x0F];
        if (name) {
            dis_format(cpu, opnd, name, M_DIR, 0, body, sizeof(body));
        }
    } else if (op <= 0x1F) {
        switch (op) {
            case 0x12: snprintf(body, sizeof(body), "NOP"); break;
            case 0x13: snprintf(body, sizeof(body), "SYNC"); break;
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
        const char *name = g_unary_names[op & 0x0F];
        if (name && (op & 0x0F) != 0x0E) {
            snprintf(body, sizeof(body), "%s%c", name, (op <= 0x4F) ? 'A' : 'B');
        }
    } else if (op <= 0x7F) {
        const char *name = g_unary_names[op & 0x0F];
        if (name) {
            dis_format(cpu, opnd, name, (op <= 0x6F) ? M_IDX : M_EXT, 0, body, sizeof(body));
        }
    } else {
        int b_block = (op & 0x40) != 0;
        int mode = (op >> 4) & 0x03;
        int col = op & 0x0F;
        const char *name = b_block ? g_col_names_b[col] : g_col_names_a[col];
        int illegal = 0;

        if (mode == 0 && (col == 0x7 || col == 0xF || (b_block && col == 0xD))) {
            illegal = 1; /* stores have no immediate form */
        }
        if (!b_block && col == 0xD && mode == 0) {
            name = "BSR";
        }
        if (!illegal) {
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
