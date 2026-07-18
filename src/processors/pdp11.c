#include "pdp11.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PDP11_MEM_SIZE 65536
#define PDP11_RESET_PC 01000

#define PSW_C 001
#define PSW_V 002
#define PSW_Z 004
#define PSW_N 010

typedef struct PDP11CPU {
    uint8_t  mem[PDP11_MEM_SIZE]; // Little-endian byte memory
    uint16_t r[8];                // R0-R5, R6=SP, R7=PC
    uint16_t psw;                 // Processor status word (N Z V C)
    uint8_t  halted;
    uint32_t ticks;
} PDP11CPU;

// Operand descriptor: either a register or a memory address.
typedef struct PDP11Operand {
    uint8_t  is_reg;
    uint8_t  reg;
    uint16_t addr;
} PDP11Operand;

static const char *pdp11_reg_names[8] = {
    "R0", "R1", "R2", "R3", "R4", "R5", "SP", "PC"
};

// ---------------------------------------------------------------------------
// Memory access
// ---------------------------------------------------------------------------

static uint16_t pdp11_rdw(const PDP11CPU *cpu, uint16_t addr) {
    return (uint16_t)(cpu->mem[addr] | (cpu->mem[(uint16_t)(addr + 1)] << 8));
}

static void pdp11_wrw(PDP11CPU *cpu, uint16_t addr, uint16_t val) {
    cpu->mem[addr] = (uint8_t)(val & 0377);
    cpu->mem[(uint16_t)(addr + 1)] = (uint8_t)(val >> 8);
}

static uint16_t pdp11_fetch(PDP11CPU *cpu) {
    uint16_t w = pdp11_rdw(cpu, cpu->r[7]);
    cpu->r[7] = (uint16_t)(cpu->r[7] + 2);
    return w;
}

// ---------------------------------------------------------------------------
// Flags
// ---------------------------------------------------------------------------

static void pdp11_flag(PDP11CPU *cpu, uint16_t bit, int cond) {
    if (cond) cpu->psw |= bit;
    else      cpu->psw = (uint16_t)(cpu->psw & ~bit);
}

static void pdp11_set_nz(PDP11CPU *cpu, uint16_t val, int is_byte) {
    uint16_t sign = is_byte ? 0200 : 0100000;
    uint16_t mask = is_byte ? 0377 : 0177777;
    pdp11_flag(cpu, PSW_N, (val & sign) != 0);
    pdp11_flag(cpu, PSW_Z, (val & mask) == 0);
}

// ---------------------------------------------------------------------------
// Addressing modes
// ---------------------------------------------------------------------------

// Decode a 6-bit operand specifier, performing any auto-increment/decrement
// and fetching index words. Byte operations step R0-R5 by 1; SP and PC always
// step by 2.
static PDP11Operand pdp11_get_operand(PDP11CPU *cpu, uint16_t spec, int is_byte) {
    PDP11Operand op = { 0, 0, 0 };
    uint16_t mode = (spec >> 3) & 07;
    uint16_t reg = spec & 07;
    uint16_t delta = (is_byte && reg < 6) ? 1 : 2;
    uint16_t x;

    switch (mode) {
        case 0: // Register
            op.is_reg = 1;
            op.reg = (uint8_t)reg;
            break;
        case 1: // Register deferred
            op.addr = cpu->r[reg];
            break;
        case 2: // Autoincrement
            op.addr = cpu->r[reg];
            cpu->r[reg] = (uint16_t)(cpu->r[reg] + delta);
            break;
        case 3: // Autoincrement deferred
            op.addr = pdp11_rdw(cpu, cpu->r[reg]);
            cpu->r[reg] = (uint16_t)(cpu->r[reg] + 2);
            break;
        case 4: // Autodecrement
            cpu->r[reg] = (uint16_t)(cpu->r[reg] - delta);
            op.addr = cpu->r[reg];
            break;
        case 5: // Autodecrement deferred
            cpu->r[reg] = (uint16_t)(cpu->r[reg] - 2);
            op.addr = pdp11_rdw(cpu, cpu->r[reg]);
            break;
        case 6: // Index
            x = pdp11_fetch(cpu);
            op.addr = (uint16_t)(cpu->r[reg] + x);
            break;
        default: // 7: Index deferred
            x = pdp11_fetch(cpu);
            op.addr = pdp11_rdw(cpu, (uint16_t)(cpu->r[reg] + x));
            break;
    }
    return op;
}

static uint16_t pdp11_op_read(PDP11CPU *cpu, const PDP11Operand *op, int is_byte) {
    if (op->is_reg) {
        return is_byte ? (uint16_t)(cpu->r[op->reg] & 0377) : cpu->r[op->reg];
    }
    return is_byte ? (uint16_t)cpu->mem[op->addr] : pdp11_rdw(cpu, op->addr);
}

static void pdp11_op_write(PDP11CPU *cpu, const PDP11Operand *op, uint16_t val, int is_byte) {
    if (op->is_reg) {
        if (is_byte) {
            cpu->r[op->reg] = (uint16_t)((cpu->r[op->reg] & 0177400) | (val & 0377));
        } else {
            cpu->r[op->reg] = val;
        }
        return;
    }
    if (is_byte) {
        cpu->mem[op->addr] = (uint8_t)(val & 0377);
    } else {
        pdp11_wrw(cpu, op->addr, val);
    }
}

// ---------------------------------------------------------------------------
// Instruction groups
// ---------------------------------------------------------------------------

// MOV(B) CMP(B) BIT(B) BIC(B) BIS(B) ADD SUB
static void pdp11_double_operand(PDP11CPU *cpu, uint16_t instr) {
    int dopc = (instr >> 12) & 07;
    int byte = (instr & 0100000) != 0 && dopc != 6; // ADD/SUB are word-only
    uint16_t sign = byte ? 0200 : 0100000;
    PDP11Operand src_op = pdp11_get_operand(cpu, (uint16_t)((instr >> 6) & 077), byte);
    uint16_t src = pdp11_op_read(cpu, &src_op, byte);
    PDP11Operand dst_op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), byte);
    uint16_t dst, r;

    switch (dopc) {
        case 1: // MOV(B)
            if (byte && dst_op.is_reg) {
                // MOVB to a register sign-extends
                cpu->r[dst_op.reg] = (uint16_t)((src & 0200) ? (src | 0177400) : src);
            } else {
                pdp11_op_write(cpu, &dst_op, src, byte);
            }
            pdp11_set_nz(cpu, src, byte);
            pdp11_flag(cpu, PSW_V, 0);
            break;
        case 2: // CMP(B): src - dst
            dst = pdp11_op_read(cpu, &dst_op, byte);
            r = (uint16_t)(src - dst);
            pdp11_set_nz(cpu, r, byte);
            pdp11_flag(cpu, PSW_V, ((src ^ dst) & (src ^ r) & sign) != 0);
            pdp11_flag(cpu, PSW_C, (src & (sign | (sign - 1))) < (dst & (sign | (sign - 1))));
            break;
        case 3: // BIT(B)
            dst = pdp11_op_read(cpu, &dst_op, byte);
            pdp11_set_nz(cpu, (uint16_t)(src & dst), byte);
            pdp11_flag(cpu, PSW_V, 0);
            break;
        case 4: // BIC(B)
            dst = pdp11_op_read(cpu, &dst_op, byte);
            r = (uint16_t)(dst & ~src);
            pdp11_op_write(cpu, &dst_op, r, byte);
            pdp11_set_nz(cpu, r, byte);
            pdp11_flag(cpu, PSW_V, 0);
            break;
        case 5: // BIS(B)
            dst = pdp11_op_read(cpu, &dst_op, byte);
            r = (uint16_t)(dst | src);
            pdp11_op_write(cpu, &dst_op, r, byte);
            pdp11_set_nz(cpu, r, byte);
            pdp11_flag(cpu, PSW_V, 0);
            break;
        default: // 6: ADD (bit 15 clear) / SUB (bit 15 set)
            dst = pdp11_op_read(cpu, &dst_op, 0);
            if (instr & 0100000) { // SUB: dst - src
                r = (uint16_t)(dst - src);
                pdp11_flag(cpu, PSW_V, ((src ^ dst) & (dst ^ r) & 0100000) != 0);
                pdp11_flag(cpu, PSW_C, dst < src);
            } else {               // ADD
                uint32_t sum = (uint32_t)dst + src;
                r = (uint16_t)sum;
                pdp11_flag(cpu, PSW_V, (~(src ^ dst) & (src ^ r) & 0100000) != 0);
                pdp11_flag(cpu, PSW_C, sum > 0177777);
            }
            pdp11_op_write(cpu, &dst_op, r, 0);
            pdp11_set_nz(cpu, r, 0);
            break;
    }
}

// CLR(B) COM(B) INC(B) DEC(B) NEG(B) ADC(B) SBC(B) TST(B) ROR(B) ROL(B)
// ASR(B) ASL(B)
static void pdp11_single_operand(PDP11CPU *cpu, uint16_t instr) {
    int sopc = ((instr >> 6) & 077) - 050; // 050..063 -> 0..11
    int byte = (instr & 0100000) != 0;
    uint16_t sign = byte ? 0200 : 0100000;
    uint16_t mask = byte ? 0377 : 0177777;
    PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), byte);
    uint16_t dst = (sopc == 0) ? 0 : pdp11_op_read(cpu, &op, byte);
    uint16_t r = dst;
    int c = (cpu->psw & PSW_C) != 0;

    switch (sopc) {
        case 0: // CLR(B)
            r = 0;
            pdp11_flag(cpu, PSW_V, 0);
            pdp11_flag(cpu, PSW_C, 0);
            break;
        case 1: // COM(B)
            r = (uint16_t)(~dst & mask);
            pdp11_flag(cpu, PSW_V, 0);
            pdp11_flag(cpu, PSW_C, 1);
            break;
        case 2: // INC(B)
            r = (uint16_t)((dst + 1) & mask);
            pdp11_flag(cpu, PSW_V, dst == (uint16_t)(sign - 1));
            break;
        case 3: // DEC(B)
            r = (uint16_t)((dst - 1) & mask);
            pdp11_flag(cpu, PSW_V, dst == sign);
            break;
        case 4: // NEG(B)
            r = (uint16_t)(-(int32_t)dst & mask);
            pdp11_flag(cpu, PSW_V, r == sign);
            pdp11_flag(cpu, PSW_C, r != 0);
            break;
        case 5: // ADC(B)
            r = (uint16_t)((dst + (c ? 1 : 0)) & mask);
            pdp11_flag(cpu, PSW_V, c && dst == (uint16_t)(sign - 1));
            pdp11_flag(cpu, PSW_C, c && dst == mask);
            break;
        case 6: // SBC(B)
            r = (uint16_t)((dst - (c ? 1 : 0)) & mask);
            pdp11_flag(cpu, PSW_V, dst == sign);
            pdp11_flag(cpu, PSW_C, c && dst == 0);
            break;
        case 7: // TST(B)
            pdp11_set_nz(cpu, dst, byte);
            pdp11_flag(cpu, PSW_V, 0);
            pdp11_flag(cpu, PSW_C, 0);
            return; // No write-back
        case 8: // ROR(B)
            r = (uint16_t)(((dst >> 1) | (c ? sign : 0)) & mask);
            pdp11_flag(cpu, PSW_C, (dst & 1) != 0);
            break;
        case 9: // ROL(B)
            r = (uint16_t)(((dst << 1) | (c ? 1 : 0)) & mask);
            pdp11_flag(cpu, PSW_C, (dst & sign) != 0);
            break;
        case 10: // ASR(B)
            r = (uint16_t)(((dst >> 1) | (dst & sign)) & mask);
            pdp11_flag(cpu, PSW_C, (dst & 1) != 0);
            break;
        default: // 11: ASL(B)
            r = (uint16_t)((dst << 1) & mask);
            pdp11_flag(cpu, PSW_C, (dst & sign) != 0);
            break;
    }

    pdp11_op_write(cpu, &op, r, byte);
    pdp11_set_nz(cpu, r, byte);
    if (sopc >= 8) { // Shifts and rotates: V = N xor C
        pdp11_flag(cpu, PSW_V, ((cpu->psw & PSW_N) != 0) != ((cpu->psw & PSW_C) != 0));
    }
}

// Conditional and unconditional branches.
static void pdp11_branch(PDP11CPU *cpu, uint16_t instr) {
    int n = (cpu->psw & PSW_N) != 0;
    int z = (cpu->psw & PSW_Z) != 0;
    int v = (cpu->psw & PSW_V) != 0;
    int c = (cpu->psw & PSW_C) != 0;
    int taken;

    switch ((instr >> 8) & 0377) {
        case 0001: taken = 1; break;          // BR
        case 0002: taken = !z; break;         // BNE
        case 0003: taken = z; break;          // BEQ
        case 0004: taken = !(n ^ v); break;   // BGE
        case 0005: taken = n ^ v; break;      // BLT
        case 0006: taken = !(z || (n ^ v)); break; // BGT
        case 0007: taken = z || (n ^ v); break;    // BLE
        case 0200: taken = !n; break;         // BPL
        case 0201: taken = n; break;          // BMI
        case 0202: taken = !c && !z; break;   // BHI
        case 0203: taken = c || z; break;     // BLOS
        case 0204: taken = !v; break;         // BVC
        case 0205: taken = v; break;          // BVS
        case 0206: taken = !c; break;         // BCC
        case 0207: taken = c; break;          // BCS
        default:   taken = 0; break;
    }

    if (taken) {
        int16_t offset = (int16_t)(int8_t)(instr & 0377);
        cpu->r[7] = (uint16_t)(cpu->r[7] + 2 * offset);
    }
}

// MUL DIV ASH ASHC (Extended Instruction Set)
static void pdp11_eis(PDP11CPU *cpu, uint16_t instr) {
    int eopc = (instr >> 9) & 07;
    uint16_t reg = (instr >> 6) & 07;
    PDP11Operand src_op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 0);
    uint16_t src = pdp11_op_read(cpu, &src_op, 0);

    if (eopc == 0) { // MUL
        int32_t prod = (int32_t)(int16_t)cpu->r[reg] * (int16_t)src;
        cpu->r[reg] = (uint16_t)((uint32_t)prod >> 16);
        cpu->r[reg | 1] = (uint16_t)prod;
        pdp11_flag(cpu, PSW_N, prod < 0);
        pdp11_flag(cpu, PSW_Z, prod == 0);
        pdp11_flag(cpu, PSW_V, 0);
        pdp11_flag(cpu, PSW_C, prod < -32768 || prod > 32767);
    }
    else if (eopc == 1) { // DIV
        int32_t dividend = (int32_t)(((uint32_t)cpu->r[reg] << 16) | cpu->r[reg | 1]);
        int32_t divisor = (int16_t)src;
        if (divisor == 0) {
            pdp11_flag(cpu, PSW_V, 1);
            pdp11_flag(cpu, PSW_C, 1);
        }
        else if (dividend == INT32_MIN && divisor == -1) {
            pdp11_flag(cpu, PSW_V, 1);
            pdp11_flag(cpu, PSW_C, 0);
        }
        else {
            int32_t quot = dividend / divisor;
            int32_t rem = dividend % divisor;
            if (quot > 32767 || quot < -32768) {
                pdp11_flag(cpu, PSW_V, 1);
                pdp11_flag(cpu, PSW_C, 0);
            } else {
                cpu->r[reg] = (uint16_t)quot;
                cpu->r[reg | 1] = (uint16_t)rem;
                pdp11_flag(cpu, PSW_N, quot < 0);
                pdp11_flag(cpu, PSW_Z, quot == 0);
                pdp11_flag(cpu, PSW_V, 0);
                pdp11_flag(cpu, PSW_C, 0);
            }
        }
    }
    else if (eopc == 2) { // ASH
        int shift = src & 077;
        uint16_t val = cpu->r[reg];
        int carry = (cpu->psw & PSW_C) != 0;
        int overflow = 0;
        if (shift & 040) shift -= 64; // -32..31
        if (shift > 0) {
            for (int i = 0; i < shift; ++i) {
                carry = (val & 0100000) != 0;
                uint16_t next = (uint16_t)(val << 1);
                if ((val ^ next) & 0100000) overflow = 1;
                val = next;
            }
        } else if (shift < 0) {
            for (int i = 0; i < -shift; ++i) {
                carry = (val & 1) != 0;
                val = (uint16_t)((val >> 1) | (val & 0100000));
            }
        }
        cpu->r[reg] = val;
        pdp11_set_nz(cpu, val, 0);
        pdp11_flag(cpu, PSW_V, overflow);
        pdp11_flag(cpu, PSW_C, carry);
    }
    else if (eopc == 3) { // ASHC
        int shift = src & 077;
        uint32_t val = ((uint32_t)cpu->r[reg] << 16) | cpu->r[reg | 1];
        int carry = (cpu->psw & PSW_C) != 0;
        int overflow = 0;
        if (shift & 040) shift -= 64;
        if (shift > 0) {
            for (int i = 0; i < shift; ++i) {
                carry = (val & 0x80000000u) != 0;
                uint32_t next = val << 1;
                if ((val ^ next) & 0x80000000u) overflow = 1;
                val = next;
            }
        } else if (shift < 0) {
            for (int i = 0; i < -shift; ++i) {
                carry = (val & 1) != 0;
                val = (val >> 1) | (val & 0x80000000u);
            }
        }
        cpu->r[reg] = (uint16_t)(val >> 16);
        cpu->r[reg | 1] = (uint16_t)val;
        pdp11_flag(cpu, PSW_N, (val & 0x80000000u) != 0);
        pdp11_flag(cpu, PSW_Z, val == 0);
        pdp11_flag(cpu, PSW_V, overflow);
        pdp11_flag(cpu, PSW_C, carry);
    }
    else { // XOR (074RDD)
        uint16_t r = (uint16_t)(src ^ cpu->r[reg]);
        pdp11_op_write(cpu, &src_op, r, 0);
        pdp11_set_nz(cpu, r, 0);
        pdp11_flag(cpu, PSW_V, 0);
    }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void* pdp11_create(void) {
    PDP11CPU *cpu = (PDP11CPU*)calloc(1, sizeof(PDP11CPU));
    if (cpu) {
        cpu->r[7] = PDP11_RESET_PC;
    }
    return cpu;
}

void pdp11_destroy(void *context) {
    free(context);
}

int pdp11_init(void *context) {
    if (!context) return -1;
    PDP11CPU *cpu = (PDP11CPU*)context;

    memset(cpu->mem, 0, sizeof(cpu->mem));
    memset(cpu->r, 0, sizeof(cpu->r));
    cpu->r[7] = PDP11_RESET_PC;
    cpu->psw = 0;
    cpu->halted = 0;
    cpu->ticks = 0;
    return 0;
}

int pdp11_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    PDP11CPU *cpu = (PDP11CPU*)context;
    if (address >= PDP11_MEM_SIZE) return -2;

    for (size_t i = 0; i < size; ++i) {
        uint32_t addr = address + (uint32_t)i;
        if (addr >= PDP11_MEM_SIZE) break;
        cpu->mem[addr] = data[i];
    }
    return 0;
}

int pdp11_step(void *context) {
    if (!context) return -1;
    PDP11CPU *cpu = (PDP11CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr = pdp11_fetch(cpu);
    cpu->ticks++;

    int dopc = (instr >> 12) & 07;
    if (dopc >= 1 && dopc <= 6) {
        pdp11_double_operand(cpu, instr);
        return 0;
    }

    if ((instr & 0100000) == 0) {
        // Opcode 0 and 07 word-space groups
        if (instr == 0000000 || instr == 0000001) { // HALT / WAIT
            cpu->halted = 1;
            return 1;
        }
        if (instr <= 0000006) { // RTI BPT IOT RESET RTT: no-op
            return 0;
        }
        if ((instr & 0177700) == 0000100) { // JMP
            PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 0);
            if (!op.is_reg) cpu->r[7] = op.addr;
            return 0;
        }
        if ((instr & 0177770) == 0000200) { // RTS
            uint16_t reg = instr & 07;
            cpu->r[7] = cpu->r[reg];
            cpu->r[reg] = pdp11_rdw(cpu, cpu->r[6]);
            cpu->r[6] = (uint16_t)(cpu->r[6] + 2);
            return 0;
        }
        if ((instr & 0177740) == 0000240) { // Condition-code ops / NOP
            uint16_t bits = instr & 017;
            if (instr & 020) cpu->psw |= bits;
            else             cpu->psw = (uint16_t)(cpu->psw & ~bits);
            return 0;
        }
        if ((instr & 0177700) == 0000300) { // SWAB
            PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 0);
            uint16_t dst = pdp11_op_read(cpu, &op, 0);
            uint16_t r = (uint16_t)((dst >> 8) | (dst << 8));
            pdp11_op_write(cpu, &op, r, 0);
            pdp11_set_nz(cpu, r, 1); // N/Z from low byte
            pdp11_flag(cpu, PSW_V, 0);
            pdp11_flag(cpu, PSW_C, 0);
            return 0;
        }
        if (instr >= 0000400 && instr <= 0003777) { // BR BNE BEQ BGE BLT BGT BLE
            pdp11_branch(cpu, instr);
            return 0;
        }
        if ((instr & 0177000) == 0004000) { // JSR
            uint16_t reg = (instr >> 6) & 07;
            PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 0);
            if (!op.is_reg) {
                cpu->r[6] = (uint16_t)(cpu->r[6] - 2);
                pdp11_wrw(cpu, cpu->r[6], cpu->r[reg]);
                cpu->r[reg] = cpu->r[7];
                cpu->r[7] = op.addr;
            }
            return 0;
        }
        if ((instr & 0177700) >= 0005000 && (instr & 0177700) <= 0006300) {
            pdp11_single_operand(cpu, instr); // CLR..ASL
            return 0;
        }
        if ((instr & 0177700) == 0006400) { // MARK
            uint16_t nn = instr & 077;
            cpu->r[6] = (uint16_t)(cpu->r[7] + 2 * nn);
            cpu->r[7] = cpu->r[5];
            cpu->r[5] = pdp11_rdw(cpu, cpu->r[6]);
            cpu->r[6] = (uint16_t)(cpu->r[6] + 2);
            return 0;
        }
        if ((instr & 0177700) == 0006700) { // SXT
            PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 0);
            uint16_t r = (cpu->psw & PSW_N) ? (uint16_t)0177777 : (uint16_t)0;
            pdp11_op_write(cpu, &op, r, 0);
            pdp11_flag(cpu, PSW_Z, (cpu->psw & PSW_N) == 0);
            pdp11_flag(cpu, PSW_V, 0);
            return 0;
        }
        if ((instr & 0177000) >= 0070000 && (instr & 0177000) <= 0074000) {
            pdp11_eis(cpu, instr); // MUL DIV ASH ASHC XOR
            return 0;
        }
        if ((instr & 0177000) == 0077000) { // SOB
            uint16_t reg = (instr >> 6) & 07;
            uint16_t off = instr & 077;
            cpu->r[reg] = (uint16_t)(cpu->r[reg] - 1);
            if (cpu->r[reg] != 0) {
                cpu->r[7] = (uint16_t)(cpu->r[7] - 2 * off);
            }
            return 0;
        }
    }
    else {
        // Byte-space groups (bit 15 set, opcode 10xxxx)
        if (instr >= 0100000 && instr <= 0103777) { // BPL..BCS
            pdp11_branch(cpu, instr);
            return 0;
        }
        if ((instr & 0177400) == 0104000 || (instr & 0177400) == 0104400) {
            return 0; // EMT / TRAP: no-op
        }
        if ((instr & 0177700) >= 0105000 && (instr & 0177700) <= 0106300) {
            pdp11_single_operand(cpu, instr); // CLRB..ASLB
            return 0;
        }
        if ((instr & 0177700) == 0106400) { // MTPS
            PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 1);
            uint16_t v = pdp11_op_read(cpu, &op, 1);
            cpu->psw = (uint16_t)((cpu->psw & ~0357u) | (v & 0357u));
            return 0;
        }
        if ((instr & 0177700) == 0106700) { // MFPS
            PDP11Operand op = pdp11_get_operand(cpu, (uint16_t)(instr & 077), 1);
            uint16_t v = (uint16_t)(cpu->psw & 0377);
            if (op.is_reg) {
                cpu->r[op.reg] = (uint16_t)((v & 0200) ? (v | 0177400) : v);
            } else {
                pdp11_op_write(cpu, &op, v, 1);
            }
            pdp11_set_nz(cpu, v, 1);
            pdp11_flag(cpu, PSW_V, 0);
            return 0;
        }
    }

    // Unimplemented/reserved instruction: no-op
    return 0;
}

void pdp11_print_state(void *context) {
    if (!context) return;
    PDP11CPU *cpu = (PDP11CPU*)context;

    printf("DEC PDP-11 State:\n");
    printf("  R0: %06o  R1: %06o  R2: %06o  R3: %06o\n",
           cpu->r[0], cpu->r[1], cpu->r[2], cpu->r[3]);
    printf("  R4: %06o  R5: %06o  SP: %06o  PC: %06o\n",
           cpu->r[4], cpu->r[5], cpu->r[6], cpu->r[7]);
    printf("  PSW: %06o [%c%c%c%c]  Ticks: %u%s\n",
           cpu->psw,
           (cpu->psw & PSW_N) ? 'N' : '-',
           (cpu->psw & PSW_Z) ? 'Z' : '-',
           (cpu->psw & PSW_V) ? 'V' : '-',
           (cpu->psw & PSW_C) ? 'C' : '-',
           cpu->ticks,
           cpu->halted ? "  [HALTED]" : "");
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

// Format a 6-bit operand specifier at *ext (address of its extension word, if
// any), advancing *ext past consumed words. All values printed in octal.
static void pdp11_dis_operand(PDP11CPU *cpu, uint16_t spec, uint16_t *ext,
                              char *out, size_t out_len) {
    uint16_t mode = (spec >> 3) & 07;
    uint16_t reg = spec & 07;
    const char *rn = pdp11_reg_names[reg];
    uint16_t x;

    switch (mode) {
        case 0: snprintf(out, out_len, "%s", rn); break;
        case 1: snprintf(out, out_len, "(%s)", rn); break;
        case 2:
            if (reg == 7) {
                x = pdp11_rdw(cpu, *ext);
                *ext = (uint16_t)(*ext + 2);
                snprintf(out, out_len, "#%o", x);
            } else {
                snprintf(out, out_len, "(%s)+", rn);
            }
            break;
        case 3:
            if (reg == 7) {
                x = pdp11_rdw(cpu, *ext);
                *ext = (uint16_t)(*ext + 2);
                snprintf(out, out_len, "@#%o", x);
            } else {
                snprintf(out, out_len, "@(%s)+", rn);
            }
            break;
        case 4: snprintf(out, out_len, "-(%s)", rn); break;
        case 5: snprintf(out, out_len, "@-(%s)", rn); break;
        case 6:
            x = pdp11_rdw(cpu, *ext);
            *ext = (uint16_t)(*ext + 2);
            if (reg == 7) snprintf(out, out_len, "%o", (uint16_t)(*ext + x));
            else          snprintf(out, out_len, "%o(%s)", x, rn);
            break;
        default: // 7
            x = pdp11_rdw(cpu, *ext);
            *ext = (uint16_t)(*ext + 2);
            if (reg == 7) snprintf(out, out_len, "@%o", (uint16_t)(*ext + x));
            else          snprintf(out, out_len, "@%o(%s)", x, rn);
            break;
    }
}

void pdp11_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    PDP11CPU *cpu = (PDP11CPU*)context;
    uint16_t pc = cpu->r[7];
    uint16_t instr = pdp11_rdw(cpu, pc);
    uint16_t ext = (uint16_t)(pc + 2);
    char src[32], dst[32];

    static const char *dop_names[6] = { "MOV", "CMP", "BIT", "BIC", "BIS", "ADD" };
    static const char *sop_names[12] = {
        "CLR", "COM", "INC", "DEC", "NEG", "ADC", "SBC", "TST",
        "ROR", "ROL", "ASR", "ASL"
    };
    static const char *br_lo[7] = { "BR", "BNE", "BEQ", "BGE", "BLT", "BGT", "BLE" };
    static const char *br_hi[8] = { "BPL", "BMI", "BHI", "BLOS", "BVC", "BVS", "BCC", "BCS" };
    static const char *eis_names[4] = { "MUL", "DIV", "ASH", "ASHC" };

    int dopc = (instr >> 12) & 07;
    int byte = (instr & 0100000) != 0;

    if (dopc >= 1 && dopc <= 6) { // Double operand
        const char *name = (dopc == 6 && byte) ? "SUB" : dop_names[dopc - 1];
        const char *suffix = (byte && dopc != 6) ? "B" : "";
        pdp11_dis_operand(cpu, (uint16_t)((instr >> 6) & 077), &ext, src, sizeof(src));
        pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
        snprintf(buf, buf_len, "%s%s %s, %s", name, suffix, src, dst);
        return;
    }

    if (!byte) {
        if (instr == 0000000) { snprintf(buf, buf_len, "HALT"); return; }
        if (instr == 0000001) { snprintf(buf, buf_len, "WAIT"); return; }
        if (instr == 0000002) { snprintf(buf, buf_len, "RTI"); return; }
        if (instr == 0000003) { snprintf(buf, buf_len, "BPT"); return; }
        if (instr == 0000004) { snprintf(buf, buf_len, "IOT"); return; }
        if (instr == 0000005) { snprintf(buf, buf_len, "RESET"); return; }
        if (instr == 0000006) { snprintf(buf, buf_len, "RTT"); return; }
        if ((instr & 0177700) == 0000100) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "JMP %s", dst);
            return;
        }
        if ((instr & 0177770) == 0000200) {
            snprintf(buf, buf_len, "RTS %s", pdp11_reg_names[instr & 07]);
            return;
        }
        if ((instr & 0177740) == 0000240) { // Condition-code ops
            uint16_t bits = instr & 017;
            if (bits == 0) {
                snprintf(buf, buf_len, "NOP");
            } else if (bits == 017) {
                snprintf(buf, buf_len, (instr & 020) ? "SCC" : "CCC");
            } else {
                char ops[32];
                size_t len = 0;
                const char *pre = (instr & 020) ? "SE" : "CL";
                static const char cc_letters[4] = { 'C', 'V', 'Z', 'N' };
                for (int i = 3; i >= 0; --i) {
                    if ((bits & (1u << i)) && len < sizeof(ops)) {
                        len += (size_t)snprintf(ops + len, sizeof(ops) - len,
                                                "%s%s%c", len ? " " : "",
                                                pre, cc_letters[i]);
                    }
                }
                snprintf(buf, buf_len, "%s", ops);
            }
            return;
        }
        if ((instr & 0177700) == 0000300) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "SWAB %s", dst);
            return;
        }
        if (instr >= 0000400 && instr <= 0003777) {
            int16_t offset = (int16_t)(int8_t)(instr & 0377);
            snprintf(buf, buf_len, "%s %o", br_lo[((instr >> 8) & 0377) - 1],
                     (uint16_t)(pc + 2 + 2 * offset));
            return;
        }
        if ((instr & 0177000) == 0004000) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "JSR %s, %s", pdp11_reg_names[(instr >> 6) & 07], dst);
            return;
        }
        if ((instr & 0177700) >= 0005000 && (instr & 0177700) <= 0006300) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "%s %s", sop_names[((instr >> 6) & 077) - 050], dst);
            return;
        }
        if ((instr & 0177700) == 0006400) {
            snprintf(buf, buf_len, "MARK %o", instr & 077);
            return;
        }
        if ((instr & 0177700) == 0006700) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "SXT %s", dst);
            return;
        }
        if ((instr & 0177000) >= 0070000 && (instr & 0177000) <= 0073000) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, src, sizeof(src));
            snprintf(buf, buf_len, "%s %s, %s", eis_names[(instr >> 9) & 03],
                     src, pdp11_reg_names[(instr >> 6) & 07]);
            return;
        }
        if ((instr & 0177000) == 0074000) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "XOR %s, %s", pdp11_reg_names[(instr >> 6) & 07], dst);
            return;
        }
        if ((instr & 0177000) == 0077000) {
            snprintf(buf, buf_len, "SOB %s, %o", pdp11_reg_names[(instr >> 6) & 07],
                     (uint16_t)(pc + 2 - 2 * (instr & 077)));
            return;
        }
    }
    else {
        if (instr >= 0100000 && instr <= 0103777) {
            int16_t offset = (int16_t)(int8_t)(instr & 0377);
            snprintf(buf, buf_len, "%s %o", br_hi[(instr >> 8) & 07],
                     (uint16_t)(pc + 2 + 2 * offset));
            return;
        }
        if ((instr & 0177400) == 0104000) {
            snprintf(buf, buf_len, "EMT %o", instr & 0377);
            return;
        }
        if ((instr & 0177400) == 0104400) {
            snprintf(buf, buf_len, "TRAP %o", instr & 0377);
            return;
        }
        if ((instr & 0177700) >= 0105000 && (instr & 0177700) <= 0106300) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "%sB %s", sop_names[((instr >> 6) & 077) - 050], dst);
            return;
        }
        if ((instr & 0177700) == 0106400) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "MTPS %s", dst);
            return;
        }
        if ((instr & 0177700) == 0106700) {
            pdp11_dis_operand(cpu, (uint16_t)(instr & 077), &ext, dst, sizeof(dst));
            snprintf(buf, buf_len, "MFPS %s", dst);
            return;
        }
    }

    snprintf(buf, buf_len, ".WORD %06o", instr);
}
