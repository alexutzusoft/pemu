#include "msp430.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 0x10000 // 64KB address space

// Status Register (R2) flag bits
#define FLAG_C      0x0001
#define FLAG_Z      0x0002
#define FLAG_N      0x0004
#define FLAG_GIE    0x0008
#define FLAG_CPUOFF 0x0010
#define FLAG_V      0x0100

#define REG_PC 0
#define REG_SP 1
#define REG_SR 2
#define REG_CG 3

typedef struct MSP430CPU {
    uint16_t regs[16]; // R0=PC, R1=SP, R2=SR/CG1, R3=CG2
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} MSP430CPU;

static const char* reg_names[16] = {
    "PC", "SP", "SR", "R3", "R4", "R5", "R6", "R7",
    "R8", "R9", "R10", "R11", "R12", "R13", "R14", "R15"
};

// Format I mnemonics, opcodes 0x4 - 0xF
static const char* fmt1_names[12] = {
    "MOV", "ADD", "ADDC", "SUBC", "SUB", "CMP",
    "DADD", "BIT", "BIC", "BIS", "XOR", "AND"
};

// Format II mnemonics, opcode field bits 9-7
static const char* fmt2_names[8] = {
    "RRC", "SWPB", "RRA", "SXT", "PUSH", "CALL", "RETI", "???"
};

// Format III (jump) mnemonics, condition bits 12-10
static const char* jump_names[8] = {
    "JNZ", "JZ", "JNC", "JC", "JN", "JGE", "JL", "JMP"
};

void* msp430_create(void) {
    MSP430CPU *cpu = (MSP430CPU*)calloc(1, sizeof(MSP430CPU));
    return cpu;
}

void msp430_destroy(void *context) {
    free(context);
}

int msp430_init(void *context) {
    if (!context) return -1;
    MSP430CPU *cpu = (MSP430CPU*)context;

    memset(cpu->regs, 0, sizeof(cpu->regs));
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->regs[REG_SP] = 0xFFFE; // SP initialized at top of RAM
    cpu->regs[REG_PC] = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int msp430_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    MSP430CPU *cpu = (MSP430CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Memory helpers (little-endian, 16-bit address space wraps naturally)
static inline uint8_t read_byte(MSP430CPU *cpu, uint16_t addr) {
    return cpu->memory[addr];
}

static inline uint16_t read_word(MSP430CPU *cpu, uint16_t addr) {
    return (uint16_t)(cpu->memory[addr] | (cpu->memory[(uint16_t)(addr + 1)] << 8));
}

static inline void write_byte(MSP430CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->memory[addr] = val;
}

static inline void write_word(MSP430CPU *cpu, uint16_t addr, uint16_t val) {
    cpu->memory[addr] = (uint8_t)(val & 0xFF);
    cpu->memory[(uint16_t)(addr + 1)] = (uint8_t)(val >> 8);
}

// Operand descriptor
#define OP_REG   0
#define OP_MEM   1
#define OP_CONST 2

typedef struct Operand {
    int kind;      // OP_REG, OP_MEM or OP_CONST
    int reg;       // register index for OP_REG
    uint16_t addr; // memory address for OP_MEM
    uint16_t cval; // constant-generator value for OP_CONST
} Operand;

// Resolve an operand given register and addressing mode (As bits, or Ad as 0/1).
// Handles the full constant generator (R2/R3) and advances PC past extension words.
static Operand resolve_operand(MSP430CPU *cpu, int reg, int mode, int bw) {
    Operand op;
    op.kind = OP_REG;
    op.reg = reg;
    op.addr = 0;
    op.cval = 0;

    // Constant generator CG2 (R3): 0, 1, 2, -1
    if (reg == REG_CG) {
        op.kind = OP_CONST;
        switch (mode) {
            case 0: op.cval = 0; break;
            case 1: op.cval = 1; break;
            case 2: op.cval = 2; break;
            default: op.cval = 0xFFFF; break;
        }
        return op;
    }
    // Constant generator CG1 (R2/SR): 4 (As=10), 8 (As=11)
    if (reg == REG_SR && mode >= 2) {
        op.kind = OP_CONST;
        op.cval = (mode == 2) ? 4 : 8;
        return op;
    }

    switch (mode) {
        case 0: // Register mode
            break;
        case 1: { // Indexed x(Rn); R0: symbolic; R2: absolute
            uint16_t ext_addr = cpu->regs[REG_PC];
            uint16_t x = read_word(cpu, ext_addr);
            cpu->regs[REG_PC] = (uint16_t)(cpu->regs[REG_PC] + 2);
            op.kind = OP_MEM;
            if (reg == REG_SR) {
                op.addr = x;                            // Absolute &addr
            } else if (reg == REG_PC) {
                op.addr = (uint16_t)(ext_addr + x);     // Symbolic
            } else {
                op.addr = (uint16_t)(cpu->regs[reg] + x);
            }
            break;
        }
        case 2: // Indirect @Rn
            op.kind = OP_MEM;
            op.addr = cpu->regs[reg];
            break;
        default: // Indirect autoincrement @Rn+; R0: immediate #x
            op.kind = OP_MEM;
            op.addr = cpu->regs[reg];
            // PC and SP always increment by 2, others by 1 in byte mode
            cpu->regs[reg] = (uint16_t)(cpu->regs[reg] + ((bw && reg > REG_SP) ? 1 : 2));
            break;
    }
    return op;
}

static uint16_t read_operand(MSP430CPU *cpu, const Operand *op, int bw) {
    uint16_t v;
    switch (op->kind) {
        case OP_REG:   v = cpu->regs[op->reg]; break;
        case OP_MEM:   v = bw ? read_byte(cpu, op->addr) : read_word(cpu, op->addr); break;
        default:       v = op->cval; break;
    }
    return bw ? (uint16_t)(v & 0xFF) : v;
}

static void write_operand(MSP430CPU *cpu, const Operand *op, int bw, uint16_t val) {
    if (op->kind == OP_REG) {
        if (op->reg == REG_CG) return; // Writes to constant generator are discarded
        if (bw) val = (uint16_t)(val & 0xFF); // Byte writes to registers clear the high byte
        if (op->reg == REG_PC) val = (uint16_t)(val & 0xFFFE); // PC is word aligned
        cpu->regs[op->reg] = val;
    } else if (op->kind == OP_MEM) {
        if (bw) write_byte(cpu, op->addr, (uint8_t)(val & 0xFF));
        else write_word(cpu, op->addr, val);
    }
    // OP_CONST: write discarded
}

// Set N/Z plus explicit C and V flags for a result
static void set_flags(MSP430CPU *cpu, uint16_t res, int bw, int carry, int overflow) {
    uint16_t mask = bw ? 0xFF : 0xFFFF;
    uint16_t msb = bw ? 0x80 : 0x8000;
    uint16_t sr = (uint16_t)(cpu->regs[REG_SR] & ~(FLAG_C | FLAG_Z | FLAG_N | FLAG_V));
    res = (uint16_t)(res & mask);
    if (res == 0) sr |= FLAG_Z;
    if (res & msb) sr |= FLAG_N;
    if (carry) sr |= FLAG_C;
    if (overflow) sr |= FLAG_V;
    cpu->regs[REG_SR] = sr;
}

// Binary add with carry-in, setting C/Z/N/V. Subtraction passes (~src & mask) with cin.
static uint16_t add_with_flags(MSP430CPU *cpu, uint16_t a, uint16_t b, unsigned cin, int bw) {
    uint16_t mask = bw ? 0xFF : 0xFFFF;
    uint16_t msb = bw ? 0x80 : 0x8000;
    uint32_t r32 = (uint32_t)(a & mask) + (uint32_t)(b & mask) + cin;
    uint16_t res = (uint16_t)(r32 & mask);
    int carry = (r32 > mask) ? 1 : 0;
    int overflow = ((uint16_t)(~(a ^ b)) & (uint16_t)(a ^ res) & msb) ? 1 : 0;
    set_flags(cpu, res, bw, carry, overflow);
    return res;
}

// Decimal (BCD) add with carry-in from C flag. V is left unchanged (undefined).
static uint16_t dadd_with_flags(MSP430CPU *cpu, uint16_t src, uint16_t dst, int bw) {
    int digits = bw ? 2 : 4;
    unsigned carry = (cpu->regs[REG_SR] & FLAG_C) ? 1 : 0;
    uint16_t res = 0;
    for (int i = 0; i < digits; i++) {
        unsigned d = ((src >> (4 * i)) & 0xF) + ((dst >> (4 * i)) & 0xF) + carry;
        if (d > 9) { d -= 10; carry = 1; } else { carry = 0; }
        res = (uint16_t)(res | (d << (4 * i)));
    }
    uint16_t msb = bw ? 0x80 : 0x8000;
    uint16_t sr = (uint16_t)(cpu->regs[REG_SR] & ~(FLAG_C | FLAG_Z | FLAG_N));
    if (res == 0) sr |= FLAG_Z;
    if (res & msb) sr |= FLAG_N;
    if (carry) sr |= FLAG_C;
    cpu->regs[REG_SR] = sr;
    return res;
}

int msp430_step(void *context) {
    if (!context) return -1;
    MSP430CPU *cpu = (MSP430CPU*)context;

    if (cpu->halted) return 1;
    if (cpu->regs[REG_SR] & FLAG_CPUOFF) {
        cpu->halted = 1;
        return 1;
    }

    uint16_t instr_addr = cpu->regs[REG_PC];
    uint16_t instr = read_word(cpu, instr_addr);
    cpu->regs[REG_PC] = (uint16_t)(cpu->regs[REG_PC] + 2);
    cpu->ticks++;

    if ((instr & 0xE000) == 0x2000) {
        // ---- Format III: conditional jumps ----
        int cond = (instr >> 10) & 7;
        int off = instr & 0x3FF;
        if (off & 0x200) off -= 0x400; // sign-extend 10-bit offset
        uint16_t target = (uint16_t)(instr_addr + 2 + 2 * off);
        uint16_t sr = cpu->regs[REG_SR];
        int n = (sr & FLAG_N) ? 1 : 0;
        int v = (sr & FLAG_V) ? 1 : 0;
        int take = 0;
        switch (cond) {
            case 0: take = !(sr & FLAG_Z); break;  // JNE/JNZ
            case 1: take = (sr & FLAG_Z) != 0; break; // JEQ/JZ
            case 2: take = !(sr & FLAG_C); break;  // JNC
            case 3: take = (sr & FLAG_C) != 0; break; // JC
            case 4: take = n; break;               // JN
            case 5: take = !(n ^ v); break;        // JGE
            case 6: take = (n ^ v); break;         // JL
            default: take = 1; break;              // JMP
        }
        if (take) {
            cpu->regs[REG_PC] = target;
            if (cond == 7 && target == instr_addr) {
                cpu->halted = 1; // JMP-to-self: idle loop, halt
                return 1;
            }
        }
        return 0;
    }

    if ((instr & 0xFC00) == 0x1000) {
        // ---- Format II: single operand ----
        int op2 = (instr >> 7) & 7;
        int bw = (instr >> 6) & 1;
        int as = (instr >> 4) & 3;
        int reg = instr & 0xF;
        uint16_t msb = bw ? 0x80 : 0x8000;

        if (op2 == 6) { // RETI
            cpu->regs[REG_SR] = read_word(cpu, cpu->regs[REG_SP]);
            cpu->regs[REG_SP] = (uint16_t)(cpu->regs[REG_SP] + 2);
            cpu->regs[REG_PC] = (uint16_t)(read_word(cpu, cpu->regs[REG_SP]) & 0xFFFE);
            cpu->regs[REG_SP] = (uint16_t)(cpu->regs[REG_SP] + 2);
        } else if (op2 == 7) {
            return -2; // Invalid opcode
        } else {
            if (op2 == 5 || op2 == 1 || op2 == 3) bw = 0; // CALL/SWPB/SXT are word only
            Operand op = resolve_operand(cpu, reg, as, bw);
            uint16_t val = read_operand(cpu, &op, bw);
            switch (op2) {
                case 0: { // RRC: rotate right through carry
                    unsigned cin = (cpu->regs[REG_SR] & FLAG_C) ? 1u : 0u;
                    uint16_t res = (uint16_t)((val >> 1) | (cin ? msb : 0));
                    set_flags(cpu, res, bw, val & 1, 0);
                    write_operand(cpu, &op, bw, res);
                    break;
                }
                case 1: { // SWPB: swap bytes
                    uint16_t res = (uint16_t)((val >> 8) | (val << 8));
                    write_operand(cpu, &op, 0, res);
                    break;
                }
                case 2: { // RRA: arithmetic shift right
                    uint16_t res = (uint16_t)((val >> 1) | (val & msb));
                    set_flags(cpu, res, bw, val & 1, 0);
                    write_operand(cpu, &op, bw, res);
                    break;
                }
                case 3: { // SXT: sign extend byte to word
                    uint16_t res = (uint16_t)((val & 0x80) ? (val | 0xFF00) : (val & 0x7F));
                    set_flags(cpu, res, 0, res != 0, 0);
                    write_operand(cpu, &op, 0, res);
                    break;
                }
                case 4: { // PUSH
                    cpu->regs[REG_SP] = (uint16_t)(cpu->regs[REG_SP] - 2);
                    if (bw) write_byte(cpu, cpu->regs[REG_SP], (uint8_t)(val & 0xFF));
                    else write_word(cpu, cpu->regs[REG_SP], val);
                    break;
                }
                default: { // CALL (op2 == 5)
                    cpu->regs[REG_SP] = (uint16_t)(cpu->regs[REG_SP] - 2);
                    write_word(cpu, cpu->regs[REG_SP], cpu->regs[REG_PC]);
                    cpu->regs[REG_PC] = (uint16_t)(val & 0xFFFE);
                    break;
                }
            }
        }
    } else if ((instr >> 12) >= 4) {
        // ---- Format I: two operand ----
        int opc = instr >> 12;
        int src_reg = (instr >> 8) & 0xF;
        int ad = (instr >> 7) & 1;
        int bw = (instr >> 6) & 1;
        int as = (instr >> 4) & 3;
        int dst_reg = instr & 0xF;
        uint16_t mask = bw ? 0xFF : 0xFFFF;

        Operand sop = resolve_operand(cpu, src_reg, as, bw);
        uint16_t src = read_operand(cpu, &sop, bw);
        Operand dop = resolve_operand(cpu, dst_reg, ad, bw);
        uint16_t dst = read_operand(cpu, &dop, bw);

        switch (opc) {
            case 0x4: // MOV: no flags
                write_operand(cpu, &dop, bw, src);
                break;
            case 0x5: // ADD
                write_operand(cpu, &dop, bw, add_with_flags(cpu, src, dst, 0, bw));
                break;
            case 0x6: { // ADDC
                unsigned cin = (cpu->regs[REG_SR] & FLAG_C) ? 1u : 0u;
                write_operand(cpu, &dop, bw, add_with_flags(cpu, src, dst, cin, bw));
                break;
            }
            case 0x7: { // SUBC: dst + ~src + C
                unsigned cin = (cpu->regs[REG_SR] & FLAG_C) ? 1u : 0u;
                write_operand(cpu, &dop, bw, add_with_flags(cpu, (uint16_t)(~src & mask), dst, cin, bw));
                break;
            }
            case 0x8: // SUB: dst + ~src + 1
                write_operand(cpu, &dop, bw, add_with_flags(cpu, (uint16_t)(~src & mask), dst, 1, bw));
                break;
            case 0x9: // CMP: SUB without writeback
                add_with_flags(cpu, (uint16_t)(~src & mask), dst, 1, bw);
                break;
            case 0xA: // DADD
                write_operand(cpu, &dop, bw, dadd_with_flags(cpu, src, dst, bw));
                break;
            case 0xB: { // BIT: AND without writeback
                uint16_t res = (uint16_t)(src & dst);
                set_flags(cpu, res, bw, res != 0, 0);
                break;
            }
            case 0xC: // BIC: no flags
                write_operand(cpu, &dop, bw, (uint16_t)(dst & ~src));
                break;
            case 0xD: // BIS: no flags
                write_operand(cpu, &dop, bw, (uint16_t)(dst | src));
                break;
            case 0xE: { // XOR
                uint16_t res = (uint16_t)(src ^ dst);
                uint16_t msb = bw ? 0x80 : 0x8000;
                set_flags(cpu, res, bw, res != 0, (src & dst & msb) ? 1 : 0);
                write_operand(cpu, &dop, bw, res);
                break;
            }
            default: { // 0xF AND
                uint16_t res = (uint16_t)(src & dst);
                set_flags(cpu, res, bw, res != 0, 0);
                write_operand(cpu, &dop, bw, res);
                break;
            }
        }
    } else {
        return -2; // Invalid opcode
    }

    // CPUOFF set (e.g. BIS #0x10,SR) halts the CPU
    if (cpu->regs[REG_SR] & FLAG_CPUOFF) {
        cpu->halted = 1;
        return 1;
    }
    return 0;
}

void msp430_print_state(void *context) {
    if (!context) return;
    MSP430CPU *cpu = (MSP430CPU*)context;
    uint16_t sr = cpu->regs[REG_SR];

    printf("MSP430 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  Flags: [C=%d Z=%d N=%d V=%d]  Halted: %s\n",
           cpu->regs[REG_PC], cpu->regs[REG_SP],
           (sr & FLAG_C) ? 1 : 0, (sr & FLAG_Z) ? 1 : 0,
           (sr & FLAG_N) ? 1 : 0, (sr & FLAG_V) ? 1 : 0,
           cpu->halted ? "Yes" : "No");
    printf("  Registers:\n");
    for (int i = 0; i < 16; ++i) {
        printf("    %-3s(R%02d): 0x%04X%s",
               reg_names[i], i, cpu->regs[i],
               (i % 4 == 3) ? "\n" : "  ");
    }
}

// Format one operand for disassembly; advances *next past any extension word.
static void dis_operand(MSP430CPU *cpu, int reg, int mode, uint16_t *next,
                        char *out, size_t out_len) {
    if (reg == REG_CG) {
        const char* cg2[4] = { "#0", "#1", "#2", "#-1" };
        snprintf(out, out_len, "%s", cg2[mode & 3]);
        return;
    }
    if (reg == REG_SR && mode >= 2) {
        snprintf(out, out_len, "%s", (mode == 2) ? "#4" : "#8");
        return;
    }
    switch (mode) {
        case 0:
            snprintf(out, out_len, "%s", reg_names[reg]);
            break;
        case 1: {
            uint16_t ext_addr = *next;
            uint16_t x = read_word(cpu, ext_addr);
            *next = (uint16_t)(*next + 2);
            if (reg == REG_SR) {
                snprintf(out, out_len, "&0x%04X", x);
            } else if (reg == REG_PC) {
                snprintf(out, out_len, "0x%04X", (uint16_t)(ext_addr + x)); // symbolic
            } else {
                snprintf(out, out_len, "%d(%s)", (int16_t)x, reg_names[reg]);
            }
            break;
        }
        case 2:
            snprintf(out, out_len, "@%s", reg_names[reg]);
            break;
        default:
            if (reg == REG_PC) { // immediate
                uint16_t x = read_word(cpu, *next);
                *next = (uint16_t)(*next + 2);
                snprintf(out, out_len, "#0x%04X", x);
            } else {
                snprintf(out, out_len, "@%s+", reg_names[reg]);
            }
            break;
    }
}

void msp430_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    MSP430CPU *cpu = (MSP430CPU*)context;

    uint16_t pc = cpu->regs[REG_PC];
    uint16_t instr = read_word(cpu, pc);
    uint16_t next = (uint16_t)(pc + 2);
    char src_str[24], dst_str[24];

    if ((instr & 0xE000) == 0x2000) {
        // Format III: jumps
        int cond = (instr >> 10) & 7;
        int off = instr & 0x3FF;
        if (off & 0x200) off -= 0x400;
        uint16_t target = (uint16_t)(pc + 2 + 2 * off);
        snprintf(buf, buf_len, "%-5s 0x%04X", jump_names[cond], target);
    } else if ((instr & 0xFC00) == 0x1000) {
        // Format II: single operand
        int op2 = (instr >> 7) & 7;
        int bw = (instr >> 6) & 1;
        int as = (instr >> 4) & 3;
        int reg = instr & 0xF;
        if (op2 == 6) {
            snprintf(buf, buf_len, "RETI");
        } else if (op2 == 7) {
            snprintf(buf, buf_len, "unknown (0x%04X)", instr);
        } else {
            if (op2 == 5 || op2 == 1 || op2 == 3) bw = 0;
            dis_operand(cpu, reg, as, &next, dst_str, sizeof(dst_str));
            snprintf(buf, buf_len, "%s%-*s %s", fmt2_names[op2],
                     (int)(5 - strlen(fmt2_names[op2])), bw ? ".B" : "", dst_str);
        }
    } else if ((instr >> 12) >= 4) {
        // Format I: two operand
        int opc = instr >> 12;
        int src_reg = (instr >> 8) & 0xF;
        int ad = (instr >> 7) & 1;
        int bw = (instr >> 6) & 1;
        int as = (instr >> 4) & 3;
        int dst_reg = instr & 0xF;
        dis_operand(cpu, src_reg, as, &next, src_str, sizeof(src_str));
        dis_operand(cpu, dst_reg, ad, &next, dst_str, sizeof(dst_str));
        snprintf(buf, buf_len, "%s%-*s %s, %s", fmt1_names[opc - 4],
                 (int)(5 - strlen(fmt1_names[opc - 4])), bw ? ".B" : "",
                 src_str, dst_str);
    } else {
        snprintf(buf, buf_len, "unknown (0x%04X)", instr);
    }
}
