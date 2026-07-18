#include "mips32.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE (128 * 1024)

// MIPS I (R2000-class) integer CPU core.
// Branch delay slots are modeled with a pc/next_pc pair: the instruction in
// the delay slot always executes before a taken branch or jump takes effect.
typedef struct MIPS32CPU {
    uint32_t regs[32];   // $zero hardwired to 0
    uint32_t pc;         // address of the instruction to execute next
    uint32_t next_pc;    // address of the instruction after that (delay slot logic)
    uint32_t hi;
    uint32_t lo;
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} MIPS32CPU;

static const char* reg_names[] = {
    "$zero", "$at", "$v0", "$v1", "$a0", "$a1", "$a2", "$a3",
    "$t0",   "$t1", "$t2", "$t3", "$t4", "$t5", "$t6", "$t7",
    "$s0",   "$s1", "$s2", "$s3", "$s4", "$s5", "$s6", "$s7",
    "$t8",   "$t9", "$k0", "$k1", "$gp", "$sp", "$fp", "$ra"
};

void* mips32_create(void) {
    MIPS32CPU *cpu = (MIPS32CPU*)calloc(1, sizeof(MIPS32CPU));
    return cpu;
}

void mips32_destroy(void *context) {
    free(context);
}

int mips32_init(void *context) {
    if (!context) return -1;
    MIPS32CPU *cpu = (MIPS32CPU*)context;

    memset(cpu->regs, 0, sizeof(cpu->regs));
    cpu->regs[29] = MEM_SIZE - 4; // $sp initialized near top of RAM
    cpu->pc = 0;
    cpu->next_pc = 4;
    cpu->hi = 0;
    cpu->lo = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int mips32_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    MIPS32CPU *cpu = (MIPS32CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Memory helpers with bounds checks (little-endian host byte order, like rv32i)
static uint32_t read_mem32(MIPS32CPU *cpu, uint32_t addr) {
    uint32_t val;
    if (addr + 3 >= MEM_SIZE) return 0;
    memcpy(&val, &cpu->memory[addr], 4);
    return val;
}

static uint16_t read_mem16(MIPS32CPU *cpu, uint32_t addr) {
    uint16_t val;
    if (addr + 1 >= MEM_SIZE) return 0;
    memcpy(&val, &cpu->memory[addr], 2);
    return val;
}

static uint8_t read_mem8(MIPS32CPU *cpu, uint32_t addr) {
    if (addr >= MEM_SIZE) return 0;
    return cpu->memory[addr];
}

static void write_mem32(MIPS32CPU *cpu, uint32_t addr, uint32_t val) {
    if (addr + 3 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 4);
}

static void write_mem16(MIPS32CPU *cpu, uint32_t addr, uint16_t val) {
    if (addr + 1 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 2);
}

static void write_mem8(MIPS32CPU *cpu, uint32_t addr, uint8_t val) {
    if (addr >= MEM_SIZE) return;
    cpu->memory[addr] = val;
}

int mips32_step(void *context) {
    MIPS32CPU *cpu;
    uint32_t instr, instr_addr, branch_target;
    uint32_t op, rs, rt, rd, shamt, funct, target26;
    int32_t simm;
    uint32_t uimm;
    int branch_taken;

    if (!context) return -1;
    cpu = (MIPS32CPU*)context;

    if (cpu->halted) return 1;
    if (cpu->pc >= MEM_SIZE || (cpu->pc & 3) != 0) {
        return -3; // PC out of bounds or misaligned
    }

    instr_addr = cpu->pc;
    instr = read_mem32(cpu, instr_addr);
    cpu->ticks++;

    // Decode fields
    op       = (instr >> 26) & 0x3F;
    rs       = (instr >> 21) & 0x1F;
    rt       = (instr >> 16) & 0x1F;
    rd       = (instr >> 11) & 0x1F;
    shamt    = (instr >> 6)  & 0x1F;
    funct    = instr & 0x3F;
    simm     = (int32_t)(int16_t)(instr & 0xFFFF);
    uimm     = instr & 0xFFFF;
    target26 = instr & 0x03FFFFFF;

    // Delay-slot semantics: the instruction at cpu->pc executes now; control
    // transfers (if any) affect the instruction *after* the one at next_pc.
    branch_taken = 0;
    branch_target = 0;

    switch (op) {
        case 0x00: { // SPECIAL (R-type)
            switch (funct) {
                case 0x00: cpu->regs[rd] = cpu->regs[rt] << shamt; break;                        // SLL
                case 0x02: cpu->regs[rd] = cpu->regs[rt] >> shamt; break;                        // SRL
                case 0x03: cpu->regs[rd] = (uint32_t)((int32_t)cpu->regs[rt] >> shamt); break;   // SRA
                case 0x04: cpu->regs[rd] = cpu->regs[rt] << (cpu->regs[rs] & 0x1F); break;       // SLLV
                case 0x06: cpu->regs[rd] = cpu->regs[rt] >> (cpu->regs[rs] & 0x1F); break;       // SRLV
                case 0x07: cpu->regs[rd] = (uint32_t)((int32_t)cpu->regs[rt] >> (cpu->regs[rs] & 0x1F)); break; // SRAV
                case 0x08: // JR
                    branch_taken = 1;
                    branch_target = cpu->regs[rs];
                    break;
                case 0x09: // JALR
                    branch_taken = 1;
                    branch_target = cpu->regs[rs];
                    cpu->regs[rd] = instr_addr + 8; // return address skips the delay slot
                    break;
                case 0x0C: // SYSCALL
                    cpu->halted = 1;
                    return 1;
                case 0x0D: // BREAK
                    cpu->halted = 1;
                    return 1;
                case 0x10: cpu->regs[rd] = cpu->hi; break; // MFHI
                case 0x11: cpu->hi = cpu->regs[rs]; break; // MTHI
                case 0x12: cpu->regs[rd] = cpu->lo; break; // MFLO
                case 0x13: cpu->lo = cpu->regs[rs]; break; // MTLO
                case 0x18: { // MULT
                    int64_t prod = (int64_t)(int32_t)cpu->regs[rs] * (int64_t)(int32_t)cpu->regs[rt];
                    cpu->lo = (uint32_t)prod;
                    cpu->hi = (uint32_t)((uint64_t)prod >> 32);
                    break;
                }
                case 0x19: { // MULTU
                    uint64_t prod = (uint64_t)cpu->regs[rs] * (uint64_t)cpu->regs[rt];
                    cpu->lo = (uint32_t)prod;
                    cpu->hi = (uint32_t)(prod >> 32);
                    break;
                }
                case 0x1A: { // DIV (result undefined on divide-by-zero; leave HI/LO unchanged)
                    int32_t a = (int32_t)cpu->regs[rs];
                    int32_t b = (int32_t)cpu->regs[rt];
                    if (b != 0) {
                        if (a == INT32_MIN && b == -1) { // avoid host UB on overflow
                            cpu->lo = (uint32_t)INT32_MIN;
                            cpu->hi = 0;
                        } else {
                            cpu->lo = (uint32_t)(a / b);
                            cpu->hi = (uint32_t)(a % b);
                        }
                    }
                    break;
                }
                case 0x1B: // DIVU
                    if (cpu->regs[rt] != 0) {
                        cpu->lo = cpu->regs[rs] / cpu->regs[rt];
                        cpu->hi = cpu->regs[rs] % cpu->regs[rt];
                    }
                    break;
                // ADD/SUB: real MIPS raises an overflow exception; here overflow just wraps.
                case 0x20: cpu->regs[rd] = cpu->regs[rs] + cpu->regs[rt]; break; // ADD
                case 0x21: cpu->regs[rd] = cpu->regs[rs] + cpu->regs[rt]; break; // ADDU
                case 0x22: cpu->regs[rd] = cpu->regs[rs] - cpu->regs[rt]; break; // SUB
                case 0x23: cpu->regs[rd] = cpu->regs[rs] - cpu->regs[rt]; break; // SUBU
                case 0x24: cpu->regs[rd] = cpu->regs[rs] & cpu->regs[rt]; break; // AND
                case 0x25: cpu->regs[rd] = cpu->regs[rs] | cpu->regs[rt]; break; // OR
                case 0x26: cpu->regs[rd] = cpu->regs[rs] ^ cpu->regs[rt]; break; // XOR
                case 0x27: cpu->regs[rd] = ~(cpu->regs[rs] | cpu->regs[rt]); break; // NOR
                case 0x2A: cpu->regs[rd] = ((int32_t)cpu->regs[rs] < (int32_t)cpu->regs[rt]) ? 1 : 0; break; // SLT
                case 0x2B: cpu->regs[rd] = (cpu->regs[rs] < cpu->regs[rt]) ? 1 : 0; break; // SLTU
                default:
                    return -4; // Unknown funct
            }
            break;
        }
        case 0x01: { // REGIMM (BLTZ, BGEZ, BLTZAL, BGEZAL)
            int cond;
            switch (rt) {
                case 0x00: cond = ((int32_t)cpu->regs[rs] < 0);  break; // BLTZ
                case 0x01: cond = ((int32_t)cpu->regs[rs] >= 0); break; // BGEZ
                case 0x10: // BLTZAL
                    cond = ((int32_t)cpu->regs[rs] < 0);
                    cpu->regs[31] = instr_addr + 8;
                    break;
                case 0x11: // BGEZAL
                    cond = ((int32_t)cpu->regs[rs] >= 0);
                    cpu->regs[31] = instr_addr + 8;
                    break;
                default:
                    return -4;
            }
            if (cond) {
                branch_taken = 1;
                branch_target = instr_addr + 4 + ((uint32_t)simm << 2);
            }
            break;
        }
        case 0x02: // J
            branch_taken = 1;
            branch_target = ((instr_addr + 4) & 0xF0000000) | (target26 << 2);
            break;
        case 0x03: // JAL
            branch_taken = 1;
            branch_target = ((instr_addr + 4) & 0xF0000000) | (target26 << 2);
            cpu->regs[31] = instr_addr + 8;
            break;
        case 0x04: // BEQ
            if (cpu->regs[rs] == cpu->regs[rt]) {
                branch_taken = 1;
                branch_target = instr_addr + 4 + ((uint32_t)simm << 2);
            }
            break;
        case 0x05: // BNE
            if (cpu->regs[rs] != cpu->regs[rt]) {
                branch_taken = 1;
                branch_target = instr_addr + 4 + ((uint32_t)simm << 2);
            }
            break;
        case 0x06: // BLEZ
            if ((int32_t)cpu->regs[rs] <= 0) {
                branch_taken = 1;
                branch_target = instr_addr + 4 + ((uint32_t)simm << 2);
            }
            break;
        case 0x07: // BGTZ
            if ((int32_t)cpu->regs[rs] > 0) {
                branch_taken = 1;
                branch_target = instr_addr + 4 + ((uint32_t)simm << 2);
            }
            break;
        // ADDI: real MIPS raises an overflow exception; here overflow just wraps.
        case 0x08: cpu->regs[rt] = cpu->regs[rs] + (uint32_t)simm; break; // ADDI
        case 0x09: cpu->regs[rt] = cpu->regs[rs] + (uint32_t)simm; break; // ADDIU
        case 0x0A: cpu->regs[rt] = ((int32_t)cpu->regs[rs] < simm) ? 1 : 0; break; // SLTI
        case 0x0B: cpu->regs[rt] = (cpu->regs[rs] < (uint32_t)simm) ? 1 : 0; break; // SLTIU
        case 0x0C: cpu->regs[rt] = cpu->regs[rs] & uimm; break; // ANDI (zero-extended)
        case 0x0D: cpu->regs[rt] = cpu->regs[rs] | uimm; break; // ORI
        case 0x0E: cpu->regs[rt] = cpu->regs[rs] ^ uimm; break; // XORI
        case 0x0F: cpu->regs[rt] = uimm << 16; break; // LUI
        case 0x20: cpu->regs[rt] = (uint32_t)(int32_t)(int8_t)read_mem8(cpu, cpu->regs[rs] + (uint32_t)simm); break;   // LB
        case 0x21: cpu->regs[rt] = (uint32_t)(int32_t)(int16_t)read_mem16(cpu, cpu->regs[rs] + (uint32_t)simm); break; // LH
        case 0x23: cpu->regs[rt] = read_mem32(cpu, cpu->regs[rs] + (uint32_t)simm); break; // LW
        case 0x24: cpu->regs[rt] = read_mem8(cpu, cpu->regs[rs] + (uint32_t)simm); break;  // LBU
        case 0x25: cpu->regs[rt] = read_mem16(cpu, cpu->regs[rs] + (uint32_t)simm); break; // LHU
        case 0x28: write_mem8(cpu, cpu->regs[rs] + (uint32_t)simm, (uint8_t)cpu->regs[rt]); break;   // SB
        case 0x29: write_mem16(cpu, cpu->regs[rs] + (uint32_t)simm, (uint16_t)cpu->regs[rt]); break; // SH
        case 0x2B: write_mem32(cpu, cpu->regs[rs] + (uint32_t)simm, cpu->regs[rt]); break; // SW
        default:
            return -4; // Unknown opcode
    }

    cpu->regs[0] = 0; // $zero hardwired

    // Advance: instruction at next_pc (the delay slot, if we branched) runs next
    cpu->pc = cpu->next_pc;
    if (branch_taken) {
        cpu->next_pc = branch_target;
    } else {
        cpu->next_pc = cpu->pc + 4;
    }

    // Detect a tight self-loop (branch to self with nop-equivalent progress)
    if (branch_taken && branch_target == instr_addr) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

void mips32_print_state(void *context) {
    MIPS32CPU *cpu;
    int i;

    if (!context) return;
    cpu = (MIPS32CPU*)context;

    printf("MIPS32 (MIPS I) CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%08X  HI: 0x%08X  LO: 0x%08X  Halted: %s\n",
           cpu->pc, cpu->hi, cpu->lo, cpu->halted ? "Yes" : "No");
    printf("  Registers:\n");
    for (i = 0; i < 32; ++i) {
        printf("    %-5s($%02d): 0x%08X%s",
               reg_names[i], i, cpu->regs[i],
               (i % 4 == 3) ? "\n" : "  ");
    }
}

void mips32_get_disassembly(void *context, char *buf, size_t buf_len) {
    MIPS32CPU *cpu;
    uint32_t instr, op, rs, rt, rd, shamt, funct, target26;
    int32_t simm;
    uint32_t uimm;

    if (!context || !buf || buf_len == 0) return;
    cpu = (MIPS32CPU*)context;

    if (cpu->pc >= MEM_SIZE) {
        snprintf(buf, buf_len, "<out of RAM>");
        return;
    }

    instr = read_mem32(cpu, cpu->pc);
    op       = (instr >> 26) & 0x3F;
    rs       = (instr >> 21) & 0x1F;
    rt       = (instr >> 16) & 0x1F;
    rd       = (instr >> 11) & 0x1F;
    shamt    = (instr >> 6)  & 0x1F;
    funct    = instr & 0x3F;
    simm     = (int32_t)(int16_t)(instr & 0xFFFF);
    uimm     = instr & 0xFFFF;
    target26 = instr & 0x03FFFFFF;

    switch (op) {
        case 0x00: { // SPECIAL
            if (instr == 0) {
                snprintf(buf, buf_len, "nop");
                break;
            }
            switch (funct) {
                case 0x00: snprintf(buf, buf_len, "sll   %s, %s, %u", reg_names[rd], reg_names[rt], shamt); break;
                case 0x02: snprintf(buf, buf_len, "srl   %s, %s, %u", reg_names[rd], reg_names[rt], shamt); break;
                case 0x03: snprintf(buf, buf_len, "sra   %s, %s, %u", reg_names[rd], reg_names[rt], shamt); break;
                case 0x04: snprintf(buf, buf_len, "sllv  %s, %s, %s", reg_names[rd], reg_names[rt], reg_names[rs]); break;
                case 0x06: snprintf(buf, buf_len, "srlv  %s, %s, %s", reg_names[rd], reg_names[rt], reg_names[rs]); break;
                case 0x07: snprintf(buf, buf_len, "srav  %s, %s, %s", reg_names[rd], reg_names[rt], reg_names[rs]); break;
                case 0x08: snprintf(buf, buf_len, "jr    %s", reg_names[rs]); break;
                case 0x09: snprintf(buf, buf_len, "jalr  %s, %s", reg_names[rd], reg_names[rs]); break;
                case 0x0C: snprintf(buf, buf_len, "syscall"); break;
                case 0x0D: snprintf(buf, buf_len, "break"); break;
                case 0x10: snprintf(buf, buf_len, "mfhi  %s", reg_names[rd]); break;
                case 0x11: snprintf(buf, buf_len, "mthi  %s", reg_names[rs]); break;
                case 0x12: snprintf(buf, buf_len, "mflo  %s", reg_names[rd]); break;
                case 0x13: snprintf(buf, buf_len, "mtlo  %s", reg_names[rs]); break;
                case 0x18: snprintf(buf, buf_len, "mult  %s, %s", reg_names[rs], reg_names[rt]); break;
                case 0x19: snprintf(buf, buf_len, "multu %s, %s", reg_names[rs], reg_names[rt]); break;
                case 0x1A: snprintf(buf, buf_len, "div   %s, %s", reg_names[rs], reg_names[rt]); break;
                case 0x1B: snprintf(buf, buf_len, "divu  %s, %s", reg_names[rs], reg_names[rt]); break;
                case 0x20: snprintf(buf, buf_len, "add   %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x21: snprintf(buf, buf_len, "addu  %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x22: snprintf(buf, buf_len, "sub   %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x23: snprintf(buf, buf_len, "subu  %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x24: snprintf(buf, buf_len, "and   %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x25: snprintf(buf, buf_len, "or    %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x26: snprintf(buf, buf_len, "xor   %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x27: snprintf(buf, buf_len, "nor   %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x2A: snprintf(buf, buf_len, "slt   %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                case 0x2B: snprintf(buf, buf_len, "sltu  %s, %s, %s", reg_names[rd], reg_names[rs], reg_names[rt]); break;
                default:   snprintf(buf, buf_len, "unknown (0x%08X)", instr); break;
            }
            break;
        }
        case 0x01: { // REGIMM
            const char *name;
            switch (rt) {
                case 0x00: name = "bltz";   break;
                case 0x01: name = "bgez";   break;
                case 0x10: name = "bltzal"; break;
                case 0x11: name = "bgezal"; break;
                default:   name = NULL;     break;
            }
            if (name) {
                snprintf(buf, buf_len, "%-5s %s, 0x%X", name, reg_names[rs],
                         cpu->pc + 4 + ((uint32_t)simm << 2));
            } else {
                snprintf(buf, buf_len, "unknown (0x%08X)", instr);
            }
            break;
        }
        case 0x02: snprintf(buf, buf_len, "j     0x%X", ((cpu->pc + 4) & 0xF0000000) | (target26 << 2)); break;
        case 0x03: snprintf(buf, buf_len, "jal   0x%X", ((cpu->pc + 4) & 0xF0000000) | (target26 << 2)); break;
        case 0x04: snprintf(buf, buf_len, "beq   %s, %s, 0x%X", reg_names[rs], reg_names[rt], cpu->pc + 4 + ((uint32_t)simm << 2)); break;
        case 0x05: snprintf(buf, buf_len, "bne   %s, %s, 0x%X", reg_names[rs], reg_names[rt], cpu->pc + 4 + ((uint32_t)simm << 2)); break;
        case 0x06: snprintf(buf, buf_len, "blez  %s, 0x%X", reg_names[rs], cpu->pc + 4 + ((uint32_t)simm << 2)); break;
        case 0x07: snprintf(buf, buf_len, "bgtz  %s, 0x%X", reg_names[rs], cpu->pc + 4 + ((uint32_t)simm << 2)); break;
        case 0x08: snprintf(buf, buf_len, "addi  %s, %s, %d", reg_names[rt], reg_names[rs], simm); break;
        case 0x09: snprintf(buf, buf_len, "addiu %s, %s, %d", reg_names[rt], reg_names[rs], simm); break;
        case 0x0A: snprintf(buf, buf_len, "slti  %s, %s, %d", reg_names[rt], reg_names[rs], simm); break;
        case 0x0B: snprintf(buf, buf_len, "sltiu %s, %s, %d", reg_names[rt], reg_names[rs], simm); break;
        case 0x0C: snprintf(buf, buf_len, "andi  %s, %s, 0x%X", reg_names[rt], reg_names[rs], uimm); break;
        case 0x0D: snprintf(buf, buf_len, "ori   %s, %s, 0x%X", reg_names[rt], reg_names[rs], uimm); break;
        case 0x0E: snprintf(buf, buf_len, "xori  %s, %s, 0x%X", reg_names[rt], reg_names[rs], uimm); break;
        case 0x0F: snprintf(buf, buf_len, "lui   %s, 0x%X", reg_names[rt], uimm); break;
        case 0x20: snprintf(buf, buf_len, "lb    %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x21: snprintf(buf, buf_len, "lh    %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x23: snprintf(buf, buf_len, "lw    %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x24: snprintf(buf, buf_len, "lbu   %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x25: snprintf(buf, buf_len, "lhu   %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x28: snprintf(buf, buf_len, "sb    %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x29: snprintf(buf, buf_len, "sh    %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        case 0x2B: snprintf(buf, buf_len, "sw    %s, %d(%s)", reg_names[rt], simm, reg_names[rs]); break;
        default:   snprintf(buf, buf_len, "unknown (0x%08X)", instr); break;
    }
}
