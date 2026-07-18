#include "rv64i.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE (128 * 1024)

typedef struct RV64ICPU {
    uint64_t regs[32];
    uint64_t pc;
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
    int exit_code;
} RV64ICPU;

static const char* reg_names[] = {
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"
};

void* rv64i_create(void) {
    RV64ICPU *cpu = (RV64ICPU*)calloc(1, sizeof(RV64ICPU));
    return cpu;
}

void rv64i_destroy(void *context) {
    free(context);
}

int rv64i_init(void *context) {
    if (!context) return -1;
    RV64ICPU *cpu = (RV64ICPU*)context;

    memset(cpu->regs, 0, sizeof(cpu->regs));
    cpu->regs[2] = MEM_SIZE - 8; // sp (Stack Pointer) initialized near top of RAM
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->exit_code = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int rv64i_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    RV64ICPU *cpu = (RV64ICPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Memory Helpers with bounds checks
static uint64_t read_mem64(RV64ICPU *cpu, uint64_t addr) {
    if (addr + 7 >= MEM_SIZE) return 0;
    uint64_t val;
    memcpy(&val, &cpu->memory[addr], 8);
    return val;
}

static uint32_t read_mem32(RV64ICPU *cpu, uint64_t addr) {
    if (addr + 3 >= MEM_SIZE) return 0;
    uint32_t val;
    memcpy(&val, &cpu->memory[addr], 4);
    return val;
}

static uint16_t read_mem16(RV64ICPU *cpu, uint64_t addr) {
    if (addr + 1 >= MEM_SIZE) return 0;
    uint16_t val;
    memcpy(&val, &cpu->memory[addr], 2);
    return val;
}

static uint8_t read_mem8(RV64ICPU *cpu, uint64_t addr) {
    if (addr >= MEM_SIZE) return 0;
    return cpu->memory[addr];
}

static void write_mem64(RV64ICPU *cpu, uint64_t addr, uint64_t val) {
    if (addr + 7 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 8);
}

static void write_mem32(RV64ICPU *cpu, uint64_t addr, uint32_t val) {
    if (addr + 3 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 4);
}

static void write_mem16(RV64ICPU *cpu, uint64_t addr, uint16_t val) {
    if (addr + 1 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 2);
}

static void write_mem8(RV64ICPU *cpu, uint64_t addr, uint8_t val) {
    if (addr >= MEM_SIZE) return;
    cpu->memory[addr] = val;
}

int rv64i_step(void *context) {
    if (!context) return -1;
    RV64ICPU *cpu = (RV64ICPU*)context;

    if (cpu->halted) return 1;
    if (cpu->pc >= MEM_SIZE || (cpu->pc & 3) != 0) {
        return -3; // PC out of bounds or misaligned
    }

    uint64_t instr_addr = cpu->pc;
    uint32_t instr = read_mem32(cpu, instr_addr);

    // Decode fields
    uint8_t opcode = (uint8_t)(instr & 0x7F);
    uint8_t rd = (uint8_t)((instr >> 7) & 0x1F);
    uint8_t funct3 = (uint8_t)((instr >> 12) & 0x07);
    uint8_t rs1 = (uint8_t)((instr >> 15) & 0x1F);
    uint8_t rs2 = (uint8_t)((instr >> 20) & 0x1F);
    uint8_t funct7 = (uint8_t)((instr >> 25) & 0x7F);

    // Default next PC
    uint64_t next_pc = cpu->pc + 4;
    cpu->ticks++;

    // Immediates (sign-extended to 64 bits)
    int64_t imm_i = (int64_t)((int32_t)instr >> 20);
    int64_t imm_s = (int64_t)(((((int32_t)instr) >> 25) << 5) | rd);
    int64_t imm_b = (int64_t)(((((int32_t)instr) >> 31) << 12) |
                              (int32_t)(((instr >> 7) & 1) << 11) |
                              (int32_t)(((instr >> 25) & 0x3F) << 5) |
                              (int32_t)(((instr >> 8) & 0x0F) << 1));
    int64_t imm_u = (int64_t)(int32_t)(instr & 0xFFFFF000);
    int64_t imm_j = (int64_t)(((((int32_t)instr) >> 31) << 20) |
                              (int32_t)(instr & 0x000FF000) |
                              (int32_t)(((instr >> 20) & 1) << 11) |
                              (int32_t)(((instr >> 21) & 0x3FF) << 1));

    switch (opcode) {
        case 0x37: { // LUI (Load Upper Immediate)
            cpu->regs[rd] = (uint64_t)imm_u;
            break;
        }
        case 0x17: { // AUIPC (Add Upper Immediate to PC)
            cpu->regs[rd] = instr_addr + (uint64_t)imm_u;
            break;
        }
        case 0x6F: { // JAL (Jump and Link)
            cpu->regs[rd] = next_pc;
            next_pc = instr_addr + (uint64_t)imm_j;
            break;
        }
        case 0x67: { // JALR (Jump and Link Register)
            uint64_t target = (cpu->regs[rs1] + (uint64_t)imm_i) & ~(uint64_t)1;
            cpu->regs[rd] = next_pc;
            next_pc = target;
            break;
        }
        case 0x63: { // Branch Group (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            int take_branch = 0;
            switch (funct3) {
                case 0x0: take_branch = (cpu->regs[rs1] == cpu->regs[rs2]); break; // BEQ
                case 0x1: take_branch = (cpu->regs[rs1] != cpu->regs[rs2]); break; // BNE
                case 0x4: take_branch = ((int64_t)cpu->regs[rs1] < (int64_t)cpu->regs[rs2]); break; // BLT
                case 0x5: take_branch = ((int64_t)cpu->regs[rs1] >= (int64_t)cpu->regs[rs2]); break; // BGE
                case 0x6: take_branch = (cpu->regs[rs1] < cpu->regs[rs2]); break; // BLTU
                case 0x7: take_branch = (cpu->regs[rs1] >= cpu->regs[rs2]); break; // BGEU
            }
            if (take_branch) {
                next_pc = instr_addr + (uint64_t)imm_b;
            }
            break;
        }
        case 0x03: { // Load Group (LB, LH, LW, LD, LBU, LHU, LWU)
            uint64_t addr = cpu->regs[rs1] + (uint64_t)imm_i;
            switch (funct3) {
                case 0x0: cpu->regs[rd] = (uint64_t)(int64_t)(int8_t)read_mem8(cpu, addr); break;   // LB
                case 0x1: cpu->regs[rd] = (uint64_t)(int64_t)(int16_t)read_mem16(cpu, addr); break; // LH
                case 0x2: cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)read_mem32(cpu, addr); break; // LW
                case 0x3: cpu->regs[rd] = read_mem64(cpu, addr); break;                             // LD
                case 0x4: cpu->regs[rd] = read_mem8(cpu, addr); break;                              // LBU
                case 0x5: cpu->regs[rd] = read_mem16(cpu, addr); break;                             // LHU
                case 0x6: cpu->regs[rd] = read_mem32(cpu, addr); break;                             // LWU
            }
            break;
        }
        case 0x23: { // Store Group (SB, SH, SW, SD)
            uint64_t addr = cpu->regs[rs1] + (uint64_t)imm_s;
            switch (funct3) {
                case 0x0: write_mem8(cpu, addr, (uint8_t)cpu->regs[rs2]); break;   // SB
                case 0x1: write_mem16(cpu, addr, (uint16_t)cpu->regs[rs2]); break; // SH
                case 0x2: write_mem32(cpu, addr, (uint32_t)cpu->regs[rs2]); break; // SW
                case 0x3: write_mem64(cpu, addr, cpu->regs[rs2]); break;           // SD
            }
            break;
        }
        case 0x13: { // Register-Immediate Group (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
            uint32_t shamt = (instr >> 20) & 0x3F; // 6-bit shift amount on RV64I
            switch (funct3) {
                case 0x0: cpu->regs[rd] = cpu->regs[rs1] + (uint64_t)imm_i; break; // ADDI
                case 0x2: cpu->regs[rd] = ((int64_t)cpu->regs[rs1] < imm_i) ? 1 : 0; break; // SLTI
                case 0x3: cpu->regs[rd] = (cpu->regs[rs1] < (uint64_t)imm_i) ? 1 : 0; break; // SLTIU
                case 0x4: cpu->regs[rd] = cpu->regs[rs1] ^ (uint64_t)imm_i; break; // XORI
                case 0x6: cpu->regs[rd] = cpu->regs[rs1] | (uint64_t)imm_i; break; // ORI
                case 0x7: cpu->regs[rd] = cpu->regs[rs1] & (uint64_t)imm_i; break; // ANDI
                case 0x1: cpu->regs[rd] = cpu->regs[rs1] << shamt; break; // SLLI
                case 0x5: {
                    if ((funct7 >> 1) == 0x00) {
                        cpu->regs[rd] = cpu->regs[rs1] >> shamt; // SRLI
                    } else if ((funct7 >> 1) == 0x10) {
                        cpu->regs[rd] = (uint64_t)((int64_t)cpu->regs[rs1] >> shamt); // SRAI
                    }
                    break;
                }
            }
            break;
        }
        case 0x1B: { // Register-Immediate Word Group (ADDIW, SLLIW, SRLIW, SRAIW)
            uint32_t shamt = (instr >> 20) & 0x1F; // 5-bit shift amount on W ops
            switch (funct3) {
                case 0x0: cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] + (uint32_t)imm_i); break; // ADDIW
                case 0x1: cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] << shamt); break; // SLLIW
                case 0x5: {
                    if (funct7 == 0x00) {
                        cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] >> shamt); // SRLIW
                    } else if (funct7 == 0x20) {
                        cpu->regs[rd] = (uint64_t)(int64_t)((int32_t)cpu->regs[rs1] >> shamt); // SRAIW
                    }
                    break;
                }
            }
            break;
        }
        case 0x33: { // Register-Register Group (ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND)
            switch (funct3) {
                case 0x0: {
                    if (funct7 == 0x00) cpu->regs[rd] = cpu->regs[rs1] + cpu->regs[rs2]; // ADD
                    else if (funct7 == 0x20) cpu->regs[rd] = cpu->regs[rs1] - cpu->regs[rs2]; // SUB
                    break;
                }
                case 0x1: cpu->regs[rd] = cpu->regs[rs1] << (cpu->regs[rs2] & 0x3F); break; // SLL
                case 0x2: cpu->regs[rd] = ((int64_t)cpu->regs[rs1] < (int64_t)cpu->regs[rs2]) ? 1 : 0; break; // SLT
                case 0x3: cpu->regs[rd] = (cpu->regs[rs1] < cpu->regs[rs2]) ? 1 : 0; break; // SLTU
                case 0x4: cpu->regs[rd] = cpu->regs[rs1] ^ cpu->regs[rs2]; break; // XOR
                case 0x5: {
                    if (funct7 == 0x00) cpu->regs[rd] = cpu->regs[rs1] >> (cpu->regs[rs2] & 0x3F); // SRL
                    else if (funct7 == 0x20) cpu->regs[rd] = (uint64_t)((int64_t)cpu->regs[rs1] >> (cpu->regs[rs2] & 0x3F)); // SRA
                    break;
                }
                case 0x6: cpu->regs[rd] = cpu->regs[rs1] | cpu->regs[rs2]; break; // OR
                case 0x7: cpu->regs[rd] = cpu->regs[rs1] & cpu->regs[rs2]; break; // AND
            }
            break;
        }
        case 0x3B: { // Register-Register Word Group (ADDW, SUBW, SLLW, SRLW, SRAW)
            switch (funct3) {
                case 0x0: {
                    if (funct7 == 0x00) cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] + (uint32_t)cpu->regs[rs2]); // ADDW
                    else if (funct7 == 0x20) cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] - (uint32_t)cpu->regs[rs2]); // SUBW
                    break;
                }
                case 0x1: cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] << (cpu->regs[rs2] & 0x1F)); break; // SLLW
                case 0x5: {
                    if (funct7 == 0x00) cpu->regs[rd] = (uint64_t)(int64_t)(int32_t)((uint32_t)cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1F)); // SRLW
                    else if (funct7 == 0x20) cpu->regs[rd] = (uint64_t)(int64_t)((int32_t)cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1F)); // SRAW
                    break;
                }
            }
            break;
        }
        case 0x73: { // System (ECALL, EBREAK)
            if (imm_i == 0) { // ECALL
                // Check if a7 (x17) == 93 (exit call)
                if (cpu->regs[17] == 93) {
                    cpu->halted = 1;
                    cpu->exit_code = (int)cpu->regs[10]; // exit code in a0 (x10)
                    return 1; // halt
                } else {
                    printf("SYSTEM ECALL: a7=%llu, a0=%llu\n",
                           (unsigned long long)cpu->regs[17],
                           (unsigned long long)cpu->regs[10]);
                }
            } else if (imm_i == 1) { // EBREAK
                cpu->halted = 1;
                return 1; // halt
            }
            break;
        }
        default:
            return -4; // Unknown opcode
    }

    cpu->regs[0] = 0; // x0 hardwired to zero
    cpu->pc = next_pc;

    // Check if pc looped to self
    if (cpu->pc == instr_addr) {
        cpu->halted = 1;
        return 1; // halt
    }

    return 0;
}

void rv64i_print_state(void *context) {
    if (!context) return;
    RV64ICPU *cpu = (RV64ICPU*)context;

    printf("RISC-V RV64I CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%016llX  Halted: %s", (unsigned long long)cpu->pc, cpu->halted ? "Yes" : "No");
    if (cpu->halted) printf(" (Exit Code: %d)", cpu->exit_code);
    printf("\n  Registers:\n");
    for (int i = 0; i < 32; ++i) {
        printf("    %-4s(x%02d): 0x%016llX%s",
               reg_names[i], i, (unsigned long long)cpu->regs[i],
               (i % 2 == 1) ? "\n" : "  ");
    }
}

void rv64i_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    RV64ICPU *cpu = (RV64ICPU*)context;

    if (cpu->pc >= MEM_SIZE) {
        snprintf(buf, buf_len, "<out of RAM>");
        return;
    }

    uint32_t instr = read_mem32(cpu, cpu->pc);
    uint8_t opcode = (uint8_t)(instr & 0x7F);
    uint8_t rd = (uint8_t)((instr >> 7) & 0x1F);
    uint8_t funct3 = (uint8_t)((instr >> 12) & 0x07);
    uint8_t rs1 = (uint8_t)((instr >> 15) & 0x1F);
    uint8_t rs2 = (uint8_t)((instr >> 20) & 0x1F);
    uint8_t funct7 = (uint8_t)((instr >> 25) & 0x7F);

    int32_t imm_i = (int32_t)instr >> 20;
    int32_t imm_s = ((((int32_t)instr) >> 25) << 5) | rd;
    int32_t imm_b = ((((int32_t)instr) >> 31) << 12) |
                    (int32_t)(((instr >> 7) & 1) << 11) |
                    (int32_t)(((instr >> 25) & 0x3F) << 5) |
                    (int32_t)(((instr >> 8) & 0x0F) << 1);
    int32_t imm_u = (int32_t)(instr & 0xFFFFF000);
    int32_t imm_j = ((((int32_t)instr) >> 31) << 20) |
                    (int32_t)(instr & 0x000FF000) |
                    (int32_t)(((instr >> 20) & 1) << 11) |
                    (int32_t)(((instr >> 21) & 0x3FF) << 1);

    switch (opcode) {
        case 0x37:
            snprintf(buf, buf_len, "lui   %s, 0x%X", reg_names[rd], (imm_u >> 12) & 0xFFFFF);
            break;
        case 0x17:
            snprintf(buf, buf_len, "auipc %s, 0x%X", reg_names[rd], (imm_u >> 12) & 0xFFFFF);
            break;
        case 0x6F:
            snprintf(buf, buf_len, "jal   %s, pc%+d (0x%llX)", reg_names[rd], imm_j,
                     (unsigned long long)(cpu->pc + (uint64_t)(int64_t)imm_j));
            break;
        case 0x67:
            snprintf(buf, buf_len, "jalr  %s, %d(%s)", reg_names[rd], imm_i, reg_names[rs1]);
            break;
        case 0x63: {
            const char* branch_ops[] = {"beq", "bne", "?", "?", "blt", "bge", "bltu", "bgeu"};
            snprintf(buf, buf_len, "%-5s %s, %s, pc%+d (0x%llX)",
                     branch_ops[funct3], reg_names[rs1], reg_names[rs2], imm_b,
                     (unsigned long long)(cpu->pc + (uint64_t)(int64_t)imm_b));
            break;
        }
        case 0x03: {
            const char* load_ops[] = {"lb", "lh", "lw", "ld", "lbu", "lhu", "lwu"};
            snprintf(buf, buf_len, "%-5s %s, %d(%s)", load_ops[funct3], reg_names[rd], imm_i, reg_names[rs1]);
            break;
        }
        case 0x23: {
            const char* store_ops[] = {"sb", "sh", "sw", "sd"};
            snprintf(buf, buf_len, "%-5s %s, %d(%s)", store_ops[funct3], reg_names[rs2], imm_s, reg_names[rs1]);
            break;
        }
        case 0x13: {
            const char* imm_ops[] = {"addi", "slli", "slti", "sltiu", "xori", "srli", "ori", "andi"};
            uint32_t shamt = (instr >> 20) & 0x3F;
            if (funct3 == 5 && (funct7 >> 1) == 0x10) {
                snprintf(buf, buf_len, "srai  %s, %s, %u", reg_names[rd], reg_names[rs1], shamt);
            } else if (funct3 == 1 || funct3 == 5) {
                snprintf(buf, buf_len, "%-5s %s, %s, %u", imm_ops[funct3], reg_names[rd], reg_names[rs1], shamt);
            } else {
                snprintf(buf, buf_len, "%-5s %s, %s, %d", imm_ops[funct3], reg_names[rd], reg_names[rs1], imm_i);
            }
            break;
        }
        case 0x1B: {
            uint32_t shamt = (instr >> 20) & 0x1F;
            if (funct3 == 0) {
                snprintf(buf, buf_len, "addiw %s, %s, %d", reg_names[rd], reg_names[rs1], imm_i);
            } else if (funct3 == 1) {
                snprintf(buf, buf_len, "slliw %s, %s, %u", reg_names[rd], reg_names[rs1], shamt);
            } else if (funct3 == 5) {
                snprintf(buf, buf_len, "%-5s %s, %s, %u", (funct7 == 0x20) ? "sraiw" : "srliw",
                         reg_names[rd], reg_names[rs1], shamt);
            } else {
                snprintf(buf, buf_len, "unknown (0x%08X)", instr);
            }
            break;
        }
        case 0x33: {
            const char* r_ops[] = {"add", "sll", "slt", "sltu", "xor", "srl", "or", "and"};
            const char* r_ops_alt[] = {"sub", "?", "?", "?", "?", "sra", "?", "?"};
            const char* op_str = (funct7 == 0x20) ? r_ops_alt[funct3] : r_ops[funct3];
            snprintf(buf, buf_len, "%-5s %s, %s, %s", op_str, reg_names[rd], reg_names[rs1], reg_names[rs2]);
            break;
        }
        case 0x3B: {
            const char* r_ops[] = {"addw", "sllw", "?", "?", "?", "srlw", "?", "?"};
            const char* r_ops_alt[] = {"subw", "?", "?", "?", "?", "sraw", "?", "?"};
            const char* op_str = (funct7 == 0x20) ? r_ops_alt[funct3] : r_ops[funct3];
            snprintf(buf, buf_len, "%-5s %s, %s, %s", op_str, reg_names[rd], reg_names[rs1], reg_names[rs2]);
            break;
        }
        case 0x73: {
            if (imm_i == 0) snprintf(buf, buf_len, "ecall");
            else if (imm_i == 1) snprintf(buf, buf_len, "ebreak");
            else snprintf(buf, buf_len, "sys   0x%X", imm_i);
            break;
        }
        default:
            snprintf(buf, buf_len, "unknown (0x%08X)", instr);
            break;
    }
}
