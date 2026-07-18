#include "rv32i.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE (128 * 1024)

typedef struct RV32ICPU {
    uint32_t regs[32];
    uint32_t pc;
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
    int exit_code;
} RV32ICPU;

static const char* reg_names[] = {
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"
};

void* rv32i_create(void) {
    RV32ICPU *cpu = (RV32ICPU*)calloc(1, sizeof(RV32ICPU));
    return cpu;
}

void rv32i_destroy(void *context) {
    free(context);
}

int rv32i_init(void *context) {
    if (!context) return -1;
    RV32ICPU *cpu = (RV32ICPU*)context;
    
    memset(cpu->regs, 0, sizeof(cpu->regs));
    cpu->regs[2] = MEM_SIZE - 4; // sp (Stack Pointer) initialized near top of RAM
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->exit_code = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int rv32i_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    RV32ICPU *cpu = (RV32ICPU*)context;
    
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Memory Helpers with alignment and bounds checks
static inline uint32_t read_mem32(RV32ICPU *cpu, uint32_t addr) {
    if (addr + 3 >= MEM_SIZE) return 0;
    // RISC-V supports unaligned access depending on implementation, 
    // but for emulator simplicity we access via memcpy or direct pointer if aligned.
    uint32_t val;
    memcpy(&val, &cpu->memory[addr], 4);
    return val;
}

static inline uint16_t read_mem16(RV32ICPU *cpu, uint32_t addr) {
    if (addr + 1 >= MEM_SIZE) return 0;
    uint16_t val;
    memcpy(&val, &cpu->memory[addr], 2);
    return val;
}

static inline uint8_t read_mem8(RV32ICPU *cpu, uint32_t addr) {
    if (addr >= MEM_SIZE) return 0;
    return cpu->memory[addr];
}

static inline void write_mem32(RV32ICPU *cpu, uint32_t addr, uint32_t val) {
    if (addr + 3 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 4);
}

static inline void write_mem16(RV32ICPU *cpu, uint32_t addr, uint16_t val) {
    if (addr + 1 >= MEM_SIZE) return;
    memcpy(&cpu->memory[addr], &val, 2);
}

static inline void write_mem8(RV32ICPU *cpu, uint32_t addr, uint8_t val) {
    if (addr >= MEM_SIZE) return;
    cpu->memory[addr] = val;
}

int rv32i_step(void *context) {
    if (!context) return -1;
    RV32ICPU *cpu = (RV32ICPU*)context;
    
    if (cpu->halted) return 1;
    if (cpu->pc >= MEM_SIZE || (cpu->pc & 3) != 0) {
        return -3; // PC out of bounds or misaligned
    }
    
    uint32_t instr_addr = cpu->pc;
    uint32_t instr = read_mem32(cpu, instr_addr);
    
    // Decode fields
    uint8_t opcode = instr & 0x7F;
    uint8_t rd = (instr >> 7) & 0x1F;
    uint8_t funct3 = (instr >> 12) & 0x07;
    uint8_t rs1 = (instr >> 15) & 0x1F;
    uint8_t rs2 = (instr >> 20) & 0x1F;
    uint8_t funct7 = (instr >> 25) & 0x7F;
    
    // Default next PC
    uint32_t next_pc = cpu->pc + 4;
    cpu->ticks++;
    
    // Immediates
    int32_t imm_i = (int32_t)instr >> 20;
    int32_t imm_s = ((((int32_t)instr) >> 25) << 5) | rd;
    int32_t imm_b = ((((int32_t)instr) >> 31) << 12) |
                    (((instr >> 7) & 1) << 11) |
                    (((instr >> 25) & 0x3F) << 5) |
                    (((instr >> 8) & 0x0F) << 1);
    int32_t imm_u = (int32_t)(instr & 0xFFFFF000);
    int32_t imm_j = ((((int32_t)instr) >> 31) << 20) |
                    (instr & 0x000FF000) |
                    (((instr >> 20) & 1) << 11) |
                    (((instr >> 21) & 0x3FF) << 1);
    
    switch (opcode) {
        case 0x37: { // LUI (Load Upper Immediate)
            cpu->regs[rd] = imm_u;
            break;
        }
        case 0x17: { // AUIPC (Add Upper Immediate to PC)
            cpu->regs[rd] = instr_addr + imm_u;
            break;
        }
        case 0x6F: { // JAL (Jump and Link)
            cpu->regs[rd] = next_pc;
            next_pc = instr_addr + imm_j;
            break;
        }
        case 0x67: { // JALR (Jump and Link Register)
            uint32_t target = (cpu->regs[rs1] + imm_i) & ~1;
            cpu->regs[rd] = next_pc;
            next_pc = target;
            break;
        }
        case 0x63: { // Branch Group (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            int take_branch = 0;
            switch (funct3) {
                case 0x0: take_branch = (cpu->regs[rs1] == cpu->regs[rs2]); break; // BEQ
                case 0x1: take_branch = (cpu->regs[rs1] != cpu->regs[rs2]); break; // BNE
                case 0x4: take_branch = ((int32_t)cpu->regs[rs1] < (int32_t)cpu->regs[rs2]); break; // BLT
                case 0x5: take_branch = ((int32_t)cpu->regs[rs1] >= (int32_t)cpu->regs[rs2]); break; // BGE
                case 0x6: take_branch = (cpu->regs[rs1] < cpu->regs[rs2]); break; // BLTU
                case 0x7: take_branch = (cpu->regs[rs1] >= cpu->regs[rs2]); break; // BGEU
            }
            if (take_branch) {
                next_pc = instr_addr + imm_b;
            }
            break;
        }
        case 0x03: { // Load Group (LB, LH, LW, LBU, LHU)
            uint32_t addr = cpu->regs[rs1] + imm_i;
            switch (funct3) {
                case 0x0: cpu->regs[rd] = (int32_t)(int8_t)read_mem8(cpu, addr); break;  // LB
                case 0x1: cpu->regs[rd] = (int32_t)(int16_t)read_mem16(cpu, addr); break; // LH
                case 0x2: cpu->regs[rd] = read_mem32(cpu, addr); break;                 // LW
                case 0x4: cpu->regs[rd] = read_mem8(cpu, addr); break;                  // LBU
                case 0x5: cpu->regs[rd] = read_mem16(cpu, addr); break;                 // LHU
            }
            break;
        }
        case 0x23: { // Store Group (SB, SH, SW)
            uint32_t addr = cpu->regs[rs1] + imm_s;
            switch (funct3) {
                case 0x0: write_mem8(cpu, addr, (uint8_t)cpu->regs[rs2]); break;  // SB
                case 0x1: write_mem16(cpu, addr, (uint16_t)cpu->regs[rs2]); break; // SH
                case 0x2: write_mem32(cpu, addr, cpu->regs[rs2]); break;           // SW
            }
            break;
        }
        case 0x13: { // Register-Immediate Group (ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI)
            uint32_t shamt = rs2; // Shift amount is rs2 field (bits 24-20)
            switch (funct3) {
                case 0x0: cpu->regs[rd] = cpu->regs[rs1] + imm_i; break; // ADDI
                case 0x2: cpu->regs[rd] = ((int32_t)cpu->regs[rs1] < imm_i) ? 1 : 0; break; // SLTI
                case 0x3: cpu->regs[rd] = (cpu->regs[rs1] < (uint32_t)imm_i) ? 1 : 0; break; // SLTIU
                case 0x4: cpu->regs[rd] = cpu->regs[rs1] ^ imm_i; break; // XORI
                case 0x6: cpu->regs[rd] = cpu->regs[rs1] | imm_i; break; // ORI
                case 0x7: cpu->regs[rd] = cpu->regs[rs1] & imm_i; break; // ANDI
                case 0x1: cpu->regs[rd] = cpu->regs[rs1] << shamt; break; // SLLI
                case 0x5: {
                    if (funct7 == 0x00) {
                        cpu->regs[rd] = cpu->regs[rs1] >> shamt; // SRLI
                    } else if (funct7 == 0x20) {
                        cpu->regs[rd] = (uint32_t)((int32_t)cpu->regs[rs1] >> shamt); // SRAI
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
                case 0x1: cpu->regs[rd] = cpu->regs[rs1] << (cpu->regs[rs2] & 0x1F); break; // SLL
                case 0x2: cpu->regs[rd] = ((int32_t)cpu->regs[rs1] < (int32_t)cpu->regs[rs2]) ? 1 : 0; break; // SLT
                case 0x3: cpu->regs[rd] = (cpu->regs[rs1] < cpu->regs[rs2]) ? 1 : 0; break; // SLTU
                case 0x4: cpu->regs[rd] = cpu->regs[rs1] ^ cpu->regs[rs2]; break; // XOR
                case 0x5: {
                    if (funct7 == 0x00) cpu->regs[rd] = cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1F); // SRL
                    else if (funct7 == 0x20) cpu->regs[rd] = (uint32_t)((int32_t)cpu->regs[rs1] >> (cpu->regs[rs2] & 0x1F)); // SRA
                    break;
                }
                case 0x6: cpu->regs[rd] = cpu->regs[rs1] | cpu->regs[rs2]; break; // OR
                case 0x7: cpu->regs[rd] = cpu->regs[rs1] & cpu->regs[rs2]; break; // AND
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
                    printf("SYSTEM ECALL: a7=%u, a0=%u\n", cpu->regs[17], cpu->regs[10]);
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

void rv32i_print_state(void *context) {
    if (!context) return;
    RV32ICPU *cpu = (RV32ICPU*)context;
    
    printf("RISC-V RV32I CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%08X  Halted: %s", cpu->pc, cpu->halted ? "Yes" : "No");
    if (cpu->halted) printf(" (Exit Code: %d)", cpu->exit_code);
    printf("\n  Registers:\n");
    for (int i = 0; i < 32; ++i) {
        printf("    %-4s(x%02d): 0x%08X%s", 
               reg_names[i], i, cpu->regs[i],
               (i % 4 == 3) ? "\n" : "  ");
    }
}

void rv32i_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    RV32ICPU *cpu = (RV32ICPU*)context;
    
    if (cpu->pc >= MEM_SIZE) {
        snprintf(buf, buf_len, "<out of RAM>");
        return;
    }
    
    uint32_t instr = read_mem32(cpu, cpu->pc);
    uint8_t opcode = instr & 0x7F;
    uint8_t rd = (instr >> 7) & 0x1F;
    uint8_t funct3 = (instr >> 12) & 0x07;
    uint8_t rs1 = (instr >> 15) & 0x1F;
    uint8_t rs2 = (instr >> 20) & 0x1F;
    uint8_t funct7 = (instr >> 25) & 0x7F;
    
    int32_t imm_i = (int32_t)instr >> 20;
    int32_t imm_s = ((((int32_t)instr) >> 25) << 5) | rd;
    int32_t imm_b = ((((int32_t)instr) >> 31) << 12) |
                    (((instr >> 7) & 1) << 11) |
                    (((instr >> 25) & 0x3F) << 5) |
                    (((instr >> 8) & 0x0F) << 1);
    int32_t imm_u = (int32_t)(instr & 0xFFFFF000);
    int32_t imm_j = ((((int32_t)instr) >> 31) << 20) |
                    (instr & 0x000FF000) |
                    (((instr >> 20) & 1) << 11) |
                    (((instr >> 21) & 0x3FF) << 1);
                    
    switch (opcode) {
        case 0x37:
            snprintf(buf, buf_len, "lui   %s, 0x%X", reg_names[rd], (imm_u >> 12) & 0xFFFFF);
            break;
        case 0x17:
            snprintf(buf, buf_len, "auipc %s, 0x%X", reg_names[rd], (imm_u >> 12) & 0xFFFFF);
            break;
        case 0x6F:
            snprintf(buf, buf_len, "jal   %s, pc%+d (0x%X)", reg_names[rd], imm_j, cpu->pc + imm_j);
            break;
        case 0x67:
            snprintf(buf, buf_len, "jalr  %s, %d(%s)", reg_names[rd], imm_i, reg_names[rs1]);
            break;
        case 0x63: {
            const char* branch_ops[] = {"beq", "bne", "?", "?", "blt", "bge", "bltu", "bgeu"};
            snprintf(buf, buf_len, "%-5s %s, %s, pc%+d (0x%X)", 
                     branch_ops[funct3], reg_names[rs1], reg_names[rs2], imm_b, cpu->pc + imm_b);
            break;
        }
        case 0x03: {
            const char* load_ops[] = {"lb", "lh", "lw", "?", "lbu", "lhu"};
            snprintf(buf, buf_len, "%-5s %s, %d(%s)", load_ops[funct3], reg_names[rd], imm_i, reg_names[rs1]);
            break;
        }
        case 0x23: {
            const char* store_ops[] = {"sb", "sh", "sw"};
            snprintf(buf, buf_len, "%-5s %s, %d(%s)", store_ops[funct3], reg_names[rs2], imm_s, reg_names[rs1]);
            break;
        }
        case 0x13: {
            const char* imm_ops[] = {"addi", "slli", "slti", "sltiu", "xori", "srli", "ori", "andi"};
            if (funct3 == 5 && funct7 == 0x20) {
                snprintf(buf, buf_len, "srai  %s, %s, %d", reg_names[rd], reg_names[rs1], rs2);
            } else if (funct3 == 1 || funct3 == 5) {
                snprintf(buf, buf_len, "%-5s %s, %s, %d", imm_ops[funct3], reg_names[rd], reg_names[rs1], rs2);
            } else {
                snprintf(buf, buf_len, "%-5s %s, %s, %d", imm_ops[funct3], reg_names[rd], reg_names[rs1], imm_i);
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
