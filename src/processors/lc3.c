#include "lc3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LC3_MEM_WORDS 65536
#define LC3_PC_START  0x3000

typedef struct LC3CPU {
    uint16_t regs[8];
    uint16_t pc;
    uint16_t memory[LC3_MEM_WORDS];
    uint8_t cc; // Condition codes: bit 2 = N, bit 1 = Z, bit 0 = P
    uint32_t ticks;
    int halted;
} LC3CPU;

#define CC_P 1
#define CC_Z 2
#define CC_N 4

void* lc3_create(void) {
    LC3CPU *cpu = (LC3CPU*)calloc(1, sizeof(LC3CPU));
    return cpu;
}

void lc3_destroy(void *context) {
    free(context);
}

int lc3_init(void *context) {
    if (!context) return -1;
    LC3CPU *cpu = (LC3CPU*)context;

    memset(cpu->regs, 0, sizeof(cpu->regs));
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->pc = LC3_PC_START;
    cpu->cc = CC_Z;
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int lc3_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    LC3CPU *cpu = (LC3CPU*)context;

    if (address == 0) address = LC3_PC_START;
    if (address >= LC3_MEM_WORDS) return -2;

    size_t words = size / 2;
    size_t i;
    for (i = 0; i < words && address + i < LC3_MEM_WORDS; ++i) {
        // Data stream is big-endian 16-bit words
        cpu->memory[address + i] = (uint16_t)((data[i * 2] << 8) | data[i * 2 + 1]);
    }
    return 0;
}

static void set_cc(LC3CPU *cpu, uint16_t val) {
    if (val == 0) cpu->cc = CC_Z;
    else if (val & 0x8000) cpu->cc = CC_N;
    else cpu->cc = CC_P;
}

static int16_t sext(uint16_t val, int bits) {
    uint16_t mask = (uint16_t)(1u << (bits - 1));
    return (int16_t)((val ^ mask) - mask);
}

static uint16_t read_mem(LC3CPU *cpu, uint16_t addr) {
    return cpu->memory[addr];
}

static void write_mem(LC3CPU *cpu, uint16_t addr, uint16_t val) {
    cpu->memory[addr] = val;
}

int lc3_step(void *context) {
    if (!context) return -1;
    LC3CPU *cpu = (LC3CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr = read_mem(cpu, cpu->pc);
    uint16_t opcode = (uint16_t)(instr >> 12);
    uint16_t dr = (instr >> 9) & 0x7;
    uint16_t sr1 = (instr >> 6) & 0x7;
    uint16_t sr2 = instr & 0x7;

    cpu->pc++;
    cpu->ticks++;

    switch (opcode) {
        case 0x1: { // ADD
            uint16_t op2 = (instr & 0x20) ? (uint16_t)sext(instr & 0x1F, 5)
                                          : cpu->regs[sr2];
            cpu->regs[dr] = (uint16_t)(cpu->regs[sr1] + op2);
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0x5: { // AND
            uint16_t op2 = (instr & 0x20) ? (uint16_t)sext(instr & 0x1F, 5)
                                          : cpu->regs[sr2];
            cpu->regs[dr] = (uint16_t)(cpu->regs[sr1] & op2);
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0x9: { // NOT
            cpu->regs[dr] = (uint16_t)~cpu->regs[sr1];
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0x0: { // BR
            uint16_t cond = (instr >> 9) & 0x7;
            if (cond & cpu->cc) {
                cpu->pc = (uint16_t)(cpu->pc + sext(instr & 0x1FF, 9));
            }
            break;
        }
        case 0xC: { // JMP / RET
            cpu->pc = cpu->regs[sr1];
            break;
        }
        case 0x4: { // JSR / JSRR
            cpu->regs[7] = cpu->pc;
            if (instr & 0x800) {
                cpu->pc = (uint16_t)(cpu->pc + sext(instr & 0x7FF, 11)); // JSR
            } else {
                cpu->pc = cpu->regs[sr1]; // JSRR
            }
            break;
        }
        case 0x2: { // LD
            cpu->regs[dr] = read_mem(cpu, (uint16_t)(cpu->pc + sext(instr & 0x1FF, 9)));
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0xA: { // LDI
            uint16_t addr = read_mem(cpu, (uint16_t)(cpu->pc + sext(instr & 0x1FF, 9)));
            cpu->regs[dr] = read_mem(cpu, addr);
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0x6: { // LDR
            cpu->regs[dr] = read_mem(cpu, (uint16_t)(cpu->regs[sr1] + sext(instr & 0x3F, 6)));
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0xE: { // LEA
            cpu->regs[dr] = (uint16_t)(cpu->pc + sext(instr & 0x1FF, 9));
            set_cc(cpu, cpu->regs[dr]);
            break;
        }
        case 0x3: { // ST
            write_mem(cpu, (uint16_t)(cpu->pc + sext(instr & 0x1FF, 9)), cpu->regs[dr]);
            break;
        }
        case 0xB: { // STI
            uint16_t addr = read_mem(cpu, (uint16_t)(cpu->pc + sext(instr & 0x1FF, 9)));
            write_mem(cpu, addr, cpu->regs[dr]);
            break;
        }
        case 0x7: { // STR
            write_mem(cpu, (uint16_t)(cpu->regs[sr1] + sext(instr & 0x3F, 6)), cpu->regs[dr]);
            break;
        }
        case 0x8: { // RTI (no-op in this emulator)
            break;
        }
        case 0xF: { // TRAP
            uint16_t vector = instr & 0xFF;
            cpu->regs[7] = cpu->pc;
            switch (vector) {
                case 0x21: // OUT: print char in R0
                    putchar((char)(cpu->regs[0] & 0xFF));
                    fflush(stdout);
                    break;
                case 0x22: { // PUTS: print NUL-terminated string at R0
                    uint16_t addr = cpu->regs[0];
                    while (read_mem(cpu, addr) != 0) {
                        putchar((char)(read_mem(cpu, addr) & 0xFF));
                        addr++;
                    }
                    fflush(stdout);
                    break;
                }
                case 0x25: // HALT
                    cpu->halted = 1;
                    return 1;
                default: // Other traps are no-ops
                    break;
            }
            break;
        }
        default:
            return -4; // Unknown opcode (0xD is reserved)
    }

    return 0;
}

void lc3_print_state(void *context) {
    if (!context) return;
    LC3CPU *cpu = (LC3CPU*)context;

    printf("LC-3 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  CC: %c  Halted: %s\n",
           cpu->pc,
           (cpu->cc & CC_N) ? 'N' : ((cpu->cc & CC_Z) ? 'Z' : 'P'),
           cpu->halted ? "Yes" : "No");
    printf("  Registers:\n");
    for (int i = 0; i < 8; ++i) {
        printf("    R%d: 0x%04X%s", i, cpu->regs[i], (i % 4 == 3) ? "\n" : "  ");
    }
}

void lc3_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    LC3CPU *cpu = (LC3CPU*)context;

    uint16_t instr = read_mem(cpu, cpu->pc);
    uint16_t opcode = (uint16_t)(instr >> 12);
    uint16_t dr = (instr >> 9) & 0x7;
    uint16_t sr1 = (instr >> 6) & 0x7;
    uint16_t sr2 = instr & 0x7;
    int16_t off9 = sext(instr & 0x1FF, 9);
    int16_t off6 = sext(instr & 0x3F, 6);
    int16_t imm5 = sext(instr & 0x1F, 5);
    uint16_t target9 = (uint16_t)(cpu->pc + 1 + off9);

    switch (opcode) {
        case 0x1: // ADD
            if (instr & 0x20)
                snprintf(buf, buf_len, "ADD  R%u, R%u, #%d", dr, sr1, imm5);
            else
                snprintf(buf, buf_len, "ADD  R%u, R%u, R%u", dr, sr1, sr2);
            break;
        case 0x5: // AND
            if (instr & 0x20)
                snprintf(buf, buf_len, "AND  R%u, R%u, #%d", dr, sr1, imm5);
            else
                snprintf(buf, buf_len, "AND  R%u, R%u, R%u", dr, sr1, sr2);
            break;
        case 0x9:
            snprintf(buf, buf_len, "NOT  R%u, R%u", dr, sr1);
            break;
        case 0x0: {
            char cond[4];
            int n = 0;
            if (instr & 0x800) cond[n++] = 'n';
            if (instr & 0x400) cond[n++] = 'z';
            if (instr & 0x200) cond[n++] = 'p';
            cond[n] = '\0';
            if (n == 0)
                snprintf(buf, buf_len, "NOP");
            else
                snprintf(buf, buf_len, "BR%-3s x%04X", cond, target9);
            break;
        }
        case 0xC:
            if (sr1 == 7)
                snprintf(buf, buf_len, "RET");
            else
                snprintf(buf, buf_len, "JMP  R%u", sr1);
            break;
        case 0x4:
            if (instr & 0x800)
                snprintf(buf, buf_len, "JSR  x%04X",
                         (uint16_t)(cpu->pc + 1 + sext(instr & 0x7FF, 11)));
            else
                snprintf(buf, buf_len, "JSRR R%u", sr1);
            break;
        case 0x2:
            snprintf(buf, buf_len, "LD   R%u, x%04X", dr, target9);
            break;
        case 0xA:
            snprintf(buf, buf_len, "LDI  R%u, x%04X", dr, target9);
            break;
        case 0x6:
            snprintf(buf, buf_len, "LDR  R%u, R%u, #%d", dr, sr1, off6);
            break;
        case 0xE:
            snprintf(buf, buf_len, "LEA  R%u, x%04X", dr, target9);
            break;
        case 0x3:
            snprintf(buf, buf_len, "ST   R%u, x%04X", dr, target9);
            break;
        case 0xB:
            snprintf(buf, buf_len, "STI  R%u, x%04X", dr, target9);
            break;
        case 0x7:
            snprintf(buf, buf_len, "STR  R%u, R%u, #%d", dr, sr1, off6);
            break;
        case 0x8:
            snprintf(buf, buf_len, "RTI");
            break;
        case 0xF: {
            uint16_t vector = instr & 0xFF;
            switch (vector) {
                case 0x20: snprintf(buf, buf_len, "GETC"); break;
                case 0x21: snprintf(buf, buf_len, "OUT"); break;
                case 0x22: snprintf(buf, buf_len, "PUTS"); break;
                case 0x23: snprintf(buf, buf_len, "IN"); break;
                case 0x24: snprintf(buf, buf_len, "PUTSP"); break;
                case 0x25: snprintf(buf, buf_len, "HALT"); break;
                default: snprintf(buf, buf_len, "TRAP x%02X", vector); break;
            }
            break;
        }
        default:
            snprintf(buf, buf_len, "unknown (0x%04X)", instr);
            break;
    }
}
