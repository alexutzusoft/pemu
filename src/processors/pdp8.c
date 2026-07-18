#include "pdp8.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PDP8_MEM_WORDS 4096
#define PDP8_WORD_MASK 07777
#define PDP8_RESET_PC  0200

typedef struct PDP8CPU {
    uint16_t mem[PDP8_MEM_WORDS]; // 12-bit words
    uint16_t ac;                  // 12-bit accumulator
    uint8_t  link;                // 1-bit link
    uint16_t pc;                  // 12-bit program counter
    uint8_t  halted;
    uint32_t ticks;
} PDP8CPU;

// Compute the effective address of a memory-reference instruction fetched at
// instr_pc. Handles zero-page/current-page selection, indirection, and the
// auto-index registers at locations 010-017.
static uint16_t pdp8_effective_address(PDP8CPU *cpu, uint16_t instr, uint16_t instr_pc) {
    uint16_t addr = instr & 0177;
    if (instr & 0200) {
        addr |= instr_pc & 07600; // Current page
    }
    if (instr & 0400) {
        if (addr >= 010 && addr <= 017) {
            cpu->mem[addr] = (cpu->mem[addr] + 1) & PDP8_WORD_MASK;
        }
        addr = cpu->mem[addr];
    }
    return addr;
}

// Execute a Group 1 OPR microinstruction (bit 8 clear).
static void pdp8_opr_group1(PDP8CPU *cpu, uint16_t instr) {
    if (instr & 0200) cpu->ac = 0;                             // CLA
    if (instr & 0100) cpu->link = 0;                           // CLL
    if (instr & 0040) cpu->ac = ~cpu->ac & PDP8_WORD_MASK;     // CMA
    if (instr & 0020) cpu->link = cpu->link ? 0 : 1;           // CML
    if (instr & 0001) {                                        // IAC
        cpu->ac = (cpu->ac + 1) & PDP8_WORD_MASK;
        if (cpu->ac == 0) cpu->link = cpu->link ? 0 : 1;
    }
    if (instr & 0010) {                                        // RAR / RTR
        int count = (instr & 0002) ? 2 : 1;
        for (int i = 0; i < count; ++i) {
            uint16_t low = cpu->ac & 1;
            cpu->ac = (cpu->ac >> 1) | ((uint16_t)cpu->link << 11);
            cpu->link = (uint8_t)low;
        }
    }
    else if (instr & 0004) {                                   // RAL / RTL
        int count = (instr & 0002) ? 2 : 1;
        for (int i = 0; i < count; ++i) {
            uint16_t high = (cpu->ac >> 11) & 1;
            cpu->ac = ((cpu->ac << 1) | cpu->link) & PDP8_WORD_MASK;
            cpu->link = (uint8_t)high;
        }
    }
    else if (instr & 0002) {                                   // BSW
        cpu->ac = ((cpu->ac << 6) | (cpu->ac >> 6)) & PDP8_WORD_MASK;
    }
}

// Execute a Group 2 OPR microinstruction (bit 8 set, bit 0 clear).
// Returns 1 if the CPU halted.
static int pdp8_opr_group2(PDP8CPU *cpu, uint16_t instr) {
    int cond = 0;
    if ((instr & 0100) && (cpu->ac & 04000)) cond = 1; // SMA
    if ((instr & 0040) && cpu->ac == 0)      cond = 1; // SZA
    if ((instr & 0020) && cpu->link)         cond = 1; // SNL
    if (instr & 0010) cond = !cond;                    // SPA/SNA/SZL sense
    if (cond) {
        cpu->pc = (cpu->pc + 1) & PDP8_WORD_MASK;
    }
    if (instr & 0200) cpu->ac = 0;                     // CLA
    // OSR (0004): no switch register attached, no-op
    if (instr & 0002) {                                // HLT
        cpu->halted = 1;
        return 1;
    }
    return 0;
}

void* pdp8_create(void) {
    PDP8CPU *cpu = (PDP8CPU*)calloc(1, sizeof(PDP8CPU));
    if (cpu) {
        cpu->pc = PDP8_RESET_PC;
    }
    return cpu;
}

void pdp8_destroy(void *context) {
    free(context);
}

int pdp8_init(void *context) {
    if (!context) return -1;
    PDP8CPU *cpu = (PDP8CPU*)context;

    memset(cpu->mem, 0, sizeof(cpu->mem));
    cpu->ac = 0;
    cpu->link = 0;
    cpu->pc = PDP8_RESET_PC;
    cpu->halted = 0;
    cpu->ticks = 0;
    return 0;
}

int pdp8_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    PDP8CPU *cpu = (PDP8CPU*)context;
    if (address >= PDP8_MEM_WORDS) return -2;

    size_t words = size / 2;
    for (size_t i = 0; i < words; ++i) {
        uint32_t word_addr = address + (uint32_t)i;
        if (word_addr >= PDP8_MEM_WORDS) break;
        // Big-endian byte pair; upper 4 bits of the first byte are ignored
        cpu->mem[word_addr] = (uint16_t)(((data[i * 2] & 0x0F) << 8) | data[i * 2 + 1]);
    }
    return 0;
}

int pdp8_step(void *context) {
    if (!context) return -1;
    PDP8CPU *cpu = (PDP8CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint16_t instr = cpu->mem[instr_pc] & PDP8_WORD_MASK;
    cpu->pc = (cpu->pc + 1) & PDP8_WORD_MASK;
    cpu->ticks++;

    uint16_t opcode = (instr >> 9) & 07;
    uint16_t ea;

    switch (opcode) {
        case 0: // AND
            ea = pdp8_effective_address(cpu, instr, instr_pc);
            cpu->ac &= cpu->mem[ea];
            break;
        case 1: { // TAD
            ea = pdp8_effective_address(cpu, instr, instr_pc);
            uint16_t sum = cpu->ac + cpu->mem[ea];
            if (sum > PDP8_WORD_MASK) cpu->link = cpu->link ? 0 : 1;
            cpu->ac = sum & PDP8_WORD_MASK;
            break;
        }
        case 2: // ISZ
            ea = pdp8_effective_address(cpu, instr, instr_pc);
            cpu->mem[ea] = (cpu->mem[ea] + 1) & PDP8_WORD_MASK;
            if (cpu->mem[ea] == 0) {
                cpu->pc = (cpu->pc + 1) & PDP8_WORD_MASK;
            }
            break;
        case 3: // DCA
            ea = pdp8_effective_address(cpu, instr, instr_pc);
            cpu->mem[ea] = cpu->ac;
            cpu->ac = 0;
            break;
        case 4: // JMS
            ea = pdp8_effective_address(cpu, instr, instr_pc);
            cpu->mem[ea] = cpu->pc;
            cpu->pc = (ea + 1) & PDP8_WORD_MASK;
            break;
        case 5: // JMP
            ea = pdp8_effective_address(cpu, instr, instr_pc);
            if (ea == instr_pc) {
                cpu->pc = ea;
                cpu->halted = 1;
                return 1; // Jump-to-self, treat as halt
            }
            cpu->pc = ea;
            break;
        case 6: // IOT
            if (instr == 06046) {
                putchar(cpu->ac & 0377);
                fflush(stdout);
            }
            // All other device codes: no-op
            break;
        case 7: // OPR
            if ((instr & 0400) == 0) {
                pdp8_opr_group1(cpu, instr);
            }
            else if ((instr & 0001) == 0) {
                if (pdp8_opr_group2(cpu, instr)) {
                    return 1;
                }
            }
            // Group 3 (EAE): no-op
            break;
        default:
            break;
    }

    return 0;
}

void pdp8_print_state(void *context) {
    if (!context) return;
    PDP8CPU *cpu = (PDP8CPU*)context;

    printf("DEC PDP-8 State:\n");
    printf("  AC: %04o    L: %d    PC: %04o    Ticks: %u%s\n",
           cpu->ac, cpu->link, cpu->pc, cpu->ticks,
           cpu->halted ? "    [HALTED]" : "");
}

void pdp8_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    PDP8CPU *cpu = (PDP8CPU*)context;
    uint16_t instr = cpu->mem[cpu->pc & PDP8_WORD_MASK] & PDP8_WORD_MASK;
    uint16_t opcode = (instr >> 9) & 07;

    static const char *mri_names[6] = { "AND", "TAD", "ISZ", "DCA", "JMS", "JMP" };

    if (opcode <= 5) {
        uint16_t addr = instr & 0177;
        if (instr & 0200) {
            addr |= cpu->pc & 07600;
        }
        snprintf(buf, buf_len, "%s%s %04o", mri_names[opcode],
                 (instr & 0400) ? " I" : "", addr);
    }
    else if (opcode == 6) {
        snprintf(buf, buf_len, "IOT  %02o,%o", (instr >> 3) & 077, instr & 07);
    }
    else if ((instr & 0400) == 0) {
        // Group 1 OPR
        char ops[64] = "";
        if (instr & 0200) strcat(ops, " CLA");
        if (instr & 0100) strcat(ops, " CLL");
        if (instr & 0040) strcat(ops, " CMA");
        if (instr & 0020) strcat(ops, " CML");
        if (instr & 0001) strcat(ops, " IAC");
        if (instr & 0010) strcat(ops, (instr & 0002) ? " RTR" : " RAR");
        else if (instr & 0004) strcat(ops, (instr & 0002) ? " RTL" : " RAL");
        else if (instr & 0002) strcat(ops, " BSW");
        if (ops[0] == '\0') {
            snprintf(buf, buf_len, "NOP");
        } else {
            snprintf(buf, buf_len, "%s", ops + 1);
        }
    }
    else if ((instr & 0001) == 0) {
        // Group 2 OPR
        char ops[64] = "";
        if (instr & 0010) {
            if (instr & 0100) strcat(ops, " SPA");
            if (instr & 0040) strcat(ops, " SNA");
            if (instr & 0020) strcat(ops, " SZL");
            if ((instr & 0160) == 0010) strcat(ops, " SKP");
        } else {
            if (instr & 0100) strcat(ops, " SMA");
            if (instr & 0040) strcat(ops, " SZA");
            if (instr & 0020) strcat(ops, " SNL");
        }
        if (instr & 0200) strcat(ops, " CLA");
        if (instr & 0004) strcat(ops, " OSR");
        if (instr & 0002) strcat(ops, " HLT");
        if (ops[0] == '\0') {
            snprintf(buf, buf_len, "NOP");
        } else {
            snprintf(buf, buf_len, "%s", ops + 1);
        }
    }
    else {
        snprintf(buf, buf_len, "OPR3 %04o", instr);
    }
}
