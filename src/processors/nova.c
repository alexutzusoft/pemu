#include "nova.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NOVA_MEM_WORDS 32768
#define NOVA_ADDR_MASK 077777
#define NOVA_WORD_MASK 0177777
#define NOVA_HALT      063077   // DOC 0,CPU

typedef struct NovaCPU {
    uint16_t mem[NOVA_MEM_WORDS]; // 16-bit words
    uint16_t ac[4];               // Accumulators AC0-AC3
    uint8_t  carry;               // 1-bit carry
    uint16_t pc;                  // 15-bit program counter
    uint8_t  halted;
    uint32_t ticks;
} NovaCPU;

// Compute the effective address of a memory-reference instruction fetched at
// instr_pc. Handles page-zero/PC-relative/AC2-relative/AC3-relative modes and
// indirect chains with auto-increment (020-027) and auto-decrement (030-037).
static uint16_t nova_effective_address(NovaCPU *cpu, uint16_t instr, uint16_t instr_pc) {
    uint16_t addr;
    switch ((instr >> 8) & 03) {
        case 0:  // Page zero: unsigned 8-bit address
            addr = instr & 0377;
            break;
        case 1:  // PC-relative: signed 8-bit displacement
            addr = (uint16_t)((instr_pc + (uint16_t)(int8_t)(instr & 0377)) & NOVA_ADDR_MASK);
            break;
        case 2:  // AC2-relative
            addr = (uint16_t)((cpu->ac[2] + (uint16_t)(int8_t)(instr & 0377)) & NOVA_ADDR_MASK);
            break;
        default: // AC3-relative
            addr = (uint16_t)((cpu->ac[3] + (uint16_t)(int8_t)(instr & 0377)) & NOVA_ADDR_MASK);
            break;
    }

    if (instr & 02000) { // Indirect
        for (int depth = 0; depth < 65536; ++depth) {
            uint16_t word;
            if (addr >= 020 && addr <= 027) {        // Auto-increment
                cpu->mem[addr] = (uint16_t)((cpu->mem[addr] + 1) & NOVA_WORD_MASK);
            }
            else if (addr >= 030 && addr <= 037) {   // Auto-decrement
                cpu->mem[addr] = (uint16_t)((cpu->mem[addr] - 1) & NOVA_WORD_MASK);
            }
            word = cpu->mem[addr];
            addr = word & NOVA_ADDR_MASK;
            if ((word & 0100000) == 0) break;        // Chain ends when bit 0 clear
        }
    }
    return addr;
}

// Execute a two-accumulator ALU instruction. Returns 1 if the following
// instruction should be skipped.
static int nova_alu(NovaCPU *cpu, uint16_t instr) {
    uint16_t src = (instr >> 13) & 03;
    uint16_t dst = (instr >> 11) & 03;
    uint32_t s = cpu->ac[src];
    uint32_t d = cpu->ac[dst];
    uint32_t c = cpu->carry;
    uint32_t res;
    int skip;

    switch ((instr >> 4) & 03) { // Carry base
        case 1: c = 0; break;      // Z
        case 2: c = 1; break;      // O
        case 3: c ^= 1; break;     // C
        default: break;
    }

    switch ((instr >> 8) & 07) { // Function
        case 0: res = (c << 16) | (~s & 0xFFFFu); break;           // COM
        case 1: res = (c << 16) + (~s & 0xFFFFu) + 1; break;       // NEG
        case 2: res = (c << 16) | s; break;                        // MOV
        case 3: res = (c << 16) + s + 1; break;                    // INC
        case 4: res = (c << 16) + d + (~s & 0xFFFFu); break;       // ADC
        case 5: res = (c << 16) + d + (~s & 0xFFFFu) + 1; break;   // SUB
        case 6: res = (c << 16) + d + s; break;                    // ADD
        default: res = (c << 16) | (d & s); break;                 // AND
    }
    res &= 0x1FFFFu;

    switch ((instr >> 6) & 03) { // Shift
        case 1: res = ((res << 1) | (res >> 16)) & 0x1FFFFu; break;        // L
        case 2: res = ((res >> 1) | ((res & 1) << 16)) & 0x1FFFFu; break;  // R
        case 3: res = (res & 0x10000u) | ((res >> 8) & 0xFFu) |
                      ((res & 0xFFu) << 8); break;                         // S
        default: break;
    }

    switch (instr & 07) { // Skip condition
        case 1: skip = 1; break;                                       // SKP
        case 2: skip = (res & 0x10000u) == 0; break;                   // SZC
        case 3: skip = (res & 0x10000u) != 0; break;                   // SNC
        case 4: skip = (res & 0xFFFFu) == 0; break;                    // SZR
        case 5: skip = (res & 0xFFFFu) != 0; break;                    // SNR
        case 6: skip = (res & 0x10000u) == 0 ||
                       (res & 0xFFFFu) == 0; break;                    // SEZ
        case 7: skip = (res & 0x10000u) != 0 &&
                       (res & 0xFFFFu) != 0; break;                    // SBN
        default: skip = 0; break;
    }

    if ((instr & 010) == 0) { // Load unless no-load bit set
        cpu->ac[dst] = (uint16_t)(res & 0xFFFFu);
        cpu->carry = (uint8_t)(res >> 16);
    }
    return skip;
}

void* nova_create(void) {
    return calloc(1, sizeof(NovaCPU));
}

void nova_destroy(void *context) {
    free(context);
}

int nova_init(void *context) {
    if (!context) return -1;
    NovaCPU *cpu = (NovaCPU*)context;

    memset(cpu, 0, sizeof(*cpu));
    return 0;
}

int nova_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    NovaCPU *cpu = (NovaCPU*)context;
    if (address >= NOVA_MEM_WORDS) return -2;

    size_t words = size / 2;
    for (size_t i = 0; i < words; ++i) {
        uint32_t word_addr = address + (uint32_t)i;
        if (word_addr >= NOVA_MEM_WORDS) break;
        // Big-endian byte pair
        cpu->mem[word_addr] = (uint16_t)((data[i * 2] << 8) | data[i * 2 + 1]);
    }
    return 0;
}

int nova_step(void *context) {
    if (!context) return -1;
    NovaCPU *cpu = (NovaCPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint16_t instr = cpu->mem[instr_pc];
    cpu->pc = (uint16_t)((cpu->pc + 1) & NOVA_ADDR_MASK);
    cpu->ticks++;

    if (instr & 0100000) { // Two-accumulator ALU format
        if (nova_alu(cpu, instr)) {
            cpu->pc = (uint16_t)((cpu->pc + 1) & NOVA_ADDR_MASK);
        }
        return 0;
    }

    uint16_t ea;
    switch ((instr >> 13) & 03) {
        case 0: // No-accumulator memory reference
            ea = nova_effective_address(cpu, instr, instr_pc);
            switch ((instr >> 11) & 03) {
                case 0: // JMP
                    cpu->pc = ea;
                    break;
                case 1: // JSR
                    cpu->ac[3] = cpu->pc;
                    cpu->pc = ea;
                    break;
                case 2: // ISZ
                    cpu->mem[ea] = (uint16_t)((cpu->mem[ea] + 1) & NOVA_WORD_MASK);
                    if (cpu->mem[ea] == 0) {
                        cpu->pc = (uint16_t)((cpu->pc + 1) & NOVA_ADDR_MASK);
                    }
                    break;
                default: // DSZ
                    cpu->mem[ea] = (uint16_t)((cpu->mem[ea] - 1) & NOVA_WORD_MASK);
                    if (cpu->mem[ea] == 0) {
                        cpu->pc = (uint16_t)((cpu->pc + 1) & NOVA_ADDR_MASK);
                    }
                    break;
            }
            break;
        case 1: // LDA
            ea = nova_effective_address(cpu, instr, instr_pc);
            cpu->ac[(instr >> 11) & 03] = cpu->mem[ea];
            break;
        case 2: // STA
            ea = nova_effective_address(cpu, instr, instr_pc);
            cpu->mem[ea] = cpu->ac[(instr >> 11) & 03];
            break;
        default: // I/O
            if (instr == NOVA_HALT) {
                cpu->halted = 1;
                return 1;
            }
            // All other I/O instructions: no-op
            break;
    }

    return 0;
}

void nova_print_state(void *context) {
    if (!context) return;
    NovaCPU *cpu = (NovaCPU*)context;

    printf("Data General Nova State:\n");
    printf("  AC0: %06o  AC1: %06o  AC2: %06o  AC3: %06o\n",
           cpu->ac[0], cpu->ac[1], cpu->ac[2], cpu->ac[3]);
    printf("  C: %d    PC: %05o    Ticks: %u%s\n",
           cpu->carry, cpu->pc, cpu->ticks,
           cpu->halted ? "    [HALTED]" : "");
}

// Format the address part of a memory-reference instruction: "[@]disp[,index]".
static void nova_format_addr(char *buf, size_t buf_len, uint16_t instr) {
    uint16_t mode = (instr >> 8) & 03;
    const char *ind = (instr & 02000) ? "@" : "";
    if (mode == 0) {
        snprintf(buf, buf_len, "%s%o", ind, instr & 0377);
    } else {
        int disp = (int8_t)(instr & 0377);
        if (disp < 0) {
            snprintf(buf, buf_len, "%s-%o,%o", ind, -disp, mode);
        } else {
            snprintf(buf, buf_len, "%s%o,%o", ind, disp, mode);
        }
    }
}

void nova_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    NovaCPU *cpu = (NovaCPU*)context;
    uint16_t instr = cpu->mem[cpu->pc & NOVA_ADDR_MASK];
    char addr[16];

    if (instr & 0100000) { // Two-accumulator ALU format
        static const char *fns[8] = {
            "COM", "NEG", "MOV", "INC", "ADC", "SUB", "ADD", "AND"
        };
        static const char *carries[4] = { "", "Z", "O", "C" };
        static const char *shifts[4]  = { "", "L", "R", "S" };
        static const char *skips[8] = {
            "", ",SKP", ",SZC", ",SNC", ",SZR", ",SNR", ",SEZ", ",SBN"
        };
        snprintf(buf, buf_len, "%s%s%s%s %o,%o%s",
                 fns[(instr >> 8) & 07],
                 carries[(instr >> 4) & 03],
                 shifts[(instr >> 6) & 03],
                 (instr & 010) ? "#" : "",
                 (instr >> 13) & 03, (instr >> 11) & 03,
                 skips[instr & 07]);
        return;
    }

    switch ((instr >> 13) & 03) {
        case 0: { // No-accumulator memory reference
            static const char *mri[4] = { "JMP", "JSR", "ISZ", "DSZ" };
            nova_format_addr(addr, sizeof(addr), instr);
            snprintf(buf, buf_len, "%s %s", mri[(instr >> 11) & 03], addr);
            break;
        }
        case 1:
        case 2:
            nova_format_addr(addr, sizeof(addr), instr);
            snprintf(buf, buf_len, "%s %o,%s",
                     ((instr >> 13) & 03) == 1 ? "LDA" : "STA",
                     (instr >> 11) & 03, addr);
            break;
        default: // I/O
            if (instr == NOVA_HALT) {
                snprintf(buf, buf_len, "HALT");
            } else {
                static const char *io[8] = {
                    "NIO", "DIA", "DOA", "DIB", "DOB", "DIC", "DOC", "SKP"
                };
                static const char *ctrls[4] = { "", "S", "C", "P" };
                snprintf(buf, buf_len, "%s%s %o,%o",
                         io[(instr >> 8) & 07], ctrls[(instr >> 6) & 03],
                         (instr >> 11) & 03, instr & 077);
            }
            break;
    }
}
