#include "j1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// J1 Forth CPU (James Bowman's design)
// - 8K x 16-bit word memory, 13-bit word addressing
// - 33-deep data stack (T register + 32-entry array), 32-deep return stack
// - 13-bit program counter
//
// Memory-mapped I/O convention: a store (N->[T]) with T >= 0xF000 prints
// the low byte of N as a character to stdout instead of writing memory.

#define J1_MEM_WORDS 8192       // 8K x 16-bit words
#define J1_PC_MASK   0x1FFF     // 13-bit program counter / word address
#define J1_DSTACK    32         // data stack body (plus T register = 33 deep)
#define J1_RSTACK    32         // return stack depth
#define J1_IO_BASE   0xF000     // writes at/above this address go to stdout

typedef struct J1CPU {
    uint16_t memory[J1_MEM_WORDS];
    uint16_t dstack[J1_DSTACK]; // data stack body (below T)
    uint16_t rstack[J1_RSTACK]; // return stack
    uint16_t t;                 // T: top of data stack register
    uint16_t pc;                // 13-bit program counter (word address)
    int dsp;                    // data stack pointer (index of N)
    int rsp;                    // return stack pointer (index of R)
    uint32_t ticks;
    int halted;
} J1CPU;

// Stack pointer deltas: 2-bit signed field {00: 0, 01: +1, 10: -2, 11: -1}
static const int stack_delta[4] = { 0, 1, -2, -1 };

// T' ALU function names (bits 11-8 of an ALU instruction)
static const char* alu_op_names[16] = {
    "T", "N", "T+N", "T&N", "T|N", "T^N", "~T", "N==T",
    "N<T", "N>>T", "T-1", "R", "[T]", "N<<T", "depth", "Nu<T"
};

void* j1_create(void) {
    J1CPU *cpu = (J1CPU*)calloc(1, sizeof(J1CPU));
    return cpu;
}

void j1_destroy(void *context) {
    free(context);
}

int j1_init(void *context) {
    if (!context) return -1;
    J1CPU *cpu = (J1CPU*)context;

    memset(cpu->memory, 0, sizeof(cpu->memory));
    memset(cpu->dstack, 0, sizeof(cpu->dstack));
    memset(cpu->rstack, 0, sizeof(cpu->rstack));
    cpu->t = 0;
    cpu->pc = 0;
    cpu->dsp = 0;
    cpu->rsp = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

// address is a word address; data holds little-endian 16-bit words
int j1_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    J1CPU *cpu = (J1CPU*)context;

    if (address >= J1_MEM_WORDS) return -2;

    size_t words = size / 2;
    if (address + words > J1_MEM_WORDS) {
        words = J1_MEM_WORDS - address;
    }
    for (size_t i = 0; i < words; ++i) {
        cpu->memory[address + i] =
            (uint16_t)(data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
    }
    // Odd trailing byte becomes the low byte of one more word
    if ((size & 1) && (address + words < J1_MEM_WORDS)) {
        cpu->memory[address + words] = data[size - 1];
    }
    return 0;
}

static inline uint16_t j1_mem_read(J1CPU *cpu, uint16_t addr) {
    if (addr >= J1_IO_BASE) return 0; // reads from I/O space return 0
    return cpu->memory[addr & J1_PC_MASK];
}

static inline void j1_mem_write(J1CPU *cpu, uint16_t addr, uint16_t val) {
    if (addr >= J1_IO_BASE) {
        // Memory-mapped I/O: print low byte as a character to stdout
        putchar(val & 0xFF);
        fflush(stdout);
        return;
    }
    cpu->memory[addr & J1_PC_MASK] = val;
}

int j1_step(void *context) {
    if (!context) return -1;
    J1CPU *cpu = (J1CPU*)context;

    if (cpu->halted) return 1;

    uint16_t pc = cpu->pc & J1_PC_MASK;
    uint16_t insn = cpu->memory[pc];
    uint16_t next_pc = (uint16_t)((pc + 1) & J1_PC_MASK);
    uint16_t new_pc = next_pc;
    uint16_t target = insn & J1_PC_MASK;
    cpu->ticks++;

    if (insn & 0x8000) {
        // Literal: push 15-bit value
        cpu->dsp = (cpu->dsp + 1) & (J1_DSTACK - 1);
        cpu->dstack[cpu->dsp] = cpu->t;
        cpu->t = insn & 0x7FFF;
    } else {
        switch ((insn >> 13) & 0x3) {
            case 0: // Jump
                new_pc = target;
                break;
            case 1: // Conditional jump: pop T, jump if zero
                if (cpu->t == 0) new_pc = target;
                cpu->t = cpu->dstack[cpu->dsp];
                cpu->dsp = (cpu->dsp - 1) & (J1_DSTACK - 1);
                break;
            case 2: // Call: push PC+1 to return stack
                cpu->rsp = (cpu->rsp + 1) & (J1_RSTACK - 1);
                cpu->rstack[cpu->rsp] = next_pc;
                new_pc = target;
                break;
            case 3: { // ALU
                uint16_t t = cpu->t;
                uint16_t n = cpu->dstack[cpu->dsp];
                uint16_t r = cpu->rstack[cpu->rsp];
                uint16_t tp = 0;

                switch ((insn >> 8) & 0xF) {
                    case 0x0: tp = t; break;
                    case 0x1: tp = n; break;
                    case 0x2: tp = (uint16_t)(t + n); break;
                    case 0x3: tp = t & n; break;
                    case 0x4: tp = t | n; break;
                    case 0x5: tp = t ^ n; break;
                    case 0x6: tp = (uint16_t)~t; break;
                    case 0x7: tp = (n == t) ? 0xFFFF : 0; break;
                    case 0x8: tp = ((int16_t)n < (int16_t)t) ? 0xFFFF : 0; break;
                    case 0x9: tp = (uint16_t)(n >> (t & 0xF)); break;
                    case 0xA: tp = (uint16_t)(t - 1); break;
                    case 0xB: tp = r; break;
                    case 0xC: tp = j1_mem_read(cpu, t); break;
                    case 0xD: tp = (uint16_t)(n << (t & 0xF)); break;
                    case 0xE: tp = (uint16_t)cpu->dsp; break;
                    case 0xF: tp = (n < t) ? 0xFFFF : 0; break;
                }

                if (insn & 0x1000) { // R->PC
                    new_pc = r & J1_PC_MASK;
                }

                cpu->dsp = (cpu->dsp + stack_delta[insn & 0x3]) & (J1_DSTACK - 1);
                cpu->rsp = (cpu->rsp + stack_delta[(insn >> 2) & 0x3]) & (J1_RSTACK - 1);

                if (insn & 0x0080) cpu->dstack[cpu->dsp] = t;  // T->N
                if (insn & 0x0040) cpu->rstack[cpu->rsp] = t;  // T->R
                if (insn & 0x0020) j1_mem_write(cpu, t, n);    // N->[T]

                cpu->t = tp;
                break;
            }
        }
    }

    // Jump-to-self halts
    if (new_pc == pc) {
        cpu->halted = 1;
        return 1;
    }

    cpu->pc = new_pc;
    return 0;
}

void j1_print_state(void *context) {
    if (!context) return;
    J1CPU *cpu = (J1CPU*)context;

    printf("J1 Forth CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  T: 0x%04X  Halted: %s\n",
           cpu->pc, cpu->t, cpu->halted ? "Yes" : "No");
    printf("  Data stack (top 8, T first):");
    printf(" 0x%04X", cpu->t);
    for (int i = 0; i < 7; ++i) {
        printf(" 0x%04X", cpu->dstack[(cpu->dsp - i) & (J1_DSTACK - 1)]);
    }
    printf("\n");
    printf("  Return stack top (R): 0x%04X\n", cpu->rstack[cpu->rsp]);
    printf("  Depths: dsp=%d rsp=%d\n", cpu->dsp, cpu->rsp);
}

// Canonical J1 macro encodings (from the J1 paper's Forth base words)
typedef struct J1Macro {
    uint16_t code;
    const char *name;
} J1Macro;

static const J1Macro j1_macros[] = {
    { 0x6081, "DUP"    }, // T,     T->N, d+1
    { 0x6181, "OVER"   }, // N,     T->N, d+1
    { 0x6600, "INVERT" }, // ~T
    { 0x6203, "+"      }, // T+N,   d-1
    { 0x6180, "SWAP"   }, // N,     T->N
    { 0x6003, "NIP"    }, // T,     d-1
    { 0x6103, "DROP"   }, // N,     d-1
    { 0x700C, ";"      }, // T,     R->PC, r-1
    { 0x6147, ">R"     }, // N,     T->R, d-1, r+1
    { 0x6B8D, "R>"     }, // R,     T->N, d+1, r-1
    { 0x6B81, "R@"     }, // R,     T->N, d+1
    { 0x6C00, "@"      }, // [T]
    { 0x6123, "!"      }, // N,     N->[T], d-1
    { 0x6703, "="      }, // N==T,  d-1
    { 0x6803, "<"      }, // N<T,   d-1
    { 0x6F03, "U<"     }, // Nu<T,  d-1
    { 0x6303, "AND"    }, // T&N,   d-1
    { 0x6403, "OR"     }, // T|N,   d-1
    { 0x6503, "XOR"    }, // T^N,   d-1
    { 0x6A00, "1-"     }, // T-1
    { 0x6903, "RSHIFT" }, // N>>T,  d-1
    { 0x6D03, "LSHIFT" }, // N<<T,  d-1
    { 0x6E81, "DEPTH"  }, // depth, T->N, d+1
};

void j1_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    J1CPU *cpu = (J1CPU*)context;

    uint16_t pc = cpu->pc & J1_PC_MASK;
    uint16_t insn = cpu->memory[pc];
    uint16_t target = insn & J1_PC_MASK;

    if (insn & 0x8000) { // Literal
        snprintf(buf, buf_len, "LIT %u (0x%04X)", insn & 0x7FFF, insn & 0x7FFF);
        return;
    }

    switch ((insn >> 13) & 0x3) {
        case 0:
            snprintf(buf, buf_len, "JMP 0x%04X", target);
            return;
        case 1:
            snprintf(buf, buf_len, "0BRANCH 0x%04X", target);
            return;
        case 2:
            snprintf(buf, buf_len, "CALL 0x%04X", target);
            return;
        case 3: {
            // Canonical Forth word if the encoding matches a known macro
            for (size_t i = 0; i < sizeof(j1_macros) / sizeof(j1_macros[0]); ++i) {
                if (j1_macros[i].code == insn) {
                    snprintf(buf, buf_len, "%s", j1_macros[i].name);
                    return;
                }
            }
            // Otherwise show the raw ALU fields
            {
                int dd = stack_delta[insn & 0x3];
                int rd = stack_delta[(insn >> 2) & 0x3];
                snprintf(buf, buf_len, "ALU T'=%s%s%s%s%s d%+d r%+d",
                         alu_op_names[(insn >> 8) & 0xF],
                         (insn & 0x1000) ? " R->PC"  : "",
                         (insn & 0x0080) ? " T->N"   : "",
                         (insn & 0x0040) ? " T->R"   : "",
                         (insn & 0x0020) ? " N->[T]" : "",
                         dd, rd);
            }
            return;
        }
    }
}
