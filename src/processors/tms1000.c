#include "tms1000.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Texas Instruments TMS1000 4-bit microcontroller core.
//
// Simplifications vs. real hardware (the metal-programmable PLAs are abstracted):
//  - The program counter on the real chip steps through a 6-bit LFSR
//    (polynomial) sequence; here it increments linearly. ROM images must be
//    laid out in linear order for this core.
//  - The output PLA (which maps A + status latch to the 8 O lines) is modeled
//    as a plain 5-bit latch: TDO latches (SL << 4) | A directly.
//  - The instruction PLA is modeled as a fixed decode of the standard
//    documented 43-instruction TMS1000 set. Custom microprogrammed variants
//    (and the TMS1100-only opcodes such as A2AAC/A3AAC/A4AAC/A5AAC/A7AAC/
//    A9AAC of the full CKI-adder family) are not supported; the TMS1000 map
//    provides A6AAC, A8AAC and A10AAC.
//  - The K1/K2/K4/K8 input lines are unconnected and read as 0.

typedef struct TMS1000CPU {
    uint8_t a;        // 4-bit accumulator
    uint8_t x;        // 2-bit RAM file address
    uint8_t y;        // 4-bit RAM word address / R-output select
    uint8_t s;        // status (branch condition latch, defaults to 1)
    uint8_t sl;       // status latch (loaded only by YNEA, output via TDO)
    uint8_t pa;       // 4-bit page address register
    uint8_t pb;       // 4-bit page buffer register
    uint8_t pc;       // 6-bit program counter (word within page)
    uint8_t sr;       // 6-bit subroutine return register (one level)
    uint8_t cl;       // call latch
    uint8_t o;        // 5-bit O output latch (abstracted output PLA)
    uint16_t r;       // 11-bit R output latch (R0..R10)

    uint8_t rom[1024]; // 16 pages x 64 words of 8-bit instructions
    uint8_t ram[64];   // 64 x 4-bit RAM, addressed by X (file) and Y (word)

    uint32_t ticks;
} TMS1000CPU;

// The 4-bit constant field of an opcode appears bit-reversed on the CKI bus
// of the real chip; assemblers/listings show the logical value.
static uint8_t tms1000_rev4(uint8_t n) {
    return (uint8_t)(((n & 1) << 3) | ((n & 2) << 1) | ((n & 4) >> 1) | ((n & 8) >> 3));
}

void* tms1000_create(void) {
    TMS1000CPU *cpu = (TMS1000CPU*)calloc(1, sizeof(TMS1000CPU));
    if (cpu) {
        cpu->s = 1; // Status idles at 1 (branches default to taken)
        cpu->pa = 0x0F; // Hardware resets PA/PB to page 15
        cpu->pb = 0x0F;
    }
    return cpu;
}

void tms1000_destroy(void *context) {
    free(context);
}

int tms1000_init(void *context) {
    if (!context) return -1;
    TMS1000CPU *cpu = (TMS1000CPU*)context;

    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->s = 1;
    cpu->sl = 0;
    cpu->pa = 0x0F;
    cpu->pb = 0x0F;
    cpu->pc = 0;
    cpu->sr = 0;
    cpu->cl = 0;
    cpu->o = 0;
    cpu->r = 0;

    memset(cpu->rom, 0, sizeof(cpu->rom));
    memset(cpu->ram, 0, sizeof(cpu->ram));

    cpu->ticks = 0;
    return 0;
}

int tms1000_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    TMS1000CPU *cpu = (TMS1000CPU*)context;
    if (address >= sizeof(cpu->rom)) return -2;

    size_t copy_len = size;
    if (address + size > sizeof(cpu->rom)) {
        copy_len = sizeof(cpu->rom) - address;
    }
    memcpy(&cpu->rom[address], data, copy_len);

    // Programs conventionally start at page 15 word 0 after reset; if the
    // image was loaded at address 0, start execution at page 0 instead.
    if (address == 0) {
        cpu->pa = 0;
        cpu->pb = 0;
    }
    return 0;
}

int tms1000_step(void *context) {
    if (!context) return -1;
    TMS1000CPU *cpu = (TMS1000CPU*)context;

    uint16_t instr_addr = (uint16_t)(((cpu->pa & 0x0F) << 6) | (cpu->pc & 0x3F));
    uint8_t op = cpu->rom[instr_addr];

    uint8_t st = cpu->s;                   // status from previous cycle gates BR/CALL
    uint8_t new_s = 1;                     // status refreshes to 1 unless written below
    uint8_t c4 = tms1000_rev4(op & 0x0F);  // logical constant (bit-reversed field)
    uint8_t *m = &cpu->ram[((cpu->x & 3) << 4) | (cpu->y & 0x0F)];
    uint8_t sum;

    // Simplification: linear PC increment instead of the hardware LFSR sequence
    cpu->pc = (uint8_t)((cpu->pc + 1) & 0x3F);
    cpu->ticks++; // Every TMS1000 instruction takes one machine cycle

    if (op >= 0xC0) {
        // CALL w: conditional call within/into page (one-level subroutine)
        if (st) {
            uint8_t prev_pa = cpu->pa;
            if (!cpu->cl) {
                cpu->cl = 1;
                cpu->sr = cpu->pc;   // return word (PC already incremented)
                cpu->pa = cpu->pb;
            }
            cpu->pb = prev_pa;       // nested CALL: same-page call, no save
            cpu->pc = op & 0x3F;
        }
    }
    else if (op >= 0x80) {
        // BR w: conditional branch; PB transfers to PA unless inside a call
        if (st) {
            if (!cpu->cl) cpu->pa = cpu->pb;
            cpu->pc = op & 0x3F;
        }
    }
    else if (op >= 0x70) {
        // ALEC c: status = (A <= constant)
        new_s = (uint8_t)(cpu->a <= c4);
    }
    else if (op >= 0x60) {
        // TCMIY c: constant to memory, increment Y
        *m = c4;
        cpu->y = (uint8_t)((cpu->y + 1) & 0x0F);
    }
    else if (op >= 0x50) {
        // YNEC c: status = (Y != constant)
        new_s = (uint8_t)(cpu->y != c4);
    }
    else if (op >= 0x40) {
        // TCY c: constant to Y
        cpu->y = c4;
    }
    else if (op >= 0x3C) {
        // LDX c: load X (2-bit field, bit-reversed like the constants)
        cpu->x = (uint8_t)(c4 >> 2);
    }
    else if (op >= 0x38) {
        // TBIT1 b: status = memory bit b
        new_s = (uint8_t)((*m >> (c4 >> 2)) & 1);
    }
    else if (op >= 0x34) {
        // RBIT b: reset memory bit b
        *m = (uint8_t)(*m & ~(1 << (c4 >> 2)) & 0x0F);
    }
    else if (op >= 0x30) {
        // SBIT b: set memory bit b
        *m = (uint8_t)((*m | (1 << (c4 >> 2))) & 0x0F);
    }
    else if (op >= 0x20) {
        switch (op) {
            case 0x20: // TAMIY: A to memory, increment Y
                *m = cpu->a;
                cpu->y = (uint8_t)((cpu->y + 1) & 0x0F);
                break;
            case 0x21: // TMA: memory to A
                cpu->a = *m;
                break;
            case 0x22: // TMY: memory to Y
                cpu->y = *m;
                break;
            case 0x23: // TYA: Y to A
                cpu->a = cpu->y;
                break;
            case 0x24: // TAY: A to Y
                cpu->y = cpu->a;
                break;
            case 0x25: // AMAAC: add memory to A, status = carry
                sum = (uint8_t)(cpu->a + *m);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x26: // MNEZ: status = (memory != 0)
                new_s = (uint8_t)(*m != 0);
                break;
            case 0x27: // SAMAN: A = memory - A, status = no borrow
                sum = (uint8_t)(*m + (~cpu->a & 0x0F) + 1);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x28: // IMAC: A = memory + 1, status = carry
                sum = (uint8_t)(*m + 1);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x29: // ALEM: status = (A <= memory)
                new_s = (uint8_t)(cpu->a <= *m);
                break;
            case 0x2A: // DMAN: A = memory - 1, status = no borrow
                sum = (uint8_t)(*m + 15);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x2B: // IYC: increment Y, status = carry
                sum = (uint8_t)(cpu->y + 1);
                cpu->y = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x2C: // DYN: decrement Y, status = no borrow
                sum = (uint8_t)(cpu->y + 15);
                cpu->y = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x2D: // CPAIZ: A = -A (two's complement), status set if A was 0
                sum = (uint8_t)((~cpu->a & 0x0F) + 1);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x2E: // XMA: exchange memory and A
                sum = cpu->a;
                cpu->a = *m;
                *m = sum;
                break;
            case 0x2F: // CLA: clear A
                cpu->a = 0;
                break;
            default:
                break;
        }
    }
    else if (op >= 0x10) {
        // LDP c: load page buffer with constant
        cpu->pb = c4;
    }
    else {
        switch (op) {
            case 0x00: // COMX: complement X
                cpu->x = (uint8_t)(cpu->x ^ 3);
                break;
            case 0x01: // A8AAC: A = A + 8, status = carry
                sum = (uint8_t)(cpu->a + 8);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x02: // YNEA: status = (Y != A), also loads the status latch
                new_s = (uint8_t)(cpu->y != cpu->a);
                cpu->sl = new_s;
                break;
            case 0x03: // TAM: A to memory
                *m = cpu->a;
                break;
            case 0x04: // TAMZA: A to memory, zero A
                *m = cpu->a;
                cpu->a = 0;
                break;
            case 0x05: // A10AAC: A = A + 10, status = carry
                sum = (uint8_t)(cpu->a + 10);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x06: // A6AAC: A = A + 6, status = carry
                sum = (uint8_t)(cpu->a + 6);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x07: // DAN: decrement A, status = no borrow
                sum = (uint8_t)(cpu->a + 15);
                cpu->a = (uint8_t)(sum & 0x0F);
                new_s = (uint8_t)(sum >> 4);
                break;
            case 0x08: // TKA: K inputs to A (K lines are unconnected, read 0)
                cpu->a = 0;
                break;
            case 0x09: // KNEZ: status = (K != 0); K reads 0, so status = 0
                new_s = 0;
                break;
            case 0x0A: // TDO: A and status latch to O outputs (PLA abstracted)
                cpu->o = (uint8_t)(((cpu->sl & 1) << 4) | (cpu->a & 0x0F));
                break;
            case 0x0B: // CLO: clear O outputs
                cpu->o = 0;
                break;
            case 0x0C: // RSTR: reset R output addressed by Y (R0..R10)
                if (cpu->y < 11) cpu->r = (uint16_t)(cpu->r & ~(1u << cpu->y));
                break;
            case 0x0D: // SETR: set R output addressed by Y (R0..R10)
                if (cpu->y < 11) cpu->r = (uint16_t)(cpu->r | (1u << cpu->y));
                break;
            case 0x0E: // IA: increment A (no status effect)
                cpu->a = (uint8_t)((cpu->a + 1) & 0x0F);
                break;
            case 0x0F: // RETN: return; PB to PA always, PC from SR if in a call
                if (cpu->cl) {
                    cpu->cl = 0;
                    cpu->pc = cpu->sr;
                }
                cpu->pa = cpu->pb;
                break;
            default:
                break;
        }
    }

    cpu->s = new_s;

    // BR-to-self (taken branch back to the same word/page) is an idle loop
    if ((uint16_t)(((cpu->pa & 0x0F) << 6) | (cpu->pc & 0x3F)) == instr_addr) {
        return 1; // Treat as HALT
    }

    return 0;
}

void tms1000_print_state(void *context) {
    if (!context) return;
    TMS1000CPU *cpu = (TMS1000CPU*)context;

    printf("TI TMS1000 State:\n");
    printf("  A:  0x%X    X: %d    Y: 0x%X    S: %d    SL: %d    Ticks: %u\n",
           cpu->a, cpu->x, cpu->y, cpu->s, cpu->sl, cpu->ticks);
    printf("  PA: 0x%X   PB: 0x%X   PC: 0x%02X   SR: 0x%02X   CL: %d   (ROM addr 0x%03X)\n",
           cpu->pa, cpu->pb, cpu->pc, cpu->sr, cpu->cl,
           ((cpu->pa & 0x0F) << 6) | (cpu->pc & 0x3F));
    printf("  O:  0x%02X   R: ", cpu->o);
    for (int i = 10; i >= 0; --i) {
        printf("%d", (cpu->r >> i) & 1);
    }
    printf(" (R10..R0)\n");
}

void tms1000_get_disassembly(void *context, char *buf, size_t buf_len) {
    static const char *fixed0[16] = {
        "COMX", "A8AAC", "YNEA", "TAM", "TAMZA", "A10AAC", "A6AAC", "DAN",
        "TKA", "KNEZ", "TDO", "CLO", "RSTR", "SETR", "IA", "RETN"
    };
    static const char *fixed2[16] = {
        "TAMIY", "TMA", "TMY", "TYA", "TAY", "AMAAC", "MNEZ", "SAMAN",
        "IMAC", "ALEM", "DMAN", "IYC", "DYN", "CPAIZ", "XMA", "CLA"
    };

    if (!context || !buf || buf_len == 0) return;
    TMS1000CPU *cpu = (TMS1000CPU*)context;

    uint16_t addr = (uint16_t)(((cpu->pa & 0x0F) << 6) | (cpu->pc & 0x3F));
    uint8_t op = cpu->rom[addr];
    uint8_t c4 = tms1000_rev4(op & 0x0F);

    if (op >= 0xC0) {
        snprintf(buf, buf_len, "CALL 0x%02X", op & 0x3F);
    } else if (op >= 0x80) {
        snprintf(buf, buf_len, "BR   0x%02X", op & 0x3F);
    } else if (op >= 0x70) {
        snprintf(buf, buf_len, "ALEC 0x%X", c4);
    } else if (op >= 0x60) {
        snprintf(buf, buf_len, "TCMIY 0x%X", c4);
    } else if (op >= 0x50) {
        snprintf(buf, buf_len, "YNEC 0x%X", c4);
    } else if (op >= 0x40) {
        snprintf(buf, buf_len, "TCY  0x%X", c4);
    } else if (op >= 0x3C) {
        snprintf(buf, buf_len, "LDX  %d", c4 >> 2);
    } else if (op >= 0x38) {
        snprintf(buf, buf_len, "TBIT1 %d", c4 >> 2);
    } else if (op >= 0x34) {
        snprintf(buf, buf_len, "RBIT %d", c4 >> 2);
    } else if (op >= 0x30) {
        snprintf(buf, buf_len, "SBIT %d", c4 >> 2);
    } else if (op >= 0x20) {
        snprintf(buf, buf_len, "%s", fixed2[op & 0x0F]);
    } else if (op >= 0x10) {
        snprintf(buf, buf_len, "LDP  0x%X", c4);
    } else {
        snprintf(buf, buf_len, "%s", fixed0[op & 0x0F]);
    }
}
