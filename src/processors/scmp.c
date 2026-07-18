#include "scmp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB (16-bit address space)

// Status register (SR) bit positions
#define SR_CY 0x80 // Carry / Link
#define SR_OV 0x40 // Overflow
#define SR_SB 0x20 // Sense B input (read-only via CAS)
#define SR_SA 0x10 // Sense A input (read-only via CAS)
#define SR_IE 0x08 // Interrupt Enable
#define SR_F2 0x04 // User flag 2 output
#define SR_F1 0x02 // User flag 1 output
#define SR_F0 0x01 // User flag 0 output

typedef struct ScmpCPU {
    uint8_t ac;      // Accumulator
    uint8_t e;       // Extension register
    uint8_t sr;      // Status register (CY/L, OV, SB, SA, IE, F2, F1, F0)
    uint16_t p[4];   // Pointer registers P0-P3 (P0 is the program counter)

    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} ScmpCPU;

// SC/MP quirk: the program counter (P0) is incremented BEFORE each fetch,
// and all address arithmetic (increment, displacement, auto-indexing) wraps
// within a 4 KB page: only the low 12 bits participate in the addition while
// the high 4 bits (the page number) stay fixed. So P0 normally holds the
// address of the LAST byte fetched, and a jump loads P0 with (target - 1).
static inline uint16_t addr_add12(uint16_t base, int16_t delta) {
    return (uint16_t)((base & 0xF000u) | ((base + delta) & 0x0FFFu));
}

// Resolve a displacement byte: 0x80 (-128) means "use register E" instead.
static inline int16_t eff_disp(ScmpCPU *cpu, uint8_t disp) {
    if (disp == 0x80) return (int16_t)(int8_t)cpu->e;
    return (int16_t)(int8_t)disp;
}

// Binary add: AC-style, updates CY and OV in SR, returns 8-bit result.
static uint8_t scmp_binadd(ScmpCPU *cpu, uint8_t a, uint8_t b) {
    uint16_t sum = (uint16_t)a + (uint16_t)b + ((cpu->sr & SR_CY) ? 1 : 0);
    uint8_t res = sum & 0xFF;
    cpu->sr &= (uint8_t)~(SR_CY | SR_OV);
    if (sum > 0xFF) cpu->sr |= SR_CY;
    if ((a ^ res) & (b ^ res) & 0x80) cpu->sr |= SR_OV;
    return res;
}

// Decimal (BCD) add: updates CY, leaves OV untouched, returns 8-bit result.
static uint8_t scmp_bcdadd(ScmpCPU *cpu, uint8_t a, uint8_t b) {
    uint16_t lo = (uint16_t)(a & 0x0F) + (uint16_t)(b & 0x0F) + ((cpu->sr & SR_CY) ? 1 : 0);
    uint16_t hi = (uint16_t)(a >> 4) + (uint16_t)(b >> 4);
    if (lo > 9) { lo -= 10; hi++; }
    cpu->sr &= (uint8_t)~SR_CY;
    if (hi > 9) { hi -= 10; cpu->sr |= SR_CY; }
    return (uint8_t)(((hi & 0x0F) << 4) | (lo & 0x0F));
}

void* scmp_create(void) {
    ScmpCPU *cpu = (ScmpCPU*)calloc(1, sizeof(ScmpCPU));
    return cpu;
}

void scmp_destroy(void *context) {
    free(context);
}

int scmp_init(void *context) {
    if (!context) return -1;
    ScmpCPU *cpu = (ScmpCPU*)context;

    cpu->ac = 0;
    cpu->e = 0;
    cpu->sr = 0;
    memset(cpu->p, 0, sizeof(cpu->p));
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int scmp_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    ScmpCPU *cpu = (ScmpCPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int scmp_step(void *context) {
    if (!context) return -1;
    ScmpCPU *cpu = (ScmpCPU*)context;

    if (cpu->halted) return 1;

    // PC increments before fetch (with 4 KB page wrap, see addr_add12).
    cpu->p[0] = addr_add12(cpu->p[0], 1);
    uint8_t op = cpu->memory[cpu->p[0]];
    cpu->ticks++;

    // --- Single-byte instructions ---
    switch (op) {
        case 0x00: // HALT
            cpu->halted = 1;
            return 1;
        case 0x01: { // XAE - exchange AC and E
            uint8_t t = cpu->ac; cpu->ac = cpu->e; cpu->e = t;
            return 0;
        }
        case 0x02: // CCL - clear carry/link
            cpu->sr &= (uint8_t)~SR_CY;
            return 0;
        case 0x03: // SCL - set carry/link
            cpu->sr |= SR_CY;
            return 0;
        case 0x04: // DINT - disable interrupts
            cpu->sr &= (uint8_t)~SR_IE;
            return 0;
        case 0x05: // IEN - enable interrupts
            cpu->sr |= SR_IE;
            return 0;
        case 0x06: // CSA - copy status to AC
            cpu->ac = cpu->sr;
            return 0;
        case 0x07: // CAS - copy AC to status (SA/SB are sense inputs, unchanged)
            cpu->sr = (uint8_t)((cpu->ac & (uint8_t)~(SR_SA | SR_SB)) | (cpu->sr & (SR_SA | SR_SB)));
            return 0;
        case 0x08: // NOP
            return 0;
        case 0x19: // SIO - serial I/O: E shifts right, SIN (0 here) enters bit 7
            cpu->e >>= 1;
            return 0;
        case 0x1C: // SR - shift right, 0 into bit 7
            cpu->ac >>= 1;
            return 0;
        case 0x1D: { // SRL - shift right, CY/L into bit 7
            uint8_t cy = (cpu->sr & SR_CY) ? 0x80 : 0x00;
            cpu->ac = (cpu->ac >> 1) | cy;
            return 0;
        }
        case 0x1E: // RR - rotate right
            cpu->ac = (uint8_t)((cpu->ac >> 1) | (cpu->ac << 7));
            return 0;
        case 0x1F: { // RRL - rotate right through CY/L
            uint8_t cy = (cpu->sr & SR_CY) ? 0x80 : 0x00;
            uint8_t lsb = cpu->ac & 0x01;
            cpu->ac = (cpu->ac >> 1) | cy;
            cpu->sr &= (uint8_t)~SR_CY;
            if (lsb) cpu->sr |= SR_CY;
            return 0;
        }
        case 0x40: // LDE
            cpu->ac = cpu->e;
            return 0;
        case 0x50: // ANE
            cpu->ac &= cpu->e;
            return 0;
        case 0x58: // ORE
            cpu->ac |= cpu->e;
            return 0;
        case 0x60: // XRE
            cpu->ac ^= cpu->e;
            return 0;
        case 0x68: // DAE - decimal add extension
            cpu->ac = scmp_bcdadd(cpu, cpu->ac, cpu->e);
            return 0;
        case 0x70: // ADE - binary add extension
            cpu->ac = scmp_binadd(cpu, cpu->ac, cpu->e);
            return 0;
        case 0x78: // CAE - complement and add extension (subtract)
            cpu->ac = scmp_binadd(cpu, cpu->ac, (uint8_t)~cpu->e);
            return 0;
        default:
            break;
    }

    // XPAL/XPAH/XPPC - pointer register exchanges (0x30-0x3F)
    if ((op & 0xF0) == 0x30) {
        uint8_t ptr = op & 0x03;
        if ((op & 0x0C) == 0x00) { // XPAL - exchange AC with low byte of Pn
            uint8_t t = cpu->ac;
            cpu->ac = cpu->p[ptr] & 0xFF;
            cpu->p[ptr] = (cpu->p[ptr] & 0xFF00) | t;
            return 0;
        }
        if ((op & 0x0C) == 0x04) { // XPAH - exchange AC with high byte of Pn
            uint8_t t = cpu->ac;
            cpu->ac = (cpu->p[ptr] >> 8) & 0xFF;
            cpu->p[ptr] = (uint16_t)((cpu->p[ptr] & 0x00FF) | ((uint16_t)t << 8));
            return 0;
        }
        if ((op & 0x0C) == 0x0C) { // XPPC - exchange PC (P0) with Pn
            uint16_t t = cpu->p[0];
            cpu->p[0] = cpu->p[ptr];
            cpu->p[ptr] = t;
            return 0;
        }
        return 0; // 0x38-0x3B undefined - treat as NOP
    }

    // --- Two-byte instructions (fetch displacement/immediate byte) ---
    cpu->p[0] = addr_add12(cpu->p[0], 1);
    uint8_t disp = cpu->memory[cpu->p[0]];

    if (op == 0x8F) { // DLY - delay: 13 + 2*AC + 514*disp microcycles, AC = 0xFF
        cpu->ticks += 13 + 2u * cpu->ac + 514u * disp;
        cpu->ac = 0xFF;
        return 0;
    }

    if ((op & 0xF0) == 0x90) { // JMP/JP/JZ/JNZ disp(Pn)
        uint8_t ptr = op & 0x03;
        uint8_t cond = (op >> 2) & 0x03;
        int taken = 0;
        switch (cond) {
            case 0: taken = 1; break;                      // JMP
            case 1: taken = !(cpu->ac & 0x80); break;      // JP  (AC >= 0)
            case 2: taken = (cpu->ac == 0); break;         // JZ
            case 3: taken = (cpu->ac != 0); break;         // JNZ
        }
        if (taken) {
            // P0 already points at the displacement byte; the base for a jump
            // via P0 is that address. Execution resumes at target+1 because of
            // the increment-before-fetch quirk (assemblers account for this).
            cpu->p[0] = addr_add12(cpu->p[ptr], eff_disp(cpu, disp));
        }
        return 0;
    }

    if ((op & 0xF4) == 0xA0 || (op & 0xF4) == 0xB0) {
        // ILD (0xA8-0xAB) / DLD (0xB8-0xBB) - increment/decrement memory, load AC
        if ((op & 0x08) == 0x08) {
            uint8_t ptr = op & 0x03;
            uint16_t ea = addr_add12(cpu->p[ptr], eff_disp(cpu, disp));
            uint8_t val = cpu->memory[ea];
            val = (op & 0x10) ? (uint8_t)(val - 1) : (uint8_t)(val + 1);
            cpu->memory[ea] = val;
            cpu->ac = val;
        }
        return 0;
    }

    if (op >= 0xC0) {
        // Memory reference group: LD/ST/AND/OR/XOR/DAD/ADD/CAD
        // Bits 5-3 select the operation, bit 2 selects auto-indexed (ptr!=0)
        // or immediate (ptr==0), bits 1-0 select the pointer register.
        uint8_t alu = (op >> 3) & 0x07;
        uint8_t ptr = op & 0x03;
        uint8_t operand = 0;
        uint16_t ea = 0;
        int is_store = (alu == 1);
        int is_imm = 0;

        if (op & 0x04) {
            if (ptr == 0) { // Immediate: operand is the literal byte
                if (is_store) return 0; // 0xCC has no immediate form - NOP
                is_imm = 1;
                operand = disp;
            } else { // Auto-indexed: negative disp pre-decrements, else post-increments
                int16_t d = eff_disp(cpu, disp);
                if (d < 0) {
                    cpu->p[ptr] = addr_add12(cpu->p[ptr], d);
                    ea = cpu->p[ptr];
                } else {
                    ea = cpu->p[ptr];
                    cpu->p[ptr] = addr_add12(cpu->p[ptr], d);
                }
            }
        } else { // PC-relative (ptr=0, base = address of disp byte) or indexed
            ea = addr_add12(cpu->p[ptr], eff_disp(cpu, disp));
        }

        if (!is_imm && !is_store) operand = cpu->memory[ea];

        switch (alu) {
            case 0: cpu->ac = operand; break;                              // LD/LDI
            case 1: cpu->memory[ea] = cpu->ac; break;                      // ST
            case 2: cpu->ac &= operand; break;                             // AND/ANI
            case 3: cpu->ac |= operand; break;                             // OR/ORI
            case 4: cpu->ac ^= operand; break;                             // XOR/XRI
            case 5: cpu->ac = scmp_bcdadd(cpu, cpu->ac, operand); break;   // DAD/DAI
            case 6: cpu->ac = scmp_binadd(cpu, cpu->ac, operand); break;   // ADD/ADI
            case 7: cpu->ac = scmp_binadd(cpu, cpu->ac, (uint8_t)~operand); break; // CAD/CAI
        }
        return 0;
    }

    // Undefined two-byte-range opcode: the displacement byte was consumed;
    // treat as NOP.
    return 0;
}

void scmp_print_state(void *context) {
    if (!context) return;
    ScmpCPU *cpu = (ScmpCPU*)context;

    printf("SC/MP (INS8060) CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  AC: 0x%02X  E: 0x%02X  Halted: %s\n", cpu->ac, cpu->e, cpu->halted ? "Yes" : "No");
    printf("  SR: 0x%02X  [CY/L=%d OV=%d SB=%d SA=%d IE=%d F2=%d F1=%d F0=%d]\n",
           cpu->sr,
           (cpu->sr & SR_CY) ? 1 : 0, (cpu->sr & SR_OV) ? 1 : 0,
           (cpu->sr & SR_SB) ? 1 : 0, (cpu->sr & SR_SA) ? 1 : 0,
           (cpu->sr & SR_IE) ? 1 : 0, (cpu->sr & SR_F2) ? 1 : 0,
           (cpu->sr & SR_F1) ? 1 : 0, (cpu->sr & SR_F0) ? 1 : 0);
    printf("  P0(PC): 0x%04X  P1: 0x%04X  P2: 0x%04X  P3: 0x%04X\n",
           cpu->p[0], cpu->p[1], cpu->p[2], cpu->p[3]);
}

void scmp_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    ScmpCPU *cpu = (ScmpCPU*)context;

    // The next instruction lives at P0+1 (increment-before-fetch quirk).
    uint16_t addr = addr_add12(cpu->p[0], 1);
    uint8_t op = cpu->memory[addr];
    uint8_t disp = cpu->memory[addr_add12(addr, 1)];

    switch (op) {
        case 0x00: snprintf(buf, buf_len, "HALT"); return;
        case 0x01: snprintf(buf, buf_len, "XAE");  return;
        case 0x02: snprintf(buf, buf_len, "CCL");  return;
        case 0x03: snprintf(buf, buf_len, "SCL");  return;
        case 0x04: snprintf(buf, buf_len, "DINT"); return;
        case 0x05: snprintf(buf, buf_len, "IEN");  return;
        case 0x06: snprintf(buf, buf_len, "CSA");  return;
        case 0x07: snprintf(buf, buf_len, "CAS");  return;
        case 0x08: snprintf(buf, buf_len, "NOP");  return;
        case 0x19: snprintf(buf, buf_len, "SIO");  return;
        case 0x1C: snprintf(buf, buf_len, "SR");   return;
        case 0x1D: snprintf(buf, buf_len, "SRL");  return;
        case 0x1E: snprintf(buf, buf_len, "RR");   return;
        case 0x1F: snprintf(buf, buf_len, "RRL");  return;
        case 0x40: snprintf(buf, buf_len, "LDE");  return;
        case 0x50: snprintf(buf, buf_len, "ANE");  return;
        case 0x58: snprintf(buf, buf_len, "ORE");  return;
        case 0x60: snprintf(buf, buf_len, "XRE");  return;
        case 0x68: snprintf(buf, buf_len, "DAE");  return;
        case 0x70: snprintf(buf, buf_len, "ADE");  return;
        case 0x78: snprintf(buf, buf_len, "CAE");  return;
        case 0x8F: snprintf(buf, buf_len, "DLY   0x%02X", disp); return;
        default: break;
    }

    if ((op & 0xF0) == 0x30) {
        uint8_t ptr = op & 0x03;
        if ((op & 0x0C) == 0x00) { snprintf(buf, buf_len, "XPAL  P%d", ptr); return; }
        if ((op & 0x0C) == 0x04) { snprintf(buf, buf_len, "XPAH  P%d", ptr); return; }
        if ((op & 0x0C) == 0x0C) { snprintf(buf, buf_len, "XPPC  P%d", ptr); return; }
        snprintf(buf, buf_len, "INV   0x%02X", op);
        return;
    }

    if ((op & 0xF0) == 0x90) {
        static const char* j_names[] = { "JMP", "JP", "JZ", "JNZ" };
        const char* jn = j_names[(op >> 2) & 0x03];
        if (disp == 0x80) snprintf(buf, buf_len, "%-5s E(P%d)", jn, op & 0x03);
        else snprintf(buf, buf_len, "%-5s %+d(P%d)", jn, (int8_t)disp, op & 0x03);
        return;
    }

    if ((op & 0xFC) == 0xA8 || (op & 0xFC) == 0xB8) {
        const char* mn = (op & 0x10) ? "DLD" : "ILD";
        if (disp == 0x80) snprintf(buf, buf_len, "%-5s E(P%d)", mn, op & 0x03);
        else snprintf(buf, buf_len, "%-5s %+d(P%d)", mn, (int8_t)disp, op & 0x03);
        return;
    }

    if (op >= 0xC0) {
        static const char* m_names[] = { "LD", "ST", "AND", "OR", "XOR", "DAD", "ADD", "CAD" };
        static const char* i_names[] = { "LDI", "-", "ANI", "ORI", "XRI", "DAI", "ADI", "CAI" };
        uint8_t alu = (op >> 3) & 0x07;
        uint8_t ptr = op & 0x03;
        if (op & 0x04) {
            if (ptr == 0) { // Immediate
                if (alu == 1) snprintf(buf, buf_len, "INV   0x%02X", op);
                else snprintf(buf, buf_len, "%-5s 0x%02X", i_names[alu], disp);
            } else { // Auto-indexed
                if (disp == 0x80) snprintf(buf, buf_len, "%-5s @E(P%d)", m_names[alu], ptr);
                else snprintf(buf, buf_len, "%-5s @%+d(P%d)", m_names[alu], (int8_t)disp, ptr);
            }
        } else { // PC-relative / indexed
            if (disp == 0x80) snprintf(buf, buf_len, "%-5s E(P%d)", m_names[alu], ptr);
            else snprintf(buf, buf_len, "%-5s %+d(P%d)", m_names[alu], (int8_t)disp, ptr);
        }
        return;
    }

    snprintf(buf, buf_len, "INV   0x%02X", op);
}
