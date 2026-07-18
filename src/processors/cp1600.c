#include "cp1600.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CP1600_MEM_WORDS 65536
#define CP1600_PC_START  0x1000

// General Instrument CP1600 (Intellivision CPU).
// 64K x 16-bit word memory, 10-bit opcodes ("decles").
// R0-R7 are 16-bit registers; R7 is the program counter, R6 the stack
// pointer. R4/R5 (and R7) auto-increment when used as data counters,
// R1-R3 do not. R6 pushes (write, post-increment) and pops
// (pre-decrement, read).
typedef struct CP1600CPU {
    uint16_t r[8];              // R7 = PC
    uint16_t memory[CP1600_MEM_WORDS];
    uint8_t flag_s;             // Sign
    uint8_t flag_z;             // Zero
    uint8_t flag_o;             // Overflow
    uint8_t flag_c;             // Carry
    uint8_t intr_enabled;       // Interrupt enable (EIS/DIS)
    uint8_t sdbd;               // SDBD prefix pending for next instruction
    uint32_t ticks;
    int halted;
} CP1600CPU;

void* cp1600_create(void) {
    CP1600CPU *cpu = (CP1600CPU*)calloc(1, sizeof(CP1600CPU));
    return cpu;
}

void cp1600_destroy(void *context) {
    free(context);
}

int cp1600_init(void *context) {
    if (!context) return -1;
    CP1600CPU *cpu = (CP1600CPU*)context;

    memset(cpu->r, 0, sizeof(cpu->r));
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->r[7] = CP1600_PC_START;
    cpu->flag_s = cpu->flag_z = cpu->flag_o = cpu->flag_c = 0;
    cpu->intr_enabled = 0;
    cpu->sdbd = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int cp1600_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    CP1600CPU *cpu = (CP1600CPU*)context;

    if (address == 0) address = CP1600_PC_START;
    if (address >= CP1600_MEM_WORDS) return -2;

    size_t words = size / 2;
    size_t i;
    for (i = 0; i < words && address + i < CP1600_MEM_WORDS; ++i) {
        // Data stream is big-endian 16-bit words; address is a word address
        cpu->memory[address + i] = (uint16_t)((data[i * 2] << 8) | data[i * 2 + 1]);
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void set_sz(CP1600CPU *cpu, uint16_t val) {
    cpu->flag_s = (uint8_t)((val >> 15) & 1);
    cpu->flag_z = (uint8_t)(val == 0);
}

// Sign flag for right shifts and SWAP comes from bit 7 of the result
// (a documented CP1600 quirk).
static void set_sz_bit7(CP1600CPU *cpu, uint16_t val) {
    cpu->flag_s = (uint8_t)((val >> 7) & 1);
    cpu->flag_z = (uint8_t)(val == 0);
}

static uint16_t do_add(CP1600CPU *cpu, uint16_t a, uint16_t b) {
    uint32_t sum = (uint32_t)a + (uint32_t)b;
    uint16_t res = (uint16_t)sum;
    cpu->flag_c = (uint8_t)((sum >> 16) & 1);
    cpu->flag_o = (uint8_t)((((uint16_t)~(a ^ b) & (uint16_t)(a ^ res)) >> 15) & 1);
    set_sz(cpu, res);
    return res;
}

// a - b; carry flag is "no borrow" (set when a >= b unsigned)
static uint16_t do_sub(CP1600CPU *cpu, uint16_t a, uint16_t b) {
    uint16_t res = (uint16_t)(a - b);
    cpu->flag_c = (uint8_t)(a >= b);
    cpu->flag_o = (uint8_t)((((uint16_t)(a ^ b) & (uint16_t)(a ^ res)) >> 15) & 1);
    set_sz(cpu, res);
    return res;
}

static uint16_t fetch(CP1600CPU *cpu) {
    uint16_t w = cpu->memory[cpu->r[7]];
    cpu->r[7] = (uint16_t)(cpu->r[7] + 1);
    return w;
}

// Read via data counter register `reg` (1-7). R4/R5/R7 post-increment,
// R6 pops (pre-decrement), R1-R3 leave the register unchanged.
// With SDBD, two 8-bit reads form a 16-bit value (low byte first);
// non-incrementing registers read the same location twice.
static uint16_t read_indirect(CP1600CPU *cpu, int reg, int sdbd) {
    uint16_t lo, hi;
    if (!sdbd) {
        if (reg == 6) {
            cpu->r[6] = (uint16_t)(cpu->r[6] - 1);
            return cpu->memory[cpu->r[6]];
        }
        lo = cpu->memory[cpu->r[reg]];
        if (reg >= 4) cpu->r[reg] = (uint16_t)(cpu->r[reg] + 1);
        return lo;
    }
    if (reg == 6) {
        cpu->r[6] = (uint16_t)(cpu->r[6] - 1);
        lo = (uint16_t)(cpu->memory[cpu->r[6]] & 0xFF);
        cpu->r[6] = (uint16_t)(cpu->r[6] - 1);
        hi = (uint16_t)(cpu->memory[cpu->r[6]] & 0xFF);
    } else {
        lo = (uint16_t)(cpu->memory[cpu->r[reg]] & 0xFF);
        if (reg >= 4) cpu->r[reg] = (uint16_t)(cpu->r[reg] + 1);
        hi = (uint16_t)(cpu->memory[cpu->r[reg]] & 0xFF);
        if (reg >= 4) cpu->r[reg] = (uint16_t)(cpu->r[reg] + 1);
    }
    return (uint16_t)(lo | (hi << 8));
}

// Write via data counter register `reg` (1-7). R4-R7 post-increment
// (R6 = push, R7 = MVOI into the instruction stream).
static void write_indirect(CP1600CPU *cpu, int reg, uint16_t val) {
    cpu->memory[cpu->r[reg]] = val;
    if (reg >= 4) cpu->r[reg] = (uint16_t)(cpu->r[reg] + 1);
}

// Evaluate branch condition for opcodes 0x200-0x23F.
// Bit 4 marks external branches (BEXT): modeled as never taken.
static int branch_taken(CP1600CPU *cpu, uint16_t op) {
    int t;
    if (op & 0x10) return 0; // BEXT: external condition, always false here
    switch (op & 7) {
        case 0: t = 1; break;                                    // B
        case 1: t = cpu->flag_c; break;                          // BC
        case 2: t = cpu->flag_o; break;                          // BOV
        case 3: t = !cpu->flag_s; break;                         // BPL
        case 4: t = cpu->flag_z; break;                          // BEQ
        case 5: t = cpu->flag_s ^ cpu->flag_o; break;            // BLT
        case 6: t = cpu->flag_z | (cpu->flag_s ^ cpu->flag_o); break; // BLE
        default: t = cpu->flag_c ^ cpu->flag_s; break;           // BUSC
    }
    if (op & 8) t = !t;
    return t != 0;
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

int cp1600_step(void *context) {
    if (!context) return -1;
    CP1600CPU *cpu = (CP1600CPU*)context;

    if (cpu->halted) return 1;

    uint16_t base = cpu->r[7];       // Address of this instruction
    uint16_t op = (uint16_t)(fetch(cpu) & 0x3FF);
    int sdbd = cpu->sdbd;
    cpu->sdbd = 0;
    cpu->ticks++;

    if (op < 0x040) {
        switch (op) {
            case 0x000: // HLT
                cpu->halted = 1;
                return 1;
            case 0x001: // SDBD
                cpu->sdbd = 1;
                break;
            case 0x002: // EIS
                cpu->intr_enabled = 1;
                break;
            case 0x003: // DIS
                cpu->intr_enabled = 0;
                break;
            case 0x004: { // J / JE / JD / JSR / JSRE / JSRD (3 words)
                uint16_t w1 = fetch(cpu);
                uint16_t w2 = fetch(cpu);
                int rr = (w1 >> 8) & 3;
                int ee = w1 & 3;
                uint16_t target = (uint16_t)((((w1 >> 2) & 0x3F) << 10) | (w2 & 0x3FF));
                if (ee == 1) cpu->intr_enabled = 1;
                else if (ee == 2) cpu->intr_enabled = 0;
                if (rr != 3) cpu->r[4 + rr] = cpu->r[7]; // JSR: save return address
                cpu->r[7] = target;
                break;
            }
            case 0x005: // TCI (terminate current interrupt): no-op here
                break;
            case 0x006: // CLRC
                cpu->flag_c = 0;
                break;
            case 0x007: // SETC
                cpu->flag_c = 1;
                break;
            default: {
                int reg = op & 7;
                if (op < 0x010) {          // INCR
                    cpu->r[reg] = (uint16_t)(cpu->r[reg] + 1);
                    set_sz(cpu, cpu->r[reg]);
                } else if (op < 0x018) {   // DECR
                    cpu->r[reg] = (uint16_t)(cpu->r[reg] - 1);
                    set_sz(cpu, cpu->r[reg]);
                } else if (op < 0x020) {   // COMR
                    cpu->r[reg] = (uint16_t)(~cpu->r[reg]);
                    set_sz(cpu, cpu->r[reg]);
                } else if (op < 0x028) {   // NEGR
                    cpu->r[reg] = do_sub(cpu, 0, cpu->r[reg]);
                } else if (op < 0x030) {   // ADCR
                    cpu->r[reg] = do_add(cpu, cpu->r[reg], (uint16_t)cpu->flag_c);
                } else if (op < 0x034) {   // GSWD R0-R3
                    uint16_t sw = (uint16_t)((cpu->flag_s << 7) | (cpu->flag_z << 6) |
                                             (cpu->flag_o << 5) | (cpu->flag_c << 4));
                    cpu->r[op & 3] = (uint16_t)(sw | (sw << 8));
                } else if (op < 0x036) {   // NOP
                    // nothing
                } else if (op < 0x038) {   // SIN (software interrupt): no-op here
                    // nothing
                } else {                   // RSWD
                    uint16_t v = cpu->r[reg];
                    cpu->flag_s = (uint8_t)((v >> 7) & 1);
                    cpu->flag_z = (uint8_t)((v >> 6) & 1);
                    cpu->flag_o = (uint8_t)((v >> 5) & 1);
                    cpu->flag_c = (uint8_t)((v >> 4) & 1);
                }
                break;
            }
        }
        return 0;
    }

    if (op < 0x080) { // Shift/rotate group (R0-R3 only, 1 or 2 positions)
        int kind = (op >> 3) & 7;
        int two = (op >> 2) & 1;
        int reg = op & 3;
        uint16_t v = cpu->r[reg];
        uint16_t res = v;
        switch (kind) {
            case 0: // SWAP (double form replicates the low byte)
                if (two) res = (uint16_t)((v & 0xFF) | ((v & 0xFF) << 8));
                else res = (uint16_t)(((v & 0xFF) << 8) | ((v >> 8) & 0xFF));
                set_sz_bit7(cpu, res);
                break;
            case 1: // SLL
                res = (uint16_t)(v << (1 + two));
                set_sz(cpu, res);
                break;
            case 2: // RLC (rotate left through C, and O for 2 positions)
                if (two) {
                    res = (uint16_t)((v << 2) | (cpu->flag_c << 1) | cpu->flag_o);
                    cpu->flag_c = (uint8_t)((v >> 15) & 1);
                    cpu->flag_o = (uint8_t)((v >> 14) & 1);
                } else {
                    res = (uint16_t)((v << 1) | cpu->flag_c);
                    cpu->flag_c = (uint8_t)((v >> 15) & 1);
                }
                set_sz(cpu, res);
                break;
            case 3: // SLLC
                res = (uint16_t)(v << (1 + two));
                cpu->flag_c = (uint8_t)((v >> 15) & 1);
                if (two) cpu->flag_o = (uint8_t)((v >> 14) & 1);
                set_sz(cpu, res);
                break;
            case 4: // SLR
                res = (uint16_t)(v >> (1 + two));
                set_sz_bit7(cpu, res);
                break;
            case 5: // SAR
                res = (uint16_t)((int16_t)v >> (1 + two));
                set_sz_bit7(cpu, res);
                break;
            case 6: // RRC (rotate right through C, and O for 2 positions)
                if (two) {
                    res = (uint16_t)((v >> 2) | (cpu->flag_c << 15) | (cpu->flag_o << 14));
                    cpu->flag_c = (uint8_t)((v >> 1) & 1);
                    cpu->flag_o = (uint8_t)(v & 1);
                } else {
                    res = (uint16_t)((v >> 1) | (cpu->flag_c << 15));
                    cpu->flag_c = (uint8_t)(v & 1);
                }
                set_sz_bit7(cpu, res);
                break;
            default: // SARC
                res = (uint16_t)((int16_t)v >> (1 + two));
                if (two) {
                    cpu->flag_c = (uint8_t)((v >> 1) & 1);
                    cpu->flag_o = (uint8_t)(v & 1);
                } else {
                    cpu->flag_c = (uint8_t)(v & 1);
                }
                set_sz_bit7(cpu, res);
                break;
        }
        cpu->r[reg] = res;
        return 0;
    }

    if (op < 0x200) { // Register-to-register: MOVR/ADDR/SUBR/CMPR/ANDR/XORR
        int src = (op >> 3) & 7;
        int dst = op & 7;
        uint16_t s = cpu->r[src];
        switch ((op >> 6) & 7) {
            case 2: // MOVR (TSTR / JR)
                cpu->r[dst] = s;
                set_sz(cpu, s);
                break;
            case 3: // ADDR
                cpu->r[dst] = do_add(cpu, cpu->r[dst], s);
                break;
            case 4: // SUBR
                cpu->r[dst] = do_sub(cpu, cpu->r[dst], s);
                break;
            case 5: // CMPR
                (void)do_sub(cpu, cpu->r[dst], s);
                break;
            case 6: // ANDR
                cpu->r[dst] = (uint16_t)(cpu->r[dst] & s);
                set_sz(cpu, cpu->r[dst]);
                break;
            default: // XORR (CLRR when src == dst)
                cpu->r[dst] = (uint16_t)(cpu->r[dst] ^ s);
                set_sz(cpu, cpu->r[dst]);
                break;
        }
        return 0;
    }

    if (op < 0x240) { // Conditional branches (2 words)
        uint16_t ofs = fetch(cpu);
        if (branch_taken(cpu, op)) {
            if (op & 0x20) cpu->r[7] = (uint16_t)(base + 1 - ofs); // backward
            else cpu->r[7] = (uint16_t)(base + 2 + ofs);           // forward
        }
        return 0;
    }

    // Memory-reference group: MVO/MVI/ADD/SUB/CMP/AND/XOR
    {
        int cls = (op >> 6) & 0xF;   // 9=MVO 10=MVI 11=ADD 12=SUB 13=CMP 14=AND 15=XOR
        int mode = (op >> 3) & 7;    // 0=direct, 1-6=indirect via R1-R6, 7=immediate
        int reg = op & 7;

        if (cls == 9) { // MVO / MVO@ / MVOI (SDBD is not defined for stores)
            uint16_t v = cpu->r[reg];
            if (mode == 0) {
                uint16_t addr = fetch(cpu);
                cpu->memory[addr] = v;
            } else {
                write_indirect(cpu, mode, v);
            }
            return 0;
        }

        uint16_t val;
        if (mode == 0) {
            uint16_t addr = fetch(cpu);
            val = cpu->memory[addr];
        } else {
            val = read_indirect(cpu, mode, sdbd);
        }

        switch (cls) {
            case 10: // MVI / MVI@ / MVII (no flags)
                cpu->r[reg] = val;
                break;
            case 11: // ADD
                cpu->r[reg] = do_add(cpu, cpu->r[reg], val);
                break;
            case 12: // SUB
                cpu->r[reg] = do_sub(cpu, cpu->r[reg], val);
                break;
            case 13: // CMP
                (void)do_sub(cpu, cpu->r[reg], val);
                break;
            case 14: // AND
                cpu->r[reg] = (uint16_t)(cpu->r[reg] & val);
                set_sz(cpu, cpu->r[reg]);
                break;
            default: // XOR
                cpu->r[reg] = (uint16_t)(cpu->r[reg] ^ val);
                set_sz(cpu, cpu->r[reg]);
                break;
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// State / disassembly
// ---------------------------------------------------------------------------

void cp1600_print_state(void *context) {
    if (!context) return;
    CP1600CPU *cpu = (CP1600CPU*)context;

    printf("CP1600 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  Flags: S=%u Z=%u O=%u C=%u  IntEn: %s  Halted: %s\n",
           cpu->r[7], cpu->flag_s, cpu->flag_z, cpu->flag_o, cpu->flag_c,
           cpu->intr_enabled ? "Yes" : "No",
           cpu->halted ? "Yes" : "No");
    printf("  Registers:\n");
    for (int i = 0; i < 8; ++i) {
        printf("    R%d: 0x%04X%s", i, cpu->r[i], (i % 4 == 3) ? "\n" : "  ");
    }
}

void cp1600_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    CP1600CPU *cpu = (CP1600CPU*)context;

    uint16_t pc = cpu->r[7];
    uint16_t op = (uint16_t)(cpu->memory[pc] & 0x3FF);
    uint16_t w1 = cpu->memory[(uint16_t)(pc + 1)];
    uint16_t w2 = cpu->memory[(uint16_t)(pc + 2)];

    if (op < 0x040) {
        int reg = op & 7;
        switch (op) {
            case 0x000: snprintf(buf, buf_len, "HLT"); return;
            case 0x001: snprintf(buf, buf_len, "SDBD"); return;
            case 0x002: snprintf(buf, buf_len, "EIS"); return;
            case 0x003: snprintf(buf, buf_len, "DIS"); return;
            case 0x004: {
                int rr = (w1 >> 8) & 3;
                int ee = w1 & 3;
                uint16_t target = (uint16_t)((((w1 >> 2) & 0x3F) << 10) | (w2 & 0x3FF));
                if (rr == 3) {
                    const char *n = (ee == 1) ? "JE" : (ee == 2) ? "JD" : "J";
                    snprintf(buf, buf_len, "%-4s $%04X", n, target);
                } else {
                    const char *n = (ee == 1) ? "JSRE" : (ee == 2) ? "JSRD" : "JSR";
                    snprintf(buf, buf_len, "%-4s R%d, $%04X", n, 4 + rr, target);
                }
                return;
            }
            case 0x005: snprintf(buf, buf_len, "TCI"); return;
            case 0x006: snprintf(buf, buf_len, "CLRC"); return;
            case 0x007: snprintf(buf, buf_len, "SETC"); return;
            default: break;
        }
        if (op < 0x010)      snprintf(buf, buf_len, "INCR R%d", reg);
        else if (op < 0x018) snprintf(buf, buf_len, "DECR R%d", reg);
        else if (op < 0x020) snprintf(buf, buf_len, "COMR R%d", reg);
        else if (op < 0x028) snprintf(buf, buf_len, "NEGR R%d", reg);
        else if (op < 0x030) snprintf(buf, buf_len, "ADCR R%d", reg);
        else if (op < 0x034) snprintf(buf, buf_len, "GSWD R%d", op & 3);
        else if (op < 0x036) snprintf(buf, buf_len, "NOP");
        else if (op < 0x038) snprintf(buf, buf_len, "SIN");
        else                 snprintf(buf, buf_len, "RSWD R%d", reg);
        return;
    }

    if (op < 0x080) { // Shifts
        static const char *snames[8] = {
            "SWAP", "SLL", "RLC", "SLLC", "SLR", "SAR", "RRC", "SARC"
        };
        snprintf(buf, buf_len, "%-4s R%d, %d",
                 snames[(op >> 3) & 7], op & 3, ((op >> 2) & 1) + 1);
        return;
    }

    if (op < 0x200) { // Register-to-register
        static const char *rnames[8] = {
            "", "", "MOVR", "ADDR", "SUBR", "CMPR", "ANDR", "XORR"
        };
        int src = (op >> 3) & 7;
        int dst = op & 7;
        int cls = (op >> 6) & 7;
        if (cls == 2 && dst == 7)
            snprintf(buf, buf_len, "JR   R%d", src);
        else if (cls == 7 && src == dst)
            snprintf(buf, buf_len, "CLRR R%d", dst);
        else
            snprintf(buf, buf_len, "%-4s R%d, R%d", rnames[cls], src, dst);
        return;
    }

    if (op < 0x240) { // Branches
        static const char *bnames[16] = {
            "B",    "BC",  "BOV",  "BPL", "BEQ",  "BLT", "BLE", "BUSC",
            "NOPP", "BNC", "BNOV", "BMI", "BNEQ", "BGE", "BGT", "BESC"
        };
        uint16_t target = (op & 0x20) ? (uint16_t)(pc + 1 - w1)
                                      : (uint16_t)(pc + 2 + w1);
        if (op & 0x10)
            snprintf(buf, buf_len, "BEXT %d, $%04X", op & 0xF, target);
        else if ((op & 0xF) == 8)
            snprintf(buf, buf_len, "NOPP");
        else
            snprintf(buf, buf_len, "%-4s $%04X", bnames[op & 0xF], target);
        return;
    }

    { // Memory-reference group
        static const char *mnames[7] = {
            "MVO", "MVI", "ADD", "SUB", "CMP", "AND", "XOR"
        };
        const char *name = mnames[((op >> 6) & 0xF) - 9];
        int mode = (op >> 3) & 7;
        int reg = op & 7;
        char mnem[8];

        if (((op >> 6) & 0xF) == 9) { // MVO: source register first
            if (mode == 0) {
                snprintf(buf, buf_len, "MVO  R%d, $%04X", reg, w1);
            } else if (mode == 7) {
                snprintf(buf, buf_len, "MVOI R%d", reg);
            } else {
                snprintf(buf, buf_len, "MVO@ R%d, R%d", reg, mode);
            }
            return;
        }
        if (mode == 0) {
            snprintf(buf, buf_len, "%-4s $%04X, R%d", name, w1, reg);
        } else if (mode == 7) {
            snprintf(mnem, sizeof(mnem), "%sI", name);
            snprintf(buf, buf_len, "%-4s #$%04X, R%d", mnem, w1, reg);
        } else {
            snprintf(mnem, sizeof(mnem), "%s@", name);
            snprintf(buf, buf_len, "%-4s R%d, R%d", mnem, mode, reg);
        }
    }
}
