#include "f8.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB (16-bit address space)
#define NUM_PORTS 256

// Fairchild F8 (3850 CPU + integrated program counter model).
// Emulates the combined system: main memory, scratchpad, PC0/PC1, DC0/DC1.
typedef struct F8CPU {
    uint8_t a;           // Accumulator
    uint8_t isar;        // Indirect Scratchpad Address Register (6-bit)
    uint8_t scratch[64]; // 64-byte scratchpad

    uint16_t pc0;        // Program Counter
    uint16_t pc1;        // Stack Register (single-level return address)
    uint16_t dc0;        // Data Counter
    uint16_t dc1;        // Auxiliary Data Counter

    // W status register bits
    uint8_t flag_s;      // Sign (1 = result positive, i.e. bit 7 clear)
    uint8_t flag_c;      // Carry
    uint8_t flag_z;      // Zero
    uint8_t flag_o;      // Overflow
    uint8_t flag_i;      // ICB (Interrupt Control Bit)

    uint8_t memory[MEM_SIZE];
    uint8_t ports[NUM_PORTS];
    uint32_t ticks;
    int halted;
} F8CPU;

// Scratchpad register field names (used by DS/AS/ASD/XS/NS/LR)
static const char* sp_names[] = {
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11",
    "S", "I", "D", "?"
};

// --- Flag helpers -----------------------------------------------------------

static inline void set_sz(F8CPU *cpu, uint8_t res) {
    cpu->flag_z = (res == 0) ? 1 : 0;
    cpu->flag_s = (res & 0x80) ? 0 : 1; // F8 sign flag is complement of bit 7
}

// Binary add through the ALU: sets Carry, Overflow, Sign, Zero.
static inline uint8_t do_add(F8CPU *cpu, uint8_t x, uint8_t y, uint8_t cin) {
    uint16_t sum = (uint16_t)x + (uint16_t)y + cin;
    uint16_t low = (uint16_t)(x & 0x7F) + (uint16_t)(y & 0x7F) + cin;
    cpu->flag_c = (sum > 0xFF) ? 1 : 0;
    cpu->flag_o = (uint8_t)(((sum >> 8) ^ (low >> 7)) & 1); // carry7 XOR carry6
    set_sz(cpu, (uint8_t)sum);
    return (uint8_t)sum;
}

// Decimal add (ASD/AMD): flags come from the binary add, then the result is
// digit-adjusted (assumes the conventional 0x66 pre-adjustment of A).
static inline uint8_t do_add_decimal(F8CPU *cpu, uint8_t x, uint8_t y) {
    uint8_t c = ((((uint16_t)x + (uint16_t)y) & 0xFF0) > 0xF0) ? 1 : 0;
    uint8_t ic = (((x & 0x0F) + (y & 0x0F)) > 0x0F) ? 1 : 0;
    uint16_t tmp = do_add(cpu, x, y, 0);

    if (c == 0 && ic == 0) tmp = ((tmp + 0xA0) & 0xF0) + ((tmp + 0x0A) & 0x0F);
    if (c == 0 && ic == 1) tmp = ((tmp + 0xA0) & 0xF0) + (tmp & 0x0F);
    if (c == 1 && ic == 0) tmp = (tmp & 0xF0) + ((tmp + 0x0A) & 0x0F);
    return (uint8_t)tmp;
}

// Logical result: sets Sign and Zero, clears Carry and Overflow.
static inline void set_logic_flags(F8CPU *cpu, uint8_t res) {
    cpu->flag_c = 0;
    cpu->flag_o = 0;
    set_sz(cpu, res);
}

static inline uint8_t get_w(F8CPU *cpu) {
    return (uint8_t)((cpu->flag_s ? 0x01 : 0) |
                     (cpu->flag_c ? 0x02 : 0) |
                     (cpu->flag_z ? 0x04 : 0) |
                     (cpu->flag_o ? 0x08 : 0) |
                     (cpu->flag_i ? 0x10 : 0));
}

static inline void set_w(F8CPU *cpu, uint8_t w) {
    cpu->flag_s = (w >> 0) & 1;
    cpu->flag_c = (w >> 1) & 1;
    cpu->flag_z = (w >> 2) & 1;
    cpu->flag_o = (w >> 3) & 1;
    cpu->flag_i = (w >> 4) & 1;
}

// --- Scratchpad addressing --------------------------------------------------
// Register field 0-11 selects the scratchpad byte directly.
// 12 (S): ISAR unchanged; 13 (I): ISAR, then increment lower octal digit;
// 14 (D): ISAR, then decrement lower octal digit. The upper octal digit of
// ISAR is never affected by the increment/decrement. 15 is illegal.
static inline int sp_addr(F8CPU *cpu, uint8_t r) {
    int addr;
    switch (r) {
        case 0x0C:
            return cpu->isar & 0x3F;
        case 0x0D:
            addr = cpu->isar & 0x3F;
            cpu->isar = (uint8_t)((cpu->isar & 0x38) | ((cpu->isar + 1) & 0x07));
            return addr;
        case 0x0E:
            addr = cpu->isar & 0x3F;
            cpu->isar = (uint8_t)((cpu->isar & 0x38) | ((cpu->isar - 1) & 0x07));
            return addr;
        case 0x0F:
            return -1; // illegal register designation
        default:
            return r;
    }
}

// --- Lifecycle ---------------------------------------------------------------

void* f8_create(void) {
    F8CPU *cpu = (F8CPU*)calloc(1, sizeof(F8CPU));
    return cpu;
}

void f8_destroy(void *context) {
    free(context);
}

int f8_init(void *context) {
    if (!context) return -1;
    F8CPU *cpu = (F8CPU*)context;

    memset(cpu, 0, sizeof(F8CPU));
    return 0;
}

int f8_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    F8CPU *cpu = (F8CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// --- Execution ---------------------------------------------------------------

int f8_step(void *context) {
    if (!context) return -1;
    F8CPU *cpu = (F8CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc0;
    uint8_t op = cpu->memory[cpu->pc0];
    uint8_t byte2 = cpu->memory[(uint16_t)(cpu->pc0 + 1)];
    uint8_t byte3 = cpu->memory[(uint16_t)(cpu->pc0 + 2)];

    // Instruction length: 3 bytes for PI/JMP/DCI (0x28-0x2A); 2 bytes for
    // immediates (0x20-0x27) and branches (0x80-0x87, 0x8F, 0x90-0x9F);
    // 1 byte otherwise.
    uint16_t len = 1;
    if (op >= 0x28 && op <= 0x2A) len = 3;
    else if (op >= 0x20 && op <= 0x27) len = 2;
    else if ((op >= 0x80 && op <= 0x87) || op == 0x8F || (op >= 0x90 && op <= 0x9F)) len = 2;

    cpu->pc0 = (uint16_t)(cpu->pc0 + len);
    cpu->ticks++;

    if (op < 0x20) {
        switch (op) {
            case 0x00: cpu->a = cpu->scratch[12]; break; // LR A, KU
            case 0x01: cpu->a = cpu->scratch[13]; break; // LR A, KL
            case 0x02: cpu->a = cpu->scratch[14]; break; // LR A, QU
            case 0x03: cpu->a = cpu->scratch[15]; break; // LR A, QL
            case 0x04: cpu->scratch[12] = cpu->a; break; // LR KU, A
            case 0x05: cpu->scratch[13] = cpu->a; break; // LR KL, A
            case 0x06: cpu->scratch[14] = cpu->a; break; // LR QU, A
            case 0x07: cpu->scratch[15] = cpu->a; break; // LR QL, A
            case 0x08: // LR K, P (K <- PC1)
                cpu->scratch[12] = (uint8_t)(cpu->pc1 >> 8);
                cpu->scratch[13] = (uint8_t)(cpu->pc1 & 0xFF);
                break;
            case 0x09: // LR P, K (PC1 <- K)
                cpu->pc1 = (uint16_t)(((uint16_t)cpu->scratch[12] << 8) | cpu->scratch[13]);
                break;
            case 0x0A: cpu->a = cpu->isar; break;        // LR A, IS
            case 0x0B: cpu->isar = cpu->a & 0x3F; break; // LR IS, A
            case 0x0C: // PK (PC1 <- PC0; PC0 <- K)
                cpu->pc1 = cpu->pc0;
                cpu->pc0 = (uint16_t)(((uint16_t)cpu->scratch[12] << 8) | cpu->scratch[13]);
                break;
            case 0x0D: // LR P0, Q (PC0 <- Q; a jump)
                cpu->pc0 = (uint16_t)(((uint16_t)cpu->scratch[14] << 8) | cpu->scratch[15]);
                break;
            case 0x0E: // LR Q, DC
                cpu->scratch[14] = (uint8_t)(cpu->dc0 >> 8);
                cpu->scratch[15] = (uint8_t)(cpu->dc0 & 0xFF);
                break;
            case 0x0F: // LR DC, Q
                cpu->dc0 = (uint16_t)(((uint16_t)cpu->scratch[14] << 8) | cpu->scratch[15]);
                break;
            case 0x10: // LR DC, H
                cpu->dc0 = (uint16_t)(((uint16_t)cpu->scratch[10] << 8) | cpu->scratch[11]);
                break;
            case 0x11: // LR H, DC
                cpu->scratch[10] = (uint8_t)(cpu->dc0 >> 8);
                cpu->scratch[11] = (uint8_t)(cpu->dc0 & 0xFF);
                break;
            case 0x12: // SR 1
                cpu->a >>= 1;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x13: // SL 1
                cpu->a = (uint8_t)(cpu->a << 1);
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x14: // SR 4
                cpu->a >>= 4;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x15: // SL 4
                cpu->a = (uint8_t)(cpu->a << 4);
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x16: // LM (A <- [DC0]; DC0++)
                cpu->a = cpu->memory[cpu->dc0];
                cpu->dc0++;
                break;
            case 0x17: // ST ([DC0] <- A; DC0++)
                cpu->memory[cpu->dc0] = cpu->a;
                cpu->dc0++;
                break;
            case 0x18: // COM
                cpu->a = (uint8_t)(~cpu->a);
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x19: // LNK (A <- A + Carry)
                cpu->a = do_add(cpu, cpu->a, cpu->flag_c, 0);
                break;
            case 0x1A: cpu->flag_i = 0; break; // DI
            case 0x1B: cpu->flag_i = 1; break; // EI
            case 0x1C: cpu->pc0 = cpu->pc1; break; // POP (return)
            case 0x1D: set_w(cpu, cpu->scratch[9]); break; // LR W, J
            case 0x1E: cpu->scratch[9] = get_w(cpu); break; // LR J, W
            case 0x1F: // INC
                cpu->a = do_add(cpu, cpu->a, 1, 0);
                break;
        }
    }
    else if (op < 0x30) {
        switch (op) {
            case 0x20: cpu->a = byte2; break; // LI
            case 0x21: // NI
                cpu->a &= byte2;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x22: // OI
                cpu->a |= byte2;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x23: // XI
                cpu->a ^= byte2;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x24: // AI
                cpu->a = do_add(cpu, cpu->a, byte2, 0);
                break;
            case 0x25: // CI (compare: operand + ~A + 1, result discarded)
                (void)do_add(cpu, (uint8_t)(~cpu->a), byte2, 1);
                break;
            case 0x26: // IN port
                cpu->a = cpu->ports[byte2];
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x27: // OUT port
                cpu->ports[byte2] = cpu->a;
                break;
            case 0x28: // PI addr (A <- hi byte; PC1 <- return; PC0 <- addr)
                cpu->a = byte2;
                cpu->pc1 = cpu->pc0;
                cpu->pc0 = (uint16_t)(((uint16_t)byte2 << 8) | byte3);
                break;
            case 0x29: // JMP addr (A <- hi byte; PC0 <- addr)
                cpu->a = byte2;
                cpu->pc0 = (uint16_t)(((uint16_t)byte2 << 8) | byte3);
                break;
            case 0x2A: // DCI addr
                cpu->dc0 = (uint16_t)(((uint16_t)byte2 << 8) | byte3);
                break;
            case 0x2B: break; // NOP
            case 0x2C: { // XDC (swap DC0 and DC1)
                uint16_t tmp = cpu->dc0;
                cpu->dc0 = cpu->dc1;
                cpu->dc1 = tmp;
                break;
            }
            default: break; // 0x2D-0x2F: unassigned, treated as NOP
        }
    }
    else if (op < 0x40) {
        // DS r (decrement scratchpad: r <- r + 0xFF through the ALU)
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) {
            cpu->scratch[addr] = do_add(cpu, cpu->scratch[addr], 0xFF, 0);
        }
    }
    else if (op < 0x50) {
        // LR A, r
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) cpu->a = cpu->scratch[addr];
    }
    else if (op < 0x60) {
        // LR r, A
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) cpu->scratch[addr] = cpu->a;
    }
    else if (op < 0x68) {
        // LISU i (upper octal digit of ISAR)
        cpu->isar = (uint8_t)((cpu->isar & 0x07) | ((op & 0x07) << 3));
    }
    else if (op < 0x70) {
        // LISL i (lower octal digit of ISAR)
        cpu->isar = (uint8_t)((cpu->isar & 0x38) | (op & 0x07));
    }
    else if (op < 0x80) {
        // LIS i (A <- 4-bit immediate, no status change)
        cpu->a = op & 0x0F;
    }
    else if (op < 0x88) {
        // BT t, disp (branch if any tested bit of W is set: 1=S, 2=C, 4=Z)
        uint8_t bits = (uint8_t)((cpu->flag_s ? 1 : 0) |
                                 (cpu->flag_c ? 2 : 0) |
                                 (cpu->flag_z ? 4 : 0));
        if (bits & (op & 0x07)) {
            cpu->pc0 = (uint16_t)(instr_pc + 1 + (int8_t)byte2);
        }
    }
    else if (op < 0x90) {
        switch (op) {
            case 0x88: // AM
                cpu->a = do_add(cpu, cpu->a, cpu->memory[cpu->dc0], 0);
                cpu->dc0++;
                break;
            case 0x89: // AMD (decimal)
                cpu->a = do_add_decimal(cpu, cpu->a, cpu->memory[cpu->dc0]);
                cpu->dc0++;
                break;
            case 0x8A: // NM
                cpu->a &= cpu->memory[cpu->dc0];
                cpu->dc0++;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x8B: // OM
                cpu->a |= cpu->memory[cpu->dc0];
                cpu->dc0++;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x8C: // XM
                cpu->a ^= cpu->memory[cpu->dc0];
                cpu->dc0++;
                set_logic_flags(cpu, cpu->a);
                break;
            case 0x8D: // CM (compare memory, result discarded)
                (void)do_add(cpu, (uint8_t)(~cpu->a), cpu->memory[cpu->dc0], 1);
                cpu->dc0++;
                break;
            case 0x8E: // ADC (DC0 <- DC0 + A, A treated as signed)
                cpu->dc0 = (uint16_t)(cpu->dc0 + (int8_t)cpu->a);
                break;
            case 0x8F: // BR7 (branch if ISAR lower octal digit != 7)
                if ((cpu->isar & 0x07) != 0x07) {
                    cpu->pc0 = (uint16_t)(instr_pc + 1 + (int8_t)byte2);
                }
                break;
        }
    }
    else if (op < 0xA0) {
        // BF t, disp (branch if all tested bits of W are clear: 1=S, 2=C, 4=Z, 8=O)
        uint8_t bits = (uint8_t)((cpu->flag_s ? 1 : 0) |
                                 (cpu->flag_c ? 2 : 0) |
                                 (cpu->flag_z ? 4 : 0) |
                                 (cpu->flag_o ? 8 : 0));
        if ((bits & (op & 0x0F)) == 0) {
            cpu->pc0 = (uint16_t)(instr_pc + 1 + (int8_t)byte2);
        }
    }
    else if (op < 0xB0) {
        // INS port (0-15)
        cpu->a = cpu->ports[op & 0x0F];
        set_logic_flags(cpu, cpu->a);
    }
    else if (op < 0xC0) {
        // OUTS port (0-15)
        cpu->ports[op & 0x0F] = cpu->a;
    }
    else if (op < 0xD0) {
        // AS r (binary add scratchpad)
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) cpu->a = do_add(cpu, cpu->a, cpu->scratch[addr], 0);
    }
    else if (op < 0xE0) {
        // ASD r (decimal add scratchpad)
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) cpu->a = do_add_decimal(cpu, cpu->a, cpu->scratch[addr]);
    }
    else if (op < 0xF0) {
        // XS r
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) {
            cpu->a ^= cpu->scratch[addr];
            set_logic_flags(cpu, cpu->a);
        }
    }
    else {
        // NS r
        int addr = sp_addr(cpu, op & 0x0F);
        if (addr >= 0) {
            cpu->a &= cpu->scratch[addr];
            set_logic_flags(cpu, cpu->a);
        }
    }

    // A branch/jump back to itself is interpreted as a software halt
    if (cpu->pc0 == instr_pc) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

// --- State display -----------------------------------------------------------

void f8_print_state(void *context) {
    if (!context) return;
    F8CPU *cpu = (F8CPU*)context;

    printf("Fairchild F8 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  A: 0x%02X  ISAR: 0%o%o (0x%02X)  Halted: %s\n",
           cpu->a, (cpu->isar >> 3) & 7, cpu->isar & 7, cpu->isar,
           cpu->halted ? "Yes" : "No");
    printf("  W: Sign=%d, Carry=%d, Zero=%d, Ovf=%d, ICB=%d\n",
           cpu->flag_s, cpu->flag_c, cpu->flag_z, cpu->flag_o, cpu->flag_i);
    printf("  PC0: 0x%04X  PC1: 0x%04X  DC0: 0x%04X  DC1: 0x%04X\n",
           cpu->pc0, cpu->pc1, cpu->dc0, cpu->dc1);
    printf("  Scratchpad 0-15:\n");
    for (int i = 0; i < 16; ++i) {
        printf("    r%-2d: 0x%02X%s", i, cpu->scratch[i], (i % 4 == 3) ? "\n" : "  ");
    }
}

// --- Disassembly -------------------------------------------------------------

void f8_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    F8CPU *cpu = (F8CPU*)context;

    uint8_t op = cpu->memory[cpu->pc0];
    uint8_t byte2 = cpu->memory[(uint16_t)(cpu->pc0 + 1)];
    uint8_t byte3 = cpu->memory[(uint16_t)(cpu->pc0 + 2)];
    uint16_t branch_target = (uint16_t)(cpu->pc0 + 1 + (int8_t)byte2);
    uint16_t addr16 = (uint16_t)(((uint16_t)byte2 << 8) | byte3);

    if (op < 0x20) {
        static const char* low_names[] = {
            "LR    A, KU", "LR    A, KL", "LR    A, QU", "LR    A, QL",
            "LR    KU, A", "LR    KL, A", "LR    QU, A", "LR    QL, A",
            "LR    K, P",  "LR    P, K",  "LR    A, IS", "LR    IS, A",
            "PK",          "LR    P0, Q", "LR    Q, DC", "LR    DC, Q",
            "LR    DC, H", "LR    H, DC", "SR    1",     "SL    1",
            "SR    4",     "SL    4",     "LM",          "ST",
            "COM",         "LNK",         "DI",          "EI",
            "POP",         "LR    W, J",  "LR    J, W",  "INC"
        };
        snprintf(buf, buf_len, "%s", low_names[op]);
    }
    else if (op >= 0x20 && op <= 0x25) {
        static const char* imm_names[] = { "LI", "NI", "OI", "XI", "AI", "CI" };
        snprintf(buf, buf_len, "%-5s 0x%02X", imm_names[op - 0x20], byte2);
    }
    else if (op == 0x26) {
        snprintf(buf, buf_len, "IN    0x%02X", byte2);
    }
    else if (op == 0x27) {
        snprintf(buf, buf_len, "OUT   0x%02X", byte2);
    }
    else if (op == 0x28) {
        snprintf(buf, buf_len, "PI    0x%04X", addr16);
    }
    else if (op == 0x29) {
        snprintf(buf, buf_len, "JMP   0x%04X", addr16);
    }
    else if (op == 0x2A) {
        snprintf(buf, buf_len, "DCI   0x%04X", addr16);
    }
    else if (op == 0x2B) {
        snprintf(buf, buf_len, "NOP");
    }
    else if (op == 0x2C) {
        snprintf(buf, buf_len, "XDC");
    }
    else if (op >= 0x2D && op <= 0x2F) {
        snprintf(buf, buf_len, "INV   0x%02X", op);
    }
    else if (op < 0x40) {
        snprintf(buf, buf_len, "DS    %s", sp_names[op & 0x0F]);
    }
    else if (op < 0x50) {
        snprintf(buf, buf_len, "LR    A, %s", sp_names[op & 0x0F]);
    }
    else if (op < 0x60) {
        snprintf(buf, buf_len, "LR    %s, A", sp_names[op & 0x0F]);
    }
    else if (op < 0x68) {
        snprintf(buf, buf_len, "LISU  %d", op & 0x07);
    }
    else if (op < 0x70) {
        snprintf(buf, buf_len, "LISL  %d", op & 0x07);
    }
    else if (op < 0x80) {
        snprintf(buf, buf_len, "LIS   0x%X", op & 0x0F);
    }
    else if (op < 0x88) {
        switch (op) {
            case 0x81: snprintf(buf, buf_len, "BP    0x%04X", branch_target); break;
            case 0x82: snprintf(buf, buf_len, "BC    0x%04X", branch_target); break;
            case 0x84: snprintf(buf, buf_len, "BZ    0x%04X", branch_target); break;
            default:
                snprintf(buf, buf_len, "BT    %d, 0x%04X", op & 0x07, branch_target);
                break;
        }
    }
    else if (op < 0x90) {
        static const char* mem_names[] = { "AM", "AMD", "NM", "OM", "XM", "CM", "ADC" };
        if (op == 0x8F) {
            snprintf(buf, buf_len, "BR7   0x%04X", branch_target);
        } else {
            snprintf(buf, buf_len, "%s", mem_names[op - 0x88]);
        }
    }
    else if (op < 0xA0) {
        switch (op) {
            case 0x90: snprintf(buf, buf_len, "BR    0x%04X", branch_target); break;
            case 0x91: snprintf(buf, buf_len, "BM    0x%04X", branch_target); break;
            case 0x92: snprintf(buf, buf_len, "BNC   0x%04X", branch_target); break;
            case 0x94: snprintf(buf, buf_len, "BNZ   0x%04X", branch_target); break;
            case 0x98: snprintf(buf, buf_len, "BNO   0x%04X", branch_target); break;
            default:
                snprintf(buf, buf_len, "BF    %d, 0x%04X", op & 0x0F, branch_target);
                break;
        }
    }
    else if (op < 0xB0) {
        snprintf(buf, buf_len, "INS   %d", op & 0x0F);
    }
    else if (op < 0xC0) {
        snprintf(buf, buf_len, "OUTS  %d", op & 0x0F);
    }
    else if (op < 0xD0) {
        snprintf(buf, buf_len, "AS    %s", sp_names[op & 0x0F]);
    }
    else if (op < 0xE0) {
        snprintf(buf, buf_len, "ASD   %s", sp_names[op & 0x0F]);
    }
    else if (op < 0xF0) {
        snprintf(buf, buf_len, "XS    %s", sp_names[op & 0x0F]);
    }
    else {
        snprintf(buf, buf_len, "NS    %s", sp_names[op & 0x0F]);
    }
}
