// Motorola MC6805 8-bit microcontroller CPU emulator core.
//
// Implements the full documented MC6805 instruction set:
//   - Accumulator A, 8-bit index register X, 13-bit program counter
//   - 5-bit stack pointer confined to 0x60-0x7F (wraps within that range)
//   - 8KB of memory (13-bit address space, addresses masked to 0x1FFF)
//   - Addressing modes: inherent, immediate, direct, extended,
//     indexed with no / 1-byte / 2-byte offset, relative
//   - Bit manipulation: BSET/BCLR n, bit-test branches BRSET/BRCLR n
//   - Condition code register flags: H I N Z C (no V flag on the 6805)
//   - Branches, JSR/BSR/RTS, SWI/RTI, transfer and flag instructions
// STOP, WAIT and any undocumented/invalid opcode halt the CPU (step returns 1).
// BIL/BIH sample the external IRQ pin, which this emulator models as held high.

#include "mc6805.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_N 0x04
#define FLAG_I 0x08
#define FLAG_H 0x10
#define CCR_FIXED 0xE0     // Top three CCR bits always read as 1

#define MEM_SIZE 8192      // 13-bit address space
#define ADDR_MASK 0x1FFF

#define SWI_VECTOR 0x1FFC  // SWI vector (reset is 0x1FFE, IRQ 0x1FFA, timer 0x1FF8)

typedef struct MC6805_CPU {
    uint8_t ram[MEM_SIZE];
    uint8_t a;      // Accumulator
    uint8_t x;      // Index register
    uint8_t sp;     // Stack pointer (5 significant bits, 0x60-0x7F)
    uint16_t pc;    // Program counter (13-bit)
    uint8_t ccr;    // Condition code register (111HINZC)
    uint32_t ticks;
    int halted;
} MC6805_CPU;

#define SET_FLAG(flag, cond) do { if (cond) cpu->ccr |= (flag); else cpu->ccr &= (uint8_t)~(flag); } while(0)
#define GET_FLAG(flag) ((cpu->ccr & (flag)) ? 1 : 0)

// --- Memory helpers ---

static uint8_t mem_read(MC6805_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr & ADDR_MASK];
}

static void mem_write(MC6805_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr & ADDR_MASK] = val;
}

static uint8_t fetch8(MC6805_CPU *cpu) {
    uint8_t val = mem_read(cpu, cpu->pc);
    cpu->pc = (uint16_t)((cpu->pc + 1) & ADDR_MASK);
    return val;
}

static uint16_t fetch16(MC6805_CPU *cpu) {
    uint16_t hi = fetch8(cpu);
    uint16_t lo = fetch8(cpu);
    return (uint16_t)((hi << 8) | lo);
}

static uint16_t mem_read16(MC6805_CPU *cpu, uint16_t addr) {
    return (uint16_t)(((uint16_t)mem_read(cpu, addr) << 8) | mem_read(cpu, (uint16_t)(addr + 1)));
}

// Stack: SP points at the next free byte in 0x60-0x7F; push stores then
// decrements, and the 5-bit pointer wraps within that window.
static void push8(MC6805_CPU *cpu, uint8_t val) {
    mem_write(cpu, cpu->sp, val);
    cpu->sp = (uint8_t)(0x60 | ((cpu->sp - 1) & 0x1F));
}

static uint8_t pull8(MC6805_CPU *cpu) {
    cpu->sp = (uint8_t)(0x60 | ((cpu->sp + 1) & 0x1F));
    return mem_read(cpu, cpu->sp);
}

// The 6805 pushes 16-bit values low byte first (JSR pushes PCL then PCH).
static void push16(MC6805_CPU *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val & 0xFF));
    push8(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16(MC6805_CPU *cpu) {
    uint16_t hi = pull8(cpu);
    uint16_t lo = pull8(cpu);
    return (uint16_t)(((hi << 8) | lo) & ADDR_MASK);
}

// --- Flag helpers ---

static void set_nz8(MC6805_CPU *cpu, uint8_t val) {
    SET_FLAG(FLAG_N, val & 0x80);
    SET_FLAG(FLAG_Z, val == 0);
}

// --- ALU helpers ---

// 8-bit add with optional carry-in; sets H N Z C, returns the result.
static uint8_t alu_add(MC6805_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t sum = (uint16_t)lhs + rhs + carry_in;
    uint8_t res = (uint8_t)sum;
    SET_FLAG(FLAG_H, ((lhs & 0x0F) + (rhs & 0x0F) + carry_in) & 0x10);
    SET_FLAG(FLAG_C, sum & 0x100);
    set_nz8(cpu, res);
    return res;
}

// 8-bit subtract with optional borrow-in; sets N Z C, returns the result.
// Used for SUB/SBC/CMP/CPX (H is not affected by subtraction on the 6805).
static uint8_t alu_sub(MC6805_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t borrow_in) {
    uint16_t diff = (uint16_t)lhs - rhs - borrow_in;
    uint8_t res = (uint8_t)diff;
    SET_FLAG(FLAG_C, diff & 0x100);
    set_nz8(cpu, res);
    return res;
}

// Single-operand read-modify-write core (opcodes 0x30-0x7F, column select).
// Returns the new value; writeback indicates the result must be stored.
// Returns via *valid whether the column exists on the 6805.
static uint8_t alu_rmw(MC6805_CPU *cpu, uint8_t sub, uint8_t val, int *writeback, int *valid) {
    uint8_t res = val;
    uint8_t carry_in = (uint8_t)GET_FLAG(FLAG_C);
    *writeback = 1;
    *valid = 1;
    switch (sub) {
        case 0x00: // NEG
            res = (uint8_t)(0 - val);
            SET_FLAG(FLAG_C, val != 0);
            set_nz8(cpu, res);
            break;
        case 0x03: // COM
            res = (uint8_t)~val;
            SET_FLAG(FLAG_C, 1);
            set_nz8(cpu, res);
            break;
        case 0x04: // LSR
            SET_FLAG(FLAG_C, val & 0x01);
            res = (uint8_t)(val >> 1);
            set_nz8(cpu, res);
            break;
        case 0x06: // ROR
            SET_FLAG(FLAG_C, val & 0x01);
            res = (uint8_t)((val >> 1) | (carry_in << 7));
            set_nz8(cpu, res);
            break;
        case 0x07: // ASR
            SET_FLAG(FLAG_C, val & 0x01);
            res = (uint8_t)((val >> 1) | (val & 0x80));
            set_nz8(cpu, res);
            break;
        case 0x08: // ASL (LSL)
            SET_FLAG(FLAG_C, val & 0x80);
            res = (uint8_t)(val << 1);
            set_nz8(cpu, res);
            break;
        case 0x09: // ROL
            SET_FLAG(FLAG_C, val & 0x80);
            res = (uint8_t)((val << 1) | carry_in);
            set_nz8(cpu, res);
            break;
        case 0x0A: // DEC (C unaffected)
            res = (uint8_t)(val - 1);
            set_nz8(cpu, res);
            break;
        case 0x0C: // INC (C unaffected)
            res = (uint8_t)(val + 1);
            set_nz8(cpu, res);
            break;
        case 0x0D: // TST
            set_nz8(cpu, val);
            *writeback = 0;
            break;
        case 0x0F: // CLR (C unaffected on the 6805)
            res = 0;
            SET_FLAG(FLAG_N, 0);
            SET_FLAG(FLAG_Z, 1);
            break;
        default: // 0x1, 0x2, 0x5, 0xB, 0xE do not exist in this group
            *writeback = 0;
            *valid = 0;
            break;
    }
    return res;
}

// Branch condition evaluation for opcodes 0x20-0x2F (low nibble selects test).
static int branch_taken(MC6805_CPU *cpu, uint8_t sub) {
    int c = GET_FLAG(FLAG_C);
    int z = GET_FLAG(FLAG_Z);
    int n = GET_FLAG(FLAG_N);
    int h = GET_FLAG(FLAG_H);
    int i = GET_FLAG(FLAG_I);
    switch (sub) {
        case 0x00: return 1;        // BRA
        case 0x01: return 0;        // BRN (branch never)
        case 0x02: return !(c | z); // BHI
        case 0x03: return c | z;    // BLS
        case 0x04: return !c;       // BCC
        case 0x05: return c;        // BCS
        case 0x06: return !z;       // BNE
        case 0x07: return z;        // BEQ
        case 0x08: return !h;       // BHCC
        case 0x09: return h;        // BHCS
        case 0x0A: return !n;       // BPL
        case 0x0B: return n;        // BMI
        case 0x0C: return !i;       // BMC (interrupt mask clear)
        case 0x0D: return i;        // BMS (interrupt mask set)
        case 0x0E: return 0;        // BIL (IRQ pin low; pin modeled as high)
        case 0x0F: return 1;        // BIH (IRQ pin high; pin modeled as high)
        default: return 0;
    }
}

// --- Lifecycle ---

void* mc6805_create(void) {
    MC6805_CPU *cpu = (MC6805_CPU*)calloc(1, sizeof(MC6805_CPU));
    return cpu;
}

void mc6805_destroy(void *context) {
    free(context);
}

int mc6805_init(void *context) {
    if (!context) return -1;
    MC6805_CPU *cpu = (MC6805_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->x = 0;
    cpu->sp = 0x7F;
    cpu->pc = 0;
    cpu->ccr = CCR_FIXED | FLAG_I; // interrupts masked on reset
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int mc6805_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    MC6805_CPU *cpu = (MC6805_CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(cpu->ram + address, data, copy_len);
    cpu->pc = (uint16_t)(address & ADDR_MASK);

    return 0;
}

// --- Execution ---

int mc6805_step(void *context) {
    if (!context) return -1;
    MC6805_CPU *cpu = (MC6805_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks++;

    // --- Bit-test branches BRSET/BRCLR n 0x00-0x0F (direct, 3 bytes) ---
    if (op < 0x10) {
        uint8_t bit = (uint8_t)(1 << (op >> 1));
        uint8_t addr = fetch8(cpu);
        int8_t rel = (int8_t)fetch8(cpu);
        uint8_t val = mem_read(cpu, addr);
        int set = (val & bit) != 0;
        SET_FLAG(FLAG_C, set); // tested bit is copied into C
        if ((op & 0x01) ? !set : set) { // even = BRSET, odd = BRCLR
            cpu->pc = (uint16_t)((cpu->pc + rel) & ADDR_MASK);
        }
        return 0;
    }

    // --- Bit set/clear BSET/BCLR n 0x10-0x1F (direct, 2 bytes) ---
    if (op < 0x20) {
        uint8_t bit = (uint8_t)(1 << ((op >> 1) & 0x07));
        uint8_t addr = fetch8(cpu);
        uint8_t val = mem_read(cpu, addr);
        if (op & 0x01) val &= (uint8_t)~bit; // even = BSET, odd = BCLR
        else val |= bit;
        mem_write(cpu, addr, val);
        return 0;
    }

    // --- Relative branches 0x20-0x2F ---
    if (op < 0x30) {
        int8_t rel = (int8_t)fetch8(cpu);
        if (branch_taken(cpu, (uint8_t)(op & 0x0F))) {
            cpu->pc = (uint16_t)((cpu->pc + rel) & ADDR_MASK);
        }
        return 0;
    }

    // --- Read-modify-write group 0x30-0x7F ---
    // Rows: 3=direct, 4=A, 5=X, 6=indexed 1-byte offset, 7=indexed no offset
    if (op < 0x80) {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)(op >> 4);
        uint16_t addr = 0;
        uint8_t val;
        int writeback, valid;

        if (mode == 3) addr = fetch8(cpu);
        else if (mode == 6) addr = (uint16_t)(cpu->x + fetch8(cpu));
        else if (mode == 7) addr = cpu->x;

        if (mode == 4) val = cpu->a;
        else if (mode == 5) val = cpu->x;
        else val = mem_read(cpu, addr);

        val = alu_rmw(cpu, sub, val, &writeback, &valid);
        if (!valid) {
            cpu->halted = 1;
            return 1;
        }
        if (writeback) {
            if (mode == 4) cpu->a = val;
            else if (mode == 5) cpu->x = val;
            else mem_write(cpu, addr, val);
        }
        return 0;
    }

    // --- Control group 0x80-0x8F ---
    if (op < 0x90) {
        switch (op) {
            case 0x80: // RTI (pull CCR, A, X, PC)
                cpu->ccr = (uint8_t)(pull8(cpu) | CCR_FIXED);
                cpu->a = pull8(cpu);
                cpu->x = pull8(cpu);
                cpu->pc = pull16(cpu);
                break;
            case 0x81: // RTS
                cpu->pc = pull16(cpu);
                break;
            case 0x83: // SWI (push PC, X, A, CCR; vector via 0x1FFC)
                push16(cpu, cpu->pc);
                push8(cpu, cpu->x);
                push8(cpu, cpu->a);
                push8(cpu, cpu->ccr);
                SET_FLAG(FLAG_I, 1);
                cpu->pc = (uint16_t)(mem_read16(cpu, SWI_VECTOR) & ADDR_MASK);
                break;
            case 0x8E: // STOP - halts this emulator
            case 0x8F: // WAIT - halts this emulator
                cpu->halted = 1;
                return 1;
            default: // invalid opcode
                cpu->halted = 1;
                return 1;
        }
        return 0;
    }

    // --- Inherent group 0x90-0x9F ---
    if (op < 0xA0) {
        switch (op) {
            case 0x97: // TAX
                cpu->x = cpu->a;
                break;
            case 0x98: // CLC
                SET_FLAG(FLAG_C, 0);
                break;
            case 0x99: // SEC
                SET_FLAG(FLAG_C, 1);
                break;
            case 0x9A: // CLI
                SET_FLAG(FLAG_I, 0);
                break;
            case 0x9B: // SEI
                SET_FLAG(FLAG_I, 1);
                break;
            case 0x9C: // RSP (reset stack pointer to 0x7F)
                cpu->sp = 0x7F;
                break;
            case 0x9D: // NOP
                break;
            case 0x9F: // TXA
                cpu->a = cpu->x;
                break;
            default: // invalid opcode
                cpu->halted = 1;
                return 1;
        }
        return 0;
    }

    // --- Register/memory group 0xA0-0xFF ---
    // Rows: A=immediate, B=direct, C=extended, D=indexed 2-byte offset,
    //       E=indexed 1-byte offset, F=indexed no offset
    {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)((op >> 4) - 0x0A);
        int is_imm = (mode == 0);
        uint16_t addr = 0;
        uint8_t m = 0;
        uint8_t carry_in = (uint8_t)GET_FLAG(FLAG_C);

        // BSR occupies the immediate slot of the JSR column
        if (sub == 0x0D && is_imm) {
            int8_t rel = (int8_t)fetch8(cpu);
            push16(cpu, cpu->pc);
            cpu->pc = (uint16_t)((cpu->pc + rel) & ADDR_MASK);
            return 0;
        }

        // STA/STX/JMP/JSR have no immediate form
        if (is_imm && (sub == 0x07 || sub == 0x0C || sub == 0x0F)) {
            cpu->halted = 1;
            return 1;
        }

        if (!is_imm) {
            switch (mode) {
                case 1: addr = fetch8(cpu); break;                            // direct
                case 2: addr = (uint16_t)(fetch16(cpu) & ADDR_MASK); break;   // extended
                case 3: addr = (uint16_t)(cpu->x + fetch16(cpu)); break;      // ix2
                case 4: addr = (uint16_t)(cpu->x + fetch8(cpu)); break;       // ix1
                default: addr = cpu->x; break;                                // ix0
            }
        }

        switch (sub) {
            case 0x00: // SUB
                cpu->a = alu_sub(cpu, cpu->a, is_imm ? fetch8(cpu) : mem_read(cpu, addr), 0);
                break;
            case 0x01: // CMP
                (void)alu_sub(cpu, cpu->a, is_imm ? fetch8(cpu) : mem_read(cpu, addr), 0);
                break;
            case 0x02: // SBC
                cpu->a = alu_sub(cpu, cpu->a, is_imm ? fetch8(cpu) : mem_read(cpu, addr), carry_in);
                break;
            case 0x03: // CPX
                (void)alu_sub(cpu, cpu->x, is_imm ? fetch8(cpu) : mem_read(cpu, addr), 0);
                break;
            case 0x04: // AND
                cpu->a &= is_imm ? fetch8(cpu) : mem_read(cpu, addr);
                set_nz8(cpu, cpu->a);
                break;
            case 0x05: // BIT (AND without storing)
                m = is_imm ? fetch8(cpu) : mem_read(cpu, addr);
                set_nz8(cpu, (uint8_t)(cpu->a & m));
                break;
            case 0x06: // LDA
                cpu->a = is_imm ? fetch8(cpu) : mem_read(cpu, addr);
                set_nz8(cpu, cpu->a);
                break;
            case 0x07: // STA
                mem_write(cpu, addr, cpu->a);
                set_nz8(cpu, cpu->a);
                break;
            case 0x08: // EOR
                cpu->a ^= is_imm ? fetch8(cpu) : mem_read(cpu, addr);
                set_nz8(cpu, cpu->a);
                break;
            case 0x09: // ADC
                cpu->a = alu_add(cpu, cpu->a, is_imm ? fetch8(cpu) : mem_read(cpu, addr), carry_in);
                break;
            case 0x0A: // ORA
                cpu->a |= is_imm ? fetch8(cpu) : mem_read(cpu, addr);
                set_nz8(cpu, cpu->a);
                break;
            case 0x0B: // ADD
                cpu->a = alu_add(cpu, cpu->a, is_imm ? fetch8(cpu) : mem_read(cpu, addr), 0);
                break;
            case 0x0C: // JMP
                cpu->pc = (uint16_t)(addr & ADDR_MASK);
                break;
            case 0x0D: // JSR
                push16(cpu, cpu->pc);
                cpu->pc = (uint16_t)(addr & ADDR_MASK);
                break;
            case 0x0E: // LDX
                cpu->x = is_imm ? fetch8(cpu) : mem_read(cpu, addr);
                set_nz8(cpu, cpu->x);
                break;
            default: // 0x0F: STX
                mem_write(cpu, addr, cpu->x);
                set_nz8(cpu, cpu->x);
                break;
        }
    }

    return 0;
}

// --- Debugging ---

void mc6805_print_state(void *context) {
    if (!context) return;
    MC6805_CPU *cpu = (MC6805_CPU*)context;

    printf("Motorola 6805 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%02X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Registers: A=0x%02X  X=0x%02X\n", cpu->a, cpu->x);
    printf("  Flags: H=%d  I=%d  N=%d  Z=%d  C=%d\n",
           GET_FLAG(FLAG_H), GET_FLAG(FLAG_I), GET_FLAG(FLAG_N),
           GET_FLAG(FLAG_Z), GET_FLAG(FLAG_C));
}

// Mnemonic tables for the disassembler
static const char *g_branch_names[16] = {
    "bra", "brn", "bhi",  "bls",  "bcc", "bcs", "bne", "beq",
    "bhcc", "bhcs", "bpl", "bmi", "bmc", "bms", "bil", "bih"
};

static const char *g_rmw_names[16] = {
    "neg", "?", "?", "com", "lsr", "?", "ror", "asr",
    "asl", "rol", "dec", "?", "inc", "tst", "?", "clr"
};

static const char *g_alu_names[16] = {
    "sub", "cmp", "sbc", "cpx", "and", "bit", "lda", "sta",
    "eor", "adc", "ora", "add", "jmp", "jsr", "ldx", "stx"
};

void mc6805_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    MC6805_CPU *cpu = (MC6805_CPU*)context;

    uint8_t op = mem_read(cpu, cpu->pc);
    uint8_t b1 = mem_read(cpu, (uint16_t)(cpu->pc + 1));
    uint8_t b2 = mem_read(cpu, (uint16_t)(cpu->pc + 2));
    uint16_t w = (uint16_t)(((uint16_t)b1 << 8) | b2);

    // Bit-test branches
    if (op < 0x10) {
        snprintf(buf, buf_len, "%s %d,$0x%02X,%+d",
                 (op & 0x01) ? "brclr" : "brset", op >> 1, b1, (int8_t)b2);
        return;
    }

    // Bit set/clear
    if (op < 0x20) {
        snprintf(buf, buf_len, "%s %d,$0x%02X",
                 (op & 0x01) ? "bclr" : "bset", (op >> 1) & 0x07, b1);
        return;
    }

    // Relative branches
    if (op < 0x30) {
        snprintf(buf, buf_len, "%s   %+d", g_branch_names[op & 0x0F], (int8_t)b1);
        return;
    }

    // Read-modify-write group
    if (op < 0x80) {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)(op >> 4);
        const char *name = g_rmw_names[sub];
        if (name[0] == '?') {
            snprintf(buf, buf_len, "unknown (0x%02X)", op);
            return;
        }
        if (mode == 3) snprintf(buf, buf_len, "%s   $0x%02X", name, b1);
        else if (mode == 4) snprintf(buf, buf_len, "%s   a", name);
        else if (mode == 5) snprintf(buf, buf_len, "%s   x", name);
        else if (mode == 6) snprintf(buf, buf_len, "%s   $0x%02X,x", name, b1);
        else snprintf(buf, buf_len, "%s   ,x", name);
        return;
    }

    // Control group
    if (op < 0x90) {
        const char *name = NULL;
        switch (op) {
            case 0x80: name = "rti"; break;
            case 0x81: name = "rts"; break;
            case 0x83: name = "swi"; break;
            case 0x8E: name = "stop"; break;
            case 0x8F: name = "wait"; break;
            default: break;
        }
        if (name) snprintf(buf, buf_len, "%s", name);
        else snprintf(buf, buf_len, "unknown (0x%02X)", op);
        return;
    }

    // Inherent group
    if (op < 0xA0) {
        const char *name = NULL;
        switch (op) {
            case 0x97: name = "tax"; break;
            case 0x98: name = "clc"; break;
            case 0x99: name = "sec"; break;
            case 0x9A: name = "cli"; break;
            case 0x9B: name = "sei"; break;
            case 0x9C: name = "rsp"; break;
            case 0x9D: name = "nop"; break;
            case 0x9F: name = "txa"; break;
            default: break;
        }
        if (name) snprintf(buf, buf_len, "%s", name);
        else snprintf(buf, buf_len, "unknown (0x%02X)", op);
        return;
    }

    // Register/memory group
    {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)((op >> 4) - 0x0A);
        const char *name = g_alu_names[sub];

        if (mode == 0) { // immediate row
            if (sub == 0x0D) { snprintf(buf, buf_len, "bsr   %+d", (int8_t)b1); return; }
            if (sub == 0x07 || sub == 0x0C || sub == 0x0F) {
                snprintf(buf, buf_len, "unknown (0x%02X)", op);
                return;
            }
            snprintf(buf, buf_len, "%s   #$0x%02X", name, b1);
        }
        else if (mode == 1) snprintf(buf, buf_len, "%s   $0x%02X", name, b1);
        else if (mode == 2) snprintf(buf, buf_len, "%s   $0x%04X", name, w);
        else if (mode == 3) snprintf(buf, buf_len, "%s   $0x%04X,x", name, w);
        else if (mode == 4) snprintf(buf, buf_len, "%s   $0x%02X,x", name, b1);
        else snprintf(buf, buf_len, "%s   ,x", name);
    }
}
