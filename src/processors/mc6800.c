// Motorola MC6800 8-bit CPU emulator core.
//
// Implements the full documented MC6800 instruction set:
//   - Accumulators A and B, 16-bit index register X, 16-bit stack pointer
//   - Addressing modes: inherent, immediate, direct, indexed, extended, relative
//   - Condition code register flags: H I N Z V C
//   - Branches, JSR/BSR/RTS, SWI/RTI, stack and transfer instructions
// WAI and any undocumented/invalid opcode halt the CPU (step returns 1).

#include "mc6800.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_C 0x01
#define FLAG_V 0x02
#define FLAG_Z 0x04
#define FLAG_N 0x08
#define FLAG_I 0x10
#define FLAG_H 0x20
#define CCR_FIXED 0xC0 // Top two CCR bits always read as 1

typedef struct MC6800_CPU {
    uint8_t ram[65536];
    uint8_t a;      // Accumulator A
    uint8_t b;      // Accumulator B
    uint16_t x;     // Index register
    uint16_t sp;    // Stack pointer
    uint16_t pc;    // Program counter
    uint8_t ccr;    // Condition code register (11HINZVC)
    uint32_t ticks;
    int halted;
} MC6800_CPU;

#define SET_FLAG(flag, cond) do { if (cond) cpu->ccr |= (flag); else cpu->ccr &= (uint8_t)~(flag); } while(0)
#define GET_FLAG(flag) ((cpu->ccr & (flag)) ? 1 : 0)

// --- Memory helpers ---

static uint8_t mem_read(MC6800_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr];
}

static void mem_write(MC6800_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr] = val;
}

static uint8_t fetch8(MC6800_CPU *cpu) {
    return mem_read(cpu, cpu->pc++);
}

static uint16_t fetch16(MC6800_CPU *cpu) {
    uint16_t hi = mem_read(cpu, cpu->pc++);
    uint16_t lo = mem_read(cpu, cpu->pc++);
    return (uint16_t)((hi << 8) | lo);
}

static uint16_t mem_read16(MC6800_CPU *cpu, uint16_t addr) {
    return (uint16_t)(((uint16_t)mem_read(cpu, addr) << 8) | mem_read(cpu, (uint16_t)(addr + 1)));
}

// Stack: SP points at the next free byte; push stores then decrements.
static void push8(MC6800_CPU *cpu, uint8_t val) {
    mem_write(cpu, cpu->sp--, val);
}

static uint8_t pull8(MC6800_CPU *cpu) {
    return mem_read(cpu, ++cpu->sp);
}

// The 6800 pushes 16-bit values low byte first (JSR pushes PCL then PCH).
static void push16(MC6800_CPU *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val & 0xFF));
    push8(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16(MC6800_CPU *cpu) {
    uint16_t hi = pull8(cpu);
    uint16_t lo = pull8(cpu);
    return (uint16_t)((hi << 8) | lo);
}

// --- Flag helpers ---

static void set_nz8(MC6800_CPU *cpu, uint8_t val) {
    SET_FLAG(FLAG_N, val & 0x80);
    SET_FLAG(FLAG_Z, val == 0);
}

static void set_nz16(MC6800_CPU *cpu, uint16_t val) {
    SET_FLAG(FLAG_N, val & 0x8000);
    SET_FLAG(FLAG_Z, val == 0);
}

// --- ALU helpers ---

// 8-bit add with optional carry-in; sets H N Z V C, returns the result.
static uint8_t alu_add(MC6800_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t sum = (uint16_t)lhs + rhs + carry_in;
    uint8_t res = (uint8_t)sum;
    SET_FLAG(FLAG_H, ((lhs & 0x0F) + (rhs & 0x0F) + carry_in) & 0x10);
    SET_FLAG(FLAG_C, sum & 0x100);
    SET_FLAG(FLAG_V, (~(lhs ^ rhs) & (lhs ^ res)) & 0x80);
    set_nz8(cpu, res);
    return res;
}

// 8-bit subtract with optional borrow-in; sets N Z V C, returns the result.
// Used for SUB/SBC/CMP/SBA/CBA (H is not affected by subtraction on the 6800).
static uint8_t alu_sub(MC6800_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t borrow_in) {
    uint16_t diff = (uint16_t)lhs - rhs - borrow_in;
    uint8_t res = (uint8_t)diff;
    SET_FLAG(FLAG_C, diff & 0x100);
    SET_FLAG(FLAG_V, ((lhs ^ rhs) & (lhs ^ res)) & 0x80);
    set_nz8(cpu, res);
    return res;
}

// Single-operand read-modify-write core (opcodes 0x40-0x7F).
// Returns the new value; performed indicates a write-back is required.
static uint8_t alu_rmw(MC6800_CPU *cpu, uint8_t sub, uint8_t val, int *writeback) {
    uint8_t res = val;
    uint8_t carry_in = (uint8_t)GET_FLAG(FLAG_C);
    *writeback = 1;
    switch (sub) {
        case 0x00: // NEG
            res = (uint8_t)(0 - val);
            SET_FLAG(FLAG_V, val == 0x80);
            SET_FLAG(FLAG_C, val != 0);
            set_nz8(cpu, res);
            break;
        case 0x03: // COM
            res = (uint8_t)~val;
            SET_FLAG(FLAG_V, 0);
            SET_FLAG(FLAG_C, 1);
            set_nz8(cpu, res);
            break;
        case 0x04: // LSR
            SET_FLAG(FLAG_C, val & 0x01);
            res = (uint8_t)(val >> 1);
            set_nz8(cpu, res);
            SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
            break;
        case 0x06: // ROR
            SET_FLAG(FLAG_C, val & 0x01);
            res = (uint8_t)((val >> 1) | (carry_in << 7));
            set_nz8(cpu, res);
            SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
            break;
        case 0x07: // ASR
            SET_FLAG(FLAG_C, val & 0x01);
            res = (uint8_t)((val >> 1) | (val & 0x80));
            set_nz8(cpu, res);
            SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
            break;
        case 0x08: // ASL
            SET_FLAG(FLAG_C, val & 0x80);
            res = (uint8_t)(val << 1);
            set_nz8(cpu, res);
            SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
            break;
        case 0x09: // ROL
            SET_FLAG(FLAG_C, val & 0x80);
            res = (uint8_t)((val << 1) | carry_in);
            set_nz8(cpu, res);
            SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
            break;
        case 0x0A: // DEC (C unaffected)
            res = (uint8_t)(val - 1);
            SET_FLAG(FLAG_V, val == 0x80);
            set_nz8(cpu, res);
            break;
        case 0x0C: // INC (C unaffected)
            res = (uint8_t)(val + 1);
            SET_FLAG(FLAG_V, val == 0x7F);
            set_nz8(cpu, res);
            break;
        case 0x0D: // TST
            set_nz8(cpu, val);
            SET_FLAG(FLAG_V, 0);
            SET_FLAG(FLAG_C, 0);
            *writeback = 0;
            break;
        case 0x0F: // CLR
            res = 0;
            SET_FLAG(FLAG_N, 0);
            SET_FLAG(FLAG_Z, 1);
            SET_FLAG(FLAG_V, 0);
            SET_FLAG(FLAG_C, 0);
            break;
        default:
            *writeback = 0;
            break;
    }
    return res;
}

// Branch condition evaluation for opcodes 0x20-0x2F (low nibble selects test).
static int branch_taken(MC6800_CPU *cpu, uint8_t sub) {
    int c = GET_FLAG(FLAG_C);
    int z = GET_FLAG(FLAG_Z);
    int n = GET_FLAG(FLAG_N);
    int v = GET_FLAG(FLAG_V);
    switch (sub) {
        case 0x00: return 1;            // BRA
        case 0x02: return !(c | z);     // BHI
        case 0x03: return c | z;        // BLS
        case 0x04: return !c;           // BCC
        case 0x05: return c;            // BCS
        case 0x06: return !z;           // BNE
        case 0x07: return z;            // BEQ
        case 0x08: return !v;           // BVC
        case 0x09: return v;            // BVS
        case 0x0A: return !n;           // BPL
        case 0x0B: return n;            // BMI
        case 0x0C: return !(n ^ v);     // BGE
        case 0x0D: return n ^ v;        // BLT
        case 0x0E: return !(z | (n ^ v)); // BGT
        case 0x0F: return z | (n ^ v);  // BLE
        default: return -1;             // invalid encoding (0x21 undocumented)
    }
}

// --- Lifecycle ---

void* mc6800_create(void) {
    MC6800_CPU *cpu = (MC6800_CPU*)calloc(1, sizeof(MC6800_CPU));
    return cpu;
}

void mc6800_destroy(void *context) {
    free(context);
}

int mc6800_init(void *context) {
    if (!context) return -1;
    MC6800_CPU *cpu = (MC6800_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->b = 0;
    cpu->x = 0;
    cpu->sp = 0x01FF;
    cpu->pc = 0;
    cpu->ccr = CCR_FIXED | FLAG_I; // interrupts masked on reset
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int mc6800_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    MC6800_CPU *cpu = (MC6800_CPU*)context;

    if (address >= 65536) return -2;
    size_t copy_len = size;
    if (address + size > 65536) {
        copy_len = 65536 - address;
    }
    memcpy(cpu->ram + address, data, copy_len);
    cpu->pc = (uint16_t)address;

    return 0;
}

// --- Execution ---

int mc6800_step(void *context) {
    if (!context) return -1;
    MC6800_CPU *cpu = (MC6800_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks++;

    // --- Inherent group 0x00-0x1F ---
    if (op < 0x20) {
        switch (op) {
            case 0x01: // NOP
                break;
            case 0x06: // TAP (A -> CCR)
                cpu->ccr = (uint8_t)(cpu->a | CCR_FIXED);
                break;
            case 0x07: // TPA (CCR -> A)
                cpu->a = (uint8_t)(cpu->ccr | CCR_FIXED);
                break;
            case 0x08: // INX
                cpu->x++;
                SET_FLAG(FLAG_Z, cpu->x == 0);
                break;
            case 0x09: // DEX
                cpu->x--;
                SET_FLAG(FLAG_Z, cpu->x == 0);
                break;
            case 0x0A: // CLV
                SET_FLAG(FLAG_V, 0);
                break;
            case 0x0B: // SEV
                SET_FLAG(FLAG_V, 1);
                break;
            case 0x0C: // CLC
                SET_FLAG(FLAG_C, 0);
                break;
            case 0x0D: // SEC
                SET_FLAG(FLAG_C, 1);
                break;
            case 0x0E: // CLI
                SET_FLAG(FLAG_I, 0);
                break;
            case 0x0F: // SEI
                SET_FLAG(FLAG_I, 1);
                break;
            case 0x10: // SBA (A = A - B)
                cpu->a = alu_sub(cpu, cpu->a, cpu->b, 0);
                break;
            case 0x11: // CBA (compare A with B)
                (void)alu_sub(cpu, cpu->a, cpu->b, 0);
                break;
            case 0x16: // TAB
                cpu->b = cpu->a;
                set_nz8(cpu, cpu->b);
                SET_FLAG(FLAG_V, 0);
                break;
            case 0x17: // TBA
                cpu->a = cpu->b;
                set_nz8(cpu, cpu->a);
                SET_FLAG(FLAG_V, 0);
                break;
            case 0x19: // DAA (decimal adjust A after BCD add)
                {
                    uint8_t adjust = 0;
                    uint8_t hi = (uint8_t)(cpu->a >> 4);
                    uint8_t lo = (uint8_t)(cpu->a & 0x0F);
                    int carry = GET_FLAG(FLAG_C);
                    if (lo > 9 || GET_FLAG(FLAG_H)) adjust |= 0x06;
                    if (hi > 9 || carry || (hi == 9 && lo > 9)) adjust |= 0x60;
                    uint16_t res = (uint16_t)cpu->a + adjust;
                    cpu->a = (uint8_t)res;
                    SET_FLAG(FLAG_C, carry || (res & 0x100));
                    set_nz8(cpu, cpu->a);
                }
                break;
            case 0x1B: // ABA (A = A + B)
                cpu->a = alu_add(cpu, cpu->a, cpu->b, 0);
                break;
            default: // invalid opcode
                cpu->halted = 1;
                return 1;
        }
        return 0;
    }

    // --- Relative branches 0x20-0x2F ---
    if (op < 0x30) {
        int8_t rel = (int8_t)fetch8(cpu);
        int taken = branch_taken(cpu, (uint8_t)(op & 0x0F));
        if (taken < 0) {
            cpu->halted = 1;
            return 1;
        }
        if (taken) {
            cpu->pc = (uint16_t)(cpu->pc + rel);
        }
        return 0;
    }

    // --- Stack / control group 0x30-0x3F ---
    if (op < 0x40) {
        switch (op) {
            case 0x30: // TSX (X = SP + 1)
                cpu->x = (uint16_t)(cpu->sp + 1);
                break;
            case 0x31: // INS
                cpu->sp++;
                break;
            case 0x32: // PULA
                cpu->a = pull8(cpu);
                break;
            case 0x33: // PULB
                cpu->b = pull8(cpu);
                break;
            case 0x34: // DES
                cpu->sp--;
                break;
            case 0x35: // TXS (SP = X - 1)
                cpu->sp = (uint16_t)(cpu->x - 1);
                break;
            case 0x36: // PSHA
                push8(cpu, cpu->a);
                break;
            case 0x37: // PSHB
                push8(cpu, cpu->b);
                break;
            case 0x39: // RTS
                cpu->pc = pull16(cpu);
                break;
            case 0x3B: // RTI (pull CCR, B, A, X, PC)
                cpu->ccr = (uint8_t)(pull8(cpu) | CCR_FIXED);
                cpu->b = pull8(cpu);
                cpu->a = pull8(cpu);
                cpu->x = pull16(cpu);
                cpu->pc = pull16(cpu);
                break;
            case 0x3E: // WAI (wait for interrupt) - halts this emulator
                cpu->halted = 1;
                return 1;
            case 0x3F: // SWI (push machine state, vector via 0xFFFA)
                push16(cpu, cpu->pc);
                push16(cpu, cpu->x);
                push8(cpu, cpu->a);
                push8(cpu, cpu->b);
                push8(cpu, cpu->ccr);
                SET_FLAG(FLAG_I, 1);
                cpu->pc = mem_read16(cpu, 0xFFFA);
                break;
            default: // invalid opcode
                cpu->halted = 1;
                return 1;
        }
        return 0;
    }

    // --- Single-operand group 0x40-0x7F (NEG/COM/shifts/DEC/INC/TST/JMP/CLR) ---
    if (op < 0x80) {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)(op >> 4); // 4=A, 5=B, 6=indexed, 7=extended
        uint16_t addr = 0;
        uint8_t val;
        int writeback;

        if (mode == 6) {
            addr = (uint16_t)(cpu->x + fetch8(cpu));
        } else if (mode == 7) {
            addr = fetch16(cpu);
        }

        if (sub == 0x0E) { // JMP (indexed/extended only)
            if (mode == 6 || mode == 7) {
                cpu->pc = addr;
                return 0;
            }
            cpu->halted = 1;
            return 1;
        }

        if (mode == 4) val = cpu->a;
        else if (mode == 5) val = cpu->b;
        else val = mem_read(cpu, addr);

        // Reject subcodes that do not exist on the 6800 (0x1, 0x2, 0x5, 0xB)
        if (sub == 0x01 || sub == 0x02 || sub == 0x05 || sub == 0x0B) {
            cpu->halted = 1;
            return 1;
        }

        val = alu_rmw(cpu, sub, val, &writeback);
        if (writeback) {
            if (mode == 4) cpu->a = val;
            else if (mode == 5) cpu->b = val;
            else mem_write(cpu, addr, val);
        }
        return 0;
    }

    // --- Accumulator/memory group 0x80-0xFF ---
    {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)((op >> 4) & 0x03); // 0=imm, 1=direct, 2=indexed, 3=extended
        int use_b = (op & 0x40) != 0;               // 0x80-0xBF -> A, 0xC0-0xFF -> B
        uint16_t addr = 0;

        // 16-bit / flow-control columns (0xC-0xF) first
        if (sub >= 0x0C) {
            if (!use_b) {
                if (sub == 0x0C) { // CPX (compare X, 16-bit; C unaffected)
                    uint16_t m;
                    if (mode == 0) {
                        m = fetch16(cpu);
                    } else {
                        if (mode == 1) addr = fetch8(cpu);
                        else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                        else addr = fetch16(cpu);
                        m = mem_read16(cpu, addr);
                    }
                    {
                        uint32_t diff = (uint32_t)cpu->x - m;
                        uint16_t res = (uint16_t)diff;
                        SET_FLAG(FLAG_V, ((cpu->x ^ m) & (cpu->x ^ res)) & 0x8000);
                        set_nz16(cpu, res);
                    }
                    return 0;
                }
                if (sub == 0x0D) { // BSR (imm slot) / JSR (indexed/extended)
                    if (mode == 0) { // 0x8D BSR
                        int8_t rel = (int8_t)fetch8(cpu);
                        push16(cpu, cpu->pc);
                        cpu->pc = (uint16_t)(cpu->pc + rel);
                        return 0;
                    }
                    if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                    else if (mode == 3) addr = fetch16(cpu);
                    else { cpu->halted = 1; return 1; } // 0x9D invalid
                    push16(cpu, cpu->pc);
                    cpu->pc = addr;
                    return 0;
                }
                if (sub == 0x0E) { // LDS
                    if (mode == 0) {
                        cpu->sp = fetch16(cpu);
                    } else {
                        if (mode == 1) addr = fetch8(cpu);
                        else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                        else addr = fetch16(cpu);
                        cpu->sp = mem_read16(cpu, addr);
                    }
                    set_nz16(cpu, cpu->sp);
                    SET_FLAG(FLAG_V, 0);
                    return 0;
                }
                // sub == 0x0F: STS (no immediate form; 0x8F invalid)
                if (mode == 0) { cpu->halted = 1; return 1; }
                if (mode == 1) addr = fetch8(cpu);
                else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                else addr = fetch16(cpu);
                mem_write(cpu, addr, (uint8_t)(cpu->sp >> 8));
                mem_write(cpu, (uint16_t)(addr + 1), (uint8_t)(cpu->sp & 0xFF));
                set_nz16(cpu, cpu->sp);
                SET_FLAG(FLAG_V, 0);
                return 0;
            } else {
                if (sub == 0x0E) { // LDX
                    if (mode == 0) {
                        cpu->x = fetch16(cpu);
                    } else {
                        if (mode == 1) addr = fetch8(cpu);
                        else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                        else addr = fetch16(cpu);
                        cpu->x = mem_read16(cpu, addr);
                    }
                    set_nz16(cpu, cpu->x);
                    SET_FLAG(FLAG_V, 0);
                    return 0;
                }
                if (sub == 0x0F) { // STX (no immediate form; 0xCF invalid)
                    if (mode == 0) { cpu->halted = 1; return 1; }
                    if (mode == 1) addr = fetch8(cpu);
                    else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                    else addr = fetch16(cpu);
                    mem_write(cpu, addr, (uint8_t)(cpu->x >> 8));
                    mem_write(cpu, (uint16_t)(addr + 1), (uint8_t)(cpu->x & 0xFF));
                    set_nz16(cpu, cpu->x);
                    SET_FLAG(FLAG_V, 0);
                    return 0;
                }
                // 0xCC, 0xCD, 0xDC, 0xDD, 0xEC, 0xED, 0xFC, 0xFD invalid on 6800
                cpu->halted = 1;
                return 1;
            }
        }

        // 8-bit operation columns 0x0-0xB
        if (sub == 0x03) { // no 0x83/0x93/... on the 6800
            cpu->halted = 1;
            return 1;
        }

        {
            uint8_t acc = use_b ? cpu->b : cpu->a;
            uint8_t m = 0;
            uint8_t carry_in = (uint8_t)GET_FLAG(FLAG_C);

            if (sub == 0x07) { // STA/STB (no immediate form)
                if (mode == 0) { cpu->halted = 1; return 1; }
                if (mode == 1) addr = fetch8(cpu);
                else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                else addr = fetch16(cpu);
                mem_write(cpu, addr, acc);
                set_nz8(cpu, acc);
                SET_FLAG(FLAG_V, 0);
                return 0;
            }

            if (mode == 0) {
                m = fetch8(cpu);
            } else {
                if (mode == 1) addr = fetch8(cpu);
                else if (mode == 2) addr = (uint16_t)(cpu->x + fetch8(cpu));
                else addr = fetch16(cpu);
                m = mem_read(cpu, addr);
            }

            switch (sub) {
                case 0x00: // SUB
                    acc = alu_sub(cpu, acc, m, 0);
                    break;
                case 0x01: // CMP
                    (void)alu_sub(cpu, acc, m, 0);
                    return 0;
                case 0x02: // SBC
                    acc = alu_sub(cpu, acc, m, carry_in);
                    break;
                case 0x04: // AND
                    acc &= m;
                    set_nz8(cpu, acc);
                    SET_FLAG(FLAG_V, 0);
                    break;
                case 0x05: // BIT (AND without storing)
                    set_nz8(cpu, (uint8_t)(acc & m));
                    SET_FLAG(FLAG_V, 0);
                    return 0;
                case 0x06: // LDA
                    acc = m;
                    set_nz8(cpu, acc);
                    SET_FLAG(FLAG_V, 0);
                    break;
                case 0x08: // EOR
                    acc ^= m;
                    set_nz8(cpu, acc);
                    SET_FLAG(FLAG_V, 0);
                    break;
                case 0x09: // ADC
                    acc = alu_add(cpu, acc, m, carry_in);
                    break;
                case 0x0A: // ORA
                    acc |= m;
                    set_nz8(cpu, acc);
                    SET_FLAG(FLAG_V, 0);
                    break;
                case 0x0B: // ADD
                    acc = alu_add(cpu, acc, m, 0);
                    break;
                default:
                    cpu->halted = 1;
                    return 1;
            }

            if (use_b) cpu->b = acc;
            else cpu->a = acc;
        }
    }

    return 0;
}

// --- Debugging ---

void mc6800_print_state(void *context) {
    if (!context) return;
    MC6800_CPU *cpu = (MC6800_CPU*)context;

    printf("Motorola 6800 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Registers: A=0x%02X  B=0x%02X  X=0x%04X\n", cpu->a, cpu->b, cpu->x);
    printf("  Flags: H=%d  I=%d  N=%d  Z=%d  V=%d  C=%d\n",
           GET_FLAG(FLAG_H), GET_FLAG(FLAG_I), GET_FLAG(FLAG_N),
           GET_FLAG(FLAG_Z), GET_FLAG(FLAG_V), GET_FLAG(FLAG_C));
}

// Mnemonic tables for the disassembler
static const char *g_branch_names[16] = {
    "bra", "?",   "bhi", "bls", "bcc", "bcs", "bne", "beq",
    "bvc", "bvs", "bpl", "bmi", "bge", "blt", "bgt", "ble"
};

static const char *g_rmw_names[16] = {
    "neg", "?", "?", "com", "lsr", "?", "ror", "asr",
    "asl", "rol", "dec", "?", "inc", "tst", "jmp", "clr"
};

static const char *g_alu_names[12] = {
    "sub", "cmp", "sbc", "?", "and", "bit", "lda", "sta",
    "eor", "adc", "ora", "add"
};

void mc6800_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    MC6800_CPU *cpu = (MC6800_CPU*)context;

    uint8_t op = mem_read(cpu, cpu->pc);
    uint8_t b1 = mem_read(cpu, (uint16_t)(cpu->pc + 1));
    uint16_t w = (uint16_t)(((uint16_t)b1 << 8) | mem_read(cpu, (uint16_t)(cpu->pc + 2)));

    // Inherent group
    if (op < 0x20) {
        const char *name = NULL;
        switch (op) {
            case 0x01: name = "nop"; break;
            case 0x06: name = "tap"; break;
            case 0x07: name = "tpa"; break;
            case 0x08: name = "inx"; break;
            case 0x09: name = "dex"; break;
            case 0x0A: name = "clv"; break;
            case 0x0B: name = "sev"; break;
            case 0x0C: name = "clc"; break;
            case 0x0D: name = "sec"; break;
            case 0x0E: name = "cli"; break;
            case 0x0F: name = "sei"; break;
            case 0x10: name = "sba"; break;
            case 0x11: name = "cba"; break;
            case 0x16: name = "tab"; break;
            case 0x17: name = "tba"; break;
            case 0x19: name = "daa"; break;
            case 0x1B: name = "aba"; break;
            default: break;
        }
        if (name) snprintf(buf, buf_len, "%s", name);
        else snprintf(buf, buf_len, "unknown (0x%02X)", op);
        return;
    }

    // Branches
    if (op < 0x30) {
        if (op == 0x21) {
            snprintf(buf, buf_len, "unknown (0x%02X)", op);
        } else {
            snprintf(buf, buf_len, "%s   %+d", g_branch_names[op & 0x0F], (int8_t)b1);
        }
        return;
    }

    // Stack / control group
    if (op < 0x40) {
        const char *name = NULL;
        switch (op) {
            case 0x30: name = "tsx"; break;
            case 0x31: name = "ins"; break;
            case 0x32: name = "pula"; break;
            case 0x33: name = "pulb"; break;
            case 0x34: name = "des"; break;
            case 0x35: name = "txs"; break;
            case 0x36: name = "psha"; break;
            case 0x37: name = "pshb"; break;
            case 0x39: name = "rts"; break;
            case 0x3B: name = "rti"; break;
            case 0x3E: name = "wai"; break;
            case 0x3F: name = "swi"; break;
            default: break;
        }
        if (name) snprintf(buf, buf_len, "%s", name);
        else snprintf(buf, buf_len, "unknown (0x%02X)", op);
        return;
    }

    // Single-operand group
    if (op < 0x80) {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)(op >> 4);
        const char *name = g_rmw_names[sub];
        if (name[0] == '?' || (sub == 0x0E && mode < 6)) {
            snprintf(buf, buf_len, "unknown (0x%02X)", op);
            return;
        }
        if (mode == 4) snprintf(buf, buf_len, "%s   a", name);
        else if (mode == 5) snprintf(buf, buf_len, "%s   b", name);
        else if (mode == 6) snprintf(buf, buf_len, "%s   $0x%02X,x", name, b1);
        else snprintf(buf, buf_len, "%s   $0x%04X", name, w);
        return;
    }

    // Accumulator/memory group
    {
        uint8_t sub = (uint8_t)(op & 0x0F);
        uint8_t mode = (uint8_t)((op >> 4) & 0x03);
        int use_b = (op & 0x40) != 0;
        const char *name = NULL;
        char reg = use_b ? 'b' : 'a';

        if (sub >= 0x0C) {
            if (!use_b) {
                if (sub == 0x0C) name = "cpx";
                else if (sub == 0x0D) {
                    if (mode == 0) { snprintf(buf, buf_len, "bsr   %+d", (int8_t)b1); return; }
                    if (mode == 1) { snprintf(buf, buf_len, "unknown (0x%02X)", op); return; }
                    name = "jsr";
                }
                else if (sub == 0x0E) name = "lds";
                else name = "sts";
            } else {
                if (sub == 0x0E) name = "ldx";
                else if (sub == 0x0F) name = "stx";
            }
            if (!name || (mode == 0 && (sub == 0x0F || name[0] == 's'))) {
                snprintf(buf, buf_len, "unknown (0x%02X)", op);
                return;
            }
            if (mode == 0) snprintf(buf, buf_len, "%s   #$0x%04X", name, w);
            else if (mode == 1) snprintf(buf, buf_len, "%s   $0x%02X", name, b1);
            else if (mode == 2) snprintf(buf, buf_len, "%s   $0x%02X,x", name, b1);
            else snprintf(buf, buf_len, "%s   $0x%04X", name, w);
            return;
        }

        name = g_alu_names[sub];
        if (name[0] == '?' || (mode == 0 && sub == 0x07)) {
            snprintf(buf, buf_len, "unknown (0x%02X)", op);
            return;
        }
        if (mode == 0) snprintf(buf, buf_len, "%s%c  #$0x%02X", name, reg, b1);
        else if (mode == 1) snprintf(buf, buf_len, "%s%c  $0x%02X", name, reg, b1);
        else if (mode == 2) snprintf(buf, buf_len, "%s%c  $0x%02X,x", name, reg, b1);
        else snprintf(buf, buf_len, "%s%c  $0x%04X", name, reg, w);
    }
}
