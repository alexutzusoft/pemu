#include "z8000.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Zilog Z8000 CPU core (practical subset, non-segmented Z8002 mode).
// 64KB of big-endian memory, sixteen 16-bit registers R0-R15 which pair up
// as RR0..RR14 and expose byte halves RH0-RH7/RL0-RL7 on R0-R7.
#define MEM_SIZE 65536

// FCW flag bits (low byte of the Flag and Control Word)
#define FLAG_C 0x80     // Carry
#define FLAG_Z 0x40     // Zero
#define FLAG_S 0x20     // Sign
#define FLAG_P 0x10     // Parity / Overflow
#define FLAG_D 0x08     // Decimal adjust (subtract)
#define FLAG_H 0x04     // Half carry

typedef struct Z8000CPU {
    uint16_t r[16];     // R0-R15 (R15 = stack pointer for @R15 push/pop)
    uint16_t pc;
    uint16_t fcw;       // Flag and Control Word
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} Z8000CPU;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* z8000_create(void) {
    Z8000CPU *cpu = (Z8000CPU*)calloc(1, sizeof(Z8000CPU));
    return cpu;
}

void z8000_destroy(void *context) {
    free(context);
}

int z8000_init(void *context) {
    if (!context) return -1;
    Z8000CPU *cpu = (Z8000CPU*)context;

    memset(cpu->r, 0, sizeof(cpu->r));
    cpu->r[15] = 0x0000;    // Stack pointer; first push wraps to top of RAM
    cpu->pc = 0;
    cpu->fcw = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int z8000_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    Z8000CPU *cpu = (Z8000CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// ---------------------------------------------------------------------------
// Memory helpers (Z8000 is big-endian; 16-bit addresses wrap naturally)
// ---------------------------------------------------------------------------

static uint8_t read8(Z8000CPU *cpu, uint16_t addr) {
    return cpu->memory[addr];
}

static uint16_t read16(Z8000CPU *cpu, uint16_t addr) {
    return (uint16_t)((cpu->memory[addr] << 8) | cpu->memory[(uint16_t)(addr + 1)]);
}

static uint32_t read32(Z8000CPU *cpu, uint16_t addr) {
    return ((uint32_t)read16(cpu, addr) << 16) | read16(cpu, (uint16_t)(addr + 2));
}

static void write8(Z8000CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->memory[addr] = val;
}

static void write16(Z8000CPU *cpu, uint16_t addr, uint16_t val) {
    cpu->memory[addr] = (uint8_t)(val >> 8);
    cpu->memory[(uint16_t)(addr + 1)] = (uint8_t)(val & 0xFF);
}

static void write32(Z8000CPU *cpu, uint16_t addr, uint32_t val) {
    write16(cpu, addr, (uint16_t)(val >> 16));
    write16(cpu, (uint16_t)(addr + 2), (uint16_t)(val & 0xFFFF));
}

static uint16_t fetch16(Z8000CPU *cpu) {
    uint16_t v = read16(cpu, cpu->pc);
    cpu->pc = (uint16_t)(cpu->pc + 2);
    return v;
}

static void push16(Z8000CPU *cpu, uint16_t val) {
    cpu->r[15] = (uint16_t)(cpu->r[15] - 2);
    write16(cpu, cpu->r[15], val);
}

static uint16_t pop16(Z8000CPU *cpu) {
    uint16_t v = read16(cpu, cpu->r[15]);
    cpu->r[15] = (uint16_t)(cpu->r[15] + 2);
    return v;
}

// ---------------------------------------------------------------------------
// Register helpers
// ---------------------------------------------------------------------------

// Byte registers: indices 0-7 are RH0-RH7 (high bytes of R0-R7),
// indices 8-15 are RL0-RL7 (low bytes of R0-R7).
static uint8_t get_rb(Z8000CPU *cpu, int idx) {
    if (idx < 8) return (uint8_t)(cpu->r[idx] >> 8);
    return (uint8_t)(cpu->r[idx - 8] & 0xFF);
}

static void set_rb(Z8000CPU *cpu, int idx, uint8_t val) {
    if (idx < 8) cpu->r[idx] = (uint16_t)((cpu->r[idx] & 0x00FF) | (val << 8));
    else cpu->r[idx - 8] = (uint16_t)((cpu->r[idx - 8] & 0xFF00) | val);
}

// Register pairs RRn (n even): Rn is the high word, Rn+1 the low word.
static uint32_t get_rl(Z8000CPU *cpu, int idx) {
    idx &= 0xE;
    return ((uint32_t)cpu->r[idx] << 16) | cpu->r[idx + 1];
}

static void set_rl(Z8000CPU *cpu, int idx, uint32_t val) {
    idx &= 0xE;
    cpu->r[idx] = (uint16_t)(val >> 16);
    cpu->r[idx + 1] = (uint16_t)(val & 0xFFFF);
}

// ---------------------------------------------------------------------------
// Flag helpers
// ---------------------------------------------------------------------------

static void flag_set(Z8000CPU *cpu, uint16_t mask, int cond) {
    if (cond) cpu->fcw = (uint16_t)(cpu->fcw | mask);
    else cpu->fcw = (uint16_t)(cpu->fcw & ~mask);
}

static void set_zs_w(Z8000CPU *cpu, uint16_t v) {
    flag_set(cpu, FLAG_Z, v == 0);
    flag_set(cpu, FLAG_S, (v & 0x8000) != 0);
}

static void set_zs_b(Z8000CPU *cpu, uint8_t v) {
    flag_set(cpu, FLAG_Z, v == 0);
    flag_set(cpu, FLAG_S, (v & 0x80) != 0);
}

static int parity_even(uint8_t v) {
    v = (uint8_t)(v ^ (v >> 4));
    v = (uint8_t)(v ^ (v >> 2));
    v = (uint8_t)(v ^ (v >> 1));
    return (v & 1) == 0;
}

static uint16_t alu_add_w(Z8000CPU *cpu, uint16_t a, uint16_t b, int cin) {
    uint32_t r = (uint32_t)a + b + (uint32_t)cin;
    uint16_t res = (uint16_t)r;
    flag_set(cpu, FLAG_C, r > 0xFFFF);
    flag_set(cpu, FLAG_P, ((a ^ res) & (b ^ res) & 0x8000) != 0);
    set_zs_w(cpu, res);
    return res;
}

static uint8_t alu_add_b(Z8000CPU *cpu, uint8_t a, uint8_t b, int cin) {
    uint32_t r = (uint32_t)a + b + (uint32_t)cin;
    uint8_t res = (uint8_t)r;
    flag_set(cpu, FLAG_C, r > 0xFF);
    flag_set(cpu, FLAG_P, ((a ^ res) & (b ^ res) & 0x80) != 0);
    flag_set(cpu, FLAG_H, ((a & 0xF) + (b & 0xF) + cin) > 0xF);
    flag_set(cpu, FLAG_D, 0);
    set_zs_b(cpu, res);
    return res;
}

static uint16_t alu_sub_w(Z8000CPU *cpu, uint16_t a, uint16_t b, int cin) {
    uint32_t r = (uint32_t)a - b - (uint32_t)cin;
    uint16_t res = (uint16_t)r;
    flag_set(cpu, FLAG_C, (uint32_t)b + (uint32_t)cin > a);
    flag_set(cpu, FLAG_P, ((a ^ b) & (a ^ res) & 0x8000) != 0);
    set_zs_w(cpu, res);
    return res;
}

static uint8_t alu_sub_b(Z8000CPU *cpu, uint8_t a, uint8_t b, int cin) {
    uint32_t r = (uint32_t)a - b - (uint32_t)cin;
    uint8_t res = (uint8_t)r;
    flag_set(cpu, FLAG_C, (uint32_t)b + (uint32_t)cin > a);
    flag_set(cpu, FLAG_P, ((a ^ b) & (a ^ res) & 0x80) != 0);
    flag_set(cpu, FLAG_H, ((uint32_t)(b & 0xF) + (uint32_t)cin) > (uint32_t)(a & 0xF));
    flag_set(cpu, FLAG_D, 1);
    set_zs_b(cpu, res);
    return res;
}

// Condition code evaluation (4-bit cc field)
static int cond_true(Z8000CPU *cpu, int cc) {
    int c = (cpu->fcw & FLAG_C) != 0;
    int z = (cpu->fcw & FLAG_Z) != 0;
    int s = (cpu->fcw & FLAG_S) != 0;
    int v = (cpu->fcw & FLAG_P) != 0;
    switch (cc & 0xF) {
        case 0x0: return 0;             // F (never)
        case 0x1: return s ^ v;         // LT
        case 0x2: return z | (s ^ v);   // LE
        case 0x3: return c | z;         // ULE
        case 0x4: return v;             // OV / PE
        case 0x5: return s;             // MI
        case 0x6: return z;             // Z / EQ
        case 0x7: return c;             // C / ULT
        case 0x8: return 1;             // always
        case 0x9: return !(s ^ v);      // GE
        case 0xA: return !(z | (s ^ v)); // GT
        case 0xB: return !(c | z);      // UGT
        case 0xC: return !v;            // NOV / PO
        case 0xD: return !s;            // PL
        case 0xE: return !z;            // NZ / NE
        default:  return !c;            // NC / UGE
    }
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

// Fetch the source operand for a two-operand instruction.
// mode: 0 = IR/IM, 1 = DA/X, 2 = R. Returns byte ops in the low 8 bits.
static uint16_t fetch_src(Z8000CPU *cpu, int mode, int f1, int word) {
    if (mode == 2) {
        return word ? cpu->r[f1] : get_rb(cpu, f1);
    }
    if (mode == 0) {
        if (f1 == 0) {  // Immediate (byte immediates repeat the value twice)
            uint16_t imm = fetch16(cpu);
            return word ? imm : (uint16_t)(imm & 0xFF);
        }
        uint16_t a = cpu->r[f1];
        return word ? read16(cpu, a) : read8(cpu, a);
    }
    // Direct address / indexed
    {
        uint16_t a = fetch16(cpu);
        if (f1) a = (uint16_t)(a + cpu->r[f1]);
        return word ? read16(cpu, a) : read8(cpu, a);
    }
}

static int exec_op(Z8000CPU *cpu, uint16_t op) {
    uint8_t hb = (uint8_t)(op >> 8);
    int f1 = (op >> 4) & 0xF;   // src / pointer / dst field (bits 7-4)
    int f2 = op & 0xF;          // dst / count / cc field (bits 3-0)

    if (op == 0x7A00) {         // HALT
        cpu->halted = 1;
        return 1;
    }
    if (op == 0x8D07) {         // NOP
        return 0;
    }

    if ((hb & 0xF0) == 0xC0) {  // LDB Rbd,#imm8 (one-word short form)
        set_rb(cpu, hb & 0xF, (uint8_t)(op & 0xFF));
        return 0;
    }
    if ((hb & 0xF0) == 0xD0) {  // CALR disp12 (target = PC - 2*disp)
        int16_t disp = (int16_t)((int16_t)((op & 0x0FFF) << 4) >> 4);
        uint16_t target = (uint16_t)(cpu->pc - 2 * disp);
        push16(cpu, cpu->pc);
        cpu->pc = target;
        return 0;
    }
    if ((hb & 0xF0) == 0xE0) {  // JR cc,disp8 (word displacement)
        if (cond_true(cpu, hb & 0xF)) {
            cpu->pc = (uint16_t)(cpu->pc + 2 * (int8_t)(op & 0xFF));
        }
        return 0;
    }

    // Two-operand ALU family: ADD/SUB/OR/AND/XOR/CP, byte and word,
    // in IR/IM (0x00-0x0B), DA/X (0x40-0x4B) and R (0x80-0x8B) modes.
    {
        int mode = hb >> 6;
        int op6 = hb & 0x3F;
        if (op6 <= 0x0B && mode <= 2) {
            int word = op6 & 1;
            int kind = op6 >> 1;
            uint16_t src = fetch_src(cpu, mode, f1, word);
            uint16_t dst = word ? cpu->r[f2] : get_rb(cpu, f2);
            uint16_t res = 0;
            int writeback = 1;
            switch (kind) {
                case 0: // ADD
                    res = word ? alu_add_w(cpu, dst, src, 0)
                               : alu_add_b(cpu, (uint8_t)dst, (uint8_t)src, 0);
                    break;
                case 1: // SUB
                    res = word ? alu_sub_w(cpu, dst, src, 0)
                               : alu_sub_b(cpu, (uint8_t)dst, (uint8_t)src, 0);
                    break;
                case 2: // OR
                    res = (uint16_t)(dst | src);
                    goto logic_flags;
                case 3: // AND
                    res = (uint16_t)(dst & src);
                    goto logic_flags;
                case 4: // XOR
                    res = (uint16_t)(dst ^ src);
                logic_flags:
                    if (word) set_zs_w(cpu, res);
                    else {
                        set_zs_b(cpu, (uint8_t)res);
                        flag_set(cpu, FLAG_P, parity_even((uint8_t)res));
                    }
                    break;
                default: // CP
                    if (word) alu_sub_w(cpu, dst, src, 0);
                    else alu_sub_b(cpu, (uint8_t)dst, (uint8_t)src, 0);
                    writeback = 0;
                    break;
            }
            if (writeback) {
                if (word) cpu->r[f2] = res;
                else set_rb(cpu, f2, (uint8_t)res);
            }
            return 0;
        }
    }

    switch (hb) {
        // --- LD/LDB/LDL loads ---
        case 0xA0: set_rb(cpu, f2, get_rb(cpu, f1)); return 0;      // LDB Rbd,Rbs
        case 0xA1: cpu->r[f2] = cpu->r[f1]; return 0;               // LD Rd,Rs
        case 0x94: set_rl(cpu, f2, get_rl(cpu, f1)); return 0;      // LDL RRd,RRs
        case 0x20:                                                  // LDB Rbd,@Rs / #imm
            set_rb(cpu, f2, (uint8_t)fetch_src(cpu, 0, f1, 0));
            return 0;
        case 0x21:                                                  // LD Rd,@Rs / #imm
            cpu->r[f2] = fetch_src(cpu, 0, f1, 1);
            return 0;
        case 0x14: {                                                // LDL RRd,@Rs / #imm32
            uint32_t v;
            if (f1 == 0) {
                v = (uint32_t)fetch16(cpu) << 16;
                v |= fetch16(cpu);
            } else {
                v = read32(cpu, cpu->r[f1]);
            }
            set_rl(cpu, f2, v);
            return 0;
        }
        case 0x60:                                                  // LDB Rbd,addr(Rx)
            set_rb(cpu, f2, (uint8_t)fetch_src(cpu, 1, f1, 0));
            return 0;
        case 0x61:                                                  // LD Rd,addr(Rx)
            cpu->r[f2] = fetch_src(cpu, 1, f1, 1);
            return 0;
        case 0x54: {                                                // LDL RRd,addr(Rx)
            uint16_t a = fetch16(cpu);
            if (f1) a = (uint16_t)(a + cpu->r[f1]);
            set_rl(cpu, f2, read32(cpu, a));
            return 0;
        }

        // --- LD/LDB/LDL stores ---
        case 0x2E: write8(cpu, cpu->r[f1], get_rb(cpu, f2)); return 0;   // LDB @Rd,Rbs
        case 0x2F: write16(cpu, cpu->r[f1], cpu->r[f2]); return 0;       // LD @Rd,Rs
        case 0x1D: write32(cpu, cpu->r[f1], get_rl(cpu, f2)); return 0;  // LDL @Rd,RRs
        case 0x6E: case 0x6F: case 0x5D: {                               // stores to addr(Rx)
            uint16_t a = fetch16(cpu);
            if (f1) a = (uint16_t)(a + cpu->r[f1]);
            if (hb == 0x6E) write8(cpu, a, get_rb(cpu, f2));
            else if (hb == 0x6F) write16(cpu, a, cpu->r[f2]);
            else write32(cpu, a, get_rl(cpu, f2));
            return 0;
        }

        // --- ADC/SBC (register mode only) ---
        case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            int word = hb & 1;
            int cin = (cpu->fcw & FLAG_C) ? 1 : 0;
            int sub = hb >= 0xB6;
            uint16_t src = word ? cpu->r[f1] : get_rb(cpu, f1);
            uint16_t dst = word ? cpu->r[f2] : get_rb(cpu, f2);
            uint16_t res;
            if (word) res = sub ? alu_sub_w(cpu, dst, src, cin)
                                : alu_add_w(cpu, dst, src, cin);
            else res = sub ? alu_sub_b(cpu, (uint8_t)dst, (uint8_t)src, cin)
                           : alu_add_b(cpu, (uint8_t)dst, (uint8_t)src, cin);
            if (word) cpu->r[f2] = res;
            else set_rb(cpu, f2, (uint8_t)res);
            return 0;
        }

        // --- INC/DEC (R, IR and DA/X modes) ---
        case 0xA8: case 0xA9: case 0xAA: case 0xAB:
        case 0x28: case 0x29: case 0x2A: case 0x2B:
        case 0x68: case 0x69: case 0x6A: case 0x6B: {
            int word = hb & 1;
            int dec = (hb & 0x02) != 0;
            int mode = hb >> 6;     // 0 = IR, 1 = DA/X, 2 = R
            uint16_t n = (uint16_t)(f2 + 1);
            uint16_t addr = 0, v, res;
            if (mode == 2) {
                v = word ? cpu->r[f1] : get_rb(cpu, f1);
            } else if (mode == 0) {
                if (f1 == 0) break; // immediate form does not exist here
                addr = cpu->r[f1];
                v = word ? read16(cpu, addr) : read8(cpu, addr);
            } else {
                addr = fetch16(cpu);
                if (f1) addr = (uint16_t)(addr + cpu->r[f1]);
                v = word ? read16(cpu, addr) : read8(cpu, addr);
            }
            res = dec ? (uint16_t)(v - n) : (uint16_t)(v + n);
            if (word) {
                flag_set(cpu, FLAG_P, dec ? (((v ^ n) & (v ^ res) & 0x8000) != 0)
                                          : (((v ^ res) & (n ^ res) & 0x8000) != 0));
                set_zs_w(cpu, res);
                if (mode == 2) cpu->r[f1] = res;
                else write16(cpu, addr, res);
            } else {
                res &= 0xFF;
                flag_set(cpu, FLAG_P, dec ? (((v ^ n) & (v ^ res) & 0x80) != 0)
                                          : (((v ^ res) & (n ^ res) & 0x80) != 0));
                set_zs_b(cpu, (uint8_t)res);
                if (mode == 2) set_rb(cpu, f1, (uint8_t)res);
                else write8(cpu, addr, (uint8_t)res);
            }
            return 0;
        }

        // --- MULT / DIV (register mode) ---
        case 0x99: {                                                // MULT RRd,Rs
            int32_t prod = (int32_t)(int16_t)cpu->r[(f2 & 0xE) | 1]
                         * (int32_t)(int16_t)cpu->r[f1];
            set_rl(cpu, f2, (uint32_t)prod);
            flag_set(cpu, FLAG_C, prod < -32768 || prod > 32767);
            flag_set(cpu, FLAG_Z, prod == 0);
            flag_set(cpu, FLAG_S, prod < 0);
            flag_set(cpu, FLAG_P, 0);
            return 0;
        }
        case 0x9B: {                                                // DIV RRd,Rs
            int32_t num = (int32_t)get_rl(cpu, f2);
            int16_t den = (int16_t)cpu->r[f1];
            if (den == 0) {
                flag_set(cpu, FLAG_P, 1);
                flag_set(cpu, FLAG_C, 1);
                return 0;
            }
            {
                int64_t q = (int64_t)num / den;
                int64_t rem = (int64_t)num % den;
                if (q < -32768 || q > 32767) {
                    flag_set(cpu, FLAG_P, 1);
                    flag_set(cpu, FLAG_C, 1);
                    return 0;
                }
                cpu->r[(f2 & 0xE) | 1] = (uint16_t)((uint64_t)q & 0xFFFF);
                cpu->r[f2 & 0xE] = (uint16_t)((uint64_t)rem & 0xFFFF);
                flag_set(cpu, FLAG_Z, q == 0);
                flag_set(cpu, FLAG_S, q < 0);
                flag_set(cpu, FLAG_P, 0);
                flag_set(cpu, FLAG_C, 0);
            }
            return 0;
        }

        // --- Shifts and rotates (register mode; dst in bits 7-4) ---
        case 0xB2: case 0xB3: {
            int word = hb & 1;
            int bits = word ? 16 : 8;
            uint16_t vmask = word ? 0xFFFF : 0xFF;
            uint16_t smask = word ? 0x8000 : 0x80;
            uint16_t v = word ? cpu->r[f1] : get_rb(cpu, f1);
            uint16_t res;
            int c;
            if (f2 == 1 || f2 == 9) {   // SLL/SRL (1) or SLA/SRA (9), count word
                int16_t cnt = (int16_t)fetch16(cpu);
                int arith = (f2 == 9);
                int right = cnt < 0;
                int n = right ? -cnt : cnt;
                if (n == 0) return 0;
                if (n > bits) n = bits;
                if (!right) {
                    c = (v >> (bits - n)) & 1;
                    res = (uint16_t)(((uint32_t)v << n) & vmask);
                } else {
                    c = (v >> (n - 1)) & 1;
                    if (arith) {
                        int32_t sv = word ? (int16_t)v : (int8_t)v;
                        int sh = (n >= bits) ? (bits - 1) : n;
                        res = (uint16_t)((uint32_t)(sv >> sh) & vmask);
                    } else {
                        res = (n >= bits) ? 0 : (uint16_t)(v >> n);
                    }
                }
                flag_set(cpu, FLAG_C, c);
                flag_set(cpu, FLAG_P, (arith && !right) ? (((res ^ v) & smask) != 0) : 0);
            } else if (f2 == 0 || f2 == 2 || f2 == 4 || f2 == 6) {  // RL/RR #1/#2
                int n = (f2 & 2) ? 2 : 1;
                if (f2 < 4) {           // RL
                    c = (v >> (bits - n)) & 1;
                    res = (uint16_t)((((uint32_t)v << n) | (v >> (bits - n))) & vmask);
                } else {                // RR
                    c = (v >> (n - 1)) & 1;
                    res = (uint16_t)(((v >> n) | ((uint32_t)v << (bits - n))) & vmask);
                }
                flag_set(cpu, FLAG_C, c);
                flag_set(cpu, FLAG_P, 0);
            } else {
                break;                  // RLC/RRC/dynamic shifts not implemented
            }
            if (word) {
                set_zs_w(cpu, res);
                cpu->r[f1] = res;
            } else {
                set_zs_b(cpu, (uint8_t)res);
                set_rb(cpu, f1, (uint8_t)res);
            }
            return 0;
        }

        // --- CLR/COM/NEG/TEST (register mode; dst in bits 7-4) ---
        case 0x8C: case 0x8D: {
            int word = hb & 1;
            uint16_t v = word ? cpu->r[f1] : get_rb(cpu, f1);
            uint16_t res = v;
            int writeback = 1;
            switch (f2) {
                case 0x0:   // COM
                    res = (uint16_t)(~v & (word ? 0xFFFF : 0xFF));
                    break;
                case 0x2:   // NEG
                    res = (uint16_t)((0 - v) & (word ? 0xFFFF : 0xFF));
                    flag_set(cpu, FLAG_C, v != 0);
                    flag_set(cpu, FLAG_P, v == (word ? 0x8000 : 0x80));
                    break;
                case 0x4:   // TEST
                    writeback = 0;
                    break;
                case 0x8:   // CLR (does not affect flags)
                    res = 0;
                    if (word) cpu->r[f1] = 0;
                    else set_rb(cpu, f1, 0);
                    return 0;
                default:
                    goto illegal;
            }
            if (word) set_zs_w(cpu, res);
            else {
                set_zs_b(cpu, (uint8_t)res);
                if (f2 == 0x0 || f2 == 0x4)
                    flag_set(cpu, FLAG_P, parity_even((uint8_t)res));
            }
            if (writeback) {
                if (word) cpu->r[f1] = res;
                else set_rb(cpu, f1, (uint8_t)res);
            }
            return 0;
        }

        // --- EX ---
        case 0xAC: {                                                // EXB Rbd,Rbs
            uint8_t t = get_rb(cpu, f2);
            set_rb(cpu, f2, get_rb(cpu, f1));
            set_rb(cpu, f1, t);
            return 0;
        }
        case 0xAD: {                                                // EX Rd,Rs
            uint16_t t = cpu->r[f2];
            cpu->r[f2] = cpu->r[f1];
            cpu->r[f1] = t;
            return 0;
        }

        // --- LDK ---
        case 0xBD:                                                  // LDK Rd,#nibble
            cpu->r[f1] = (uint16_t)f2;
            return 0;

        // --- Jumps / calls / returns ---
        case 0x1E:                                                  // JP cc,@Rd
            if (f1 == 0) break;
            if (cond_true(cpu, f2)) cpu->pc = cpu->r[f1];
            return 0;
        case 0x5E: {                                                // JP cc,addr(Rx)
            uint16_t a = fetch16(cpu);
            if (f1) a = (uint16_t)(a + cpu->r[f1]);
            if (cond_true(cpu, f2)) cpu->pc = a;
            return 0;
        }
        case 0x1F:                                                  // CALL @Rd
            if (f1 == 0 || f2 != 0) break;
            push16(cpu, cpu->pc);
            cpu->pc = cpu->r[f1];
            return 0;
        case 0x5F: {                                                // CALL addr(Rx)
            uint16_t a = fetch16(cpu);
            if (f1) a = (uint16_t)(a + cpu->r[f1]);
            push16(cpu, cpu->pc);
            cpu->pc = a;
            return 0;
        }
        case 0x9E:                                                  // RET cc
            if (cond_true(cpu, f2)) cpu->pc = pop16(cpu);
            return 0;

        // --- PUSH / POP ---
        case 0x93:                                                  // PUSH @Rd,Rs
            cpu->r[f1] = (uint16_t)(cpu->r[f1] - 2);
            write16(cpu, cpu->r[f1], cpu->r[f2]);
            return 0;
        case 0x97: {                                                // POP Rd,@Rs
            uint16_t v = read16(cpu, cpu->r[f1]);
            cpu->r[f1] = (uint16_t)(cpu->r[f1] + 2);
            cpu->r[f2] = v;
            return 0;
        }

        default:
            break;
    }

illegal:
    cpu->halted = 1;
    return -2;
}

int z8000_step(void *context) {
    if (!context) return -1;
    Z8000CPU *cpu = (Z8000CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint16_t op = fetch16(cpu);
    cpu->ticks++;

    int result = exec_op(cpu, op);
    if (result != 0) return result;

    // Self-loop (e.g. JR $) interpreted as a software halt, matching other cores.
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// State display
// ---------------------------------------------------------------------------

void z8000_print_state(void *context) {
    if (!context) return;
    Z8000CPU *cpu = (Z8000CPU*)context;

    printf("Z8000 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  FCW: 0x%04X [%c%c%c%c%c%c]  Halted: %s\n",
           cpu->pc, cpu->fcw,
           (cpu->fcw & FLAG_C) ? 'C' : '-',
           (cpu->fcw & FLAG_Z) ? 'Z' : '-',
           (cpu->fcw & FLAG_S) ? 'S' : '-',
           (cpu->fcw & FLAG_P) ? 'P' : '-',
           (cpu->fcw & FLAG_D) ? 'D' : '-',
           (cpu->fcw & FLAG_H) ? 'H' : '-',
           cpu->halted ? "Yes" : "No");
    printf("  Registers:\n");
    for (int row = 0; row < 4; ++row) {
        printf("   ");
        for (int col = 0; col < 4; ++col) {
            int r = row * 4 + col;
            printf(" R%-2d: 0x%04X", r, cpu->r[r]);
        }
        printf("\n");
    }
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

static uint16_t dis_fetch16(Z8000CPU *cpu, uint16_t *p) {
    uint16_t v = read16(cpu, *p);
    *p = (uint16_t)(*p + 2);
    return v;
}

static void breg_name(int idx, char *out, size_t len) {
    if (idx < 8) snprintf(out, len, "RH%d", idx);
    else snprintf(out, len, "RL%d", idx - 8);
}

static const char* cc_name(int cc) {
    static const char *names[16] = {
        "F", "LT", "LE", "ULE", "OV", "MI", "Z", "C",
        "", "GE", "GT", "UGT", "NOV", "PL", "NZ", "NC"
    };
    return names[cc & 0xF];
}

// Format the source operand of a two-operand instruction (see fetch_src).
static void dis_src(Z8000CPU *cpu, uint16_t *p, int mode, int f1, int word,
                    char *out, size_t len) {
    if (mode == 2) {
        if (word) snprintf(out, len, "R%d", f1);
        else breg_name(f1, out, len);
    } else if (mode == 0) {
        if (f1 == 0) {
            uint16_t imm = dis_fetch16(cpu, p);
            if (word) snprintf(out, len, "#0x%04X", imm);
            else snprintf(out, len, "#0x%02X", imm & 0xFF);
        } else {
            snprintf(out, len, "@R%d", f1);
        }
    } else {
        uint16_t a = dis_fetch16(cpu, p);
        if (f1) snprintf(out, len, "0x%04X(R%d)", a, f1);
        else snprintf(out, len, "0x%04X", a);
    }
}

void z8000_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    Z8000CPU *cpu = (Z8000CPU*)context;

    uint16_t p = cpu->pc;
    uint16_t op = dis_fetch16(cpu, &p);
    uint8_t hb = (uint8_t)(op >> 8);
    int f1 = (op >> 4) & 0xF;
    int f2 = op & 0xF;
    char src[32], dst[32];

    if (op == 0x7A00) { snprintf(buf, buf_len, "HALT"); return; }
    if (op == 0x8D07) { snprintf(buf, buf_len, "NOP"); return; }

    if ((hb & 0xF0) == 0xC0) {
        breg_name(hb & 0xF, dst, sizeof(dst));
        snprintf(buf, buf_len, "LDB %s, #0x%02X", dst, op & 0xFF);
        return;
    }
    if ((hb & 0xF0) == 0xD0) {
        int16_t disp = (int16_t)((int16_t)((op & 0x0FFF) << 4) >> 4);
        snprintf(buf, buf_len, "CALR 0x%04X", (uint16_t)(p - 2 * disp));
        return;
    }
    if ((hb & 0xF0) == 0xE0) {
        uint16_t target = (uint16_t)(p + 2 * (int8_t)(op & 0xFF));
        if ((hb & 0xF) == 8) snprintf(buf, buf_len, "JR 0x%04X", target);
        else snprintf(buf, buf_len, "JR %s, 0x%04X", cc_name(hb & 0xF), target);
        return;
    }

    // Two-operand ALU family
    {
        int mode = hb >> 6;
        int op6 = hb & 0x3F;
        if (op6 <= 0x0B && mode <= 2) {
            static const char *word_ops[6] = {"ADD", "SUB", "OR", "AND", "XOR", "CP"};
            static const char *byte_ops[6] = {"ADDB", "SUBB", "ORB", "ANDB", "XORB", "CPB"};
            int word = op6 & 1;
            int kind = op6 >> 1;
            dis_src(cpu, &p, mode, f1, word, src, sizeof(src));
            if (word) snprintf(dst, sizeof(dst), "R%d", f2);
            else breg_name(f2, dst, sizeof(dst));
            snprintf(buf, buf_len, "%s %s, %s",
                     word ? word_ops[kind] : byte_ops[kind], dst, src);
            return;
        }
    }

    switch (hb) {
        case 0xA0:
            breg_name(f1, src, sizeof(src));
            breg_name(f2, dst, sizeof(dst));
            snprintf(buf, buf_len, "LDB %s, %s", dst, src);
            return;
        case 0xA1:
            snprintf(buf, buf_len, "LD R%d, R%d", f2, f1);
            return;
        case 0x94:
            snprintf(buf, buf_len, "LDL RR%d, RR%d", f2 & 0xE, f1 & 0xE);
            return;
        case 0x20:
            dis_src(cpu, &p, 0, f1, 0, src, sizeof(src));
            breg_name(f2, dst, sizeof(dst));
            snprintf(buf, buf_len, "LDB %s, %s", dst, src);
            return;
        case 0x21:
            dis_src(cpu, &p, 0, f1, 1, src, sizeof(src));
            snprintf(buf, buf_len, "LD R%d, %s", f2, src);
            return;
        case 0x14:
            if (f1 == 0) {
                uint32_t imm = (uint32_t)dis_fetch16(cpu, &p) << 16;
                imm |= dis_fetch16(cpu, &p);
                snprintf(buf, buf_len, "LDL RR%d, #0x%08X", f2 & 0xE, imm);
            } else {
                snprintf(buf, buf_len, "LDL RR%d, @R%d", f2 & 0xE, f1);
            }
            return;
        case 0x60:
            dis_src(cpu, &p, 1, f1, 0, src, sizeof(src));
            breg_name(f2, dst, sizeof(dst));
            snprintf(buf, buf_len, "LDB %s, %s", dst, src);
            return;
        case 0x61:
            dis_src(cpu, &p, 1, f1, 1, src, sizeof(src));
            snprintf(buf, buf_len, "LD R%d, %s", f2, src);
            return;
        case 0x54:
            dis_src(cpu, &p, 1, f1, 1, src, sizeof(src));
            snprintf(buf, buf_len, "LDL RR%d, %s", f2 & 0xE, src);
            return;
        case 0x2E:
            breg_name(f2, src, sizeof(src));
            snprintf(buf, buf_len, "LDB @R%d, %s", f1, src);
            return;
        case 0x2F:
            snprintf(buf, buf_len, "LD @R%d, R%d", f1, f2);
            return;
        case 0x1D:
            snprintf(buf, buf_len, "LDL @R%d, RR%d", f1, f2 & 0xE);
            return;
        case 0x6E: case 0x6F: case 0x5D:
            dis_src(cpu, &p, 1, f1, 1, dst, sizeof(dst));
            if (hb == 0x6E) {
                breg_name(f2, src, sizeof(src));
                snprintf(buf, buf_len, "LDB %s, %s", dst, src);
            } else if (hb == 0x6F) {
                snprintf(buf, buf_len, "LD %s, R%d", dst, f2);
            } else {
                snprintf(buf, buf_len, "LDL %s, RR%d", dst, f2 & 0xE);
            }
            return;

        case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            static const char *ops[4] = {"ADCB", "ADC", "SBCB", "SBC"};
            int word = hb & 1;
            if (word) {
                snprintf(buf, buf_len, "%s R%d, R%d", ops[hb - 0xB4], f2, f1);
            } else {
                breg_name(f1, src, sizeof(src));
                breg_name(f2, dst, sizeof(dst));
                snprintf(buf, buf_len, "%s %s, %s", ops[hb - 0xB4], dst, src);
            }
            return;
        }

        case 0xA8: case 0xA9: case 0xAA: case 0xAB:
        case 0x28: case 0x29: case 0x2A: case 0x2B:
        case 0x68: case 0x69: case 0x6A: case 0x6B: {
            int word = hb & 1;
            int dec = (hb & 0x02) != 0;
            int mode = hb >> 6;
            const char *mn = dec ? (word ? "DEC" : "DECB") : (word ? "INC" : "INCB");
            if (mode == 2) {
                if (word) snprintf(dst, sizeof(dst), "R%d", f1);
                else breg_name(f1, dst, sizeof(dst));
            } else if (mode == 0) {
                if (f1 == 0) break;
                snprintf(dst, sizeof(dst), "@R%d", f1);
            } else {
                dis_src(cpu, &p, 1, f1, 1, dst, sizeof(dst));
            }
            snprintf(buf, buf_len, "%s %s, #%d", mn, dst, f2 + 1);
            return;
        }

        case 0x99:
            snprintf(buf, buf_len, "MULT RR%d, R%d", f2 & 0xE, f1);
            return;
        case 0x9B:
            snprintf(buf, buf_len, "DIV RR%d, R%d", f2 & 0xE, f1);
            return;

        case 0xB2: case 0xB3: {
            int word = hb & 1;
            if (word) snprintf(dst, sizeof(dst), "R%d", f1);
            else breg_name(f1, dst, sizeof(dst));
            if (f2 == 1 || f2 == 9) {
                int16_t cnt = (int16_t)dis_fetch16(cpu, &p);
                int arith = (f2 == 9);
                const char *mn;
                int n = cnt < 0 ? -cnt : cnt;
                if (arith) mn = (cnt < 0) ? (word ? "SRA" : "SRAB") : (word ? "SLA" : "SLAB");
                else mn = (cnt < 0) ? (word ? "SRL" : "SRLB") : (word ? "SLL" : "SLLB");
                snprintf(buf, buf_len, "%s %s, #%d", mn, dst, n);
                return;
            }
            if (f2 == 0 || f2 == 2 || f2 == 4 || f2 == 6) {
                const char *mn = (f2 < 4) ? (word ? "RL" : "RLB") : (word ? "RR" : "RRB");
                snprintf(buf, buf_len, "%s %s, #%d", mn, dst, (f2 & 2) ? 2 : 1);
                return;
            }
            break;
        }

        case 0x8C: case 0x8D: {
            int word = hb & 1;
            const char *mn;
            if (f2 == 0x0) mn = word ? "COM" : "COMB";
            else if (f2 == 0x2) mn = word ? "NEG" : "NEGB";
            else if (f2 == 0x4) mn = word ? "TEST" : "TESTB";
            else if (f2 == 0x8) mn = word ? "CLR" : "CLRB";
            else break;
            if (word) snprintf(dst, sizeof(dst), "R%d", f1);
            else breg_name(f1, dst, sizeof(dst));
            snprintf(buf, buf_len, "%s %s", mn, dst);
            return;
        }

        case 0xAC:
            breg_name(f1, src, sizeof(src));
            breg_name(f2, dst, sizeof(dst));
            snprintf(buf, buf_len, "EXB %s, %s", dst, src);
            return;
        case 0xAD:
            snprintf(buf, buf_len, "EX R%d, R%d", f2, f1);
            return;

        case 0xBD:
            snprintf(buf, buf_len, "LDK R%d, #%d", f1, f2);
            return;

        case 0x1E:
            if (f1 == 0) break;
            if (f2 == 8) snprintf(buf, buf_len, "JP @R%d", f1);
            else snprintf(buf, buf_len, "JP %s, @R%d", cc_name(f2), f1);
            return;
        case 0x5E:
            dis_src(cpu, &p, 1, f1, 1, dst, sizeof(dst));
            if (f2 == 8) snprintf(buf, buf_len, "JP %s", dst);
            else snprintf(buf, buf_len, "JP %s, %s", cc_name(f2), dst);
            return;
        case 0x1F:
            if (f1 == 0 || f2 != 0) break;
            snprintf(buf, buf_len, "CALL @R%d", f1);
            return;
        case 0x5F:
            dis_src(cpu, &p, 1, f1, 1, dst, sizeof(dst));
            snprintf(buf, buf_len, "CALL %s", dst);
            return;
        case 0x9E:
            if (f2 == 8) snprintf(buf, buf_len, "RET");
            else snprintf(buf, buf_len, "RET %s", cc_name(f2));
            return;

        case 0x93:
            snprintf(buf, buf_len, "PUSH @R%d, R%d", f1, f2);
            return;
        case 0x97:
            snprintf(buf, buf_len, "POP R%d, @R%d", f2, f1);
            return;

        default:
            break;
    }

    snprintf(buf, buf_len, ".word 0x%04X", op);
}
