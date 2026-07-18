// Motorola MC68HC11 8-bit MCU CPU emulator core.
//
// Implements the full 6800-style base instruction set plus the HC11 additions:
//   - Accumulators A and B (pairable as 16-bit D), index registers X and Y,
//     16-bit stack pointer, program counter
//   - Y-register forms via the 0x18 prefix; page 3 (0x1A) and page 4 (0x1D
//     era 0xCD) prefixes for CPD and cross-index CPX/CPY/LDX/LDY/STX/STY
//   - MUL, IDIV, FDIV, XGDX/XGDY, ABX/ABY, LSRD/ASLD
//   - LDD/STD/ADDD/SUBD/CPD 16-bit accumulator operations
//   - BSET/BCLR/BRSET/BRCLR bit manipulation (direct and indexed)
//   - PSHX/PULX/PSHY/PULY, INY/DEY, TSY/TYS
//   - Condition code register flags: S X H I N Z V C
// STOP, WAI and any undocumented/invalid opcode halt the CPU (step returns 1).

#include "mc68hc11.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_C 0x01
#define FLAG_V 0x02
#define FLAG_Z 0x04
#define FLAG_N 0x08
#define FLAG_I 0x10
#define FLAG_H 0x20
#define FLAG_X 0x40 // XIRQ mask (can be cleared by TAP, never set)
#define FLAG_S 0x80 // Stop disable

typedef struct MC68HC11_CPU {
    uint8_t ram[65536];
    uint8_t a;      // Accumulator A (high byte of D)
    uint8_t b;      // Accumulator B (low byte of D)
    uint16_t x;     // Index register X
    uint16_t y;     // Index register Y
    uint16_t sp;    // Stack pointer
    uint16_t pc;    // Program counter
    uint8_t ccr;    // Condition code register (SXHINZVC)
    uint32_t ticks;
    int halted;
} MC68HC11_CPU;

#define SET_FLAG(flag, cond) do { if (cond) cpu->ccr |= (flag); else cpu->ccr &= (uint8_t)~(flag); } while(0)
#define GET_FLAG(flag) ((cpu->ccr & (flag)) ? 1 : 0)

#define GET_D() ((uint16_t)(((uint16_t)cpu->a << 8) | cpu->b))
#define SET_D(v) do { cpu->a = (uint8_t)((uint16_t)(v) >> 8); cpu->b = (uint8_t)((v) & 0xFF); } while(0)

// --- Memory helpers ---

static uint8_t mem_read(MC68HC11_CPU *cpu, uint16_t addr) {
    return cpu->ram[addr];
}

static void mem_write(MC68HC11_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[addr] = val;
}

static uint8_t fetch8(MC68HC11_CPU *cpu) {
    return mem_read(cpu, cpu->pc++);
}

static uint16_t fetch16(MC68HC11_CPU *cpu) {
    uint16_t hi = mem_read(cpu, cpu->pc++);
    uint16_t lo = mem_read(cpu, cpu->pc++);
    return (uint16_t)((hi << 8) | lo);
}

static uint16_t mem_read16(MC68HC11_CPU *cpu, uint16_t addr) {
    return (uint16_t)(((uint16_t)mem_read(cpu, addr) << 8) | mem_read(cpu, (uint16_t)(addr + 1)));
}

static void mem_write16(MC68HC11_CPU *cpu, uint16_t addr, uint16_t val) {
    mem_write(cpu, addr, (uint8_t)(val >> 8));
    mem_write(cpu, (uint16_t)(addr + 1), (uint8_t)(val & 0xFF));
}

// Fetch the effective address for mode 1=direct, 2=indexed, 3=extended.
// ixbase is X, or Y when the instruction carries a Y-page prefix.
static uint16_t fetch_addr(MC68HC11_CPU *cpu, uint8_t mode, uint16_t ixbase) {
    if (mode == 1) return fetch8(cpu);
    if (mode == 2) return (uint16_t)(ixbase + fetch8(cpu));
    return fetch16(cpu);
}

// Stack: SP points at the next free byte; push stores then decrements.
static void push8(MC68HC11_CPU *cpu, uint8_t val) {
    mem_write(cpu, cpu->sp--, val);
}

static uint8_t pull8(MC68HC11_CPU *cpu) {
    return mem_read(cpu, ++cpu->sp);
}

// 16-bit values are pushed low byte first (JSR pushes PCL then PCH).
static void push16(MC68HC11_CPU *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val & 0xFF));
    push8(cpu, (uint8_t)(val >> 8));
}

static uint16_t pull16(MC68HC11_CPU *cpu) {
    uint16_t hi = pull8(cpu);
    uint16_t lo = pull8(cpu);
    return (uint16_t)((hi << 8) | lo);
}

// --- Flag helpers ---

static void set_nz8(MC68HC11_CPU *cpu, uint8_t val) {
    SET_FLAG(FLAG_N, val & 0x80);
    SET_FLAG(FLAG_Z, val == 0);
}

static void set_nz16(MC68HC11_CPU *cpu, uint16_t val) {
    SET_FLAG(FLAG_N, val & 0x8000);
    SET_FLAG(FLAG_Z, val == 0);
}

// --- ALU helpers ---

// 8-bit add with optional carry-in; sets H N Z V C, returns the result.
static uint8_t alu_add(MC68HC11_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t carry_in) {
    uint16_t sum = (uint16_t)lhs + rhs + carry_in;
    uint8_t res = (uint8_t)sum;
    SET_FLAG(FLAG_H, ((lhs & 0x0F) + (rhs & 0x0F) + carry_in) & 0x10);
    SET_FLAG(FLAG_C, sum & 0x100);
    SET_FLAG(FLAG_V, (~(lhs ^ rhs) & (lhs ^ res)) & 0x80);
    set_nz8(cpu, res);
    return res;
}

// 8-bit subtract with optional borrow-in; sets N Z V C, returns the result.
static uint8_t alu_sub(MC68HC11_CPU *cpu, uint8_t lhs, uint8_t rhs, uint8_t borrow_in) {
    uint16_t diff = (uint16_t)lhs - rhs - borrow_in;
    uint8_t res = (uint8_t)diff;
    SET_FLAG(FLAG_C, diff & 0x100);
    SET_FLAG(FLAG_V, ((lhs ^ rhs) & (lhs ^ res)) & 0x80);
    set_nz8(cpu, res);
    return res;
}

// 16-bit add; sets N Z V C (used by ADDD).
static uint16_t alu_add16(MC68HC11_CPU *cpu, uint16_t lhs, uint16_t rhs) {
    uint32_t sum = (uint32_t)lhs + rhs;
    uint16_t res = (uint16_t)sum;
    SET_FLAG(FLAG_C, sum & 0x10000);
    SET_FLAG(FLAG_V, (~(lhs ^ rhs) & (lhs ^ res)) & 0x8000);
    set_nz16(cpu, res);
    return res;
}

// 16-bit subtract; sets N Z V C (used by SUBD/CPD/CPX/CPY).
static uint16_t alu_sub16(MC68HC11_CPU *cpu, uint16_t lhs, uint16_t rhs) {
    uint32_t diff = (uint32_t)lhs - rhs;
    uint16_t res = (uint16_t)diff;
    SET_FLAG(FLAG_C, diff & 0x10000);
    SET_FLAG(FLAG_V, ((lhs ^ rhs) & (lhs ^ res)) & 0x8000);
    set_nz16(cpu, res);
    return res;
}

// Single-operand read-modify-write core (opcodes 0x40-0x7F).
// Returns the new value; writeback indicates a write-back is required.
static uint8_t alu_rmw(MC68HC11_CPU *cpu, uint8_t sub, uint8_t val, int *writeback) {
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
static int branch_taken(MC68HC11_CPU *cpu, uint8_t sub) {
    int c = GET_FLAG(FLAG_C);
    int z = GET_FLAG(FLAG_Z);
    int n = GET_FLAG(FLAG_N);
    int v = GET_FLAG(FLAG_V);
    switch (sub) {
        case 0x00: return 1;            // BRA
        case 0x01: return 0;            // BRN (branch never)
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
        default:   return z | (n ^ v);  // BLE
    }
}

// --- Lifecycle ---

void* mc68hc11_create(void) {
    MC68HC11_CPU *cpu = (MC68HC11_CPU*)calloc(1, sizeof(MC68HC11_CPU));
    return cpu;
}

void mc68hc11_destroy(void *context) {
    free(context);
}

int mc68hc11_init(void *context) {
    if (!context) return -1;
    MC68HC11_CPU *cpu = (MC68HC11_CPU*)context;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->b = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0x01FF;
    cpu->pc = 0;
    cpu->ccr = FLAG_S | FLAG_X | FLAG_I; // stop disabled, interrupts masked on reset
    cpu->ticks = 0;
    cpu->halted = 0;

    return 0;
}

int mc68hc11_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    MC68HC11_CPU *cpu = (MC68HC11_CPU*)context;

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

int mc68hc11_step(void *context) {
    if (!context) return -1;
    MC68HC11_CPU *cpu = (MC68HC11_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    int page = 1;
    cpu->ticks++;

    if (op == 0x18) { page = 2; op = fetch8(cpu); }        // Y-register page
    else if (op == 0x1A) { page = 3; op = fetch8(cpu); }   // CPD / cross-index (X)
    else if (op == 0xCD) { page = 4; op = fetch8(cpu); }   // CPD / cross-index (Y)

    // --- Page 3 (0x1A) and page 4 (0xCD) prefixed opcodes ---
    if (page == 3 || page == 4) {
        uint16_t base = (page == 3) ? cpu->x : cpu->y; // indexed base register
        switch (op) {
            case 0x83: // CPD immediate (page 3 only)
                if (page != 3) break;
                (void)alu_sub16(cpu, GET_D(), fetch16(cpu));
                return 0;
            case 0x93: // CPD direct (page 3 only)
                if (page != 3) break;
                (void)alu_sub16(cpu, GET_D(), mem_read16(cpu, fetch8(cpu)));
                return 0;
            case 0xA3: // CPD indexed (,X on page 3 / ,Y on page 4)
                (void)alu_sub16(cpu, GET_D(), mem_read16(cpu, (uint16_t)(base + fetch8(cpu))));
                return 0;
            case 0xB3: // CPD extended (page 3 only)
                if (page != 3) break;
                (void)alu_sub16(cpu, GET_D(), mem_read16(cpu, fetch16(cpu)));
                return 0;
            case 0xAC: // CPY ,X (page 3) / CPX ,Y (page 4)
                (void)alu_sub16(cpu, (page == 3) ? cpu->y : cpu->x,
                                mem_read16(cpu, (uint16_t)(base + fetch8(cpu))));
                return 0;
            case 0xEE: { // LDY ,X (page 3) / LDX ,Y (page 4)
                uint16_t m = mem_read16(cpu, (uint16_t)(base + fetch8(cpu)));
                if (page == 3) cpu->y = m; else cpu->x = m;
                set_nz16(cpu, m);
                SET_FLAG(FLAG_V, 0);
                return 0;
            }
            case 0xEF: { // STY ,X (page 3) / STX ,Y (page 4)
                uint16_t addr = (uint16_t)(base + fetch8(cpu));
                uint16_t v = (page == 3) ? cpu->y : cpu->x;
                mem_write16(cpu, addr, v);
                set_nz16(cpu, v);
                SET_FLAG(FLAG_V, 0);
                return 0;
            }
            default:
                break;
        }
        cpu->halted = 1;
        return 1;
    }

    // From here on: page 1 (X forms) or page 2 (0x18 prefix, Y forms)
    {
        int use_y = (page == 2);
        uint16_t ixb = use_y ? cpu->y : cpu->x; // base for indexed addressing

        // --- Inherent / bit-manipulation group 0x00-0x1F ---
        if (op < 0x20) {
            switch (op) {
                case 0x01: // NOP
                    break;
                case 0x02: { // IDIV (X = D / X, D = remainder)
                    uint16_t d = GET_D();
                    if (cpu->x == 0) {
                        cpu->x = 0xFFFF;
                        SET_FLAG(FLAG_C, 1);
                        SET_FLAG(FLAG_V, 0);
                        SET_FLAG(FLAG_Z, 0);
                    } else {
                        uint16_t q = (uint16_t)(d / cpu->x);
                        uint16_t r = (uint16_t)(d % cpu->x);
                        cpu->x = q;
                        SET_D(r);
                        SET_FLAG(FLAG_Z, q == 0);
                        SET_FLAG(FLAG_V, 0);
                        SET_FLAG(FLAG_C, 0);
                    }
                    break;
                }
                case 0x03: { // FDIV (X = (D:0000) / X fractional, D = remainder)
                    uint16_t d = GET_D();
                    if (cpu->x == 0) {
                        cpu->x = 0xFFFF;
                        SET_FLAG(FLAG_C, 1);
                        SET_FLAG(FLAG_V, 1);
                        SET_FLAG(FLAG_Z, 0);
                    } else if (cpu->x <= d) { // overflow: result would not fit
                        cpu->x = 0xFFFF;
                        SET_FLAG(FLAG_C, 0);
                        SET_FLAG(FLAG_V, 1);
                        SET_FLAG(FLAG_Z, 0);
                    } else {
                        uint32_t num = (uint32_t)d << 16;
                        uint16_t q = (uint16_t)(num / cpu->x);
                        uint16_t r = (uint16_t)(num % cpu->x);
                        cpu->x = q;
                        SET_D(r);
                        SET_FLAG(FLAG_Z, q == 0);
                        SET_FLAG(FLAG_V, 0);
                        SET_FLAG(FLAG_C, 0);
                    }
                    break;
                }
                case 0x04: { // LSRD
                    uint16_t d = GET_D();
                    SET_FLAG(FLAG_C, d & 0x0001);
                    d >>= 1;
                    SET_D(d);
                    set_nz16(cpu, d);
                    SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
                    break;
                }
                case 0x05: { // ASLD (LSLD)
                    uint16_t d = GET_D();
                    SET_FLAG(FLAG_C, d & 0x8000);
                    d = (uint16_t)(d << 1);
                    SET_D(d);
                    set_nz16(cpu, d);
                    SET_FLAG(FLAG_V, GET_FLAG(FLAG_N) ^ GET_FLAG(FLAG_C));
                    break;
                }
                case 0x06: { // TAP (A -> CCR; X bit can be cleared but not set)
                    uint8_t xbit = (uint8_t)(cpu->ccr & FLAG_X);
                    cpu->ccr = cpu->a;
                    if (!xbit) cpu->ccr &= (uint8_t)~FLAG_X;
                    break;
                }
                case 0x07: // TPA (CCR -> A)
                    cpu->a = cpu->ccr;
                    break;
                case 0x08: // INX / INY
                    if (use_y) { cpu->y++; SET_FLAG(FLAG_Z, cpu->y == 0); }
                    else { cpu->x++; SET_FLAG(FLAG_Z, cpu->x == 0); }
                    break;
                case 0x09: // DEX / DEY
                    if (use_y) { cpu->y--; SET_FLAG(FLAG_Z, cpu->y == 0); }
                    else { cpu->x--; SET_FLAG(FLAG_Z, cpu->x == 0); }
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
                case 0x12:   // BRSET direct
                case 0x13:   // BRCLR direct
                case 0x1E:   // BRSET indexed
                case 0x1F: { // BRCLR indexed
                    uint16_t addr;
                    uint8_t mask, m;
                    int8_t rel;
                    int taken;
                    if (op >= 0x1E) addr = (uint16_t)(ixb + fetch8(cpu));
                    else addr = fetch8(cpu);
                    mask = fetch8(cpu);
                    rel = (int8_t)fetch8(cpu);
                    m = mem_read(cpu, addr);
                    if (op == 0x12 || op == 0x1E) taken = ((uint8_t)~m & mask) == 0; // all set
                    else taken = (m & mask) == 0;                                    // all clear
                    if (taken) cpu->pc = (uint16_t)(cpu->pc + rel);
                    break;
                }
                case 0x14:   // BSET direct
                case 0x15:   // BCLR direct
                case 0x1C:   // BSET indexed
                case 0x1D: { // BCLR indexed
                    uint16_t addr;
                    uint8_t mask, res;
                    if (op >= 0x1C) addr = (uint16_t)(ixb + fetch8(cpu));
                    else addr = fetch8(cpu);
                    mask = fetch8(cpu);
                    res = mem_read(cpu, addr);
                    if (op == 0x14 || op == 0x1C) res |= mask;
                    else res &= (uint8_t)~mask;
                    mem_write(cpu, addr, res);
                    set_nz8(cpu, res);
                    SET_FLAG(FLAG_V, 0);
                    break;
                }
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
                case 0x19: { // DAA (decimal adjust A after BCD add)
                    uint8_t adjust = 0;
                    uint8_t hi = (uint8_t)(cpu->a >> 4);
                    uint8_t lo = (uint8_t)(cpu->a & 0x0F);
                    int carry = GET_FLAG(FLAG_C);
                    uint16_t res;
                    if (lo > 9 || GET_FLAG(FLAG_H)) adjust |= 0x06;
                    if (hi > 9 || carry || (hi == 9 && lo > 9)) adjust |= 0x60;
                    res = (uint16_t)(cpu->a + adjust);
                    cpu->a = (uint8_t)res;
                    SET_FLAG(FLAG_C, carry || (res & 0x100));
                    set_nz8(cpu, cpu->a);
                    break;
                }
                case 0x1B: // ABA (A = A + B)
                    cpu->a = alu_add(cpu, cpu->a, cpu->b, 0);
                    break;
                default: // 0x00 TEST and prefix-after-prefix are illegal here
                    cpu->halted = 1;
                    return 1;
            }
            return 0;
        }

        // --- Relative branches 0x20-0x2F ---
        if (op < 0x30) {
            int8_t rel = (int8_t)fetch8(cpu);
            if (branch_taken(cpu, (uint8_t)(op & 0x0F))) {
                cpu->pc = (uint16_t)(cpu->pc + rel);
            }
            return 0;
        }

        // --- Stack / control group 0x30-0x3F ---
        if (op < 0x40) {
            switch (op) {
                case 0x30: // TSX / TSY (= SP + 1)
                    if (use_y) cpu->y = (uint16_t)(cpu->sp + 1);
                    else cpu->x = (uint16_t)(cpu->sp + 1);
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
                case 0x35: // TXS / TYS (SP = reg - 1)
                    cpu->sp = (uint16_t)((use_y ? cpu->y : cpu->x) - 1);
                    break;
                case 0x36: // PSHA
                    push8(cpu, cpu->a);
                    break;
                case 0x37: // PSHB
                    push8(cpu, cpu->b);
                    break;
                case 0x38: // PULX / PULY
                    if (use_y) cpu->y = pull16(cpu);
                    else cpu->x = pull16(cpu);
                    break;
                case 0x39: // RTS
                    cpu->pc = pull16(cpu);
                    break;
                case 0x3A: // ABX / ABY (no flags)
                    if (use_y) cpu->y = (uint16_t)(cpu->y + cpu->b);
                    else cpu->x = (uint16_t)(cpu->x + cpu->b);
                    break;
                case 0x3B: // RTI (pull CCR, B, A, X, Y, PC)
                    cpu->ccr = pull8(cpu);
                    cpu->b = pull8(cpu);
                    cpu->a = pull8(cpu);
                    cpu->x = pull16(cpu);
                    cpu->y = pull16(cpu);
                    cpu->pc = pull16(cpu);
                    break;
                case 0x3C: // PSHX / PSHY
                    push16(cpu, use_y ? cpu->y : cpu->x);
                    break;
                case 0x3D: { // MUL (D = A * B; C = bit 7 of result for rounding)
                    uint16_t prod = (uint16_t)((uint16_t)cpu->a * cpu->b);
                    SET_D(prod);
                    SET_FLAG(FLAG_C, prod & 0x0080);
                    break;
                }
                case 0x3E: // WAI (wait for interrupt) - halts this emulator
                    cpu->halted = 1;
                    return 1;
                case 0x3F: // SWI (push machine state, vector via 0xFFF6)
                    push16(cpu, cpu->pc);
                    push16(cpu, cpu->y);
                    push16(cpu, cpu->x);
                    push8(cpu, cpu->a);
                    push8(cpu, cpu->b);
                    push8(cpu, cpu->ccr);
                    SET_FLAG(FLAG_I, 1);
                    cpu->pc = mem_read16(cpu, 0xFFF6);
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
                addr = (uint16_t)(ixb + fetch8(cpu));
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

            // Reject subcodes that do not exist on the HC11 (0x1, 0x2, 0x5, 0xB)
            if (sub == 0x01 || sub == 0x02 || sub == 0x05 || sub == 0x0B) {
                cpu->halted = 1;
                return 1;
            }

            if (mode == 4) val = cpu->a;
            else if (mode == 5) val = cpu->b;
            else val = mem_read(cpu, addr);

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

            if (sub == 0x03) { // SUBD (0x83..) / ADDD (0xC3..)
                uint16_t m = (mode == 0) ? fetch16(cpu)
                                         : mem_read16(cpu, fetch_addr(cpu, mode, ixb));
                uint16_t d = GET_D();
                d = use_b ? alu_add16(cpu, d, m) : alu_sub16(cpu, d, m);
                SET_D(d);
                return 0;
            }

            // 16-bit / flow-control columns (0xC-0xF)
            if (sub >= 0x0C) {
                if (!use_b) {
                    if (sub == 0x0C) { // CPX / CPY (full N Z V C on the HC11)
                        uint16_t m = (mode == 0) ? fetch16(cpu)
                                                 : mem_read16(cpu, fetch_addr(cpu, mode, ixb));
                        (void)alu_sub16(cpu, use_y ? cpu->y : cpu->x, m);
                        return 0;
                    }
                    if (sub == 0x0D) { // BSR (imm slot) / JSR (direct/indexed/extended)
                        if (mode == 0) { // 0x8D BSR
                            int8_t rel = (int8_t)fetch8(cpu);
                            push16(cpu, cpu->pc);
                            cpu->pc = (uint16_t)(cpu->pc + rel);
                            return 0;
                        }
                        addr = fetch_addr(cpu, mode, ixb); // 0x9D JSR direct is valid on HC11
                        push16(cpu, cpu->pc);
                        cpu->pc = addr;
                        return 0;
                    }
                    if (sub == 0x0E) { // LDS
                        if (mode == 0) cpu->sp = fetch16(cpu);
                        else cpu->sp = mem_read16(cpu, fetch_addr(cpu, mode, ixb));
                        set_nz16(cpu, cpu->sp);
                        SET_FLAG(FLAG_V, 0);
                        return 0;
                    }
                    // sub == 0x0F: 0x8F is XGDX / XGDY; other modes are STS
                    if (mode == 0) { // XGDX / XGDY (no flags)
                        uint16_t d = GET_D();
                        if (use_y) { SET_D(cpu->y); cpu->y = d; }
                        else { SET_D(cpu->x); cpu->x = d; }
                        return 0;
                    }
                    addr = fetch_addr(cpu, mode, ixb);
                    mem_write16(cpu, addr, cpu->sp);
                    set_nz16(cpu, cpu->sp);
                    SET_FLAG(FLAG_V, 0);
                    return 0;
                } else {
                    if (sub == 0x0C) { // LDD
                        uint16_t m = (mode == 0) ? fetch16(cpu)
                                                 : mem_read16(cpu, fetch_addr(cpu, mode, ixb));
                        SET_D(m);
                        set_nz16(cpu, m);
                        SET_FLAG(FLAG_V, 0);
                        return 0;
                    }
                    if (sub == 0x0D) { // STD (0xCD is the page-4 prefix, handled above)
                        if (mode == 0) { cpu->halted = 1; return 1; }
                        addr = fetch_addr(cpu, mode, ixb);
                        mem_write16(cpu, addr, GET_D());
                        set_nz16(cpu, GET_D());
                        SET_FLAG(FLAG_V, 0);
                        return 0;
                    }
                    if (sub == 0x0E) { // LDX / LDY
                        uint16_t m = (mode == 0) ? fetch16(cpu)
                                                 : mem_read16(cpu, fetch_addr(cpu, mode, ixb));
                        if (use_y) cpu->y = m;
                        else cpu->x = m;
                        set_nz16(cpu, m);
                        SET_FLAG(FLAG_V, 0);
                        return 0;
                    }
                    // sub == 0x0F: 0xCF is STOP; other modes are STX / STY
                    if (mode == 0) { // STOP - halts this emulator
                        cpu->halted = 1;
                        return 1;
                    }
                    {
                        uint16_t v = use_y ? cpu->y : cpu->x;
                        addr = fetch_addr(cpu, mode, ixb);
                        mem_write16(cpu, addr, v);
                        set_nz16(cpu, v);
                        SET_FLAG(FLAG_V, 0);
                        return 0;
                    }
                }
            }

            // 8-bit operation columns 0x0-0xB
            {
                uint8_t acc = use_b ? cpu->b : cpu->a;
                uint8_t m = 0;
                uint8_t carry_in = (uint8_t)GET_FLAG(FLAG_C);

                if (sub == 0x07) { // STA/STB (no immediate form)
                    if (mode == 0) { cpu->halted = 1; return 1; }
                    addr = fetch_addr(cpu, mode, ixb);
                    mem_write(cpu, addr, acc);
                    set_nz8(cpu, acc);
                    SET_FLAG(FLAG_V, 0);
                    return 0;
                }

                if (mode == 0) m = fetch8(cpu);
                else m = mem_read(cpu, fetch_addr(cpu, mode, ixb));

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
                    default: // 0x0B ADD
                        acc = alu_add(cpu, acc, m, 0);
                        break;
                }

                if (use_b) cpu->b = acc;
                else cpu->a = acc;
            }
        }
    }

    return 0;
}

// --- Debugging ---

void mc68hc11_print_state(void *context) {
    if (!context) return;
    MC68HC11_CPU *cpu = (MC68HC11_CPU*)context;

    printf("Motorola 68HC11 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Registers: A=0x%02X  B=0x%02X  D=0x%04X  X=0x%04X  Y=0x%04X\n",
           cpu->a, cpu->b, GET_D(), cpu->x, cpu->y);
    printf("  Flags: S=%d  X=%d  H=%d  I=%d  N=%d  Z=%d  V=%d  C=%d\n",
           GET_FLAG(FLAG_S), GET_FLAG(FLAG_X), GET_FLAG(FLAG_H), GET_FLAG(FLAG_I),
           GET_FLAG(FLAG_N), GET_FLAG(FLAG_Z), GET_FLAG(FLAG_V), GET_FLAG(FLAG_C));
}

// Mnemonic tables for the disassembler
static const char *g_branch_names[16] = {
    "bra", "brn", "bhi", "bls", "bcc", "bcs", "bne", "beq",
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

void mc68hc11_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    MC68HC11_CPU *cpu = (MC68HC11_CPU*)context;

    uint16_t pc = cpu->pc;
    uint8_t op = mem_read(cpu, pc);
    int page = 1;
    uint8_t b1, b2;
    int8_t b3;
    uint16_t w;
    char ix = 'x';

    if (op == 0x18) { page = 2; pc++; op = mem_read(cpu, pc); ix = 'y'; }
    else if (op == 0x1A) { page = 3; pc++; op = mem_read(cpu, pc); }
    else if (op == 0xCD) { page = 4; pc++; op = mem_read(cpu, pc); ix = 'y'; }

    b1 = mem_read(cpu, (uint16_t)(pc + 1));
    b2 = mem_read(cpu, (uint16_t)(pc + 2));
    b3 = (int8_t)mem_read(cpu, (uint16_t)(pc + 3));
    w = (uint16_t)(((uint16_t)b1 << 8) | b2);

    // Page 3 (0x1A) / page 4 (0xCD) prefixed opcodes
    if (page == 3 || page == 4) {
        switch (op) {
            case 0x83:
                if (page == 3) { snprintf(buf, buf_len, "cpd   #$0x%04X", w); return; }
                break;
            case 0x93:
                if (page == 3) { snprintf(buf, buf_len, "cpd   $0x%02X", b1); return; }
                break;
            case 0xA3:
                snprintf(buf, buf_len, "cpd   $0x%02X,%c", b1, ix);
                return;
            case 0xB3:
                if (page == 3) { snprintf(buf, buf_len, "cpd   $0x%04X", w); return; }
                break;
            case 0xAC:
                snprintf(buf, buf_len, "%s   $0x%02X,%c", (page == 3) ? "cpy" : "cpx", b1, ix);
                return;
            case 0xEE:
                snprintf(buf, buf_len, "%s   $0x%02X,%c", (page == 3) ? "ldy" : "ldx", b1, ix);
                return;
            case 0xEF:
                snprintf(buf, buf_len, "%s   $0x%02X,%c", (page == 3) ? "sty" : "stx", b1, ix);
                return;
            default:
                break;
        }
        snprintf(buf, buf_len, "unknown (0x%02X 0x%02X)", (page == 3) ? 0x1A : 0xCD, op);
        return;
    }

    // Inherent / bit-manipulation group
    if (op < 0x20) {
        const char *name = NULL;
        switch (op) {
            case 0x00: name = "test"; break;
            case 0x01: name = "nop"; break;
            case 0x02: name = "idiv"; break;
            case 0x03: name = "fdiv"; break;
            case 0x04: name = "lsrd"; break;
            case 0x05: name = "asld"; break;
            case 0x06: name = "tap"; break;
            case 0x07: name = "tpa"; break;
            case 0x08: name = (page == 2) ? "iny" : "inx"; break;
            case 0x09: name = (page == 2) ? "dey" : "dex"; break;
            case 0x0A: name = "clv"; break;
            case 0x0B: name = "sev"; break;
            case 0x0C: name = "clc"; break;
            case 0x0D: name = "sec"; break;
            case 0x0E: name = "cli"; break;
            case 0x0F: name = "sei"; break;
            case 0x10: name = "sba"; break;
            case 0x11: name = "cba"; break;
            case 0x12:
                snprintf(buf, buf_len, "brset $0x%02X #$0x%02X %+d", b1, b2, b3);
                return;
            case 0x13:
                snprintf(buf, buf_len, "brclr $0x%02X #$0x%02X %+d", b1, b2, b3);
                return;
            case 0x14:
                snprintf(buf, buf_len, "bset  $0x%02X #$0x%02X", b1, b2);
                return;
            case 0x15:
                snprintf(buf, buf_len, "bclr  $0x%02X #$0x%02X", b1, b2);
                return;
            case 0x16: name = "tab"; break;
            case 0x17: name = "tba"; break;
            case 0x19: name = "daa"; break;
            case 0x1B: name = "aba"; break;
            case 0x1C:
                snprintf(buf, buf_len, "bset  $0x%02X,%c #$0x%02X", b1, ix, b2);
                return;
            case 0x1D:
                snprintf(buf, buf_len, "bclr  $0x%02X,%c #$0x%02X", b1, ix, b2);
                return;
            case 0x1E:
                snprintf(buf, buf_len, "brset $0x%02X,%c #$0x%02X %+d", b1, ix, b2, b3);
                return;
            case 0x1F:
                snprintf(buf, buf_len, "brclr $0x%02X,%c #$0x%02X %+d", b1, ix, b2, b3);
                return;
            default: break;
        }
        if (name) snprintf(buf, buf_len, "%s", name);
        else snprintf(buf, buf_len, "unknown (0x%02X)", op);
        return;
    }

    // Branches
    if (op < 0x30) {
        snprintf(buf, buf_len, "%s   %+d", g_branch_names[op & 0x0F], (int8_t)b1);
        return;
    }

    // Stack / control group
    if (op < 0x40) {
        const char *name = NULL;
        switch (op) {
            case 0x30: name = (page == 2) ? "tsy" : "tsx"; break;
            case 0x31: name = "ins"; break;
            case 0x32: name = "pula"; break;
            case 0x33: name = "pulb"; break;
            case 0x34: name = "des"; break;
            case 0x35: name = (page == 2) ? "tys" : "txs"; break;
            case 0x36: name = "psha"; break;
            case 0x37: name = "pshb"; break;
            case 0x38: name = (page == 2) ? "puly" : "pulx"; break;
            case 0x39: name = "rts"; break;
            case 0x3A: name = (page == 2) ? "aby" : "abx"; break;
            case 0x3B: name = "rti"; break;
            case 0x3C: name = (page == 2) ? "pshy" : "pshx"; break;
            case 0x3D: name = "mul"; break;
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
        else if (mode == 6) snprintf(buf, buf_len, "%s   $0x%02X,%c", name, b1, ix);
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
        int imm16 = 1; // 16-bit / word ops use word immediates

        if (sub == 0x03) { // SUBD / ADDD
            name = use_b ? "addd" : "subd";
            if (mode == 0) snprintf(buf, buf_len, "%s  #$0x%04X", name, w);
            else if (mode == 1) snprintf(buf, buf_len, "%s  $0x%02X", name, b1);
            else if (mode == 2) snprintf(buf, buf_len, "%s  $0x%02X,%c", name, b1, ix);
            else snprintf(buf, buf_len, "%s  $0x%04X", name, w);
            return;
        }

        if (sub >= 0x0C) {
            if (!use_b) {
                if (sub == 0x0C) name = (page == 2) ? "cpy" : "cpx";
                else if (sub == 0x0D) {
                    if (mode == 0) { snprintf(buf, buf_len, "bsr   %+d", (int8_t)b1); return; }
                    name = "jsr"; // direct JSR (0x9D) is valid on the HC11
                }
                else if (sub == 0x0E) name = "lds";
                else { // sub == 0x0F
                    if (mode == 0) { // XGDX / XGDY
                        snprintf(buf, buf_len, "%s", (page == 2) ? "xgdy" : "xgdx");
                        return;
                    }
                    name = "sts";
                }
            } else {
                if (sub == 0x0C) name = "ldd";
                else if (sub == 0x0D) name = "std"; // 0xCD imm slot is the page-4 prefix
                else if (sub == 0x0E) name = (page == 2) ? "ldy" : "ldx";
                else { // sub == 0x0F
                    if (mode == 0) { snprintf(buf, buf_len, "stop"); return; }
                    name = (page == 2) ? "sty" : "stx";
                }
            }
            if (mode == 0 && (name[0] == 's' || name[0] == 'j')) { // no immediate store/jsr
                snprintf(buf, buf_len, "unknown (0x%02X)", op);
                return;
            }
        } else {
            name = g_alu_names[sub];
            imm16 = 0;
            if (name[0] == '?' || (mode == 0 && sub == 0x07)) {
                snprintf(buf, buf_len, "unknown (0x%02X)", op);
                return;
            }
        }

        if (imm16) {
            if (mode == 0) snprintf(buf, buf_len, "%s   #$0x%04X", name, w);
            else if (mode == 1) snprintf(buf, buf_len, "%s   $0x%02X", name, b1);
            else if (mode == 2) snprintf(buf, buf_len, "%s   $0x%02X,%c", name, b1, ix);
            else snprintf(buf, buf_len, "%s   $0x%04X", name, w);
        } else {
            if (mode == 0) snprintf(buf, buf_len, "%s%c  #$0x%02X", name, reg, b1);
            else if (mode == 1) snprintf(buf, buf_len, "%s%c  $0x%02X", name, reg, b1);
            else if (mode == 2) snprintf(buf, buf_len, "%s%c  $0x%02X,%c", name, reg, b1, ix);
            else snprintf(buf, buf_len, "%s%c  $0x%04X", name, reg, w);
        }
    }
}
