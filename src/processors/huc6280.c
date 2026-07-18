#include "huc6280.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Hudson HuC6280 (PC Engine / TurboGrafx-16 CPU core)
// 65C02-class core (full NMOS set + CMOS additions) plus HuC6280 specifics:
//   - 8 MPR mapping registers (TAM/TMA) mapping 8KB logical pages into 256KB physical space
//   - CLA/CLX/CLY, SXY/SAX/SAY swaps
//   - Block transfers TII/TDD/TIN/TIA/TAI
//   - ST0/ST1/ST2 latch to a VDC port array (otherwise no-ops)
//   - SET: only records the T flag. T-flag-modified ops (ORA/AND/EOR/ADC against
//     the memory location at zp,X) are simplified: they execute as normal A-based
//     ALU operations here, which is sufficient for code that never uses SET.
//   - CSL/CSH speed toggles are functional no-ops (a mode bit is tracked)
// BRK halts the core (used as the STP-equivalent stop condition).
// Decimal mode is not emulated (ADC/SBC always operate in binary).

#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_D 0x08
#define FLAG_B 0x10
#define FLAG_T 0x20  // HuC6280: the 6502's unused bit is the T (memory transfer) flag
#define FLAG_V 0x40
#define FLAG_N 0x80

#define PHYS_RAM_SIZE 262144u // 256 KB physical space (0x00000 - 0x3FFFF)

typedef struct HUC6280_CPU {
    uint8_t ram[PHYS_RAM_SIZE]; // flat 256 KB physical space addressed through the MPRs
    uint8_t mpr[8];             // MPR0-7: physical 8KB page mapped into each logical 8KB slot
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint8_t p;
    uint16_t pc;
    uint32_t ticks;
    int halted;
    int high_speed;             // CSL/CSH latch (no timing effect in this core)
    uint8_t vdc_port[3];        // ST0/ST1/ST2 sink (no VDC attached)
} HUC6280_CPU;

#define SET_FLAG_C(cond) do { if (cond) cpu->p |= FLAG_C; else cpu->p &= ~FLAG_C; } while(0)
#define SET_FLAG_Z(val) do { if ((val) == 0) cpu->p |= FLAG_Z; else cpu->p &= ~FLAG_Z; } while(0)
#define SET_FLAG_N(val) do { if ((val) & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N; } while(0)
#define GET_FLAG(flag) ((cpu->p & (flag)) ? 1 : 0)

// --- Memory access through the MPR banking registers ---

static uint32_t map_addr(HUC6280_CPU *cpu, uint16_t addr) {
    return (((uint32_t)cpu->mpr[addr >> 13] << 13) | (addr & 0x1FFF)) & (PHYS_RAM_SIZE - 1);
}

static uint8_t mem_read(HUC6280_CPU *cpu, uint16_t addr) {
    return cpu->ram[map_addr(cpu, addr)];
}

static void mem_write(HUC6280_CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->ram[map_addr(cpu, addr)] = val;
}

static uint8_t fetch8(HUC6280_CPU *cpu) {
    return mem_read(cpu, cpu->pc++);
}

static uint16_t fetch16(HUC6280_CPU *cpu) {
    uint16_t lo = fetch8(cpu);
    uint16_t hi = fetch8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

static uint16_t read16_zp(HUC6280_CPU *cpu, uint8_t zp) {
    uint16_t lo = mem_read(cpu, zp);
    uint16_t hi = mem_read(cpu, (uint16_t)((zp + 1) & 0xFF));
    return (uint16_t)(lo | (hi << 8));
}

// --- Effective address helpers (each consumes operand bytes at PC) ---

static uint16_t ea_zp(HUC6280_CPU *cpu)   { return fetch8(cpu); }
static uint16_t ea_zpx(HUC6280_CPU *cpu)  { return (uint16_t)((fetch8(cpu) + cpu->x) & 0xFF); }
static uint16_t ea_zpy(HUC6280_CPU *cpu)  { return (uint16_t)((fetch8(cpu) + cpu->y) & 0xFF); }
static uint16_t ea_abs(HUC6280_CPU *cpu)  { return fetch16(cpu); }
static uint16_t ea_absx(HUC6280_CPU *cpu) { return (uint16_t)(fetch16(cpu) + cpu->x); }
static uint16_t ea_absy(HUC6280_CPU *cpu) { return (uint16_t)(fetch16(cpu) + cpu->y); }
static uint16_t ea_indx(HUC6280_CPU *cpu) { return read16_zp(cpu, (uint8_t)((fetch8(cpu) + cpu->x) & 0xFF)); }
static uint16_t ea_indy(HUC6280_CPU *cpu) { return (uint16_t)(read16_zp(cpu, fetch8(cpu)) + cpu->y); }
static uint16_t ea_izp(HUC6280_CPU *cpu)  { return read16_zp(cpu, fetch8(cpu)); } // 65C02 (zp) mode

// --- Stack ---

static void push_byte(HUC6280_CPU *cpu, uint8_t val) {
    mem_write(cpu, (uint16_t)(0x0100 + cpu->sp), val);
    cpu->sp--;
}

static uint8_t pop_byte(HUC6280_CPU *cpu) {
    cpu->sp++;
    return mem_read(cpu, (uint16_t)(0x0100 + cpu->sp));
}

// --- ALU helpers ---

static void set_zn(HUC6280_CPU *cpu, uint8_t val) {
    SET_FLAG_Z(val);
    SET_FLAG_N(val);
}

static void op_ora(HUC6280_CPU *cpu, uint8_t val) { cpu->a |= val; set_zn(cpu, cpu->a); }
static void op_and(HUC6280_CPU *cpu, uint8_t val) { cpu->a &= val; set_zn(cpu, cpu->a); }
static void op_eor(HUC6280_CPU *cpu, uint8_t val) { cpu->a ^= val; set_zn(cpu, cpu->a); }

static void op_adc(HUC6280_CPU *cpu, uint8_t val) {
    uint16_t sum = (uint16_t)(cpu->a + val + GET_FLAG(FLAG_C));
    uint8_t result = (uint8_t)sum;
    if (~(cpu->a ^ val) & (cpu->a ^ result) & 0x80) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
    SET_FLAG_C(sum > 0xFF);
    cpu->a = result;
    set_zn(cpu, cpu->a);
}

static void op_sbc(HUC6280_CPU *cpu, uint8_t val) {
    op_adc(cpu, (uint8_t)(val ^ 0xFF));
}

static void op_cmp(HUC6280_CPU *cpu, uint8_t reg, uint8_t val) {
    uint8_t diff = (uint8_t)(reg - val);
    SET_FLAG_C(reg >= val);
    set_zn(cpu, diff);
}

static void op_bit(HUC6280_CPU *cpu, uint8_t val) {
    SET_FLAG_Z(cpu->a & val);
    if (val & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N;
    if (val & 0x40) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
}

static uint8_t op_asl(HUC6280_CPU *cpu, uint8_t val) {
    SET_FLAG_C(val & 0x80);
    val = (uint8_t)(val << 1);
    set_zn(cpu, val);
    return val;
}

static uint8_t op_lsr(HUC6280_CPU *cpu, uint8_t val) {
    SET_FLAG_C(val & 0x01);
    val >>= 1;
    set_zn(cpu, val);
    return val;
}

static uint8_t op_rol(HUC6280_CPU *cpu, uint8_t val) {
    uint8_t old_c = (uint8_t)GET_FLAG(FLAG_C);
    SET_FLAG_C(val & 0x80);
    val = (uint8_t)((val << 1) | old_c);
    set_zn(cpu, val);
    return val;
}

static uint8_t op_ror(HUC6280_CPU *cpu, uint8_t val) {
    uint8_t old_c = (uint8_t)GET_FLAG(FLAG_C);
    SET_FLAG_C(val & 0x01);
    val = (uint8_t)((val >> 1) | (old_c << 7));
    set_zn(cpu, val);
    return val;
}

static uint8_t op_inc(HUC6280_CPU *cpu, uint8_t val) { val = (uint8_t)(val + 1); set_zn(cpu, val); return val; }
static uint8_t op_dec(HUC6280_CPU *cpu, uint8_t val) { val = (uint8_t)(val - 1); set_zn(cpu, val); return val; }

static uint8_t op_trb(HUC6280_CPU *cpu, uint8_t val) { SET_FLAG_Z(cpu->a & val); return (uint8_t)(val & ~cpu->a); }
static uint8_t op_tsb(HUC6280_CPU *cpu, uint8_t val) { SET_FLAG_Z(cpu->a & val); return (uint8_t)(val | cpu->a); }

static void rmw(HUC6280_CPU *cpu, uint16_t addr, uint8_t (*op)(HUC6280_CPU*, uint8_t)) {
    mem_write(cpu, addr, op(cpu, mem_read(cpu, addr)));
}

// TST #imm, <mem>: BIT-like test of an immediate mask against memory (does not use A)
static void op_tst(HUC6280_CPU *cpu, uint8_t imm, uint8_t val) {
    SET_FLAG_Z(imm & val);
    if (val & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N;
    if (val & 0x40) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
}

static void branch(HUC6280_CPU *cpu, int cond) {
    int8_t off = (int8_t)fetch8(cpu);
    if (cond) cpu->pc = (uint16_t)(cpu->pc + off);
}

// BBRn/BBSn: operands are zp then relative offset
static void branch_bit(HUC6280_CPU *cpu, uint8_t bit, int set) {
    uint8_t zp = fetch8(cpu);
    int8_t off = (int8_t)fetch8(cpu);
    uint8_t val = mem_read(cpu, zp);
    int taken = set ? ((val >> bit) & 1) : !((val >> bit) & 1);
    if (taken) cpu->pc = (uint16_t)(cpu->pc + off);
}

static void rmb_smb(HUC6280_CPU *cpu, uint8_t bit, int set) {
    uint8_t zp = fetch8(cpu);
    uint8_t val = mem_read(cpu, zp);
    if (set) val = (uint8_t)(val | (1 << bit));
    else val = (uint8_t)(val & ~(1 << bit));
    mem_write(cpu, zp, val);
}

// Block transfers: 6 operand bytes (src lo/hi, dst lo/hi, len lo/hi; len 0 = 65536).
// The real chip saves/restores A, X, Y on the stack around the transfer; this core
// leaves the registers untouched, which is equivalent from the program's view.
enum { BLK_TII, BLK_TDD, BLK_TIN, BLK_TIA, BLK_TAI };

static void block_transfer(HUC6280_CPU *cpu, int type) {
    uint16_t src = fetch16(cpu);
    uint16_t dst = fetch16(cpu);
    uint16_t len16 = fetch16(cpu);
    uint32_t len = len16 ? len16 : 65536u;
    uint32_t i;
    for (i = 0; i < len; ++i) {
        uint8_t v;
        switch (type) {
            case BLK_TII: // increment src, increment dst
                v = mem_read(cpu, src); mem_write(cpu, dst, v); src++; dst++;
                break;
            case BLK_TDD: // decrement src, decrement dst
                v = mem_read(cpu, src); mem_write(cpu, dst, v); src--; dst--;
                break;
            case BLK_TIN: // increment src, fixed dst
                v = mem_read(cpu, src); mem_write(cpu, dst, v); src++;
                break;
            case BLK_TIA: // increment src, dst alternates dst / dst+1
                v = mem_read(cpu, src); mem_write(cpu, (uint16_t)(dst + (i & 1)), v); src++;
                break;
            case BLK_TAI: // src alternates src / src+1, increment dst
            default:
                v = mem_read(cpu, (uint16_t)(src + (i & 1))); mem_write(cpu, dst, v); dst++;
                break;
        }
    }
    cpu->ticks += 6 * len + 17; // approximate: 6 cycles per byte plus overhead
}

// --- Lifecycle ---

void* huc6280_create(void) {
    HUC6280_CPU *cpu = (HUC6280_CPU*)calloc(1, sizeof(HUC6280_CPU));
    return cpu;
}

void huc6280_destroy(void *context) {
    free(context);
}

int huc6280_init(void *context) {
    if (!context) return -1;
    HUC6280_CPU *cpu = (HUC6280_CPU*)context;
    int i;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->p = FLAG_I;
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->high_speed = 0;
    memset(cpu->vdc_port, 0, sizeof(cpu->vdc_port));

    // Identity-ish MPR defaults: logical page N -> physical page N, except MPR7 = 0
    // so that reset at logical 0xE000+ still lands in physical page 0 territory.
    for (i = 0; i < 8; ++i) cpu->mpr[i] = (uint8_t)i;
    cpu->mpr[7] = 0;

    return 0;
}

int huc6280_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    HUC6280_CPU *cpu = (HUC6280_CPU*)context;

    if (address >= PHYS_RAM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > PHYS_RAM_SIZE) {
        copy_len = PHYS_RAM_SIZE - address;
    }
    memcpy(cpu->ram + address, data, copy_len);
    cpu->pc = (uint16_t)address; // with identity MPRs the low 64KB is mapped 1:1

    return 0;
}

// --- Execution ---

int huc6280_step(void *context) {
    if (!context) return -1;
    HUC6280_CPU *cpu = (HUC6280_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch8(cpu);
    cpu->ticks++;

    switch (op) {
        // --- BRK: halt (STP-equivalent stop condition) ---
        case 0x00:
            cpu->halted = 1;
            return 1;

        // --- HuC6280 specifics ---
        case 0x02: { uint8_t t = cpu->x; cpu->x = cpu->y; cpu->y = t; } break; // SXY
        case 0x22: { uint8_t t = cpu->a; cpu->a = cpu->x; cpu->x = t; } break; // SAX
        case 0x42: { uint8_t t = cpu->a; cpu->a = cpu->y; cpu->y = t; } break; // SAY
        case 0x62: cpu->a = 0; break; // CLA
        case 0x82: cpu->x = 0; break; // CLX
        case 0xC2: cpu->y = 0; break; // CLY
        case 0x53: { // TAM #mask: copy A into every MPR selected by the mask
                uint8_t mask = fetch8(cpu);
                int i;
                for (i = 0; i < 8; ++i) {
                    if (mask & (1 << i)) cpu->mpr[i] = cpu->a;
                }
            }
            break;
        case 0x43: { // TMA #mask: copy the lowest selected MPR into A
                uint8_t mask = fetch8(cpu);
                int i;
                for (i = 0; i < 8; ++i) {
                    if (mask & (1 << i)) { cpu->a = cpu->mpr[i]; break; }
                }
            }
            break;
        case 0x03: cpu->vdc_port[0] = fetch8(cpu); break; // ST0 (VDC address port; no VDC attached)
        case 0x13: cpu->vdc_port[1] = fetch8(cpu); break; // ST1 (VDC data low)
        case 0x23: cpu->vdc_port[2] = fetch8(cpu); break; // ST2 (VDC data high)
        case 0x54: cpu->high_speed = 0; break; // CSL: 1.79 MHz mode (no-op here)
        case 0xD4: cpu->high_speed = 1; break; // CSH: 7.16 MHz mode (no-op here)
        case 0xF4: cpu->p |= FLAG_T; break;    // SET: T flag noted only; next ALU op runs as normal A-based ALU
        case 0x73: block_transfer(cpu, BLK_TII); break;
        case 0xC3: block_transfer(cpu, BLK_TDD); break;
        case 0xD3: block_transfer(cpu, BLK_TIN); break;
        case 0xE3: block_transfer(cpu, BLK_TIA); break;
        case 0xF3: block_transfer(cpu, BLK_TAI); break;
        case 0x83: { uint8_t imm = fetch8(cpu); op_tst(cpu, imm, mem_read(cpu, ea_zp(cpu))); } break;   // TST #imm, zp
        case 0xA3: { uint8_t imm = fetch8(cpu); op_tst(cpu, imm, mem_read(cpu, ea_zpx(cpu))); } break;  // TST #imm, zp,X
        case 0x93: { uint8_t imm = fetch8(cpu); op_tst(cpu, imm, mem_read(cpu, ea_abs(cpu))); } break;  // TST #imm, abs
        case 0xB3: { uint8_t imm = fetch8(cpu); op_tst(cpu, imm, mem_read(cpu, ea_absx(cpu))); } break; // TST #imm, abs,X

        // --- LDA ---
        case 0xA9: cpu->a = fetch8(cpu); set_zn(cpu, cpu->a); break;
        case 0xA5: cpu->a = mem_read(cpu, ea_zp(cpu)); set_zn(cpu, cpu->a); break;
        case 0xB5: cpu->a = mem_read(cpu, ea_zpx(cpu)); set_zn(cpu, cpu->a); break;
        case 0xAD: cpu->a = mem_read(cpu, ea_abs(cpu)); set_zn(cpu, cpu->a); break;
        case 0xBD: cpu->a = mem_read(cpu, ea_absx(cpu)); set_zn(cpu, cpu->a); break;
        case 0xB9: cpu->a = mem_read(cpu, ea_absy(cpu)); set_zn(cpu, cpu->a); break;
        case 0xA1: cpu->a = mem_read(cpu, ea_indx(cpu)); set_zn(cpu, cpu->a); break;
        case 0xB1: cpu->a = mem_read(cpu, ea_indy(cpu)); set_zn(cpu, cpu->a); break;
        case 0xB2: cpu->a = mem_read(cpu, ea_izp(cpu)); set_zn(cpu, cpu->a); break;

        // --- LDX ---
        case 0xA2: cpu->x = fetch8(cpu); set_zn(cpu, cpu->x); break;
        case 0xA6: cpu->x = mem_read(cpu, ea_zp(cpu)); set_zn(cpu, cpu->x); break;
        case 0xB6: cpu->x = mem_read(cpu, ea_zpy(cpu)); set_zn(cpu, cpu->x); break;
        case 0xAE: cpu->x = mem_read(cpu, ea_abs(cpu)); set_zn(cpu, cpu->x); break;
        case 0xBE: cpu->x = mem_read(cpu, ea_absy(cpu)); set_zn(cpu, cpu->x); break;

        // --- LDY ---
        case 0xA0: cpu->y = fetch8(cpu); set_zn(cpu, cpu->y); break;
        case 0xA4: cpu->y = mem_read(cpu, ea_zp(cpu)); set_zn(cpu, cpu->y); break;
        case 0xB4: cpu->y = mem_read(cpu, ea_zpx(cpu)); set_zn(cpu, cpu->y); break;
        case 0xAC: cpu->y = mem_read(cpu, ea_abs(cpu)); set_zn(cpu, cpu->y); break;
        case 0xBC: cpu->y = mem_read(cpu, ea_absx(cpu)); set_zn(cpu, cpu->y); break;

        // --- STA ---
        case 0x85: mem_write(cpu, ea_zp(cpu), cpu->a); break;
        case 0x95: mem_write(cpu, ea_zpx(cpu), cpu->a); break;
        case 0x8D: mem_write(cpu, ea_abs(cpu), cpu->a); break;
        case 0x9D: mem_write(cpu, ea_absx(cpu), cpu->a); break;
        case 0x99: mem_write(cpu, ea_absy(cpu), cpu->a); break;
        case 0x81: mem_write(cpu, ea_indx(cpu), cpu->a); break;
        case 0x91: mem_write(cpu, ea_indy(cpu), cpu->a); break;
        case 0x92: mem_write(cpu, ea_izp(cpu), cpu->a); break;

        // --- STX / STY / STZ ---
        case 0x86: mem_write(cpu, ea_zp(cpu), cpu->x); break;
        case 0x96: mem_write(cpu, ea_zpy(cpu), cpu->x); break;
        case 0x8E: mem_write(cpu, ea_abs(cpu), cpu->x); break;
        case 0x84: mem_write(cpu, ea_zp(cpu), cpu->y); break;
        case 0x94: mem_write(cpu, ea_zpx(cpu), cpu->y); break;
        case 0x8C: mem_write(cpu, ea_abs(cpu), cpu->y); break;
        case 0x64: mem_write(cpu, ea_zp(cpu), 0); break;
        case 0x74: mem_write(cpu, ea_zpx(cpu), 0); break;
        case 0x9C: mem_write(cpu, ea_abs(cpu), 0); break;
        case 0x9E: mem_write(cpu, ea_absx(cpu), 0); break;

        // --- ORA ---
        case 0x09: op_ora(cpu, fetch8(cpu)); break;
        case 0x05: op_ora(cpu, mem_read(cpu, ea_zp(cpu))); break;
        case 0x15: op_ora(cpu, mem_read(cpu, ea_zpx(cpu))); break;
        case 0x0D: op_ora(cpu, mem_read(cpu, ea_abs(cpu))); break;
        case 0x1D: op_ora(cpu, mem_read(cpu, ea_absx(cpu))); break;
        case 0x19: op_ora(cpu, mem_read(cpu, ea_absy(cpu))); break;
        case 0x01: op_ora(cpu, mem_read(cpu, ea_indx(cpu))); break;
        case 0x11: op_ora(cpu, mem_read(cpu, ea_indy(cpu))); break;
        case 0x12: op_ora(cpu, mem_read(cpu, ea_izp(cpu))); break;

        // --- AND ---
        case 0x29: op_and(cpu, fetch8(cpu)); break;
        case 0x25: op_and(cpu, mem_read(cpu, ea_zp(cpu))); break;
        case 0x35: op_and(cpu, mem_read(cpu, ea_zpx(cpu))); break;
        case 0x2D: op_and(cpu, mem_read(cpu, ea_abs(cpu))); break;
        case 0x3D: op_and(cpu, mem_read(cpu, ea_absx(cpu))); break;
        case 0x39: op_and(cpu, mem_read(cpu, ea_absy(cpu))); break;
        case 0x21: op_and(cpu, mem_read(cpu, ea_indx(cpu))); break;
        case 0x31: op_and(cpu, mem_read(cpu, ea_indy(cpu))); break;
        case 0x32: op_and(cpu, mem_read(cpu, ea_izp(cpu))); break;

        // --- EOR ---
        case 0x49: op_eor(cpu, fetch8(cpu)); break;
        case 0x45: op_eor(cpu, mem_read(cpu, ea_zp(cpu))); break;
        case 0x55: op_eor(cpu, mem_read(cpu, ea_zpx(cpu))); break;
        case 0x4D: op_eor(cpu, mem_read(cpu, ea_abs(cpu))); break;
        case 0x5D: op_eor(cpu, mem_read(cpu, ea_absx(cpu))); break;
        case 0x59: op_eor(cpu, mem_read(cpu, ea_absy(cpu))); break;
        case 0x41: op_eor(cpu, mem_read(cpu, ea_indx(cpu))); break;
        case 0x51: op_eor(cpu, mem_read(cpu, ea_indy(cpu))); break;
        case 0x52: op_eor(cpu, mem_read(cpu, ea_izp(cpu))); break;

        // --- ADC ---
        case 0x69: op_adc(cpu, fetch8(cpu)); break;
        case 0x65: op_adc(cpu, mem_read(cpu, ea_zp(cpu))); break;
        case 0x75: op_adc(cpu, mem_read(cpu, ea_zpx(cpu))); break;
        case 0x6D: op_adc(cpu, mem_read(cpu, ea_abs(cpu))); break;
        case 0x7D: op_adc(cpu, mem_read(cpu, ea_absx(cpu))); break;
        case 0x79: op_adc(cpu, mem_read(cpu, ea_absy(cpu))); break;
        case 0x61: op_adc(cpu, mem_read(cpu, ea_indx(cpu))); break;
        case 0x71: op_adc(cpu, mem_read(cpu, ea_indy(cpu))); break;
        case 0x72: op_adc(cpu, mem_read(cpu, ea_izp(cpu))); break;

        // --- SBC ---
        case 0xE9: op_sbc(cpu, fetch8(cpu)); break;
        case 0xE5: op_sbc(cpu, mem_read(cpu, ea_zp(cpu))); break;
        case 0xF5: op_sbc(cpu, mem_read(cpu, ea_zpx(cpu))); break;
        case 0xED: op_sbc(cpu, mem_read(cpu, ea_abs(cpu))); break;
        case 0xFD: op_sbc(cpu, mem_read(cpu, ea_absx(cpu))); break;
        case 0xF9: op_sbc(cpu, mem_read(cpu, ea_absy(cpu))); break;
        case 0xE1: op_sbc(cpu, mem_read(cpu, ea_indx(cpu))); break;
        case 0xF1: op_sbc(cpu, mem_read(cpu, ea_indy(cpu))); break;
        case 0xF2: op_sbc(cpu, mem_read(cpu, ea_izp(cpu))); break;

        // --- CMP / CPX / CPY ---
        case 0xC9: op_cmp(cpu, cpu->a, fetch8(cpu)); break;
        case 0xC5: op_cmp(cpu, cpu->a, mem_read(cpu, ea_zp(cpu))); break;
        case 0xD5: op_cmp(cpu, cpu->a, mem_read(cpu, ea_zpx(cpu))); break;
        case 0xCD: op_cmp(cpu, cpu->a, mem_read(cpu, ea_abs(cpu))); break;
        case 0xDD: op_cmp(cpu, cpu->a, mem_read(cpu, ea_absx(cpu))); break;
        case 0xD9: op_cmp(cpu, cpu->a, mem_read(cpu, ea_absy(cpu))); break;
        case 0xC1: op_cmp(cpu, cpu->a, mem_read(cpu, ea_indx(cpu))); break;
        case 0xD1: op_cmp(cpu, cpu->a, mem_read(cpu, ea_indy(cpu))); break;
        case 0xD2: op_cmp(cpu, cpu->a, mem_read(cpu, ea_izp(cpu))); break;
        case 0xE0: op_cmp(cpu, cpu->x, fetch8(cpu)); break;
        case 0xE4: op_cmp(cpu, cpu->x, mem_read(cpu, ea_zp(cpu))); break;
        case 0xEC: op_cmp(cpu, cpu->x, mem_read(cpu, ea_abs(cpu))); break;
        case 0xC0: op_cmp(cpu, cpu->y, fetch8(cpu)); break;
        case 0xC4: op_cmp(cpu, cpu->y, mem_read(cpu, ea_zp(cpu))); break;
        case 0xCC: op_cmp(cpu, cpu->y, mem_read(cpu, ea_abs(cpu))); break;

        // --- BIT ---
        case 0x89: { uint8_t val = fetch8(cpu); SET_FLAG_Z(cpu->a & val); } break; // BIT #imm: only Z
        case 0x24: op_bit(cpu, mem_read(cpu, ea_zp(cpu))); break;
        case 0x34: op_bit(cpu, mem_read(cpu, ea_zpx(cpu))); break;
        case 0x2C: op_bit(cpu, mem_read(cpu, ea_abs(cpu))); break;
        case 0x3C: op_bit(cpu, mem_read(cpu, ea_absx(cpu))); break;

        // --- TRB / TSB ---
        case 0x14: rmw(cpu, ea_zp(cpu), op_trb); break;
        case 0x1C: rmw(cpu, ea_abs(cpu), op_trb); break;
        case 0x04: rmw(cpu, ea_zp(cpu), op_tsb); break;
        case 0x0C: rmw(cpu, ea_abs(cpu), op_tsb); break;

        // --- Shifts / rotates ---
        case 0x0A: cpu->a = op_asl(cpu, cpu->a); break;
        case 0x06: rmw(cpu, ea_zp(cpu), op_asl); break;
        case 0x16: rmw(cpu, ea_zpx(cpu), op_asl); break;
        case 0x0E: rmw(cpu, ea_abs(cpu), op_asl); break;
        case 0x1E: rmw(cpu, ea_absx(cpu), op_asl); break;
        case 0x4A: cpu->a = op_lsr(cpu, cpu->a); break;
        case 0x46: rmw(cpu, ea_zp(cpu), op_lsr); break;
        case 0x56: rmw(cpu, ea_zpx(cpu), op_lsr); break;
        case 0x4E: rmw(cpu, ea_abs(cpu), op_lsr); break;
        case 0x5E: rmw(cpu, ea_absx(cpu), op_lsr); break;
        case 0x2A: cpu->a = op_rol(cpu, cpu->a); break;
        case 0x26: rmw(cpu, ea_zp(cpu), op_rol); break;
        case 0x36: rmw(cpu, ea_zpx(cpu), op_rol); break;
        case 0x2E: rmw(cpu, ea_abs(cpu), op_rol); break;
        case 0x3E: rmw(cpu, ea_absx(cpu), op_rol); break;
        case 0x6A: cpu->a = op_ror(cpu, cpu->a); break;
        case 0x66: rmw(cpu, ea_zp(cpu), op_ror); break;
        case 0x76: rmw(cpu, ea_zpx(cpu), op_ror); break;
        case 0x6E: rmw(cpu, ea_abs(cpu), op_ror); break;
        case 0x7E: rmw(cpu, ea_absx(cpu), op_ror); break;

        // --- INC / DEC ---
        case 0x1A: cpu->a = op_inc(cpu, cpu->a); break; // INC A (65C02)
        case 0xE6: rmw(cpu, ea_zp(cpu), op_inc); break;
        case 0xF6: rmw(cpu, ea_zpx(cpu), op_inc); break;
        case 0xEE: rmw(cpu, ea_abs(cpu), op_inc); break;
        case 0xFE: rmw(cpu, ea_absx(cpu), op_inc); break;
        case 0x3A: cpu->a = op_dec(cpu, cpu->a); break; // DEC A (65C02)
        case 0xC6: rmw(cpu, ea_zp(cpu), op_dec); break;
        case 0xD6: rmw(cpu, ea_zpx(cpu), op_dec); break;
        case 0xCE: rmw(cpu, ea_abs(cpu), op_dec); break;
        case 0xDE: rmw(cpu, ea_absx(cpu), op_dec); break;
        case 0xE8: cpu->x = op_inc(cpu, cpu->x); break;
        case 0xC8: cpu->y = op_inc(cpu, cpu->y); break;
        case 0xCA: cpu->x = op_dec(cpu, cpu->x); break;
        case 0x88: cpu->y = op_dec(cpu, cpu->y); break;

        // --- Transfers ---
        case 0xAA: cpu->x = cpu->a; set_zn(cpu, cpu->x); break;
        case 0x8A: cpu->a = cpu->x; set_zn(cpu, cpu->a); break;
        case 0xA8: cpu->y = cpu->a; set_zn(cpu, cpu->y); break;
        case 0x98: cpu->a = cpu->y; set_zn(cpu, cpu->a); break;
        case 0x9A: cpu->sp = cpu->x; break;
        case 0xBA: cpu->x = cpu->sp; set_zn(cpu, cpu->x); break;

        // --- Stack ---
        case 0x48: push_byte(cpu, cpu->a); break;
        case 0x68: cpu->a = pop_byte(cpu); set_zn(cpu, cpu->a); break;
        case 0x08: push_byte(cpu, (uint8_t)(cpu->p | FLAG_B)); break;
        case 0x28: cpu->p = pop_byte(cpu); break;
        case 0xDA: push_byte(cpu, cpu->x); break; // PHX
        case 0xFA: cpu->x = pop_byte(cpu); set_zn(cpu, cpu->x); break; // PLX
        case 0x5A: push_byte(cpu, cpu->y); break; // PHY
        case 0x7A: cpu->y = pop_byte(cpu); set_zn(cpu, cpu->y); break; // PLY

        // --- Jumps / subroutines ---
        case 0x4C: cpu->pc = fetch16(cpu); break;
        case 0x6C: { // JMP (ind) - 65C02 behavior: no page-wrap bug
                uint16_t addr = fetch16(cpu);
                cpu->pc = (uint16_t)(mem_read(cpu, addr) | ((uint16_t)mem_read(cpu, (uint16_t)(addr + 1)) << 8));
            }
            break;
        case 0x7C: { // JMP (abs,X)
                uint16_t addr = (uint16_t)(fetch16(cpu) + cpu->x);
                cpu->pc = (uint16_t)(mem_read(cpu, addr) | ((uint16_t)mem_read(cpu, (uint16_t)(addr + 1)) << 8));
            }
            break;
        case 0x20: { // JSR
                uint16_t target = fetch16(cpu);
                uint16_t ret = (uint16_t)(cpu->pc - 1);
                push_byte(cpu, (uint8_t)(ret >> 8));
                push_byte(cpu, (uint8_t)(ret & 0xFF));
                cpu->pc = target;
            }
            break;
        case 0x60: { // RTS
                uint16_t lo = pop_byte(cpu);
                uint16_t hi = pop_byte(cpu);
                cpu->pc = (uint16_t)((lo | (hi << 8)) + 1);
            }
            break;
        case 0x40: { // RTI
                cpu->p = pop_byte(cpu);
                uint16_t lo = pop_byte(cpu);
                uint16_t hi = pop_byte(cpu);
                cpu->pc = (uint16_t)(lo | (hi << 8));
            }
            break;
        case 0x44: { // BSR (HuC6280 relative subroutine call)
                int8_t off = (int8_t)fetch8(cpu);
                uint16_t ret = (uint16_t)(cpu->pc - 1);
                push_byte(cpu, (uint8_t)(ret >> 8));
                push_byte(cpu, (uint8_t)(ret & 0xFF));
                cpu->pc = (uint16_t)(cpu->pc + off);
            }
            break;

        // --- Branches ---
        case 0x10: branch(cpu, !GET_FLAG(FLAG_N)); break;
        case 0x30: branch(cpu, GET_FLAG(FLAG_N)); break;
        case 0x50: branch(cpu, !GET_FLAG(FLAG_V)); break;
        case 0x70: branch(cpu, GET_FLAG(FLAG_V)); break;
        case 0x90: branch(cpu, !GET_FLAG(FLAG_C)); break;
        case 0xB0: branch(cpu, GET_FLAG(FLAG_C)); break;
        case 0xD0: branch(cpu, !GET_FLAG(FLAG_Z)); break;
        case 0xF0: branch(cpu, GET_FLAG(FLAG_Z)); break;
        case 0x80: branch(cpu, 1); break; // BRA

        // --- BBRn / BBSn ---
        case 0x0F: branch_bit(cpu, 0, 0); break;
        case 0x1F: branch_bit(cpu, 1, 0); break;
        case 0x2F: branch_bit(cpu, 2, 0); break;
        case 0x3F: branch_bit(cpu, 3, 0); break;
        case 0x4F: branch_bit(cpu, 4, 0); break;
        case 0x5F: branch_bit(cpu, 5, 0); break;
        case 0x6F: branch_bit(cpu, 6, 0); break;
        case 0x7F: branch_bit(cpu, 7, 0); break;
        case 0x8F: branch_bit(cpu, 0, 1); break;
        case 0x9F: branch_bit(cpu, 1, 1); break;
        case 0xAF: branch_bit(cpu, 2, 1); break;
        case 0xBF: branch_bit(cpu, 3, 1); break;
        case 0xCF: branch_bit(cpu, 4, 1); break;
        case 0xDF: branch_bit(cpu, 5, 1); break;
        case 0xEF: branch_bit(cpu, 6, 1); break;
        case 0xFF: branch_bit(cpu, 7, 1); break;

        // --- RMBn / SMBn ---
        case 0x07: rmb_smb(cpu, 0, 0); break;
        case 0x17: rmb_smb(cpu, 1, 0); break;
        case 0x27: rmb_smb(cpu, 2, 0); break;
        case 0x37: rmb_smb(cpu, 3, 0); break;
        case 0x47: rmb_smb(cpu, 4, 0); break;
        case 0x57: rmb_smb(cpu, 5, 0); break;
        case 0x67: rmb_smb(cpu, 6, 0); break;
        case 0x77: rmb_smb(cpu, 7, 0); break;
        case 0x87: rmb_smb(cpu, 0, 1); break;
        case 0x97: rmb_smb(cpu, 1, 1); break;
        case 0xA7: rmb_smb(cpu, 2, 1); break;
        case 0xB7: rmb_smb(cpu, 3, 1); break;
        case 0xC7: rmb_smb(cpu, 4, 1); break;
        case 0xD7: rmb_smb(cpu, 5, 1); break;
        case 0xE7: rmb_smb(cpu, 6, 1); break;
        case 0xF7: rmb_smb(cpu, 7, 1); break;

        // --- Flag operations ---
        case 0x18: cpu->p &= ~FLAG_C; break;
        case 0x38: cpu->p |= FLAG_C; break;
        case 0x58: cpu->p &= ~FLAG_I; break;
        case 0x78: cpu->p |= FLAG_I; break;
        case 0xD8: cpu->p &= ~FLAG_D; break;
        case 0xF8: cpu->p |= FLAG_D; break;
        case 0xB8: cpu->p &= ~FLAG_V; break;

        // --- NOP and remaining undefined slots (single-byte NOPs on 65C02-class parts) ---
        case 0xEA:
        default:
            break;
    }

    return 0;
}

// --- State / disassembly ---

void huc6280_print_state(void *context) {
    if (!context) return;
    HUC6280_CPU *cpu = (HUC6280_CPU*)context;

    printf("HuC6280 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%02X  Halted: %s  Speed: %s\n",
           cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No",
           cpu->high_speed ? "7.16MHz" : "1.79MHz");
    printf("  Registers: A=0x%02X  X=0x%02X  Y=0x%02X\n", cpu->a, cpu->x, cpu->y);
    printf("  Flags: N=%d  V=%d  T=%d  B=%d  D=%d  I=%d  Z=%d  C=%d\n",
           GET_FLAG(FLAG_N), GET_FLAG(FLAG_V), GET_FLAG(FLAG_T), GET_FLAG(FLAG_B),
           GET_FLAG(FLAG_D), GET_FLAG(FLAG_I), GET_FLAG(FLAG_Z), GET_FLAG(FLAG_C));
    printf("  MPR: 0=0x%02X 1=0x%02X 2=0x%02X 3=0x%02X 4=0x%02X 5=0x%02X 6=0x%02X 7=0x%02X\n",
           cpu->mpr[0], cpu->mpr[1], cpu->mpr[2], cpu->mpr[3],
           cpu->mpr[4], cpu->mpr[5], cpu->mpr[6], cpu->mpr[7]);
}

// Addressing modes for the disassembler table
enum {
    AM_IMP,   // implied
    AM_ACC,   // accumulator
    AM_IMM,   // #imm
    AM_ZP,    // zp
    AM_ZPX,   // zp,X
    AM_ZPY,   // zp,Y
    AM_ABS,   // abs
    AM_ABSX,  // abs,X
    AM_ABSY,  // abs,Y
    AM_IND,   // (abs)
    AM_IABSX, // (abs,X)
    AM_INDX,  // (zp,X)
    AM_INDY,  // (zp),Y
    AM_IZP,   // (zp)
    AM_REL,   // relative branch
    AM_ZPREL, // zp, relative (BBRn/BBSn)
    AM_BLK,   // src, dst, len (block transfers)
    AM_IMZP,  // #imm, zp (TST)
    AM_IMZPX, // #imm, zp,X (TST)
    AM_IMAB,  // #imm, abs (TST)
    AM_IMABX  // #imm, abs,X (TST)
};

typedef struct DisasmEntry {
    const char *name;
    uint8_t mode;
} DisasmEntry;

static const DisasmEntry g_disasm[256] = {
    [0x00] = {"brk", AM_IMP},   [0x01] = {"ora", AM_INDX},  [0x02] = {"sxy", AM_IMP},
    [0x03] = {"st0", AM_IMM},   [0x04] = {"tsb", AM_ZP},    [0x05] = {"ora", AM_ZP},
    [0x06] = {"asl", AM_ZP},    [0x07] = {"rmb0", AM_ZP},   [0x08] = {"php", AM_IMP},
    [0x09] = {"ora", AM_IMM},   [0x0A] = {"asl", AM_ACC},   [0x0C] = {"tsb", AM_ABS},
    [0x0D] = {"ora", AM_ABS},   [0x0E] = {"asl", AM_ABS},   [0x0F] = {"bbr0", AM_ZPREL},
    [0x10] = {"bpl", AM_REL},   [0x11] = {"ora", AM_INDY},  [0x12] = {"ora", AM_IZP},
    [0x13] = {"st1", AM_IMM},   [0x14] = {"trb", AM_ZP},    [0x15] = {"ora", AM_ZPX},
    [0x16] = {"asl", AM_ZPX},   [0x17] = {"rmb1", AM_ZP},   [0x18] = {"clc", AM_IMP},
    [0x19] = {"ora", AM_ABSY},  [0x1A] = {"inc", AM_ACC},   [0x1C] = {"trb", AM_ABS},
    [0x1D] = {"ora", AM_ABSX},  [0x1E] = {"asl", AM_ABSX},  [0x1F] = {"bbr1", AM_ZPREL},
    [0x20] = {"jsr", AM_ABS},   [0x21] = {"and", AM_INDX},  [0x22] = {"sax", AM_IMP},
    [0x23] = {"st2", AM_IMM},   [0x24] = {"bit", AM_ZP},    [0x25] = {"and", AM_ZP},
    [0x26] = {"rol", AM_ZP},    [0x27] = {"rmb2", AM_ZP},   [0x28] = {"plp", AM_IMP},
    [0x29] = {"and", AM_IMM},   [0x2A] = {"rol", AM_ACC},   [0x2C] = {"bit", AM_ABS},
    [0x2D] = {"and", AM_ABS},   [0x2E] = {"rol", AM_ABS},   [0x2F] = {"bbr2", AM_ZPREL},
    [0x30] = {"bmi", AM_REL},   [0x31] = {"and", AM_INDY},  [0x32] = {"and", AM_IZP},
    [0x34] = {"bit", AM_ZPX},   [0x35] = {"and", AM_ZPX},   [0x36] = {"rol", AM_ZPX},
    [0x37] = {"rmb3", AM_ZP},   [0x38] = {"sec", AM_IMP},   [0x39] = {"and", AM_ABSY},
    [0x3A] = {"dec", AM_ACC},   [0x3C] = {"bit", AM_ABSX},  [0x3D] = {"and", AM_ABSX},
    [0x3E] = {"rol", AM_ABSX},  [0x3F] = {"bbr3", AM_ZPREL},
    [0x40] = {"rti", AM_IMP},   [0x41] = {"eor", AM_INDX},  [0x42] = {"say", AM_IMP},
    [0x43] = {"tma", AM_IMM},   [0x44] = {"bsr", AM_REL},   [0x45] = {"eor", AM_ZP},
    [0x46] = {"lsr", AM_ZP},    [0x47] = {"rmb4", AM_ZP},   [0x48] = {"pha", AM_IMP},
    [0x49] = {"eor", AM_IMM},   [0x4A] = {"lsr", AM_ACC},   [0x4C] = {"jmp", AM_ABS},
    [0x4D] = {"eor", AM_ABS},   [0x4E] = {"lsr", AM_ABS},   [0x4F] = {"bbr4", AM_ZPREL},
    [0x50] = {"bvc", AM_REL},   [0x51] = {"eor", AM_INDY},  [0x52] = {"eor", AM_IZP},
    [0x53] = {"tam", AM_IMM},   [0x54] = {"csl", AM_IMP},   [0x55] = {"eor", AM_ZPX},
    [0x56] = {"lsr", AM_ZPX},   [0x57] = {"rmb5", AM_ZP},   [0x58] = {"cli", AM_IMP},
    [0x59] = {"eor", AM_ABSY},  [0x5A] = {"phy", AM_IMP},   [0x5D] = {"eor", AM_ABSX},
    [0x5E] = {"lsr", AM_ABSX},  [0x5F] = {"bbr5", AM_ZPREL},
    [0x60] = {"rts", AM_IMP},   [0x61] = {"adc", AM_INDX},  [0x62] = {"cla", AM_IMP},
    [0x64] = {"stz", AM_ZP},    [0x65] = {"adc", AM_ZP},    [0x66] = {"ror", AM_ZP},
    [0x67] = {"rmb6", AM_ZP},   [0x68] = {"pla", AM_IMP},   [0x69] = {"adc", AM_IMM},
    [0x6A] = {"ror", AM_ACC},   [0x6C] = {"jmp", AM_IND},   [0x6D] = {"adc", AM_ABS},
    [0x6E] = {"ror", AM_ABS},   [0x6F] = {"bbr6", AM_ZPREL},
    [0x70] = {"bvs", AM_REL},   [0x71] = {"adc", AM_INDY},  [0x72] = {"adc", AM_IZP},
    [0x73] = {"tii", AM_BLK},   [0x74] = {"stz", AM_ZPX},   [0x75] = {"adc", AM_ZPX},
    [0x76] = {"ror", AM_ZPX},   [0x77] = {"rmb7", AM_ZP},   [0x78] = {"sei", AM_IMP},
    [0x79] = {"adc", AM_ABSY},  [0x7A] = {"ply", AM_IMP},   [0x7C] = {"jmp", AM_IABSX},
    [0x7D] = {"adc", AM_ABSX},  [0x7E] = {"ror", AM_ABSX},  [0x7F] = {"bbr7", AM_ZPREL},
    [0x80] = {"bra", AM_REL},   [0x81] = {"sta", AM_INDX},  [0x82] = {"clx", AM_IMP},
    [0x83] = {"tst", AM_IMZP},  [0x84] = {"sty", AM_ZP},    [0x85] = {"sta", AM_ZP},
    [0x86] = {"stx", AM_ZP},    [0x87] = {"smb0", AM_ZP},   [0x88] = {"dey", AM_IMP},
    [0x89] = {"bit", AM_IMM},   [0x8A] = {"txa", AM_IMP},   [0x8C] = {"sty", AM_ABS},
    [0x8D] = {"sta", AM_ABS},   [0x8E] = {"stx", AM_ABS},   [0x8F] = {"bbs0", AM_ZPREL},
    [0x90] = {"bcc", AM_REL},   [0x91] = {"sta", AM_INDY},  [0x92] = {"sta", AM_IZP},
    [0x93] = {"tst", AM_IMAB},  [0x94] = {"sty", AM_ZPX},   [0x95] = {"sta", AM_ZPX},
    [0x96] = {"stx", AM_ZPY},   [0x97] = {"smb1", AM_ZP},   [0x98] = {"tya", AM_IMP},
    [0x99] = {"sta", AM_ABSY},  [0x9A] = {"txs", AM_IMP},   [0x9C] = {"stz", AM_ABS},
    [0x9D] = {"sta", AM_ABSX},  [0x9E] = {"stz", AM_ABSX},  [0x9F] = {"bbs1", AM_ZPREL},
    [0xA0] = {"ldy", AM_IMM},   [0xA1] = {"lda", AM_INDX},  [0xA2] = {"ldx", AM_IMM},
    [0xA3] = {"tst", AM_IMZPX}, [0xA4] = {"ldy", AM_ZP},    [0xA5] = {"lda", AM_ZP},
    [0xA6] = {"ldx", AM_ZP},    [0xA7] = {"smb2", AM_ZP},   [0xA8] = {"tay", AM_IMP},
    [0xA9] = {"lda", AM_IMM},   [0xAA] = {"tax", AM_IMP},   [0xAC] = {"ldy", AM_ABS},
    [0xAD] = {"lda", AM_ABS},   [0xAE] = {"ldx", AM_ABS},   [0xAF] = {"bbs2", AM_ZPREL},
    [0xB0] = {"bcs", AM_REL},   [0xB1] = {"lda", AM_INDY},  [0xB2] = {"lda", AM_IZP},
    [0xB3] = {"tst", AM_IMABX}, [0xB4] = {"ldy", AM_ZPX},   [0xB5] = {"lda", AM_ZPX},
    [0xB6] = {"ldx", AM_ZPY},   [0xB7] = {"smb3", AM_ZP},   [0xB8] = {"clv", AM_IMP},
    [0xB9] = {"lda", AM_ABSY},  [0xBA] = {"tsx", AM_IMP},   [0xBC] = {"ldy", AM_ABSX},
    [0xBD] = {"lda", AM_ABSX},  [0xBE] = {"ldx", AM_ABSY},  [0xBF] = {"bbs3", AM_ZPREL},
    [0xC0] = {"cpy", AM_IMM},   [0xC1] = {"cmp", AM_INDX},  [0xC2] = {"cly", AM_IMP},
    [0xC3] = {"tdd", AM_BLK},   [0xC4] = {"cpy", AM_ZP},    [0xC5] = {"cmp", AM_ZP},
    [0xC6] = {"dec", AM_ZP},    [0xC7] = {"smb4", AM_ZP},   [0xC8] = {"iny", AM_IMP},
    [0xC9] = {"cmp", AM_IMM},   [0xCA] = {"dex", AM_IMP},   [0xCC] = {"cpy", AM_ABS},
    [0xCD] = {"cmp", AM_ABS},   [0xCE] = {"dec", AM_ABS},   [0xCF] = {"bbs4", AM_ZPREL},
    [0xD0] = {"bne", AM_REL},   [0xD1] = {"cmp", AM_INDY},  [0xD2] = {"cmp", AM_IZP},
    [0xD3] = {"tin", AM_BLK},   [0xD4] = {"csh", AM_IMP},   [0xD5] = {"cmp", AM_ZPX},
    [0xD6] = {"dec", AM_ZPX},   [0xD7] = {"smb5", AM_ZP},   [0xD8] = {"cld", AM_IMP},
    [0xD9] = {"cmp", AM_ABSY},  [0xDA] = {"phx", AM_IMP},   [0xDD] = {"cmp", AM_ABSX},
    [0xDE] = {"dec", AM_ABSX},  [0xDF] = {"bbs5", AM_ZPREL},
    [0xE0] = {"cpx", AM_IMM},   [0xE1] = {"sbc", AM_INDX},  [0xE3] = {"tia", AM_BLK},
    [0xE4] = {"cpx", AM_ZP},    [0xE5] = {"sbc", AM_ZP},    [0xE6] = {"inc", AM_ZP},
    [0xE7] = {"smb6", AM_ZP},   [0xE8] = {"inx", AM_IMP},   [0xE9] = {"sbc", AM_IMM},
    [0xEA] = {"nop", AM_IMP},   [0xEC] = {"cpx", AM_ABS},   [0xED] = {"sbc", AM_ABS},
    [0xEE] = {"inc", AM_ABS},   [0xEF] = {"bbs6", AM_ZPREL},
    [0xF0] = {"beq", AM_REL},   [0xF1] = {"sbc", AM_INDY},  [0xF2] = {"sbc", AM_IZP},
    [0xF3] = {"tai", AM_BLK},   [0xF4] = {"set", AM_IMP},   [0xF5] = {"sbc", AM_ZPX},
    [0xF6] = {"inc", AM_ZPX},   [0xF7] = {"smb7", AM_ZP},   [0xF8] = {"sed", AM_IMP},
    [0xF9] = {"sbc", AM_ABSY},  [0xFA] = {"plx", AM_IMP},   [0xFD] = {"sbc", AM_ABSX},
    [0xFE] = {"inc", AM_ABSX},  [0xFF] = {"bbs7", AM_ZPREL}
};

void huc6280_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    HUC6280_CPU *cpu = (HUC6280_CPU*)context;

    uint8_t op = mem_read(cpu, cpu->pc);
    const DisasmEntry *e = &g_disasm[op];
    uint8_t b1 = mem_read(cpu, (uint16_t)(cpu->pc + 1));
    uint8_t b2 = mem_read(cpu, (uint16_t)(cpu->pc + 2));
    uint16_t w1 = (uint16_t)(b1 | (b2 << 8));

    if (!e->name) {
        snprintf(buf, buf_len, "unknown (0x%02X)", op);
        return;
    }

    switch (e->mode) {
        case AM_IMP:   snprintf(buf, buf_len, "%s", e->name); break;
        case AM_ACC:   snprintf(buf, buf_len, "%s   a", e->name); break;
        case AM_IMM:   snprintf(buf, buf_len, "%s   #$0x%02X", e->name, b1); break;
        case AM_ZP:    snprintf(buf, buf_len, "%s   $0x%02X", e->name, b1); break;
        case AM_ZPX:   snprintf(buf, buf_len, "%s   $0x%02X,X", e->name, b1); break;
        case AM_ZPY:   snprintf(buf, buf_len, "%s   $0x%02X,Y", e->name, b1); break;
        case AM_ABS:   snprintf(buf, buf_len, "%s   $0x%04X", e->name, w1); break;
        case AM_ABSX:  snprintf(buf, buf_len, "%s   $0x%04X,X", e->name, w1); break;
        case AM_ABSY:  snprintf(buf, buf_len, "%s   $0x%04X,Y", e->name, w1); break;
        case AM_IND:   snprintf(buf, buf_len, "%s   ($0x%04X)", e->name, w1); break;
        case AM_IABSX: snprintf(buf, buf_len, "%s   ($0x%04X,X)", e->name, w1); break;
        case AM_INDX:  snprintf(buf, buf_len, "%s   ($0x%02X,X)", e->name, b1); break;
        case AM_INDY:  snprintf(buf, buf_len, "%s   ($0x%02X),Y", e->name, b1); break;
        case AM_IZP:   snprintf(buf, buf_len, "%s   ($0x%02X)", e->name, b1); break;
        case AM_REL:   snprintf(buf, buf_len, "%s   %+d", e->name, (int8_t)b1); break;
        case AM_ZPREL: snprintf(buf, buf_len, "%s  $0x%02X,%+d", e->name, b1, (int8_t)b2); break;
        case AM_BLK: {
                uint8_t b3 = mem_read(cpu, (uint16_t)(cpu->pc + 3));
                uint8_t b4 = mem_read(cpu, (uint16_t)(cpu->pc + 4));
                uint8_t b5 = mem_read(cpu, (uint16_t)(cpu->pc + 5));
                uint8_t b6 = mem_read(cpu, (uint16_t)(cpu->pc + 6));
                snprintf(buf, buf_len, "%s   $0x%04X,$0x%04X,$0x%04X",
                         e->name,
                         (uint16_t)(b1 | (b2 << 8)),
                         (uint16_t)(b3 | (b4 << 8)),
                         (uint16_t)(b5 | (b6 << 8)));
            }
            break;
        case AM_IMZP:  snprintf(buf, buf_len, "%s   #$0x%02X,$0x%02X", e->name, b1, b2); break;
        case AM_IMZPX: snprintf(buf, buf_len, "%s   #$0x%02X,$0x%02X,X", e->name, b1, b2); break;
        case AM_IMAB: {
                uint8_t b3 = mem_read(cpu, (uint16_t)(cpu->pc + 3));
                snprintf(buf, buf_len, "%s   #$0x%02X,$0x%04X", e->name, b1, (uint16_t)(b2 | (b3 << 8)));
            }
            break;
        case AM_IMABX: {
                uint8_t b3 = mem_read(cpu, (uint16_t)(cpu->pc + 3));
                snprintf(buf, buf_len, "%s   #$0x%02X,$0x%04X,X", e->name, b1, (uint16_t)(b2 | (b3 << 8)));
            }
            break;
        default:
            snprintf(buf, buf_len, "unknown (0x%02X)", op);
            break;
    }
}
