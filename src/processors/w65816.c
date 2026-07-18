#include "w65816.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// WDC 65C816 core.
// Full 16 MB (24-bit) flat address space, allocated as part of the calloc'd
// context. All bus accesses are masked to 24 bits.

#define W65816_MEM_SIZE (16u * 1024u * 1024u)

#define FLAG_C 0x01 // Carry
#define FLAG_Z 0x02 // Zero
#define FLAG_I 0x04 // IRQ disable
#define FLAG_D 0x08 // Decimal
#define FLAG_X 0x10 // Index width (native) / Break (emulation)
#define FLAG_M 0x20 // Accumulator width (native) / unused (emulation)
#define FLAG_V 0x40 // Overflow
#define FLAG_N 0x80 // Negative

typedef struct W65816_CPU {
    uint16_t c;    // 16-bit accumulator: A = low byte, B = high byte
    uint16_t x;
    uint16_t y;
    uint16_t sp;   // full 16-bit in native mode, forced to page 1 in emulation
    uint16_t d;    // direct page register
    uint16_t pc;
    uint8_t p;
    uint8_t dbr;   // data bank register
    uint8_t pbr;   // program bank register
    uint8_t e;     // emulation flag
    int halted;    // set by BRK/WAI/STP
    uint32_t ticks;
    uint8_t mem[W65816_MEM_SIZE];
} W65816_CPU;

// --- Flag helpers -----------------------------------------------------------

static void set_flag(W65816_CPU *cpu, uint8_t flag, int cond) {
    if (cond) cpu->p |= flag; else cpu->p = (uint8_t)(cpu->p & ~flag);
}

static int get_flag(const W65816_CPU *cpu, uint8_t flag) {
    return (cpu->p & flag) ? 1 : 0;
}

// Accumulator/memory operations are 8-bit when M is set (or in emulation mode)
static int m8(const W65816_CPU *cpu) {
    return cpu->e || (cpu->p & FLAG_M);
}

// Index register operations are 8-bit when X is set (or in emulation mode)
static int x8(const W65816_CPU *cpu) {
    return cpu->e || (cpu->p & FLAG_X);
}

static void set_nz(W65816_CPU *cpu, uint16_t v, int is8) {
    if (is8) {
        set_flag(cpu, FLAG_Z, (v & 0xFF) == 0);
        set_flag(cpu, FLAG_N, (v & 0x80) != 0);
    } else {
        set_flag(cpu, FLAG_Z, v == 0);
        set_flag(cpu, FLAG_N, (v & 0x8000) != 0);
    }
}

// When the index registers are 8 bits wide their high bytes are forced to zero
static void update_index_width(W65816_CPU *cpu) {
    if (x8(cpu)) {
        cpu->x = (uint16_t)(cpu->x & 0xFF);
        cpu->y = (uint16_t)(cpu->y & 0xFF);
    }
}

// --- Bus access -------------------------------------------------------------

static uint8_t rd8(W65816_CPU *cpu, uint32_t addr) {
    return cpu->mem[addr & 0xFFFFFF];
}

static void wr8(W65816_CPU *cpu, uint32_t addr, uint8_t val) {
    cpu->mem[addr & 0xFFFFFF] = val;
}

static uint16_t rd16(W65816_CPU *cpu, uint32_t addr) {
    return (uint16_t)(rd8(cpu, addr) | ((uint16_t)rd8(cpu, addr + 1) << 8));
}

// 16-bit pointer read wrapping within bank 0 (direct page / stack pointers)
static uint16_t rd16b0(W65816_CPU *cpu, uint16_t addr) {
    return (uint16_t)(rd8(cpu, addr) | ((uint16_t)rd8(cpu, (uint16_t)(addr + 1)) << 8));
}

// 24-bit pointer read wrapping within bank 0 (for [dp] indirect long)
static uint32_t rd24b0(W65816_CPU *cpu, uint16_t addr) {
    uint32_t lo = rd8(cpu, addr);
    uint32_t mid = rd8(cpu, (uint16_t)(addr + 1));
    uint32_t hi = rd8(cpu, (uint16_t)(addr + 2));
    return lo | (mid << 8) | (hi << 16);
}

static uint8_t fetch8(W65816_CPU *cpu) {
    uint8_t v = rd8(cpu, ((uint32_t)cpu->pbr << 16) | cpu->pc);
    cpu->pc = (uint16_t)(cpu->pc + 1);
    return v;
}

static uint16_t fetch16(W65816_CPU *cpu) {
    uint16_t lo = fetch8(cpu);
    uint16_t hi = fetch8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

static uint32_t fetch24(W65816_CPU *cpu) {
    uint32_t lo = fetch16(cpu);
    uint32_t hi = fetch8(cpu);
    return lo | (hi << 16);
}

// --- Stack ------------------------------------------------------------------

static void push8(W65816_CPU *cpu, uint8_t val) {
    wr8(cpu, cpu->sp, val);
    if (cpu->e) cpu->sp = (uint16_t)(0x0100 | ((cpu->sp - 1) & 0xFF));
    else cpu->sp = (uint16_t)(cpu->sp - 1);
}

static uint8_t pop8(W65816_CPU *cpu) {
    if (cpu->e) cpu->sp = (uint16_t)(0x0100 | ((cpu->sp + 1) & 0xFF));
    else cpu->sp = (uint16_t)(cpu->sp + 1);
    return rd8(cpu, cpu->sp);
}

static void push16(W65816_CPU *cpu, uint16_t val) {
    push8(cpu, (uint8_t)(val >> 8));
    push8(cpu, (uint8_t)(val & 0xFF));
}

static uint16_t pop16(W65816_CPU *cpu) {
    uint16_t lo = pop8(cpu);
    uint16_t hi = pop8(cpu);
    return (uint16_t)(lo | (hi << 8));
}

// --- Accumulator access -----------------------------------------------------

static uint16_t get_a(const W65816_CPU *cpu) {
    return m8(cpu) ? (uint16_t)(cpu->c & 0xFF) : cpu->c;
}

static void set_a(W65816_CPU *cpu, uint16_t v) {
    if (m8(cpu)) cpu->c = (uint16_t)((cpu->c & 0xFF00) | (v & 0xFF));
    else cpu->c = v;
}

// --- Width-aware memory access ----------------------------------------------

static uint16_t ldm(W65816_CPU *cpu, uint32_t addr) {
    return m8(cpu) ? rd8(cpu, addr) : rd16(cpu, addr);
}

static uint16_t ldxw(W65816_CPU *cpu, uint32_t addr) {
    return x8(cpu) ? rd8(cpu, addr) : rd16(cpu, addr);
}

static void stm(W65816_CPU *cpu, uint32_t addr, uint16_t val) {
    wr8(cpu, addr, (uint8_t)(val & 0xFF));
    if (!m8(cpu)) wr8(cpu, addr + 1, (uint8_t)(val >> 8));
}

static void stxw(W65816_CPU *cpu, uint32_t addr, uint16_t val) {
    wr8(cpu, addr, (uint8_t)(val & 0xFF));
    if (!x8(cpu)) wr8(cpu, addr + 1, (uint8_t)(val >> 8));
}

static uint16_t imm_m(W65816_CPU *cpu) {
    return m8(cpu) ? fetch8(cpu) : fetch16(cpu);
}

static uint16_t imm_x(W65816_CPU *cpu) {
    return x8(cpu) ? fetch8(cpu) : fetch16(cpu);
}

// --- Effective address calculation (returns 24-bit address) -----------------

static uint32_t ea_abs(W65816_CPU *cpu) {
    return ((uint32_t)cpu->dbr << 16) | fetch16(cpu);
}

static uint32_t ea_absx(W65816_CPU *cpu) {
    return ((((uint32_t)cpu->dbr << 16) | fetch16(cpu)) + cpu->x) & 0xFFFFFF;
}

static uint32_t ea_absy(W65816_CPU *cpu) {
    return ((((uint32_t)cpu->dbr << 16) | fetch16(cpu)) + cpu->y) & 0xFFFFFF;
}

static uint32_t ea_absl(W65816_CPU *cpu) {
    return fetch24(cpu);
}

static uint32_t ea_abslx(W65816_CPU *cpu) {
    return (fetch24(cpu) + cpu->x) & 0xFFFFFF;
}

static uint32_t ea_dp(W65816_CPU *cpu) {
    return (uint16_t)(cpu->d + fetch8(cpu));
}

static uint32_t ea_dpx(W65816_CPU *cpu) {
    return (uint16_t)(cpu->d + fetch8(cpu) + cpu->x);
}

static uint32_t ea_dpy(W65816_CPU *cpu) {
    return (uint16_t)(cpu->d + fetch8(cpu) + cpu->y);
}

static uint32_t ea_dpi(W65816_CPU *cpu) { // (dp)
    uint16_t ptr = (uint16_t)(cpu->d + fetch8(cpu));
    return ((uint32_t)cpu->dbr << 16) | rd16b0(cpu, ptr);
}

static uint32_t ea_dpxi(W65816_CPU *cpu) { // (dp,X)
    uint16_t ptr = (uint16_t)(cpu->d + fetch8(cpu) + cpu->x);
    return ((uint32_t)cpu->dbr << 16) | rd16b0(cpu, ptr);
}

static uint32_t ea_dpiy(W65816_CPU *cpu) { // (dp),Y
    uint16_t ptr = (uint16_t)(cpu->d + fetch8(cpu));
    return ((((uint32_t)cpu->dbr << 16) | rd16b0(cpu, ptr)) + cpu->y) & 0xFFFFFF;
}

static uint32_t ea_dpil(W65816_CPU *cpu) { // [dp]
    uint16_t ptr = (uint16_t)(cpu->d + fetch8(cpu));
    return rd24b0(cpu, ptr);
}

static uint32_t ea_dpily(W65816_CPU *cpu) { // [dp],Y
    uint16_t ptr = (uint16_t)(cpu->d + fetch8(cpu));
    return (rd24b0(cpu, ptr) + cpu->y) & 0xFFFFFF;
}

static uint32_t ea_sr(W65816_CPU *cpu) { // sr,S
    return (uint16_t)(cpu->sp + fetch8(cpu));
}

static uint32_t ea_sriy(W65816_CPU *cpu) { // (sr,S),Y
    uint16_t ptr = (uint16_t)(cpu->sp + fetch8(cpu));
    return ((((uint32_t)cpu->dbr << 16) | rd16b0(cpu, ptr)) + cpu->y) & 0xFFFFFF;
}

// --- ALU operations ---------------------------------------------------------

static void op_lda(W65816_CPU *cpu, uint16_t v) {
    set_a(cpu, v);
    set_nz(cpu, v, m8(cpu));
}

static void op_ora(W65816_CPU *cpu, uint16_t v) {
    uint16_t a = (uint16_t)(get_a(cpu) | v);
    set_a(cpu, a);
    set_nz(cpu, a, m8(cpu));
}

static void op_and(W65816_CPU *cpu, uint16_t v) {
    uint16_t a = (uint16_t)(get_a(cpu) & v);
    set_a(cpu, a);
    set_nz(cpu, a, m8(cpu));
}

static void op_eor(W65816_CPU *cpu, uint16_t v) {
    uint16_t a = (uint16_t)(get_a(cpu) ^ v);
    set_a(cpu, a);
    set_nz(cpu, a, m8(cpu));
}

// BCD addition on 'nibbles' BCD digits, returns adjusted result and carry-out
static uint32_t bcd_add(uint32_t a, uint32_t b, uint32_t carry, int nibbles, uint32_t *cout) {
    uint32_t r = 0;
    int i;
    for (i = 0; i < nibbles; ++i) {
        uint32_t t = ((a >> (4 * i)) & 0xF) + ((b >> (4 * i)) & 0xF) + carry;
        if (t > 9) { t = (t + 6) & 0xF; carry = 1; } else { carry = 0; }
        r |= t << (4 * i);
    }
    *cout = carry;
    return r;
}

static uint32_t bcd_sub(uint32_t a, uint32_t b, uint32_t borrow, int nibbles, uint32_t *bout) {
    uint32_t r = 0;
    int i;
    for (i = 0; i < nibbles; ++i) {
        int t = (int)((a >> (4 * i)) & 0xF) - (int)((b >> (4 * i)) & 0xF) - (int)borrow;
        if (t < 0) { t += 10; borrow = 1; } else { borrow = 0; }
        r |= (uint32_t)t << (4 * i);
    }
    *bout = borrow;
    return r;
}

static void op_adc(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    uint32_t mask = is8 ? 0xFFu : 0xFFFFu;
    uint32_t sign = is8 ? 0x80u : 0x8000u;
    uint32_t a = get_a(cpu);
    uint32_t cin = (uint32_t)get_flag(cpu, FLAG_C);
    uint32_t bin = a + v + cin;
    uint32_t r;
    if (get_flag(cpu, FLAG_D)) {
        uint32_t cout;
        r = bcd_add(a, v, cin, is8 ? 2 : 4, &cout);
        set_flag(cpu, FLAG_C, cout != 0);
    } else {
        r = bin;
        set_flag(cpu, FLAG_C, r > mask);
    }
    set_flag(cpu, FLAG_V, ((~(a ^ v) & (a ^ bin)) & sign) != 0);
    set_a(cpu, (uint16_t)(r & mask));
    set_nz(cpu, (uint16_t)(r & mask), is8);
}

static void op_sbc(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    uint32_t mask = is8 ? 0xFFu : 0xFFFFu;
    uint32_t sign = is8 ? 0x80u : 0x8000u;
    uint32_t a = get_a(cpu);
    uint32_t cin = (uint32_t)get_flag(cpu, FLAG_C);
    uint32_t bin = a + ((~(uint32_t)v) & mask) + cin;
    uint32_t r;
    if (get_flag(cpu, FLAG_D)) {
        uint32_t bout;
        r = bcd_sub(a, v, cin ? 0u : 1u, is8 ? 2 : 4, &bout);
        set_flag(cpu, FLAG_C, bout == 0);
    } else {
        r = bin;
        set_flag(cpu, FLAG_C, bin > mask);
    }
    set_flag(cpu, FLAG_V, (((a ^ v) & (a ^ bin)) & sign) != 0);
    set_a(cpu, (uint16_t)(r & mask));
    set_nz(cpu, (uint16_t)(r & mask), is8);
}

static void cmp_gen(W65816_CPU *cpu, uint16_t reg, uint16_t v, int is8) {
    uint32_t diff = (uint32_t)reg - v;
    set_flag(cpu, FLAG_C, reg >= v);
    set_nz(cpu, (uint16_t)diff, is8);
}

static void op_cmp(W65816_CPU *cpu, uint16_t v) {
    cmp_gen(cpu, get_a(cpu), v, m8(cpu));
}

static void op_cpx(W65816_CPU *cpu, uint16_t v) {
    cmp_gen(cpu, cpu->x, v, x8(cpu));
}

static void op_cpy(W65816_CPU *cpu, uint16_t v) {
    cmp_gen(cpu, cpu->y, v, x8(cpu));
}

static void op_bit(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    set_flag(cpu, FLAG_Z, (get_a(cpu) & v) == 0);
    set_flag(cpu, FLAG_N, (v & (is8 ? 0x80u : 0x8000u)) != 0);
    set_flag(cpu, FLAG_V, (v & (is8 ? 0x40u : 0x4000u)) != 0);
}

// --- Read-modify-write value operations -------------------------------------

static uint16_t v_asl(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    uint32_t mask = is8 ? 0xFFu : 0xFFFFu;
    set_flag(cpu, FLAG_C, (v & (is8 ? 0x80u : 0x8000u)) != 0);
    v = (uint16_t)(((uint32_t)v << 1) & mask);
    set_nz(cpu, v, is8);
    return v;
}

static uint16_t v_lsr(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    set_flag(cpu, FLAG_C, (v & 0x01) != 0);
    v = (uint16_t)(v >> 1);
    set_nz(cpu, v, is8);
    return v;
}

static uint16_t v_rol(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    uint32_t mask = is8 ? 0xFFu : 0xFFFFu;
    uint32_t old_c = (uint32_t)get_flag(cpu, FLAG_C);
    set_flag(cpu, FLAG_C, (v & (is8 ? 0x80u : 0x8000u)) != 0);
    v = (uint16_t)((((uint32_t)v << 1) | old_c) & mask);
    set_nz(cpu, v, is8);
    return v;
}

static uint16_t v_ror(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    uint16_t old_c = (uint16_t)get_flag(cpu, FLAG_C);
    set_flag(cpu, FLAG_C, (v & 0x01) != 0);
    v = (uint16_t)((v >> 1) | (old_c ? (is8 ? 0x80u : 0x8000u) : 0u));
    set_nz(cpu, v, is8);
    return v;
}

static uint16_t v_inc(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    v = (uint16_t)((v + 1) & (is8 ? 0xFFu : 0xFFFFu));
    set_nz(cpu, v, is8);
    return v;
}

static uint16_t v_dec(W65816_CPU *cpu, uint16_t v) {
    int is8 = m8(cpu);
    v = (uint16_t)((v - 1) & (is8 ? 0xFFu : 0xFFFFu));
    set_nz(cpu, v, is8);
    return v;
}

static uint16_t v_tsb(W65816_CPU *cpu, uint16_t v) {
    uint16_t a = get_a(cpu);
    set_flag(cpu, FLAG_Z, (a & v) == 0);
    return (uint16_t)(v | a);
}

static uint16_t v_trb(W65816_CPU *cpu, uint16_t v) {
    uint16_t a = get_a(cpu);
    set_flag(cpu, FLAG_Z, (a & v) == 0);
    return (uint16_t)(v & (uint16_t)~a);
}

static void branch(W65816_CPU *cpu, int cond) {
    int8_t rel = (int8_t)fetch8(cpu);
    if (cond) cpu->pc = (uint16_t)(cpu->pc + rel);
}

// --- Lifecycle --------------------------------------------------------------

void* w65816_create(void) {
    W65816_CPU *cpu = (W65816_CPU*)calloc(1, sizeof(W65816_CPU));
    return cpu;
}

void w65816_destroy(void *context) {
    free(context);
}

int w65816_init(void *context) {
    if (!context) return -1;
    {
        W65816_CPU *cpu = (W65816_CPU*)context;
        memset(cpu->mem, 0, W65816_MEM_SIZE);
        cpu->c = 0;
        cpu->x = 0;
        cpu->y = 0;
        cpu->sp = 0x01FD;
        cpu->d = 0;
        cpu->pc = 0; // reset at PC = 0
        cpu->p = FLAG_M | FLAG_X | FLAG_I;
        cpu->dbr = 0;
        cpu->pbr = 0;
        cpu->e = 1;  // reset always enters emulation mode
        cpu->halted = 0;
        cpu->ticks = 0;
    }
    return 0;
}

int w65816_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    {
        W65816_CPU *cpu = (W65816_CPU*)context;
        uint32_t addr = address & 0xFFFFFF;
        size_t space = (size_t)(W65816_MEM_SIZE - addr);
        size_t copy_len = size < space ? size : space;
        memcpy(cpu->mem + addr, data, copy_len);
        cpu->pc = (uint16_t)(addr & 0xFFFF);
        cpu->pbr = (uint8_t)(addr >> 16);
    }
    return 0;
}

// --- Execution --------------------------------------------------------------

int w65816_step(void *context) {
    W65816_CPU *cpu = (W65816_CPU*)context;
    uint8_t op;
    if (!cpu) return -1;
    if (cpu->halted) return 1;

    op = fetch8(cpu);
    cpu->ticks++;

    switch (op) {
        // --- ORA ---
        case 0x01: op_ora(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0x03: op_ora(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0x05: op_ora(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0x07: op_ora(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0x09: op_ora(cpu, imm_m(cpu)); break;
        case 0x0D: op_ora(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0x0F: op_ora(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0x11: op_ora(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0x12: op_ora(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0x13: op_ora(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0x15: op_ora(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0x17: op_ora(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0x19: op_ora(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0x1D: op_ora(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0x1F: op_ora(cpu, ldm(cpu, ea_abslx(cpu))); break;

        // --- AND ---
        case 0x21: op_and(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0x23: op_and(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0x25: op_and(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0x27: op_and(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0x29: op_and(cpu, imm_m(cpu)); break;
        case 0x2D: op_and(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0x2F: op_and(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0x31: op_and(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0x32: op_and(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0x33: op_and(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0x35: op_and(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0x37: op_and(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0x39: op_and(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0x3D: op_and(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0x3F: op_and(cpu, ldm(cpu, ea_abslx(cpu))); break;

        // --- EOR ---
        case 0x41: op_eor(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0x43: op_eor(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0x45: op_eor(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0x47: op_eor(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0x49: op_eor(cpu, imm_m(cpu)); break;
        case 0x4D: op_eor(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0x4F: op_eor(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0x51: op_eor(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0x52: op_eor(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0x53: op_eor(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0x55: op_eor(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0x57: op_eor(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0x59: op_eor(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0x5D: op_eor(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0x5F: op_eor(cpu, ldm(cpu, ea_abslx(cpu))); break;

        // --- ADC ---
        case 0x61: op_adc(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0x63: op_adc(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0x65: op_adc(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0x67: op_adc(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0x69: op_adc(cpu, imm_m(cpu)); break;
        case 0x6D: op_adc(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0x6F: op_adc(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0x71: op_adc(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0x72: op_adc(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0x73: op_adc(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0x75: op_adc(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0x77: op_adc(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0x79: op_adc(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0x7D: op_adc(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0x7F: op_adc(cpu, ldm(cpu, ea_abslx(cpu))); break;

        // --- SBC ---
        case 0xE1: op_sbc(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0xE3: op_sbc(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0xE5: op_sbc(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0xE7: op_sbc(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0xE9: op_sbc(cpu, imm_m(cpu)); break;
        case 0xED: op_sbc(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0xEF: op_sbc(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0xF1: op_sbc(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0xF2: op_sbc(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0xF3: op_sbc(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0xF5: op_sbc(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0xF7: op_sbc(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0xF9: op_sbc(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0xFD: op_sbc(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0xFF: op_sbc(cpu, ldm(cpu, ea_abslx(cpu))); break;

        // --- CMP / CPX / CPY ---
        case 0xC1: op_cmp(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0xC3: op_cmp(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0xC5: op_cmp(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0xC7: op_cmp(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0xC9: op_cmp(cpu, imm_m(cpu)); break;
        case 0xCD: op_cmp(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0xCF: op_cmp(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0xD1: op_cmp(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0xD2: op_cmp(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0xD3: op_cmp(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0xD5: op_cmp(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0xD7: op_cmp(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0xD9: op_cmp(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0xDD: op_cmp(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0xDF: op_cmp(cpu, ldm(cpu, ea_abslx(cpu))); break;
        case 0xE0: op_cpx(cpu, imm_x(cpu)); break;
        case 0xE4: op_cpx(cpu, ldxw(cpu, ea_dp(cpu))); break;
        case 0xEC: op_cpx(cpu, ldxw(cpu, ea_abs(cpu))); break;
        case 0xC0: op_cpy(cpu, imm_x(cpu)); break;
        case 0xC4: op_cpy(cpu, ldxw(cpu, ea_dp(cpu))); break;
        case 0xCC: op_cpy(cpu, ldxw(cpu, ea_abs(cpu))); break;

        // --- BIT ---
        case 0x24: op_bit(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0x2C: op_bit(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0x34: op_bit(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0x3C: op_bit(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0x89: // BIT immediate only affects Z
            set_flag(cpu, FLAG_Z, (get_a(cpu) & imm_m(cpu)) == 0);
            break;

        // --- LDA ---
        case 0xA1: op_lda(cpu, ldm(cpu, ea_dpxi(cpu))); break;
        case 0xA3: op_lda(cpu, ldm(cpu, ea_sr(cpu))); break;
        case 0xA5: op_lda(cpu, ldm(cpu, ea_dp(cpu))); break;
        case 0xA7: op_lda(cpu, ldm(cpu, ea_dpil(cpu))); break;
        case 0xA9: op_lda(cpu, imm_m(cpu)); break;
        case 0xAD: op_lda(cpu, ldm(cpu, ea_abs(cpu))); break;
        case 0xAF: op_lda(cpu, ldm(cpu, ea_absl(cpu))); break;
        case 0xB1: op_lda(cpu, ldm(cpu, ea_dpiy(cpu))); break;
        case 0xB2: op_lda(cpu, ldm(cpu, ea_dpi(cpu))); break;
        case 0xB3: op_lda(cpu, ldm(cpu, ea_sriy(cpu))); break;
        case 0xB5: op_lda(cpu, ldm(cpu, ea_dpx(cpu))); break;
        case 0xB7: op_lda(cpu, ldm(cpu, ea_dpily(cpu))); break;
        case 0xB9: op_lda(cpu, ldm(cpu, ea_absy(cpu))); break;
        case 0xBD: op_lda(cpu, ldm(cpu, ea_absx(cpu))); break;
        case 0xBF: op_lda(cpu, ldm(cpu, ea_abslx(cpu))); break;

        // --- LDX / LDY ---
        case 0xA2: cpu->x = imm_x(cpu); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xA6: cpu->x = ldxw(cpu, ea_dp(cpu)); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xAE: cpu->x = ldxw(cpu, ea_abs(cpu)); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xB6: cpu->x = ldxw(cpu, ea_dpy(cpu)); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xBE: cpu->x = ldxw(cpu, ea_absy(cpu)); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xA0: cpu->y = imm_x(cpu); set_nz(cpu, cpu->y, x8(cpu)); break;
        case 0xA4: cpu->y = ldxw(cpu, ea_dp(cpu)); set_nz(cpu, cpu->y, x8(cpu)); break;
        case 0xAC: cpu->y = ldxw(cpu, ea_abs(cpu)); set_nz(cpu, cpu->y, x8(cpu)); break;
        case 0xB4: cpu->y = ldxw(cpu, ea_dpx(cpu)); set_nz(cpu, cpu->y, x8(cpu)); break;
        case 0xBC: cpu->y = ldxw(cpu, ea_absx(cpu)); set_nz(cpu, cpu->y, x8(cpu)); break;

        // --- STA ---
        case 0x81: stm(cpu, ea_dpxi(cpu), get_a(cpu)); break;
        case 0x83: stm(cpu, ea_sr(cpu), get_a(cpu)); break;
        case 0x85: stm(cpu, ea_dp(cpu), get_a(cpu)); break;
        case 0x87: stm(cpu, ea_dpil(cpu), get_a(cpu)); break;
        case 0x8D: stm(cpu, ea_abs(cpu), get_a(cpu)); break;
        case 0x8F: stm(cpu, ea_absl(cpu), get_a(cpu)); break;
        case 0x91: stm(cpu, ea_dpiy(cpu), get_a(cpu)); break;
        case 0x92: stm(cpu, ea_dpi(cpu), get_a(cpu)); break;
        case 0x93: stm(cpu, ea_sriy(cpu), get_a(cpu)); break;
        case 0x95: stm(cpu, ea_dpx(cpu), get_a(cpu)); break;
        case 0x97: stm(cpu, ea_dpily(cpu), get_a(cpu)); break;
        case 0x99: stm(cpu, ea_absy(cpu), get_a(cpu)); break;
        case 0x9D: stm(cpu, ea_absx(cpu), get_a(cpu)); break;
        case 0x9F: stm(cpu, ea_abslx(cpu), get_a(cpu)); break;

        // --- STX / STY / STZ ---
        case 0x86: stxw(cpu, ea_dp(cpu), cpu->x); break;
        case 0x8E: stxw(cpu, ea_abs(cpu), cpu->x); break;
        case 0x96: stxw(cpu, ea_dpy(cpu), cpu->x); break;
        case 0x84: stxw(cpu, ea_dp(cpu), cpu->y); break;
        case 0x8C: stxw(cpu, ea_abs(cpu), cpu->y); break;
        case 0x94: stxw(cpu, ea_dpx(cpu), cpu->y); break;
        case 0x64: stm(cpu, ea_dp(cpu), 0); break;
        case 0x74: stm(cpu, ea_dpx(cpu), 0); break;
        case 0x9C: stm(cpu, ea_abs(cpu), 0); break;
        case 0x9E: stm(cpu, ea_absx(cpu), 0); break;

        // --- ASL / LSR / ROL / ROR ---
        case 0x0A: set_a(cpu, v_asl(cpu, get_a(cpu))); break;
        case 0x06: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_asl(cpu, ldm(cpu, ea))); } break;
        case 0x0E: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_asl(cpu, ldm(cpu, ea))); } break;
        case 0x16: { uint32_t ea = ea_dpx(cpu); stm(cpu, ea, v_asl(cpu, ldm(cpu, ea))); } break;
        case 0x1E: { uint32_t ea = ea_absx(cpu); stm(cpu, ea, v_asl(cpu, ldm(cpu, ea))); } break;
        case 0x4A: set_a(cpu, v_lsr(cpu, get_a(cpu))); break;
        case 0x46: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_lsr(cpu, ldm(cpu, ea))); } break;
        case 0x4E: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_lsr(cpu, ldm(cpu, ea))); } break;
        case 0x56: { uint32_t ea = ea_dpx(cpu); stm(cpu, ea, v_lsr(cpu, ldm(cpu, ea))); } break;
        case 0x5E: { uint32_t ea = ea_absx(cpu); stm(cpu, ea, v_lsr(cpu, ldm(cpu, ea))); } break;
        case 0x2A: set_a(cpu, v_rol(cpu, get_a(cpu))); break;
        case 0x26: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_rol(cpu, ldm(cpu, ea))); } break;
        case 0x2E: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_rol(cpu, ldm(cpu, ea))); } break;
        case 0x36: { uint32_t ea = ea_dpx(cpu); stm(cpu, ea, v_rol(cpu, ldm(cpu, ea))); } break;
        case 0x3E: { uint32_t ea = ea_absx(cpu); stm(cpu, ea, v_rol(cpu, ldm(cpu, ea))); } break;
        case 0x6A: set_a(cpu, v_ror(cpu, get_a(cpu))); break;
        case 0x66: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_ror(cpu, ldm(cpu, ea))); } break;
        case 0x6E: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_ror(cpu, ldm(cpu, ea))); } break;
        case 0x76: { uint32_t ea = ea_dpx(cpu); stm(cpu, ea, v_ror(cpu, ldm(cpu, ea))); } break;
        case 0x7E: { uint32_t ea = ea_absx(cpu); stm(cpu, ea, v_ror(cpu, ldm(cpu, ea))); } break;

        // --- INC / DEC ---
        case 0x1A: set_a(cpu, v_inc(cpu, get_a(cpu))); break;
        case 0xE6: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_inc(cpu, ldm(cpu, ea))); } break;
        case 0xEE: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_inc(cpu, ldm(cpu, ea))); } break;
        case 0xF6: { uint32_t ea = ea_dpx(cpu); stm(cpu, ea, v_inc(cpu, ldm(cpu, ea))); } break;
        case 0xFE: { uint32_t ea = ea_absx(cpu); stm(cpu, ea, v_inc(cpu, ldm(cpu, ea))); } break;
        case 0x3A: set_a(cpu, v_dec(cpu, get_a(cpu))); break;
        case 0xC6: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_dec(cpu, ldm(cpu, ea))); } break;
        case 0xCE: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_dec(cpu, ldm(cpu, ea))); } break;
        case 0xD6: { uint32_t ea = ea_dpx(cpu); stm(cpu, ea, v_dec(cpu, ldm(cpu, ea))); } break;
        case 0xDE: { uint32_t ea = ea_absx(cpu); stm(cpu, ea, v_dec(cpu, ldm(cpu, ea))); } break;
        case 0xE8: cpu->x = (uint16_t)((cpu->x + 1) & (x8(cpu) ? 0xFFu : 0xFFFFu)); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xCA: cpu->x = (uint16_t)((cpu->x - 1) & (x8(cpu) ? 0xFFu : 0xFFFFu)); set_nz(cpu, cpu->x, x8(cpu)); break;
        case 0xC8: cpu->y = (uint16_t)((cpu->y + 1) & (x8(cpu) ? 0xFFu : 0xFFFFu)); set_nz(cpu, cpu->y, x8(cpu)); break;
        case 0x88: cpu->y = (uint16_t)((cpu->y - 1) & (x8(cpu) ? 0xFFu : 0xFFFFu)); set_nz(cpu, cpu->y, x8(cpu)); break;

        // --- TSB / TRB ---
        case 0x04: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_tsb(cpu, ldm(cpu, ea))); } break;
        case 0x0C: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_tsb(cpu, ldm(cpu, ea))); } break;
        case 0x14: { uint32_t ea = ea_dp(cpu); stm(cpu, ea, v_trb(cpu, ldm(cpu, ea))); } break;
        case 0x1C: { uint32_t ea = ea_abs(cpu); stm(cpu, ea, v_trb(cpu, ldm(cpu, ea))); } break;

        // --- Branches ---
        case 0x10: branch(cpu, !get_flag(cpu, FLAG_N)); break; // BPL
        case 0x30: branch(cpu, get_flag(cpu, FLAG_N)); break;  // BMI
        case 0x50: branch(cpu, !get_flag(cpu, FLAG_V)); break; // BVC
        case 0x70: branch(cpu, get_flag(cpu, FLAG_V)); break;  // BVS
        case 0x90: branch(cpu, !get_flag(cpu, FLAG_C)); break; // BCC
        case 0xB0: branch(cpu, get_flag(cpu, FLAG_C)); break;  // BCS
        case 0xD0: branch(cpu, !get_flag(cpu, FLAG_Z)); break; // BNE
        case 0xF0: branch(cpu, get_flag(cpu, FLAG_Z)); break;  // BEQ
        case 0x80: branch(cpu, 1); break;                      // BRA
        case 0x82: // BRL (16-bit relative)
            {
                int16_t rel = (int16_t)fetch16(cpu);
                cpu->pc = (uint16_t)(cpu->pc + rel);
            }
            break;

        // --- Jumps / calls ---
        case 0x4C: cpu->pc = fetch16(cpu); break; // JMP abs
        case 0x5C: // JMP absolute long
            {
                uint32_t target = fetch24(cpu);
                cpu->pc = (uint16_t)(target & 0xFFFF);
                cpu->pbr = (uint8_t)(target >> 16);
            }
            break;
        case 0x6C: // JMP (abs) - pointer in bank 0
            cpu->pc = rd16b0(cpu, fetch16(cpu));
            break;
        case 0x7C: // JMP (abs,X) - pointer in program bank
            {
                uint16_t ptr = (uint16_t)(fetch16(cpu) + cpu->x);
                cpu->pc = rd16(cpu, ((uint32_t)cpu->pbr << 16) | ptr);
            }
            break;
        case 0xDC: // JMP [abs] - 24-bit pointer in bank 0
            {
                uint16_t ptr = fetch16(cpu);
                uint32_t target = rd24b0(cpu, ptr);
                cpu->pc = (uint16_t)(target & 0xFFFF);
                cpu->pbr = (uint8_t)(target >> 16);
            }
            break;
        case 0x20: // JSR abs
            {
                uint16_t target = fetch16(cpu);
                push16(cpu, (uint16_t)(cpu->pc - 1));
                cpu->pc = target;
            }
            break;
        case 0xFC: // JSR (abs,X)
            {
                uint16_t ptr = (uint16_t)(fetch16(cpu) + cpu->x);
                uint16_t target = rd16(cpu, ((uint32_t)cpu->pbr << 16) | ptr);
                push16(cpu, (uint16_t)(cpu->pc - 1));
                cpu->pc = target;
            }
            break;
        case 0x22: // JSL absolute long
            {
                uint32_t target = fetch24(cpu);
                push8(cpu, cpu->pbr);
                push16(cpu, (uint16_t)(cpu->pc - 1));
                cpu->pc = (uint16_t)(target & 0xFFFF);
                cpu->pbr = (uint8_t)(target >> 16);
            }
            break;
        case 0x60: cpu->pc = (uint16_t)(pop16(cpu) + 1); break; // RTS
        case 0x6B: // RTL
            cpu->pc = (uint16_t)(pop16(cpu) + 1);
            cpu->pbr = pop8(cpu);
            break;
        case 0x40: // RTI
            cpu->p = pop8(cpu);
            if (cpu->e) cpu->p |= FLAG_M | FLAG_X;
            update_index_width(cpu);
            cpu->pc = pop16(cpu);
            if (!cpu->e) cpu->pbr = pop8(cpu);
            break;

        // --- Stack pushes / pulls ---
        case 0x48: if (m8(cpu)) push8(cpu, (uint8_t)(cpu->c & 0xFF)); else push16(cpu, cpu->c); break; // PHA
        case 0x68: // PLA
            {
                uint16_t v = m8(cpu) ? pop8(cpu) : pop16(cpu);
                set_a(cpu, v);
                set_nz(cpu, v, m8(cpu));
            }
            break;
        case 0xDA: if (x8(cpu)) push8(cpu, (uint8_t)(cpu->x & 0xFF)); else push16(cpu, cpu->x); break; // PHX
        case 0xFA: cpu->x = x8(cpu) ? pop8(cpu) : pop16(cpu); set_nz(cpu, cpu->x, x8(cpu)); break;     // PLX
        case 0x5A: if (x8(cpu)) push8(cpu, (uint8_t)(cpu->y & 0xFF)); else push16(cpu, cpu->y); break; // PHY
        case 0x7A: cpu->y = x8(cpu) ? pop8(cpu) : pop16(cpu); set_nz(cpu, cpu->y, x8(cpu)); break;     // PLY
        case 0x08: push8(cpu, cpu->p); break; // PHP
        case 0x28: // PLP
            cpu->p = pop8(cpu);
            if (cpu->e) cpu->p |= FLAG_M | FLAG_X;
            update_index_width(cpu);
            break;
        case 0x8B: push8(cpu, cpu->dbr); break;                              // PHB
        case 0xAB: cpu->dbr = pop8(cpu); set_nz(cpu, cpu->dbr, 1); break;    // PLB
        case 0x0B: push16(cpu, cpu->d); break;                               // PHD
        case 0x2B: cpu->d = pop16(cpu); set_nz(cpu, cpu->d, 0); break;       // PLD
        case 0x4B: push8(cpu, cpu->pbr); break;                              // PHK
        case 0xF4: push16(cpu, fetch16(cpu)); break;                         // PEA
        case 0xD4: // PEI - push 16-bit value from direct page
            {
                uint16_t ptr = (uint16_t)(cpu->d + fetch8(cpu));
                push16(cpu, rd16b0(cpu, ptr));
            }
            break;
        case 0x62: // PER - push PC-relative address
            {
                int16_t rel = (int16_t)fetch16(cpu);
                push16(cpu, (uint16_t)(cpu->pc + rel));
            }
            break;

        // --- Transfers ---
        case 0xAA: cpu->x = (uint16_t)(x8(cpu) ? (cpu->c & 0xFF) : cpu->c); set_nz(cpu, cpu->x, x8(cpu)); break; // TAX
        case 0xA8: cpu->y = (uint16_t)(x8(cpu) ? (cpu->c & 0xFF) : cpu->c); set_nz(cpu, cpu->y, x8(cpu)); break; // TAY
        case 0x8A: set_a(cpu, cpu->x); set_nz(cpu, get_a(cpu), m8(cpu)); break; // TXA
        case 0x98: set_a(cpu, cpu->y); set_nz(cpu, get_a(cpu), m8(cpu)); break; // TYA
        case 0xBA: cpu->x = (uint16_t)(x8(cpu) ? (cpu->sp & 0xFF) : cpu->sp); set_nz(cpu, cpu->x, x8(cpu)); break; // TSX
        case 0x9A: cpu->sp = cpu->e ? (uint16_t)(0x0100 | (cpu->x & 0xFF)) : cpu->x; break; // TXS
        case 0x9B: cpu->y = cpu->x; set_nz(cpu, cpu->y, x8(cpu)); break; // TXY
        case 0xBB: cpu->x = cpu->y; set_nz(cpu, cpu->x, x8(cpu)); break; // TYX
        case 0x1B: cpu->sp = cpu->e ? (uint16_t)(0x0100 | (cpu->c & 0xFF)) : cpu->c; break; // TCS
        case 0x3B: cpu->c = cpu->sp; set_nz(cpu, cpu->c, 0); break; // TSC
        case 0x5B: cpu->d = cpu->c; set_nz(cpu, cpu->d, 0); break;  // TCD
        case 0x7B: cpu->c = cpu->d; set_nz(cpu, cpu->c, 0); break;  // TDC
        case 0xEB: // XBA - swap A and B accumulator halves
            cpu->c = (uint16_t)((cpu->c >> 8) | (cpu->c << 8));
            set_nz(cpu, (uint16_t)(cpu->c & 0xFF), 1);
            break;

        // --- Flag operations ---
        case 0x18: set_flag(cpu, FLAG_C, 0); break; // CLC
        case 0x38: set_flag(cpu, FLAG_C, 1); break; // SEC
        case 0x58: set_flag(cpu, FLAG_I, 0); break; // CLI
        case 0x78: set_flag(cpu, FLAG_I, 1); break; // SEI
        case 0xD8: set_flag(cpu, FLAG_D, 0); break; // CLD
        case 0xF8: set_flag(cpu, FLAG_D, 1); break; // SED
        case 0xB8: set_flag(cpu, FLAG_V, 0); break; // CLV
        case 0xC2: // REP - reset status bits
            cpu->p = (uint8_t)(cpu->p & ~fetch8(cpu));
            if (cpu->e) cpu->p |= FLAG_M | FLAG_X;
            update_index_width(cpu);
            break;
        case 0xE2: // SEP - set status bits
            cpu->p = (uint8_t)(cpu->p | fetch8(cpu));
            update_index_width(cpu);
            break;
        case 0xFB: // XCE - exchange carry and emulation flags
            {
                int old_c = get_flag(cpu, FLAG_C);
                set_flag(cpu, FLAG_C, cpu->e);
                cpu->e = (uint8_t)old_c;
                if (cpu->e) {
                    cpu->p |= FLAG_M | FLAG_X;
                    cpu->sp = (uint16_t)(0x0100 | (cpu->sp & 0xFF));
                    update_index_width(cpu);
                }
            }
            break;

        // --- Block moves (whole move executed in one step call) ---
        case 0x54: // MVN - move block negative (ascending)
        case 0x44: // MVP - move block positive (descending)
            {
                uint8_t dst_bank = fetch8(cpu);
                uint8_t src_bank = fetch8(cpu);
                uint16_t xm = x8(cpu) ? (uint16_t)0xFF : (uint16_t)0xFFFF;
                int dir = (op == 0x54) ? 1 : -1;
                cpu->dbr = dst_bank;
                for (;;) {
                    wr8(cpu, ((uint32_t)dst_bank << 16) | (cpu->y & xm),
                        rd8(cpu, ((uint32_t)src_bank << 16) | (cpu->x & xm)));
                    cpu->x = (uint16_t)((cpu->x + dir) & xm);
                    cpu->y = (uint16_t)((cpu->y + dir) & xm);
                    cpu->ticks += 7;
                    if (cpu->c == 0) { cpu->c = 0xFFFF; break; }
                    cpu->c = (uint16_t)(cpu->c - 1);
                }
            }
            break;

        // --- Misc ---
        case 0xEA: break; // NOP
        case 0x42: fetch8(cpu); break; // WDM (reserved, skips operand)
        case 0x02: fetch8(cpu); break; // COP treated as no-op (skips signature)
        case 0x00: // BRK halts the core (no interrupt handler model)
            cpu->halted = 1;
            return 1;
        case 0xCB: // WAI
        case 0xDB: // STP
            cpu->halted = 1;
            return 1;

        default:
            // Unreachable: all 256 opcodes are defined on the 65816
            break;
    }

    return 0;
}

// --- Disassembler -----------------------------------------------------------

enum {
    M_IMP, M_ACC, M_IMMM, M_IMMX, M_IMM8,
    M_DP, M_DPX, M_DPY, M_IDP, M_IDPX, M_IDPY, M_ILDP, M_ILDPY,
    M_SR, M_ISRY, M_ABS, M_ABSX, M_ABSY, M_ABSL, M_ABSLX,
    M_IND, M_INDX, M_INDL, M_REL8, M_REL16, M_MOVE
};

typedef struct DisEntry {
    const char *mn;
    uint8_t mode;
} DisEntry;

static const DisEntry dis_table[256] = {
    {"brk",M_IMM8},{"ora",M_IDPX},{"cop",M_IMM8},{"ora",M_SR},{"tsb",M_DP},{"ora",M_DP},{"asl",M_DP},{"ora",M_ILDP},
    {"php",M_IMP},{"ora",M_IMMM},{"asl",M_ACC},{"phd",M_IMP},{"tsb",M_ABS},{"ora",M_ABS},{"asl",M_ABS},{"ora",M_ABSL},
    {"bpl",M_REL8},{"ora",M_IDPY},{"ora",M_IDP},{"ora",M_ISRY},{"trb",M_DP},{"ora",M_DPX},{"asl",M_DPX},{"ora",M_ILDPY},
    {"clc",M_IMP},{"ora",M_ABSY},{"inc",M_ACC},{"tcs",M_IMP},{"trb",M_ABS},{"ora",M_ABSX},{"asl",M_ABSX},{"ora",M_ABSLX},
    {"jsr",M_ABS},{"and",M_IDPX},{"jsl",M_ABSL},{"and",M_SR},{"bit",M_DP},{"and",M_DP},{"rol",M_DP},{"and",M_ILDP},
    {"plp",M_IMP},{"and",M_IMMM},{"rol",M_ACC},{"pld",M_IMP},{"bit",M_ABS},{"and",M_ABS},{"rol",M_ABS},{"and",M_ABSL},
    {"bmi",M_REL8},{"and",M_IDPY},{"and",M_IDP},{"and",M_ISRY},{"bit",M_DPX},{"and",M_DPX},{"rol",M_DPX},{"and",M_ILDPY},
    {"sec",M_IMP},{"and",M_ABSY},{"dec",M_ACC},{"tsc",M_IMP},{"bit",M_ABSX},{"and",M_ABSX},{"rol",M_ABSX},{"and",M_ABSLX},
    {"rti",M_IMP},{"eor",M_IDPX},{"wdm",M_IMM8},{"eor",M_SR},{"mvp",M_MOVE},{"eor",M_DP},{"lsr",M_DP},{"eor",M_ILDP},
    {"pha",M_IMP},{"eor",M_IMMM},{"lsr",M_ACC},{"phk",M_IMP},{"jmp",M_ABS},{"eor",M_ABS},{"lsr",M_ABS},{"eor",M_ABSL},
    {"bvc",M_REL8},{"eor",M_IDPY},{"eor",M_IDP},{"eor",M_ISRY},{"mvn",M_MOVE},{"eor",M_DPX},{"lsr",M_DPX},{"eor",M_ILDPY},
    {"cli",M_IMP},{"eor",M_ABSY},{"phy",M_IMP},{"tcd",M_IMP},{"jmp",M_ABSL},{"eor",M_ABSX},{"lsr",M_ABSX},{"eor",M_ABSLX},
    {"rts",M_IMP},{"adc",M_IDPX},{"per",M_REL16},{"adc",M_SR},{"stz",M_DP},{"adc",M_DP},{"ror",M_DP},{"adc",M_ILDP},
    {"pla",M_IMP},{"adc",M_IMMM},{"ror",M_ACC},{"rtl",M_IMP},{"jmp",M_IND},{"adc",M_ABS},{"ror",M_ABS},{"adc",M_ABSL},
    {"bvs",M_REL8},{"adc",M_IDPY},{"adc",M_IDP},{"adc",M_ISRY},{"stz",M_DPX},{"adc",M_DPX},{"ror",M_DPX},{"adc",M_ILDPY},
    {"sei",M_IMP},{"adc",M_ABSY},{"ply",M_IMP},{"tdc",M_IMP},{"jmp",M_INDX},{"adc",M_ABSX},{"ror",M_ABSX},{"adc",M_ABSLX},
    {"bra",M_REL8},{"sta",M_IDPX},{"brl",M_REL16},{"sta",M_SR},{"sty",M_DP},{"sta",M_DP},{"stx",M_DP},{"sta",M_ILDP},
    {"dey",M_IMP},{"bit",M_IMMM},{"txa",M_IMP},{"phb",M_IMP},{"sty",M_ABS},{"sta",M_ABS},{"stx",M_ABS},{"sta",M_ABSL},
    {"bcc",M_REL8},{"sta",M_IDPY},{"sta",M_IDP},{"sta",M_ISRY},{"sty",M_DPX},{"sta",M_DPX},{"stx",M_DPY},{"sta",M_ILDPY},
    {"tya",M_IMP},{"sta",M_ABSY},{"txs",M_IMP},{"txy",M_IMP},{"stz",M_ABS},{"sta",M_ABSX},{"stz",M_ABSX},{"sta",M_ABSLX},
    {"ldy",M_IMMX},{"lda",M_IDPX},{"ldx",M_IMMX},{"lda",M_SR},{"ldy",M_DP},{"lda",M_DP},{"ldx",M_DP},{"lda",M_ILDP},
    {"tay",M_IMP},{"lda",M_IMMM},{"tax",M_IMP},{"plb",M_IMP},{"ldy",M_ABS},{"lda",M_ABS},{"ldx",M_ABS},{"lda",M_ABSL},
    {"bcs",M_REL8},{"lda",M_IDPY},{"lda",M_IDP},{"lda",M_ISRY},{"ldy",M_DPX},{"lda",M_DPX},{"ldx",M_DPY},{"lda",M_ILDPY},
    {"clv",M_IMP},{"lda",M_ABSY},{"tsx",M_IMP},{"tyx",M_IMP},{"ldy",M_ABSX},{"lda",M_ABSX},{"ldx",M_ABSY},{"lda",M_ABSLX},
    {"cpy",M_IMMX},{"cmp",M_IDPX},{"rep",M_IMM8},{"cmp",M_SR},{"cpy",M_DP},{"cmp",M_DP},{"dec",M_DP},{"cmp",M_ILDP},
    {"iny",M_IMP},{"cmp",M_IMMM},{"dex",M_IMP},{"wai",M_IMP},{"cpy",M_ABS},{"cmp",M_ABS},{"dec",M_ABS},{"cmp",M_ABSL},
    {"bne",M_REL8},{"cmp",M_IDPY},{"cmp",M_IDP},{"cmp",M_ISRY},{"pei",M_IDP},{"cmp",M_DPX},{"dec",M_DPX},{"cmp",M_ILDPY},
    {"cld",M_IMP},{"cmp",M_ABSY},{"phx",M_IMP},{"stp",M_IMP},{"jmp",M_INDL},{"cmp",M_ABSX},{"dec",M_ABSX},{"cmp",M_ABSLX},
    {"cpx",M_IMMX},{"sbc",M_IDPX},{"sep",M_IMM8},{"sbc",M_SR},{"cpx",M_DP},{"sbc",M_DP},{"inc",M_DP},{"sbc",M_ILDP},
    {"inx",M_IMP},{"sbc",M_IMMM},{"nop",M_IMP},{"xba",M_IMP},{"cpx",M_ABS},{"sbc",M_ABS},{"inc",M_ABS},{"sbc",M_ABSL},
    {"beq",M_REL8},{"sbc",M_IDPY},{"sbc",M_IDP},{"sbc",M_ISRY},{"pea",M_ABS},{"sbc",M_DPX},{"inc",M_DPX},{"sbc",M_ILDPY},
    {"sed",M_IMP},{"sbc",M_ABSY},{"plx",M_IMP},{"xce",M_IMP},{"jsr",M_INDX},{"sbc",M_ABSX},{"inc",M_ABSX},{"sbc",M_ABSLX}
};

void w65816_get_disassembly(void *context, char *buf, size_t buf_len) {
    W65816_CPU *cpu;
    uint32_t base;
    uint8_t op, b1, b2, b3;
    uint16_t w;
    uint32_t l;
    const DisEntry *de;

    if (!context || !buf || buf_len == 0) return;
    cpu = (W65816_CPU*)context;

    base = ((uint32_t)cpu->pbr << 16) | cpu->pc;
    op = rd8(cpu, base);
    b1 = rd8(cpu, base + 1);
    b2 = rd8(cpu, base + 2);
    b3 = rd8(cpu, base + 3);
    w = (uint16_t)(b1 | ((uint16_t)b2 << 8));
    l = (uint32_t)w | ((uint32_t)b3 << 16);
    de = &dis_table[op];

    switch (de->mode) {
        case M_IMP:   snprintf(buf, buf_len, "%s", de->mn); break;
        case M_ACC:   snprintf(buf, buf_len, "%-4s a", de->mn); break;
        case M_IMMM:  // width follows the M flag
            if (m8(cpu)) snprintf(buf, buf_len, "%-4s #$%02X", de->mn, b1);
            else snprintf(buf, buf_len, "%-4s #$%04X", de->mn, w);
            break;
        case M_IMMX:  // width follows the X flag
            if (x8(cpu)) snprintf(buf, buf_len, "%-4s #$%02X", de->mn, b1);
            else snprintf(buf, buf_len, "%-4s #$%04X", de->mn, w);
            break;
        case M_IMM8:  snprintf(buf, buf_len, "%-4s #$%02X", de->mn, b1); break;
        case M_DP:    snprintf(buf, buf_len, "%-4s $%02X", de->mn, b1); break;
        case M_DPX:   snprintf(buf, buf_len, "%-4s $%02X,x", de->mn, b1); break;
        case M_DPY:   snprintf(buf, buf_len, "%-4s $%02X,y", de->mn, b1); break;
        case M_IDP:   snprintf(buf, buf_len, "%-4s ($%02X)", de->mn, b1); break;
        case M_IDPX:  snprintf(buf, buf_len, "%-4s ($%02X,x)", de->mn, b1); break;
        case M_IDPY:  snprintf(buf, buf_len, "%-4s ($%02X),y", de->mn, b1); break;
        case M_ILDP:  snprintf(buf, buf_len, "%-4s [$%02X]", de->mn, b1); break;
        case M_ILDPY: snprintf(buf, buf_len, "%-4s [$%02X],y", de->mn, b1); break;
        case M_SR:    snprintf(buf, buf_len, "%-4s $%02X,s", de->mn, b1); break;
        case M_ISRY:  snprintf(buf, buf_len, "%-4s ($%02X,s),y", de->mn, b1); break;
        case M_ABS:   snprintf(buf, buf_len, "%-4s $%04X", de->mn, w); break;
        case M_ABSX:  snprintf(buf, buf_len, "%-4s $%04X,x", de->mn, w); break;
        case M_ABSY:  snprintf(buf, buf_len, "%-4s $%04X,y", de->mn, w); break;
        case M_ABSL:  snprintf(buf, buf_len, "%-4s $%06X", de->mn, l); break;
        case M_ABSLX: snprintf(buf, buf_len, "%-4s $%06X,x", de->mn, l); break;
        case M_IND:   snprintf(buf, buf_len, "%-4s ($%04X)", de->mn, w); break;
        case M_INDX:  snprintf(buf, buf_len, "%-4s ($%04X,x)", de->mn, w); break;
        case M_INDL:  snprintf(buf, buf_len, "%-4s [$%04X]", de->mn, w); break;
        case M_REL8:  snprintf(buf, buf_len, "%-4s $%04X", de->mn, (uint16_t)(cpu->pc + 2 + (int8_t)b1)); break;
        case M_REL16: snprintf(buf, buf_len, "%-4s $%04X", de->mn, (uint16_t)(cpu->pc + 3 + (int16_t)w)); break;
        case M_MOVE:  snprintf(buf, buf_len, "%-4s $%02X,$%02X", de->mn, b2, b1); break; // src, dst
        default:      snprintf(buf, buf_len, "???"); break;
    }
}

// --- State dump -------------------------------------------------------------

void w65816_print_state(void *context) {
    W65816_CPU *cpu;
    if (!context) return;
    cpu = (W65816_CPU*)context;

    printf("WDC 65C816 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: %02X:%04X  SP: 0x%04X  Halted: %s\n",
           cpu->pbr, cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  C: 0x%04X (A=0x%02X B=0x%02X)  X: 0x%04X  Y: 0x%04X\n",
           cpu->c, cpu->c & 0xFF, (cpu->c >> 8) & 0xFF, cpu->x, cpu->y);
    printf("  D: 0x%04X  DBR: 0x%02X  PBR: 0x%02X  P: 0x%02X  E: %d\n",
           cpu->d, cpu->dbr, cpu->pbr, cpu->p, cpu->e);
    printf("  Flags: N=%d  V=%d  M=%d  X=%d  D=%d  I=%d  Z=%d  C=%d\n",
           get_flag(cpu, FLAG_N), get_flag(cpu, FLAG_V), get_flag(cpu, FLAG_M),
           get_flag(cpu, FLAG_X), get_flag(cpu, FLAG_D), get_flag(cpu, FLAG_I),
           get_flag(cpu, FLAG_Z), get_flag(cpu, FLAG_C));
}
