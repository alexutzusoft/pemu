// NEC V20 (uPD70108) CPU core.
//
// Full 8086-compatible instruction set (adapted from the i8086 core) plus the
// 186-class extensions the V20 implements (PUSHA/POPA, BOUND, PUSH imm,
// IMUL r16,r/m16,imm, INS/OUTS, shift r/m,imm8, ENTER/LEAVE) and the
// NEC-specific 0F-prefixed instructions: TEST1/CLR1/SET1/NOT1 bit operations
// and the ADD4S/SUB4S/CMP4S packed-BCD string operations.
//
// The V20's 8080 emulation mode (BRKEM / RETEM / CALLN, entered via 0F FF)
// is NOT implemented; those opcodes are treated as NOPs.

#include "necv20.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 0x100000 // 1 MB (20-bit address space)

// 16-bit register indices as encoded in 8086 opcodes
enum { R_AX = 0, R_CX, R_DX, R_BX, R_SP, R_BP, R_SI, R_DI };
// Segment register indices as encoded in 8086 opcodes
enum { SEG_ES = 0, SEG_CS, SEG_SS, SEG_DS };

typedef struct V20CPU {
    uint16_t regs[8]; // AX CX DX BX SP BP SI DI
    uint16_t segs[4]; // ES CS SS DS
    uint16_t ip;

    // Flags
    uint8_t flags_c;  // Carry
    uint8_t flags_p;  // Parity
    uint8_t flags_a;  // Auxiliary carry
    uint8_t flags_z;  // Zero
    uint8_t flags_s;  // Sign
    uint8_t flags_o;  // Overflow
    uint8_t flags_d;  // Direction
    uint8_t flags_i;  // Interrupt enable

    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} V20CPU;

static const char* reg16_names[] = { "AX", "CX", "DX", "BX", "SP", "BP", "SI", "DI" };
static const char* reg8_names[]  = { "AL", "CL", "DL", "BL", "AH", "CH", "DH", "BH" };
static const char* seg_names[]   = { "ES", "CS", "SS", "DS" };
static const char* cc_names[]    = { "O", "NO", "B", "NB", "Z", "NZ", "BE", "A",
                                     "S", "NS", "P", "NP", "L", "GE", "LE", "G" };
static const char* alu_names[]   = { "ADD", "OR", "ADC", "SBB", "AND", "SUB", "XOR", "CMP" };
static const char* shift_names[] = { "ROL", "ROR", "RCL", "RCR", "SHL", "SHR", "SHL", "SAR" };
static const char* rm_base_names[] = { "BX+SI", "BX+DI", "BP+SI", "BP+DI", "SI", "DI", "BP", "BX" };
static const char* bit_op_names[]  = { "TEST1", "CLR1", "SET1", "NOT1" };

// --- Memory access (segmented, 20-bit linear) ---

static inline uint32_t linear(uint16_t seg, uint16_t off) {
    return (((uint32_t)seg << 4) + off) & 0xFFFFF;
}

static inline uint8_t mem_read8(V20CPU *cpu, uint16_t seg, uint16_t off) {
    return cpu->memory[linear(seg, off)];
}

static inline void mem_write8(V20CPU *cpu, uint16_t seg, uint16_t off, uint8_t val) {
    cpu->memory[linear(seg, off)] = val;
}

static inline uint16_t mem_read16(V20CPU *cpu, uint16_t seg, uint16_t off) {
    return (uint16_t)(mem_read8(cpu, seg, off) | ((uint16_t)mem_read8(cpu, seg, (uint16_t)(off + 1)) << 8));
}

static inline void mem_write16(V20CPU *cpu, uint16_t seg, uint16_t off, uint16_t val) {
    mem_write8(cpu, seg, off, (uint8_t)(val & 0xFF));
    mem_write8(cpu, seg, (uint16_t)(off + 1), (uint8_t)(val >> 8));
}

static inline uint8_t fetch8(V20CPU *cpu) {
    uint8_t v = mem_read8(cpu, cpu->segs[SEG_CS], cpu->ip);
    cpu->ip++;
    return v;
}

static inline uint16_t fetch16(V20CPU *cpu) {
    uint8_t lo = fetch8(cpu);
    uint8_t hi = fetch8(cpu);
    return (uint16_t)(((uint16_t)hi << 8) | lo);
}

// --- 8-bit register access (AL CL DL BL AH CH DH BH) ---

static inline uint8_t get_reg8(V20CPU *cpu, uint8_t r) {
    if (r < 4) return (uint8_t)(cpu->regs[r] & 0xFF);
    return (uint8_t)(cpu->regs[r - 4] >> 8);
}

static inline void set_reg8(V20CPU *cpu, uint8_t r, uint8_t val) {
    if (r < 4) cpu->regs[r] = (uint16_t)((cpu->regs[r] & 0xFF00) | val);
    else cpu->regs[r - 4] = (uint16_t)((cpu->regs[r - 4] & 0x00FF) | ((uint16_t)val << 8));
}

// --- ModRM decoding ---

typedef struct RMOperand {
    int is_reg;   // 1: register operand (reg holds index), 0: memory operand
    uint8_t reg;  // register index when is_reg
    uint16_t seg; // segment value for memory operand
    uint16_t off; // offset for memory operand
} RMOperand;

// seg_ovr: -1 for none, else segment register index from a prefix
static RMOperand decode_modrm(V20CPU *cpu, uint8_t modrm, int seg_ovr) {
    RMOperand op;
    uint8_t mod = (uint8_t)(modrm >> 6);
    uint8_t rm = (uint8_t)(modrm & 7);

    memset(&op, 0, sizeof(op));
    if (mod == 3) {
        op.is_reg = 1;
        op.reg = rm;
        return op;
    }

    {
        uint16_t off = 0;
        int def_seg = SEG_DS;
        switch (rm) {
            case 0: off = (uint16_t)(cpu->regs[R_BX] + cpu->regs[R_SI]); break;
            case 1: off = (uint16_t)(cpu->regs[R_BX] + cpu->regs[R_DI]); break;
            case 2: off = (uint16_t)(cpu->regs[R_BP] + cpu->regs[R_SI]); def_seg = SEG_SS; break;
            case 3: off = (uint16_t)(cpu->regs[R_BP] + cpu->regs[R_DI]); def_seg = SEG_SS; break;
            case 4: off = cpu->regs[R_SI]; break;
            case 5: off = cpu->regs[R_DI]; break;
            case 6:
                if (mod == 0) off = 0; // direct address, disp16 added below
                else { off = cpu->regs[R_BP]; def_seg = SEG_SS; }
                break;
            case 7: off = cpu->regs[R_BX]; break;
        }
        if (mod == 1) off = (uint16_t)(off + (uint16_t)(int16_t)(int8_t)fetch8(cpu));
        else if (mod == 2 || (mod == 0 && rm == 6)) off = (uint16_t)(off + fetch16(cpu));

        op.is_reg = 0;
        op.off = off;
        op.seg = cpu->segs[(seg_ovr >= 0) ? seg_ovr : def_seg];
    }
    return op;
}

static inline uint8_t read_rm8(V20CPU *cpu, const RMOperand *op) {
    if (op->is_reg) return get_reg8(cpu, op->reg);
    return mem_read8(cpu, op->seg, op->off);
}

static inline void write_rm8(V20CPU *cpu, const RMOperand *op, uint8_t val) {
    if (op->is_reg) set_reg8(cpu, op->reg, val);
    else mem_write8(cpu, op->seg, op->off, val);
}

static inline uint16_t read_rm16(V20CPU *cpu, const RMOperand *op) {
    if (op->is_reg) return cpu->regs[op->reg];
    return mem_read16(cpu, op->seg, op->off);
}

static inline void write_rm16(V20CPU *cpu, const RMOperand *op, uint16_t val) {
    if (op->is_reg) cpu->regs[op->reg] = val;
    else mem_write16(cpu, op->seg, op->off, val);
}

// --- Flags ---

static uint8_t calculate_parity(uint8_t val) {
    uint8_t count = 0;
    for (int i = 0; i < 8; ++i) {
        if ((val >> i) & 1) count++;
    }
    return (count % 2 == 0) ? 1 : 0;
}

static inline void update_flags_zsp8(V20CPU *cpu, uint8_t res) {
    cpu->flags_z = (res == 0) ? 1 : 0;
    cpu->flags_s = (res & 0x80) ? 1 : 0;
    cpu->flags_p = calculate_parity(res);
}

static inline void update_flags_zsp16(V20CPU *cpu, uint16_t res) {
    cpu->flags_z = (res == 0) ? 1 : 0;
    cpu->flags_s = (res & 0x8000) ? 1 : 0;
    cpu->flags_p = calculate_parity((uint8_t)(res & 0xFF));
}

static uint8_t add8(V20CPU *cpu, uint8_t a, uint8_t b, uint8_t carry_in) {
    uint16_t sum = (uint16_t)a + b + carry_in;
    uint8_t res = (uint8_t)(sum & 0xFF);
    cpu->flags_c = (sum > 0xFF) ? 1 : 0;
    cpu->flags_a = (((a & 0x0F) + (b & 0x0F) + carry_in) > 0x0F) ? 1 : 0;
    cpu->flags_o = (((a ^ res) & (b ^ res)) & 0x80) ? 1 : 0;
    update_flags_zsp8(cpu, res);
    return res;
}

static uint16_t add16(V20CPU *cpu, uint16_t a, uint16_t b, uint8_t carry_in) {
    uint32_t sum = (uint32_t)a + b + carry_in;
    uint16_t res = (uint16_t)(sum & 0xFFFF);
    cpu->flags_c = (sum > 0xFFFF) ? 1 : 0;
    cpu->flags_a = (((a & 0x0F) + (b & 0x0F) + carry_in) > 0x0F) ? 1 : 0;
    cpu->flags_o = (((a ^ res) & (b ^ res)) & 0x8000) ? 1 : 0;
    update_flags_zsp16(cpu, res);
    return res;
}

static uint8_t sub8(V20CPU *cpu, uint8_t a, uint8_t b, uint8_t borrow_in) {
    uint16_t sub = (uint16_t)b + borrow_in;
    uint8_t res = (uint8_t)((a - sub) & 0xFF);
    cpu->flags_c = ((uint16_t)a < sub) ? 1 : 0;
    cpu->flags_a = (((a & 0x0F) - (b & 0x0F) - borrow_in) & 0x10) ? 1 : 0;
    cpu->flags_o = (((a ^ b) & (a ^ res)) & 0x80) ? 1 : 0;
    update_flags_zsp8(cpu, res);
    return res;
}

static uint16_t sub16(V20CPU *cpu, uint16_t a, uint16_t b, uint8_t borrow_in) {
    uint32_t sub = (uint32_t)b + borrow_in;
    uint16_t res = (uint16_t)((a - sub) & 0xFFFF);
    cpu->flags_c = ((uint32_t)a < sub) ? 1 : 0;
    cpu->flags_a = (((a & 0x0F) - (b & 0x0F) - borrow_in) & 0x10) ? 1 : 0;
    cpu->flags_o = (((a ^ b) & (a ^ res)) & 0x8000) ? 1 : 0;
    update_flags_zsp16(cpu, res);
    return res;
}

static inline void flags_logic8(V20CPU *cpu, uint8_t res) {
    cpu->flags_c = 0;
    cpu->flags_o = 0;
    cpu->flags_a = 0;
    update_flags_zsp8(cpu, res);
}

static inline void flags_logic16(V20CPU *cpu, uint16_t res) {
    cpu->flags_c = 0;
    cpu->flags_o = 0;
    cpu->flags_a = 0;
    update_flags_zsp16(cpu, res);
}

// FLAGS word: CF | 1 | PF | 0 | AF | 0 | ZF | SF | TF | IF | DF | OF | 1111
static uint16_t get_flags_word(V20CPU *cpu) {
    uint16_t f = 0xF002;
    if (cpu->flags_c) f |= 0x0001;
    if (cpu->flags_p) f |= 0x0004;
    if (cpu->flags_a) f |= 0x0010;
    if (cpu->flags_z) f |= 0x0040;
    if (cpu->flags_s) f |= 0x0080;
    if (cpu->flags_i) f |= 0x0200;
    if (cpu->flags_d) f |= 0x0400;
    if (cpu->flags_o) f |= 0x0800;
    return f;
}

static void set_flags_word(V20CPU *cpu, uint16_t f) {
    cpu->flags_c = (f & 0x0001) ? 1 : 0;
    cpu->flags_p = (f & 0x0004) ? 1 : 0;
    cpu->flags_a = (f & 0x0010) ? 1 : 0;
    cpu->flags_z = (f & 0x0040) ? 1 : 0;
    cpu->flags_s = (f & 0x0080) ? 1 : 0;
    cpu->flags_i = (f & 0x0200) ? 1 : 0;
    cpu->flags_d = (f & 0x0400) ? 1 : 0;
    cpu->flags_o = (f & 0x0800) ? 1 : 0;
}

// ALU dispatch, index encoded in bits 5-3 of the opcode:
// 0=ADD 1=OR 2=ADC 3=SBB 4=AND 5=SUB 6=XOR 7=CMP
static uint8_t alu8(V20CPU *cpu, uint8_t idx, uint8_t a, uint8_t b) {
    switch (idx) {
        case 0: return add8(cpu, a, b, 0);
        case 1: { uint8_t r = a | b; flags_logic8(cpu, r); return r; }
        case 2: return add8(cpu, a, b, cpu->flags_c);
        case 3: return sub8(cpu, a, b, cpu->flags_c);
        case 4: { uint8_t r = a & b; flags_logic8(cpu, r); return r; }
        case 5: return sub8(cpu, a, b, 0);
        case 6: { uint8_t r = a ^ b; flags_logic8(cpu, r); return r; }
        case 7: sub8(cpu, a, b, 0); return a; // CMP: result discarded
    }
    return a;
}

static uint16_t alu16(V20CPU *cpu, uint8_t idx, uint16_t a, uint16_t b) {
    switch (idx) {
        case 0: return add16(cpu, a, b, 0);
        case 1: { uint16_t r = a | b; flags_logic16(cpu, r); return r; }
        case 2: return add16(cpu, a, b, cpu->flags_c);
        case 3: return sub16(cpu, a, b, cpu->flags_c);
        case 4: { uint16_t r = a & b; flags_logic16(cpu, r); return r; }
        case 5: return sub16(cpu, a, b, 0);
        case 6: { uint16_t r = a ^ b; flags_logic16(cpu, r); return r; }
        case 7: sub16(cpu, a, b, 0); return a; // CMP: result discarded
    }
    return a;
}

static int check_cond(V20CPU *cpu, uint8_t cond) {
    int r = 0;
    switch (cond >> 1) {
        case 0: r = cpu->flags_o; break;                              // O
        case 1: r = cpu->flags_c; break;                              // B
        case 2: r = cpu->flags_z; break;                              // Z
        case 3: r = cpu->flags_c | cpu->flags_z; break;               // BE
        case 4: r = cpu->flags_s; break;                              // S
        case 5: r = cpu->flags_p; break;                              // P
        case 6: r = (cpu->flags_s != cpu->flags_o); break;            // L
        case 7: r = cpu->flags_z | (cpu->flags_s != cpu->flags_o); break; // LE
    }
    return (cond & 1) ? !r : (r != 0);
}

// --- Stack ---

static inline void push16(V20CPU *cpu, uint16_t val) {
    cpu->regs[R_SP] = (uint16_t)(cpu->regs[R_SP] - 2);
    mem_write16(cpu, cpu->segs[SEG_SS], cpu->regs[R_SP], val);
}

static inline uint16_t pop16(V20CPU *cpu) {
    uint16_t val = mem_read16(cpu, cpu->segs[SEG_SS], cpu->regs[R_SP]);
    cpu->regs[R_SP] = (uint16_t)(cpu->regs[R_SP] + 2);
    return val;
}

// --- Shifts / rotates (group index in bits 5-3 of ModRM) ---

static uint8_t shift8(V20CPU *cpu, uint8_t idx, uint8_t val, uint8_t count) {
    count &= 0x1F;
    if (count == 0) return val;
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t msb = (uint8_t)((val >> 7) & 1);
        uint8_t lsb = (uint8_t)(val & 1);
        switch (idx) {
            case 0: val = (uint8_t)((val << 1) | msb); cpu->flags_c = msb; break;           // ROL
            case 1: val = (uint8_t)((val >> 1) | (lsb << 7)); cpu->flags_c = lsb; break;    // ROR
            case 2: val = (uint8_t)((val << 1) | cpu->flags_c); cpu->flags_c = msb; break;  // RCL
            case 3: val = (uint8_t)((val >> 1) | (cpu->flags_c << 7)); cpu->flags_c = lsb; break; // RCR
            case 4: case 6: val = (uint8_t)(val << 1); cpu->flags_c = msb; break;           // SHL
            case 5: val = (uint8_t)(val >> 1); cpu->flags_c = lsb; break;                   // SHR
            case 7: val = (uint8_t)((val >> 1) | (val & 0x80)); cpu->flags_c = lsb; break;  // SAR
        }
    }
    cpu->flags_o = (uint8_t)(((val >> 7) & 1) ^ cpu->flags_c);
    if (idx >= 4) update_flags_zsp8(cpu, val);
    return val;
}

static uint16_t shift16(V20CPU *cpu, uint8_t idx, uint16_t val, uint8_t count) {
    count &= 0x1F;
    if (count == 0) return val;
    for (uint8_t i = 0; i < count; ++i) {
        uint8_t msb = (uint8_t)((val >> 15) & 1);
        uint8_t lsb = (uint8_t)(val & 1);
        switch (idx) {
            case 0: val = (uint16_t)((val << 1) | msb); cpu->flags_c = msb; break;          // ROL
            case 1: val = (uint16_t)((val >> 1) | (lsb << 15)); cpu->flags_c = lsb; break;  // ROR
            case 2: val = (uint16_t)((val << 1) | cpu->flags_c); cpu->flags_c = msb; break; // RCL
            case 3: val = (uint16_t)((val >> 1) | ((uint16_t)cpu->flags_c << 15)); cpu->flags_c = lsb; break; // RCR
            case 4: case 6: val = (uint16_t)(val << 1); cpu->flags_c = msb; break;          // SHL
            case 5: val = (uint16_t)(val >> 1); cpu->flags_c = lsb; break;                  // SHR
            case 7: val = (uint16_t)((val >> 1) | (val & 0x8000)); cpu->flags_c = lsb; break; // SAR
        }
    }
    cpu->flags_o = (uint8_t)(((val >> 15) & 1) ^ cpu->flags_c);
    if (idx >= 4) update_flags_zsp16(cpu, val);
    return val;
}

// --- Lifecycle ---

void* necv20_create(void) {
    return calloc(1, sizeof(V20CPU));
}

void necv20_destroy(void *context) {
    free(context);
}

int necv20_init(void *context) {
    if (!context) return -1;
    memset(context, 0, sizeof(V20CPU));
    return 0;
}

int necv20_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    V20CPU *cpu = (V20CPU*)context;
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) copy_len = MEM_SIZE - address;
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// --- NEC packed-BCD string operations (0F 20 ADD4S, 0F 22 SUB4S, 0F 26 CMP4S) ---
//
// Destination string at ES:DI, source at DS:SI (segment-overridable), length
// in CL as a count of BCD digits (two per byte). CF holds the final decimal
// carry/borrow, ZF is set when the whole result is zero.
static void bcd_string_op(V20CPU *cpu, uint8_t sub, int seg_ovr) {
    int nbytes = (get_reg8(cpu, 1) + 1) / 2; // CL digits -> bytes
    uint16_t src_seg = cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS];
    uint8_t carry = 0;
    int nonzero = 0;
    for (int i = 0; i < nbytes; ++i) {
        uint8_t s = mem_read8(cpu, src_seg, (uint16_t)(cpu->regs[R_SI] + i));
        uint8_t d = mem_read8(cpu, cpu->segs[SEG_ES], (uint16_t)(cpu->regs[R_DI] + i));
        uint8_t lo, hi, res;
        if (sub == 0x20) { // ADD4S: dest + src
            int t = (d & 0x0F) + (s & 0x0F) + carry;
            carry = 0;
            if (t > 9) { t -= 10; carry = 1; }
            lo = (uint8_t)t;
            t = (d >> 4) + (s >> 4) + carry;
            carry = 0;
            if (t > 9) { t -= 10; carry = 1; }
            hi = (uint8_t)t;
        } else { // SUB4S / CMP4S: dest - src
            int t = (d & 0x0F) - (s & 0x0F) - carry;
            carry = 0;
            if (t < 0) { t += 10; carry = 1; }
            lo = (uint8_t)t;
            t = (d >> 4) - (s >> 4) - carry;
            carry = 0;
            if (t < 0) { t += 10; carry = 1; }
            hi = (uint8_t)t;
        }
        res = (uint8_t)((hi << 4) | lo);
        if (res != 0) nonzero = 1;
        if (sub != 0x26) // CMP4S discards the result
            mem_write8(cpu, cpu->segs[SEG_ES], (uint16_t)(cpu->regs[R_DI] + i), res);
    }
    cpu->flags_c = carry;
    cpu->flags_z = nonzero ? 0 : 1;
}

// --- Execution ---

int necv20_step(void *context) {
    if (!context) return -1;
    V20CPU *cpu = (V20CPU*)context;

    if (cpu->halted) return 1;

    uint16_t start_ip = cpu->ip;
    uint16_t start_cs = cpu->segs[SEG_CS];
    int seg_ovr = -1;  // segment override prefix
    int rep = 0;       // 0=none, 1=REP/REPE, 2=REPNE
    uint8_t op;

    // Consume prefixes
    for (int i = 0; i < 15; ++i) {
        op = fetch8(cpu);
        if (op == 0x26) { seg_ovr = SEG_ES; continue; }
        if (op == 0x2E) { seg_ovr = SEG_CS; continue; }
        if (op == 0x36) { seg_ovr = SEG_SS; continue; }
        if (op == 0x3E) { seg_ovr = SEG_DS; continue; }
        if (op == 0xF3) { rep = 1; continue; }
        if (op == 0xF2) { rep = 2; continue; }
        if (op == 0xF0) { continue; } // LOCK: ignored
        break;
    }
    op = mem_read8(cpu, start_cs, (uint16_t)(cpu->ip - 1)); // last fetched byte is the opcode
    cpu->ticks++;

    // --- ALU: 00-3F, forms 0-5 (r/m,r / r,r/m / AL,imm / AX,imm) ---
    if (op < 0x40 && (op & 7) < 6) {
        uint8_t idx = (uint8_t)((op >> 3) & 7);
        uint8_t form = (uint8_t)(op & 7);
        switch (form) {
            case 0: { // r/m8, r8
                uint8_t modrm = fetch8(cpu);
                RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
                uint8_t res = alu8(cpu, idx, read_rm8(cpu, &rm), get_reg8(cpu, (uint8_t)((modrm >> 3) & 7)));
                if (idx != 7) write_rm8(cpu, &rm, res);
                break;
            }
            case 1: { // r/m16, r16
                uint8_t modrm = fetch8(cpu);
                RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
                uint16_t res = alu16(cpu, idx, read_rm16(cpu, &rm), cpu->regs[(modrm >> 3) & 7]);
                if (idx != 7) write_rm16(cpu, &rm, res);
                break;
            }
            case 2: { // r8, r/m8
                uint8_t modrm = fetch8(cpu);
                uint8_t reg = (uint8_t)((modrm >> 3) & 7);
                RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
                uint8_t res = alu8(cpu, idx, get_reg8(cpu, reg), read_rm8(cpu, &rm));
                if (idx != 7) set_reg8(cpu, reg, res);
                break;
            }
            case 3: { // r16, r/m16
                uint8_t modrm = fetch8(cpu);
                uint8_t reg = (uint8_t)((modrm >> 3) & 7);
                RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
                uint16_t res = alu16(cpu, idx, cpu->regs[reg], read_rm16(cpu, &rm));
                if (idx != 7) cpu->regs[reg] = res;
                break;
            }
            case 4: { // AL, imm8
                uint8_t res = alu8(cpu, idx, get_reg8(cpu, 0), fetch8(cpu));
                if (idx != 7) set_reg8(cpu, 0, res);
                break;
            }
            case 5: { // AX, imm16
                uint16_t res = alu16(cpu, idx, cpu->regs[R_AX], fetch16(cpu));
                if (idx != 7) cpu->regs[R_AX] = res;
                break;
            }
        }
        goto done;
    }

    // --- INC/DEC r16 (40-4F): CF unaffected ---
    if (op >= 0x40 && op <= 0x4F) {
        uint8_t r = (uint8_t)(op & 7);
        uint8_t saved_c = cpu->flags_c;
        if (op < 0x48) cpu->regs[r] = add16(cpu, cpu->regs[r], 1, 0);
        else cpu->regs[r] = sub16(cpu, cpu->regs[r], 1, 0);
        cpu->flags_c = saved_c;
        goto done;
    }

    // --- PUSH/POP r16 (50-5F) ---
    if (op >= 0x50 && op <= 0x57) { push16(cpu, cpu->regs[op & 7]); goto done; }
    if (op >= 0x58 && op <= 0x5F) { cpu->regs[op & 7] = pop16(cpu); goto done; }

    // --- Jcc rel8 (70-7F) ---
    if (op >= 0x70 && op <= 0x7F) {
        int8_t rel = (int8_t)fetch8(cpu);
        if (check_cond(cpu, (uint8_t)(op & 0x0F))) cpu->ip = (uint16_t)(cpu->ip + rel);
        goto done;
    }

    // --- MOV r, imm (B0-BF) ---
    if (op >= 0xB0 && op <= 0xB7) { set_reg8(cpu, (uint8_t)(op & 7), fetch8(cpu)); goto done; }
    if (op >= 0xB8 && op <= 0xBF) { cpu->regs[op & 7] = fetch16(cpu); goto done; }

    // --- XCHG AX, r16 (90-97; 90 = NOP) ---
    if (op >= 0x90 && op <= 0x97) {
        uint16_t t = cpu->regs[R_AX];
        cpu->regs[R_AX] = cpu->regs[op & 7];
        cpu->regs[op & 7] = t;
        goto done;
    }

    switch (op) {
        // --- PUSH/POP segment registers ---
        case 0x06: case 0x0E: case 0x16: case 0x1E:
            push16(cpu, cpu->segs[(op >> 3) & 3]);
            break;
        case 0x07: case 0x17: case 0x1F:
            cpu->segs[(op >> 3) & 3] = pop16(cpu);
            break;

        // --- 0F: NEC V20 extended instructions ---
        case 0x0F: {
            uint8_t sub = fetch8(cpu);
            if (sub >= 0x10 && sub <= 0x1F) {
                // TEST1/CLR1/SET1/NOT1 r/m, CL (10-17) or imm (18-1F)
                int wide = sub & 1;
                uint8_t modrm = fetch8(cpu);
                RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
                uint8_t bit = (sub < 0x18) ? get_reg8(cpu, 1) : fetch8(cpu);
                uint16_t mask, val;
                bit = (uint8_t)(bit & (wide ? 15 : 7));
                mask = (uint16_t)(1u << bit);
                val = wide ? read_rm16(cpu, &rm) : read_rm8(cpu, &rm);
                switch ((sub >> 1) & 3) {
                    case 0: // TEST1: ZF set when the selected bit is 0
                        cpu->flags_z = ((val & mask) == 0) ? 1 : 0;
                        cpu->flags_c = 0;
                        cpu->flags_o = 0;
                        break;
                    case 1: // CLR1
                        val = (uint16_t)(val & (uint16_t)~mask);
                        if (wide) write_rm16(cpu, &rm, val);
                        else write_rm8(cpu, &rm, (uint8_t)val);
                        break;
                    case 2: // SET1
                        val = (uint16_t)(val | mask);
                        if (wide) write_rm16(cpu, &rm, val);
                        else write_rm8(cpu, &rm, (uint8_t)val);
                        break;
                    case 3: // NOT1
                        val = (uint16_t)(val ^ mask);
                        if (wide) write_rm16(cpu, &rm, val);
                        else write_rm8(cpu, &rm, (uint8_t)val);
                        break;
                }
            } else if (sub == 0x20 || sub == 0x22 || sub == 0x26) {
                bcd_string_op(cpu, sub, seg_ovr); // ADD4S / SUB4S / CMP4S
            } else {
                // ROL4/ROR4, BRKEM and the rest of the 8080 emulation mode
                // machinery are not implemented; treated as NOPs.
            }
            break;
        }

        // --- PUSHA / POPA (186-class) ---
        case 0x60: {
            uint16_t sp0 = cpu->regs[R_SP];
            push16(cpu, cpu->regs[R_AX]);
            push16(cpu, cpu->regs[R_CX]);
            push16(cpu, cpu->regs[R_DX]);
            push16(cpu, cpu->regs[R_BX]);
            push16(cpu, sp0);
            push16(cpu, cpu->regs[R_BP]);
            push16(cpu, cpu->regs[R_SI]);
            push16(cpu, cpu->regs[R_DI]);
            break;
        }
        case 0x61:
            cpu->regs[R_DI] = pop16(cpu);
            cpu->regs[R_SI] = pop16(cpu);
            cpu->regs[R_BP] = pop16(cpu);
            (void)pop16(cpu); // SP discarded
            cpu->regs[R_BX] = pop16(cpu);
            cpu->regs[R_DX] = pop16(cpu);
            cpu->regs[R_CX] = pop16(cpu);
            cpu->regs[R_AX] = pop16(cpu);
            break;

        // --- BOUND r16, m16&16 (186-class) ---
        case 0x62: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            if (!rm.is_reg) {
                int16_t idx_v = (int16_t)cpu->regs[(modrm >> 3) & 7];
                int16_t lo = (int16_t)mem_read16(cpu, rm.seg, rm.off);
                int16_t hi = (int16_t)mem_read16(cpu, rm.seg, (uint16_t)(rm.off + 2));
                if (idx_v < lo || idx_v > hi) { cpu->halted = 1; return -2; } // BR trap: no vector table attached
            }
            break;
        }

        // --- PUSH imm (186-class) ---
        case 0x68: push16(cpu, fetch16(cpu)); break;
        case 0x6A: push16(cpu, (uint16_t)(int16_t)(int8_t)fetch8(cpu)); break;

        // --- IMUL r16, r/m16, imm (186-class) ---
        case 0x69: case 0x6B: {
            uint8_t modrm = fetch8(cpu);
            uint8_t reg = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            int16_t a = (int16_t)read_rm16(cpu, &rm);
            int16_t b = (op == 0x69) ? (int16_t)fetch16(cpu) : (int16_t)(int8_t)fetch8(cpu);
            int32_t prod = (int32_t)a * b;
            cpu->regs[reg] = (uint16_t)((uint32_t)prod & 0xFFFF);
            cpu->flags_c = cpu->flags_o = (prod != (int16_t)((uint32_t)prod & 0xFFFF)) ? 1 : 0;
            break;
        }

        // --- INS / OUTS (186-class): no I/O bus attached, so INS stores 0
        //     and OUTS discards the data; pointers and REP are still honored ---
        case 0x6C: case 0x6D: case 0x6E: case 0x6F: {
            int wide = op & 1;
            int16_t delta = (int16_t)(cpu->flags_d ? -(wide ? 2 : 1) : (wide ? 2 : 1));
            uint16_t src_seg = cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS];
            for (;;) {
                if (rep && cpu->regs[R_CX] == 0) break;
                if (op < 0x6E) { // INS
                    if (wide) mem_write16(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI], 0);
                    else mem_write8(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI], 0);
                    cpu->regs[R_DI] = (uint16_t)(cpu->regs[R_DI] + delta);
                } else { // OUTS
                    (void)mem_read8(cpu, src_seg, cpu->regs[R_SI]);
                    cpu->regs[R_SI] = (uint16_t)(cpu->regs[R_SI] + delta);
                }
                if (!rep) break;
                cpu->regs[R_CX]--;
            }
            break;
        }

        // --- Group 80/81/83: ALU r/m, imm ---
        case 0x80: {
            uint8_t modrm = fetch8(cpu);
            uint8_t idx = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t res = alu8(cpu, idx, read_rm8(cpu, &rm), fetch8(cpu));
            if (idx != 7) write_rm8(cpu, &rm, res);
            break;
        }
        case 0x81: case 0x83: {
            uint8_t modrm = fetch8(cpu);
            uint8_t idx = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint16_t imm = (op == 0x81) ? fetch16(cpu) : (uint16_t)(int16_t)(int8_t)fetch8(cpu);
            uint16_t res = alu16(cpu, idx, read_rm16(cpu, &rm), imm);
            if (idx != 7) write_rm16(cpu, &rm, res);
            break;
        }

        // --- TEST r/m, r ---
        case 0x84: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            flags_logic8(cpu, (uint8_t)(read_rm8(cpu, &rm) & get_reg8(cpu, (uint8_t)((modrm >> 3) & 7))));
            break;
        }
        case 0x85: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            flags_logic16(cpu, (uint16_t)(read_rm16(cpu, &rm) & cpu->regs[(modrm >> 3) & 7]));
            break;
        }

        // --- XCHG r/m, r ---
        case 0x86: {
            uint8_t modrm = fetch8(cpu);
            uint8_t reg = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t t = read_rm8(cpu, &rm);
            write_rm8(cpu, &rm, get_reg8(cpu, reg));
            set_reg8(cpu, reg, t);
            break;
        }
        case 0x87: {
            uint8_t modrm = fetch8(cpu);
            uint8_t reg = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint16_t t = read_rm16(cpu, &rm);
            write_rm16(cpu, &rm, cpu->regs[reg]);
            cpu->regs[reg] = t;
            break;
        }

        // --- MOV (all ModRM forms) ---
        case 0x88: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            write_rm8(cpu, &rm, get_reg8(cpu, (uint8_t)((modrm >> 3) & 7)));
            break;
        }
        case 0x89: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            write_rm16(cpu, &rm, cpu->regs[(modrm >> 3) & 7]);
            break;
        }
        case 0x8A: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            set_reg8(cpu, (uint8_t)((modrm >> 3) & 7), read_rm8(cpu, &rm));
            break;
        }
        case 0x8B: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            cpu->regs[(modrm >> 3) & 7] = read_rm16(cpu, &rm);
            break;
        }
        case 0x8C: { // MOV r/m16, sreg
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            write_rm16(cpu, &rm, cpu->segs[(modrm >> 3) & 3]);
            break;
        }
        case 0x8E: { // MOV sreg, r/m16
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            cpu->segs[(modrm >> 3) & 3] = read_rm16(cpu, &rm);
            break;
        }

        // --- LEA r16, m ---
        case 0x8D: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            if (!rm.is_reg) cpu->regs[(modrm >> 3) & 7] = rm.off;
            break;
        }

        // --- POP r/m16 ---
        case 0x8F: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            write_rm16(cpu, &rm, pop16(cpu));
            break;
        }

        // --- CBW / CWD ---
        case 0x98: cpu->regs[R_AX] = (uint16_t)(int16_t)(int8_t)(cpu->regs[R_AX] & 0xFF); break;
        case 0x99: cpu->regs[R_DX] = (cpu->regs[R_AX] & 0x8000) ? 0xFFFF : 0x0000; break;

        // --- PUSHF / POPF / SAHF / LAHF ---
        case 0x9C: push16(cpu, get_flags_word(cpu)); break;
        case 0x9D: set_flags_word(cpu, pop16(cpu)); break;
        case 0x9E: { // SAHF
            uint8_t ah = get_reg8(cpu, 4);
            cpu->flags_c = (ah & 0x01) ? 1 : 0;
            cpu->flags_p = (ah & 0x04) ? 1 : 0;
            cpu->flags_a = (ah & 0x10) ? 1 : 0;
            cpu->flags_z = (ah & 0x40) ? 1 : 0;
            cpu->flags_s = (ah & 0x80) ? 1 : 0;
            break;
        }
        case 0x9F: set_reg8(cpu, 4, (uint8_t)(get_flags_word(cpu) & 0xFF)); break; // LAHF

        // --- MOV AL/AX <-> moffs ---
        case 0xA0: {
            uint16_t off = fetch16(cpu);
            set_reg8(cpu, 0, mem_read8(cpu, cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS], off));
            break;
        }
        case 0xA1: {
            uint16_t off = fetch16(cpu);
            cpu->regs[R_AX] = mem_read16(cpu, cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS], off);
            break;
        }
        case 0xA2: {
            uint16_t off = fetch16(cpu);
            mem_write8(cpu, cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS], off, get_reg8(cpu, 0));
            break;
        }
        case 0xA3: {
            uint16_t off = fetch16(cpu);
            mem_write16(cpu, cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS], off, cpu->regs[R_AX]);
            break;
        }

        // --- String operations (with optional REP/REPE/REPNE) ---
        case 0xA4: case 0xA5: case 0xA6: case 0xA7:
        case 0xAA: case 0xAB: case 0xAC: case 0xAD: case 0xAE: case 0xAF: {
            int wide = op & 1;
            int16_t delta = (int16_t)(cpu->flags_d ? -(wide ? 2 : 1) : (wide ? 2 : 1));
            uint16_t src_seg = cpu->segs[(seg_ovr >= 0) ? seg_ovr : SEG_DS];
            for (;;) {
                if (rep && cpu->regs[R_CX] == 0) break;
                switch (op & 0xFE) {
                    case 0xA4: // MOVS
                        if (wide) mem_write16(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI], mem_read16(cpu, src_seg, cpu->regs[R_SI]));
                        else mem_write8(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI], mem_read8(cpu, src_seg, cpu->regs[R_SI]));
                        cpu->regs[R_SI] = (uint16_t)(cpu->regs[R_SI] + delta);
                        cpu->regs[R_DI] = (uint16_t)(cpu->regs[R_DI] + delta);
                        break;
                    case 0xA6: // CMPS
                        if (wide) sub16(cpu, mem_read16(cpu, src_seg, cpu->regs[R_SI]), mem_read16(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI]), 0);
                        else sub8(cpu, mem_read8(cpu, src_seg, cpu->regs[R_SI]), mem_read8(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI]), 0);
                        cpu->regs[R_SI] = (uint16_t)(cpu->regs[R_SI] + delta);
                        cpu->regs[R_DI] = (uint16_t)(cpu->regs[R_DI] + delta);
                        break;
                    case 0xAA: // STOS
                        if (wide) mem_write16(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI], cpu->regs[R_AX]);
                        else mem_write8(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI], get_reg8(cpu, 0));
                        cpu->regs[R_DI] = (uint16_t)(cpu->regs[R_DI] + delta);
                        break;
                    case 0xAC: // LODS
                        if (wide) cpu->regs[R_AX] = mem_read16(cpu, src_seg, cpu->regs[R_SI]);
                        else set_reg8(cpu, 0, mem_read8(cpu, src_seg, cpu->regs[R_SI]));
                        cpu->regs[R_SI] = (uint16_t)(cpu->regs[R_SI] + delta);
                        break;
                    case 0xAE: // SCAS
                        if (wide) sub16(cpu, cpu->regs[R_AX], mem_read16(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI]), 0);
                        else sub8(cpu, get_reg8(cpu, 0), mem_read8(cpu, cpu->segs[SEG_ES], cpu->regs[R_DI]), 0);
                        cpu->regs[R_DI] = (uint16_t)(cpu->regs[R_DI] + delta);
                        break;
                }
                if (!rep) break;
                cpu->regs[R_CX]--;
                // REPE/REPNE termination applies to CMPS/SCAS only
                if ((op & 0xFE) == 0xA6 || (op & 0xFE) == 0xAE) {
                    if (rep == 1 && !cpu->flags_z) break;
                    if (rep == 2 && cpu->flags_z) break;
                }
            }
            break;
        }

        // --- TEST AL/AX, imm ---
        case 0xA8: flags_logic8(cpu, (uint8_t)(get_reg8(cpu, 0) & fetch8(cpu))); break;
        case 0xA9: flags_logic16(cpu, (uint16_t)(cpu->regs[R_AX] & fetch16(cpu))); break;

        // --- Shifts / rotates by imm8 (186-class) ---
        case 0xC0: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t count = fetch8(cpu);
            write_rm8(cpu, &rm, shift8(cpu, (uint8_t)((modrm >> 3) & 7), read_rm8(cpu, &rm), count));
            break;
        }
        case 0xC1: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t count = fetch8(cpu);
            write_rm16(cpu, &rm, shift16(cpu, (uint8_t)((modrm >> 3) & 7), read_rm16(cpu, &rm), count));
            break;
        }

        // --- RET near ---
        case 0xC2: {
            uint16_t n = fetch16(cpu);
            cpu->ip = pop16(cpu);
            cpu->regs[R_SP] = (uint16_t)(cpu->regs[R_SP] + n);
            break;
        }
        case 0xC3: cpu->ip = pop16(cpu); break;

        // --- MOV r/m, imm ---
        case 0xC6: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            write_rm8(cpu, &rm, fetch8(cpu));
            break;
        }
        case 0xC7: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            write_rm16(cpu, &rm, fetch16(cpu));
            break;
        }

        // --- ENTER / LEAVE (186-class) ---
        case 0xC8: {
            uint16_t locals = fetch16(cpu);
            uint8_t level = (uint8_t)(fetch8(cpu) & 0x1F);
            uint16_t frame;
            push16(cpu, cpu->regs[R_BP]);
            frame = cpu->regs[R_SP];
            for (uint8_t i = 1; i < level; ++i) {
                cpu->regs[R_BP] = (uint16_t)(cpu->regs[R_BP] - 2);
                push16(cpu, mem_read16(cpu, cpu->segs[SEG_SS], cpu->regs[R_BP]));
            }
            if (level > 0) push16(cpu, frame);
            cpu->regs[R_BP] = frame;
            cpu->regs[R_SP] = (uint16_t)(cpu->regs[R_SP] - locals);
            break;
        }
        case 0xC9:
            cpu->regs[R_SP] = cpu->regs[R_BP];
            cpu->regs[R_BP] = pop16(cpu);
            break;

        // --- INT: INT 3 halts, others are no-ops (no BIOS/DOS attached) ---
        case 0xCC:
            cpu->halted = 1;
            return 1;
        case 0xCD:
            (void)fetch8(cpu); // interrupt number ignored
            break;

        // --- Shifts / rotates ---
        case 0xD0: case 0xD2: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t count = (op == 0xD0) ? 1 : get_reg8(cpu, 1); // 1 or CL
            write_rm8(cpu, &rm, shift8(cpu, (uint8_t)((modrm >> 3) & 7), read_rm8(cpu, &rm), count));
            break;
        }
        case 0xD1: case 0xD3: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t count = (op == 0xD1) ? 1 : get_reg8(cpu, 1); // 1 or CL
            write_rm16(cpu, &rm, shift16(cpu, (uint8_t)((modrm >> 3) & 7), read_rm16(cpu, &rm), count));
            break;
        }

        // --- LOOP / JCXZ ---
        case 0xE0: { // LOOPNE
            int8_t rel = (int8_t)fetch8(cpu);
            cpu->regs[R_CX]--;
            if (cpu->regs[R_CX] != 0 && !cpu->flags_z) cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }
        case 0xE1: { // LOOPE
            int8_t rel = (int8_t)fetch8(cpu);
            cpu->regs[R_CX]--;
            if (cpu->regs[R_CX] != 0 && cpu->flags_z) cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }
        case 0xE2: { // LOOP
            int8_t rel = (int8_t)fetch8(cpu);
            cpu->regs[R_CX]--;
            if (cpu->regs[R_CX] != 0) cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }
        case 0xE3: { // JCXZ
            int8_t rel = (int8_t)fetch8(cpu);
            if (cpu->regs[R_CX] == 0) cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }

        // --- CALL / JMP ---
        case 0xE8: { // CALL rel16
            int16_t rel = (int16_t)fetch16(cpu);
            push16(cpu, cpu->ip);
            cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }
        case 0xE9: { // JMP rel16
            int16_t rel = (int16_t)fetch16(cpu);
            cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }
        case 0xEA: { // JMP far
            uint16_t off = fetch16(cpu);
            uint16_t seg = fetch16(cpu);
            cpu->ip = off;
            cpu->segs[SEG_CS] = seg;
            break;
        }
        case 0xEB: { // JMP rel8
            int8_t rel = (int8_t)fetch8(cpu);
            cpu->ip = (uint16_t)(cpu->ip + rel);
            break;
        }

        // --- HLT / flag ops ---
        case 0xF4: cpu->halted = 1; return 1;                    // HLT
        case 0xF5: cpu->flags_c = !cpu->flags_c; break;          // CMC
        case 0xF8: cpu->flags_c = 0; break;                      // CLC
        case 0xF9: cpu->flags_c = 1; break;                      // STC
        case 0xFA: cpu->flags_i = 0; break;                      // CLI
        case 0xFB: cpu->flags_i = 1; break;                      // STI
        case 0xFC: cpu->flags_d = 0; break;                      // CLD
        case 0xFD: cpu->flags_d = 1; break;                      // STD

        // --- Group F6/F7: TEST/NOT/NEG/MUL/IMUL/DIV/IDIV ---
        case 0xF6: {
            uint8_t modrm = fetch8(cpu);
            uint8_t idx = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t val = read_rm8(cpu, &rm);
            switch (idx) {
                case 0: case 1: flags_logic8(cpu, (uint8_t)(val & fetch8(cpu))); break; // TEST
                case 2: write_rm8(cpu, &rm, (uint8_t)~val); break;                      // NOT
                case 3: write_rm8(cpu, &rm, sub8(cpu, 0, val, 0)); break;               // NEG
                case 4: { // MUL
                    uint16_t prod = (uint16_t)get_reg8(cpu, 0) * val;
                    cpu->regs[R_AX] = prod;
                    cpu->flags_c = cpu->flags_o = (prod > 0xFF) ? 1 : 0;
                    break;
                }
                case 5: { // IMUL
                    int16_t prod = (int16_t)(int8_t)get_reg8(cpu, 0) * (int8_t)val;
                    cpu->regs[R_AX] = (uint16_t)prod;
                    cpu->flags_c = cpu->flags_o = (prod != (int8_t)(prod & 0xFF)) ? 1 : 0;
                    break;
                }
                case 6: { // DIV
                    if (val == 0) { cpu->halted = 1; return -2; }
                    uint16_t ax = cpu->regs[R_AX];
                    cpu->regs[R_AX] = (uint16_t)(((ax % val) << 8) | ((ax / val) & 0xFF));
                    break;
                }
                case 7: { // IDIV
                    if (val == 0) { cpu->halted = 1; return -2; }
                    int16_t ax = (int16_t)cpu->regs[R_AX];
                    int16_t q = (int16_t)(ax / (int8_t)val);
                    int16_t r = (int16_t)(ax % (int8_t)val);
                    cpu->regs[R_AX] = (uint16_t)(((r & 0xFF) << 8) | (q & 0xFF));
                    break;
                }
            }
            break;
        }
        case 0xF7: {
            uint8_t modrm = fetch8(cpu);
            uint8_t idx = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint16_t val = read_rm16(cpu, &rm);
            switch (idx) {
                case 0: case 1: flags_logic16(cpu, (uint16_t)(val & fetch16(cpu))); break; // TEST
                case 2: write_rm16(cpu, &rm, (uint16_t)~val); break;                       // NOT
                case 3: write_rm16(cpu, &rm, sub16(cpu, 0, val, 0)); break;                // NEG
                case 4: { // MUL
                    uint32_t prod = (uint32_t)cpu->regs[R_AX] * val;
                    cpu->regs[R_AX] = (uint16_t)(prod & 0xFFFF);
                    cpu->regs[R_DX] = (uint16_t)(prod >> 16);
                    cpu->flags_c = cpu->flags_o = (cpu->regs[R_DX] != 0) ? 1 : 0;
                    break;
                }
                case 5: { // IMUL
                    int32_t prod = (int32_t)(int16_t)cpu->regs[R_AX] * (int16_t)val;
                    cpu->regs[R_AX] = (uint16_t)(prod & 0xFFFF);
                    cpu->regs[R_DX] = (uint16_t)((uint32_t)prod >> 16);
                    cpu->flags_c = cpu->flags_o = (prod != (int16_t)(prod & 0xFFFF)) ? 1 : 0;
                    break;
                }
                case 6: { // DIV
                    if (val == 0) { cpu->halted = 1; return -2; }
                    uint32_t dividend = ((uint32_t)cpu->regs[R_DX] << 16) | cpu->regs[R_AX];
                    cpu->regs[R_AX] = (uint16_t)((dividend / val) & 0xFFFF);
                    cpu->regs[R_DX] = (uint16_t)(dividend % val);
                    break;
                }
                case 7: { // IDIV
                    if (val == 0) { cpu->halted = 1; return -2; }
                    int32_t dividend = (int32_t)(((uint32_t)cpu->regs[R_DX] << 16) | cpu->regs[R_AX]);
                    cpu->regs[R_AX] = (uint16_t)((dividend / (int16_t)val) & 0xFFFF);
                    cpu->regs[R_DX] = (uint16_t)((dividend % (int16_t)val) & 0xFFFF);
                    break;
                }
            }
            break;
        }

        // --- Group FE: INC/DEC r/m8 (CF unaffected) ---
        case 0xFE: {
            uint8_t modrm = fetch8(cpu);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            uint8_t saved_c = cpu->flags_c;
            uint8_t val = read_rm8(cpu, &rm);
            if (((modrm >> 3) & 7) == 0) write_rm8(cpu, &rm, add8(cpu, val, 1, 0));
            else write_rm8(cpu, &rm, sub8(cpu, val, 1, 0));
            cpu->flags_c = saved_c;
            break;
        }

        // --- Group FF: INC/DEC/CALL/JMP/PUSH r/m16 ---
        case 0xFF: {
            uint8_t modrm = fetch8(cpu);
            uint8_t idx = (uint8_t)((modrm >> 3) & 7);
            RMOperand rm = decode_modrm(cpu, modrm, seg_ovr);
            switch (idx) {
                case 0: { // INC
                    uint8_t saved_c = cpu->flags_c;
                    write_rm16(cpu, &rm, add16(cpu, read_rm16(cpu, &rm), 1, 0));
                    cpu->flags_c = saved_c;
                    break;
                }
                case 1: { // DEC
                    uint8_t saved_c = cpu->flags_c;
                    write_rm16(cpu, &rm, sub16(cpu, read_rm16(cpu, &rm), 1, 0));
                    cpu->flags_c = saved_c;
                    break;
                }
                case 2: { // CALL near r/m
                    uint16_t target = read_rm16(cpu, &rm);
                    push16(cpu, cpu->ip);
                    cpu->ip = target;
                    break;
                }
                case 4: cpu->ip = read_rm16(cpu, &rm); break; // JMP near r/m
                case 6: push16(cpu, read_rm16(cpu, &rm)); break; // PUSH r/m
                default: break; // far CALL/JMP not supported
            }
            break;
        }

        default:
            // Unimplemented opcodes treated as NOP.
            break;
    }

done:
    // Self-loop (e.g. JMP $) interpreted as a software halt, matching other cores.
    if (cpu->ip == start_ip && cpu->segs[SEG_CS] == start_cs) {
        cpu->halted = 1;
        return 1;
    }
    return 0;
}

// --- State printing ---

void necv20_print_state(void *context) {
    if (!context) return;
    V20CPU *cpu = (V20CPU*)context;

    printf("NEC V20 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  CS:IP: %04X:%04X (linear 0x%05X)  Halted: %s\n",
           cpu->segs[SEG_CS], cpu->ip, linear(cpu->segs[SEG_CS], cpu->ip),
           cpu->halted ? "Yes" : "No");
    printf("  AX: 0x%04X  BX: 0x%04X  CX: 0x%04X  DX: 0x%04X\n",
           cpu->regs[R_AX], cpu->regs[R_BX], cpu->regs[R_CX], cpu->regs[R_DX]);
    printf("  SP: 0x%04X  BP: 0x%04X  SI: 0x%04X  DI: 0x%04X\n",
           cpu->regs[R_SP], cpu->regs[R_BP], cpu->regs[R_SI], cpu->regs[R_DI]);
    printf("  CS: 0x%04X  DS: 0x%04X  ES: 0x%04X  SS: 0x%04X\n",
           cpu->segs[SEG_CS], cpu->segs[SEG_DS], cpu->segs[SEG_ES], cpu->segs[SEG_SS]);
    printf("  Flags: CF=%d PF=%d AF=%d ZF=%d SF=%d OF=%d DF=%d IF=%d\n",
           cpu->flags_c, cpu->flags_p, cpu->flags_a, cpu->flags_z,
           cpu->flags_s, cpu->flags_o, cpu->flags_d, cpu->flags_i);
}

// --- Disassembly ---

static uint8_t dpeek8(V20CPU *cpu, uint16_t *off) {
    uint8_t v = mem_read8(cpu, cpu->segs[SEG_CS], *off);
    (*off)++;
    return v;
}

static uint16_t dpeek16(V20CPU *cpu, uint16_t *off) {
    uint8_t lo = dpeek8(cpu, off);
    uint8_t hi = dpeek8(cpu, off);
    return (uint16_t)(((uint16_t)hi << 8) | lo);
}

// Consumes the ModRM displacement and formats the r/m operand text.
static void dasm_rm(V20CPU *cpu, uint16_t *off, uint8_t modrm, int wide, char *out, size_t n) {
    uint8_t mod = (uint8_t)(modrm >> 6);
    uint8_t rm = (uint8_t)(modrm & 7);

    if (mod == 3) {
        snprintf(out, n, "%s", wide ? reg16_names[rm] : reg8_names[rm]);
        return;
    }
    if (mod == 0 && rm == 6) {
        snprintf(out, n, "[0x%04X]", dpeek16(cpu, off));
        return;
    }
    if (mod == 0) {
        snprintf(out, n, "[%s]", rm_base_names[rm]);
    } else if (mod == 1) {
        int8_t d = (int8_t)dpeek8(cpu, off);
        snprintf(out, n, "[%s%s0x%02X]", rm_base_names[rm], d < 0 ? "-" : "+", (d < 0) ? (uint8_t)(-d) : (uint8_t)d);
    } else {
        snprintf(out, n, "[%s+0x%04X]", rm_base_names[rm], dpeek16(cpu, off));
    }
}

void necv20_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    V20CPU *cpu = (V20CPU*)context;

    uint16_t off = cpu->ip;
    char prefix[16] = "";
    char rm_str[32];
    uint8_t op;

    // Consume prefixes
    for (int i = 0; i < 15; ++i) {
        op = dpeek8(cpu, &off);
        if (op == 0x26 || op == 0x2E || op == 0x36 || op == 0x3E) {
            snprintf(prefix, sizeof(prefix), "%s: ", seg_names[(op >> 3) & 3]);
            continue;
        }
        if (op == 0xF3) { snprintf(prefix, sizeof(prefix), "REP "); continue; }
        if (op == 0xF2) { snprintf(prefix, sizeof(prefix), "REPNE "); continue; }
        if (op == 0xF0) { snprintf(prefix, sizeof(prefix), "LOCK "); continue; }
        break;
    }
    op = mem_read8(cpu, cpu->segs[SEG_CS], (uint16_t)(off - 1));

    // ALU 00-3F
    if (op < 0x40 && (op & 7) < 6) {
        const char *name = alu_names[(op >> 3) & 7];
        uint8_t form = (uint8_t)(op & 7);
        uint8_t modrm;
        switch (form) {
            case 0: case 1: case 2: case 3:
                modrm = dpeek8(cpu, &off);
                dasm_rm(cpu, &off, modrm, form & 1, rm_str, sizeof(rm_str));
                if (form < 2)
                    snprintf(buf, buf_len, "%s%-5s %s, %s", prefix, name, rm_str,
                             (form & 1) ? reg16_names[(modrm >> 3) & 7] : reg8_names[(modrm >> 3) & 7]);
                else
                    snprintf(buf, buf_len, "%s%-5s %s, %s", prefix, name,
                             (form & 1) ? reg16_names[(modrm >> 3) & 7] : reg8_names[(modrm >> 3) & 7], rm_str);
                return;
            case 4: snprintf(buf, buf_len, "%s%-5s AL, 0x%02X", prefix, name, dpeek8(cpu, &off)); return;
            case 5: snprintf(buf, buf_len, "%s%-5s AX, 0x%04X", prefix, name, dpeek16(cpu, &off)); return;
        }
    }

    if (op >= 0x40 && op <= 0x47) { snprintf(buf, buf_len, "INC   %s", reg16_names[op & 7]); return; }
    if (op >= 0x48 && op <= 0x4F) { snprintf(buf, buf_len, "DEC   %s", reg16_names[op & 7]); return; }
    if (op >= 0x50 && op <= 0x57) { snprintf(buf, buf_len, "PUSH  %s", reg16_names[op & 7]); return; }
    if (op >= 0x58 && op <= 0x5F) { snprintf(buf, buf_len, "POP   %s", reg16_names[op & 7]); return; }
    if (op >= 0x70 && op <= 0x7F) {
        int8_t rel = (int8_t)dpeek8(cpu, &off);
        snprintf(buf, buf_len, "J%-4s 0x%04X", cc_names[op & 0x0F], (uint16_t)(off + rel));
        return;
    }
    if (op == 0x90) { snprintf(buf, buf_len, "NOP"); return; }
    if (op >= 0x91 && op <= 0x97) { snprintf(buf, buf_len, "XCHG  AX, %s", reg16_names[op & 7]); return; }
    if (op >= 0xB0 && op <= 0xB7) { snprintf(buf, buf_len, "MOV   %s, 0x%02X", reg8_names[op & 7], dpeek8(cpu, &off)); return; }
    if (op >= 0xB8 && op <= 0xBF) { snprintf(buf, buf_len, "MOV   %s, 0x%04X", reg16_names[op & 7], dpeek16(cpu, &off)); return; }

    switch (op) {
        case 0x06: case 0x0E: case 0x16: case 0x1E:
            snprintf(buf, buf_len, "PUSH  %s", seg_names[(op >> 3) & 3]); return;
        case 0x07: case 0x17: case 0x1F:
            snprintf(buf, buf_len, "POP   %s", seg_names[(op >> 3) & 3]); return;
        case 0x0F: { // NEC V20 extended instructions
            uint8_t sub = dpeek8(cpu, &off);
            if (sub >= 0x10 && sub <= 0x1F) {
                uint8_t modrm = dpeek8(cpu, &off);
                dasm_rm(cpu, &off, modrm, sub & 1, rm_str, sizeof(rm_str));
                if (sub < 0x18)
                    snprintf(buf, buf_len, "%s%-5s %s, CL", prefix, bit_op_names[(sub >> 1) & 3], rm_str);
                else
                    snprintf(buf, buf_len, "%s%-5s %s, %u", prefix, bit_op_names[(sub >> 1) & 3], rm_str,
                             (unsigned)(dpeek8(cpu, &off) & ((sub & 1) ? 15 : 7)));
                return;
            }
            if (sub == 0x20) { snprintf(buf, buf_len, "%sADD4S", prefix); return; }
            if (sub == 0x22) { snprintf(buf, buf_len, "%sSUB4S", prefix); return; }
            if (sub == 0x26) { snprintf(buf, buf_len, "%sCMP4S", prefix); return; }
            snprintf(buf, buf_len, "DB    0x0F, 0x%02X", sub);
            return;
        }
        case 0x60: snprintf(buf, buf_len, "PUSHA"); return;
        case 0x61: snprintf(buf, buf_len, "POPA"); return;
        case 0x62: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%sBOUND %s, %s", prefix, reg16_names[(modrm >> 3) & 7], rm_str);
            return;
        }
        case 0x68: snprintf(buf, buf_len, "PUSH  0x%04X", dpeek16(cpu, &off)); return;
        case 0x6A: snprintf(buf, buf_len, "PUSH  0x%04X", (uint16_t)(int16_t)(int8_t)dpeek8(cpu, &off)); return;
        case 0x69: case 0x6B: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 1, rm_str, sizeof(rm_str));
            if (op == 0x69)
                snprintf(buf, buf_len, "%sIMUL  %s, %s, 0x%04X", prefix, reg16_names[(modrm >> 3) & 7], rm_str, dpeek16(cpu, &off));
            else
                snprintf(buf, buf_len, "%sIMUL  %s, %s, 0x%04X", prefix, reg16_names[(modrm >> 3) & 7], rm_str,
                         (uint16_t)(int16_t)(int8_t)dpeek8(cpu, &off));
            return;
        }
        case 0x6C: snprintf(buf, buf_len, "%sINSB", prefix); return;
        case 0x6D: snprintf(buf, buf_len, "%sINSW", prefix); return;
        case 0x6E: snprintf(buf, buf_len, "%sOUTSB", prefix); return;
        case 0x6F: snprintf(buf, buf_len, "%sOUTSW", prefix); return;
        case 0x80: case 0x81: case 0x83: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, op != 0x80, rm_str, sizeof(rm_str));
            if (op == 0x80)
                snprintf(buf, buf_len, "%s%-5s %s, 0x%02X", prefix, alu_names[(modrm >> 3) & 7], rm_str, dpeek8(cpu, &off));
            else if (op == 0x81)
                snprintf(buf, buf_len, "%s%-5s %s, 0x%04X", prefix, alu_names[(modrm >> 3) & 7], rm_str, dpeek16(cpu, &off));
            else
                snprintf(buf, buf_len, "%s%-5s %s, 0x%04X", prefix, alu_names[(modrm >> 3) & 7], rm_str,
                         (uint16_t)(int16_t)(int8_t)dpeek8(cpu, &off));
            return;
        }
        case 0x84: case 0x85: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, op & 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%sTEST  %s, %s", prefix, rm_str,
                     (op & 1) ? reg16_names[(modrm >> 3) & 7] : reg8_names[(modrm >> 3) & 7]);
            return;
        }
        case 0x86: case 0x87: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, op & 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%sXCHG  %s, %s", prefix, rm_str,
                     (op & 1) ? reg16_names[(modrm >> 3) & 7] : reg8_names[(modrm >> 3) & 7]);
            return;
        }
        case 0x88: case 0x89: case 0x8A: case 0x8B: {
            uint8_t modrm = dpeek8(cpu, &off);
            int wide = op & 1;
            dasm_rm(cpu, &off, modrm, wide, rm_str, sizeof(rm_str));
            if (op < 0x8A)
                snprintf(buf, buf_len, "%sMOV   %s, %s", prefix, rm_str,
                         wide ? reg16_names[(modrm >> 3) & 7] : reg8_names[(modrm >> 3) & 7]);
            else
                snprintf(buf, buf_len, "%sMOV   %s, %s", prefix,
                         wide ? reg16_names[(modrm >> 3) & 7] : reg8_names[(modrm >> 3) & 7], rm_str);
            return;
        }
        case 0x8C: case 0x8E: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 1, rm_str, sizeof(rm_str));
            if (op == 0x8C)
                snprintf(buf, buf_len, "%sMOV   %s, %s", prefix, rm_str, seg_names[(modrm >> 3) & 3]);
            else
                snprintf(buf, buf_len, "%sMOV   %s, %s", prefix, seg_names[(modrm >> 3) & 3], rm_str);
            return;
        }
        case 0x8D: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%sLEA   %s, %s", prefix, reg16_names[(modrm >> 3) & 7], rm_str);
            return;
        }
        case 0x8F: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%sPOP   %s", prefix, rm_str);
            return;
        }
        case 0x98: snprintf(buf, buf_len, "CBW"); return;
        case 0x99: snprintf(buf, buf_len, "CWD"); return;
        case 0x9C: snprintf(buf, buf_len, "PUSHF"); return;
        case 0x9D: snprintf(buf, buf_len, "POPF"); return;
        case 0x9E: snprintf(buf, buf_len, "SAHF"); return;
        case 0x9F: snprintf(buf, buf_len, "LAHF"); return;
        case 0xA0: snprintf(buf, buf_len, "%sMOV   AL, [0x%04X]", prefix, dpeek16(cpu, &off)); return;
        case 0xA1: snprintf(buf, buf_len, "%sMOV   AX, [0x%04X]", prefix, dpeek16(cpu, &off)); return;
        case 0xA2: snprintf(buf, buf_len, "%sMOV   [0x%04X], AL", prefix, dpeek16(cpu, &off)); return;
        case 0xA3: snprintf(buf, buf_len, "%sMOV   [0x%04X], AX", prefix, dpeek16(cpu, &off)); return;
        case 0xA4: snprintf(buf, buf_len, "%sMOVSB", prefix); return;
        case 0xA5: snprintf(buf, buf_len, "%sMOVSW", prefix); return;
        case 0xA6: snprintf(buf, buf_len, "%sCMPSB", prefix); return;
        case 0xA7: snprintf(buf, buf_len, "%sCMPSW", prefix); return;
        case 0xA8: snprintf(buf, buf_len, "TEST  AL, 0x%02X", dpeek8(cpu, &off)); return;
        case 0xA9: snprintf(buf, buf_len, "TEST  AX, 0x%04X", dpeek16(cpu, &off)); return;
        case 0xAA: snprintf(buf, buf_len, "%sSTOSB", prefix); return;
        case 0xAB: snprintf(buf, buf_len, "%sSTOSW", prefix); return;
        case 0xAC: snprintf(buf, buf_len, "%sLODSB", prefix); return;
        case 0xAD: snprintf(buf, buf_len, "%sLODSW", prefix); return;
        case 0xAE: snprintf(buf, buf_len, "%sSCASB", prefix); return;
        case 0xAF: snprintf(buf, buf_len, "%sSCASW", prefix); return;
        case 0xC0: case 0xC1: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, op & 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%s%-5s %s, %u", prefix, shift_names[(modrm >> 3) & 7], rm_str,
                     (unsigned)dpeek8(cpu, &off));
            return;
        }
        case 0xC2: snprintf(buf, buf_len, "RET   0x%04X", dpeek16(cpu, &off)); return;
        case 0xC3: snprintf(buf, buf_len, "RET"); return;
        case 0xC6: case 0xC7: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, op & 1, rm_str, sizeof(rm_str));
            if (op & 1) snprintf(buf, buf_len, "%sMOV   %s, 0x%04X", prefix, rm_str, dpeek16(cpu, &off));
            else snprintf(buf, buf_len, "%sMOV   %s, 0x%02X", prefix, rm_str, dpeek8(cpu, &off));
            return;
        }
        case 0xC8: {
            uint16_t locals = dpeek16(cpu, &off);
            snprintf(buf, buf_len, "ENTER 0x%04X, %u", locals, (unsigned)dpeek8(cpu, &off));
            return;
        }
        case 0xC9: snprintf(buf, buf_len, "LEAVE"); return;
        case 0xCC: snprintf(buf, buf_len, "INT   3"); return;
        case 0xCD: snprintf(buf, buf_len, "INT   0x%02X", dpeek8(cpu, &off)); return;
        case 0xD0: case 0xD1: case 0xD2: case 0xD3: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, op & 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%s%-5s %s, %s", prefix, shift_names[(modrm >> 3) & 7], rm_str,
                     (op < 0xD2) ? "1" : "CL");
            return;
        }
        case 0xE0: case 0xE1: case 0xE2: case 0xE3: {
            const char* loop_names[] = { "LOOPNE", "LOOPE", "LOOP", "JCXZ" };
            int8_t rel = (int8_t)dpeek8(cpu, &off);
            snprintf(buf, buf_len, "%-5s 0x%04X", loop_names[op - 0xE0], (uint16_t)(off + rel));
            return;
        }
        case 0xE8: {
            int16_t rel = (int16_t)dpeek16(cpu, &off);
            snprintf(buf, buf_len, "CALL  0x%04X", (uint16_t)(off + rel));
            return;
        }
        case 0xE9: {
            int16_t rel = (int16_t)dpeek16(cpu, &off);
            snprintf(buf, buf_len, "JMP   0x%04X", (uint16_t)(off + rel));
            return;
        }
        case 0xEA: {
            uint16_t o = dpeek16(cpu, &off);
            uint16_t s = dpeek16(cpu, &off);
            snprintf(buf, buf_len, "JMP   %04X:%04X", s, o);
            return;
        }
        case 0xEB: {
            int8_t rel = (int8_t)dpeek8(cpu, &off);
            snprintf(buf, buf_len, "JMP   0x%04X", (uint16_t)(off + rel));
            return;
        }
        case 0xF4: snprintf(buf, buf_len, "HLT"); return;
        case 0xF5: snprintf(buf, buf_len, "CMC"); return;
        case 0xF8: snprintf(buf, buf_len, "CLC"); return;
        case 0xF9: snprintf(buf, buf_len, "STC"); return;
        case 0xFA: snprintf(buf, buf_len, "CLI"); return;
        case 0xFB: snprintf(buf, buf_len, "STI"); return;
        case 0xFC: snprintf(buf, buf_len, "CLD"); return;
        case 0xFD: snprintf(buf, buf_len, "STD"); return;
        case 0xF6: case 0xF7: {
            const char* grp_names[] = { "TEST", "TEST", "NOT", "NEG", "MUL", "IMUL", "DIV", "IDIV" };
            uint8_t modrm = dpeek8(cpu, &off);
            uint8_t idx = (uint8_t)((modrm >> 3) & 7);
            dasm_rm(cpu, &off, modrm, op & 1, rm_str, sizeof(rm_str));
            if (idx < 2) {
                if (op & 1) snprintf(buf, buf_len, "%sTEST  %s, 0x%04X", prefix, rm_str, dpeek16(cpu, &off));
                else snprintf(buf, buf_len, "%sTEST  %s, 0x%02X", prefix, rm_str, dpeek8(cpu, &off));
            } else {
                snprintf(buf, buf_len, "%s%-5s %s", prefix, grp_names[idx], rm_str);
            }
            return;
        }
        case 0xFE: {
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 0, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%s%-5s %s", prefix, (((modrm >> 3) & 7) == 0) ? "INC" : "DEC", rm_str);
            return;
        }
        case 0xFF: {
            const char* grp_names[] = { "INC", "DEC", "CALL", "CALLF", "JMP", "JMPF", "PUSH", "???" };
            uint8_t modrm = dpeek8(cpu, &off);
            dasm_rm(cpu, &off, modrm, 1, rm_str, sizeof(rm_str));
            snprintf(buf, buf_len, "%s%-5s %s", prefix, grp_names[(modrm >> 3) & 7], rm_str);
            return;
        }
        default:
            snprintf(buf, buf_len, "DB    0x%02X", op);
            return;
    }
}
