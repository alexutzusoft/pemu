#include "arm7tdmi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE (1024 * 1024)
#define MEM_MASK (MEM_SIZE - 1)

// CPSR flag bits
#define FLAG_N 0x80000000u
#define FLAG_Z 0x40000000u
#define FLAG_C 0x20000000u
#define FLAG_V 0x10000000u
#define FLAG_T 0x00000020u // Thumb state
#define MODE_USR 0x10u     // User mode bits (only mode modeled)

typedef struct ARM7TDMI {
    uint32_t r[16];   // R0-R15 (R15 = PC, holds fetch address of current instruction)
    uint32_t cpsr;    // N Z C V flags, T bit, mode bits.
                      // Banked registers, SPSRs and exception modes (FIQ/IRQ/SVC/
                      // ABT/UND) are not modeled: the core runs in user mode only.
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
    int pc_written;   // Set when the executing instruction wrote R15
} ARM7TDMI;

static const char *reg_names[16] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"
};

static const char *cond_names[16] = {
    "eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc",
    "hi", "ls", "ge", "lt", "gt", "le", "", "nv"
};

static const char *dp_names[16] = {
    "and", "eor", "sub", "rsb", "add", "adc", "sbc", "rsc",
    "tst", "teq", "cmp", "cmn", "orr", "mov", "bic", "mvn"
};

static const char *shift_names[4] = { "lsl", "lsr", "asr", "ror" };

void* arm7tdmi_create(void) {
    ARM7TDMI *cpu = (ARM7TDMI*)calloc(1, sizeof(ARM7TDMI));
    return cpu;
}

void arm7tdmi_destroy(void *context) {
    free(context);
}

int arm7tdmi_init(void *context) {
    if (!context) return -1;
    ARM7TDMI *cpu = (ARM7TDMI*)context;

    memset(cpu->r, 0, sizeof(cpu->r));
    cpu->r[13] = MEM_SIZE - 4; // SP initialized near top of RAM
    cpu->cpsr = MODE_USR;      // User mode, ARM state, flags clear
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->pc_written = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int arm7tdmi_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    ARM7TDMI *cpu = (ARM7TDMI*)context;

    address &= MEM_MASK;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// ---------------------------------------------------------------------------
// Memory helpers (1MB RAM, addresses masked; word/halfword accesses aligned)
// ---------------------------------------------------------------------------

static uint32_t mem_read32(ARM7TDMI *cpu, uint32_t addr) {
    uint32_t val;
    addr &= MEM_MASK & ~3u;
    memcpy(&val, &cpu->memory[addr], 4);
    return val;
}

static uint16_t mem_read16(ARM7TDMI *cpu, uint32_t addr) {
    uint16_t val;
    addr &= MEM_MASK & ~1u;
    memcpy(&val, &cpu->memory[addr], 2);
    return val;
}

static uint8_t mem_read8(ARM7TDMI *cpu, uint32_t addr) {
    return cpu->memory[addr & MEM_MASK];
}

static void mem_write32(ARM7TDMI *cpu, uint32_t addr, uint32_t val) {
    addr &= MEM_MASK & ~3u;
    memcpy(&cpu->memory[addr], &val, 4);
}

static void mem_write16(ARM7TDMI *cpu, uint32_t addr, uint16_t val) {
    addr &= MEM_MASK & ~1u;
    memcpy(&cpu->memory[addr], &val, 2);
}

static void mem_write8(ARM7TDMI *cpu, uint32_t addr, uint8_t val) {
    cpu->memory[addr & MEM_MASK] = val;
}

// ---------------------------------------------------------------------------
// Register / flag helpers
// ---------------------------------------------------------------------------

// R15 reads as current instruction + 8 in ARM state, + 4 in Thumb state.
static uint32_t rreg(ARM7TDMI *cpu, int n) {
    if (n == 15) return cpu->r[15] + ((cpu->cpsr & FLAG_T) ? 4u : 8u);
    return cpu->r[n];
}

static void wreg(ARM7TDMI *cpu, int n, uint32_t val) {
    if (n == 15) {
        cpu->r[15] = val & ((cpu->cpsr & FLAG_T) ? ~1u : ~3u);
        cpu->pc_written = 1;
    } else {
        cpu->r[n] = val;
    }
}

static void set_nz(ARM7TDMI *cpu, uint32_t res) {
    cpu->cpsr &= ~(FLAG_N | FLAG_Z);
    if (res & 0x80000000u) cpu->cpsr |= FLAG_N;
    if (res == 0) cpu->cpsr |= FLAG_Z;
}

static void set_c(ARM7TDMI *cpu, int c) {
    if (c) cpu->cpsr |= FLAG_C; else cpu->cpsr &= ~FLAG_C;
}

static void set_v(ARM7TDMI *cpu, int v) {
    if (v) cpu->cpsr |= FLAG_V; else cpu->cpsr &= ~FLAG_V;
}

// a + b + cin, setting N Z C V. Subtraction is a + ~b + 1 (or + carry for SBC).
static uint32_t add_with_flags(ARM7TDMI *cpu, uint32_t a, uint32_t b, uint32_t cin) {
    uint64_t sum = (uint64_t)a + (uint64_t)b + (uint64_t)cin;
    uint32_t res = (uint32_t)sum;
    set_nz(cpu, res);
    set_c(cpu, sum > 0xFFFFFFFFu);
    set_v(cpu, (int)(((~(a ^ b) & (a ^ res)) >> 31) & 1));
    return res;
}

static int cond_pass(ARM7TDMI *cpu, uint32_t cond) {
    uint32_t f = cpu->cpsr;
    int n = (f >> 31) & 1, z = (f >> 30) & 1, c = (f >> 29) & 1, v = (f >> 28) & 1;
    switch (cond) {
        case 0x0: return z;             // EQ
        case 0x1: return !z;            // NE
        case 0x2: return c;             // CS
        case 0x3: return !c;            // CC
        case 0x4: return n;             // MI
        case 0x5: return !n;            // PL
        case 0x6: return v;             // VS
        case 0x7: return !v;            // VC
        case 0x8: return c && !z;       // HI
        case 0x9: return !c || z;       // LS
        case 0xA: return n == v;        // GE
        case 0xB: return n != v;        // LT
        case 0xC: return !z && n == v;  // GT
        case 0xD: return z || n != v;   // LE
        case 0xE: return 1;             // AL
        default:  return 0;             // NV (never)
    }
}

static uint32_t ror32(uint32_t v, uint32_t n) {
    n &= 31;
    return n ? ((v >> n) | (v << (32 - n))) : v;
}

static int popcount16(uint32_t v) {
    int count = 0;
    for (int i = 0; i < 16; i++) {
        if (v & (1u << i)) count++;
    }
    return count;
}

// ---------------------------------------------------------------------------
// Barrel shifter
// *carry must be preloaded with the current C flag; updated with the shifter
// carry-out where the architecture defines one.
// ---------------------------------------------------------------------------

// Shift by immediate amount (ARM DP operand2, LDR/STR register offsets,
// Thumb format 1). Amount 0 encodes: LSL #0 (pass-through), LSR/ASR #32, RRX.
static uint32_t shift_by_imm(uint32_t val, uint32_t type, uint32_t amount, int *carry) {
    switch (type) {
        case 0: // LSL
            if (amount == 0) return val;
            *carry = (int)((val >> (32 - amount)) & 1);
            return val << amount;
        case 1: // LSR (#0 means #32)
            if (amount == 0) { *carry = (int)((val >> 31) & 1); return 0; }
            *carry = (int)((val >> (amount - 1)) & 1);
            return val >> amount;
        case 2: // ASR (#0 means #32)
            if (amount == 0) {
                *carry = (int)((val >> 31) & 1);
                return (uint32_t)((int32_t)val >> 31);
            }
            *carry = (int)(((uint32_t)((int32_t)val >> (amount - 1))) & 1);
            return (uint32_t)((int32_t)val >> amount);
        default: // ROR (#0 means RRX)
            if (amount == 0) {
                uint32_t old_c = (uint32_t)(*carry & 1);
                *carry = (int)(val & 1);
                return (old_c << 31) | (val >> 1);
            }
            *carry = (int)((val >> (amount - 1)) & 1);
            return ror32(val, amount);
    }
}

// Shift by register amount (bottom byte of Rs). Amount 0 leaves value and
// carry unchanged; amounts >= 32 defined per the ARM ARM.
static uint32_t shift_by_reg(uint32_t val, uint32_t type, uint32_t amount, int *carry) {
    if (amount == 0) return val;
    switch (type) {
        case 0: // LSL
            if (amount < 32) { *carry = (int)((val >> (32 - amount)) & 1); return val << amount; }
            *carry = (amount == 32) ? (int)(val & 1) : 0;
            return 0;
        case 1: // LSR
            if (amount < 32) { *carry = (int)((val >> (amount - 1)) & 1); return val >> amount; }
            *carry = (amount == 32) ? (int)((val >> 31) & 1) : 0;
            return 0;
        case 2: // ASR
            if (amount >= 32) {
                *carry = (int)((val >> 31) & 1);
                return (uint32_t)((int32_t)val >> 31);
            }
            *carry = (int)(((uint32_t)((int32_t)val >> (amount - 1))) & 1);
            return (uint32_t)((int32_t)val >> amount);
        default: { // ROR
            uint32_t rot = amount & 31;
            if (rot == 0) { *carry = (int)((val >> 31) & 1); return val; }
            *carry = (int)((val >> (rot - 1)) & 1);
            return ror32(val, rot);
        }
    }
}

// Decode an ARM register-form operand2 (Rm shifted). Used by data processing
// and by LDR/STR register offsets (which only use the immediate-shift form).
// Note: when shifting by register, real hardware reads PC as instruction + 12;
// this core uses + 8 uniformly for simplicity.
static uint32_t arm_shifter_operand(ARM7TDMI *cpu, uint32_t instr, int *carry) {
    uint32_t val = rreg(cpu, (int)(instr & 0xF));
    uint32_t type = (instr >> 5) & 3;
    if (instr & 0x10) { // shift amount in bottom byte of Rs
        uint32_t amount = rreg(cpu, (int)((instr >> 8) & 0xF)) & 0xFF;
        return shift_by_reg(val, type, amount, carry);
    }
    return shift_by_imm(val, type, (instr >> 7) & 0x1F, carry);
}

// ---------------------------------------------------------------------------
// ARM state execution
// ---------------------------------------------------------------------------

static int arm_exec(ARM7TDMI *cpu, uint32_t instr) {
    if (!cond_pass(cpu, instr >> 28)) return 0;

    // BX: branch and exchange (bit 0 of target selects Thumb state)
    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) {
        uint32_t target = rreg(cpu, (int)(instr & 0xF));
        if (target & 1) {
            cpu->cpsr |= FLAG_T;
            cpu->r[15] = target & ~1u;
        } else {
            cpu->cpsr &= ~FLAG_T;
            cpu->r[15] = target & ~3u;
        }
        cpu->pc_written = 1;
        return 0;
    }

    // MUL / MLA
    if ((instr & 0x0FC000F0u) == 0x00000090u) {
        int rd = (int)((instr >> 16) & 0xF);
        int rn = (int)((instr >> 12) & 0xF);
        int rs = (int)((instr >> 8) & 0xF);
        int rm = (int)(instr & 0xF);
        uint32_t res = rreg(cpu, rm) * rreg(cpu, rs);
        if (instr & (1u << 21)) res += rreg(cpu, rn); // MLA
        wreg(cpu, rd, res);
        if (instr & (1u << 20)) set_nz(cpu, res); // C is unpredictable; left unchanged
        return 0;
    }

    // UMULL / UMLAL / SMULL / SMLAL
    if ((instr & 0x0F8000F0u) == 0x00800090u) {
        int rdhi = (int)((instr >> 16) & 0xF);
        int rdlo = (int)((instr >> 12) & 0xF);
        int rs = (int)((instr >> 8) & 0xF);
        int rm = (int)(instr & 0xF);
        uint64_t res;
        if (instr & (1u << 22)) { // signed
            res = (uint64_t)((int64_t)(int32_t)rreg(cpu, rm) * (int64_t)(int32_t)rreg(cpu, rs));
        } else {
            res = (uint64_t)rreg(cpu, rm) * (uint64_t)rreg(cpu, rs);
        }
        if (instr & (1u << 21)) { // accumulate
            res += ((uint64_t)rreg(cpu, rdhi) << 32) | (uint64_t)rreg(cpu, rdlo);
        }
        wreg(cpu, rdlo, (uint32_t)res);
        wreg(cpu, rdhi, (uint32_t)(res >> 32));
        if (instr & (1u << 20)) {
            cpu->cpsr &= ~(FLAG_N | FLAG_Z);
            if (res & 0x8000000000000000ull) cpu->cpsr |= FLAG_N;
            if (res == 0) cpu->cpsr |= FLAG_Z;
        }
        return 0;
    }

    // SWP / SWPB
    if ((instr & 0x0FB00FF0u) == 0x01000090u) {
        uint32_t addr = rreg(cpu, (int)((instr >> 16) & 0xF));
        int rd = (int)((instr >> 12) & 0xF);
        int rm = (int)(instr & 0xF);
        if (instr & (1u << 22)) { // byte
            uint32_t tmp = mem_read8(cpu, addr);
            mem_write8(cpu, addr, (uint8_t)rreg(cpu, rm));
            wreg(cpu, rd, tmp);
        } else {
            uint32_t tmp = ror32(mem_read32(cpu, addr), (addr & 3) * 8);
            mem_write32(cpu, addr, rreg(cpu, rm));
            wreg(cpu, rd, tmp);
        }
        return 0;
    }

    // Halfword and signed transfers: LDRH / STRH / LDRSB / LDRSH
    if ((instr & 0x0E000090u) == 0x00000090u && (instr & 0x60u) != 0) {
        int p = (int)((instr >> 24) & 1);
        int u = (int)((instr >> 23) & 1);
        int imm = (int)((instr >> 22) & 1);
        int w = (int)((instr >> 21) & 1);
        int l = (int)((instr >> 20) & 1);
        int rn = (int)((instr >> 16) & 0xF);
        int rd = (int)((instr >> 12) & 0xF);
        uint32_t sh = (instr >> 5) & 3;
        uint32_t offset = imm ? ((instr & 0xF) | ((instr >> 4) & 0xF0))
                              : rreg(cpu, (int)(instr & 0xF));
        uint32_t base = rreg(cpu, rn);
        uint32_t new_base = u ? base + offset : base - offset;
        uint32_t eff = p ? new_base : base;
        if (l) {
            uint32_t val;
            switch (sh) {
                case 1:  val = mem_read16(cpu, eff); break;                          // LDRH
                case 2:  val = (uint32_t)(int32_t)(int8_t)mem_read8(cpu, eff); break; // LDRSB
                default: val = (uint32_t)(int32_t)(int16_t)mem_read16(cpu, eff);      // LDRSH
            }
            if (!p || w) wreg(cpu, rn, new_base);
            wreg(cpu, rd, val); // loaded value wins over writeback when rd == rn
        } else {
            if (sh != 1) return -4; // LDRD/STRD are ARMv5E, absent on ARM7TDMI
            mem_write16(cpu, eff, (uint16_t)rreg(cpu, rd)); // STRH
            if (!p || w) wreg(cpu, rn, new_base);
        }
        return 0;
    }

    // MRS: read status register (SPSR access reads CPSR; no banked SPSRs)
    if ((instr & 0x0FBF0FFFu) == 0x010F0000u) {
        wreg(cpu, (int)((instr >> 12) & 0xF), cpu->cpsr);
        return 0;
    }

    // MSR: write status register fields
    if ((instr & 0x0DB0F000u) == 0x0120F000u) {
        uint32_t val;
        if (instr & (1u << 25)) {
            val = ror32(instr & 0xFF, ((instr >> 8) & 0xF) * 2);
        } else {
            val = rreg(cpu, (int)(instr & 0xF));
        }
        // User mode: only the flags field (bits 31-24) is writable; the control
        // field and SPSR targets (bit 22) are ignored (privileged modes not modeled).
        if ((instr & (1u << 19)) && !(instr & (1u << 22))) {
            cpu->cpsr = (cpu->cpsr & 0x00FFFFFFu) | (val & 0xFF000000u);
        }
        return 0;
    }

    // Data processing (AND EOR SUB RSB ADD ADC SBC RSC TST TEQ CMP CMN ORR MOV BIC MVN)
    if ((instr & 0x0C000000u) == 0x00000000u) {
        uint32_t op = (instr >> 21) & 0xF;
        int s = (int)((instr >> 20) & 1);
        int rn = (int)((instr >> 16) & 0xF);
        int rd = (int)((instr >> 12) & 0xF);
        int carry = (int)((cpu->cpsr >> 29) & 1);
        uint32_t cin = (cpu->cpsr >> 29) & 1;
        uint32_t op2;
        if (instr & (1u << 25)) { // immediate, rotated by 2 * rot field
            uint32_t rot = ((instr >> 8) & 0xF) * 2;
            op2 = ror32(instr & 0xFF, rot);
            if (rot != 0) carry = (int)((op2 >> 31) & 1);
        } else {
            op2 = arm_shifter_operand(cpu, instr, &carry);
        }
        uint32_t a = rreg(cpu, rn);
        uint32_t res = 0;
        int write = 1, logical = 0;
        switch (op) {
            case 0x0: res = a & op2; logical = 1; break;                              // AND
            case 0x1: res = a ^ op2; logical = 1; break;                              // EOR
            case 0x2: res = s ? add_with_flags(cpu, a, ~op2, 1) : a - op2; break;     // SUB
            case 0x3: res = s ? add_with_flags(cpu, op2, ~a, 1) : op2 - a; break;     // RSB
            case 0x4: res = s ? add_with_flags(cpu, a, op2, 0) : a + op2; break;      // ADD
            case 0x5: res = s ? add_with_flags(cpu, a, op2, cin)
                              : a + op2 + cin; break;                                 // ADC
            case 0x6: res = s ? add_with_flags(cpu, a, ~op2, cin)
                              : a - op2 + cin - 1; break;                             // SBC
            case 0x7: res = s ? add_with_flags(cpu, op2, ~a, cin)
                              : op2 - a + cin - 1; break;                             // RSC
            case 0x8: res = a & op2; logical = 1; write = 0; break;                   // TST
            case 0x9: res = a ^ op2; logical = 1; write = 0; break;                   // TEQ
            case 0xA: res = add_with_flags(cpu, a, ~op2, 1); write = 0; break;        // CMP
            case 0xB: res = add_with_flags(cpu, a, op2, 0); write = 0; break;         // CMN
            case 0xC: res = a | op2; logical = 1; break;                              // ORR
            case 0xD: res = op2; logical = 1; break;                                  // MOV
            case 0xE: res = a & ~op2; logical = 1; break;                             // BIC
            default:  res = ~op2; logical = 1; break;                                 // MVN
        }
        if (s && logical) { set_nz(cpu, res); set_c(cpu, carry); }
        // rd == 15 with S set restores SPSR (exception return) on real hardware;
        // simplified here: flags update normally, no banked SPSR exists.
        if (write) wreg(cpu, rd, res);
        return 0;
    }

    // LDR / STR (word/byte, immediate or scaled register offset)
    if ((instr & 0x0C000000u) == 0x04000000u) {
        int p = (int)((instr >> 24) & 1);
        int u = (int)((instr >> 23) & 1);
        int b = (int)((instr >> 22) & 1);
        int w = (int)((instr >> 21) & 1);
        int l = (int)((instr >> 20) & 1);
        int rn = (int)((instr >> 16) & 0xF);
        int rd = (int)((instr >> 12) & 0xF);
        uint32_t offset;
        if (instr & (1u << 25)) { // register offset, shifted by immediate
            int carry = (int)((cpu->cpsr >> 29) & 1); // shifter carry discarded
            offset = arm_shifter_operand(cpu, instr, &carry);
        } else {
            offset = instr & 0xFFF;
        }
        uint32_t base = rreg(cpu, rn);
        uint32_t new_base = u ? base + offset : base - offset;
        uint32_t eff = p ? new_base : base;
        if (l) {
            uint32_t val = b ? mem_read8(cpu, eff)
                             : ror32(mem_read32(cpu, eff), (eff & 3) * 8);
            if (!p || w) wreg(cpu, rn, new_base); // post-index always writes back
            wreg(cpu, rd, val);                   // loaded value wins when rd == rn
        } else {
            uint32_t val = rreg(cpu, rd); // STR of PC stores PC+12 on hardware; PC+8 here
            if (b) mem_write8(cpu, eff, (uint8_t)val);
            else mem_write32(cpu, eff, val);
            if (!p || w) wreg(cpu, rn, new_base);
        }
        return 0;
    }

    // LDM / STM (all addressing modes: IA IB DA DB)
    if ((instr & 0x0E000000u) == 0x08000000u) {
        int p = (int)((instr >> 24) & 1);
        int u = (int)((instr >> 23) & 1);
        int w = (int)((instr >> 21) & 1);
        int l = (int)((instr >> 20) & 1);
        // S bit (22) selects user-bank transfer / SPSR restore on real
        // hardware; ignored here since only user mode is modeled.
        int rn = (int)((instr >> 16) & 0xF);
        uint32_t list = instr & 0xFFFF;
        uint32_t count = (uint32_t)popcount16(list);
        uint32_t base = rreg(cpu, rn);
        uint32_t addr, wb;
        if (u) {
            wb = base + 4u * count;
            addr = p ? base + 4 : base;
        } else {
            wb = base - 4u * count;
            addr = p ? wb : wb + 4;
        }
        for (int i = 0; i < 16; i++) {
            if (!(list & (1u << i))) continue;
            if (l) {
                uint32_t val = mem_read32(cpu, addr);
                if (i == 15) wreg(cpu, 15, val);
                else cpu->r[i] = val;
            } else {
                mem_write32(cpu, addr, rreg(cpu, i)); // R15 stores PC+8 (hardware: +12)
            }
            addr += 4;
        }
        // Writeback; on LDM with the base in the list the loaded value wins.
        // (STM with base in the list stores the original base, which matches
        // hardware when the base is the first register listed.)
        if (w && rn != 15 && !(l && (list & (1u << rn)))) cpu->r[rn] = wb;
        return 0;
    }

    // B / BL
    if ((instr & 0x0E000000u) == 0x0A000000u) {
        int32_t off = (int32_t)(instr << 8) >> 6; // sign-extended 24-bit offset << 2
        if (instr & (1u << 24)) cpu->r[14] = cpu->r[15] + 4; // BL: link to next instruction
        wreg(cpu, 15, rreg(cpu, 15) + (uint32_t)off);
        return 0;
    }

    // SWI: software interrupt. Real hardware enters supervisor mode via the
    // exception vector; simplified here to halt the emulator.
    if ((instr & 0x0F000000u) == 0x0F000000u) {
        cpu->halted = 1;
        return 1;
    }

    return -4; // Unknown/undefined instruction
}

// ---------------------------------------------------------------------------
// Thumb state execution (all 19 Thumb instruction formats)
// ---------------------------------------------------------------------------

static int thumb_exec(ARM7TDMI *cpu, uint16_t instr) {
    switch (instr >> 13) {
        case 0: { // Formats 1-2
            if ((instr & 0x1800) == 0x1800) { // Format 2: ADD/SUB reg or 3-bit imm
                int i = (instr >> 10) & 1;
                int op = (instr >> 9) & 1;
                int rn_imm = (instr >> 6) & 7;
                int rs = (instr >> 3) & 7;
                int rd = instr & 7;
                uint32_t b = i ? (uint32_t)rn_imm : cpu->r[rn_imm];
                uint32_t a = cpu->r[rs];
                cpu->r[rd] = op ? add_with_flags(cpu, a, ~b, 1)
                                : add_with_flags(cpu, a, b, 0);
            } else { // Format 1: LSL/LSR/ASR by immediate
                uint32_t op = (instr >> 11) & 3;
                uint32_t amount = (instr >> 6) & 0x1F;
                int rs = (instr >> 3) & 7;
                int rd = instr & 7;
                int carry = (int)((cpu->cpsr >> 29) & 1);
                uint32_t res = shift_by_imm(cpu->r[rs], op, amount, &carry);
                cpu->r[rd] = res;
                set_nz(cpu, res);
                set_c(cpu, carry);
            }
            return 0;
        }
        case 1: { // Format 3: MOV/CMP/ADD/SUB with 8-bit immediate
            uint32_t op = (instr >> 11) & 3;
            int rd = (instr >> 8) & 7;
            uint32_t imm = instr & 0xFF;
            switch (op) {
                case 0: cpu->r[rd] = imm; set_nz(cpu, imm); break;                     // MOV
                case 1: add_with_flags(cpu, cpu->r[rd], ~imm, 1); break;               // CMP
                case 2: cpu->r[rd] = add_with_flags(cpu, cpu->r[rd], imm, 0); break;   // ADD
                default: cpu->r[rd] = add_with_flags(cpu, cpu->r[rd], ~imm, 1); break; // SUB
            }
            return 0;
        }
        case 2: { // Formats 4-8
            if ((instr & 0xFC00) == 0x4000) { // Format 4: ALU operations
                uint32_t op = (instr >> 6) & 0xF;
                int rs = (instr >> 3) & 7;
                int rd = instr & 7;
                uint32_t a = cpu->r[rd], b = cpu->r[rs];
                uint32_t cin = (cpu->cpsr >> 29) & 1;
                int carry = (int)cin;
                uint32_t res;
                switch (op) {
                    case 0x0: res = a & b; set_nz(cpu, res); cpu->r[rd] = res; break;  // AND
                    case 0x1: res = a ^ b; set_nz(cpu, res); cpu->r[rd] = res; break;  // EOR
                    case 0x2: res = shift_by_reg(a, 0, b & 0xFF, &carry);              // LSL
                              set_nz(cpu, res); set_c(cpu, carry); cpu->r[rd] = res; break;
                    case 0x3: res = shift_by_reg(a, 1, b & 0xFF, &carry);              // LSR
                              set_nz(cpu, res); set_c(cpu, carry); cpu->r[rd] = res; break;
                    case 0x4: res = shift_by_reg(a, 2, b & 0xFF, &carry);              // ASR
                              set_nz(cpu, res); set_c(cpu, carry); cpu->r[rd] = res; break;
                    case 0x5: cpu->r[rd] = add_with_flags(cpu, a, b, cin); break;      // ADC
                    case 0x6: cpu->r[rd] = add_with_flags(cpu, a, ~b, cin); break;     // SBC
                    case 0x7: res = shift_by_reg(a, 3, b & 0xFF, &carry);              // ROR
                              set_nz(cpu, res); set_c(cpu, carry); cpu->r[rd] = res; break;
                    case 0x8: set_nz(cpu, a & b); break;                               // TST
                    case 0x9: cpu->r[rd] = add_with_flags(cpu, 0, ~b, 1); break;       // NEG
                    case 0xA: add_with_flags(cpu, a, ~b, 1); break;                    // CMP
                    case 0xB: add_with_flags(cpu, a, b, 0); break;                     // CMN
                    case 0xC: res = a | b; set_nz(cpu, res); cpu->r[rd] = res; break;  // ORR
                    case 0xD: res = a * b; set_nz(cpu, res); cpu->r[rd] = res; break;  // MUL
                    case 0xE: res = a & ~b; set_nz(cpu, res); cpu->r[rd] = res; break; // BIC
                    default:  res = ~b; set_nz(cpu, res); cpu->r[rd] = res; break;     // MVN
                }
            } else if ((instr & 0xFC00) == 0x4400) { // Format 5: hi register ops / BX
                uint32_t op = (instr >> 8) & 3;
                int rd = (instr & 7) | ((instr >> 4) & 8);
                int rs = (instr >> 3) & 0xF;
                uint32_t vs = rreg(cpu, rs);
                switch (op) {
                    case 0: wreg(cpu, rd, rreg(cpu, rd) + vs); break;   // ADD (no flags)
                    case 1: add_with_flags(cpu, rreg(cpu, rd), ~vs, 1); break; // CMP
                    case 2: wreg(cpu, rd, vs); break;                   // MOV
                    default: // BX
                        if (vs & 1) {
                            cpu->cpsr |= FLAG_T;
                            cpu->r[15] = vs & ~1u;
                        } else {
                            cpu->cpsr &= ~FLAG_T;
                            cpu->r[15] = vs & ~3u;
                        }
                        cpu->pc_written = 1;
                        break;
                }
            } else if ((instr & 0xF800) == 0x4800) { // Format 6: PC-relative load
                int rd = (instr >> 8) & 7;
                uint32_t imm = (uint32_t)(instr & 0xFF) * 4;
                cpu->r[rd] = mem_read32(cpu, ((cpu->r[15] + 4) & ~3u) + imm);
            } else if ((instr & 0xF200) == 0x5000) { // Format 7: ld/st with register offset
                int l = (instr >> 11) & 1;
                int b = (instr >> 10) & 1;
                uint32_t addr = cpu->r[(instr >> 3) & 7] + cpu->r[(instr >> 6) & 7];
                int rd = instr & 7;
                if (l) {
                    cpu->r[rd] = b ? mem_read8(cpu, addr)
                                   : ror32(mem_read32(cpu, addr), (addr & 3) * 8);
                } else {
                    if (b) mem_write8(cpu, addr, (uint8_t)cpu->r[rd]);
                    else mem_write32(cpu, addr, cpu->r[rd]);
                }
            } else { // Format 8: ld/st sign-extended byte/halfword
                int h = (instr >> 11) & 1;
                int s = (instr >> 10) & 1;
                uint32_t addr = cpu->r[(instr >> 3) & 7] + cpu->r[(instr >> 6) & 7];
                int rd = instr & 7;
                if (!s && !h) mem_write16(cpu, addr, (uint16_t)cpu->r[rd]);               // STRH
                else if (!s) cpu->r[rd] = mem_read16(cpu, addr);                          // LDRH
                else if (!h) cpu->r[rd] = (uint32_t)(int32_t)(int8_t)mem_read8(cpu, addr);   // LDRSB
                else cpu->r[rd] = (uint32_t)(int32_t)(int16_t)mem_read16(cpu, addr);         // LDRSH
            }
            return 0;
        }
        case 3: { // Format 9: ld/st word/byte with 5-bit immediate offset
            int b = (instr >> 12) & 1;
            int l = (instr >> 11) & 1;
            uint32_t off = (instr >> 6) & 0x1F;
            int rb = (instr >> 3) & 7;
            int rd = instr & 7;
            if (b) {
                uint32_t addr = cpu->r[rb] + off;
                if (l) cpu->r[rd] = mem_read8(cpu, addr);
                else mem_write8(cpu, addr, (uint8_t)cpu->r[rd]);
            } else {
                uint32_t addr = cpu->r[rb] + off * 4;
                if (l) cpu->r[rd] = ror32(mem_read32(cpu, addr), (addr & 3) * 8);
                else mem_write32(cpu, addr, cpu->r[rd]);
            }
            return 0;
        }
        case 4: { // Formats 10-11
            int l = (instr >> 11) & 1;
            if (!(instr & 0x1000)) { // Format 10: ld/st halfword, imm offset
                uint32_t addr = cpu->r[(instr >> 3) & 7] + ((instr >> 6) & 0x1F) * 2u;
                int rd = instr & 7;
                if (l) cpu->r[rd] = mem_read16(cpu, addr);
                else mem_write16(cpu, addr, (uint16_t)cpu->r[rd]);
            } else { // Format 11: SP-relative ld/st
                int rd = (instr >> 8) & 7;
                uint32_t addr = cpu->r[13] + (uint32_t)(instr & 0xFF) * 4;
                if (l) cpu->r[rd] = ror32(mem_read32(cpu, addr), (addr & 3) * 8);
                else mem_write32(cpu, addr, cpu->r[rd]);
            }
            return 0;
        }
        case 5: { // Formats 12-14
            if (!(instr & 0x1000)) { // Format 12: load address (ADD Rd, PC/SP, #imm)
                int rd = (instr >> 8) & 7;
                uint32_t imm = (uint32_t)(instr & 0xFF) * 4;
                if (instr & 0x0800) cpu->r[rd] = cpu->r[13] + imm;
                else cpu->r[rd] = ((cpu->r[15] + 4) & ~3u) + imm;
            } else if ((instr & 0xFF00) == 0xB000) { // Format 13: add offset to SP
                uint32_t imm = (uint32_t)(instr & 0x7F) * 4;
                if (instr & 0x80) cpu->r[13] -= imm;
                else cpu->r[13] += imm;
            } else if ((instr & 0xF600) == 0xB400) { // Format 14: PUSH/POP
                int l = (instr >> 11) & 1;
                int lrpc = (instr >> 8) & 1;
                uint32_t list = instr & 0xFF;
                if (l) { // POP {rlist[, PC]}
                    uint32_t addr = cpu->r[13];
                    for (int i = 0; i < 8; i++) {
                        if (list & (1u << i)) { cpu->r[i] = mem_read32(cpu, addr); addr += 4; }
                    }
                    if (lrpc) {
                        // ARMv4T: POP to PC stays in Thumb state (no BX behavior)
                        wreg(cpu, 15, mem_read32(cpu, addr));
                        addr += 4;
                    }
                    cpu->r[13] = addr;
                } else { // PUSH {rlist[, LR]}
                    uint32_t count = (uint32_t)popcount16(list) + (uint32_t)lrpc;
                    uint32_t addr = cpu->r[13] - 4u * count;
                    cpu->r[13] = addr;
                    for (int i = 0; i < 8; i++) {
                        if (list & (1u << i)) { mem_write32(cpu, addr, cpu->r[i]); addr += 4; }
                    }
                    if (lrpc) mem_write32(cpu, addr, cpu->r[14]);
                }
            } else {
                return -4; // BKPT and other 0xB* gaps are not ARMv4T
            }
            return 0;
        }
        case 6: { // Formats 15-17
            if (!(instr & 0x1000)) { // Format 15: LDMIA/STMIA Rb!, {rlist}
                int l = (instr >> 11) & 1;
                int rb = (instr >> 8) & 7;
                uint32_t list = instr & 0xFF;
                uint32_t addr = cpu->r[rb];
                for (int i = 0; i < 8; i++) {
                    if (!(list & (1u << i))) continue;
                    if (l) cpu->r[i] = mem_read32(cpu, addr);
                    else mem_write32(cpu, addr, cpu->r[i]);
                    addr += 4;
                }
                // Writeback; on LDMIA with base in the list, the loaded value wins
                if (!(l && (list & (1u << rb)))) cpu->r[rb] = addr;
            } else {
                uint32_t cond = (instr >> 8) & 0xF;
                if (cond == 0xF) { // Format 17: SWI halts (see ARM SWI note)
                    cpu->halted = 1;
                    return 1;
                }
                if (cond == 0xE) return -4; // undefined
                if (cond_pass(cpu, cond)) { // Format 16: conditional branch
                    int32_t off = (int32_t)(int8_t)(instr & 0xFF) * 2;
                    wreg(cpu, 15, cpu->r[15] + 4 + (uint32_t)off);
                }
            }
            return 0;
        }
        default: { // Formats 18-19
            if ((instr & 0xF800) == 0xE000) { // Format 18: unconditional branch
                int32_t off = (int32_t)((uint32_t)(instr & 0x7FF) << 21) >> 20;
                wreg(cpu, 15, cpu->r[15] + 4 + (uint32_t)off);
                return 0;
            }
            if ((instr & 0xF800) == 0xF000) { // Format 19: BL prefix (high offset)
                int32_t off = (int32_t)((uint32_t)(instr & 0x7FF) << 21) >> 9;
                cpu->r[14] = cpu->r[15] + 4 + (uint32_t)off;
                return 0;
            }
            if ((instr & 0xF800) == 0xF800) { // Format 19: BL suffix (low offset)
                uint32_t target = cpu->r[14] + ((uint32_t)(instr & 0x7FF) << 1);
                cpu->r[14] = (cpu->r[15] + 2) | 1; // return address, Thumb bit set
                wreg(cpu, 15, target);
                return 0;
            }
            return -4; // 0xE800 range is BLX suffix (ARMv5), undefined on ARMv4T
        }
    }
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------

int arm7tdmi_step(void *context) {
    if (!context) return -1;
    ARM7TDMI *cpu = (ARM7TDMI*)context;

    if (cpu->halted) return 1;

    uint32_t old_pc = cpu->r[15];
    cpu->pc_written = 0;
    cpu->ticks++;

    int ret;
    if (cpu->cpsr & FLAG_T) {
        ret = thumb_exec(cpu, mem_read16(cpu, old_pc));
        if (!cpu->pc_written) cpu->r[15] = old_pc + 2;
    } else {
        ret = arm_exec(cpu, mem_read32(cpu, old_pc));
        if (!cpu->pc_written) cpu->r[15] = old_pc + 4;
    }
    if (ret != 0) return ret;

    // Branch-to-self idiom halts (common "end of program" spin)
    if (cpu->r[15] == old_pc) {
        cpu->halted = 1;
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// State dump
// ---------------------------------------------------------------------------

void arm7tdmi_print_state(void *context) {
    if (!context) return;
    ARM7TDMI *cpu = (ARM7TDMI*)context;
    uint32_t f = cpu->cpsr;

    printf("ARM7TDMI CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%08X  Mode: %s  Halted: %s\n",
           cpu->r[15], (f & FLAG_T) ? "Thumb" : "ARM", cpu->halted ? "Yes" : "No");
    printf("  CPSR: 0x%08X [%c%c%c%c]\n", f,
           (f & FLAG_N) ? 'N' : '-', (f & FLAG_Z) ? 'Z' : '-',
           (f & FLAG_C) ? 'C' : '-', (f & FLAG_V) ? 'V' : '-');
    printf("  Registers:\n");
    for (int i = 0; i < 16; i++) {
        printf("    %-3s(r%02d): 0x%08X%s",
               reg_names[i], i, cpu->r[i],
               (i % 4 == 3) ? "\n" : "  ");
    }
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

static void arm_format_op2(uint32_t instr, char *buf, size_t len) {
    if (instr & (1u << 25)) {
        snprintf(buf, len, "#0x%X", ror32(instr & 0xFF, ((instr >> 8) & 0xF) * 2));
        return;
    }
    uint32_t rm = instr & 0xF;
    uint32_t type = (instr >> 5) & 3;
    if (instr & 0x10) { // shift by register
        snprintf(buf, len, "%s, %s %s", reg_names[rm], shift_names[type],
                 reg_names[(instr >> 8) & 0xF]);
        return;
    }
    uint32_t amount = (instr >> 7) & 0x1F;
    if (amount == 0) {
        if (type == 0) snprintf(buf, len, "%s", reg_names[rm]);
        else if (type == 3) snprintf(buf, len, "%s, rrx", reg_names[rm]);
        else snprintf(buf, len, "%s, %s #32", reg_names[rm], shift_names[type]);
    } else {
        snprintf(buf, len, "%s, %s #%u", reg_names[rm], shift_names[type], amount);
    }
}

static void arm_format_reglist(uint32_t list, char *buf, size_t len) {
    size_t pos = 0;
    int first = 1;
    pos += (size_t)snprintf(buf + pos, len - pos, "{");
    for (int i = 0; i < 16 && pos < len; i++) {
        if (!(list & (1u << i))) continue;
        pos += (size_t)snprintf(buf + pos, len - pos, first ? "%s" : ", %s", reg_names[i]);
        first = 0;
        if (pos >= len) return;
    }
    if (pos < len) snprintf(buf + pos, len - pos, "}");
}

static void arm_disasm(uint32_t instr, uint32_t pc, char *buf, size_t len) {
    const char *cc = cond_names[instr >> 28];
    char op2[64];

    if ((instr >> 28) == 0xF) {
        snprintf(buf, len, "undefined (0x%08X)", instr);
        return;
    }
    if ((instr & 0x0FFFFFF0u) == 0x012FFF10u) {
        snprintf(buf, len, "bx%s %s", cc, reg_names[instr & 0xF]);
        return;
    }
    if ((instr & 0x0FC000F0u) == 0x00000090u) {
        const char *s = (instr & (1u << 20)) ? "s" : "";
        if (instr & (1u << 21)) {
            snprintf(buf, len, "mla%s%s %s, %s, %s, %s", cc, s,
                     reg_names[(instr >> 16) & 0xF], reg_names[instr & 0xF],
                     reg_names[(instr >> 8) & 0xF], reg_names[(instr >> 12) & 0xF]);
        } else {
            snprintf(buf, len, "mul%s%s %s, %s, %s", cc, s,
                     reg_names[(instr >> 16) & 0xF], reg_names[instr & 0xF],
                     reg_names[(instr >> 8) & 0xF]);
        }
        return;
    }
    if ((instr & 0x0F8000F0u) == 0x00800090u) {
        static const char *lmul[4] = { "umull", "umlal", "smull", "smlal" };
        snprintf(buf, len, "%s%s%s %s, %s, %s, %s",
                 lmul[(instr >> 21) & 3], cc, (instr & (1u << 20)) ? "s" : "",
                 reg_names[(instr >> 12) & 0xF], reg_names[(instr >> 16) & 0xF],
                 reg_names[instr & 0xF], reg_names[(instr >> 8) & 0xF]);
        return;
    }
    if ((instr & 0x0FB00FF0u) == 0x01000090u) {
        snprintf(buf, len, "swp%s%s %s, %s, [%s]", cc,
                 (instr & (1u << 22)) ? "b" : "",
                 reg_names[(instr >> 12) & 0xF], reg_names[instr & 0xF],
                 reg_names[(instr >> 16) & 0xF]);
        return;
    }
    if ((instr & 0x0E000090u) == 0x00000090u && (instr & 0x60u) != 0) {
        static const char *hw_ld[4] = { "?", "ldrh", "ldrsb", "ldrsh" };
        uint32_t sh = (instr >> 5) & 3;
        const char *op = (instr & (1u << 20)) ? hw_ld[sh] : "strh";
        const char *rn = reg_names[(instr >> 16) & 0xF];
        const char *sign = (instr & (1u << 23)) ? "" : "-";
        if (instr & (1u << 22)) { // immediate offset
            uint32_t off = (instr & 0xF) | ((instr >> 4) & 0xF0);
            if (instr & (1u << 24)) {
                snprintf(buf, len, "%s%s %s, [%s, #%s%u]%s", op, cc,
                         reg_names[(instr >> 12) & 0xF], rn, sign, off,
                         (instr & (1u << 21)) ? "!" : "");
            } else {
                snprintf(buf, len, "%s%s %s, [%s], #%s%u", op, cc,
                         reg_names[(instr >> 12) & 0xF], rn, sign, off);
            }
        } else { // register offset
            if (instr & (1u << 24)) {
                snprintf(buf, len, "%s%s %s, [%s, %s%s]%s", op, cc,
                         reg_names[(instr >> 12) & 0xF], rn, sign,
                         reg_names[instr & 0xF], (instr & (1u << 21)) ? "!" : "");
            } else {
                snprintf(buf, len, "%s%s %s, [%s], %s%s", op, cc,
                         reg_names[(instr >> 12) & 0xF], rn, sign, reg_names[instr & 0xF]);
            }
        }
        return;
    }
    if ((instr & 0x0FBF0FFFu) == 0x010F0000u) {
        snprintf(buf, len, "mrs%s %s, %s", cc, reg_names[(instr >> 12) & 0xF],
                 (instr & (1u << 22)) ? "spsr" : "cpsr");
        return;
    }
    if ((instr & 0x0DB0F000u) == 0x0120F000u) {
        char fields[8];
        int fp = 0;
        if (instr & (1u << 19)) fields[fp++] = 'f';
        if (instr & (1u << 18)) fields[fp++] = 's';
        if (instr & (1u << 17)) fields[fp++] = 'x';
        if (instr & (1u << 16)) fields[fp++] = 'c';
        fields[fp] = '\0';
        if (instr & (1u << 25)) {
            snprintf(buf, len, "msr%s %s_%s, #0x%X", cc,
                     (instr & (1u << 22)) ? "spsr" : "cpsr", fields,
                     ror32(instr & 0xFF, ((instr >> 8) & 0xF) * 2));
        } else {
            snprintf(buf, len, "msr%s %s_%s, %s", cc,
                     (instr & (1u << 22)) ? "spsr" : "cpsr", fields,
                     reg_names[instr & 0xF]);
        }
        return;
    }
    if ((instr & 0x0C000000u) == 0x00000000u) {
        uint32_t op = (instr >> 21) & 0xF;
        const char *s = (instr & (1u << 20)) ? "s" : "";
        arm_format_op2(instr, op2, sizeof(op2));
        if (op >= 0x8 && op <= 0xB) { // TST TEQ CMP CMN: no rd
            snprintf(buf, len, "%s%s %s, %s", dp_names[op], cc,
                     reg_names[(instr >> 16) & 0xF], op2);
        } else if (op == 0xD || op == 0xF) { // MOV MVN: no rn
            snprintf(buf, len, "%s%s%s %s, %s", dp_names[op], cc, s,
                     reg_names[(instr >> 12) & 0xF], op2);
        } else {
            snprintf(buf, len, "%s%s%s %s, %s, %s", dp_names[op], cc, s,
                     reg_names[(instr >> 12) & 0xF], reg_names[(instr >> 16) & 0xF], op2);
        }
        return;
    }
    if ((instr & 0x0C000000u) == 0x04000000u) {
        const char *op = (instr & (1u << 20)) ? "ldr" : "str";
        const char *b = (instr & (1u << 22)) ? "b" : "";
        const char *rd = reg_names[(instr >> 12) & 0xF];
        const char *rn = reg_names[(instr >> 16) & 0xF];
        const char *sign = (instr & (1u << 23)) ? "" : "-";
        if (instr & (1u << 25)) { // register offset
            arm_format_op2(instr & ~(1u << 25), op2, sizeof(op2));
            if (instr & (1u << 24)) {
                snprintf(buf, len, "%s%s%s %s, [%s, %s%s]%s", op, cc, b, rd, rn, sign,
                         op2, (instr & (1u << 21)) ? "!" : "");
            } else {
                snprintf(buf, len, "%s%s%s %s, [%s], %s%s", op, cc, b, rd, rn, sign, op2);
            }
        } else {
            uint32_t off = instr & 0xFFF;
            if (instr & (1u << 24)) {
                snprintf(buf, len, "%s%s%s %s, [%s, #%s%u]%s", op, cc, b, rd, rn, sign,
                         off, (instr & (1u << 21)) ? "!" : "");
            } else {
                snprintf(buf, len, "%s%s%s %s, [%s], #%s%u", op, cc, b, rd, rn, sign, off);
            }
        }
        return;
    }
    if ((instr & 0x0E000000u) == 0x08000000u) {
        static const char *am[4] = { "da", "ia", "db", "ib" };
        uint32_t mode = ((instr >> 23) & 1) | ((instr >> 23) & 2); // P<<1 | U
        arm_format_reglist(instr & 0xFFFF, op2, sizeof(op2));
        snprintf(buf, len, "%s%s%s %s%s, %s%s",
                 (instr & (1u << 20)) ? "ldm" : "stm", cc, am[mode],
                 reg_names[(instr >> 16) & 0xF], (instr & (1u << 21)) ? "!" : "",
                 op2, (instr & (1u << 22)) ? "^" : "");
        return;
    }
    if ((instr & 0x0E000000u) == 0x0A000000u) {
        int32_t off = (int32_t)(instr << 8) >> 6;
        snprintf(buf, len, "b%s%s 0x%X", (instr & (1u << 24)) ? "l" : "", cc,
                 pc + 8 + (uint32_t)off);
        return;
    }
    if ((instr & 0x0F000000u) == 0x0F000000u) {
        snprintf(buf, len, "swi%s 0x%X", cc, instr & 0x00FFFFFF);
        return;
    }
    snprintf(buf, len, "unknown (0x%08X)", instr);
}

static void thumb_disasm(ARM7TDMI *cpu, uint32_t pc, char *buf, size_t len) {
    uint16_t instr = mem_read16(cpu, pc);
    char rl[64];

    switch (instr >> 13) {
        case 0:
            if ((instr & 0x1800) == 0x1800) { // Format 2
                const char *op = (instr & 0x0200) ? "sub" : "add";
                if (instr & 0x0400) {
                    snprintf(buf, len, "%s %s, %s, #%u", op, reg_names[instr & 7],
                             reg_names[(instr >> 3) & 7], (instr >> 6) & 7);
                } else {
                    snprintf(buf, len, "%s %s, %s, %s", op, reg_names[instr & 7],
                             reg_names[(instr >> 3) & 7], reg_names[(instr >> 6) & 7]);
                }
            } else { // Format 1
                static const char *sh[3] = { "lsl", "lsr", "asr" };
                snprintf(buf, len, "%s %s, %s, #%u", sh[(instr >> 11) & 3],
                         reg_names[instr & 7], reg_names[(instr >> 3) & 7],
                         (instr >> 6) & 0x1F);
            }
            return;
        case 1: { // Format 3
            static const char *ops[4] = { "mov", "cmp", "add", "sub" };
            snprintf(buf, len, "%s %s, #%u", ops[(instr >> 11) & 3],
                     reg_names[(instr >> 8) & 7], instr & 0xFF);
            return;
        }
        case 2:
            if ((instr & 0xFC00) == 0x4000) { // Format 4
                static const char *alu[16] = {
                    "and", "eor", "lsl", "lsr", "asr", "adc", "sbc", "ror",
                    "tst", "neg", "cmp", "cmn", "orr", "mul", "bic", "mvn"
                };
                snprintf(buf, len, "%s %s, %s", alu[(instr >> 6) & 0xF],
                         reg_names[instr & 7], reg_names[(instr >> 3) & 7]);
            } else if ((instr & 0xFC00) == 0x4400) { // Format 5
                static const char *ops[4] = { "add", "cmp", "mov", "bx" };
                uint32_t op = (instr >> 8) & 3;
                int rd = (instr & 7) | ((instr >> 4) & 8);
                int rs = (instr >> 3) & 0xF;
                if (op == 3) snprintf(buf, len, "bx %s", reg_names[rs]);
                else snprintf(buf, len, "%s %s, %s", ops[op], reg_names[rd], reg_names[rs]);
            } else if ((instr & 0xF800) == 0x4800) { // Format 6
                snprintf(buf, len, "ldr %s, [pc, #%u]", reg_names[(instr >> 8) & 7],
                         (instr & 0xFF) * 4);
            } else if ((instr & 0xF200) == 0x5000) { // Format 7
                static const char *ops[4] = { "str", "strb", "ldr", "ldrb" };
                snprintf(buf, len, "%s %s, [%s, %s]",
                         ops[((instr >> 11) & 1) * 2 + ((instr >> 10) & 1)],
                         reg_names[instr & 7], reg_names[(instr >> 3) & 7],
                         reg_names[(instr >> 6) & 7]);
            } else { // Format 8
                static const char *ops[4] = { "strh", "ldrsb", "ldrh", "ldrsh" };
                snprintf(buf, len, "%s %s, [%s, %s]",
                         ops[((instr >> 11) & 1) * 2 + ((instr >> 10) & 1)],
                         reg_names[instr & 7], reg_names[(instr >> 3) & 7],
                         reg_names[(instr >> 6) & 7]);
            }
            return;
        case 3: { // Format 9
            static const char *ops[4] = { "str", "ldr", "strb", "ldrb" };
            uint32_t idx = ((instr >> 12) & 1) * 2 + ((instr >> 11) & 1);
            uint32_t off = (instr >> 6) & 0x1F;
            snprintf(buf, len, "%s %s, [%s, #%u]", ops[idx], reg_names[instr & 7],
                     reg_names[(instr >> 3) & 7], (idx < 2) ? off * 4 : off);
            return;
        }
        case 4:
            if (!(instr & 0x1000)) { // Format 10
                snprintf(buf, len, "%s %s, [%s, #%u]",
                         (instr & 0x0800) ? "ldrh" : "strh", reg_names[instr & 7],
                         reg_names[(instr >> 3) & 7], ((instr >> 6) & 0x1F) * 2);
            } else { // Format 11
                snprintf(buf, len, "%s %s, [sp, #%u]",
                         (instr & 0x0800) ? "ldr" : "str", reg_names[(instr >> 8) & 7],
                         (instr & 0xFF) * 4);
            }
            return;
        case 5:
            if (!(instr & 0x1000)) { // Format 12
                snprintf(buf, len, "add %s, %s, #%u", reg_names[(instr >> 8) & 7],
                         (instr & 0x0800) ? "sp" : "pc", (instr & 0xFF) * 4);
            } else if ((instr & 0xFF00) == 0xB000) { // Format 13
                snprintf(buf, len, "%s sp, #%u", (instr & 0x80) ? "sub" : "add",
                         (instr & 0x7F) * 4);
            } else if ((instr & 0xF600) == 0xB400) { // Format 14
                uint32_t list = instr & 0xFF;
                int l = (instr >> 11) & 1;
                if (instr & 0x0100) list |= l ? 0x8000u : 0x4000u; // PC on pop, LR on push
                arm_format_reglist(list, rl, sizeof(rl));
                snprintf(buf, len, "%s %s", l ? "pop" : "push", rl);
            } else {
                snprintf(buf, len, "undefined (0x%04X)", instr);
            }
            return;
        case 6:
            if (!(instr & 0x1000)) { // Format 15
                arm_format_reglist(instr & 0xFF, rl, sizeof(rl));
                snprintf(buf, len, "%s %s!, %s", (instr & 0x0800) ? "ldmia" : "stmia",
                         reg_names[(instr >> 8) & 7], rl);
            } else {
                uint32_t cond = (instr >> 8) & 0xF;
                if (cond == 0xF) { // Format 17
                    snprintf(buf, len, "swi 0x%X", instr & 0xFF);
                } else if (cond == 0xE) {
                    snprintf(buf, len, "undefined (0x%04X)", instr);
                } else { // Format 16
                    int32_t off = (int32_t)(int8_t)(instr & 0xFF) * 2;
                    snprintf(buf, len, "b%s 0x%X", cond_names[cond],
                             pc + 4 + (uint32_t)off);
                }
            }
            return;
        default:
            if ((instr & 0xF800) == 0xE000) { // Format 18
                int32_t off = (int32_t)((uint32_t)(instr & 0x7FF) << 21) >> 20;
                snprintf(buf, len, "b 0x%X", pc + 4 + (uint32_t)off);
            } else if ((instr & 0xF800) == 0xF000) { // Format 19 (both halves)
                uint16_t next = mem_read16(cpu, pc + 2);
                if ((next & 0xF800) == 0xF800) {
                    int32_t hi = (int32_t)((uint32_t)(instr & 0x7FF) << 21) >> 9;
                    snprintf(buf, len, "bl 0x%X",
                             pc + 4 + (uint32_t)hi + ((uint32_t)(next & 0x7FF) << 1));
                } else {
                    snprintf(buf, len, "bl (prefix only)");
                }
            } else if ((instr & 0xF800) == 0xF800) {
                snprintf(buf, len, "bl (suffix)");
            } else {
                snprintf(buf, len, "undefined (0x%04X)", instr);
            }
            return;
    }
}

void arm7tdmi_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    ARM7TDMI *cpu = (ARM7TDMI*)context;

    if (cpu->cpsr & FLAG_T) {
        thumb_disasm(cpu, cpu->r[15], buf, buf_len);
    } else {
        arm_disasm(mem_read32(cpu, cpu->r[15]), cpu->r[15], buf, buf_len);
    }
}
