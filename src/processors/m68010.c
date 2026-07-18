#include "m68010.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Motorola 68010 CPU core (practical subset).
// Same programming model and instruction coverage as the 68000 core, plus the
// 68010 additions: MOVEC (VBR/SFC/DFC control registers), MOVES (treated as a
// normal move, since we have a flat memory model with no function codes) and
// RTD.
//
// Loop mode note: the real 68010 has a "loop mode" where a tight DBcc loop
// whose body is a single word instruction is executed from a 2-word prefetch
// cache without re-fetching from memory. This is purely a timing optimisation
// with no architectural effect, so this core does not model it; DBcc loops
// simply re-fetch each iteration.
//
// The real chip has a 24-bit (16MB) address bus; we emulate 1MB of RAM and
// mask all addresses down to 20 bits.
#define MEM_SIZE (1024 * 1024)
#define ADDR_MASK 0xFFFFFu

// Status register / CCR flag bits
#define FLAG_C 0x0001
#define FLAG_V 0x0002
#define FLAG_Z 0x0004
#define FLAG_N 0x0008
#define FLAG_X 0x0010

// Operand sizes
#define SZ_BYTE 0
#define SZ_WORD 1
#define SZ_LONG 2

typedef struct M68010CPU {
    uint32_t d[8];      // Data registers D0-D7
    uint32_t a[8];      // Address registers A0-A7 (A7 = SP)
    uint32_t pc;
    uint16_t sr;        // Status register (low byte = CCR: X N Z V C)
    uint32_t vbr;       // Vector base register (68010)
    uint32_t sfc;       // Source function code (68010, 3 bits)
    uint32_t dfc;       // Destination function code (68010, 3 bits)
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;         // Illegal instruction / error halt
    int stopped;        // STOP instruction executed
} M68010CPU;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* m68010_create(void) {
    M68010CPU *cpu = (M68010CPU*)calloc(1, sizeof(M68010CPU));
    return cpu;
}

void m68010_destroy(void *context) {
    free(context);
}

int m68010_init(void *context) {
    if (!context) return -1;
    M68010CPU *cpu = (M68010CPU*)context;

    memset(cpu->d, 0, sizeof(cpu->d));
    memset(cpu->a, 0, sizeof(cpu->a));
    cpu->a[7] = MEM_SIZE;   // SP at top of RAM (0x100000)
    cpu->pc = 0;
    cpu->sr = 0x2700;       // Supervisor mode, interrupts masked
    cpu->vbr = 0;
    cpu->sfc = 0;
    cpu->dfc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->stopped = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int m68010_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    M68010CPU *cpu = (M68010CPU*)context;

    address &= ADDR_MASK;
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// ---------------------------------------------------------------------------
// Memory helpers (68010 is big-endian)
// ---------------------------------------------------------------------------

static uint8_t read8(M68010CPU *cpu, uint32_t addr) {
    return cpu->memory[addr & ADDR_MASK];
}

static uint16_t read16(M68010CPU *cpu, uint32_t addr) {
    uint32_t a0 = addr & ADDR_MASK, a1 = (addr + 1) & ADDR_MASK;
    return (uint16_t)((cpu->memory[a0] << 8) | cpu->memory[a1]);
}

static uint32_t read32(M68010CPU *cpu, uint32_t addr) {
    return ((uint32_t)read16(cpu, addr) << 16) | read16(cpu, addr + 2);
}

static void write8(M68010CPU *cpu, uint32_t addr, uint8_t val) {
    cpu->memory[addr & ADDR_MASK] = val;
}

static void write16(M68010CPU *cpu, uint32_t addr, uint16_t val) {
    cpu->memory[addr & ADDR_MASK] = (uint8_t)(val >> 8);
    cpu->memory[(addr + 1) & ADDR_MASK] = (uint8_t)(val & 0xFF);
}

static void write32(M68010CPU *cpu, uint32_t addr, uint32_t val) {
    write16(cpu, addr, (uint16_t)(val >> 16));
    write16(cpu, addr + 2, (uint16_t)(val & 0xFFFF));
}

static uint32_t read_sz(M68010CPU *cpu, uint32_t addr, int size) {
    if (size == SZ_BYTE) return read8(cpu, addr);
    if (size == SZ_WORD) return read16(cpu, addr);
    return read32(cpu, addr);
}

static void write_sz(M68010CPU *cpu, uint32_t addr, uint32_t val, int size) {
    if (size == SZ_BYTE) write8(cpu, addr, (uint8_t)val);
    else if (size == SZ_WORD) write16(cpu, addr, (uint16_t)val);
    else write32(cpu, addr, val);
}

static uint16_t fetch16(M68010CPU *cpu) {
    uint16_t v = read16(cpu, cpu->pc);
    cpu->pc += 2;
    return v;
}

static uint32_t fetch32(M68010CPU *cpu) {
    uint32_t v = read32(cpu, cpu->pc);
    cpu->pc += 4;
    return v;
}

static void push32(M68010CPU *cpu, uint32_t val) {
    cpu->a[7] -= 4;
    write32(cpu, cpu->a[7], val);
}

static uint32_t pop32(M68010CPU *cpu) {
    uint32_t v = read32(cpu, cpu->a[7]);
    cpu->a[7] += 4;
    return v;
}

// ---------------------------------------------------------------------------
// Size / flag helpers
// ---------------------------------------------------------------------------

static uint32_t size_mask(int size) {
    if (size == SZ_BYTE) return 0xFFu;
    if (size == SZ_WORD) return 0xFFFFu;
    return 0xFFFFFFFFu;
}

static uint32_t size_msb(int size) {
    if (size == SZ_BYTE) return 0x80u;
    if (size == SZ_WORD) return 0x8000u;
    return 0x80000000u;
}

static int32_t sign_ext(uint32_t val, int size) {
    if (size == SZ_BYTE) return (int32_t)(int8_t)val;
    if (size == SZ_WORD) return (int32_t)(int16_t)val;
    return (int32_t)val;
}

static void set_nz(M68010CPU *cpu, uint32_t val, int size) {
    val &= size_mask(size);
    cpu->sr &= (uint16_t)~(FLAG_N | FLAG_Z);
    if (val == 0) cpu->sr |= FLAG_Z;
    if (val & size_msb(size)) cpu->sr |= FLAG_N;
}

static void set_logic_flags(M68010CPU *cpu, uint32_t val, int size) {
    set_nz(cpu, val, size);
    cpu->sr &= (uint16_t)~(FLAG_V | FLAG_C);
}

static uint32_t do_add(M68010CPU *cpu, uint32_t dst, uint32_t src, int size) {
    uint32_t mask = size_mask(size), msb = size_msb(size);
    src &= mask; dst &= mask;
    uint32_t r = (dst + src) & mask;
    cpu->sr &= (uint16_t)~(FLAG_C | FLAG_V | FLAG_X);
    if (((src & dst) | ((src | dst) & ~r)) & msb) cpu->sr |= FLAG_C | FLAG_X;
    if ((src ^ r) & (dst ^ r) & msb) cpu->sr |= FLAG_V;
    set_nz(cpu, r, size);
    return r;
}

static uint32_t do_sub(M68010CPU *cpu, uint32_t dst, uint32_t src, int size, int set_x) {
    uint32_t mask = size_mask(size), msb = size_msb(size);
    src &= mask; dst &= mask;
    uint32_t r = (dst - src) & mask;
    cpu->sr &= (uint16_t)~(FLAG_C | FLAG_V);
    if (set_x) cpu->sr &= (uint16_t)~FLAG_X;
    if (((src & ~dst) | ((src | ~dst) & r)) & msb) {
        cpu->sr |= FLAG_C;
        if (set_x) cpu->sr |= FLAG_X;
    }
    if ((src ^ dst) & (r ^ dst) & msb) cpu->sr |= FLAG_V;
    set_nz(cpu, r, size);
    return r;
}

// type: 0 = AS, 1 = LS, 3 = RO
static uint32_t do_shift(M68010CPU *cpu, uint32_t val, unsigned count, int type,
                         int left, int size) {
    uint32_t mask = size_mask(size), msb = size_msb(size);
    val &= mask;
    if (count == 0) {
        cpu->sr &= (uint16_t)~(FLAG_C | FLAG_V); // X unaffected
        return val;
    }
    unsigned carry = 0, vflag = 0;
    for (unsigned i = 0; i < count; ++i) {
        if (left) {
            carry = (val & msb) ? 1u : 0u;
            val = (val << 1) & mask;
            if (type == 3 && carry) val |= 1;                 // ROL
            if (type == 0 && (((val & msb) ? 1u : 0u) != carry)) vflag = 1; // ASL
        } else {
            uint32_t top = (type == 0) ? (val & msb) : 0;     // ASR keeps sign
            carry = val & 1;
            val = (val >> 1) | top;
            if (type == 3 && carry) val |= msb;               // ROR
        }
    }
    cpu->sr &= (uint16_t)~(FLAG_C | FLAG_V);
    if (carry) cpu->sr |= FLAG_C;
    if (vflag) cpu->sr |= FLAG_V;
    if (type != 3) { // shifts update X, rotates do not
        cpu->sr &= (uint16_t)~FLAG_X;
        if (carry) cpu->sr |= FLAG_X;
    }
    return val;
}

static int cond_true(const M68010CPU *cpu, int cc) {
    int c = (cpu->sr & FLAG_C) != 0;
    int v = (cpu->sr & FLAG_V) != 0;
    int z = (cpu->sr & FLAG_Z) != 0;
    int n = (cpu->sr & FLAG_N) != 0;
    switch (cc) {
        case 0:  return 1;                // T
        case 1:  return 0;                // F
        case 2:  return !c && !z;         // HI
        case 3:  return c || z;           // LS
        case 4:  return !c;               // CC
        case 5:  return c;                // CS
        case 6:  return !z;               // NE
        case 7:  return z;                // EQ
        case 8:  return !v;               // VC
        case 9:  return v;                // VS
        case 10: return !n;               // PL
        case 11: return n;                // MI
        case 12: return n == v;           // GE
        case 13: return n != v;           // LT
        case 14: return !z && (n == v);   // GT
        default: return z || (n != v);    // LE
    }
}

// ---------------------------------------------------------------------------
// Effective address handling
// ---------------------------------------------------------------------------

#define EA_DREG 0
#define EA_AREG 1
#define EA_MEM  2
#define EA_IMM  3

typedef struct EA {
    int kind;
    int reg;
    uint32_t addr;
    uint32_t imm;
} EA;

static uint32_t index_ext(M68010CPU *cpu, uint32_t base) {
    uint16_t ext = fetch16(cpu);
    int xreg = (ext >> 12) & 7;
    uint32_t xval = (ext & 0x8000) ? cpu->a[xreg] : cpu->d[xreg];
    if (!(ext & 0x0800)) xval = (uint32_t)(int32_t)(int16_t)xval; // .w index
    return base + (int32_t)(int8_t)(ext & 0xFF) + xval;
}

// Computes an effective address, fetching extension words and applying
// post-increment / pre-decrement side effects. Returns 0 on success.
static int ea_calc(M68010CPU *cpu, int mode, int reg, int size, EA *ea) {
    uint32_t step = (size == SZ_BYTE) ? 1 : (size == SZ_WORD) ? 2 : 4;
    if (size == SZ_BYTE && reg == 7) step = 2; // keep SP word-aligned

    ea->reg = reg;
    switch (mode) {
        case 0: ea->kind = EA_DREG; return 0;
        case 1: ea->kind = EA_AREG; return 0;
        case 2: ea->kind = EA_MEM; ea->addr = cpu->a[reg]; return 0;
        case 3:
            ea->kind = EA_MEM;
            ea->addr = cpu->a[reg];
            cpu->a[reg] += step;
            return 0;
        case 4:
            ea->kind = EA_MEM;
            cpu->a[reg] -= step;
            ea->addr = cpu->a[reg];
            return 0;
        case 5:
            ea->kind = EA_MEM;
            ea->addr = cpu->a[reg] + (int32_t)(int16_t)fetch16(cpu);
            return 0;
        case 6:
            ea->kind = EA_MEM;
            ea->addr = index_ext(cpu, cpu->a[reg]);
            return 0;
        case 7:
            switch (reg) {
                case 0:
                    ea->kind = EA_MEM;
                    ea->addr = (uint32_t)(int32_t)(int16_t)fetch16(cpu);
                    return 0;
                case 1:
                    ea->kind = EA_MEM;
                    ea->addr = fetch32(cpu);
                    return 0;
                case 2: {
                    uint32_t base = cpu->pc;
                    ea->kind = EA_MEM;
                    ea->addr = base + (int32_t)(int16_t)fetch16(cpu);
                    return 0;
                }
                case 3:
                    ea->kind = EA_MEM;
                    ea->addr = index_ext(cpu, cpu->pc);
                    return 0;
                case 4:
                    ea->kind = EA_IMM;
                    if (size == SZ_LONG) ea->imm = fetch32(cpu);
                    else if (size == SZ_WORD) ea->imm = fetch16(cpu);
                    else ea->imm = fetch16(cpu) & 0xFF;
                    return 0;
                default:
                    return -1;
            }
        default:
            return -1;
    }
}

static uint32_t ea_read(M68010CPU *cpu, const EA *ea, int size) {
    switch (ea->kind) {
        case EA_DREG: return cpu->d[ea->reg] & size_mask(size);
        case EA_AREG: return cpu->a[ea->reg] & size_mask(size);
        case EA_MEM:  return read_sz(cpu, ea->addr, size);
        default:      return ea->imm & size_mask(size);
    }
}

static void ea_write(M68010CPU *cpu, const EA *ea, uint32_t val, int size) {
    uint32_t mask = size_mask(size);
    switch (ea->kind) {
        case EA_DREG:
            cpu->d[ea->reg] = (cpu->d[ea->reg] & ~mask) | (val & mask);
            break;
        case EA_AREG:
            cpu->a[ea->reg] = (size == SZ_WORD)
                ? (uint32_t)(int32_t)(int16_t)val : val;
            break;
        case EA_MEM:
            write_sz(cpu, ea->addr, val, size);
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Execution
// ---------------------------------------------------------------------------

static int do_illegal(M68010CPU *cpu) {
    cpu->halted = 1;
    return 1;
}

// MOVEM helper
static int exec_movem(M68010CPU *cpu, uint16_t op) {
    int to_regs = (op >> 10) & 1;
    int size = (op & 0x40) ? SZ_LONG : SZ_WORD;
    uint32_t step = (size == SZ_LONG) ? 4 : 2;
    int mode = (op >> 3) & 7, reg = op & 7;
    uint16_t regmask = fetch16(cpu);

    if (!to_regs && mode == 4) { // registers to memory, pre-decrement
        uint32_t addr = cpu->a[reg];
        for (int i = 0; i < 16; ++i) {
            if (!(regmask & (1 << i))) continue;
            // bit 0 = A7 ... bit 7 = A0, bit 8 = D7 ... bit 15 = D0
            uint32_t val = (i < 8) ? cpu->a[7 - i] : cpu->d[15 - i];
            addr -= step;
            write_sz(cpu, addr, val, size);
        }
        cpu->a[reg] = addr;
        return 0;
    }

    uint32_t addr;
    int postinc = (to_regs && mode == 3);
    if (postinc) {
        addr = cpu->a[reg];
    } else {
        if (mode < 2 || mode == 3 || mode == 4) return do_illegal(cpu);
        EA ea;
        if (ea_calc(cpu, mode, reg, size, &ea) != 0 || ea.kind != EA_MEM)
            return do_illegal(cpu);
        addr = ea.addr;
    }
    // bit 0 = D0 ... bit 7 = D7, bit 8 = A0 ... bit 15 = A7
    for (int i = 0; i < 16; ++i) {
        if (!(regmask & (1 << i))) continue;
        if (to_regs) {
            uint32_t val = read_sz(cpu, addr, size);
            if (size == SZ_WORD) val = (uint32_t)(int32_t)(int16_t)val;
            if (i < 8) cpu->d[i] = val; else cpu->a[i - 8] = val;
        } else {
            uint32_t val = (i < 8) ? cpu->d[i] : cpu->a[i - 8];
            write_sz(cpu, addr, val, size);
        }
        addr += step;
    }
    if (postinc) cpu->a[reg] = addr;
    return 0;
}

// MOVEC Rc,Rn / MOVEC Rn,Rc (68010). Supported control registers:
// SFC (0x000), DFC (0x001), VBR (0x801).
static int exec_movec(M68010CPU *cpu, uint16_t op) {
    uint16_t ext = fetch16(cpu);
    int rn = (ext >> 12) & 7;
    int is_areg = (ext & 0x8000) != 0;
    int to_ctrl = op & 1; // 0x4E7A: Rc->Rn, 0x4E7B: Rn->Rc
    uint32_t *ctrl;
    uint32_t ctrl_mask;
    switch (ext & 0x0FFF) {
        case 0x000: ctrl = &cpu->sfc; ctrl_mask = 0x7u; break;
        case 0x001: ctrl = &cpu->dfc; ctrl_mask = 0x7u; break;
        case 0x801: ctrl = &cpu->vbr; ctrl_mask = 0xFFFFFFFFu; break;
        default:    return do_illegal(cpu);
    }
    uint32_t *gen = is_areg ? &cpu->a[rn] : &cpu->d[rn];
    if (to_ctrl) *ctrl = *gen & ctrl_mask;
    else *gen = *ctrl;
    return 0;
}

// MOVES <ea>,Rn / MOVES Rn,<ea> (68010). On real hardware this accesses the
// address space selected by SFC/DFC; we have a flat memory model, so it is
// treated as a normal move. CCR is unaffected (as on the real chip).
static int exec_moves(M68010CPU *cpu, uint16_t op) {
    int size = (op >> 6) & 3;
    if (size == 3) return do_illegal(cpu);
    uint16_t ext = fetch16(cpu);
    int rn = (ext >> 12) & 7;
    int is_areg = (ext & 0x8000) != 0;
    int mode = (op >> 3) & 7, reg = op & 7;
    EA ea;
    if (ea_calc(cpu, mode, reg, size, &ea) != 0 || ea.kind != EA_MEM)
        return do_illegal(cpu);
    if (ext & 0x0800) { // register to memory
        uint32_t val = is_areg ? cpu->a[rn] : cpu->d[rn];
        write_sz(cpu, ea.addr, val, size);
    } else {            // memory to register
        uint32_t val = read_sz(cpu, ea.addr, size);
        if (is_areg) {
            cpu->a[rn] = (uint32_t)sign_ext(val, size);
        } else {
            uint32_t mask = size_mask(size);
            cpu->d[rn] = (cpu->d[rn] & ~mask) | (val & mask);
        }
    }
    return 0;
}

int m68010_step(void *context) {
    if (!context) return -1;
    M68010CPU *cpu = (M68010CPU*)context;

    if (cpu->halted || cpu->stopped) return 1;
    if (cpu->pc & 1) return -3; // odd PC: address error

    uint16_t op = fetch16(cpu);
    cpu->ticks++;

    int mode = (op >> 3) & 7;
    int reg = op & 7;
    EA ea;

    switch (op >> 12) {
        case 0x0: { // Immediate group: ORI/ANDI/SUBI/ADDI/EORI/CMPI, MOVES
            if ((op & 0xFF00) == 0x0E00)              // MOVES (68010)
                return exec_moves(cpu, op);
            if (op & 0x0100) return do_illegal(cpu); // bit ops / MOVEP: unsupported
            int type = (op >> 9) & 7;
            int size = (op >> 6) & 3;
            if (size == 3) return do_illegal(cpu);
            if (type == 4 || type == 7) return do_illegal(cpu);
            uint32_t imm;
            if (size == SZ_LONG) imm = fetch32(cpu);
            else if (size == SZ_WORD) imm = fetch16(cpu);
            else imm = fetch16(cpu) & 0xFF;
            if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
            if (ea.kind == EA_AREG || ea.kind == EA_IMM) return do_illegal(cpu);
            uint32_t dst = ea_read(cpu, &ea, size);
            uint32_t r;
            switch (type) {
                case 0: r = dst | imm; set_logic_flags(cpu, r, size); break; // ORI
                case 1: r = dst & imm; set_logic_flags(cpu, r, size); break; // ANDI
                case 2: r = do_sub(cpu, dst, imm, size, 1); break;           // SUBI
                case 3: r = do_add(cpu, dst, imm, size); break;              // ADDI
                case 5: r = dst ^ imm; set_logic_flags(cpu, r, size); break; // EORI
                default: // CMPI
                    do_sub(cpu, dst, imm, size, 0);
                    return 0;
            }
            ea_write(cpu, &ea, r, size);
            return 0;
        }

        case 0x1: case 0x2: case 0x3: { // MOVE / MOVEA
            int size = (op >> 12) == 1 ? SZ_BYTE : ((op >> 12) == 2 ? SZ_LONG : SZ_WORD);
            if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
            if (ea.kind == EA_AREG && size == SZ_BYTE) return do_illegal(cpu);
            uint32_t val = ea_read(cpu, &ea, size);
            int dmode = (op >> 6) & 7, dreg = (op >> 9) & 7;
            if (dmode == 1) { // MOVEA
                if (size == SZ_BYTE) return do_illegal(cpu);
                cpu->a[dreg] = (size == SZ_WORD)
                    ? (uint32_t)(int32_t)(int16_t)val : val;
                return 0;
            }
            if (dmode == 7 && dreg > 1) return do_illegal(cpu);
            EA dst;
            if (ea_calc(cpu, dmode, dreg, size, &dst) != 0) return do_illegal(cpu);
            if (dst.kind == EA_IMM) return do_illegal(cpu);
            ea_write(cpu, &dst, val, size);
            set_logic_flags(cpu, val, size);
            return 0;
        }

        case 0x4: { // Miscellaneous
            if (op == 0x4E71) return 0;               // NOP
            if (op == 0x4E75) {                       // RTS
                cpu->pc = pop32(cpu);
                return 0;
            }
            if (op == 0x4E74) {                       // RTD #disp (68010)
                int32_t disp = (int32_t)(int16_t)fetch16(cpu);
                cpu->pc = pop32(cpu);
                cpu->a[7] += (uint32_t)disp;
                return 0;
            }
            if (op == 0x4E72) {                       // STOP #imm
                cpu->sr = fetch16(cpu);
                cpu->stopped = 1;
                return 1;
            }
            if ((op & 0xFFFE) == 0x4E7A)              // MOVEC (68010)
                return exec_movec(cpu, op);
            if ((op & 0xFFC0) == 0x4E80) {            // JSR <ea>
                if (ea_calc(cpu, mode, reg, SZ_LONG, &ea) != 0 || ea.kind != EA_MEM)
                    return do_illegal(cpu);
                push32(cpu, cpu->pc);
                cpu->pc = ea.addr & ADDR_MASK;
                return 0;
            }
            if ((op & 0xFFC0) == 0x4EC0) {            // JMP <ea>
                if (ea_calc(cpu, mode, reg, SZ_LONG, &ea) != 0 || ea.kind != EA_MEM)
                    return do_illegal(cpu);
                cpu->pc = ea.addr & ADDR_MASK;
                return 0;
            }
            if ((op & 0xFFF8) == 0x4840) {            // SWAP Dn
                cpu->d[reg] = (cpu->d[reg] >> 16) | (cpu->d[reg] << 16);
                set_logic_flags(cpu, cpu->d[reg], SZ_LONG);
                return 0;
            }
            if ((op & 0xFFC0) == 0x4840) {            // PEA <ea>
                if (ea_calc(cpu, mode, reg, SZ_LONG, &ea) != 0 || ea.kind != EA_MEM)
                    return do_illegal(cpu);
                push32(cpu, ea.addr);
                return 0;
            }
            if ((op & 0xFFB8) == 0x4880) {            // EXT.W / EXT.L Dn
                if (op & 0x40) {
                    cpu->d[reg] = (uint32_t)(int32_t)(int16_t)cpu->d[reg];
                    set_logic_flags(cpu, cpu->d[reg], SZ_LONG);
                } else {
                    uint32_t v = (uint32_t)(uint16_t)(int16_t)(int8_t)cpu->d[reg];
                    cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000u) | v;
                    set_logic_flags(cpu, v, SZ_WORD);
                }
                return 0;
            }
            if ((op & 0xFB80) == 0x4880)              // MOVEM
                return exec_movem(cpu, op);
            if ((op & 0xF1C0) == 0x41C0) {            // LEA <ea>,An
                if (ea_calc(cpu, mode, reg, SZ_LONG, &ea) != 0 || ea.kind != EA_MEM)
                    return do_illegal(cpu);
                cpu->a[(op >> 9) & 7] = ea.addr;
                return 0;
            }
            if ((op & 0xFF00) == 0x4200 || (op & 0xFF00) == 0x4400 ||
                (op & 0xFF00) == 0x4600 || (op & 0xFF00) == 0x4A00) {
                int size = (op >> 6) & 3;
                if (size == 3) return do_illegal(cpu);
                if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
                if (ea.kind == EA_AREG || ea.kind == EA_IMM) return do_illegal(cpu);
                uint32_t dst = ea_read(cpu, &ea, size);
                switch ((op >> 8) & 0xF) {
                    case 0x2: // CLR
                        ea_write(cpu, &ea, 0, size);
                        set_logic_flags(cpu, 0, size);
                        return 0;
                    case 0x4: { // NEG
                        uint32_t r = do_sub(cpu, 0, dst, size, 1);
                        ea_write(cpu, &ea, r, size);
                        return 0;
                    }
                    case 0x6: { // NOT
                        uint32_t r = ~dst & size_mask(size);
                        ea_write(cpu, &ea, r, size);
                        set_logic_flags(cpu, r, size);
                        return 0;
                    }
                    default:  // TST
                        set_logic_flags(cpu, dst, size);
                        return 0;
                }
            }
            return do_illegal(cpu);
        }

        case 0x5: { // ADDQ / SUBQ / Scc / DBcc
            int size = (op >> 6) & 3;
            if (size == 3) {
                int cc = (op >> 8) & 0xF;
                if (mode == 1) { // DBcc Dn,label (68010 loop mode not modelled)
                    uint32_t base = cpu->pc;
                    int32_t disp = (int32_t)(int16_t)fetch16(cpu);
                    if (!cond_true(cpu, cc)) {
                        uint16_t cnt = (uint16_t)(cpu->d[reg] - 1);
                        cpu->d[reg] = (cpu->d[reg] & 0xFFFF0000u) | cnt;
                        if (cnt != 0xFFFF) cpu->pc = base + disp;
                    }
                    return 0;
                }
                // Scc <ea>
                if (ea_calc(cpu, mode, reg, SZ_BYTE, &ea) != 0) return do_illegal(cpu);
                if (ea.kind == EA_AREG || ea.kind == EA_IMM) return do_illegal(cpu);
                ea_write(cpu, &ea, cond_true(cpu, cc) ? 0xFFu : 0x00u, SZ_BYTE);
                return 0;
            }
            uint32_t data = (op >> 9) & 7;
            if (data == 0) data = 8;
            int is_sub = (op >> 8) & 1;
            if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
            if (ea.kind == EA_IMM) return do_illegal(cpu);
            if (ea.kind == EA_AREG) { // no flags, full 32-bit
                if (size == SZ_BYTE) return do_illegal(cpu);
                if (is_sub) cpu->a[ea.reg] -= data;
                else cpu->a[ea.reg] += data;
                return 0;
            }
            uint32_t dst = ea_read(cpu, &ea, size);
            uint32_t r = is_sub ? do_sub(cpu, dst, data, size, 1)
                                : do_add(cpu, dst, data, size);
            ea_write(cpu, &ea, r, size);
            return 0;
        }

        case 0x6: { // BRA / BSR / Bcc
            int cc = (op >> 8) & 0xF;
            uint32_t base = cpu->pc;
            int32_t disp = (int32_t)(int8_t)(op & 0xFF);
            if (disp == 0) disp = (int32_t)(int16_t)fetch16(cpu);
            if (cc == 1) { // BSR
                push32(cpu, cpu->pc);
                cpu->pc = base + disp;
                return 0;
            }
            if (cond_true(cpu, cc)) cpu->pc = base + disp;
            return 0;
        }

        case 0x7: { // MOVEQ #imm,Dn
            if (op & 0x0100) return do_illegal(cpu);
            uint32_t val = (uint32_t)(int32_t)(int8_t)(op & 0xFF);
            cpu->d[(op >> 9) & 7] = val;
            set_logic_flags(cpu, val, SZ_LONG);
            return 0;
        }

        case 0x8: case 0xC: { // OR/DIVU/DIVS and AND/MULU/MULS
            int is_and = (op >> 12) == 0xC;
            int opmode = (op >> 6) & 7;
            int dn = (op >> 9) & 7;
            if (opmode == 3 || opmode == 7) { // DIVx / MULx (word source)
                if (ea_calc(cpu, mode, reg, SZ_WORD, &ea) != 0) return do_illegal(cpu);
                if (ea.kind == EA_AREG) return do_illegal(cpu);
                uint32_t src = ea_read(cpu, &ea, SZ_WORD);
                if (is_and) { // MULU / MULS
                    uint32_t r;
                    if (opmode == 7)
                        r = (uint32_t)((int32_t)(int16_t)src *
                                       (int32_t)(int16_t)cpu->d[dn]);
                    else
                        r = (src & 0xFFFF) * (cpu->d[dn] & 0xFFFF);
                    cpu->d[dn] = r;
                    set_logic_flags(cpu, r, SZ_LONG);
                    return 0;
                }
                // DIVU / DIVS
                if ((src & 0xFFFF) == 0) return do_illegal(cpu); // div by zero: halt
                if (opmode == 7) { // DIVS
                    int32_t num = (int32_t)cpu->d[dn];
                    int32_t den = (int32_t)(int16_t)src;
                    int32_t q = num / den, rmd = num % den;
                    if (q > 32767 || q < -32768) {
                        cpu->sr |= FLAG_V;
                        return 0;
                    }
                    cpu->d[dn] = ((uint32_t)(uint16_t)rmd << 16) | (uint16_t)q;
                    set_logic_flags(cpu, (uint32_t)(uint16_t)q, SZ_WORD);
                } else { // DIVU
                    uint32_t num = cpu->d[dn], den = src & 0xFFFF;
                    uint32_t q = num / den, rmd = num % den;
                    if (q > 0xFFFF) {
                        cpu->sr |= FLAG_V;
                        return 0;
                    }
                    cpu->d[dn] = (rmd << 16) | q;
                    set_logic_flags(cpu, q, SZ_WORD);
                }
                return 0;
            }
            int size = opmode & 3;
            if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
            if (ea.kind == EA_AREG) return do_illegal(cpu);
            if (opmode < 3) { // <ea> op Dn -> Dn
                uint32_t src = ea_read(cpu, &ea, size);
                uint32_t dst = cpu->d[dn] & size_mask(size);
                uint32_t r = is_and ? (dst & src) : (dst | src);
                cpu->d[dn] = (cpu->d[dn] & ~size_mask(size)) | r;
                set_logic_flags(cpu, r, size);
            } else { // Dn op <ea> -> <ea>
                if (ea.kind != EA_MEM) return do_illegal(cpu);
                uint32_t dst = ea_read(cpu, &ea, size);
                uint32_t src = cpu->d[dn];
                uint32_t r = (is_and ? (dst & src) : (dst | src)) & size_mask(size);
                ea_write(cpu, &ea, r, size);
                set_logic_flags(cpu, r, size);
            }
            return 0;
        }

        case 0x9: case 0xD: { // SUB/SUBA and ADD/ADDA
            int is_add = (op >> 12) == 0xD;
            int opmode = (op >> 6) & 7;
            int dn = (op >> 9) & 7;
            if (opmode == 3 || opmode == 7) { // SUBA / ADDA
                int size = (opmode == 3) ? SZ_WORD : SZ_LONG;
                if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
                uint32_t src = (uint32_t)sign_ext(ea_read(cpu, &ea, size), size);
                if (is_add) cpu->a[dn] += src;
                else cpu->a[dn] -= src;
                return 0;
            }
            int size = opmode & 3;
            if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
            if (opmode < 3) { // <ea> op Dn -> Dn
                if (ea.kind == EA_AREG && size == SZ_BYTE) return do_illegal(cpu);
                uint32_t src = ea_read(cpu, &ea, size);
                uint32_t dst = cpu->d[dn];
                uint32_t r = is_add ? do_add(cpu, dst, src, size)
                                    : do_sub(cpu, dst, src, size, 1);
                cpu->d[dn] = (cpu->d[dn] & ~size_mask(size)) | r;
            } else { // Dn op <ea> -> <ea>
                if (ea.kind != EA_MEM) return do_illegal(cpu);
                uint32_t dst = ea_read(cpu, &ea, size);
                uint32_t r = is_add ? do_add(cpu, dst, cpu->d[dn], size)
                                    : do_sub(cpu, dst, cpu->d[dn], size, 1);
                ea_write(cpu, &ea, r, size);
            }
            return 0;
        }

        case 0xB: { // CMP / CMPA / EOR
            int opmode = (op >> 6) & 7;
            int dn = (op >> 9) & 7;
            if (opmode == 3 || opmode == 7) { // CMPA
                int size = (opmode == 3) ? SZ_WORD : SZ_LONG;
                if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
                uint32_t src = (uint32_t)sign_ext(ea_read(cpu, &ea, size), size);
                do_sub(cpu, cpu->a[dn], src, SZ_LONG, 0);
                return 0;
            }
            int size = opmode & 3;
            if (opmode < 3) { // CMP <ea>,Dn
                if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
                if (ea.kind == EA_AREG && size == SZ_BYTE) return do_illegal(cpu);
                do_sub(cpu, cpu->d[dn], ea_read(cpu, &ea, size), size, 0);
            } else { // EOR Dn,<ea>
                if (mode == 1) return do_illegal(cpu); // CMPM: unsupported
                if (ea_calc(cpu, mode, reg, size, &ea) != 0) return do_illegal(cpu);
                if (ea.kind == EA_IMM) return do_illegal(cpu);
                uint32_t r = (ea_read(cpu, &ea, size) ^ cpu->d[dn]) & size_mask(size);
                ea_write(cpu, &ea, r, size);
                set_logic_flags(cpu, r, size);
            }
            return 0;
        }

        case 0xE: { // Shifts and rotates: ASd/LSd/ROd
            int left = (op >> 8) & 1;
            int size = (op >> 6) & 3;
            if (size == 3) { // memory form: shift by one, word only
                int type = (op >> 9) & 3;
                if (type == 2) return do_illegal(cpu); // ROXd unsupported
                if (ea_calc(cpu, mode, reg, SZ_WORD, &ea) != 0 || ea.kind != EA_MEM)
                    return do_illegal(cpu);
                uint32_t r = do_shift(cpu, ea_read(cpu, &ea, SZ_WORD), 1, type,
                                      left, SZ_WORD);
                ea_write(cpu, &ea, r, SZ_WORD);
                set_nz(cpu, r, SZ_WORD);
                return 0;
            }
            int type = (op >> 3) & 3;
            if (type == 2) return do_illegal(cpu); // ROXd unsupported
            unsigned count;
            unsigned cfield = (op >> 9) & 7;
            if (op & 0x20) count = cpu->d[cfield] & 63;
            else count = cfield ? cfield : 8;
            uint32_t r = do_shift(cpu, cpu->d[reg], count, type, left, size);
            cpu->d[reg] = (cpu->d[reg] & ~size_mask(size)) | r;
            set_nz(cpu, r, size);
            return 0;
        }

        default: // 0xA (line-A), 0xF (line-F) and anything unhandled
            return do_illegal(cpu);
    }
}

// ---------------------------------------------------------------------------
// State display
// ---------------------------------------------------------------------------

void m68010_print_state(void *context) {
    if (!context) return;
    M68010CPU *cpu = (M68010CPU*)context;

    printf("M68010 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%08X  SR: 0x%04X [%c%c%c%c%c]  Halted: %s  Stopped: %s\n",
           cpu->pc, cpu->sr,
           (cpu->sr & FLAG_X) ? 'X' : '-',
           (cpu->sr & FLAG_N) ? 'N' : '-',
           (cpu->sr & FLAG_Z) ? 'Z' : '-',
           (cpu->sr & FLAG_V) ? 'V' : '-',
           (cpu->sr & FLAG_C) ? 'C' : '-',
           cpu->halted ? "Yes" : "No",
           cpu->stopped ? "Yes" : "No");
    printf("  VBR: 0x%08X  SFC: %u  DFC: %u\n", cpu->vbr, cpu->sfc, cpu->dfc);
    printf("  Data:\n");
    for (int i = 0; i < 8; ++i) {
        printf("    D%d: 0x%08X%s", i, cpu->d[i], (i % 4 == 3) ? "\n" : "  ");
    }
    printf("  Address:\n");
    for (int i = 0; i < 8; ++i) {
        printf("    A%d: 0x%08X%s", i, cpu->a[i], (i % 4 == 3) ? "\n" : "  ");
    }
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

static uint16_t dis_fetch16(M68010CPU *cpu, uint32_t *p) {
    uint16_t v = read16(cpu, *p);
    *p += 2;
    return v;
}

static const char *dis_size_suffix(int size) {
    return (size == SZ_BYTE) ? ".b" : (size == SZ_WORD) ? ".w" : ".l";
}

static const char *dis_ctrl_name(uint16_t code) {
    switch (code & 0x0FFF) {
        case 0x000: return "SFC";
        case 0x001: return "DFC";
        case 0x801: return "VBR";
        default:    return "Rc?";
    }
}

static void dis_ea(M68010CPU *cpu, uint32_t *p, int mode, int reg, int size,
                   char *out, size_t n) {
    switch (mode) {
        case 0: snprintf(out, n, "D%d", reg); return;
        case 1: snprintf(out, n, "A%d", reg); return;
        case 2: snprintf(out, n, "(A%d)", reg); return;
        case 3: snprintf(out, n, "(A%d)+", reg); return;
        case 4: snprintf(out, n, "-(A%d)", reg); return;
        case 5: snprintf(out, n, "%d(A%d)", (int16_t)dis_fetch16(cpu, p), reg); return;
        case 6: {
            uint16_t ext = dis_fetch16(cpu, p);
            snprintf(out, n, "%d(A%d,%c%d.%c)", (int8_t)(ext & 0xFF), reg,
                     (ext & 0x8000) ? 'A' : 'D', (ext >> 12) & 7,
                     (ext & 0x0800) ? 'l' : 'w');
            return;
        }
        case 7:
            switch (reg) {
                case 0: snprintf(out, n, "$%X.w", dis_fetch16(cpu, p)); return;
                case 1: {
                    uint32_t v = ((uint32_t)dis_fetch16(cpu, p) << 16);
                    v |= dis_fetch16(cpu, p);
                    snprintf(out, n, "$%X.l", v);
                    return;
                }
                case 2: snprintf(out, n, "%d(PC)", (int16_t)dis_fetch16(cpu, p)); return;
                case 3: {
                    uint16_t ext = dis_fetch16(cpu, p);
                    snprintf(out, n, "%d(PC,%c%d.%c)", (int8_t)(ext & 0xFF),
                             (ext & 0x8000) ? 'A' : 'D', (ext >> 12) & 7,
                             (ext & 0x0800) ? 'l' : 'w');
                    return;
                }
                case 4:
                    if (size == SZ_LONG) {
                        uint32_t v = ((uint32_t)dis_fetch16(cpu, p) << 16);
                        v |= dis_fetch16(cpu, p);
                        snprintf(out, n, "#$%X", v);
                    } else {
                        snprintf(out, n, "#$%X", dis_fetch16(cpu, p) &
                                 (size == SZ_BYTE ? 0xFFu : 0xFFFFu));
                    }
                    return;
                default: snprintf(out, n, "<bad ea>"); return;
            }
        default: snprintf(out, n, "<bad ea>"); return;
    }
}

static const char *cc_names[16] = {
    "t", "f", "hi", "ls", "cc", "cs", "ne", "eq",
    "vc", "vs", "pl", "mi", "ge", "lt", "gt", "le"
};

void m68010_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    M68010CPU *cpu = (M68010CPU*)context;

    uint32_t p = cpu->pc;
    uint16_t op = dis_fetch16(cpu, &p);
    int mode = (op >> 3) & 7;
    int reg = op & 7;
    char src[48], dst[48];

    switch (op >> 12) {
        case 0x0: {
            if ((op & 0xFF00) == 0x0E00) { // MOVES
                int size = (op >> 6) & 3;
                if (size == 3) break;
                uint16_t ext = dis_fetch16(cpu, &p);
                char rn[4];
                snprintf(rn, sizeof(rn), "%c%d",
                         (ext & 0x8000) ? 'A' : 'D', (ext >> 12) & 7);
                dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
                if (ext & 0x0800)
                    snprintf(buf, buf_len, "moves%s %s,%s",
                             dis_size_suffix(size), rn, src);
                else
                    snprintf(buf, buf_len, "moves%s %s,%s",
                             dis_size_suffix(size), src, rn);
                return;
            }
            if (op & 0x0100) break;
            static const char *imm_ops[8] =
                {"ori", "andi", "subi", "addi", "?", "eori", "cmpi", "?"};
            int type = (op >> 9) & 7;
            int size = (op >> 6) & 3;
            if (size == 3 || type == 4 || type == 7) break;
            uint32_t imm;
            if (size == SZ_LONG) {
                imm = ((uint32_t)dis_fetch16(cpu, &p) << 16);
                imm |= dis_fetch16(cpu, &p);
            } else {
                imm = dis_fetch16(cpu, &p) & (size == SZ_BYTE ? 0xFFu : 0xFFFFu);
            }
            dis_ea(cpu, &p, mode, reg, size, dst, sizeof(dst));
            snprintf(buf, buf_len, "%s%s #$%X,%s",
                     imm_ops[type], dis_size_suffix(size), imm, dst);
            return;
        }
        case 0x1: case 0x2: case 0x3: {
            int size = (op >> 12) == 1 ? SZ_BYTE : ((op >> 12) == 2 ? SZ_LONG : SZ_WORD);
            int dmode = (op >> 6) & 7, dreg = (op >> 9) & 7;
            dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
            if (dmode == 1) {
                snprintf(buf, buf_len, "movea%s %s,A%d",
                         dis_size_suffix(size), src, dreg);
                return;
            }
            dis_ea(cpu, &p, dmode, dreg, size, dst, sizeof(dst));
            snprintf(buf, buf_len, "move%s %s,%s", dis_size_suffix(size), src, dst);
            return;
        }
        case 0x4: {
            if (op == 0x4E71) { snprintf(buf, buf_len, "nop"); return; }
            if (op == 0x4E75) { snprintf(buf, buf_len, "rts"); return; }
            if (op == 0x4E74) {
                snprintf(buf, buf_len, "rtd #%d", (int16_t)dis_fetch16(cpu, &p));
                return;
            }
            if (op == 0x4E72) {
                snprintf(buf, buf_len, "stop #$%X", dis_fetch16(cpu, &p));
                return;
            }
            if ((op & 0xFFFE) == 0x4E7A) { // MOVEC
                uint16_t ext = dis_fetch16(cpu, &p);
                char rn[4];
                snprintf(rn, sizeof(rn), "%c%d",
                         (ext & 0x8000) ? 'A' : 'D', (ext >> 12) & 7);
                if (op & 1)
                    snprintf(buf, buf_len, "movec %s,%s", rn, dis_ctrl_name(ext));
                else
                    snprintf(buf, buf_len, "movec %s,%s", dis_ctrl_name(ext), rn);
                return;
            }
            if ((op & 0xFFC0) == 0x4E80 || (op & 0xFFC0) == 0x4EC0) {
                dis_ea(cpu, &p, mode, reg, SZ_LONG, src, sizeof(src));
                snprintf(buf, buf_len, "%s %s",
                         ((op & 0xFFC0) == 0x4E80) ? "jsr" : "jmp", src);
                return;
            }
            if ((op & 0xFFF8) == 0x4840) {
                snprintf(buf, buf_len, "swap D%d", reg);
                return;
            }
            if ((op & 0xFFC0) == 0x4840) {
                dis_ea(cpu, &p, mode, reg, SZ_LONG, src, sizeof(src));
                snprintf(buf, buf_len, "pea %s", src);
                return;
            }
            if ((op & 0xFFB8) == 0x4880) {
                snprintf(buf, buf_len, "ext%s D%d", (op & 0x40) ? ".l" : ".w", reg);
                return;
            }
            if ((op & 0xFB80) == 0x4880) {
                uint16_t regmask = dis_fetch16(cpu, &p);
                const char *sz = (op & 0x40) ? ".l" : ".w";
                dis_ea(cpu, &p, mode, reg, (op & 0x40) ? SZ_LONG : SZ_WORD,
                       src, sizeof(src));
                if ((op >> 10) & 1)
                    snprintf(buf, buf_len, "movem%s %s,#$%04X", sz, src, regmask);
                else
                    snprintf(buf, buf_len, "movem%s #$%04X,%s", sz, regmask, src);
                return;
            }
            if ((op & 0xF1C0) == 0x41C0) {
                dis_ea(cpu, &p, mode, reg, SZ_LONG, src, sizeof(src));
                snprintf(buf, buf_len, "lea %s,A%d", src, (op >> 9) & 7);
                return;
            }
            if ((op & 0xFF00) == 0x4200 || (op & 0xFF00) == 0x4400 ||
                (op & 0xFF00) == 0x4600 || (op & 0xFF00) == 0x4A00) {
                int size = (op >> 6) & 3;
                if (size == 3) break;
                const char *name = "tst";
                if ((op & 0xFF00) == 0x4200) name = "clr";
                else if ((op & 0xFF00) == 0x4400) name = "neg";
                else if ((op & 0xFF00) == 0x4600) name = "not";
                dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
                snprintf(buf, buf_len, "%s%s %s", name, dis_size_suffix(size), src);
                return;
            }
            break;
        }
        case 0x5: {
            int size = (op >> 6) & 3;
            int cc = (op >> 8) & 0xF;
            if (size == 3) {
                if (mode == 1) {
                    int16_t disp = (int16_t)dis_fetch16(cpu, &p);
                    snprintf(buf, buf_len, "db%s D%d,$%X",
                             cc_names[cc], reg, cpu->pc + 2 + disp);
                    return;
                }
                dis_ea(cpu, &p, mode, reg, SZ_BYTE, src, sizeof(src));
                snprintf(buf, buf_len, "s%s %s", cc_names[cc], src);
                return;
            }
            uint32_t data = (op >> 9) & 7;
            if (data == 0) data = 8;
            dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
            snprintf(buf, buf_len, "%s%s #%u,%s",
                     ((op >> 8) & 1) ? "subq" : "addq",
                     dis_size_suffix(size), data, src);
            return;
        }
        case 0x6: {
            int cc = (op >> 8) & 0xF;
            uint32_t base = cpu->pc + 2;
            int32_t disp = (int32_t)(int8_t)(op & 0xFF);
            if (disp == 0) disp = (int16_t)dis_fetch16(cpu, &p);
            const char *name = (cc == 0) ? "bra" : (cc == 1) ? "bsr" : cc_names[cc];
            if (cc >= 2) snprintf(buf, buf_len, "b%s $%X", name, base + disp);
            else snprintf(buf, buf_len, "%s $%X", name, base + disp);
            return;
        }
        case 0x7: {
            if (op & 0x0100) break;
            snprintf(buf, buf_len, "moveq #%d,D%d",
                     (int)(int8_t)(op & 0xFF), (op >> 9) & 7);
            return;
        }
        case 0x8: case 0xC: {
            int is_and = (op >> 12) == 0xC;
            int opmode = (op >> 6) & 7;
            int dn = (op >> 9) & 7;
            if (opmode == 3 || opmode == 7) {
                const char *name = is_and ? ((opmode == 7) ? "muls" : "mulu")
                                          : ((opmode == 7) ? "divs" : "divu");
                dis_ea(cpu, &p, mode, reg, SZ_WORD, src, sizeof(src));
                snprintf(buf, buf_len, "%s %s,D%d", name, src, dn);
                return;
            }
            int size = opmode & 3;
            dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
            if (opmode < 3)
                snprintf(buf, buf_len, "%s%s %s,D%d", is_and ? "and" : "or",
                         dis_size_suffix(size), src, dn);
            else
                snprintf(buf, buf_len, "%s%s D%d,%s", is_and ? "and" : "or",
                         dis_size_suffix(size), dn, src);
            return;
        }
        case 0x9: case 0xD: case 0xB: {
            int grp = op >> 12;
            int opmode = (op >> 6) & 7;
            int dn = (op >> 9) & 7;
            const char *name = (grp == 0x9) ? "sub" : (grp == 0xD) ? "add" : "cmp";
            if (opmode == 3 || opmode == 7) {
                int size = (opmode == 3) ? SZ_WORD : SZ_LONG;
                dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
                snprintf(buf, buf_len, "%sa%s %s,A%d",
                         name, dis_size_suffix(size), src, dn);
                return;
            }
            int size = opmode & 3;
            dis_ea(cpu, &p, mode, reg, size, src, sizeof(src));
            if (opmode < 3)
                snprintf(buf, buf_len, "%s%s %s,D%d",
                         name, dis_size_suffix(size), src, dn);
            else if (grp == 0xB)
                snprintf(buf, buf_len, "eor%s D%d,%s",
                         dis_size_suffix(size), dn, src);
            else
                snprintf(buf, buf_len, "%s%s D%d,%s",
                         name, dis_size_suffix(size), dn, src);
            return;
        }
        case 0xE: {
            static const char *shift_ops[4] = {"as", "ls", "rox", "ro"};
            int left = (op >> 8) & 1;
            int size = (op >> 6) & 3;
            if (size == 3) {
                int type = (op >> 9) & 3;
                dis_ea(cpu, &p, mode, reg, SZ_WORD, src, sizeof(src));
                snprintf(buf, buf_len, "%s%c %s",
                         shift_ops[type], left ? 'l' : 'r', src);
                return;
            }
            int type = (op >> 3) & 3;
            unsigned cfield = (op >> 9) & 7;
            if (op & 0x20)
                snprintf(buf, buf_len, "%s%c%s D%u,D%d", shift_ops[type],
                         left ? 'l' : 'r', dis_size_suffix(size), cfield, reg);
            else
                snprintf(buf, buf_len, "%s%c%s #%u,D%d", shift_ops[type],
                         left ? 'l' : 'r', dis_size_suffix(size),
                         cfield ? cfield : 8u, reg);
            return;
        }
        default:
            break;
    }
    snprintf(buf, buf_len, "dc.w $%04X", op);
}
