#include "tms9900.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536       // 64 KB (16-bit byte address space)
#define CRU_BITS 4096        // CRU single-bit I/O space
#define RESET_WP 0x8300      // TI-99/4A scratchpad convention

// Status register bits
#define ST_LGT 0x8000        // L>  logical greater than
#define ST_AGT 0x4000        // A>  arithmetic greater than
#define ST_EQ  0x2000        // =   equal
#define ST_C   0x1000        // C   carry
#define ST_OV  0x0800        // OV  overflow
#define ST_OP  0x0400        // OP  odd parity
#define ST_IMASK 0x000F      // interrupt mask

typedef struct TMS9900CPU {
    uint16_t pc;
    uint16_t wp;             // workspace pointer; R0-R15 live in memory at WP
    uint16_t st;             // status register

    uint8_t memory[MEM_SIZE];
    uint8_t cru[CRU_BITS];   // one byte per CRU bit
    uint32_t ticks;
    int halted;
} TMS9900CPU;

// --- Memory access (big-endian 16-bit words, LSB of address ignored) ---

static inline uint16_t mem_rw(TMS9900CPU *cpu, uint16_t addr) {
    addr &= 0xFFFE;
    return ((uint16_t)cpu->memory[addr] << 8) | cpu->memory[addr + 1];
}

static inline void mem_ww(TMS9900CPU *cpu, uint16_t addr, uint16_t val) {
    addr &= 0xFFFE;
    cpu->memory[addr] = (uint8_t)(val >> 8);
    cpu->memory[addr + 1] = (uint8_t)(val & 0xFF);
}

static inline uint8_t mem_rb(TMS9900CPU *cpu, uint16_t addr) {
    return cpu->memory[addr];
}

static inline void mem_wb(TMS9900CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->memory[addr] = val;
}

// --- Workspace registers (memory-resident at WP) ---

static inline uint16_t reg_addr(TMS9900CPU *cpu, int r) {
    return (uint16_t)(cpu->wp + 2 * r);
}

static inline uint16_t get_reg(TMS9900CPU *cpu, int r) {
    return mem_rw(cpu, reg_addr(cpu, r));
}

static inline void set_reg(TMS9900CPU *cpu, int r, uint16_t val) {
    mem_ww(cpu, reg_addr(cpu, r), val);
}

static inline uint16_t fetch_word(TMS9900CPU *cpu) {
    uint16_t v = mem_rw(cpu, cpu->pc);
    cpu->pc = (cpu->pc + 2) & 0xFFFF;
    return v;
}

// --- Status helpers ---

static inline void st_set(TMS9900CPU *cpu, uint16_t mask, int cond) {
    if (cond) cpu->st |= mask;
    else cpu->st &= (uint16_t)~mask;
}

static uint8_t odd_parity(uint8_t val) {
    uint8_t count = 0;
    for (int i = 0; i < 8; ++i) {
        if ((val >> i) & 1) count++;
    }
    return (uint8_t)(count & 1); // 1 when number of set bits is odd
}

// Set L>, A>, EQ by comparing a word result against zero.
static void set_lae16(TMS9900CPU *cpu, uint16_t res) {
    st_set(cpu, ST_LGT, res != 0);
    st_set(cpu, ST_AGT, (int16_t)res > 0);
    st_set(cpu, ST_EQ, res == 0);
}

// Byte results also set OP (odd parity).
static void set_lae8(TMS9900CPU *cpu, uint8_t res) {
    st_set(cpu, ST_LGT, res != 0);
    st_set(cpu, ST_AGT, (int8_t)res > 0);
    st_set(cpu, ST_EQ, res == 0);
    st_set(cpu, ST_OP, odd_parity(res));
}

// Compare source against destination (C, CB, CI flag semantics).
static void compare16(TMS9900CPU *cpu, uint16_t src, uint16_t dst) {
    st_set(cpu, ST_LGT, src > dst);
    st_set(cpu, ST_AGT, (int16_t)src > (int16_t)dst);
    st_set(cpu, ST_EQ, src == dst);
}

static void compare8(TMS9900CPU *cpu, uint8_t src, uint8_t dst) {
    st_set(cpu, ST_LGT, src > dst);
    st_set(cpu, ST_AGT, (int8_t)src > (int8_t)dst);
    st_set(cpu, ST_EQ, src == dst);
    st_set(cpu, ST_OP, odd_parity(src));
}

// --- ALU (carry and overflow) ---

static uint16_t alu_add16(TMS9900CPU *cpu, uint16_t d, uint16_t s) {
    uint32_t sum = (uint32_t)d + s;
    uint16_t res = (uint16_t)sum;
    st_set(cpu, ST_C, sum > 0xFFFF);
    st_set(cpu, ST_OV, (~(d ^ s) & (d ^ res) & 0x8000) != 0);
    return res;
}

static uint16_t alu_sub16(TMS9900CPU *cpu, uint16_t d, uint16_t s) {
    uint32_t sum = (uint32_t)d + (uint16_t)~s + 1; // carry set means no borrow
    uint16_t res = (uint16_t)sum;
    st_set(cpu, ST_C, sum > 0xFFFF);
    st_set(cpu, ST_OV, ((d ^ s) & (d ^ res) & 0x8000) != 0);
    return res;
}

static uint8_t alu_add8(TMS9900CPU *cpu, uint8_t d, uint8_t s) {
    uint16_t sum = (uint16_t)d + s;
    uint8_t res = (uint8_t)sum;
    st_set(cpu, ST_C, sum > 0xFF);
    st_set(cpu, ST_OV, (~(d ^ s) & (d ^ res) & 0x80) != 0);
    return res;
}

static uint8_t alu_sub8(TMS9900CPU *cpu, uint8_t d, uint8_t s) {
    uint16_t sum = (uint16_t)d + (uint8_t)~s + 1;
    uint8_t res = (uint8_t)sum;
    st_set(cpu, ST_C, sum > 0xFF);
    st_set(cpu, ST_OV, ((d ^ s) & (d ^ res) & 0x80) != 0);
    return res;
}

// --- Addressing modes ---
// ts: 0 = workspace register, 1 = indirect, 2 = symbolic/indexed, 3 = auto-increment
static uint16_t operand_addr(TMS9900CPU *cpu, int ts, int r, int is_byte) {
    uint16_t addr;
    switch (ts) {
        case 0: // Rn
            return reg_addr(cpu, r);
        case 1: // *Rn
            return get_reg(cpu, r);
        case 2: { // @addr or @addr(Rn)
            uint16_t base = fetch_word(cpu);
            if (r != 0) base = (uint16_t)(base + get_reg(cpu, r));
            return base;
        }
        case 3: // *Rn+
            addr = get_reg(cpu, r);
            set_reg(cpu, r, (uint16_t)(addr + (is_byte ? 1 : 2)));
            return addr;
    }
    return 0;
}

// --- CRU ---

static void cru_write(TMS9900CPU *cpu, uint16_t bit, int val) {
    cpu->cru[bit & (CRU_BITS - 1)] = (uint8_t)(val ? 1 : 0);
}

static int cru_read(TMS9900CPU *cpu, uint16_t bit) {
    return cpu->cru[bit & (CRU_BITS - 1)];
}

static inline uint16_t cru_base(TMS9900CPU *cpu) {
    return (uint16_t)(get_reg(cpu, 12) >> 1);
}

// --- Jump condition evaluation ---
// code 0x10..0x1C: JMP JLT JLE JEQ JHE JGT JNE JNC JOC JNO JL JH JOP
static int jump_taken(TMS9900CPU *cpu, int code) {
    uint16_t st = cpu->st;
    int lgt = (st & ST_LGT) != 0;
    int agt = (st & ST_AGT) != 0;
    int eq = (st & ST_EQ) != 0;
    int c = (st & ST_C) != 0;
    int ov = (st & ST_OV) != 0;
    int op = (st & ST_OP) != 0;
    switch (code) {
        case 0x10: return 1;            // JMP
        case 0x11: return !agt && !eq;  // JLT
        case 0x12: return !lgt || eq;   // JLE
        case 0x13: return eq;           // JEQ
        case 0x14: return lgt || eq;    // JHE
        case 0x15: return agt;          // JGT
        case 0x16: return !eq;          // JNE
        case 0x17: return !c;           // JNC
        case 0x18: return c;            // JOC
        case 0x19: return !ov;          // JNO
        case 0x1A: return !lgt && !eq;  // JL
        case 0x1B: return lgt && !eq;   // JH
        case 0x1C: return op;           // JOP
    }
    return 0;
}

// --- Instruction execution ---
// Executes one already-fetched opcode; extra operand words come from the PC
// stream. Split out from step so X can substitute an instruction word.
static int exec_op(TMS9900CPU *cpu, uint16_t op) {
    // Format I: dual operand, general addressing (0x4000-0xFFFF)
    if (op >= 0x4000) {
        int op4 = op >> 12;
        int is_byte = op4 & 1;
        int ts = (op >> 4) & 3, s = op & 0xF;
        int td = (op >> 10) & 3, d = (op >> 6) & 0xF;
        uint16_t sa = operand_addr(cpu, ts, s, is_byte);
        if (is_byte) {
            uint8_t sv = mem_rb(cpu, sa);
            uint16_t da = operand_addr(cpu, td, d, 1);
            uint8_t dv = mem_rb(cpu, da);
            uint8_t res = 0;
            switch (op4) {
                case 0x5: res = dv & (uint8_t)~sv; break;        // SZCB
                case 0x7: res = alu_sub8(cpu, dv, sv); break;    // SB
                case 0x9: compare8(cpu, sv, dv); return 0;       // CB
                case 0xB: res = alu_add8(cpu, dv, sv); break;    // AB
                case 0xD: res = sv; break;                       // MOVB
                case 0xF: res = dv | sv; break;                  // SOCB
            }
            mem_wb(cpu, da, res);
            set_lae8(cpu, res);
        } else {
            uint16_t sv = mem_rw(cpu, sa);
            uint16_t da = operand_addr(cpu, td, d, 0);
            uint16_t dv = mem_rw(cpu, da);
            uint16_t res = 0;
            switch (op4) {
                case 0x4: res = dv & (uint16_t)~sv; break;       // SZC
                case 0x6: res = alu_sub16(cpu, dv, sv); break;   // S
                case 0x8: compare16(cpu, sv, dv); return 0;      // C
                case 0xA: res = alu_add16(cpu, dv, sv); break;   // A
                case 0xC: res = sv; break;                       // MOV
                case 0xE: res = dv | sv; break;                  // SOC
            }
            mem_ww(cpu, da, res);
            set_lae16(cpu, res);
        }
        return 0;
    }

    // Formats III/IV/IX: 0x2000-0x3FFF (COC CZC XOR XOP LDCR STCR MPY DIV)
    if (op >= 0x2000) {
        int code = op >> 10;
        int d = (op >> 6) & 0xF;
        int ts = (op >> 4) & 3, s = op & 0xF;
        switch (code) {
            case 0x8: { // COC: EQ when all source 1-bits are set in Rd
                uint16_t sv = mem_rw(cpu, operand_addr(cpu, ts, s, 0));
                st_set(cpu, ST_EQ, (sv & (uint16_t)~get_reg(cpu, d)) == 0);
                break;
            }
            case 0x9: { // CZC: EQ when all source 1-bits are clear in Rd
                uint16_t sv = mem_rw(cpu, operand_addr(cpu, ts, s, 0));
                st_set(cpu, ST_EQ, (sv & get_reg(cpu, d)) == 0);
                break;
            }
            case 0xA: { // XOR
                uint16_t sv = mem_rw(cpu, operand_addr(cpu, ts, s, 0));
                uint16_t res = get_reg(cpu, d) ^ sv;
                set_reg(cpu, d, res);
                set_lae16(cpu, res);
                break;
            }
            case 0xB: // XOP (no extended operations attached)
                (void)operand_addr(cpu, ts, s, 0);
                break;
            case 0xC: { // LDCR: transfer count bits from source to CRU
                int count = d ? d : 16;
                int byte_op = count <= 8;
                uint16_t sa = operand_addr(cpu, ts, s, byte_op);
                uint16_t val = byte_op ? mem_rb(cpu, sa) : mem_rw(cpu, sa);
                uint16_t base = cru_base(cpu);
                for (int i = 0; i < count; ++i)
                    cru_write(cpu, (uint16_t)(base + i), (val >> i) & 1);
                if (byte_op) set_lae8(cpu, (uint8_t)val);
                else set_lae16(cpu, val);
                break;
            }
            case 0xD: { // STCR: transfer count bits from CRU to destination
                int count = d ? d : 16;
                int byte_op = count <= 8;
                uint16_t da = operand_addr(cpu, ts, s, byte_op);
                uint16_t base = cru_base(cpu);
                uint16_t val = 0;
                for (int i = 0; i < count; ++i)
                    if (cru_read(cpu, (uint16_t)(base + i))) val |= (uint16_t)(1 << i);
                if (byte_op) {
                    mem_wb(cpu, da, (uint8_t)val);
                    set_lae8(cpu, (uint8_t)val);
                } else {
                    mem_ww(cpu, da, val);
                    set_lae16(cpu, val);
                }
                break;
            }
            case 0xE: { // MPY: unsigned Rd * source -> Rd:Rd+1 (32-bit)
                uint16_t sv = mem_rw(cpu, operand_addr(cpu, ts, s, 0));
                uint32_t prod = (uint32_t)get_reg(cpu, d) * sv;
                set_reg(cpu, d, (uint16_t)(prod >> 16));
                set_reg(cpu, (d + 1) & 0xF, (uint16_t)(prod & 0xFFFF));
                break;
            }
            case 0xF: { // DIV: (Rd:Rd+1) / source, unsigned
                uint16_t sv = mem_rw(cpu, operand_addr(cpu, ts, s, 0));
                uint16_t hi = get_reg(cpu, d);
                if (sv <= hi) { // includes divide by zero
                    st_set(cpu, ST_OV, 1);
                } else {
                    uint32_t dividend = ((uint32_t)hi << 16) | get_reg(cpu, (d + 1) & 0xF);
                    set_reg(cpu, d, (uint16_t)(dividend / sv));
                    set_reg(cpu, (d + 1) & 0xF, (uint16_t)(dividend % sv));
                    st_set(cpu, ST_OV, 0);
                }
                break;
            }
        }
        return 0;
    }

    // Format II: jumps and single-bit CRU (0x1000-0x1FFF)
    if (op >= 0x1000) {
        int code = (op >> 8) & 0xFF;
        int8_t disp = (int8_t)(op & 0xFF);
        if (code <= 0x1C) {
            if (jump_taken(cpu, code))
                cpu->pc = (uint16_t)(cpu->pc + 2 * disp);
        } else if (code == 0x1D) { // SBO
            cru_write(cpu, (uint16_t)(cru_base(cpu) + disp), 1);
        } else if (code == 0x1E) { // SBZ
            cru_write(cpu, (uint16_t)(cru_base(cpu) + disp), 0);
        } else { // 0x1F TB
            st_set(cpu, ST_EQ, cru_read(cpu, (uint16_t)(cru_base(cpu) + disp)));
        }
        return 0;
    }

    // Format V: shifts (0x0800-0x0BFF)
    if (op >= 0x0800) {
        int r = op & 0xF;
        int count = (op >> 4) & 0xF;
        if (count == 0) { // count taken from R0 low nibble; 0 there means 16
            count = get_reg(cpu, 0) & 0xF;
            if (count == 0) count = 16;
        }
        uint16_t val = get_reg(cpu, r);
        int carry = 0;
        switch ((op >> 8) & 3) {
            case 0: // SRA
                for (int i = 0; i < count; ++i) {
                    carry = val & 1;
                    val = (uint16_t)((val >> 1) | (val & 0x8000));
                }
                break;
            case 1: // SRL
                for (int i = 0; i < count; ++i) {
                    carry = val & 1;
                    val >>= 1;
                }
                break;
            case 2: { // SLA (OV when sign changes during shift)
                int ov = 0;
                for (int i = 0; i < count; ++i) {
                    carry = (val >> 15) & 1;
                    uint16_t next = (uint16_t)(val << 1);
                    if ((val ^ next) & 0x8000) ov = 1;
                    val = next;
                }
                st_set(cpu, ST_OV, ov);
                break;
            }
            case 3: // SRC (rotate right)
                for (int i = 0; i < count; ++i) {
                    carry = val & 1;
                    val = (uint16_t)((val >> 1) | (carry << 15));
                }
                break;
        }
        st_set(cpu, ST_C, carry);
        set_reg(cpu, r, val);
        set_lae16(cpu, val);
        return 0;
    }

    // Format VI: single operand (0x0400-0x07FF)
    if (op >= 0x0400) {
        int code = (op >> 6) & 0xF;
        int ts = (op >> 4) & 3, s = op & 0xF;
        uint16_t a = operand_addr(cpu, ts, s, 0);
        uint16_t v, res;
        switch (code) {
            case 0x0: { // BLWP: vector -> new WP/PC, link in new R13-R15
                uint16_t old_wp = cpu->wp, old_pc = cpu->pc;
                cpu->wp = mem_rw(cpu, a);
                cpu->pc = mem_rw(cpu, (uint16_t)(a + 2));
                set_reg(cpu, 13, old_wp);
                set_reg(cpu, 14, old_pc);
                set_reg(cpu, 15, cpu->st);
                break;
            }
            case 0x1: cpu->pc = a; break;                          // B
            case 0x2: return exec_op(cpu, mem_rw(cpu, a));         // X
            case 0x3: mem_ww(cpu, a, 0); break;                    // CLR
            case 0x4: // NEG
                res = alu_sub16(cpu, 0, mem_rw(cpu, a));
                mem_ww(cpu, a, res);
                set_lae16(cpu, res);
                break;
            case 0x5: // INV
                res = (uint16_t)~mem_rw(cpu, a);
                mem_ww(cpu, a, res);
                set_lae16(cpu, res);
                break;
            case 0x6: case 0x7: case 0x8: case 0x9: { // INC INCT DEC DECT
                uint16_t amount = (code == 0x6 || code == 0x8) ? 1 : 2;
                v = mem_rw(cpu, a);
                res = (code <= 0x7) ? alu_add16(cpu, v, amount)
                                    : alu_sub16(cpu, v, amount);
                mem_ww(cpu, a, res);
                set_lae16(cpu, res);
                break;
            }
            case 0xA: // BL
                set_reg(cpu, 11, cpu->pc);
                cpu->pc = a;
                break;
            case 0xB: // SWPB
                v = mem_rw(cpu, a);
                mem_ww(cpu, a, (uint16_t)((v << 8) | (v >> 8)));
                break;
            case 0xC: mem_ww(cpu, a, 0xFFFF); break;               // SETO
            case 0xD: // ABS (flags reflect the original operand)
                v = mem_rw(cpu, a);
                set_lae16(cpu, v);
                st_set(cpu, ST_C, 0);
                st_set(cpu, ST_OV, v == 0x8000);
                if (v & 0x8000) mem_ww(cpu, a, (uint16_t)(0 - v));
                break;
            default: // 0xE/0xF illegal; treated as NOP
                break;
        }
        return 0;
    }

    // Formats VII/VIII: immediate and control (0x0200-0x03FF)
    if (op >= 0x0200) {
        int code = (op >> 5) & 0xF;
        int w = op & 0xF;
        uint16_t imm, res;
        switch (code) {
            case 0x0: // LI
                imm = fetch_word(cpu);
                set_reg(cpu, w, imm);
                set_lae16(cpu, imm);
                break;
            case 0x1: // AI
                imm = fetch_word(cpu);
                res = alu_add16(cpu, get_reg(cpu, w), imm);
                set_reg(cpu, w, res);
                set_lae16(cpu, res);
                break;
            case 0x2: // ANDI
                res = get_reg(cpu, w) & fetch_word(cpu);
                set_reg(cpu, w, res);
                set_lae16(cpu, res);
                break;
            case 0x3: // ORI
                res = get_reg(cpu, w) | fetch_word(cpu);
                set_reg(cpu, w, res);
                set_lae16(cpu, res);
                break;
            case 0x4: // CI
                imm = fetch_word(cpu);
                compare16(cpu, get_reg(cpu, w), imm);
                break;
            case 0x5: set_reg(cpu, w, cpu->wp); break;             // STWP
            case 0x6: set_reg(cpu, w, cpu->st); break;             // STST
            case 0x7: cpu->wp = fetch_word(cpu); break;            // LWPI
            case 0x8: // LIMI
                cpu->st = (uint16_t)((cpu->st & ~ST_IMASK) | (fetch_word(cpu) & ST_IMASK));
                break;
            case 0xA: // IDLE
                cpu->halted = 1;
                return 1;
            case 0xB: cpu->st &= (uint16_t)~ST_IMASK; break;       // RSET
            case 0xC: { // RTWP
                uint16_t nwp = get_reg(cpu, 13);
                uint16_t npc = get_reg(cpu, 14);
                uint16_t nst = get_reg(cpu, 15);
                cpu->wp = nwp;
                cpu->pc = npc;
                cpu->st = nst;
                break;
            }
            case 0xD: case 0xE: case 0xF: // CKON CKOF LREX (no hardware)
                break;
            default:
                break;
        }
        return 0;
    }

    // 0x0000-0x01FF illegal; treated as NOP
    return 0;
}

// --- Public interface ---

void* tms9900_create(void) {
    return calloc(1, sizeof(TMS9900CPU));
}

void tms9900_destroy(void *context) {
    free(context);
}

int tms9900_init(void *context) {
    if (!context) return -1;
    TMS9900CPU *cpu = (TMS9900CPU*)context;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    memset(cpu->cru, 0, sizeof(cpu->cru));
    cpu->pc = 0;
    cpu->wp = RESET_WP;
    cpu->st = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int tms9900_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    TMS9900CPU *cpu = (TMS9900CPU*)context;
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) copy_len = MEM_SIZE - address;
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int tms9900_step(void *context) {
    if (!context) return -1;
    TMS9900CPU *cpu = (TMS9900CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint16_t op = fetch_word(cpu);
    cpu->ticks++;

    int result = exec_op(cpu, op);
    if (result != 0) return result;

    // Self-loop (e.g. JMP $) interpreted as a software halt, matching other cores.
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

// --- Disassembly ---

static const char* jmp_names[] = {
    "JMP", "JLT", "JLE", "JEQ", "JHE", "JGT", "JNE",
    "JNC", "JOC", "JNO", "JL", "JH", "JOP"
};

// Format a general-addressing operand; consumes an extra word for mode 2.
static void fmt_operand(TMS9900CPU *cpu, int ts, int r, uint16_t *offset,
                        char *out, size_t out_len) {
    switch (ts) {
        case 0:
            snprintf(out, out_len, "R%d", r);
            break;
        case 1:
            snprintf(out, out_len, "*R%d", r);
            break;
        case 2: {
            uint16_t w = mem_rw(cpu, (uint16_t)(cpu->pc + *offset));
            *offset += 2;
            if (r != 0) snprintf(out, out_len, "@>%04X(R%d)", w, r);
            else snprintf(out, out_len, "@>%04X", w);
            break;
        }
        case 3:
            snprintf(out, out_len, "*R%d+", r);
            break;
    }
}

void tms9900_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    TMS9900CPU *cpu = (TMS9900CPU*)context;

    uint16_t op = mem_rw(cpu, cpu->pc);
    uint16_t offset = 2; // bytes past PC where extra operand words live
    char src[24], dst[24];

    // Format I: dual operand
    if (op >= 0x4000) {
        static const char* names[] = {
            "SZC", "SZCB", "S", "SB", "C", "CB", "A", "AB",
            "MOV", "MOVB", "SOC", "SOCB"
        };
        fmt_operand(cpu, (op >> 4) & 3, op & 0xF, &offset, src, sizeof(src));
        fmt_operand(cpu, (op >> 10) & 3, (op >> 6) & 0xF, &offset, dst, sizeof(dst));
        snprintf(buf, buf_len, "%-5s %s, %s", names[(op >> 12) - 4], src, dst);
        return;
    }

    // Formats III/IV/IX
    if (op >= 0x2000) {
        static const char* names[] = {
            "COC", "CZC", "XOR", "XOP", "LDCR", "STCR", "MPY", "DIV"
        };
        int code = (op >> 10) & 7;
        int d = (op >> 6) & 0xF;
        fmt_operand(cpu, (op >> 4) & 3, op & 0xF, &offset, src, sizeof(src));
        if (code == 4 || code == 5) // LDCR/STCR take a bit count
            snprintf(buf, buf_len, "%-5s %s, %d", names[code], src, d ? d : 16);
        else
            snprintf(buf, buf_len, "%-5s %s, R%d", names[code], src, d);
        return;
    }

    // Format II: jumps and single-bit CRU
    if (op >= 0x1000) {
        int code = (op >> 8) & 0xFF;
        int8_t disp = (int8_t)(op & 0xFF);
        if (code <= 0x1C) {
            uint16_t target = (uint16_t)(cpu->pc + 2 + 2 * disp);
            snprintf(buf, buf_len, "%-5s >%04X", jmp_names[code - 0x10], target);
        } else if (code == 0x1D) {
            snprintf(buf, buf_len, "SBO   %d", disp);
        } else if (code == 0x1E) {
            snprintf(buf, buf_len, "SBZ   %d", disp);
        } else {
            snprintf(buf, buf_len, "TB    %d", disp);
        }
        return;
    }

    // Format V: shifts
    if (op >= 0x0800 && op < 0x0C00) {
        static const char* names[] = { "SRA", "SRL", "SLA", "SRC" };
        int count = (op >> 4) & 0xF;
        if (count == 0)
            snprintf(buf, buf_len, "%-5s R%d, R0", names[(op >> 8) & 3], op & 0xF);
        else
            snprintf(buf, buf_len, "%-5s R%d, %d", names[(op >> 8) & 3], op & 0xF, count);
        return;
    }

    // Format VI: single operand
    if (op >= 0x0400 && op < 0x0800) {
        static const char* names[] = {
            "BLWP", "B", "X", "CLR", "NEG", "INV", "INC", "INCT",
            "DEC", "DECT", "BL", "SWPB", "SETO", "ABS", "INV?", "INV?"
        };
        fmt_operand(cpu, (op >> 4) & 3, op & 0xF, &offset, src, sizeof(src));
        snprintf(buf, buf_len, "%-5s %s", names[(op >> 6) & 0xF], src);
        return;
    }

    // Formats VII/VIII: immediate and control
    if (op >= 0x0200 && op < 0x0400) {
        int code = (op >> 5) & 0xF;
        int w = op & 0xF;
        uint16_t imm = mem_rw(cpu, (uint16_t)(cpu->pc + 2));
        switch (code) {
            case 0x0: snprintf(buf, buf_len, "LI    R%d, >%04X", w, imm); return;
            case 0x1: snprintf(buf, buf_len, "AI    R%d, >%04X", w, imm); return;
            case 0x2: snprintf(buf, buf_len, "ANDI  R%d, >%04X", w, imm); return;
            case 0x3: snprintf(buf, buf_len, "ORI   R%d, >%04X", w, imm); return;
            case 0x4: snprintf(buf, buf_len, "CI    R%d, >%04X", w, imm); return;
            case 0x5: snprintf(buf, buf_len, "STWP  R%d", w); return;
            case 0x6: snprintf(buf, buf_len, "STST  R%d", w); return;
            case 0x7: snprintf(buf, buf_len, "LWPI  >%04X", imm); return;
            case 0x8: snprintf(buf, buf_len, "LIMI  %d", imm & ST_IMASK); return;
            case 0xA: snprintf(buf, buf_len, "IDLE"); return;
            case 0xB: snprintf(buf, buf_len, "RSET"); return;
            case 0xC: snprintf(buf, buf_len, "RTWP"); return;
            case 0xD: snprintf(buf, buf_len, "CKON"); return;
            case 0xE: snprintf(buf, buf_len, "CKOF"); return;
            case 0xF: snprintf(buf, buf_len, "LREX"); return;
        }
    }

    snprintf(buf, buf_len, "DATA  >%04X", op);
}

void tms9900_print_state(void *context) {
    if (!context) return;
    TMS9900CPU *cpu = (TMS9900CPU*)context;

    printf("TMS9900 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  WP: 0x%04X  ST: 0x%04X  Halted: %s\n",
           cpu->pc, cpu->wp, cpu->st, cpu->halted ? "Yes" : "No");
    printf("  Flags: L>=%d A>=%d EQ=%d C=%d OV=%d OP=%d IntMask=%d\n",
           (cpu->st & ST_LGT) != 0, (cpu->st & ST_AGT) != 0,
           (cpu->st & ST_EQ) != 0, (cpu->st & ST_C) != 0,
           (cpu->st & ST_OV) != 0, (cpu->st & ST_OP) != 0,
           cpu->st & ST_IMASK);
    printf("  Workspace registers (at WP 0x%04X):\n", cpu->wp);
    for (int row = 0; row < 4; ++row) {
        printf("   ");
        for (int col = 0; col < 4; ++col) {
            int r = row * 4 + col;
            printf(" R%-2d: 0x%04X", r, get_reg(cpu, r));
        }
        printf("\n");
    }
}
