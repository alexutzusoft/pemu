#include "i8051.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CODE_SIZE 65536
#define XRAM_SIZE 65536
#define IRAM_SIZE 256
#define SFR_SIZE  128

typedef struct I8051_CPU {
    uint8_t code[CODE_SIZE];   // 64 KB program memory
    uint8_t iram[IRAM_SIZE];   // 256-byte internal RAM (upper 128 indirect-only)
    uint8_t sfr[SFR_SIZE];     // SFR space, direct addresses 0x80-0xFF
    uint8_t xram[XRAM_SIZE];   // 64 KB external RAM (MOVX)
    uint16_t pc;
    uint32_t ticks;
    int halted;
} I8051_CPU;

// SFR accessors (direct address minus 0x80)
#define SFR_AT(a)  cpu->sfr[(a) - 0x80]
#define ACC        SFR_AT(0xE0)
#define B_REG      SFR_AT(0xF0)
#define PSW        SFR_AT(0xD0)
#define SP_REG     SFR_AT(0x81)
#define DPL        SFR_AT(0x82)
#define DPH        SFR_AT(0x83)
#define DPTR       ((uint16_t)(((uint16_t)DPH << 8) | DPL))

// PSW flag bits
#define FLAG_CY 7
#define FLAG_AC 6
#define FLAG_OV 2
#define FLAG_P  0

#define GET_CY() ((PSW >> FLAG_CY) & 1)
#define SET_PSW_BIT(bit, val) do { if (val) PSW |= (uint8_t)(1 << (bit)); else PSW &= (uint8_t)~(1 << (bit)); } while (0)

// Active register bank base: RS1:RS0 are PSW bits 4:3, base = bank * 8
#define RBANK ((uint8_t)(PSW & 0x18))
#define REG(n) cpu->iram[RBANK + (n)]

// Instruction length table (indexed by opcode)
static const uint8_t oplen[256] = {
    // 0x00-0x0F
    1,2,3,1,1,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x10-0x1F
    3,2,3,1,1,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x20-0x2F
    3,2,1,1,2,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x30-0x3F
    3,2,1,1,2,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x40-0x4F
    2,2,2,3,2,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x50-0x5F
    2,2,2,3,2,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x60-0x6F
    2,2,2,3,2,2,1,1, 1,1,1,1,1,1,1,1,
    // 0x70-0x7F
    2,2,2,1,2,3,2,2, 2,2,2,2,2,2,2,2,
    // 0x80-0x8F
    2,2,2,1,1,3,2,2, 2,2,2,2,2,2,2,2,
    // 0x90-0x9F
    3,2,2,1,2,2,1,1, 1,1,1,1,1,1,1,1,
    // 0xA0-0xAF
    2,2,2,1,1,1,2,2, 2,2,2,2,2,2,2,2,
    // 0xB0-0xBF
    2,2,2,1,3,3,3,3, 3,3,3,3,3,3,3,3,
    // 0xC0-0xCF
    2,2,2,1,1,2,1,1, 1,1,1,1,1,1,1,1,
    // 0xD0-0xDF
    2,2,2,1,1,3,1,1, 2,2,2,2,2,2,2,2,
    // 0xE0-0xEF
    1,2,1,1,1,2,1,1, 1,1,1,1,1,1,1,1,
    // 0xF0-0xFF
    1,2,1,1,1,2,1,1, 1,1,1,1,1,1,1,1
};

void* i8051_create(void) {
    I8051_CPU *cpu = (I8051_CPU*)calloc(1, sizeof(I8051_CPU));
    return cpu;
}

void i8051_destroy(void *context) {
    free(context);
}

int i8051_init(void *context) {
    if (!context) return -1;
    I8051_CPU *cpu = (I8051_CPU*)context;

    memset(cpu->iram, 0, sizeof(cpu->iram));
    memset(cpu->sfr, 0, sizeof(cpu->sfr));
    memset(cpu->xram, 0, sizeof(cpu->xram));
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;

    // Reset values
    SP_REG = 0x07;
    SFR_AT(0x80) = 0xFF; // P0
    SFR_AT(0x90) = 0xFF; // P1
    SFR_AT(0xA0) = 0xFF; // P2
    SFR_AT(0xB0) = 0xFF; // P3

    return 0;
}

int i8051_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    I8051_CPU *cpu = (I8051_CPU*)context;

    if (address >= CODE_SIZE) return -2;

    size_t copy_len = size;
    if (address + size > CODE_SIZE) {
        copy_len = CODE_SIZE - address;
    }

    memcpy(cpu->code + address, data, copy_len);
    return 0;
}

// Direct addressing: 0x00-0x7F -> lower internal RAM, 0x80-0xFF -> SFR space
static uint8_t rd_direct(I8051_CPU *cpu, uint8_t addr) {
    return (addr < 0x80) ? cpu->iram[addr] : cpu->sfr[addr - 0x80];
}

static void wr_direct(I8051_CPU *cpu, uint8_t addr, uint8_t val) {
    if (addr < 0x80) cpu->iram[addr] = val;
    else cpu->sfr[addr - 0x80] = val;
}

// Bit addressing: 0x00-0x7F -> bytes 0x20-0x2F of internal RAM, 0x80-0xFF -> SFRs at addr & 0xF8
static uint8_t bit_byte_addr(uint8_t bit) {
    return (bit < 0x80) ? (uint8_t)(0x20 + (bit >> 3)) : (uint8_t)(bit & 0xF8);
}

static uint8_t rd_bit(I8051_CPU *cpu, uint8_t bit) {
    uint8_t v = rd_direct(cpu, bit_byte_addr(bit));
    return (uint8_t)((v >> (bit & 7)) & 1);
}

static void wr_bit(I8051_CPU *cpu, uint8_t bit, uint8_t val) {
    uint8_t addr = bit_byte_addr(bit);
    uint8_t v = rd_direct(cpu, addr);
    uint8_t mask = (uint8_t)(1 << (bit & 7));
    wr_direct(cpu, addr, val ? (uint8_t)(v | mask) : (uint8_t)(v & ~mask));
}

static void push_byte(I8051_CPU *cpu, uint8_t val) {
    SP_REG = (uint8_t)(SP_REG + 1);
    cpu->iram[SP_REG] = val;
}

static uint8_t pop_byte(I8051_CPU *cpu) {
    uint8_t val = cpu->iram[SP_REG];
    SP_REG = (uint8_t)(SP_REG - 1);
    return val;
}

// Source operand for arithmetic/logic rows (low nibble 4=imm, 5=direct, 6/7=@Ri, 8-F=Rn)
static uint8_t src_operand(I8051_CPU *cpu, uint8_t op, uint8_t b1) {
    uint8_t lo = (uint8_t)(op & 0x0F);
    if (lo == 4) return b1;
    if (lo == 5) return rd_direct(cpu, b1);
    if (lo == 6 || lo == 7) return cpu->iram[REG(op & 1)];
    return REG(op & 7);
}

static void do_add(I8051_CPU *cpu, uint8_t val, uint8_t carry_in) {
    uint8_t a = ACC;
    uint16_t r = (uint16_t)(a + val + carry_in);
    uint8_t half = (uint8_t)((a & 0x0F) + (val & 0x0F) + carry_in);
    SET_PSW_BIT(FLAG_CY, r > 0xFF);
    SET_PSW_BIT(FLAG_AC, half > 0x0F);
    SET_PSW_BIT(FLAG_OV, ((~(a ^ val) & (a ^ (uint8_t)r)) & 0x80) != 0);
    ACC = (uint8_t)r;
}

static void do_subb(I8051_CPU *cpu, uint8_t val) {
    uint8_t a = ACC;
    uint8_t c = GET_CY();
    int r = (int)a - (int)val - (int)c;
    SET_PSW_BIT(FLAG_CY, r < 0);
    SET_PSW_BIT(FLAG_AC, (a & 0x0F) < (uint8_t)((val & 0x0F) + c));
    SET_PSW_BIT(FLAG_OV, (((a ^ val) & (a ^ (uint8_t)r)) & 0x80) != 0);
    ACC = (uint8_t)r;
}

static void update_parity(I8051_CPU *cpu) {
    uint8_t v = ACC;
    v ^= (uint8_t)(v >> 4);
    v ^= (uint8_t)(v >> 2);
    v ^= (uint8_t)(v >> 1);
    SET_PSW_BIT(FLAG_P, v & 1);
}

int i8051_step(void *context) {
    if (!context) return -1;
    I8051_CPU *cpu = (I8051_CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = cpu->code[cpu->pc];
    uint8_t b1 = cpu->code[(uint16_t)(cpu->pc + 1)];
    uint8_t b2 = cpu->code[(uint16_t)(cpu->pc + 2)];
    uint8_t hi = (uint8_t)(op >> 4);
    uint8_t lo = (uint8_t)(op & 0x0F);
    uint16_t npc = (uint16_t)(cpu->pc + oplen[op]);
    cpu->ticks++;

    // AJMP / ACALL (low nibble 1)
    if (lo == 1) {
        uint16_t target = (uint16_t)((npc & 0xF800) | ((uint16_t)(op & 0xE0) << 3) | b1);
        if (hi & 1) { // ACALL
            push_byte(cpu, (uint8_t)(npc & 0xFF));
            push_byte(cpu, (uint8_t)(npc >> 8));
        }
        npc = target;
        cpu->pc = npc;
        update_parity(cpu);
        return 0;
    }

    switch (op) {
    case 0x00: break; // NOP
    case 0x02: npc = (uint16_t)(((uint16_t)b1 << 8) | b2); break; // LJMP
    case 0x03: ACC = (uint8_t)((ACC << 7) | (ACC >> 1)); break;   // RR A
    case 0x04: ACC = (uint8_t)(ACC + 1); break;                   // INC A
    case 0x05: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) + 1)); break; // INC dir
    case 0x06: case 0x07: cpu->iram[REG(op & 1)] = (uint8_t)(cpu->iram[REG(op & 1)] + 1); break; // INC @Ri
    case 0x08: case 0x09: case 0x0A: case 0x0B:
    case 0x0C: case 0x0D: case 0x0E: case 0x0F:
        REG(op & 7) = (uint8_t)(REG(op & 7) + 1); break; // INC Rn

    case 0x10: // JBC bit, rel
        if (rd_bit(cpu, b1)) {
            wr_bit(cpu, b1, 0);
            npc = (uint16_t)(npc + (int8_t)b2);
        }
        break;
    case 0x12: // LCALL addr16
        push_byte(cpu, (uint8_t)(npc & 0xFF));
        push_byte(cpu, (uint8_t)(npc >> 8));
        npc = (uint16_t)(((uint16_t)b1 << 8) | b2);
        break;
    case 0x13: { // RRC A
        uint8_t c = GET_CY();
        SET_PSW_BIT(FLAG_CY, ACC & 1);
        ACC = (uint8_t)((ACC >> 1) | (c << 7));
        break;
    }
    case 0x14: ACC = (uint8_t)(ACC - 1); break; // DEC A
    case 0x15: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) - 1)); break; // DEC dir
    case 0x16: case 0x17: cpu->iram[REG(op & 1)] = (uint8_t)(cpu->iram[REG(op & 1)] - 1); break; // DEC @Ri
    case 0x18: case 0x19: case 0x1A: case 0x1B:
    case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        REG(op & 7) = (uint8_t)(REG(op & 7) - 1); break; // DEC Rn

    case 0x20: if (rd_bit(cpu, b1)) npc = (uint16_t)(npc + (int8_t)b2); break;  // JB
    case 0x22: { // RET
        uint8_t h = pop_byte(cpu);
        uint8_t l = pop_byte(cpu);
        npc = (uint16_t)(((uint16_t)h << 8) | l);
        break;
    }
    case 0x23: ACC = (uint8_t)((ACC << 1) | (ACC >> 7)); break; // RL A
    case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x28: case 0x29: case 0x2A: case 0x2B:
    case 0x2C: case 0x2D: case 0x2E: case 0x2F:
        do_add(cpu, src_operand(cpu, op, b1), 0); break; // ADD

    case 0x30: if (!rd_bit(cpu, b1)) npc = (uint16_t)(npc + (int8_t)b2); break; // JNB
    case 0x32: { // RETI (no interrupt controller: behaves as RET)
        uint8_t h = pop_byte(cpu);
        uint8_t l = pop_byte(cpu);
        npc = (uint16_t)(((uint16_t)h << 8) | l);
        break;
    }
    case 0x33: { // RLC A
        uint8_t c = GET_CY();
        SET_PSW_BIT(FLAG_CY, ACC >> 7);
        ACC = (uint8_t)((ACC << 1) | c);
        break;
    }
    case 0x34: case 0x35: case 0x36: case 0x37:
    case 0x38: case 0x39: case 0x3A: case 0x3B:
    case 0x3C: case 0x3D: case 0x3E: case 0x3F:
        do_add(cpu, src_operand(cpu, op, b1), GET_CY()); break; // ADDC

    case 0x40: if (GET_CY()) npc = (uint16_t)(npc + (int8_t)b1); break; // JC
    case 0x42: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) | ACC)); break; // ORL dir, A
    case 0x43: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) | b2)); break;  // ORL dir, #
    case 0x44: case 0x45: case 0x46: case 0x47:
    case 0x48: case 0x49: case 0x4A: case 0x4B:
    case 0x4C: case 0x4D: case 0x4E: case 0x4F:
        ACC = (uint8_t)(ACC | src_operand(cpu, op, b1)); break; // ORL A, src

    case 0x50: if (!GET_CY()) npc = (uint16_t)(npc + (int8_t)b1); break; // JNC
    case 0x52: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) & ACC)); break; // ANL dir, A
    case 0x53: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) & b2)); break;  // ANL dir, #
    case 0x54: case 0x55: case 0x56: case 0x57:
    case 0x58: case 0x59: case 0x5A: case 0x5B:
    case 0x5C: case 0x5D: case 0x5E: case 0x5F:
        ACC = (uint8_t)(ACC & src_operand(cpu, op, b1)); break; // ANL A, src

    case 0x60: if (ACC == 0) npc = (uint16_t)(npc + (int8_t)b1); break; // JZ
    case 0x62: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) ^ ACC)); break; // XRL dir, A
    case 0x63: wr_direct(cpu, b1, (uint8_t)(rd_direct(cpu, b1) ^ b2)); break;  // XRL dir, #
    case 0x64: case 0x65: case 0x66: case 0x67:
    case 0x68: case 0x69: case 0x6A: case 0x6B:
    case 0x6C: case 0x6D: case 0x6E: case 0x6F:
        ACC = (uint8_t)(ACC ^ src_operand(cpu, op, b1)); break; // XRL A, src

    case 0x70: if (ACC != 0) npc = (uint16_t)(npc + (int8_t)b1); break; // JNZ
    case 0x72: SET_PSW_BIT(FLAG_CY, GET_CY() | rd_bit(cpu, b1)); break; // ORL C, bit
    case 0x73: npc = (uint16_t)(DPTR + ACC); break; // JMP @A+DPTR
    case 0x74: ACC = b1; break;                     // MOV A, #
    case 0x75: wr_direct(cpu, b1, b2); break;       // MOV dir, #
    case 0x76: case 0x77: cpu->iram[REG(op & 1)] = b1; break; // MOV @Ri, #
    case 0x78: case 0x79: case 0x7A: case 0x7B:
    case 0x7C: case 0x7D: case 0x7E: case 0x7F:
        REG(op & 7) = b1; break; // MOV Rn, #

    case 0x80: // SJMP rel
        if ((int8_t)b1 == -2) {
            cpu->halted = 1;
            update_parity(cpu);
            return 1;
        }
        npc = (uint16_t)(npc + (int8_t)b1);
        break;
    case 0x82: SET_PSW_BIT(FLAG_CY, GET_CY() & rd_bit(cpu, b1)); break; // ANL C, bit
    case 0x83: ACC = cpu->code[(uint16_t)(npc + ACC)]; break; // MOVC A, @A+PC
    case 0x84: // DIV AB
        SET_PSW_BIT(FLAG_CY, 0);
        if (B_REG == 0) {
            SET_PSW_BIT(FLAG_OV, 1);
        } else {
            uint8_t q = (uint8_t)(ACC / B_REG);
            uint8_t r = (uint8_t)(ACC % B_REG);
            ACC = q;
            B_REG = r;
            SET_PSW_BIT(FLAG_OV, 0);
        }
        break;
    case 0x85: wr_direct(cpu, b2, rd_direct(cpu, b1)); break; // MOV dir, dir (src first)
    case 0x86: case 0x87: wr_direct(cpu, b1, cpu->iram[REG(op & 1)]); break; // MOV dir, @Ri
    case 0x88: case 0x89: case 0x8A: case 0x8B:
    case 0x8C: case 0x8D: case 0x8E: case 0x8F:
        wr_direct(cpu, b1, REG(op & 7)); break; // MOV dir, Rn

    case 0x90: DPH = b1; DPL = b2; break; // MOV DPTR, #16
    case 0x92: wr_bit(cpu, b1, GET_CY()); break; // MOV bit, C
    case 0x93: ACC = cpu->code[(uint16_t)(DPTR + ACC)]; break; // MOVC A, @A+DPTR
    case 0x94: case 0x95: case 0x96: case 0x97:
    case 0x98: case 0x99: case 0x9A: case 0x9B:
    case 0x9C: case 0x9D: case 0x9E: case 0x9F:
        do_subb(cpu, src_operand(cpu, op, b1)); break; // SUBB A, src

    case 0xA0: SET_PSW_BIT(FLAG_CY, GET_CY() | (uint8_t)(rd_bit(cpu, b1) ^ 1)); break; // ORL C, /bit
    case 0xA2: SET_PSW_BIT(FLAG_CY, rd_bit(cpu, b1)); break; // MOV C, bit
    case 0xA3: { // INC DPTR
        uint16_t d = (uint16_t)(DPTR + 1);
        DPH = (uint8_t)(d >> 8);
        DPL = (uint8_t)(d & 0xFF);
        break;
    }
    case 0xA4: { // MUL AB
        uint16_t p = (uint16_t)(ACC * B_REG);
        ACC = (uint8_t)(p & 0xFF);
        B_REG = (uint8_t)(p >> 8);
        SET_PSW_BIT(FLAG_CY, 0);
        SET_PSW_BIT(FLAG_OV, p > 0xFF);
        break;
    }
    case 0xA5: return -2; // reserved opcode
    case 0xA6: case 0xA7: cpu->iram[REG(op & 1)] = rd_direct(cpu, b1); break; // MOV @Ri, dir
    case 0xA8: case 0xA9: case 0xAA: case 0xAB:
    case 0xAC: case 0xAD: case 0xAE: case 0xAF:
        REG(op & 7) = rd_direct(cpu, b1); break; // MOV Rn, dir

    case 0xB0: SET_PSW_BIT(FLAG_CY, GET_CY() & (uint8_t)(rd_bit(cpu, b1) ^ 1)); break; // ANL C, /bit
    case 0xB2: wr_bit(cpu, b1, (uint8_t)(rd_bit(cpu, b1) ^ 1)); break; // CPL bit
    case 0xB3: SET_PSW_BIT(FLAG_CY, GET_CY() ^ 1); break; // CPL C
    case 0xB4: case 0xB5: case 0xB6: case 0xB7:
    case 0xB8: case 0xB9: case 0xBA: case 0xBB:
    case 0xBC: case 0xBD: case 0xBE: case 0xBF: { // CJNE
        uint8_t first, second, rel;
        if (op == 0xB4) { first = ACC; second = b1; }
        else if (op == 0xB5) { first = ACC; second = rd_direct(cpu, b1); }
        else if (op == 0xB6 || op == 0xB7) { first = cpu->iram[REG(op & 1)]; second = b1; }
        else { first = REG(op & 7); second = b1; }
        rel = b2;
        SET_PSW_BIT(FLAG_CY, first < second);
        if (first != second) npc = (uint16_t)(npc + (int8_t)rel);
        break;
    }

    case 0xC0: push_byte(cpu, rd_direct(cpu, b1)); break; // PUSH dir
    case 0xC2: wr_bit(cpu, b1, 0); break; // CLR bit
    case 0xC3: SET_PSW_BIT(FLAG_CY, 0); break; // CLR C
    case 0xC4: ACC = (uint8_t)((ACC << 4) | (ACC >> 4)); break; // SWAP A
    case 0xC5: { // XCH A, dir
        uint8_t t = rd_direct(cpu, b1);
        wr_direct(cpu, b1, ACC);
        ACC = t;
        break;
    }
    case 0xC6: case 0xC7: { // XCH A, @Ri
        uint8_t addr = REG(op & 1);
        uint8_t t = cpu->iram[addr];
        cpu->iram[addr] = ACC;
        ACC = t;
        break;
    }
    case 0xC8: case 0xC9: case 0xCA: case 0xCB:
    case 0xCC: case 0xCD: case 0xCE: case 0xCF: { // XCH A, Rn
        uint8_t t = REG(op & 7);
        REG(op & 7) = ACC;
        ACC = t;
        break;
    }

    case 0xD0: wr_direct(cpu, b1, pop_byte(cpu)); break; // POP dir
    case 0xD2: wr_bit(cpu, b1, 1); break; // SETB bit
    case 0xD3: SET_PSW_BIT(FLAG_CY, 1); break; // SETB C
    case 0xD4: { // DA A
        uint16_t a = ACC;
        if ((a & 0x0F) > 9 || ((PSW >> FLAG_AC) & 1)) a += 0x06;
        if (a > 0xFF) SET_PSW_BIT(FLAG_CY, 1);
        a &= 0xFF;
        if ((a & 0xF0) > 0x90 || GET_CY()) a += 0x60;
        if (a > 0xFF) SET_PSW_BIT(FLAG_CY, 1);
        ACC = (uint8_t)a;
        break;
    }
    case 0xD5: { // DJNZ dir, rel
        uint8_t v = (uint8_t)(rd_direct(cpu, b1) - 1);
        wr_direct(cpu, b1, v);
        if (v != 0) npc = (uint16_t)(npc + (int8_t)b2);
        break;
    }
    case 0xD6: case 0xD7: { // XCHD A, @Ri
        uint8_t addr = REG(op & 1);
        uint8_t t = cpu->iram[addr];
        cpu->iram[addr] = (uint8_t)((t & 0xF0) | (ACC & 0x0F));
        ACC = (uint8_t)((ACC & 0xF0) | (t & 0x0F));
        break;
    }
    case 0xD8: case 0xD9: case 0xDA: case 0xDB:
    case 0xDC: case 0xDD: case 0xDE: case 0xDF: { // DJNZ Rn, rel
        uint8_t v = (uint8_t)(REG(op & 7) - 1);
        REG(op & 7) = v;
        if (v != 0) npc = (uint16_t)(npc + (int8_t)b1);
        break;
    }

    case 0xE0: ACC = cpu->xram[DPTR]; break; // MOVX A, @DPTR
    case 0xE2: case 0xE3: // MOVX A, @Ri (page from P2)
        ACC = cpu->xram[(uint16_t)(((uint16_t)SFR_AT(0xA0) << 8) | REG(op & 1))];
        break;
    case 0xE4: ACC = 0; break; // CLR A
    case 0xE5: case 0xE6: case 0xE7:
    case 0xE8: case 0xE9: case 0xEA: case 0xEB:
    case 0xEC: case 0xED: case 0xEE: case 0xEF:
        ACC = src_operand(cpu, op, b1); break; // MOV A, src

    case 0xF0: cpu->xram[DPTR] = ACC; break; // MOVX @DPTR, A
    case 0xF2: case 0xF3: // MOVX @Ri, A (page from P2)
        cpu->xram[(uint16_t)(((uint16_t)SFR_AT(0xA0) << 8) | REG(op & 1))] = ACC;
        break;
    case 0xF4: ACC = (uint8_t)~ACC; break; // CPL A
    case 0xF5: wr_direct(cpu, b1, ACC); break; // MOV dir, A
    case 0xF6: case 0xF7: cpu->iram[REG(op & 1)] = ACC; break; // MOV @Ri, A
    case 0xF8: case 0xF9: case 0xFA: case 0xFB:
    case 0xFC: case 0xFD: case 0xFE: case 0xFF:
        REG(op & 7) = ACC; break; // MOV Rn, A

    default:
        return -2;
    }

    cpu->pc = npc;
    update_parity(cpu);
    return 0;
}

void i8051_print_state(void *context) {
    if (!context) return;
    I8051_CPU *cpu = (I8051_CPU*)context;

    printf("Intel 8051 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  Halted: %s\n", cpu->pc, cpu->halted ? "Yes" : "No");
    printf("  ACC: 0x%02X  B: 0x%02X  PSW: 0x%02X (CY=%d AC=%d F0=%d RS1=%d RS0=%d OV=%d P=%d)\n",
           ACC, B_REG, PSW,
           (PSW >> 7) & 1, (PSW >> 6) & 1, (PSW >> 5) & 1,
           (PSW >> 4) & 1, (PSW >> 3) & 1, (PSW >> 2) & 1, PSW & 1);
    printf("  SP: 0x%02X  DPTR: 0x%04X\n", SP_REG, DPTR);

    printf("  Registers (bank %d):\n", (PSW >> 3) & 3);
    for (int i = 0; i < 8; ++i) {
        printf("    R%d: 0x%02X%s", i, REG(i), (i == 3 || i == 7) ? "\n" : "  ");
    }

    printf("  Ports: P0=0x%02X  P1=0x%02X  P2=0x%02X  P3=0x%02X\n",
           SFR_AT(0x80), SFR_AT(0x90), SFR_AT(0xA0), SFR_AT(0xB0));
}

void i8051_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    I8051_CPU *cpu = (I8051_CPU*)context;

    uint8_t op = cpu->code[cpu->pc];
    uint8_t b1 = cpu->code[(uint16_t)(cpu->pc + 1)];
    uint8_t b2 = cpu->code[(uint16_t)(cpu->pc + 2)];
    uint8_t hi = (uint8_t)(op >> 4);
    uint8_t lo = (uint8_t)(op & 0x0F);
    uint16_t npc = (uint16_t)(cpu->pc + oplen[op]);

    if (lo == 1) { // AJMP / ACALL
        uint16_t target = (uint16_t)((npc & 0xF800) | ((uint16_t)(op & 0xE0) << 3) | b1);
        snprintf(buf, buf_len, "%s 0x%04X", (hi & 1) ? "acall" : "ajmp ", target);
        return;
    }

    // Rows with a common source operand pattern (low nibble 4-F)
    if (lo >= 4) {
        static const char *row_ops[16] = {
            "inc  ", "dec  ", "add  ", "addc ", "orl  ", "anl  ", "xrl  ", "mov  ",
            "mov  ", "subb ", "mov  ", "cjne ", "xch  ", "djnz ", "mov  ", "mov  "
        };
        char operand[24];
        int handled = 1;

        if (lo == 4) snprintf(operand, sizeof(operand), "#0x%02X", b1);
        else if (lo == 5) snprintf(operand, sizeof(operand), "0x%02X", b1);
        else if (lo == 6 || lo == 7) snprintf(operand, sizeof(operand), "@R%d", op & 1);
        else snprintf(operand, sizeof(operand), "R%d", op & 7);

        switch (hi) {
        case 0x0: case 0x1: // INC / DEC (lo==4 means A)
            if (lo == 4) snprintf(buf, buf_len, "%sA", row_ops[hi]);
            else snprintf(buf, buf_len, "%s%s", row_ops[hi], operand);
            break;
        case 0x2: case 0x3: case 0x4: case 0x5: case 0x6: case 0x9:
            snprintf(buf, buf_len, "%sA, %s", row_ops[hi], operand);
            break;
        case 0x7: // MOV dst, # (lo==4 is MOV A,#, lo==5 is MOV dir,#)
            if (lo == 4) snprintf(buf, buf_len, "mov   A, #0x%02X", b1);
            else if (lo == 5) snprintf(buf, buf_len, "mov   0x%02X, #0x%02X", b1, b2);
            else snprintf(buf, buf_len, "mov   %s, #0x%02X", operand, b1);
            break;
        case 0x8: // MOV dir, src (lo==4 is DIV AB, lo==5 is MOV dir,dir)
            if (lo == 4) snprintf(buf, buf_len, "div   AB");
            else if (lo == 5) snprintf(buf, buf_len, "mov   0x%02X, 0x%02X", b2, b1);
            else snprintf(buf, buf_len, "mov   0x%02X, %s", b1, operand);
            break;
        case 0xA: // MOV dst, dir (lo==4 is MUL AB, lo==5 reserved)
            if (lo == 4) snprintf(buf, buf_len, "mul   AB");
            else if (lo == 5) snprintf(buf, buf_len, "db    0xA5");
            else snprintf(buf, buf_len, "mov   %s, 0x%02X", operand, b1);
            break;
        case 0xB: // CJNE
            if (lo == 4) snprintf(buf, buf_len, "cjne  A, #0x%02X, 0x%04X", b1, (uint16_t)(npc + (int8_t)b2));
            else if (lo == 5) snprintf(buf, buf_len, "cjne  A, 0x%02X, 0x%04X", b1, (uint16_t)(npc + (int8_t)b2));
            else snprintf(buf, buf_len, "cjne  %s, #0x%02X, 0x%04X", operand, b1, (uint16_t)(npc + (int8_t)b2));
            break;
        case 0xC: // XCH (lo==4 is SWAP A)
            if (lo == 4) snprintf(buf, buf_len, "swap  A");
            else snprintf(buf, buf_len, "xch   A, %s", operand);
            break;
        case 0xD: // DJNZ (lo==4 DA A, lo==5 DJNZ dir, lo==6/7 XCHD)
            if (lo == 4) snprintf(buf, buf_len, "da    A");
            else if (lo == 5) snprintf(buf, buf_len, "djnz  0x%02X, 0x%04X", b1, (uint16_t)(npc + (int8_t)b2));
            else if (lo == 6 || lo == 7) snprintf(buf, buf_len, "xchd  A, %s", operand);
            else snprintf(buf, buf_len, "djnz  %s, 0x%04X", operand, (uint16_t)(npc + (int8_t)b1));
            break;
        case 0xE: // MOV A, src (lo==4 CLR A)
            if (lo == 4) snprintf(buf, buf_len, "clr   A");
            else snprintf(buf, buf_len, "mov   A, %s", operand);
            break;
        case 0xF: // MOV dst, A (lo==4 CPL A)
            if (lo == 4) snprintf(buf, buf_len, "cpl   A");
            else snprintf(buf, buf_len, "mov   %s, A", operand);
            break;
        default:
            handled = 0;
            break;
        }
        if (handled) return;
    }

    switch (op) {
    case 0x00: snprintf(buf, buf_len, "nop"); break;
    case 0x02: snprintf(buf, buf_len, "ljmp  0x%04X", (uint16_t)(((uint16_t)b1 << 8) | b2)); break;
    case 0x03: snprintf(buf, buf_len, "rr    A"); break;
    case 0x10: snprintf(buf, buf_len, "jbc   0x%02X, 0x%04X", b1, (uint16_t)(npc + (int8_t)b2)); break;
    case 0x12: snprintf(buf, buf_len, "lcall 0x%04X", (uint16_t)(((uint16_t)b1 << 8) | b2)); break;
    case 0x13: snprintf(buf, buf_len, "rrc   A"); break;
    case 0x20: snprintf(buf, buf_len, "jb    0x%02X, 0x%04X", b1, (uint16_t)(npc + (int8_t)b2)); break;
    case 0x22: snprintf(buf, buf_len, "ret"); break;
    case 0x23: snprintf(buf, buf_len, "rl    A"); break;
    case 0x30: snprintf(buf, buf_len, "jnb   0x%02X, 0x%04X", b1, (uint16_t)(npc + (int8_t)b2)); break;
    case 0x32: snprintf(buf, buf_len, "reti"); break;
    case 0x33: snprintf(buf, buf_len, "rlc   A"); break;
    case 0x40: snprintf(buf, buf_len, "jc    0x%04X", (uint16_t)(npc + (int8_t)b1)); break;
    case 0x42: snprintf(buf, buf_len, "orl   0x%02X, A", b1); break;
    case 0x43: snprintf(buf, buf_len, "orl   0x%02X, #0x%02X", b1, b2); break;
    case 0x50: snprintf(buf, buf_len, "jnc   0x%04X", (uint16_t)(npc + (int8_t)b1)); break;
    case 0x52: snprintf(buf, buf_len, "anl   0x%02X, A", b1); break;
    case 0x53: snprintf(buf, buf_len, "anl   0x%02X, #0x%02X", b1, b2); break;
    case 0x60: snprintf(buf, buf_len, "jz    0x%04X", (uint16_t)(npc + (int8_t)b1)); break;
    case 0x62: snprintf(buf, buf_len, "xrl   0x%02X, A", b1); break;
    case 0x63: snprintf(buf, buf_len, "xrl   0x%02X, #0x%02X", b1, b2); break;
    case 0x70: snprintf(buf, buf_len, "jnz   0x%04X", (uint16_t)(npc + (int8_t)b1)); break;
    case 0x72: snprintf(buf, buf_len, "orl   C, 0x%02X", b1); break;
    case 0x73: snprintf(buf, buf_len, "jmp   @A+DPTR"); break;
    case 0x80: snprintf(buf, buf_len, "sjmp  0x%04X", (uint16_t)(npc + (int8_t)b1)); break;
    case 0x82: snprintf(buf, buf_len, "anl   C, 0x%02X", b1); break;
    case 0x83: snprintf(buf, buf_len, "movc  A, @A+PC"); break;
    case 0x90: snprintf(buf, buf_len, "mov   DPTR, #0x%04X", (uint16_t)(((uint16_t)b1 << 8) | b2)); break;
    case 0x92: snprintf(buf, buf_len, "mov   0x%02X, C", b1); break;
    case 0x93: snprintf(buf, buf_len, "movc  A, @A+DPTR"); break;
    case 0xA0: snprintf(buf, buf_len, "orl   C, /0x%02X", b1); break;
    case 0xA2: snprintf(buf, buf_len, "mov   C, 0x%02X", b1); break;
    case 0xA3: snprintf(buf, buf_len, "inc   DPTR"); break;
    case 0xB0: snprintf(buf, buf_len, "anl   C, /0x%02X", b1); break;
    case 0xB2: snprintf(buf, buf_len, "cpl   0x%02X", b1); break;
    case 0xB3: snprintf(buf, buf_len, "cpl   C"); break;
    case 0xC0: snprintf(buf, buf_len, "push  0x%02X", b1); break;
    case 0xC2: snprintf(buf, buf_len, "clr   0x%02X", b1); break;
    case 0xC3: snprintf(buf, buf_len, "clr   C"); break;
    case 0xD0: snprintf(buf, buf_len, "pop   0x%02X", b1); break;
    case 0xD2: snprintf(buf, buf_len, "setb  0x%02X", b1); break;
    case 0xD3: snprintf(buf, buf_len, "setb  C"); break;
    case 0xE0: snprintf(buf, buf_len, "movx  A, @DPTR"); break;
    case 0xE2: case 0xE3: snprintf(buf, buf_len, "movx  A, @R%d", op & 1); break;
    case 0xF0: snprintf(buf, buf_len, "movx  @DPTR, A"); break;
    case 0xF2: case 0xF3: snprintf(buf, buf_len, "movx  @R%d, A", op & 1); break;
    default: snprintf(buf, buf_len, "db    0x%02X", op); break;
    }
}
