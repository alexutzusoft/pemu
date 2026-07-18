#include "mos6510.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_D 0x08
#define FLAG_B 0x10
#define FLAG_U 0x20
#define FLAG_V 0x40
#define FLAG_N 0x80

typedef struct MOS6510_CPU {
    uint8_t ram[65536];
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint8_t p;
    uint16_t pc;
    uint32_t ticks;
    int halted;

    // On-chip I/O port ($0000 = data direction register, $0001 = port register)
    uint8_t port_ddr;
    uint8_t port_data;
} MOS6510_CPU;

#define SET_FLAG_C(cond) do { if (cond) cpu->p |= FLAG_C; else cpu->p &= ~FLAG_C; } while(0)
#define SET_FLAG_Z(val) do { if ((val) == 0) cpu->p |= FLAG_Z; else cpu->p &= ~FLAG_Z; } while(0)
#define SET_FLAG_N(val) do { if ((val) & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N; } while(0)
#define SET_FLAG_V(cond) do { if (cond) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V; } while(0)
#define GET_FLAG(flag) ((cpu->p & (flag)) ? 1 : 0)

// --- Addressing modes ---
enum {
    M_IMP, M_ACC, M_IMM, M_ZP, M_ZPX, M_ZPY,
    M_ABS, M_ABX, M_ABY, M_IND, M_IZX, M_IZY, M_REL
};

// --- Operations (documented + stable undocumented) ---
enum {
    OP_ADC, OP_AND, OP_ASL, OP_BCC, OP_BCS, OP_BEQ, OP_BIT, OP_BMI,
    OP_BNE, OP_BPL, OP_BRK, OP_BVC, OP_BVS, OP_CLC, OP_CLD, OP_CLI,
    OP_CLV, OP_CMP, OP_CPX, OP_CPY, OP_DEC, OP_DEX, OP_DEY, OP_EOR,
    OP_INC, OP_INX, OP_INY, OP_JMP, OP_JSR, OP_LDA, OP_LDX, OP_LDY,
    OP_LSR, OP_NOP, OP_ORA, OP_PHA, OP_PHP, OP_PLA, OP_PLP, OP_ROL,
    OP_ROR, OP_RTI, OP_RTS, OP_SBC, OP_SEC, OP_SED, OP_SEI, OP_STA,
    OP_STX, OP_STY, OP_TAX, OP_TAY, OP_TSX, OP_TXA, OP_TXS, OP_TYA,
    OP_LAX, OP_SAX, OP_DCP, OP_ISC, OP_SLO, OP_RLA, OP_SRE, OP_RRA,
    OP_ANC, OP_ALR, OP_ARR, OP_SBX, OP_JAM,
    OP_COUNT
};

static const char *g_mnemonics[OP_COUNT] = {
    "adc", "and", "asl", "bcc", "bcs", "beq", "bit", "bmi",
    "bne", "bpl", "brk", "bvc", "bvs", "clc", "cld", "cli",
    "clv", "cmp", "cpx", "cpy", "dec", "dex", "dey", "eor",
    "inc", "inx", "iny", "jmp", "jsr", "lda", "ldx", "ldy",
    "lsr", "nop", "ora", "pha", "php", "pla", "plp", "rol",
    "ror", "rti", "rts", "sbc", "sec", "sed", "sei", "sta",
    "stx", "sty", "tax", "tay", "tsx", "txa", "txs", "tya",
    "lax", "sax", "dcp", "isc", "slo", "rla", "sre", "rra",
    "anc", "alr", "arr", "axs", "jam"
};

typedef struct OpEntry {
    uint8_t op;
    uint8_t mode;
    uint8_t cycles;
} OpEntry;

static OpEntry g_optab[256];
static int g_optab_ready = 0;

static void set_op(uint8_t code, uint8_t op, uint8_t mode, uint8_t cycles) {
    g_optab[code].op = op;
    g_optab[code].mode = mode;
    g_optab[code].cycles = cycles;
}

static void build_optab(void) {
    int i;
    if (g_optab_ready) return;

    // Default: JAM (halts) — the true JAM codes are set explicitly below,
    // and every remaining slot gets an explicit entry as well.
    for (i = 0; i < 256; ++i) set_op((uint8_t)i, OP_JAM, M_IMP, 2);

    // ADC
    set_op(0x69, OP_ADC, M_IMM, 2); set_op(0x65, OP_ADC, M_ZP, 3);
    set_op(0x75, OP_ADC, M_ZPX, 4); set_op(0x6D, OP_ADC, M_ABS, 4);
    set_op(0x7D, OP_ADC, M_ABX, 4); set_op(0x79, OP_ADC, M_ABY, 4);
    set_op(0x61, OP_ADC, M_IZX, 6); set_op(0x71, OP_ADC, M_IZY, 5);
    // AND
    set_op(0x29, OP_AND, M_IMM, 2); set_op(0x25, OP_AND, M_ZP, 3);
    set_op(0x35, OP_AND, M_ZPX, 4); set_op(0x2D, OP_AND, M_ABS, 4);
    set_op(0x3D, OP_AND, M_ABX, 4); set_op(0x39, OP_AND, M_ABY, 4);
    set_op(0x21, OP_AND, M_IZX, 6); set_op(0x31, OP_AND, M_IZY, 5);
    // ASL
    set_op(0x0A, OP_ASL, M_ACC, 2); set_op(0x06, OP_ASL, M_ZP, 5);
    set_op(0x16, OP_ASL, M_ZPX, 6); set_op(0x0E, OP_ASL, M_ABS, 6);
    set_op(0x1E, OP_ASL, M_ABX, 7);
    // Branches
    set_op(0x90, OP_BCC, M_REL, 2); set_op(0xB0, OP_BCS, M_REL, 2);
    set_op(0xF0, OP_BEQ, M_REL, 2); set_op(0x30, OP_BMI, M_REL, 2);
    set_op(0xD0, OP_BNE, M_REL, 2); set_op(0x10, OP_BPL, M_REL, 2);
    set_op(0x50, OP_BVC, M_REL, 2); set_op(0x70, OP_BVS, M_REL, 2);
    // BIT
    set_op(0x24, OP_BIT, M_ZP, 3); set_op(0x2C, OP_BIT, M_ABS, 4);
    // BRK
    set_op(0x00, OP_BRK, M_IMP, 7);
    // Flag ops
    set_op(0x18, OP_CLC, M_IMP, 2); set_op(0xD8, OP_CLD, M_IMP, 2);
    set_op(0x58, OP_CLI, M_IMP, 2); set_op(0xB8, OP_CLV, M_IMP, 2);
    set_op(0x38, OP_SEC, M_IMP, 2); set_op(0xF8, OP_SED, M_IMP, 2);
    set_op(0x78, OP_SEI, M_IMP, 2);
    // CMP
    set_op(0xC9, OP_CMP, M_IMM, 2); set_op(0xC5, OP_CMP, M_ZP, 3);
    set_op(0xD5, OP_CMP, M_ZPX, 4); set_op(0xCD, OP_CMP, M_ABS, 4);
    set_op(0xDD, OP_CMP, M_ABX, 4); set_op(0xD9, OP_CMP, M_ABY, 4);
    set_op(0xC1, OP_CMP, M_IZX, 6); set_op(0xD1, OP_CMP, M_IZY, 5);
    // CPX / CPY
    set_op(0xE0, OP_CPX, M_IMM, 2); set_op(0xE4, OP_CPX, M_ZP, 3);
    set_op(0xEC, OP_CPX, M_ABS, 4);
    set_op(0xC0, OP_CPY, M_IMM, 2); set_op(0xC4, OP_CPY, M_ZP, 3);
    set_op(0xCC, OP_CPY, M_ABS, 4);
    // DEC / DEX / DEY
    set_op(0xC6, OP_DEC, M_ZP, 5); set_op(0xD6, OP_DEC, M_ZPX, 6);
    set_op(0xCE, OP_DEC, M_ABS, 6); set_op(0xDE, OP_DEC, M_ABX, 7);
    set_op(0xCA, OP_DEX, M_IMP, 2); set_op(0x88, OP_DEY, M_IMP, 2);
    // EOR
    set_op(0x49, OP_EOR, M_IMM, 2); set_op(0x45, OP_EOR, M_ZP, 3);
    set_op(0x55, OP_EOR, M_ZPX, 4); set_op(0x4D, OP_EOR, M_ABS, 4);
    set_op(0x5D, OP_EOR, M_ABX, 4); set_op(0x59, OP_EOR, M_ABY, 4);
    set_op(0x41, OP_EOR, M_IZX, 6); set_op(0x51, OP_EOR, M_IZY, 5);
    // INC / INX / INY
    set_op(0xE6, OP_INC, M_ZP, 5); set_op(0xF6, OP_INC, M_ZPX, 6);
    set_op(0xEE, OP_INC, M_ABS, 6); set_op(0xFE, OP_INC, M_ABX, 7);
    set_op(0xE8, OP_INX, M_IMP, 2); set_op(0xC8, OP_INY, M_IMP, 2);
    // JMP / JSR
    set_op(0x4C, OP_JMP, M_ABS, 3); set_op(0x6C, OP_JMP, M_IND, 5);
    set_op(0x20, OP_JSR, M_ABS, 6);
    // LDA
    set_op(0xA9, OP_LDA, M_IMM, 2); set_op(0xA5, OP_LDA, M_ZP, 3);
    set_op(0xB5, OP_LDA, M_ZPX, 4); set_op(0xAD, OP_LDA, M_ABS, 4);
    set_op(0xBD, OP_LDA, M_ABX, 4); set_op(0xB9, OP_LDA, M_ABY, 4);
    set_op(0xA1, OP_LDA, M_IZX, 6); set_op(0xB1, OP_LDA, M_IZY, 5);
    // LDX
    set_op(0xA2, OP_LDX, M_IMM, 2); set_op(0xA6, OP_LDX, M_ZP, 3);
    set_op(0xB6, OP_LDX, M_ZPY, 4); set_op(0xAE, OP_LDX, M_ABS, 4);
    set_op(0xBE, OP_LDX, M_ABY, 4);
    // LDY
    set_op(0xA0, OP_LDY, M_IMM, 2); set_op(0xA4, OP_LDY, M_ZP, 3);
    set_op(0xB4, OP_LDY, M_ZPX, 4); set_op(0xAC, OP_LDY, M_ABS, 4);
    set_op(0xBC, OP_LDY, M_ABX, 4);
    // LSR
    set_op(0x4A, OP_LSR, M_ACC, 2); set_op(0x46, OP_LSR, M_ZP, 5);
    set_op(0x56, OP_LSR, M_ZPX, 6); set_op(0x4E, OP_LSR, M_ABS, 6);
    set_op(0x5E, OP_LSR, M_ABX, 7);
    // NOP (documented)
    set_op(0xEA, OP_NOP, M_IMP, 2);
    // ORA
    set_op(0x09, OP_ORA, M_IMM, 2); set_op(0x05, OP_ORA, M_ZP, 3);
    set_op(0x15, OP_ORA, M_ZPX, 4); set_op(0x0D, OP_ORA, M_ABS, 4);
    set_op(0x1D, OP_ORA, M_ABX, 4); set_op(0x19, OP_ORA, M_ABY, 4);
    set_op(0x01, OP_ORA, M_IZX, 6); set_op(0x11, OP_ORA, M_IZY, 5);
    // Stack
    set_op(0x48, OP_PHA, M_IMP, 3); set_op(0x08, OP_PHP, M_IMP, 3);
    set_op(0x68, OP_PLA, M_IMP, 4); set_op(0x28, OP_PLP, M_IMP, 4);
    // ROL / ROR
    set_op(0x2A, OP_ROL, M_ACC, 2); set_op(0x26, OP_ROL, M_ZP, 5);
    set_op(0x36, OP_ROL, M_ZPX, 6); set_op(0x2E, OP_ROL, M_ABS, 6);
    set_op(0x3E, OP_ROL, M_ABX, 7);
    set_op(0x6A, OP_ROR, M_ACC, 2); set_op(0x66, OP_ROR, M_ZP, 5);
    set_op(0x76, OP_ROR, M_ZPX, 6); set_op(0x6E, OP_ROR, M_ABS, 6);
    set_op(0x7E, OP_ROR, M_ABX, 7);
    // RTI / RTS
    set_op(0x40, OP_RTI, M_IMP, 6); set_op(0x60, OP_RTS, M_IMP, 6);
    // SBC
    set_op(0xE9, OP_SBC, M_IMM, 2); set_op(0xE5, OP_SBC, M_ZP, 3);
    set_op(0xF5, OP_SBC, M_ZPX, 4); set_op(0xED, OP_SBC, M_ABS, 4);
    set_op(0xFD, OP_SBC, M_ABX, 4); set_op(0xF9, OP_SBC, M_ABY, 4);
    set_op(0xE1, OP_SBC, M_IZX, 6); set_op(0xF1, OP_SBC, M_IZY, 5);
    set_op(0xEB, OP_SBC, M_IMM, 2); // undocumented SBC immediate
    // STA
    set_op(0x85, OP_STA, M_ZP, 3); set_op(0x95, OP_STA, M_ZPX, 4);
    set_op(0x8D, OP_STA, M_ABS, 4); set_op(0x9D, OP_STA, M_ABX, 5);
    set_op(0x99, OP_STA, M_ABY, 5); set_op(0x81, OP_STA, M_IZX, 6);
    set_op(0x91, OP_STA, M_IZY, 6);
    // STX / STY
    set_op(0x86, OP_STX, M_ZP, 3); set_op(0x96, OP_STX, M_ZPY, 4);
    set_op(0x8E, OP_STX, M_ABS, 4);
    set_op(0x84, OP_STY, M_ZP, 3); set_op(0x94, OP_STY, M_ZPX, 4);
    set_op(0x8C, OP_STY, M_ABS, 4);
    // Transfers
    set_op(0xAA, OP_TAX, M_IMP, 2); set_op(0xA8, OP_TAY, M_IMP, 2);
    set_op(0xBA, OP_TSX, M_IMP, 2); set_op(0x8A, OP_TXA, M_IMP, 2);
    set_op(0x9A, OP_TXS, M_IMP, 2); set_op(0x98, OP_TYA, M_IMP, 2);

    // --- Stable undocumented opcodes ---
    // LAX
    set_op(0xA7, OP_LAX, M_ZP, 3); set_op(0xB7, OP_LAX, M_ZPY, 4);
    set_op(0xAF, OP_LAX, M_ABS, 4); set_op(0xBF, OP_LAX, M_ABY, 4);
    set_op(0xA3, OP_LAX, M_IZX, 6); set_op(0xB3, OP_LAX, M_IZY, 5);
    // SAX
    set_op(0x87, OP_SAX, M_ZP, 3); set_op(0x97, OP_SAX, M_ZPY, 4);
    set_op(0x8F, OP_SAX, M_ABS, 4); set_op(0x83, OP_SAX, M_IZX, 6);
    // SLO
    set_op(0x07, OP_SLO, M_ZP, 5); set_op(0x17, OP_SLO, M_ZPX, 6);
    set_op(0x0F, OP_SLO, M_ABS, 6); set_op(0x1F, OP_SLO, M_ABX, 7);
    set_op(0x1B, OP_SLO, M_ABY, 7); set_op(0x03, OP_SLO, M_IZX, 8);
    set_op(0x13, OP_SLO, M_IZY, 8);
    // RLA
    set_op(0x27, OP_RLA, M_ZP, 5); set_op(0x37, OP_RLA, M_ZPX, 6);
    set_op(0x2F, OP_RLA, M_ABS, 6); set_op(0x3F, OP_RLA, M_ABX, 7);
    set_op(0x3B, OP_RLA, M_ABY, 7); set_op(0x23, OP_RLA, M_IZX, 8);
    set_op(0x33, OP_RLA, M_IZY, 8);
    // SRE
    set_op(0x47, OP_SRE, M_ZP, 5); set_op(0x57, OP_SRE, M_ZPX, 6);
    set_op(0x4F, OP_SRE, M_ABS, 6); set_op(0x5F, OP_SRE, M_ABX, 7);
    set_op(0x5B, OP_SRE, M_ABY, 7); set_op(0x43, OP_SRE, M_IZX, 8);
    set_op(0x53, OP_SRE, M_IZY, 8);
    // RRA
    set_op(0x67, OP_RRA, M_ZP, 5); set_op(0x77, OP_RRA, M_ZPX, 6);
    set_op(0x6F, OP_RRA, M_ABS, 6); set_op(0x7F, OP_RRA, M_ABX, 7);
    set_op(0x7B, OP_RRA, M_ABY, 7); set_op(0x63, OP_RRA, M_IZX, 8);
    set_op(0x73, OP_RRA, M_IZY, 8);
    // DCP
    set_op(0xC7, OP_DCP, M_ZP, 5); set_op(0xD7, OP_DCP, M_ZPX, 6);
    set_op(0xCF, OP_DCP, M_ABS, 6); set_op(0xDF, OP_DCP, M_ABX, 7);
    set_op(0xDB, OP_DCP, M_ABY, 7); set_op(0xC3, OP_DCP, M_IZX, 8);
    set_op(0xD3, OP_DCP, M_IZY, 8);
    // ISC
    set_op(0xE7, OP_ISC, M_ZP, 5); set_op(0xF7, OP_ISC, M_ZPX, 6);
    set_op(0xEF, OP_ISC, M_ABS, 6); set_op(0xFF, OP_ISC, M_ABX, 7);
    set_op(0xFB, OP_ISC, M_ABY, 7); set_op(0xE3, OP_ISC, M_IZX, 8);
    set_op(0xF3, OP_ISC, M_IZY, 8);
    // Immediate-mode illegals
    set_op(0x0B, OP_ANC, M_IMM, 2); set_op(0x2B, OP_ANC, M_IMM, 2);
    set_op(0x4B, OP_ALR, M_IMM, 2); set_op(0x6B, OP_ARR, M_IMM, 2);
    set_op(0xCB, OP_SBX, M_IMM, 2);
    // Undocumented NOPs (multi-byte variants consume their operand)
    set_op(0x1A, OP_NOP, M_IMP, 2); set_op(0x3A, OP_NOP, M_IMP, 2);
    set_op(0x5A, OP_NOP, M_IMP, 2); set_op(0x7A, OP_NOP, M_IMP, 2);
    set_op(0xDA, OP_NOP, M_IMP, 2); set_op(0xFA, OP_NOP, M_IMP, 2);
    set_op(0x80, OP_NOP, M_IMM, 2); set_op(0x82, OP_NOP, M_IMM, 2);
    set_op(0x89, OP_NOP, M_IMM, 2); set_op(0xC2, OP_NOP, M_IMM, 2);
    set_op(0xE2, OP_NOP, M_IMM, 2);
    set_op(0x04, OP_NOP, M_ZP, 3); set_op(0x44, OP_NOP, M_ZP, 3);
    set_op(0x64, OP_NOP, M_ZP, 3);
    set_op(0x14, OP_NOP, M_ZPX, 4); set_op(0x34, OP_NOP, M_ZPX, 4);
    set_op(0x54, OP_NOP, M_ZPX, 4); set_op(0x74, OP_NOP, M_ZPX, 4);
    set_op(0xD4, OP_NOP, M_ZPX, 4); set_op(0xF4, OP_NOP, M_ZPX, 4);
    set_op(0x0C, OP_NOP, M_ABS, 4);
    set_op(0x1C, OP_NOP, M_ABX, 4); set_op(0x3C, OP_NOP, M_ABX, 4);
    set_op(0x5C, OP_NOP, M_ABX, 4); set_op(0x7C, OP_NOP, M_ABX, 4);
    set_op(0xDC, OP_NOP, M_ABX, 4); set_op(0xFC, OP_NOP, M_ABX, 4);
    // Unstable illegals (ANE, LXA, LAS, TAS, SHA, SHX, SHY) — treated as
    // NOPs of the correct length so execution stays in sync.
    set_op(0x8B, OP_NOP, M_IMM, 2); set_op(0xAB, OP_NOP, M_IMM, 2);
    set_op(0xBB, OP_NOP, M_ABY, 4); set_op(0x9B, OP_NOP, M_ABY, 5);
    set_op(0x9C, OP_NOP, M_ABX, 5); set_op(0x9E, OP_NOP, M_ABY, 5);
    set_op(0x9F, OP_NOP, M_ABY, 5); set_op(0x93, OP_NOP, M_IZY, 6);
    // JAM / KIL — processor halts
    set_op(0x02, OP_JAM, M_IMP, 2); set_op(0x12, OP_JAM, M_IMP, 2);
    set_op(0x22, OP_JAM, M_IMP, 2); set_op(0x32, OP_JAM, M_IMP, 2);
    set_op(0x42, OP_JAM, M_IMP, 2); set_op(0x52, OP_JAM, M_IMP, 2);
    set_op(0x62, OP_JAM, M_IMP, 2); set_op(0x72, OP_JAM, M_IMP, 2);
    set_op(0x92, OP_JAM, M_IMP, 2); set_op(0xB2, OP_JAM, M_IMP, 2);
    set_op(0xD2, OP_JAM, M_IMP, 2); set_op(0xF2, OP_JAM, M_IMP, 2);

    g_optab_ready = 1;
}

// --- Bus access (routes $0000/$0001 to the on-chip port) ---
static uint8_t bus_read(MOS6510_CPU *cpu, uint16_t addr) {
    if (addr == 0x0000) {
        return cpu->port_ddr;
    }
    if (addr == 0x0001) {
        // Output bits read back the data register; input bits (external
        // pins are pulled up on the C64) read as 1s.
        return (uint8_t)((cpu->port_data & cpu->port_ddr) | (uint8_t)~cpu->port_ddr);
    }
    return cpu->ram[addr];
}

static void bus_write(MOS6510_CPU *cpu, uint16_t addr, uint8_t val) {
    if (addr == 0x0000) {
        cpu->port_ddr = val;
    } else if (addr == 0x0001) {
        cpu->port_data = val;
    } else {
        cpu->ram[addr] = val;
    }
}

static void push_byte(MOS6510_CPU *cpu, uint8_t val) {
    bus_write(cpu, (uint16_t)(0x0100 + cpu->sp), val);
    cpu->sp--;
}

static uint8_t pop_byte(MOS6510_CPU *cpu) {
    cpu->sp++;
    return bus_read(cpu, (uint16_t)(0x0100 + cpu->sp));
}

// Resolves the effective address for every mode that has one.
static uint16_t operand_addr(MOS6510_CPU *cpu, uint8_t mode) {
    switch (mode) {
        case M_IMM:
            return cpu->pc++;
        case M_ZP:
            return bus_read(cpu, cpu->pc++);
        case M_ZPX:
            return (uint8_t)(bus_read(cpu, cpu->pc++) + cpu->x);
        case M_ZPY:
            return (uint8_t)(bus_read(cpu, cpu->pc++) + cpu->y);
        case M_ABS: {
            uint16_t addr = bus_read(cpu, cpu->pc) | ((uint16_t)bus_read(cpu, (uint16_t)(cpu->pc + 1)) << 8);
            cpu->pc += 2;
            return addr;
        }
        case M_ABX: {
            uint16_t addr = bus_read(cpu, cpu->pc) | ((uint16_t)bus_read(cpu, (uint16_t)(cpu->pc + 1)) << 8);
            cpu->pc += 2;
            return (uint16_t)(addr + cpu->x);
        }
        case M_ABY: {
            uint16_t addr = bus_read(cpu, cpu->pc) | ((uint16_t)bus_read(cpu, (uint16_t)(cpu->pc + 1)) << 8);
            cpu->pc += 2;
            return (uint16_t)(addr + cpu->y);
        }
        case M_IND: {
            uint16_t ptr = bus_read(cpu, cpu->pc) | ((uint16_t)bus_read(cpu, (uint16_t)(cpu->pc + 1)) << 8);
            cpu->pc += 2;
            // NMOS page-wrap bug: ($xxFF) fetches the high byte from $xx00
            if ((ptr & 0x00FF) == 0x00FF) {
                return bus_read(cpu, ptr) | ((uint16_t)bus_read(cpu, (uint16_t)(ptr & 0xFF00)) << 8);
            }
            return bus_read(cpu, ptr) | ((uint16_t)bus_read(cpu, (uint16_t)(ptr + 1)) << 8);
        }
        case M_IZX: {
            uint8_t zp = (uint8_t)(bus_read(cpu, cpu->pc++) + cpu->x);
            return bus_read(cpu, zp) | ((uint16_t)bus_read(cpu, (uint8_t)(zp + 1)) << 8);
        }
        case M_IZY: {
            uint8_t zp = bus_read(cpu, cpu->pc++);
            uint16_t base = bus_read(cpu, zp) | ((uint16_t)bus_read(cpu, (uint8_t)(zp + 1)) << 8);
            return (uint16_t)(base + cpu->y);
        }
        default:
            return 0;
    }
}

// --- ALU helpers ---
static void do_adc(MOS6510_CPU *cpu, uint8_t val) {
    uint16_t bin = (uint16_t)(cpu->a + val + GET_FLAG(FLAG_C));
    if (cpu->p & FLAG_D) {
        uint16_t lo = (uint16_t)((cpu->a & 0x0F) + (val & 0x0F) + GET_FLAG(FLAG_C));
        uint16_t hi = (uint16_t)((cpu->a >> 4) + (val >> 4));
        if (lo > 9) { lo = (uint16_t)(lo + 6); hi++; }
        SET_FLAG_Z(bin & 0xFF); // NMOS: Z comes from the binary result
        SET_FLAG_N((hi << 4) & 0x80);
        SET_FLAG_V((~(cpu->a ^ val) & (cpu->a ^ (uint8_t)(hi << 4))) & 0x80);
        if (hi > 9) hi = (uint16_t)(hi + 6);
        SET_FLAG_C(hi > 15);
        cpu->a = (uint8_t)((hi << 4) | (lo & 0x0F));
    } else {
        SET_FLAG_C(bin > 0xFF);
        SET_FLAG_V((~(cpu->a ^ val) & (cpu->a ^ bin)) & 0x80);
        cpu->a = (uint8_t)bin;
        SET_FLAG_Z(cpu->a);
        SET_FLAG_N(cpu->a);
    }
}

static void do_sbc(MOS6510_CPU *cpu, uint8_t val) {
    int borrow = 1 - GET_FLAG(FLAG_C);
    uint16_t diff = (uint16_t)(cpu->a - val - borrow);
    uint8_t result = (uint8_t)diff;
    SET_FLAG_V((cpu->a ^ val) & (cpu->a ^ result) & 0x80);
    SET_FLAG_C(!(diff & 0x100));
    SET_FLAG_Z(result);
    SET_FLAG_N(result);
    if (cpu->p & FLAG_D) {
        int lo = (cpu->a & 0x0F) - (val & 0x0F) - borrow;
        int hi = (cpu->a >> 4) - (val >> 4);
        if (lo & 0x10) { lo -= 6; hi--; }
        if (hi & 0x10) hi -= 6;
        cpu->a = (uint8_t)(((hi & 0x0F) << 4) | (lo & 0x0F));
    } else {
        cpu->a = result;
    }
}

static void do_compare(MOS6510_CPU *cpu, uint8_t reg, uint8_t val) {
    uint8_t diff = (uint8_t)(reg - val);
    SET_FLAG_C(reg >= val);
    SET_FLAG_Z(diff);
    SET_FLAG_N(diff);
}

static uint8_t do_asl(MOS6510_CPU *cpu, uint8_t val) {
    SET_FLAG_C(val & 0x80);
    val = (uint8_t)(val << 1);
    SET_FLAG_Z(val);
    SET_FLAG_N(val);
    return val;
}

static uint8_t do_lsr(MOS6510_CPU *cpu, uint8_t val) {
    SET_FLAG_C(val & 0x01);
    val >>= 1;
    SET_FLAG_Z(val);
    SET_FLAG_N(val);
    return val;
}

static uint8_t do_rol(MOS6510_CPU *cpu, uint8_t val) {
    uint8_t old_c = (uint8_t)GET_FLAG(FLAG_C);
    SET_FLAG_C(val & 0x80);
    val = (uint8_t)((val << 1) | old_c);
    SET_FLAG_Z(val);
    SET_FLAG_N(val);
    return val;
}

static uint8_t do_ror(MOS6510_CPU *cpu, uint8_t val) {
    uint8_t old_c = (uint8_t)GET_FLAG(FLAG_C);
    SET_FLAG_C(val & 0x01);
    val = (uint8_t)((val >> 1) | (old_c << 7));
    SET_FLAG_Z(val);
    SET_FLAG_N(val);
    return val;
}

static void do_branch(MOS6510_CPU *cpu, int cond) {
    int8_t offset = (int8_t)bus_read(cpu, cpu->pc++);
    if (cond) {
        cpu->ticks++;
        cpu->pc = (uint16_t)(cpu->pc + offset);
    }
}

// --- Public interface ---
void* mos6510_create(void) {
    MOS6510_CPU *cpu = (MOS6510_CPU*)calloc(1, sizeof(MOS6510_CPU));
    build_optab();
    return cpu;
}

void mos6510_destroy(void *context) {
    free(context);
}

int mos6510_init(void *context) {
    MOS6510_CPU *cpu = (MOS6510_CPU*)context;
    if (!cpu) return -1;

    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->p = FLAG_U | FLAG_I;
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;

    // C64 KERNAL reset values for the processor port
    cpu->port_ddr = 0x2F;
    cpu->port_data = 0x37;

    return 0;
}

int mos6510_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    MOS6510_CPU *cpu = (MOS6510_CPU*)context;
    if (!cpu || !data) return -1;

    if (address >= 65536) return -2;
    {
        size_t copy_len = size;
        if (address + size > 65536) {
            copy_len = 65536 - address;
        }
        memcpy(cpu->ram + address, data, copy_len);
    }
    cpu->pc = (uint16_t)address;

    return 0;
}

int mos6510_step(void *context) {
    MOS6510_CPU *cpu = (MOS6510_CPU*)context;
    uint16_t instr_pc;
    uint16_t addr = 0;
    uint8_t opcode;
    const OpEntry *ent;

    if (!cpu) return -1;
    if (cpu->halted) return 1;

    instr_pc = cpu->pc;
    opcode = bus_read(cpu, cpu->pc);
    ent = &g_optab[opcode];

    if (ent->op == OP_JAM) {
        // JAM / KIL: the processor locks up, PC does not advance
        cpu->halted = 1;
        return 1;
    }

    cpu->pc++;
    cpu->ticks += ent->cycles;

    if (ent->mode != M_IMP && ent->mode != M_ACC && ent->mode != M_REL) {
        addr = operand_addr(cpu, ent->mode);
    }

    switch (ent->op) {
        case OP_LDA:
            cpu->a = bus_read(cpu, addr);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_LDX:
            cpu->x = bus_read(cpu, addr);
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case OP_LDY:
            cpu->y = bus_read(cpu, addr);
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case OP_STA:
            bus_write(cpu, addr, cpu->a);
            break;
        case OP_STX:
            bus_write(cpu, addr, cpu->x);
            break;
        case OP_STY:
            bus_write(cpu, addr, cpu->y);
            break;
        case OP_ORA:
            cpu->a |= bus_read(cpu, addr);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_AND:
            cpu->a &= bus_read(cpu, addr);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_EOR:
            cpu->a ^= bus_read(cpu, addr);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_ADC:
            do_adc(cpu, bus_read(cpu, addr));
            break;
        case OP_SBC:
            do_sbc(cpu, bus_read(cpu, addr));
            break;
        case OP_CMP:
            do_compare(cpu, cpu->a, bus_read(cpu, addr));
            break;
        case OP_CPX:
            do_compare(cpu, cpu->x, bus_read(cpu, addr));
            break;
        case OP_CPY:
            do_compare(cpu, cpu->y, bus_read(cpu, addr));
            break;
        case OP_BIT: {
            uint8_t val = bus_read(cpu, addr);
            SET_FLAG_Z(cpu->a & val);
            SET_FLAG_N(val);
            SET_FLAG_V(val & 0x40);
            break;
        }
        case OP_ASL:
            if (ent->mode == M_ACC) {
                cpu->a = do_asl(cpu, cpu->a);
            } else {
                bus_write(cpu, addr, do_asl(cpu, bus_read(cpu, addr)));
            }
            break;
        case OP_LSR:
            if (ent->mode == M_ACC) {
                cpu->a = do_lsr(cpu, cpu->a);
            } else {
                bus_write(cpu, addr, do_lsr(cpu, bus_read(cpu, addr)));
            }
            break;
        case OP_ROL:
            if (ent->mode == M_ACC) {
                cpu->a = do_rol(cpu, cpu->a);
            } else {
                bus_write(cpu, addr, do_rol(cpu, bus_read(cpu, addr)));
            }
            break;
        case OP_ROR:
            if (ent->mode == M_ACC) {
                cpu->a = do_ror(cpu, cpu->a);
            } else {
                bus_write(cpu, addr, do_ror(cpu, bus_read(cpu, addr)));
            }
            break;
        case OP_INC: {
            uint8_t val = (uint8_t)(bus_read(cpu, addr) + 1);
            bus_write(cpu, addr, val);
            SET_FLAG_Z(val);
            SET_FLAG_N(val);
            break;
        }
        case OP_DEC: {
            uint8_t val = (uint8_t)(bus_read(cpu, addr) - 1);
            bus_write(cpu, addr, val);
            SET_FLAG_Z(val);
            SET_FLAG_N(val);
            break;
        }
        case OP_INX:
            cpu->x++;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case OP_INY:
            cpu->y++;
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case OP_DEX:
            cpu->x--;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case OP_DEY:
            cpu->y--;
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case OP_TAX:
            cpu->x = cpu->a;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case OP_TAY:
            cpu->y = cpu->a;
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case OP_TXA:
            cpu->a = cpu->x;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_TYA:
            cpu->a = cpu->y;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_TSX:
            cpu->x = cpu->sp;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case OP_TXS:
            cpu->sp = cpu->x;
            break;
        case OP_PHA:
            push_byte(cpu, cpu->a);
            break;
        case OP_PHP:
            push_byte(cpu, (uint8_t)(cpu->p | FLAG_B | FLAG_U));
            break;
        case OP_PLA:
            cpu->a = pop_byte(cpu);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_PLP:
            cpu->p = (uint8_t)((pop_byte(cpu) | FLAG_U) & (uint8_t)~FLAG_B);
            break;
        case OP_JMP:
            cpu->pc = addr;
            break;
        case OP_JSR: {
            uint16_t ret_pc = (uint16_t)(cpu->pc - 1);
            push_byte(cpu, (uint8_t)(ret_pc >> 8));
            push_byte(cpu, (uint8_t)(ret_pc & 0xFF));
            cpu->pc = addr;
            break;
        }
        case OP_RTS: {
            uint8_t low = pop_byte(cpu);
            uint8_t high = pop_byte(cpu);
            cpu->pc = (uint16_t)((((uint16_t)high << 8) | low) + 1);
            break;
        }
        case OP_RTI: {
            uint8_t low, high;
            cpu->p = (uint8_t)((pop_byte(cpu) | FLAG_U) & (uint8_t)~FLAG_B);
            low = pop_byte(cpu);
            high = pop_byte(cpu);
            cpu->pc = (uint16_t)(((uint16_t)high << 8) | low);
            break;
        }
        case OP_BCC: do_branch(cpu, !GET_FLAG(FLAG_C)); break;
        case OP_BCS: do_branch(cpu, GET_FLAG(FLAG_C)); break;
        case OP_BEQ: do_branch(cpu, GET_FLAG(FLAG_Z)); break;
        case OP_BNE: do_branch(cpu, !GET_FLAG(FLAG_Z)); break;
        case OP_BMI: do_branch(cpu, GET_FLAG(FLAG_N)); break;
        case OP_BPL: do_branch(cpu, !GET_FLAG(FLAG_N)); break;
        case OP_BVC: do_branch(cpu, !GET_FLAG(FLAG_V)); break;
        case OP_BVS: do_branch(cpu, GET_FLAG(FLAG_V)); break;
        case OP_CLC: cpu->p &= (uint8_t)~FLAG_C; break;
        case OP_SEC: cpu->p |= FLAG_C; break;
        case OP_CLI: cpu->p &= (uint8_t)~FLAG_I; break;
        case OP_SEI: cpu->p |= FLAG_I; break;
        case OP_CLD: cpu->p &= (uint8_t)~FLAG_D; break;
        case OP_SED: cpu->p |= FLAG_D; break;
        case OP_CLV: cpu->p &= (uint8_t)~FLAG_V; break;
        case OP_NOP:
            break;
        case OP_BRK:
            // Standalone core convention: BRK halts execution
            cpu->halted = 1;
            return 1;

        // --- Stable undocumented opcodes ---
        case OP_LAX:
            cpu->a = bus_read(cpu, addr);
            cpu->x = cpu->a;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case OP_SAX:
            bus_write(cpu, addr, (uint8_t)(cpu->a & cpu->x));
            break;
        case OP_DCP: {
            uint8_t val = (uint8_t)(bus_read(cpu, addr) - 1);
            bus_write(cpu, addr, val);
            do_compare(cpu, cpu->a, val);
            break;
        }
        case OP_ISC: {
            uint8_t val = (uint8_t)(bus_read(cpu, addr) + 1);
            bus_write(cpu, addr, val);
            do_sbc(cpu, val);
            break;
        }
        case OP_SLO: {
            uint8_t val = do_asl(cpu, bus_read(cpu, addr));
            bus_write(cpu, addr, val);
            cpu->a |= val;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        }
        case OP_RLA: {
            uint8_t val = do_rol(cpu, bus_read(cpu, addr));
            bus_write(cpu, addr, val);
            cpu->a &= val;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        }
        case OP_SRE: {
            uint8_t val = do_lsr(cpu, bus_read(cpu, addr));
            bus_write(cpu, addr, val);
            cpu->a ^= val;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        }
        case OP_RRA: {
            uint8_t val = do_ror(cpu, bus_read(cpu, addr));
            bus_write(cpu, addr, val);
            do_adc(cpu, val);
            break;
        }
        case OP_ANC:
            cpu->a &= bus_read(cpu, addr);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            SET_FLAG_C(cpu->a & 0x80);
            break;
        case OP_ALR:
            cpu->a &= bus_read(cpu, addr);
            cpu->a = do_lsr(cpu, cpu->a);
            break;
        case OP_ARR: {
            uint8_t and_val = (uint8_t)(cpu->a & bus_read(cpu, addr));
            cpu->a = (uint8_t)((and_val >> 1) | (GET_FLAG(FLAG_C) << 7));
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            SET_FLAG_C(cpu->a & 0x40);
            SET_FLAG_V(((cpu->a >> 6) ^ (cpu->a >> 5)) & 0x01);
            break;
        }
        case OP_SBX: { // aka AXS
            uint8_t val = bus_read(cpu, addr);
            uint8_t ax = (uint8_t)(cpu->a & cpu->x);
            SET_FLAG_C(ax >= val);
            cpu->x = (uint8_t)(ax - val);
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        }

        default:
            cpu->halted = 1;
            return 1;
    }

    if (cpu->pc == instr_pc) {
        // Tight infinite loop (e.g. jmp *) — treat as program end
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

void mos6510_print_state(void *context) {
    MOS6510_CPU *cpu = (MOS6510_CPU*)context;
    if (!cpu) return;

    printf("MOS 6510 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%02X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Registers: A=0x%02X  X=0x%02X  Y=0x%02X\n", cpu->a, cpu->x, cpu->y);
    printf("  Flags: N=%d  V=%d  U=%d  B=%d  D=%d  I=%d  Z=%d  C=%d\n",
           GET_FLAG(FLAG_N), GET_FLAG(FLAG_V), GET_FLAG(FLAG_U), GET_FLAG(FLAG_B),
           GET_FLAG(FLAG_D), GET_FLAG(FLAG_I), GET_FLAG(FLAG_Z), GET_FLAG(FLAG_C));
    printf("  I/O Port: DDR=0x%02X  DATA=0x%02X  (pins read as 0x%02X)\n",
           cpu->port_ddr, cpu->port_data, bus_read(cpu, 0x0001));
}

void mos6510_get_disassembly(void *context, char *buf, size_t buf_len) {
    MOS6510_CPU *cpu = (MOS6510_CPU*)context;
    const OpEntry *ent;
    const char *mn;
    uint8_t opcode, b1;
    uint16_t w;

    if (!cpu || !buf || buf_len == 0) return;
    build_optab();

    opcode = bus_read(cpu, cpu->pc);
    ent = &g_optab[opcode];
    mn = g_mnemonics[ent->op];
    b1 = bus_read(cpu, (uint16_t)(cpu->pc + 1));
    w = b1 | ((uint16_t)bus_read(cpu, (uint16_t)(cpu->pc + 2)) << 8);

    switch (ent->mode) {
        case M_IMP:
            snprintf(buf, buf_len, "%s", mn);
            break;
        case M_ACC:
            snprintf(buf, buf_len, "%s   a", mn);
            break;
        case M_IMM:
            snprintf(buf, buf_len, "%s   #$%02X", mn, b1);
            break;
        case M_ZP:
            snprintf(buf, buf_len, "%s   $%02X", mn, b1);
            break;
        case M_ZPX:
            snprintf(buf, buf_len, "%s   $%02X,X", mn, b1);
            break;
        case M_ZPY:
            snprintf(buf, buf_len, "%s   $%02X,Y", mn, b1);
            break;
        case M_ABS:
            snprintf(buf, buf_len, "%s   $%04X", mn, w);
            break;
        case M_ABX:
            snprintf(buf, buf_len, "%s   $%04X,X", mn, w);
            break;
        case M_ABY:
            snprintf(buf, buf_len, "%s   $%04X,Y", mn, w);
            break;
        case M_IND:
            snprintf(buf, buf_len, "%s   ($%04X)", mn, w);
            break;
        case M_IZX:
            snprintf(buf, buf_len, "%s   ($%02X,X)", mn, b1);
            break;
        case M_IZY:
            snprintf(buf, buf_len, "%s   ($%02X),Y", mn, b1);
            break;
        case M_REL:
            snprintf(buf, buf_len, "%s   $%04X", mn, (uint16_t)(cpu->pc + 2 + (int8_t)b1));
            break;
        default:
            snprintf(buf, buf_len, "unknown (0x%02X)", opcode);
            break;
    }
}
