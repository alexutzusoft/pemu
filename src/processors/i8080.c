#include "i8080.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB (16-bit address space)

// Register indices as encoded in 8080 opcodes: B,C,D,E,H,L,M,A
enum { R_B = 0, R_C, R_D, R_E, R_H, R_L, R_M, R_A };

typedef struct I8080CPU {
    uint8_t regs[8]; // index R_M (6) is unused storage; memory access is virtual
    uint16_t pc;
    uint16_t sp;

    // Flags
    uint8_t flags_c;  // Carry
    uint8_t flags_z;  // Zero
    uint8_t flags_s;  // Sign
    uint8_t flags_p;  // Parity
    uint8_t flags_ac; // Auxiliary (half) carry

    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
    int interrupts_enabled;
} I8080CPU;

static const char* r_names[] = { "B", "C", "D", "E", "H", "L", "M", "A" };
static const char* rp_names[] = { "B", "D", "H", "SP" };
static const char* c_names[] = { "NZ", "Z", "NC", "C", "PO", "PE", "P", "M" };

static uint8_t calculate_parity(uint8_t val) {
    uint8_t count = 0;
    for (int i = 0; i < 8; ++i) {
        if ((val >> i) & 1) count++;
    }
    return (count % 2 == 0) ? 1 : 0; // parity flag set when count is even
}

static inline void update_flags_zsp(I8080CPU *cpu, uint8_t res) {
    cpu->flags_z = (res == 0) ? 1 : 0;
    cpu->flags_s = (res & 0x80) ? 1 : 0;
    cpu->flags_p = calculate_parity(res);
}

static inline uint16_t hl_addr(I8080CPU *cpu) {
    return ((uint16_t)cpu->regs[R_H] << 8) | cpu->regs[R_L];
}

static inline uint8_t get_reg(I8080CPU *cpu, uint8_t idx) {
    if (idx == R_M) return cpu->memory[hl_addr(cpu)];
    return cpu->regs[idx];
}

static inline void set_reg(I8080CPU *cpu, uint8_t idx, uint8_t val) {
    if (idx == R_M) cpu->memory[hl_addr(cpu)] = val;
    else cpu->regs[idx] = val;
}

// Register pair getter: 0=BC, 1=DE, 2=HL, 3=SP
static inline uint16_t get_rp(I8080CPU *cpu, uint8_t rp) {
    switch (rp) {
        case 0: return ((uint16_t)cpu->regs[R_B] << 8) | cpu->regs[R_C];
        case 1: return ((uint16_t)cpu->regs[R_D] << 8) | cpu->regs[R_E];
        case 2: return hl_addr(cpu);
        case 3: return cpu->sp;
    }
    return 0;
}

static inline void set_rp(I8080CPU *cpu, uint8_t rp, uint16_t val) {
    switch (rp) {
        case 0: cpu->regs[R_B] = val >> 8; cpu->regs[R_C] = val & 0xFF; break;
        case 1: cpu->regs[R_D] = val >> 8; cpu->regs[R_E] = val & 0xFF; break;
        case 2: cpu->regs[R_H] = val >> 8; cpu->regs[R_L] = val & 0xFF; break;
        case 3: cpu->sp = val; break;
    }
}

static inline int check_cond(I8080CPU *cpu, uint8_t cond) {
    switch (cond) {
        case 0: return cpu->flags_z == 0; // NZ
        case 1: return cpu->flags_z == 1; // Z
        case 2: return cpu->flags_c == 0; // NC
        case 3: return cpu->flags_c == 1; // C
        case 4: return cpu->flags_p == 0; // PO (odd)
        case 5: return cpu->flags_p == 1; // PE (even)
        case 6: return cpu->flags_s == 0; // P (positive)
        case 7: return cpu->flags_s == 1; // M (minus)
    }
    return 0;
}

static inline void push16(I8080CPU *cpu, uint16_t val) {
    cpu->sp = (cpu->sp - 1) & 0xFFFF;
    cpu->memory[cpu->sp] = val >> 8;
    cpu->sp = (cpu->sp - 1) & 0xFFFF;
    cpu->memory[cpu->sp] = val & 0xFF;
}

static inline uint16_t pop16(I8080CPU *cpu) {
    uint8_t lo = cpu->memory[cpu->sp];
    cpu->sp = (cpu->sp + 1) & 0xFFFF;
    uint8_t hi = cpu->memory[cpu->sp];
    cpu->sp = (cpu->sp + 1) & 0xFFFF;
    return ((uint16_t)hi << 8) | lo;
}

// Pack flags into the PSW byte (S Z 0 AC 0 P 1 C)
static inline uint8_t get_psw(I8080CPU *cpu) {
    uint8_t psw = 0x02; // bit 1 always 1
    if (cpu->flags_c)  psw |= 0x01;
    if (cpu->flags_p)  psw |= 0x04;
    if (cpu->flags_ac) psw |= 0x10;
    if (cpu->flags_z)  psw |= 0x40;
    if (cpu->flags_s)  psw |= 0x80;
    return psw;
}

static inline void set_psw(I8080CPU *cpu, uint8_t psw) {
    cpu->flags_c  = (psw & 0x01) ? 1 : 0;
    cpu->flags_p  = (psw & 0x04) ? 1 : 0;
    cpu->flags_ac = (psw & 0x10) ? 1 : 0;
    cpu->flags_z  = (psw & 0x40) ? 1 : 0;
    cpu->flags_s  = (psw & 0x80) ? 1 : 0;
}

// Core ALU used by both register (10aaasss) and immediate (11aaa110) forms.
static void alu_op(I8080CPU *cpu, uint8_t alu, uint8_t val) {
    uint8_t acc = cpu->regs[R_A];
    switch (alu) {
        case 0: { // ADD
            uint16_t sum = (uint16_t)acc + val;
            cpu->flags_ac = (((acc & 0x0F) + (val & 0x0F)) > 0x0F) ? 1 : 0;
            cpu->regs[R_A] = sum & 0xFF;
            cpu->flags_c = (sum > 0xFF) ? 1 : 0;
            update_flags_zsp(cpu, cpu->regs[R_A]);
            break;
        }
        case 1: { // ADC
            uint16_t sum = (uint16_t)acc + val + cpu->flags_c;
            cpu->flags_ac = (((acc & 0x0F) + (val & 0x0F) + cpu->flags_c) > 0x0F) ? 1 : 0;
            cpu->regs[R_A] = sum & 0xFF;
            cpu->flags_c = (sum > 0xFF) ? 1 : 0;
            update_flags_zsp(cpu, cpu->regs[R_A]);
            break;
        }
        case 2: { // SUB
            uint16_t diff = (uint16_t)acc - val;
            cpu->flags_ac = (((acc & 0x0F) - (val & 0x0F)) & 0x10) ? 0 : 1;
            cpu->regs[R_A] = diff & 0xFF;
            cpu->flags_c = (acc < val) ? 1 : 0;
            update_flags_zsp(cpu, cpu->regs[R_A]);
            break;
        }
        case 3: { // SBB
            uint16_t sub = val + cpu->flags_c;
            uint16_t diff = (uint16_t)acc - sub;
            cpu->flags_ac = (((acc & 0x0F) - (val & 0x0F) - cpu->flags_c) & 0x10) ? 0 : 1;
            cpu->regs[R_A] = diff & 0xFF;
            cpu->flags_c = (acc < sub) ? 1 : 0;
            update_flags_zsp(cpu, cpu->regs[R_A]);
            break;
        }
        case 4: { // ANA
            uint8_t res = acc & val;
            cpu->flags_ac = ((acc | val) & 0x08) ? 1 : 0; // 8080 AC quirk on AND
            cpu->regs[R_A] = res;
            cpu->flags_c = 0;
            update_flags_zsp(cpu, res);
            break;
        }
        case 5: { // XRA
            cpu->regs[R_A] = acc ^ val;
            cpu->flags_c = 0;
            cpu->flags_ac = 0;
            update_flags_zsp(cpu, cpu->regs[R_A]);
            break;
        }
        case 6: { // ORA
            cpu->regs[R_A] = acc | val;
            cpu->flags_c = 0;
            cpu->flags_ac = 0;
            update_flags_zsp(cpu, cpu->regs[R_A]);
            break;
        }
        case 7: { // CMP
            uint16_t diff = (uint16_t)acc - val;
            cpu->flags_ac = (((acc & 0x0F) - (val & 0x0F)) & 0x10) ? 0 : 1;
            cpu->flags_c = (acc < val) ? 1 : 0;
            update_flags_zsp(cpu, diff & 0xFF);
            break;
        }
    }
}

void* i8080_create(void) {
    return calloc(1, sizeof(I8080CPU));
}

void i8080_destroy(void *context) {
    free(context);
}

int i8080_init(void *context) {
    if (!context) return -1;
    I8080CPU *cpu = (I8080CPU*)context;
    memset(cpu->regs, 0, sizeof(cpu->regs));
    cpu->pc = 0;
    cpu->sp = 0;
    cpu->flags_c = cpu->flags_z = cpu->flags_s = cpu->flags_p = cpu->flags_ac = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int i8080_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    I8080CPU *cpu = (I8080CPU*)context;
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) copy_len = MEM_SIZE - address;
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Instruction length lookup keyed by opcode: 1, 2, or 3 bytes.
static int instr_len(uint8_t op) {
    // 3-byte: LXI (00rp0001), SHLD/LHLD/STA/LDA (0x22/0x2A/0x32/0x3A),
    //         JMP/Jcond (11ccc010 + 0xC3), CALL/Ccond (11ccc100 + 0xCD)
    if ((op & 0xCF) == 0x01) return 3;              // LXI
    if (op == 0x22 || op == 0x2A || op == 0x32 || op == 0x3A) return 3;
    if (op == 0xC3 || op == 0xCB) return 3;          // JMP (0xCB alt)
    if ((op & 0xC7) == 0xC2) return 3;               // Jcond
    if (op == 0xCD || op == 0xDD || op == 0xED || op == 0xFD) return 3; // CALL (+alts)
    if ((op & 0xC7) == 0xC4) return 3;               // Ccond
    // 2-byte: MVI (00ddd110), ALU immediate (11aaa110), IN/OUT, ADI-family already covered
    if ((op & 0xC7) == 0x06) return 2;               // MVI
    if ((op & 0xC7) == 0xC6) return 2;               // ADI/ACI/SUI/SBI/ANI/XRI/ORI/CPI
    if (op == 0xDB || op == 0xD3) return 2;          // IN / OUT
    return 1;
}

int i8080_step(void *context) {
    if (!context) return -1;
    I8080CPU *cpu = (I8080CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint8_t op = cpu->memory[cpu->pc];
    uint8_t byte2 = cpu->memory[(cpu->pc + 1) & 0xFFFF];
    uint8_t byte3 = cpu->memory[(cpu->pc + 2) & 0xFFFF];
    uint16_t imm16 = ((uint16_t)byte3 << 8) | byte2;

    int len = instr_len(op);
    uint16_t next_pc = (cpu->pc + len) & 0xFFFF;
    cpu->pc = next_pc;
    cpu->ticks++;

    // --- MOV / HLT (0x40..0x7F) ---
    if (op >= 0x40 && op <= 0x7F) {
        if (op == 0x76) { // HLT
            cpu->halted = 1;
            return 1;
        }
        uint8_t dest = (op >> 3) & 0x07;
        uint8_t src = op & 0x07;
        set_reg(cpu, dest, get_reg(cpu, src));
        return 0;
    }

    // --- ALU register ops (0x80..0xBF) ---
    if (op >= 0x80 && op <= 0xBF) {
        alu_op(cpu, (op >> 3) & 0x07, get_reg(cpu, op & 0x07));
        return 0;
    }

    switch (op) {
        case 0x00: // NOP
        case 0x08: case 0x10: case 0x18: case 0x20: case 0x28: case 0x30: case 0x38: // undocumented NOPs
            break;

        // --- LXI rp, d16 ---
        case 0x01: case 0x11: case 0x21: case 0x31:
            set_rp(cpu, (op >> 4) & 0x03, imm16);
            break;

        // --- STAX / LDAX (BC, DE) ---
        case 0x02: cpu->memory[get_rp(cpu, 0)] = cpu->regs[R_A]; break; // STAX B
        case 0x12: cpu->memory[get_rp(cpu, 1)] = cpu->regs[R_A]; break; // STAX D
        case 0x0A: cpu->regs[R_A] = cpu->memory[get_rp(cpu, 0)]; break; // LDAX B
        case 0x1A: cpu->regs[R_A] = cpu->memory[get_rp(cpu, 1)]; break; // LDAX D

        // --- Direct addressing ---
        case 0x32: cpu->memory[imm16] = cpu->regs[R_A]; break;         // STA
        case 0x3A: cpu->regs[R_A] = cpu->memory[imm16]; break;         // LDA
        case 0x22: // SHLD
            cpu->memory[imm16] = cpu->regs[R_L];
            cpu->memory[(imm16 + 1) & 0xFFFF] = cpu->regs[R_H];
            break;
        case 0x2A: // LHLD
            cpu->regs[R_L] = cpu->memory[imm16];
            cpu->regs[R_H] = cpu->memory[(imm16 + 1) & 0xFFFF];
            break;

        // --- INX / DCX ---
        case 0x03: case 0x13: case 0x23: case 0x33:
            set_rp(cpu, (op >> 4) & 3, (get_rp(cpu, (op >> 4) & 3) + 1) & 0xFFFF);
            break;
        case 0x0B: case 0x1B: case 0x2B: case 0x3B:
            set_rp(cpu, (op >> 4) & 3, (get_rp(cpu, (op >> 4) & 3) - 1) & 0xFFFF);
            break;

        // --- DAD rp (HL += rp) ---
        case 0x09: case 0x19: case 0x29: case 0x39: {
            uint32_t sum = (uint32_t)hl_addr(cpu) + get_rp(cpu, (op >> 4) & 3);
            cpu->flags_c = (sum > 0xFFFF) ? 1 : 0;
            set_rp(cpu, 2, sum & 0xFFFF);
            break;
        }

        // --- INR / DCR (00ddd100 / 00ddd101) ---
        case 0x04: case 0x0C: case 0x14: case 0x1C:
        case 0x24: case 0x2C: case 0x34: case 0x3C: {
            uint8_t reg = (op >> 3) & 7;
            uint8_t old = get_reg(cpu, reg);
            uint8_t res = old + 1;
            cpu->flags_ac = ((old & 0x0F) == 0x0F) ? 1 : 0;
            set_reg(cpu, reg, res);
            update_flags_zsp(cpu, res);
            break;
        }
        case 0x05: case 0x0D: case 0x15: case 0x1D:
        case 0x25: case 0x2D: case 0x35: case 0x3D: {
            uint8_t reg = (op >> 3) & 7;
            uint8_t old = get_reg(cpu, reg);
            uint8_t res = old - 1;
            cpu->flags_ac = ((old & 0x0F) == 0x00) ? 0 : 1;
            set_reg(cpu, reg, res);
            update_flags_zsp(cpu, res);
            break;
        }

        // --- MVI r, d8 ---
        case 0x06: case 0x0E: case 0x16: case 0x1E:
        case 0x26: case 0x2E: case 0x36: case 0x3E:
            set_reg(cpu, (op >> 3) & 7, byte2);
            break;

        // --- Rotates ---
        case 0x07: { // RLC
            uint8_t msb = (cpu->regs[R_A] >> 7) & 1;
            cpu->regs[R_A] = (cpu->regs[R_A] << 1) | msb;
            cpu->flags_c = msb;
            break;
        }
        case 0x0F: { // RRC
            uint8_t lsb = cpu->regs[R_A] & 1;
            cpu->regs[R_A] = (cpu->regs[R_A] >> 1) | (lsb << 7);
            cpu->flags_c = lsb;
            break;
        }
        case 0x17: { // RAL
            uint8_t msb = (cpu->regs[R_A] >> 7) & 1;
            cpu->regs[R_A] = (cpu->regs[R_A] << 1) | cpu->flags_c;
            cpu->flags_c = msb;
            break;
        }
        case 0x1F: { // RAR
            uint8_t lsb = cpu->regs[R_A] & 1;
            cpu->regs[R_A] = (cpu->regs[R_A] >> 1) | (cpu->flags_c << 7);
            cpu->flags_c = lsb;
            break;
        }

        // --- DAA ---
        case 0x27: {
            uint8_t a = cpu->regs[R_A];
            uint8_t correction = 0;
            uint8_t new_carry = cpu->flags_c;
            if (cpu->flags_ac || (a & 0x0F) > 9) correction |= 0x06;
            if (cpu->flags_c || (a >> 4) > 9 || ((a >> 4) >= 9 && (a & 0x0F) > 9)) {
                correction |= 0x60;
                new_carry = 1;
            }
            cpu->flags_ac = (((a & 0x0F) + (correction & 0x0F)) > 0x0F) ? 1 : 0;
            a += correction;
            cpu->regs[R_A] = a;
            cpu->flags_c = new_carry;
            update_flags_zsp(cpu, a);
            break;
        }

        // --- CMA / STC / CMC ---
        case 0x2F: cpu->regs[R_A] = ~cpu->regs[R_A]; break;       // CMA
        case 0x37: cpu->flags_c = 1; break;                       // STC
        case 0x3F: cpu->flags_c = !cpu->flags_c; break;           // CMC

        // --- ALU immediate (11aaa110) ---
        case 0xC6: case 0xCE: case 0xD6: case 0xDE:
        case 0xE6: case 0xEE: case 0xF6: case 0xFE:
            alu_op(cpu, (op >> 3) & 7, byte2);
            break;

        // --- Jumps ---
        case 0xC3: case 0xCB: cpu->pc = imm16; break; // JMP
        case 0xC2: case 0xCA: case 0xD2: case 0xDA:
        case 0xE2: case 0xEA: case 0xF2: case 0xFA:
            if (check_cond(cpu, (op >> 3) & 7)) cpu->pc = imm16;
            break;

        // --- Calls ---
        case 0xCD: case 0xDD: case 0xED: case 0xFD:
            push16(cpu, next_pc);
            cpu->pc = imm16;
            break;
        case 0xC4: case 0xCC: case 0xD4: case 0xDC:
        case 0xE4: case 0xEC: case 0xF4: case 0xFC:
            if (check_cond(cpu, (op >> 3) & 7)) {
                push16(cpu, next_pc);
                cpu->pc = imm16;
            }
            break;

        // --- Returns ---
        case 0xC9: case 0xD9: cpu->pc = pop16(cpu); break; // RET
        case 0xC0: case 0xC8: case 0xD0: case 0xD8:
        case 0xE0: case 0xE8: case 0xF0: case 0xF8:
            if (check_cond(cpu, (op >> 3) & 7)) cpu->pc = pop16(cpu);
            break;

        // --- RST n ---
        case 0xC7: case 0xCF: case 0xD7: case 0xDF:
        case 0xE7: case 0xEF: case 0xF7: case 0xFF:
            push16(cpu, next_pc);
            cpu->pc = ((op >> 3) & 7) * 8;
            break;

        // --- Stack ops: PUSH / POP rp (11rp0101 / 11rp0001) ---
        case 0xC5: push16(cpu, get_rp(cpu, 0)); break;                 // PUSH B
        case 0xD5: push16(cpu, get_rp(cpu, 1)); break;                 // PUSH D
        case 0xE5: push16(cpu, get_rp(cpu, 2)); break;                 // PUSH H
        case 0xF5: push16(cpu, ((uint16_t)cpu->regs[R_A] << 8) | get_psw(cpu)); break; // PUSH PSW
        case 0xC1: set_rp(cpu, 0, pop16(cpu)); break;                  // POP B
        case 0xD1: set_rp(cpu, 1, pop16(cpu)); break;                  // POP D
        case 0xE1: set_rp(cpu, 2, pop16(cpu)); break;                  // POP H
        case 0xF1: { uint16_t v = pop16(cpu); set_psw(cpu, v & 0xFF); cpu->regs[R_A] = v >> 8; break; } // POP PSW

        // --- Exchange / stack pointer ops ---
        case 0xEB: { // XCHG
            uint8_t t;
            t = cpu->regs[R_H]; cpu->regs[R_H] = cpu->regs[R_D]; cpu->regs[R_D] = t;
            t = cpu->regs[R_L]; cpu->regs[R_L] = cpu->regs[R_E]; cpu->regs[R_E] = t;
            break;
        }
        case 0xE3: { // XTHL
            uint8_t lo = cpu->memory[cpu->sp];
            uint8_t hi = cpu->memory[(cpu->sp + 1) & 0xFFFF];
            cpu->memory[cpu->sp] = cpu->regs[R_L];
            cpu->memory[(cpu->sp + 1) & 0xFFFF] = cpu->regs[R_H];
            cpu->regs[R_L] = lo;
            cpu->regs[R_H] = hi;
            break;
        }
        case 0xF9: cpu->sp = hl_addr(cpu); break; // SPHL
        case 0xE9: cpu->pc = hl_addr(cpu); break; // PCHL

        // --- I/O and interrupt control ---
        case 0xD3: // OUT port (no device attached)
            break;
        case 0xDB: // IN port (no device attached)
            cpu->regs[R_A] = 0;
            break;
        case 0xF3: cpu->interrupts_enabled = 0; break; // DI
        case 0xFB: cpu->interrupts_enabled = 1; break; // EI

        default:
            // Should be unreachable; treat as NOP.
            break;
    }

    // Self-loop (e.g. JMP $) interpreted as a software halt, matching other cores.
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

void i8080_print_state(void *context) {
    if (!context) return;
    I8080CPU *cpu = (I8080CPU*)context;

    printf("Intel 8080 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%04X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Flags: Sign=%d Zero=%d AuxCarry=%d Parity=%d Carry=%d\n",
           cpu->flags_s, cpu->flags_z, cpu->flags_ac, cpu->flags_p, cpu->flags_c);
    printf("  Registers:\n");
    printf("    A: 0x%02X\n", cpu->regs[R_A]);
    printf("    B: 0x%02X  C: 0x%02X   (BC: 0x%04X)\n", cpu->regs[R_B], cpu->regs[R_C], get_rp(cpu, 0));
    printf("    D: 0x%02X  E: 0x%02X   (DE: 0x%04X)\n", cpu->regs[R_D], cpu->regs[R_E], get_rp(cpu, 1));
    printf("    H: 0x%02X  L: 0x%02X   (HL: 0x%04X)\n", cpu->regs[R_H], cpu->regs[R_L], get_rp(cpu, 2));
    printf("  M (at HL 0x%04X): 0x%02X\n", hl_addr(cpu), cpu->memory[hl_addr(cpu)]);
}

void i8080_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    I8080CPU *cpu = (I8080CPU*)context;

    uint8_t op = cpu->memory[cpu->pc];
    uint8_t byte2 = cpu->memory[(cpu->pc + 1) & 0xFFFF];
    uint16_t imm16 = ((uint16_t)cpu->memory[(cpu->pc + 2) & 0xFFFF] << 8) | byte2;

    if (op == 0x76) { snprintf(buf, buf_len, "HLT"); return; }
    if (op >= 0x40 && op <= 0x7F) {
        snprintf(buf, buf_len, "MOV   %s, %s", r_names[(op >> 3) & 7], r_names[op & 7]);
        return;
    }
    if (op >= 0x80 && op <= 0xBF) {
        const char* alu_names[] = { "ADD", "ADC", "SUB", "SBB", "ANA", "XRA", "ORA", "CMP" };
        snprintf(buf, buf_len, "%-5s %s", alu_names[(op >> 3) & 7], r_names[op & 7]);
        return;
    }

    switch (op) {
        case 0x00: case 0x08: case 0x10: case 0x18:
        case 0x20: case 0x28: case 0x30: case 0x38:
            snprintf(buf, buf_len, "NOP"); break;
        case 0x01: case 0x11: case 0x21: case 0x31:
            snprintf(buf, buf_len, "LXI   %s, 0x%04X", rp_names[(op >> 4) & 3], imm16); break;
        case 0x02: snprintf(buf, buf_len, "STAX  B"); break;
        case 0x12: snprintf(buf, buf_len, "STAX  D"); break;
        case 0x0A: snprintf(buf, buf_len, "LDAX  B"); break;
        case 0x1A: snprintf(buf, buf_len, "LDAX  D"); break;
        case 0x32: snprintf(buf, buf_len, "STA   0x%04X", imm16); break;
        case 0x3A: snprintf(buf, buf_len, "LDA   0x%04X", imm16); break;
        case 0x22: snprintf(buf, buf_len, "SHLD  0x%04X", imm16); break;
        case 0x2A: snprintf(buf, buf_len, "LHLD  0x%04X", imm16); break;
        case 0x03: case 0x13: case 0x23: case 0x33:
            snprintf(buf, buf_len, "INX   %s", rp_names[(op >> 4) & 3]); break;
        case 0x0B: case 0x1B: case 0x2B: case 0x3B:
            snprintf(buf, buf_len, "DCX   %s", rp_names[(op >> 4) & 3]); break;
        case 0x09: case 0x19: case 0x29: case 0x39:
            snprintf(buf, buf_len, "DAD   %s", rp_names[(op >> 4) & 3]); break;
        case 0x04: case 0x0C: case 0x14: case 0x1C:
        case 0x24: case 0x2C: case 0x34: case 0x3C:
            snprintf(buf, buf_len, "INR   %s", r_names[(op >> 3) & 7]); break;
        case 0x05: case 0x0D: case 0x15: case 0x1D:
        case 0x25: case 0x2D: case 0x35: case 0x3D:
            snprintf(buf, buf_len, "DCR   %s", r_names[(op >> 3) & 7]); break;
        case 0x06: case 0x0E: case 0x16: case 0x1E:
        case 0x26: case 0x2E: case 0x36: case 0x3E:
            snprintf(buf, buf_len, "MVI   %s, 0x%02X", r_names[(op >> 3) & 7], byte2); break;
        case 0x07: snprintf(buf, buf_len, "RLC"); break;
        case 0x0F: snprintf(buf, buf_len, "RRC"); break;
        case 0x17: snprintf(buf, buf_len, "RAL"); break;
        case 0x1F: snprintf(buf, buf_len, "RAR"); break;
        case 0x27: snprintf(buf, buf_len, "DAA"); break;
        case 0x2F: snprintf(buf, buf_len, "CMA"); break;
        case 0x37: snprintf(buf, buf_len, "STC"); break;
        case 0x3F: snprintf(buf, buf_len, "CMC"); break;
        case 0xC6: case 0xCE: case 0xD6: case 0xDE:
        case 0xE6: case 0xEE: case 0xF6: case 0xFE: {
            const char* imm_names[] = { "ADI", "ACI", "SUI", "SBI", "ANI", "XRI", "ORI", "CPI" };
            snprintf(buf, buf_len, "%-5s 0x%02X", imm_names[(op >> 3) & 7], byte2); break;
        }
        case 0xC3: case 0xCB: snprintf(buf, buf_len, "JMP   0x%04X", imm16); break;
        case 0xC2: case 0xCA: case 0xD2: case 0xDA:
        case 0xE2: case 0xEA: case 0xF2: case 0xFA:
            snprintf(buf, buf_len, "J%-4s 0x%04X", c_names[(op >> 3) & 7], imm16); break;
        case 0xCD: case 0xDD: case 0xED: case 0xFD:
            snprintf(buf, buf_len, "CALL  0x%04X", imm16); break;
        case 0xC4: case 0xCC: case 0xD4: case 0xDC:
        case 0xE4: case 0xEC: case 0xF4: case 0xFC:
            snprintf(buf, buf_len, "C%-4s 0x%04X", c_names[(op >> 3) & 7], imm16); break;
        case 0xC9: case 0xD9: snprintf(buf, buf_len, "RET"); break;
        case 0xC0: case 0xC8: case 0xD0: case 0xD8:
        case 0xE0: case 0xE8: case 0xF0: case 0xF8:
            snprintf(buf, buf_len, "R%-4s", c_names[(op >> 3) & 7]); break;
        case 0xC7: case 0xCF: case 0xD7: case 0xDF:
        case 0xE7: case 0xEF: case 0xF7: case 0xFF:
            snprintf(buf, buf_len, "RST   %d", (op >> 3) & 7); break;
        case 0xC5: snprintf(buf, buf_len, "PUSH  B"); break;
        case 0xD5: snprintf(buf, buf_len, "PUSH  D"); break;
        case 0xE5: snprintf(buf, buf_len, "PUSH  H"); break;
        case 0xF5: snprintf(buf, buf_len, "PUSH  PSW"); break;
        case 0xC1: snprintf(buf, buf_len, "POP   B"); break;
        case 0xD1: snprintf(buf, buf_len, "POP   D"); break;
        case 0xE1: snprintf(buf, buf_len, "POP   H"); break;
        case 0xF1: snprintf(buf, buf_len, "POP   PSW"); break;
        case 0xEB: snprintf(buf, buf_len, "XCHG"); break;
        case 0xE3: snprintf(buf, buf_len, "XTHL"); break;
        case 0xF9: snprintf(buf, buf_len, "SPHL"); break;
        case 0xE9: snprintf(buf, buf_len, "PCHL"); break;
        case 0xD3: snprintf(buf, buf_len, "OUT   0x%02X", byte2); break;
        case 0xDB: snprintf(buf, buf_len, "IN    0x%02X", byte2); break;
        case 0xF3: snprintf(buf, buf_len, "DI"); break;
        case 0xFB: snprintf(buf, buf_len, "EI"); break;
        default: snprintf(buf, buf_len, "INV   0x%02X", op); break;
    }
}
