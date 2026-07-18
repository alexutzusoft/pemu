#include "cdp1802.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 65536 // 64 KB (16-bit address space)
#define NUM_PORTS 8

typedef struct CDP1802CPU {
    uint16_t r[16];  // Scratchpad registers R0-RF
    uint8_t d;       // D accumulator
    uint8_t df;      // Data Flag (carry/no-borrow)
    uint8_t p;       // Program counter selector (4-bit)
    uint8_t x;       // Data pointer selector (4-bit)
    uint8_t t;       // Temporary register (saved X,P)
    uint8_t q;       // Q output flip-flop
    uint8_t ie;      // Interrupt enable
    uint8_t ef[4];   // External flag inputs EF1-EF4 (active = 1)
    uint8_t ports[NUM_PORTS]; // Simple I/O port latches (index 1..7 used)

    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;      // Set by IDL
} CDP1802CPU;

static inline uint8_t mem_read(CDP1802CPU *cpu, uint16_t addr) {
    return cpu->memory[addr];
}

static inline void mem_write(CDP1802CPU *cpu, uint16_t addr, uint8_t val) {
    cpu->memory[addr] = val;
}

// Fetch a byte at R(P) and advance R(P)
static inline uint8_t fetch(CDP1802CPU *cpu) {
    uint8_t val = mem_read(cpu, cpu->r[cpu->p]);
    cpu->r[cpu->p]++;
    return val;
}

// ALU helpers operating on D, DF
static inline void alu_add(CDP1802CPU *cpu, uint8_t val, uint8_t carry_in) {
    uint16_t sum = (uint16_t)cpu->d + val + carry_in;
    cpu->d = sum & 0xFF;
    cpu->df = (sum > 0xFF) ? 1 : 0;
}

// D = val - D - borrow_in (borrow_in = 1 when DF was 0); DF = 1 if no borrow
static inline void alu_sub(CDP1802CPU *cpu, uint8_t minuend, uint8_t subtrahend, uint8_t borrow_in) {
    uint16_t diff = (uint16_t)minuend - subtrahend - borrow_in;
    cpu->d = diff & 0xFF;
    cpu->df = (diff > 0xFF) ? 0 : 1;
}

// Evaluate short branch condition for opcodes 0x30..0x3F. Returns 1 if taken.
static int short_branch_cond(CDP1802CPU *cpu, uint8_t n) {
    switch (n) {
        case 0x0: return 1;                 // BR
        case 0x1: return cpu->q != 0;       // BQ
        case 0x2: return cpu->d == 0;       // BZ
        case 0x3: return cpu->df != 0;      // BDF
        case 0x4: return cpu->ef[0] != 0;   // B1
        case 0x5: return cpu->ef[1] != 0;   // B2
        case 0x6: return cpu->ef[2] != 0;   // B3
        case 0x7: return cpu->ef[3] != 0;   // B4
        case 0x8: return 0;                 // SKP (NBR)
        case 0x9: return cpu->q == 0;       // BNQ
        case 0xA: return cpu->d != 0;       // BNZ
        case 0xB: return cpu->df == 0;      // BNF
        case 0xC: return cpu->ef[0] == 0;   // BN1
        case 0xD: return cpu->ef[1] == 0;   // BN2
        case 0xE: return cpu->ef[2] == 0;   // BN3
        case 0xF: return cpu->ef[3] == 0;   // BN4
    }
    return 0;
}

void* cdp1802_create(void) {
    CDP1802CPU *cpu = (CDP1802CPU*)calloc(1, sizeof(CDP1802CPU));
    return cpu;
}

void cdp1802_destroy(void *context) {
    free(context);
}

int cdp1802_init(void *context) {
    if (!context) return -1;
    CDP1802CPU *cpu = (CDP1802CPU*)context;

    memset(cpu, 0, sizeof(CDP1802CPU));
    // Hardware reset state: P = X = 0, R(0) = 0, IE = 1, Q reset
    cpu->ie = 1;
    return 0;
}

int cdp1802_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    CDP1802CPU *cpu = (CDP1802CPU*)context;

    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int cdp1802_step(void *context) {
    if (!context) return -1;
    CDP1802CPU *cpu = (CDP1802CPU*)context;

    if (cpu->halted) return 1;

    uint8_t op = fetch(cpu);
    uint8_t hi = (op >> 4) & 0x0F;
    uint8_t n = op & 0x0F;

    cpu->ticks++;

    switch (hi) {
        case 0x0:
            if (n == 0) {
                // IDL - idle/halt until interrupt or DMA (we treat as halt)
                cpu->halted = 1;
                return 1;
            }
            // LDN RN - D = M(R(N)), N != 0
            cpu->d = mem_read(cpu, cpu->r[n]);
            break;

        case 0x1: // INC RN
            cpu->r[n]++;
            break;

        case 0x2: // DEC RN
            cpu->r[n]--;
            break;

        case 0x3: { // Short branches / SKP
            if (n == 0x8) {
                // SKP - skip the immediate byte
                cpu->r[cpu->p]++;
            } else {
                uint8_t addr_lo = mem_read(cpu, cpu->r[cpu->p]);
                if (short_branch_cond(cpu, n)) {
                    cpu->r[cpu->p] = (cpu->r[cpu->p] & 0xFF00) | addr_lo;
                } else {
                    cpu->r[cpu->p]++;
                }
            }
            break;
        }

        case 0x4: // LDA RN - D = M(R(N)); R(N)++
            cpu->d = mem_read(cpu, cpu->r[n]);
            cpu->r[n]++;
            break;

        case 0x5: // STR RN - M(R(N)) = D
            mem_write(cpu, cpu->r[n], cpu->d);
            break;

        case 0x6:
            if (n == 0x0) {
                // IRX - R(X)++
                cpu->r[cpu->x]++;
            } else if (n <= 0x7) {
                // OUT p - port[p] = M(R(X)); R(X)++
                cpu->ports[n] = mem_read(cpu, cpu->r[cpu->x]);
                cpu->r[cpu->x]++;
            } else if (n == 0x8) {
                // 0x68 is undefined on the 1802 (1806 extension prefix); treat as NOP
            } else {
                // INP p - M(R(X)) = port[p]; D = port[p]
                uint8_t val = cpu->ports[n & 0x7];
                mem_write(cpu, cpu->r[cpu->x], val);
                cpu->d = val;
            }
            break;

        case 0x7:
            switch (n) {
                case 0x0: { // RET - (X,P) = M(R(X)); R(X)++; IE = 1
                    uint8_t val = mem_read(cpu, cpu->r[cpu->x]);
                    cpu->r[cpu->x]++;
                    cpu->p = val & 0x0F;
                    cpu->x = (val >> 4) & 0x0F;
                    cpu->ie = 1;
                    break;
                }
                case 0x1: { // DIS - (X,P) = M(R(X)); R(X)++; IE = 0
                    uint8_t val = mem_read(cpu, cpu->r[cpu->x]);
                    cpu->r[cpu->x]++;
                    cpu->p = val & 0x0F;
                    cpu->x = (val >> 4) & 0x0F;
                    cpu->ie = 0;
                    break;
                }
                case 0x2: // LDXA - D = M(R(X)); R(X)++
                    cpu->d = mem_read(cpu, cpu->r[cpu->x]);
                    cpu->r[cpu->x]++;
                    break;
                case 0x3: // STXD - M(R(X)) = D; R(X)--
                    mem_write(cpu, cpu->r[cpu->x], cpu->d);
                    cpu->r[cpu->x]--;
                    break;
                case 0x4: // ADC - D = M(R(X)) + D + DF
                    alu_add(cpu, mem_read(cpu, cpu->r[cpu->x]), cpu->df);
                    break;
                case 0x5: // SDB - D = M(R(X)) - D - (NOT DF)
                    alu_sub(cpu, mem_read(cpu, cpu->r[cpu->x]), cpu->d, (uint8_t)(cpu->df ? 0 : 1));
                    break;
                case 0x6: { // SHRC - shift right, DF into MSB, LSB into DF
                    uint8_t lsb = cpu->d & 1;
                    cpu->d = (cpu->d >> 1) | (uint8_t)(cpu->df << 7);
                    cpu->df = lsb;
                    break;
                }
                case 0x7: // SMB - D = D - M(R(X)) - (NOT DF)
                    alu_sub(cpu, cpu->d, mem_read(cpu, cpu->r[cpu->x]), (uint8_t)(cpu->df ? 0 : 1));
                    break;
                case 0x8: // SAV - M(R(X)) = T
                    mem_write(cpu, cpu->r[cpu->x], cpu->t);
                    break;
                case 0x9: // MARK - T = (X,P); M(R(2)) = T; X = P; R(2)--
                    cpu->t = (uint8_t)((cpu->x << 4) | cpu->p);
                    mem_write(cpu, cpu->r[2], cpu->t);
                    cpu->x = cpu->p;
                    cpu->r[2]--;
                    break;
                case 0xA: // REQ - Q = 0
                    cpu->q = 0;
                    break;
                case 0xB: // SEQ - Q = 1
                    cpu->q = 1;
                    break;
                case 0xC: // ADCI - D = imm + D + DF
                    alu_add(cpu, fetch(cpu), cpu->df);
                    break;
                case 0xD: // SDBI - D = imm - D - (NOT DF)
                    alu_sub(cpu, fetch(cpu), cpu->d, (uint8_t)(cpu->df ? 0 : 1));
                    break;
                case 0xE: { // SHLC - shift left, DF into LSB, MSB into DF
                    uint8_t msb = (cpu->d >> 7) & 1;
                    cpu->d = (uint8_t)((cpu->d << 1) | cpu->df);
                    cpu->df = msb;
                    break;
                }
                case 0xF: // SMBI - D = D - imm - (NOT DF)
                    alu_sub(cpu, cpu->d, fetch(cpu), (uint8_t)(cpu->df ? 0 : 1));
                    break;
            }
            break;

        case 0x8: // GLO RN - D = R(N).0
            cpu->d = cpu->r[n] & 0xFF;
            break;

        case 0x9: // GHI RN - D = R(N).1
            cpu->d = (cpu->r[n] >> 8) & 0xFF;
            break;

        case 0xA: // PLO RN - R(N).0 = D
            cpu->r[n] = (cpu->r[n] & 0xFF00) | cpu->d;
            break;

        case 0xB: // PHI RN - R(N).1 = D
            cpu->r[n] = (uint16_t)((cpu->d << 8) | (cpu->r[n] & 0x00FF));
            break;

        case 0xC:
            switch (n) {
                case 0x0: case 0x1: case 0x2: case 0x3:
                case 0x9: case 0xA: case 0xB: { // Long branches
                    int taken = 0;
                    switch (n) {
                        case 0x0: taken = 1; break;               // LBR
                        case 0x1: taken = (cpu->q != 0); break;   // LBQ
                        case 0x2: taken = (cpu->d == 0); break;   // LBZ
                        case 0x3: taken = (cpu->df != 0); break;  // LBDF
                        case 0x9: taken = (cpu->q == 0); break;   // LBNQ
                        case 0xA: taken = (cpu->d != 0); break;   // LBNZ
                        case 0xB: taken = (cpu->df == 0); break;  // LBNF
                    }
                    if (taken) {
                        uint8_t addr_hi = mem_read(cpu, cpu->r[cpu->p]);
                        uint8_t addr_lo = mem_read(cpu, (uint16_t)(cpu->r[cpu->p] + 1));
                        cpu->r[cpu->p] = (uint16_t)((addr_hi << 8) | addr_lo);
                    } else {
                        cpu->r[cpu->p] += 2;
                    }
                    break;
                }
                case 0x4: // NOP
                    break;
                case 0x5: // LSNQ - long skip if Q = 0
                    if (cpu->q == 0) cpu->r[cpu->p] += 2;
                    break;
                case 0x6: // LSNZ - long skip if D != 0
                    if (cpu->d != 0) cpu->r[cpu->p] += 2;
                    break;
                case 0x7: // LSNF - long skip if DF = 0
                    if (cpu->df == 0) cpu->r[cpu->p] += 2;
                    break;
                case 0x8: // LSKP (NLBR) - unconditional long skip
                    cpu->r[cpu->p] += 2;
                    break;
                case 0xC: // LSIE - long skip if IE = 1
                    if (cpu->ie != 0) cpu->r[cpu->p] += 2;
                    break;
                case 0xD: // LSQ - long skip if Q = 1
                    if (cpu->q != 0) cpu->r[cpu->p] += 2;
                    break;
                case 0xE: // LSZ - long skip if D = 0
                    if (cpu->d == 0) cpu->r[cpu->p] += 2;
                    break;
                case 0xF: // LSDF - long skip if DF = 1
                    if (cpu->df != 0) cpu->r[cpu->p] += 2;
                    break;
            }
            break;

        case 0xD: // SEP RN - P = N
            cpu->p = n;
            break;

        case 0xE: // SEX RN - X = N
            cpu->x = n;
            break;

        case 0xF:
            switch (n) {
                case 0x0: // LDX - D = M(R(X))
                    cpu->d = mem_read(cpu, cpu->r[cpu->x]);
                    break;
                case 0x1: // OR - D = M(R(X)) | D
                    cpu->d |= mem_read(cpu, cpu->r[cpu->x]);
                    break;
                case 0x2: // AND - D = M(R(X)) & D
                    cpu->d &= mem_read(cpu, cpu->r[cpu->x]);
                    break;
                case 0x3: // XOR - D = M(R(X)) ^ D
                    cpu->d ^= mem_read(cpu, cpu->r[cpu->x]);
                    break;
                case 0x4: // ADD - D = M(R(X)) + D
                    alu_add(cpu, mem_read(cpu, cpu->r[cpu->x]), 0);
                    break;
                case 0x5: // SD - D = M(R(X)) - D
                    alu_sub(cpu, mem_read(cpu, cpu->r[cpu->x]), cpu->d, 0);
                    break;
                case 0x6: { // SHR - shift right, 0 into MSB, LSB into DF
                    uint8_t lsb = cpu->d & 1;
                    cpu->d >>= 1;
                    cpu->df = lsb;
                    break;
                }
                case 0x7: // SM - D = D - M(R(X))
                    alu_sub(cpu, cpu->d, mem_read(cpu, cpu->r[cpu->x]), 0);
                    break;
                case 0x8: // LDI - D = imm
                    cpu->d = fetch(cpu);
                    break;
                case 0x9: // ORI - D = imm | D
                    cpu->d |= fetch(cpu);
                    break;
                case 0xA: // ANI - D = imm & D
                    cpu->d &= fetch(cpu);
                    break;
                case 0xB: // XRI - D = imm ^ D
                    cpu->d ^= fetch(cpu);
                    break;
                case 0xC: // ADI - D = imm + D
                    alu_add(cpu, fetch(cpu), 0);
                    break;
                case 0xD: // SDI - D = imm - D
                    alu_sub(cpu, fetch(cpu), cpu->d, 0);
                    break;
                case 0xE: { // SHL - shift left, 0 into LSB, MSB into DF
                    uint8_t msb = (cpu->d >> 7) & 1;
                    cpu->d <<= 1;
                    cpu->df = msb;
                    break;
                }
                case 0xF: // SMI - D = D - imm
                    alu_sub(cpu, cpu->d, fetch(cpu), 0);
                    break;
            }
            break;
    }

    return 0;
}

void cdp1802_print_state(void *context) {
    if (!context) return;
    CDP1802CPU *cpu = (CDP1802CPU*)context;

    printf("RCA CDP1802 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  D: 0x%02X  DF: %d  P: %X  X: %X  T: 0x%02X  Q: %d  IE: %d  Halted: %s\n",
           cpu->d, cpu->df, cpu->p, cpu->x, cpu->t, cpu->q, cpu->ie,
           cpu->halted ? "Yes" : "No");
    printf("  Scratchpad registers:\n");
    for (int i = 0; i < 16; ++i) {
        printf("    R%X: 0x%04X%s", i, cpu->r[i], (i % 4 == 3) ? "\n" : "  ");
    }
    printf("  R(P): 0x%04X (program counter)  R(X): 0x%04X (data pointer)\n",
           cpu->r[cpu->p], cpu->r[cpu->x]);
    printf("  EF1-4: %d %d %d %d\n", cpu->ef[0], cpu->ef[1], cpu->ef[2], cpu->ef[3]);
    printf("  Ports: [1]=0x%02X [2]=0x%02X [3]=0x%02X [4]=0x%02X [5]=0x%02X [6]=0x%02X [7]=0x%02X\n",
           cpu->ports[1], cpu->ports[2], cpu->ports[3], cpu->ports[4],
           cpu->ports[5], cpu->ports[6], cpu->ports[7]);
}

void cdp1802_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    CDP1802CPU *cpu = (CDP1802CPU*)context;

    uint16_t pc = cpu->r[cpu->p];
    uint8_t op = cpu->memory[pc];
    uint8_t hi = (op >> 4) & 0x0F;
    uint8_t n = op & 0x0F;
    uint8_t b2 = cpu->memory[(uint16_t)(pc + 1)];
    uint8_t b3 = cpu->memory[(uint16_t)(pc + 2)];
    uint16_t branch_lo = (uint16_t)(((uint16_t)(pc + 1) & 0xFF00) | b2);
    uint16_t branch_long = (uint16_t)((b2 << 8) | b3);

    static const char* sb_names[16] = {
        "BR", "BQ", "BZ", "BDF", "B1", "B2", "B3", "B4",
        "SKP", "BNQ", "BNZ", "BNF", "BN1", "BN2", "BN3", "BN4"
    };
    static const char* c_names[16] = {
        "LBR", "LBQ", "LBZ", "LBDF", "NOP", "LSNQ", "LSNZ", "LSNF",
        "LSKP", "LBNQ", "LBNZ", "LBNF", "LSIE", "LSQ", "LSZ", "LSDF"
    };
    static const char* s7_names[16] = {
        "RET", "DIS", "LDXA", "STXD", "ADC", "SDB", "SHRC", "SMB",
        "SAV", "MARK", "REQ", "SEQ", "ADCI", "SDBI", "SHLC", "SMBI"
    };
    static const char* sf_names[16] = {
        "LDX", "OR", "AND", "XOR", "ADD", "SD", "SHR", "SM",
        "LDI", "ORI", "ANI", "XRI", "ADI", "SDI", "SHL", "SMI"
    };

    switch (hi) {
        case 0x0:
            if (n == 0) snprintf(buf, buf_len, "IDL");
            else snprintf(buf, buf_len, "LDN   R%X", n);
            break;
        case 0x1:
            snprintf(buf, buf_len, "INC   R%X", n);
            break;
        case 0x2:
            snprintf(buf, buf_len, "DEC   R%X", n);
            break;
        case 0x3:
            if (n == 0x8) snprintf(buf, buf_len, "SKP");
            else snprintf(buf, buf_len, "%-5s 0x%04X", sb_names[n], branch_lo);
            break;
        case 0x4:
            snprintf(buf, buf_len, "LDA   R%X", n);
            break;
        case 0x5:
            snprintf(buf, buf_len, "STR   R%X", n);
            break;
        case 0x6:
            if (n == 0x0) snprintf(buf, buf_len, "IRX");
            else if (n <= 0x7) snprintf(buf, buf_len, "OUT   %d", n);
            else if (n == 0x8) snprintf(buf, buf_len, "INV   0x68");
            else snprintf(buf, buf_len, "INP   %d", n & 0x7);
            break;
        case 0x7:
            if (n >= 0xC && n != 0xE) {
                // Immediate forms: ADCI, SDBI, SMBI (SHLC at 0xE has no operand)
                snprintf(buf, buf_len, "%-5s 0x%02X", s7_names[n], b2);
            } else {
                snprintf(buf, buf_len, "%s", s7_names[n]);
            }
            break;
        case 0x8:
            snprintf(buf, buf_len, "GLO   R%X", n);
            break;
        case 0x9:
            snprintf(buf, buf_len, "GHI   R%X", n);
            break;
        case 0xA:
            snprintf(buf, buf_len, "PLO   R%X", n);
            break;
        case 0xB:
            snprintf(buf, buf_len, "PHI   R%X", n);
            break;
        case 0xC:
            if (n == 0x0 || n == 0x1 || n == 0x2 || n == 0x3 ||
                n == 0x9 || n == 0xA || n == 0xB) {
                snprintf(buf, buf_len, "%-5s 0x%04X", c_names[n], branch_long);
            } else {
                snprintf(buf, buf_len, "%s", c_names[n]);
            }
            break;
        case 0xD:
            snprintf(buf, buf_len, "SEP   R%X", n);
            break;
        case 0xE:
            snprintf(buf, buf_len, "SEX   R%X", n);
            break;
        case 0xF:
            if (n >= 0x8 && n != 0xE) {
                // Immediate forms: LDI, ORI, ANI, XRI, ADI, SDI, SMI (SHL at 0xE has none)
                snprintf(buf, buf_len, "%-5s 0x%02X", sf_names[n], b2);
            } else {
                snprintf(buf, buf_len, "%s", sf_names[n]);
            }
            break;
        default:
            snprintf(buf, buf_len, "INV   0x%02X", op);
            break;
    }
}
