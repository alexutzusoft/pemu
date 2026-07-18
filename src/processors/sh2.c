#include "sh2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE (1024 * 1024)
#define MEM_MASK (MEM_SIZE - 1)

// SR flag bits
#define SR_T 0x00000001u
#define SR_S 0x00000002u
#define SR_Q 0x00000100u
#define SR_M 0x00000200u
#define SR_MASK 0x000003F3u

typedef struct SH2CPU {
    uint32_t r[16];         // R0-R15 (R15 = SP)
    uint32_t pc;
    uint32_t sr;            // M Q I3-I0 S T
    uint32_t gbr;
    uint32_t vbr;
    uint32_t mach;
    uint32_t macl;
    uint32_t pr;
    uint8_t memory[MEM_SIZE];   // big-endian RAM
    uint32_t ticks;
    int halted;
    int delay_pending;      // a delayed branch was executed
    uint32_t delay_target;
} SH2CPU;

void* sh2_create(void) {
    SH2CPU *cpu = (SH2CPU*)calloc(1, sizeof(SH2CPU));
    return cpu;
}

void sh2_destroy(void *context) {
    free(context);
}

int sh2_init(void *context) {
    if (!context) return -1;
    SH2CPU *cpu = (SH2CPU*)context;

    memset(cpu->r, 0, sizeof(cpu->r));
    cpu->r[15] = MEM_SIZE - 4; // SP initialized near top of RAM
    cpu->pc = 0;
    cpu->sr = 0x000000F0; // interrupts masked (I3-I0 = 1111), all flags clear
    cpu->gbr = 0;
    cpu->vbr = 0;
    cpu->mach = 0;
    cpu->macl = 0;
    cpu->pr = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->delay_pending = 0;
    cpu->delay_target = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int sh2_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    SH2CPU *cpu = (SH2CPU*)context;

    address &= MEM_MASK;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

// Big-endian memory helpers; addresses are masked to the 1MB RAM window
static uint32_t rd8(SH2CPU *cpu, uint32_t addr) {
    return cpu->memory[addr & MEM_MASK];
}

static uint32_t rd16(SH2CPU *cpu, uint32_t addr) {
    addr &= MEM_MASK & ~1u;
    return ((uint32_t)cpu->memory[addr] << 8) | cpu->memory[addr + 1];
}

static uint32_t rd32(SH2CPU *cpu, uint32_t addr) {
    addr &= MEM_MASK & ~3u;
    return ((uint32_t)cpu->memory[addr] << 24) |
           ((uint32_t)cpu->memory[addr + 1] << 16) |
           ((uint32_t)cpu->memory[addr + 2] << 8) |
           (uint32_t)cpu->memory[addr + 3];
}

static void wr8(SH2CPU *cpu, uint32_t addr, uint32_t val) {
    cpu->memory[addr & MEM_MASK] = (uint8_t)val;
}

static void wr16(SH2CPU *cpu, uint32_t addr, uint32_t val) {
    addr &= MEM_MASK & ~1u;
    cpu->memory[addr] = (uint8_t)(val >> 8);
    cpu->memory[addr + 1] = (uint8_t)val;
}

static void wr32(SH2CPU *cpu, uint32_t addr, uint32_t val) {
    addr &= MEM_MASK & ~3u;
    cpu->memory[addr] = (uint8_t)(val >> 24);
    cpu->memory[addr + 1] = (uint8_t)(val >> 16);
    cpu->memory[addr + 2] = (uint8_t)(val >> 8);
    cpu->memory[addr + 3] = (uint8_t)val;
}

static void set_t(SH2CPU *cpu, int cond) {
    if (cond) cpu->sr |= SR_T;
    else cpu->sr &= ~SR_T;
}

static uint32_t sext8(uint32_t v)  { return (uint32_t)(int32_t)(int8_t)(uint8_t)v; }
static uint32_t sext16(uint32_t v) { return (uint32_t)(int32_t)(int16_t)(uint16_t)v; }

static void mac_add(SH2CPU *cpu, int64_t prod, int is_macw) {
    if (cpu->sr & SR_S) {
        if (is_macw) {
            // MAC.W with S=1: 32-bit saturating accumulate into MACL
            int64_t sum = (int64_t)(int32_t)cpu->macl + prod;
            if (sum > (int64_t)0x7FFFFFFF) {
                cpu->macl = 0x7FFFFFFFu;
                cpu->mach |= 1u;
            } else if (sum < -(int64_t)0x80000000) {
                cpu->macl = 0x80000000u;
                cpu->mach |= 1u;
            } else {
                cpu->macl = (uint32_t)sum;
            }
        } else {
            // MAC.L with S=1: saturate to 48-bit signed range
            int64_t acc = (int64_t)(((uint64_t)cpu->mach << 32) | cpu->macl);
            int64_t sum = acc + prod;
            if (sum > (int64_t)0x00007FFFFFFFFFFFLL) sum = (int64_t)0x00007FFFFFFFFFFFLL;
            else if (sum < (int64_t)0xFFFF800000000000LL) sum = (int64_t)0xFFFF800000000000LL;
            cpu->mach = (uint32_t)((uint64_t)sum >> 32);
            cpu->macl = (uint32_t)(uint64_t)sum;
        }
    } else {
        uint64_t acc = ((uint64_t)cpu->mach << 32) | cpu->macl;
        acc += (uint64_t)prod;
        cpu->mach = (uint32_t)(acc >> 32);
        cpu->macl = (uint32_t)acc;
    }
}

static void delayed_branch(SH2CPU *cpu, uint32_t target) {
    cpu->delay_pending = 1;
    cpu->delay_target = target;
}

// Execute one instruction. cpu->pc has already been advanced to addr+2.
// Returns 0 on success, 1 on halt, negative on error.
static int exec_instr(SH2CPU *cpu, uint16_t instr, uint32_t addr) {
    uint32_t n = (instr >> 8) & 0xF;
    uint32_t m = (instr >> 4) & 0xF;
    uint32_t d8 = instr & 0xFF;
    uint32_t d4 = instr & 0xF;
    uint32_t *R = cpu->r;

    switch (instr >> 12) {
        case 0x0:
            switch (instr & 0xF) {
                case 0x2: // STC xx,Rn
                    switch (m) {
                        case 0: R[n] = cpu->sr; return 0;  // STC SR,Rn
                        case 1: R[n] = cpu->gbr; return 0; // STC GBR,Rn
                        case 2: R[n] = cpu->vbr; return 0; // STC VBR,Rn
                        default: return -4;
                    }
                case 0x3:
                    switch (m) {
                        case 0: // BSRF Rn
                            cpu->pr = addr + 4;
                            delayed_branch(cpu, addr + 4 + R[n]);
                            return 0;
                        case 2: // BRAF Rn
                            delayed_branch(cpu, addr + 4 + R[n]);
                            return 0;
                        default: return -4;
                    }
                case 0x4: wr8(cpu, R[0] + R[n], R[m]); return 0;  // MOV.B Rm,@(R0,Rn)
                case 0x5: wr16(cpu, R[0] + R[n], R[m]); return 0; // MOV.W Rm,@(R0,Rn)
                case 0x6: wr32(cpu, R[0] + R[n], R[m]); return 0; // MOV.L Rm,@(R0,Rn)
                case 0x7: // MUL.L Rm,Rn
                    cpu->macl = R[n] * R[m];
                    return 0;
                case 0x8:
                    switch (m) {
                        case 0: cpu->sr &= ~SR_T; return 0; // CLRT
                        case 1: cpu->sr |= SR_T; return 0;  // SETT
                        case 2: cpu->mach = 0; cpu->macl = 0; return 0; // CLRMAC
                        default: return -4;
                    }
                case 0x9:
                    switch (m) {
                        case 0: return 0; // NOP
                        case 1: // DIV0U
                            cpu->sr &= ~(SR_M | SR_Q | SR_T);
                            return 0;
                        case 2: R[n] = cpu->sr & SR_T; return 0; // MOVT Rn
                        default: return -4;
                    }
                case 0xA: // STS xx,Rn
                    switch (m) {
                        case 0: R[n] = cpu->mach; return 0; // STS MACH,Rn
                        case 1: R[n] = cpu->macl; return 0; // STS MACL,Rn
                        case 2: R[n] = cpu->pr; return 0;   // STS PR,Rn
                        default: return -4;
                    }
                case 0xB:
                    switch (m) {
                        case 0: // RTS
                            delayed_branch(cpu, cpu->pr);
                            return 0;
                        case 1: // SLEEP
                            cpu->halted = 1;
                            return 1;
                        case 2: // RTE
                            delayed_branch(cpu, rd32(cpu, R[15]));
                            R[15] += 4;
                            cpu->sr = rd32(cpu, R[15]) & SR_MASK;
                            R[15] += 4;
                            return 0;
                        default: return -4;
                    }
                case 0xC: R[n] = sext8(rd8(cpu, R[0] + R[m])); return 0;   // MOV.B @(R0,Rm),Rn
                case 0xD: R[n] = sext16(rd16(cpu, R[0] + R[m])); return 0; // MOV.W @(R0,Rm),Rn
                case 0xE: R[n] = rd32(cpu, R[0] + R[m]); return 0;         // MOV.L @(R0,Rm),Rn
                case 0xF: { // MAC.L @Rm+,@Rn+
                    int32_t a = (int32_t)rd32(cpu, R[n]);
                    R[n] += 4;
                    int32_t b = (int32_t)rd32(cpu, R[m]);
                    R[m] += 4;
                    mac_add(cpu, (int64_t)a * (int64_t)b, 0);
                    return 0;
                }
                default: return -4;
            }

        case 0x1: // MOV.L Rm,@(disp*4,Rn)
            wr32(cpu, R[n] + d4 * 4, R[m]);
            return 0;

        case 0x2:
            switch (instr & 0xF) {
                case 0x0: wr8(cpu, R[n], R[m]); return 0;  // MOV.B Rm,@Rn
                case 0x1: wr16(cpu, R[n], R[m]); return 0; // MOV.W Rm,@Rn
                case 0x2: wr32(cpu, R[n], R[m]); return 0; // MOV.L Rm,@Rn
                case 0x4: R[n] -= 1; wr8(cpu, R[n], R[m]); return 0;  // MOV.B Rm,@-Rn
                case 0x5: R[n] -= 2; wr16(cpu, R[n], R[m]); return 0; // MOV.W Rm,@-Rn
                case 0x6: R[n] -= 4; wr32(cpu, R[n], R[m]); return 0; // MOV.L Rm,@-Rn
                case 0x7: { // DIV0S Rm,Rn
                    cpu->sr &= ~(SR_M | SR_Q | SR_T);
                    if (R[n] & 0x80000000u) cpu->sr |= SR_Q;
                    if (R[m] & 0x80000000u) cpu->sr |= SR_M;
                    set_t(cpu, ((cpu->sr & SR_Q) != 0) != ((cpu->sr & SR_M) != 0));
                    return 0;
                }
                case 0x8: set_t(cpu, (R[n] & R[m]) == 0); return 0; // TST Rm,Rn
                case 0x9: R[n] &= R[m]; return 0; // AND Rm,Rn
                case 0xA: R[n] ^= R[m]; return 0; // XOR Rm,Rn
                case 0xB: R[n] |= R[m]; return 0; // OR Rm,Rn
                case 0xC: { // CMP/STR Rm,Rn (T=1 if any byte equal)
                    uint32_t tmp = R[n] ^ R[m];
                    int eq = ((tmp & 0xFF000000u) == 0) || ((tmp & 0x00FF0000u) == 0) ||
                             ((tmp & 0x0000FF00u) == 0) || ((tmp & 0x000000FFu) == 0);
                    set_t(cpu, eq);
                    return 0;
                }
                case 0xD: R[n] = (R[m] << 16) | (R[n] >> 16); return 0; // XTRCT Rm,Rn
                case 0xE: // MULU.W Rm,Rn
                    cpu->macl = (uint32_t)(uint16_t)R[n] * (uint32_t)(uint16_t)R[m];
                    return 0;
                case 0xF: // MULS.W Rm,Rn
                    cpu->macl = (uint32_t)((int32_t)(int16_t)(uint16_t)R[n] *
                                           (int32_t)(int16_t)(uint16_t)R[m]);
                    return 0;
                default: return -4;
            }

        case 0x3:
            switch (instr & 0xF) {
                case 0x0: set_t(cpu, R[n] == R[m]); return 0; // CMP/EQ Rm,Rn
                case 0x2: set_t(cpu, R[n] >= R[m]); return 0; // CMP/HS Rm,Rn
                case 0x3: set_t(cpu, (int32_t)R[n] >= (int32_t)R[m]); return 0; // CMP/GE
                case 0x4: { // DIV1 Rm,Rn
                    uint32_t old_q = (cpu->sr & SR_Q) ? 1 : 0;
                    uint32_t M = (cpu->sr & SR_M) ? 1 : 0;
                    uint32_t q = (R[n] >> 31) & 1;
                    uint32_t tmp0, tmp1;
                    uint32_t tmp2 = R[m];
                    R[n] = (R[n] << 1) | (cpu->sr & SR_T);
                    if (old_q == 0) {
                        if (M == 0) {
                            tmp0 = R[n]; R[n] -= tmp2; tmp1 = (R[n] > tmp0);
                            q = q ? (tmp1 == 0) : tmp1;
                        } else {
                            tmp0 = R[n]; R[n] += tmp2; tmp1 = (R[n] < tmp0);
                            q = q ? tmp1 : (tmp1 == 0);
                        }
                    } else {
                        if (M == 0) {
                            tmp0 = R[n]; R[n] += tmp2; tmp1 = (R[n] < tmp0);
                            q = q ? (tmp1 == 0) : tmp1;
                        } else {
                            tmp0 = R[n]; R[n] -= tmp2; tmp1 = (R[n] > tmp0);
                            q = q ? tmp1 : (tmp1 == 0);
                        }
                    }
                    if (q) cpu->sr |= SR_Q; else cpu->sr &= ~SR_Q;
                    set_t(cpu, q == M);
                    return 0;
                }
                case 0x5: { // DMULU.L Rm,Rn
                    uint64_t prod = (uint64_t)R[n] * (uint64_t)R[m];
                    cpu->mach = (uint32_t)(prod >> 32);
                    cpu->macl = (uint32_t)prod;
                    return 0;
                }
                case 0x6: set_t(cpu, R[n] > R[m]); return 0; // CMP/HI Rm,Rn
                case 0x7: set_t(cpu, (int32_t)R[n] > (int32_t)R[m]); return 0; // CMP/GT
                case 0x8: R[n] -= R[m]; return 0; // SUB Rm,Rn
                case 0xA: { // SUBC Rm,Rn
                    uint32_t t = (cpu->sr & SR_T);
                    uint32_t tmp = R[n] - R[m];
                    uint32_t res = tmp - t;
                    set_t(cpu, (R[n] < R[m]) || (tmp < t));
                    R[n] = res;
                    return 0;
                }
                case 0xB: { // SUBV Rm,Rn
                    uint32_t res = R[n] - R[m];
                    set_t(cpu, (((R[n] ^ R[m]) & (R[n] ^ res)) >> 31) != 0);
                    R[n] = res;
                    return 0;
                }
                case 0xC: R[n] += R[m]; return 0; // ADD Rm,Rn
                case 0xD: { // DMULS.L Rm,Rn
                    int64_t prod = (int64_t)(int32_t)R[n] * (int64_t)(int32_t)R[m];
                    cpu->mach = (uint32_t)((uint64_t)prod >> 32);
                    cpu->macl = (uint32_t)(uint64_t)prod;
                    return 0;
                }
                case 0xE: { // ADDC Rm,Rn
                    uint32_t t = (cpu->sr & SR_T);
                    uint32_t tmp = R[n] + R[m];
                    uint32_t res = tmp + t;
                    set_t(cpu, (tmp < R[n]) || (res < tmp));
                    R[n] = res;
                    return 0;
                }
                case 0xF: { // ADDV Rm,Rn
                    uint32_t res = R[n] + R[m];
                    set_t(cpu, ((~(R[n] ^ R[m]) & (R[n] ^ res)) >> 31) != 0);
                    R[n] = res;
                    return 0;
                }
                default: return -4;
            }

        case 0x4:
            if ((instr & 0xF) == 0xF) { // MAC.W @Rm+,@Rn+
                int32_t a = (int32_t)(int16_t)(uint16_t)rd16(cpu, R[n]);
                R[n] += 2;
                int32_t b = (int32_t)(int16_t)(uint16_t)rd16(cpu, R[m]);
                R[m] += 2;
                mac_add(cpu, (int64_t)(a * b), 1);
                return 0;
            }
            switch (instr & 0xFF) {
                case 0x00: // SHLL Rn
                    set_t(cpu, (R[n] & 0x80000000u) != 0);
                    R[n] <<= 1;
                    return 0;
                case 0x01: // SHLR Rn
                    set_t(cpu, (R[n] & 1) != 0);
                    R[n] >>= 1;
                    return 0;
                case 0x02: R[n] -= 4; wr32(cpu, R[n], cpu->mach); return 0; // STS.L MACH,@-Rn
                case 0x03: R[n] -= 4; wr32(cpu, R[n], cpu->sr); return 0;   // STC.L SR,@-Rn
                case 0x04: { // ROTL Rn
                    uint32_t msb = (R[n] >> 31) & 1;
                    R[n] = (R[n] << 1) | msb;
                    set_t(cpu, msb != 0);
                    return 0;
                }
                case 0x05: { // ROTR Rn
                    uint32_t lsb = R[n] & 1;
                    R[n] = (R[n] >> 1) | (lsb << 31);
                    set_t(cpu, lsb != 0);
                    return 0;
                }
                case 0x06: cpu->mach = rd32(cpu, R[n]); R[n] += 4; return 0; // LDS.L @Rm+,MACH
                case 0x07: cpu->sr = rd32(cpu, R[n]) & SR_MASK; R[n] += 4; return 0; // LDC.L @Rm+,SR
                case 0x08: R[n] <<= 2; return 0;  // SHLL2 Rn
                case 0x09: R[n] >>= 2; return 0;  // SHLR2 Rn
                case 0x0A: cpu->mach = R[n]; return 0; // LDS Rm,MACH
                case 0x0B: // JSR @Rm
                    cpu->pr = addr + 4;
                    delayed_branch(cpu, R[n]);
                    return 0;
                case 0x0E: cpu->sr = R[n] & SR_MASK; return 0; // LDC Rm,SR
                case 0x10: // DT Rn
                    R[n] -= 1;
                    set_t(cpu, R[n] == 0);
                    return 0;
                case 0x11: set_t(cpu, (int32_t)R[n] >= 0); return 0; // CMP/PZ Rn
                case 0x12: R[n] -= 4; wr32(cpu, R[n], cpu->macl); return 0; // STS.L MACL,@-Rn
                case 0x13: R[n] -= 4; wr32(cpu, R[n], cpu->gbr); return 0;  // STC.L GBR,@-Rn
                case 0x15: set_t(cpu, (int32_t)R[n] > 0); return 0; // CMP/PL Rn
                case 0x16: cpu->macl = rd32(cpu, R[n]); R[n] += 4; return 0; // LDS.L @Rm+,MACL
                case 0x17: cpu->gbr = rd32(cpu, R[n]); R[n] += 4; return 0;  // LDC.L @Rm+,GBR
                case 0x18: R[n] <<= 8; return 0;  // SHLL8 Rn
                case 0x19: R[n] >>= 8; return 0;  // SHLR8 Rn
                case 0x1A: cpu->macl = R[n]; return 0; // LDS Rm,MACL
                case 0x1B: { // TAS.B @Rn
                    uint32_t val = rd8(cpu, R[n]);
                    set_t(cpu, val == 0);
                    wr8(cpu, R[n], val | 0x80u);
                    return 0;
                }
                case 0x1E: cpu->gbr = R[n]; return 0; // LDC Rm,GBR
                case 0x20: // SHAL Rn
                    set_t(cpu, (R[n] & 0x80000000u) != 0);
                    R[n] <<= 1;
                    return 0;
                case 0x21: // SHAR Rn
                    set_t(cpu, (R[n] & 1) != 0);
                    R[n] = (uint32_t)((int32_t)R[n] >> 1);
                    return 0;
                case 0x22: R[n] -= 4; wr32(cpu, R[n], cpu->pr); return 0;  // STS.L PR,@-Rn
                case 0x23: R[n] -= 4; wr32(cpu, R[n], cpu->vbr); return 0; // STC.L VBR,@-Rn
                case 0x24: { // ROTCL Rn
                    uint32_t msb = (R[n] >> 31) & 1;
                    R[n] = (R[n] << 1) | (cpu->sr & SR_T);
                    set_t(cpu, msb != 0);
                    return 0;
                }
                case 0x25: { // ROTCR Rn
                    uint32_t lsb = R[n] & 1;
                    R[n] = (R[n] >> 1) | ((cpu->sr & SR_T) << 31);
                    set_t(cpu, lsb != 0);
                    return 0;
                }
                case 0x26: cpu->pr = rd32(cpu, R[n]); R[n] += 4; return 0;  // LDS.L @Rm+,PR
                case 0x27: cpu->vbr = rd32(cpu, R[n]); R[n] += 4; return 0; // LDC.L @Rm+,VBR
                case 0x28: R[n] <<= 16; return 0; // SHLL16 Rn
                case 0x29: R[n] >>= 16; return 0; // SHLR16 Rn
                case 0x2A: cpu->pr = R[n]; return 0; // LDS Rm,PR
                case 0x2B: // JMP @Rm
                    delayed_branch(cpu, R[n]);
                    return 0;
                case 0x2E: cpu->vbr = R[n]; return 0; // LDC Rm,VBR
                default: return -4;
            }

        case 0x5: // MOV.L @(disp*4,Rm),Rn
            R[n] = rd32(cpu, R[m] + d4 * 4);
            return 0;

        case 0x6:
            switch (instr & 0xF) {
                case 0x0: R[n] = sext8(rd8(cpu, R[m])); return 0;   // MOV.B @Rm,Rn
                case 0x1: R[n] = sext16(rd16(cpu, R[m])); return 0; // MOV.W @Rm,Rn
                case 0x2: R[n] = rd32(cpu, R[m]); return 0;         // MOV.L @Rm,Rn
                case 0x3: R[n] = R[m]; return 0;                    // MOV Rm,Rn
                case 0x4: // MOV.B @Rm+,Rn
                    R[n] = sext8(rd8(cpu, R[m]));
                    if (n != m) R[m] += 1;
                    return 0;
                case 0x5: // MOV.W @Rm+,Rn
                    R[n] = sext16(rd16(cpu, R[m]));
                    if (n != m) R[m] += 2;
                    return 0;
                case 0x6: // MOV.L @Rm+,Rn
                    R[n] = rd32(cpu, R[m]);
                    if (n != m) R[m] += 4;
                    return 0;
                case 0x7: R[n] = ~R[m]; return 0; // NOT Rm,Rn
                case 0x8: // SWAP.B Rm,Rn (swap two low bytes)
                    R[n] = (R[m] & 0xFFFF0000u) | ((R[m] & 0xFFu) << 8) | ((R[m] >> 8) & 0xFFu);
                    return 0;
                case 0x9: R[n] = (R[m] << 16) | (R[m] >> 16); return 0; // SWAP.W Rm,Rn
                case 0xA: { // NEGC Rm,Rn
                    uint32_t tmp = 0u - R[m];
                    uint32_t t = (cpu->sr & SR_T);
                    uint32_t res = tmp - t;
                    set_t(cpu, (0 < R[m]) || (tmp < t));
                    R[n] = res;
                    return 0;
                }
                case 0xB: R[n] = 0u - R[m]; return 0; // NEG Rm,Rn
                case 0xC: R[n] = R[m] & 0xFFu; return 0;    // EXTU.B Rm,Rn
                case 0xD: R[n] = R[m] & 0xFFFFu; return 0;  // EXTU.W Rm,Rn
                case 0xE: R[n] = sext8(R[m]); return 0;     // EXTS.B Rm,Rn
                case 0xF: R[n] = sext16(R[m]); return 0;    // EXTS.W Rm,Rn
                default: return -4;
            }

        case 0x7: // ADD #imm,Rn
            R[n] += sext8(d8);
            return 0;

        case 0x8:
            switch (n) { // n field holds the sub-opcode here
                case 0x0: wr8(cpu, R[m] + d4, R[0]); return 0;      // MOV.B R0,@(disp,Rn)
                case 0x1: wr16(cpu, R[m] + d4 * 2, R[0]); return 0; // MOV.W R0,@(disp*2,Rn)
                case 0x4: R[0] = sext8(rd8(cpu, R[m] + d4)); return 0;       // MOV.B @(disp,Rm),R0
                case 0x5: R[0] = sext16(rd16(cpu, R[m] + d4 * 2)); return 0; // MOV.W @(disp*2,Rm),R0
                case 0x8: set_t(cpu, R[0] == sext8(d8)); return 0; // CMP/EQ #imm,R0
                case 0x9: // BT disp
                    if (cpu->sr & SR_T) cpu->pc = addr + 4 + sext8(d8) * 2;
                    return 0;
                case 0xB: // BF disp
                    if (!(cpu->sr & SR_T)) cpu->pc = addr + 4 + sext8(d8) * 2;
                    return 0;
                case 0xD: // BT/S disp
                    if (cpu->sr & SR_T) delayed_branch(cpu, addr + 4 + sext8(d8) * 2);
                    return 0;
                case 0xF: // BF/S disp
                    if (!(cpu->sr & SR_T)) delayed_branch(cpu, addr + 4 + sext8(d8) * 2);
                    return 0;
                default: return -4;
            }

        case 0x9: // MOV.W @(disp*2,PC),Rn
            R[n] = sext16(rd16(cpu, addr + 4 + d8 * 2));
            return 0;

        case 0xA: { // BRA disp12
            int32_t disp = (int32_t)(instr & 0xFFF);
            if (disp & 0x800) disp -= 0x1000;
            delayed_branch(cpu, addr + 4 + (uint32_t)(disp * 2));
            return 0;
        }

        case 0xB: { // BSR disp12
            int32_t disp = (int32_t)(instr & 0xFFF);
            if (disp & 0x800) disp -= 0x1000;
            cpu->pr = addr + 4;
            delayed_branch(cpu, addr + 4 + (uint32_t)(disp * 2));
            return 0;
        }

        case 0xC:
            switch (n) {
                case 0x0: wr8(cpu, cpu->gbr + d8, R[0]); return 0;      // MOV.B R0,@(disp,GBR)
                case 0x1: wr16(cpu, cpu->gbr + d8 * 2, R[0]); return 0; // MOV.W R0,@(disp*2,GBR)
                case 0x2: wr32(cpu, cpu->gbr + d8 * 4, R[0]); return 0; // MOV.L R0,@(disp*4,GBR)
                case 0x3: // TRAPA #imm (halts the emulator)
                    cpu->halted = 1;
                    return 1;
                case 0x4: R[0] = sext8(rd8(cpu, cpu->gbr + d8)); return 0;       // MOV.B @(disp,GBR),R0
                case 0x5: R[0] = sext16(rd16(cpu, cpu->gbr + d8 * 2)); return 0; // MOV.W @(disp*2,GBR),R0
                case 0x6: R[0] = rd32(cpu, cpu->gbr + d8 * 4); return 0;         // MOV.L @(disp*4,GBR),R0
                case 0x7: // MOVA @(disp*4,PC),R0
                    R[0] = ((addr + 4) & ~3u) + d8 * 4;
                    return 0;
                case 0x8: set_t(cpu, (R[0] & d8) == 0); return 0; // TST #imm,R0
                case 0x9: R[0] &= d8; return 0; // AND #imm,R0
                case 0xA: R[0] ^= d8; return 0; // XOR #imm,R0
                case 0xB: R[0] |= d8; return 0; // OR #imm,R0
                case 0xC: // TST.B #imm,@(R0,GBR)
                    set_t(cpu, (rd8(cpu, cpu->gbr + R[0]) & d8) == 0);
                    return 0;
                case 0xD: { // AND.B #imm,@(R0,GBR)
                    uint32_t a = cpu->gbr + R[0];
                    wr8(cpu, a, rd8(cpu, a) & d8);
                    return 0;
                }
                case 0xE: { // XOR.B #imm,@(R0,GBR)
                    uint32_t a = cpu->gbr + R[0];
                    wr8(cpu, a, rd8(cpu, a) ^ d8);
                    return 0;
                }
                case 0xF: { // OR.B #imm,@(R0,GBR)
                    uint32_t a = cpu->gbr + R[0];
                    wr8(cpu, a, rd8(cpu, a) | d8);
                    return 0;
                }
                default: return -4;
            }

        case 0xD: // MOV.L @(disp*4,PC),Rn
            R[n] = rd32(cpu, ((addr + 4) & ~3u) + d8 * 4);
            return 0;

        case 0xE: // MOV #imm,Rn
            R[n] = sext8(d8);
            return 0;

        default: // 0xF: no FPU on the base SH-2
            return -4;
    }
}

int sh2_step(void *context) {
    if (!context) return -1;
    SH2CPU *cpu = (SH2CPU*)context;

    if (cpu->halted) return 1;

    uint32_t instr_addr = cpu->pc & ~1u;
    uint16_t instr = (uint16_t)rd16(cpu, instr_addr);
    cpu->pc = instr_addr + 2;
    cpu->ticks++;

    int ret = exec_instr(cpu, instr, instr_addr);
    if (ret != 0) return ret;

    if (cpu->delay_pending) {
        uint32_t target = cpu->delay_target;
        cpu->delay_pending = 0;

        // Execute the delay slot instruction before taking the branch
        uint32_t slot_addr = cpu->pc & ~1u;
        uint16_t slot = (uint16_t)rd16(cpu, slot_addr);
        cpu->pc = slot_addr + 2;
        cpu->ticks++;

        ret = exec_instr(cpu, slot, slot_addr);
        cpu->delay_pending = 0; // branches in delay slots are illegal; ignore them
        cpu->pc = target;
        if (ret != 0) return ret;

        // Branch-to-self infinite loop: treat as halt
        if (target == instr_addr) {
            cpu->halted = 1;
            return 1;
        }
    }

    return 0;
}

void sh2_print_state(void *context) {
    if (!context) return;
    SH2CPU *cpu = (SH2CPU*)context;

    printf("Hitachi SH-2 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC:   0x%08X  Halted: %s\n", cpu->pc, cpu->halted ? "Yes" : "No");
    printf("  SR:   0x%08X [M=%d Q=%d S=%d T=%d]\n", cpu->sr,
           (cpu->sr & SR_M) ? 1 : 0, (cpu->sr & SR_Q) ? 1 : 0,
           (cpu->sr & SR_S) ? 1 : 0, (cpu->sr & SR_T) ? 1 : 0);
    printf("  PR:   0x%08X  GBR: 0x%08X  VBR: 0x%08X\n", cpu->pr, cpu->gbr, cpu->vbr);
    printf("  MACH: 0x%08X  MACL: 0x%08X\n", cpu->mach, cpu->macl);
    printf("  Registers:\n");
    for (int i = 0; i < 16; ++i) {
        char name[8];
        if (i == 15) snprintf(name, sizeof(name), "r15/sp");
        else snprintf(name, sizeof(name), "r%d", i);
        printf("    %-6s: 0x%08X%s", name, cpu->r[i], (i % 4 == 3) ? "\n" : "  ");
    }
}

void sh2_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    SH2CPU *cpu = (SH2CPU*)context;

    uint32_t addr = cpu->pc & ~1u;
    uint16_t instr = (uint16_t)rd16(cpu, addr);
    uint32_t n = (instr >> 8) & 0xF;
    uint32_t m = (instr >> 4) & 0xF;
    uint32_t d8 = instr & 0xFF;
    uint32_t d4 = instr & 0xF;
    int32_t sd8 = (int32_t)sext8(d8);
    static const char *sz[] = {"b", "w", "l"};

    switch (instr >> 12) {
        case 0x0:
            switch (instr & 0xF) {
                case 0x2: {
                    static const char *cr[] = {"sr", "gbr", "vbr"};
                    if (m <= 2) { snprintf(buf, buf_len, "stc   %s, r%u", cr[m], n); return; }
                    break;
                }
                case 0x3:
                    if (m == 0) { snprintf(buf, buf_len, "bsrf  r%u", n); return; }
                    if (m == 2) { snprintf(buf, buf_len, "braf  r%u", n); return; }
                    break;
                case 0x4: case 0x5: case 0x6:
                    snprintf(buf, buf_len, "mov.%s r%u, @(r0,r%u)", sz[(instr & 0xF) - 4], m, n);
                    return;
                case 0x7: snprintf(buf, buf_len, "mul.l r%u, r%u", m, n); return;
                case 0x8:
                    if (m == 0) { snprintf(buf, buf_len, "clrt"); return; }
                    if (m == 1) { snprintf(buf, buf_len, "sett"); return; }
                    if (m == 2) { snprintf(buf, buf_len, "clrmac"); return; }
                    break;
                case 0x9:
                    if (m == 0) { snprintf(buf, buf_len, "nop"); return; }
                    if (m == 1) { snprintf(buf, buf_len, "div0u"); return; }
                    if (m == 2) { snprintf(buf, buf_len, "movt  r%u", n); return; }
                    break;
                case 0xA: {
                    static const char *sysr[] = {"mach", "macl", "pr"};
                    if (m <= 2) { snprintf(buf, buf_len, "sts   %s, r%u", sysr[m], n); return; }
                    break;
                }
                case 0xB:
                    if (m == 0) { snprintf(buf, buf_len, "rts"); return; }
                    if (m == 1) { snprintf(buf, buf_len, "sleep"); return; }
                    if (m == 2) { snprintf(buf, buf_len, "rte"); return; }
                    break;
                case 0xC: case 0xD: case 0xE:
                    snprintf(buf, buf_len, "mov.%s @(r0,r%u), r%u", sz[(instr & 0xF) - 0xC], m, n);
                    return;
                case 0xF: snprintf(buf, buf_len, "mac.l @r%u+, @r%u+", m, n); return;
                default: break;
            }
            break;

        case 0x1:
            snprintf(buf, buf_len, "mov.l r%u, @(%u,r%u)", m, d4 * 4, n);
            return;

        case 0x2: {
            static const char *ops[] = {
                "mov.b", "mov.w", "mov.l", NULL, "mov.b", "mov.w", "mov.l", "div0s",
                "tst", "and", "xor", "or", "cmp/str", "xtrct", "mulu.w", "muls.w"
            };
            uint32_t lo = instr & 0xF;
            if (lo <= 2) { snprintf(buf, buf_len, "%s r%u, @r%u", ops[lo], m, n); return; }
            if (lo >= 4 && lo <= 6) { snprintf(buf, buf_len, "%s r%u, @-r%u", ops[lo], m, n); return; }
            if (lo >= 7) { snprintf(buf, buf_len, "%-5s r%u, r%u", ops[lo], m, n); return; }
            break;
        }

        case 0x3: {
            static const char *ops[] = {
                "cmp/eq", NULL, "cmp/hs", "cmp/ge", "div1", "dmulu.l", "cmp/hi", "cmp/gt",
                "sub", NULL, "subc", "subv", "add", "dmuls.l", "addc", "addv"
            };
            uint32_t lo = instr & 0xF;
            if (ops[lo]) { snprintf(buf, buf_len, "%-5s r%u, r%u", ops[lo], m, n); return; }
            break;
        }

        case 0x4:
            if ((instr & 0xF) == 0xF) {
                snprintf(buf, buf_len, "mac.w @r%u+, @r%u+", m, n);
                return;
            }
            switch (instr & 0xFF) {
                case 0x00: snprintf(buf, buf_len, "shll  r%u", n); return;
                case 0x01: snprintf(buf, buf_len, "shlr  r%u", n); return;
                case 0x02: snprintf(buf, buf_len, "sts.l mach, @-r%u", n); return;
                case 0x03: snprintf(buf, buf_len, "stc.l sr, @-r%u", n); return;
                case 0x04: snprintf(buf, buf_len, "rotl  r%u", n); return;
                case 0x05: snprintf(buf, buf_len, "rotr  r%u", n); return;
                case 0x06: snprintf(buf, buf_len, "lds.l @r%u+, mach", n); return;
                case 0x07: snprintf(buf, buf_len, "ldc.l @r%u+, sr", n); return;
                case 0x08: snprintf(buf, buf_len, "shll2 r%u", n); return;
                case 0x09: snprintf(buf, buf_len, "shlr2 r%u", n); return;
                case 0x0A: snprintf(buf, buf_len, "lds   r%u, mach", n); return;
                case 0x0B: snprintf(buf, buf_len, "jsr   @r%u", n); return;
                case 0x0E: snprintf(buf, buf_len, "ldc   r%u, sr", n); return;
                case 0x10: snprintf(buf, buf_len, "dt    r%u", n); return;
                case 0x11: snprintf(buf, buf_len, "cmp/pz r%u", n); return;
                case 0x12: snprintf(buf, buf_len, "sts.l macl, @-r%u", n); return;
                case 0x13: snprintf(buf, buf_len, "stc.l gbr, @-r%u", n); return;
                case 0x15: snprintf(buf, buf_len, "cmp/pl r%u", n); return;
                case 0x16: snprintf(buf, buf_len, "lds.l @r%u+, macl", n); return;
                case 0x17: snprintf(buf, buf_len, "ldc.l @r%u+, gbr", n); return;
                case 0x18: snprintf(buf, buf_len, "shll8 r%u", n); return;
                case 0x19: snprintf(buf, buf_len, "shlr8 r%u", n); return;
                case 0x1A: snprintf(buf, buf_len, "lds   r%u, macl", n); return;
                case 0x1B: snprintf(buf, buf_len, "tas.b @r%u", n); return;
                case 0x1E: snprintf(buf, buf_len, "ldc   r%u, gbr", n); return;
                case 0x20: snprintf(buf, buf_len, "shal  r%u", n); return;
                case 0x21: snprintf(buf, buf_len, "shar  r%u", n); return;
                case 0x22: snprintf(buf, buf_len, "sts.l pr, @-r%u", n); return;
                case 0x23: snprintf(buf, buf_len, "stc.l vbr, @-r%u", n); return;
                case 0x24: snprintf(buf, buf_len, "rotcl r%u", n); return;
                case 0x25: snprintf(buf, buf_len, "rotcr r%u", n); return;
                case 0x26: snprintf(buf, buf_len, "lds.l @r%u+, pr", n); return;
                case 0x27: snprintf(buf, buf_len, "ldc.l @r%u+, vbr", n); return;
                case 0x28: snprintf(buf, buf_len, "shll16 r%u", n); return;
                case 0x29: snprintf(buf, buf_len, "shlr16 r%u", n); return;
                case 0x2A: snprintf(buf, buf_len, "lds   r%u, pr", n); return;
                case 0x2B: snprintf(buf, buf_len, "jmp   @r%u", n); return;
                case 0x2E: snprintf(buf, buf_len, "ldc   r%u, vbr", n); return;
                default: break;
            }
            break;

        case 0x5:
            snprintf(buf, buf_len, "mov.l @(%u,r%u), r%u", d4 * 4, m, n);
            return;

        case 0x6: {
            static const char *ops[] = {
                "mov.b", "mov.w", "mov.l", "mov", "mov.b", "mov.w", "mov.l", "not",
                "swap.b", "swap.w", "negc", "neg", "extu.b", "extu.w", "exts.b", "exts.w"
            };
            uint32_t lo = instr & 0xF;
            if (lo <= 2) { snprintf(buf, buf_len, "%s @r%u, r%u", ops[lo], m, n); return; }
            if (lo >= 4 && lo <= 6) { snprintf(buf, buf_len, "%s @r%u+, r%u", ops[lo], m, n); return; }
            snprintf(buf, buf_len, "%-5s r%u, r%u", ops[lo], m, n);
            return;
        }

        case 0x7:
            snprintf(buf, buf_len, "add   #%d, r%u", sd8, n);
            return;

        case 0x8:
            switch (n) {
                case 0x0: snprintf(buf, buf_len, "mov.b r0, @(%u,r%u)", d4, m); return;
                case 0x1: snprintf(buf, buf_len, "mov.w r0, @(%u,r%u)", d4 * 2, m); return;
                case 0x4: snprintf(buf, buf_len, "mov.b @(%u,r%u), r0", d4, m); return;
                case 0x5: snprintf(buf, buf_len, "mov.w @(%u,r%u), r0", d4 * 2, m); return;
                case 0x8: snprintf(buf, buf_len, "cmp/eq #%d, r0", sd8); return;
                case 0x9: snprintf(buf, buf_len, "bt    0x%X", addr + 4 + (uint32_t)(sd8 * 2)); return;
                case 0xB: snprintf(buf, buf_len, "bf    0x%X", addr + 4 + (uint32_t)(sd8 * 2)); return;
                case 0xD: snprintf(buf, buf_len, "bt/s  0x%X", addr + 4 + (uint32_t)(sd8 * 2)); return;
                case 0xF: snprintf(buf, buf_len, "bf/s  0x%X", addr + 4 + (uint32_t)(sd8 * 2)); return;
                default: break;
            }
            break;

        case 0x9:
            snprintf(buf, buf_len, "mov.w @(0x%X,pc), r%u ; =0x%X", d8 * 2, n, addr + 4 + d8 * 2);
            return;

        case 0xA: case 0xB: {
            int32_t disp = (int32_t)(instr & 0xFFF);
            if (disp & 0x800) disp -= 0x1000;
            snprintf(buf, buf_len, "%s   0x%X", (instr >> 12) == 0xA ? "bra" : "bsr",
                     addr + 4 + (uint32_t)(disp * 2));
            return;
        }

        case 0xC:
            switch (n) {
                case 0x0: snprintf(buf, buf_len, "mov.b r0, @(%u,gbr)", d8); return;
                case 0x1: snprintf(buf, buf_len, "mov.w r0, @(%u,gbr)", d8 * 2); return;
                case 0x2: snprintf(buf, buf_len, "mov.l r0, @(%u,gbr)", d8 * 4); return;
                case 0x3: snprintf(buf, buf_len, "trapa #%u", d8); return;
                case 0x4: snprintf(buf, buf_len, "mov.b @(%u,gbr), r0", d8); return;
                case 0x5: snprintf(buf, buf_len, "mov.w @(%u,gbr), r0", d8 * 2); return;
                case 0x6: snprintf(buf, buf_len, "mov.l @(%u,gbr), r0", d8 * 4); return;
                case 0x7: snprintf(buf, buf_len, "mova  @(0x%X,pc), r0 ; =0x%X",
                                   d8 * 4, ((addr + 4) & ~3u) + d8 * 4); return;
                case 0x8: snprintf(buf, buf_len, "tst   #%u, r0", d8); return;
                case 0x9: snprintf(buf, buf_len, "and   #%u, r0", d8); return;
                case 0xA: snprintf(buf, buf_len, "xor   #%u, r0", d8); return;
                case 0xB: snprintf(buf, buf_len, "or    #%u, r0", d8); return;
                case 0xC: snprintf(buf, buf_len, "tst.b #%u, @(r0,gbr)", d8); return;
                case 0xD: snprintf(buf, buf_len, "and.b #%u, @(r0,gbr)", d8); return;
                case 0xE: snprintf(buf, buf_len, "xor.b #%u, @(r0,gbr)", d8); return;
                case 0xF: snprintf(buf, buf_len, "or.b  #%u, @(r0,gbr)", d8); return;
                default: break;
            }
            break;

        case 0xD:
            snprintf(buf, buf_len, "mov.l @(0x%X,pc), r%u ; =0x%X",
                     d8 * 4, n, ((addr + 4) & ~3u) + d8 * 4);
            return;

        case 0xE:
            snprintf(buf, buf_len, "mov   #%d, r%u", sd8, n);
            return;

        default:
            break;
    }

    snprintf(buf, buf_len, "unknown (0x%04X)", instr);
}
