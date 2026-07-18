#include "i8048.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ROM_SIZE 4096 // 4 KB program memory (two 2 KB banks, 12-bit PC)
#define RAM_SIZE 64   // 64-byte internal data RAM

// Data RAM layout:
//   0-7   : register bank 0 (R0-R7)
//   8-23  : 8-level stack (2 bytes per level)
//   24-31 : register bank 1 (R0-R7)
//   32-63 : general purpose scratch
#define STACK_BASE 8
#define RB1_BASE 24

typedef struct I8048CPU {
    uint8_t a;       // Accumulator
    uint16_t pc;     // 12-bit Program Counter

    // PSW components (CY AC F0 BS 1 SP2 SP1 SP0)
    uint8_t cy;      // Carry
    uint8_t ac;      // Auxiliary carry
    uint8_t f0;      // Flag 0
    uint8_t bs;      // Register bank select (0 = RB0, 1 = RB1)
    uint8_t sp;      // Stack pointer (0-7)

    uint8_t f1;      // Flag 1 (not part of PSW)
    uint8_t mb;      // Memory bank select latch (SEL MB0/MB1), applied on JMP/CALL

    // Timer / counter
    uint8_t timer;       // T register
    uint8_t timer_run;   // 0 = stopped, 1 = timer mode, 2 = counter mode
    uint8_t timer_flag;  // TF, set on timer overflow
    uint8_t prescaler;   // /32 prescaler for timer mode

    // Interrupt enable latches (no external interrupt sources modelled)
    uint8_t int_enabled;   // EN I / DIS I
    uint8_t tcnti_enabled; // EN TCNTI / DIS TCNTI

    // I/O ports (plain latches)
    uint8_t p1;
    uint8_t p2;
    uint8_t bus;

    // Test inputs (T0/T1 pins, held low)
    uint8_t t0;
    uint8_t t1;

    uint8_t rom[ROM_SIZE];
    uint8_t ram[RAM_SIZE];
    uint32_t ticks;
    int halted;
} I8048CPU;

static inline uint8_t reg_base(const I8048CPU *cpu) {
    return cpu->bs ? RB1_BASE : 0;
}

static inline uint8_t get_r(const I8048CPU *cpu, uint8_t r) {
    return cpu->ram[reg_base(cpu) + (r & 7)];
}

static inline void set_r(I8048CPU *cpu, uint8_t r, uint8_t val) {
    cpu->ram[reg_base(cpu) + (r & 7)] = val;
}

static inline uint8_t ind_addr(const I8048CPU *cpu, uint8_t r) {
    return get_r(cpu, r) & 0x3F;
}

static inline uint8_t get_psw(const I8048CPU *cpu) {
    return (uint8_t)((cpu->cy << 7) | (cpu->ac << 6) | (cpu->f0 << 5) |
                     (cpu->bs << 4) | 0x08 | (cpu->sp & 7));
}

static inline void set_psw(I8048CPU *cpu, uint8_t val) {
    cpu->cy = (val >> 7) & 1;
    cpu->ac = (val >> 6) & 1;
    cpu->f0 = (val >> 5) & 1;
    cpu->bs = (val >> 4) & 1;
    cpu->sp = val & 7;
}

static inline void do_add(I8048CPU *cpu, uint8_t val, uint8_t carry_in) {
    uint16_t sum = (uint16_t)cpu->a + val + carry_in;
    cpu->ac = (((cpu->a & 0x0F) + (val & 0x0F) + carry_in) > 0x0F) ? 1 : 0;
    cpu->cy = (sum > 0xFF) ? 1 : 0;
    cpu->a = sum & 0xFF;
}

static inline void push_pc(I8048CPU *cpu, uint16_t ret_addr) {
    uint8_t base = STACK_BASE + (uint8_t)((cpu->sp & 7) * 2);
    cpu->ram[base] = ret_addr & 0xFF;
    cpu->ram[base + 1] = (uint8_t)(((ret_addr >> 8) & 0x0F) | (get_psw(cpu) & 0xF0));
    cpu->sp = (cpu->sp + 1) & 7;
}

static inline uint16_t pop_pc(I8048CPU *cpu, int restore_psw) {
    cpu->sp = (cpu->sp - 1) & 7;
    uint8_t base = STACK_BASE + (uint8_t)((cpu->sp & 7) * 2);
    uint8_t hi = cpu->ram[base + 1];
    if (restore_psw) {
        cpu->cy = (hi >> 7) & 1;
        cpu->ac = (hi >> 6) & 1;
        cpu->f0 = (hi >> 5) & 1;
        cpu->bs = (hi >> 4) & 1;
    }
    return (uint16_t)(((hi & 0x0F) << 8) | cpu->ram[base]);
}

static void timer_advance(I8048CPU *cpu, uint8_t cycles) {
    if (cpu->timer_run != 1) return; // counter mode has no modelled T1 events
    cpu->prescaler = (uint8_t)(cpu->prescaler + cycles);
    while (cpu->prescaler >= 32) {
        cpu->prescaler -= 32;
        cpu->timer++;
        if (cpu->timer == 0) {
            cpu->timer_flag = 1;
        }
    }
}

// Instruction length in bytes (1 or 2)
static uint8_t instr_len(uint8_t op) {
    if ((op & 0x1F) == 0x04) return 2; // JMP addr
    if ((op & 0x1F) == 0x14) return 2; // CALL addr
    if ((op & 0x1F) == 0x12) return 2; // JBb addr
    if ((op & 0xF8) == 0xE8) return 2; // DJNZ Rr, addr
    if ((op & 0xFE) == 0xB0) return 2; // MOV @Ri, #imm
    if ((op & 0xF8) == 0xB8) return 2; // MOV Rr, #imm
    switch (op) {
        case 0x03: case 0x13: case 0x23: case 0x43: case 0x53: case 0xD3: // ALU A, #imm
        case 0x88: case 0x89: case 0x8A: case 0x98: case 0x99: case 0x9A: // ORL/ANL port, #imm
        case 0x16: case 0x26: case 0x36: case 0x46: case 0x56: case 0x76:
        case 0x86: case 0x96: case 0xB6: case 0xC6: case 0xE6: case 0xF6: // conditional jumps
            return 2;
        default:
            return 1;
    }
}

// Approximate machine cycles (for the /32 timer prescaler)
static uint8_t instr_cycles(uint8_t op, uint8_t len) {
    if (len == 2) return 2;
    switch (op) {
        case 0x02: case 0x08: case 0x09: case 0x0A: // OUTL BUS,A / INS/IN A
        case 0x39: case 0x3A:                       // OUTL P1/P2, A
        case 0x83: case 0x93:                       // RET / RETR
        case 0xA3: case 0xB3: case 0xE3:            // MOVP / JMPP / MOVP3
            return 2;
        default:
            return 1;
    }
}

void* i8048_create(void) {
    I8048CPU *cpu = (I8048CPU*)calloc(1, sizeof(I8048CPU));
    return cpu;
}

void i8048_destroy(void *context) {
    free(context);
}

int i8048_init(void *context) {
    if (!context) return -1;
    I8048CPU *cpu = (I8048CPU*)context;

    uint8_t rom_copy[ROM_SIZE];
    memcpy(rom_copy, cpu->rom, ROM_SIZE);
    memset(cpu, 0, sizeof(I8048CPU));
    memcpy(cpu->rom, rom_copy, ROM_SIZE);
    return 0;
}

int i8048_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    I8048CPU *cpu = (I8048CPU*)context;

    if (address >= ROM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > ROM_SIZE) {
        copy_len = ROM_SIZE - address;
    }
    memcpy(&cpu->rom[address], data, copy_len);
    return 0;
}

int i8048_step(void *context) {
    if (!context) return -1;
    I8048CPU *cpu = (I8048CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc & 0x0FFF;
    uint8_t op = cpu->rom[instr_pc];
    uint8_t len = instr_len(op);
    uint8_t byte2 = (len == 2) ? cpu->rom[(instr_pc + 1) & 0x0FFF] : 0;
    uint16_t next_pc = (instr_pc + len) & 0x0FFF;
    uint8_t cycles = instr_cycles(op, len);

    cpu->pc = next_pc;
    cpu->ticks += cycles;
    timer_advance(cpu, cycles);

    // In-page target for conditional jumps / DJNZ (page of the next instruction)
    uint16_t page_target = (uint16_t)((next_pc & 0x0F00) | byte2);

    if ((op & 0x1F) == 0x04) {
        // JMP addr (page bits in opcode, bank bit from MB latch)
        cpu->pc = (uint16_t)((((uint16_t)op & 0xE0) << 3) | byte2 | ((uint16_t)cpu->mb << 11));
    }
    else if ((op & 0x1F) == 0x14) {
        // CALL addr
        push_pc(cpu, next_pc);
        cpu->pc = (uint16_t)((((uint16_t)op & 0xE0) << 3) | byte2 | ((uint16_t)cpu->mb << 11));
    }
    else if ((op & 0x1F) == 0x12) {
        // JBb addr (jump if accumulator bit b set)
        uint8_t bit = (op >> 5) & 7;
        if (cpu->a & (1 << bit)) cpu->pc = page_target;
    }
    else if ((op & 0xF8) == 0xE8) {
        // DJNZ Rr, addr
        uint8_t val = (uint8_t)(get_r(cpu, op & 7) - 1);
        set_r(cpu, op & 7, val);
        if (val != 0) cpu->pc = page_target;
    }
    else if ((op & 0xF8) == 0x68) {
        do_add(cpu, get_r(cpu, op & 7), 0); // ADD A, Rr
    }
    else if ((op & 0xFE) == 0x60) {
        do_add(cpu, cpu->ram[ind_addr(cpu, op & 1)], 0); // ADD A, @Ri
    }
    else if ((op & 0xF8) == 0x78) {
        do_add(cpu, get_r(cpu, op & 7), cpu->cy); // ADDC A, Rr
    }
    else if ((op & 0xFE) == 0x70) {
        do_add(cpu, cpu->ram[ind_addr(cpu, op & 1)], cpu->cy); // ADDC A, @Ri
    }
    else if ((op & 0xF8) == 0x58) {
        cpu->a &= get_r(cpu, op & 7); // ANL A, Rr
    }
    else if ((op & 0xFE) == 0x50) {
        cpu->a &= cpu->ram[ind_addr(cpu, op & 1)]; // ANL A, @Ri
    }
    else if ((op & 0xF8) == 0x48) {
        cpu->a |= get_r(cpu, op & 7); // ORL A, Rr
    }
    else if ((op & 0xFE) == 0x40) {
        cpu->a |= cpu->ram[ind_addr(cpu, op & 1)]; // ORL A, @Ri
    }
    else if ((op & 0xF8) == 0xD8) {
        cpu->a ^= get_r(cpu, op & 7); // XRL A, Rr
    }
    else if ((op & 0xFE) == 0xD0) {
        cpu->a ^= cpu->ram[ind_addr(cpu, op & 1)]; // XRL A, @Ri
    }
    else if ((op & 0xF8) == 0x18) {
        set_r(cpu, op & 7, (uint8_t)(get_r(cpu, op & 7) + 1)); // INC Rr
    }
    else if ((op & 0xFE) == 0x10) {
        uint8_t addr = ind_addr(cpu, op & 1); // INC @Ri
        cpu->ram[addr]++;
    }
    else if ((op & 0xF8) == 0xC8) {
        set_r(cpu, op & 7, (uint8_t)(get_r(cpu, op & 7) - 1)); // DEC Rr
    }
    else if ((op & 0xF8) == 0x28) {
        // XCH A, Rr
        uint8_t tmp = cpu->a;
        cpu->a = get_r(cpu, op & 7);
        set_r(cpu, op & 7, tmp);
    }
    else if ((op & 0xFE) == 0x20) {
        // XCH A, @Ri
        uint8_t addr = ind_addr(cpu, op & 1);
        uint8_t tmp = cpu->a;
        cpu->a = cpu->ram[addr];
        cpu->ram[addr] = tmp;
    }
    else if ((op & 0xFE) == 0x30) {
        // XCHD A, @Ri (exchange low nibbles)
        uint8_t addr = ind_addr(cpu, op & 1);
        uint8_t tmp = cpu->a & 0x0F;
        cpu->a = (cpu->a & 0xF0) | (cpu->ram[addr] & 0x0F);
        cpu->ram[addr] = (cpu->ram[addr] & 0xF0) | tmp;
    }
    else if ((op & 0xF8) == 0xF8) {
        cpu->a = get_r(cpu, op & 7); // MOV A, Rr
    }
    else if ((op & 0xFE) == 0xF0) {
        cpu->a = cpu->ram[ind_addr(cpu, op & 1)]; // MOV A, @Ri
    }
    else if ((op & 0xF8) == 0xA8) {
        set_r(cpu, op & 7, cpu->a); // MOV Rr, A
    }
    else if ((op & 0xFE) == 0xA0) {
        cpu->ram[ind_addr(cpu, op & 1)] = cpu->a; // MOV @Ri, A
    }
    else if ((op & 0xF8) == 0xB8) {
        set_r(cpu, op & 7, byte2); // MOV Rr, #imm
    }
    else if ((op & 0xFE) == 0xB0) {
        cpu->ram[ind_addr(cpu, op & 1)] = byte2; // MOV @Ri, #imm
    }
    else {
        switch (op) {
            case 0x00: break; // NOP
            case 0x03: do_add(cpu, byte2, 0); break;        // ADD A, #imm
            case 0x13: do_add(cpu, byte2, cpu->cy); break;  // ADDC A, #imm
            case 0x23: cpu->a = byte2; break;               // MOV A, #imm
            case 0x43: cpu->a |= byte2; break;              // ORL A, #imm
            case 0x53: cpu->a &= byte2; break;              // ANL A, #imm
            case 0xD3: cpu->a ^= byte2; break;              // XRL A, #imm

            case 0x07: cpu->a--; break;                     // DEC A
            case 0x17: cpu->a++; break;                     // INC A
            case 0x27: cpu->a = 0; break;                   // CLR A
            case 0x37: cpu->a = (uint8_t)~cpu->a; break;    // CPL A
            case 0x47: cpu->a = (uint8_t)((cpu->a << 4) | (cpu->a >> 4)); break; // SWAP A

            case 0x57: { // DA A
                if ((cpu->a & 0x0F) > 9 || cpu->ac) {
                    if ((uint16_t)cpu->a + 6 > 0xFF) cpu->cy = 1;
                    cpu->a += 6;
                }
                if ((cpu->a >> 4) > 9 || cpu->cy) {
                    if ((uint16_t)cpu->a + 0x60 > 0xFF) cpu->cy = 1;
                    cpu->a += 0x60;
                }
                break;
            }

            case 0x77: cpu->a = (uint8_t)((cpu->a >> 1) | (cpu->a << 7)); break; // RR A
            case 0xE7: cpu->a = (uint8_t)((cpu->a << 1) | (cpu->a >> 7)); break; // RL A
            case 0x67: { // RRC A
                uint8_t lsb = cpu->a & 1;
                cpu->a = (uint8_t)((cpu->a >> 1) | (cpu->cy << 7));
                cpu->cy = lsb;
                break;
            }
            case 0xF7: { // RLC A
                uint8_t msb = (cpu->a >> 7) & 1;
                cpu->a = (uint8_t)((cpu->a << 1) | cpu->cy);
                cpu->cy = msb;
                break;
            }

            case 0x97: cpu->cy = 0; break;       // CLR C
            case 0xA7: cpu->cy ^= 1; break;      // CPL C
            case 0x85: cpu->f0 = 0; break;       // CLR F0
            case 0x95: cpu->f0 ^= 1; break;      // CPL F0
            case 0xA5: cpu->f1 = 0; break;       // CLR F1
            case 0xB5: cpu->f1 ^= 1; break;      // CPL F1

            case 0xC7: cpu->a = get_psw(cpu); break;  // MOV A, PSW
            case 0xD7: set_psw(cpu, cpu->a); break;   // MOV PSW, A
            case 0x42: cpu->a = cpu->timer; break;    // MOV A, T
            case 0x62: cpu->timer = cpu->a; break;    // MOV T, A

            case 0x45: cpu->timer_run = 2; break;     // STRT CNT
            case 0x55: cpu->timer_run = 1; cpu->prescaler = 0; break; // STRT T
            case 0x65: cpu->timer_run = 0; break;     // STOP TCNT

            case 0x05: cpu->int_enabled = 1; break;   // EN I
            case 0x15: cpu->int_enabled = 0; break;   // DIS I
            case 0x25: cpu->tcnti_enabled = 1; break; // EN TCNTI
            case 0x35: cpu->tcnti_enabled = 0; break; // DIS TCNTI
            case 0x75: break;                         // ENT0 CLK (no-op)

            case 0xC5: cpu->bs = 0; break; // SEL RB0
            case 0xD5: cpu->bs = 1; break; // SEL RB1
            case 0xE5: cpu->mb = 0; break; // SEL MB0
            case 0xF5: cpu->mb = 1; break; // SEL MB1

            case 0xA3: // MOVP A, @A (current page)
                cpu->a = cpu->rom[(instr_pc & 0x0F00) | cpu->a];
                break;
            case 0xE3: // MOVP3 A, @A (page 3)
                cpu->a = cpu->rom[0x0300 | cpu->a];
                break;
            case 0xB3: // JMPP @A (indirect within current page)
                cpu->pc = (uint16_t)((next_pc & 0x0F00) | cpu->rom[(next_pc & 0x0F00) | cpu->a]);
                break;

            case 0x83: cpu->pc = pop_pc(cpu, 0); break; // RET
            case 0x93: cpu->pc = pop_pc(cpu, 1); break; // RETR

            // Conditional jumps
            case 0x16: if (cpu->timer_flag) { cpu->timer_flag = 0; cpu->pc = page_target; } break; // JTF
            case 0x26: if (!cpu->t0) cpu->pc = page_target; break;      // JNT0
            case 0x36: if (cpu->t0) cpu->pc = page_target; break;       // JT0
            case 0x46: if (!cpu->t1) cpu->pc = page_target; break;      // JNT1
            case 0x56: if (cpu->t1) cpu->pc = page_target; break;       // JT1
            case 0x76: if (cpu->f1) cpu->pc = page_target; break;       // JF1
            case 0x86: break;                                           // JNI (INT pin high: never taken)
            case 0x96: if (cpu->a != 0) cpu->pc = page_target; break;   // JNZ
            case 0xB6: if (cpu->f0) cpu->pc = page_target; break;       // JF0
            case 0xC6: if (cpu->a == 0) cpu->pc = page_target; break;   // JZ
            case 0xE6: if (!cpu->cy) cpu->pc = page_target; break;      // JNC
            case 0xF6: if (cpu->cy) cpu->pc = page_target; break;       // JC

            // Port I/O
            case 0x08: cpu->a = cpu->bus; break;        // INS A, BUS
            case 0x09: cpu->a = cpu->p1; break;         // IN A, P1
            case 0x0A: cpu->a = cpu->p2; break;         // IN A, P2
            case 0x02: cpu->bus = cpu->a; break;        // OUTL BUS, A
            case 0x39: cpu->p1 = cpu->a; break;         // OUTL P1, A
            case 0x3A: cpu->p2 = cpu->a; break;         // OUTL P2, A
            case 0x88: cpu->bus |= byte2; break;        // ORL BUS, #imm
            case 0x89: cpu->p1 |= byte2; break;         // ORL P1, #imm
            case 0x8A: cpu->p2 |= byte2; break;         // ORL P2, #imm
            case 0x98: cpu->bus &= byte2; break;        // ANL BUS, #imm
            case 0x99: cpu->p1 &= byte2; break;         // ANL P1, #imm
            case 0x9A: cpu->p2 &= byte2; break;         // ANL P2, #imm

            default: break; // Unimplemented/invalid opcodes act as NOP
        }
    }

    // JMP-to-self is interpreted as a software halt
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }

    return 0;
}

void i8048_print_state(void *context) {
    if (!context) return;
    I8048CPU *cpu = (I8048CPU*)context;

    printf("Intel 8048 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  A: 0x%02X  PC: 0x%03X  PSW: 0x%02X  Halted: %s\n",
           cpu->a, cpu->pc & 0x0FFF, get_psw(cpu), cpu->halted ? "Yes" : "No");
    printf("  Flags: CY=%d AC=%d F0=%d F1=%d BS=RB%d MB=%d SP=%d\n",
           cpu->cy, cpu->ac, cpu->f0, cpu->f1, cpu->bs, cpu->mb, cpu->sp & 7);
    printf("  Timer: T=0x%02X %s TF=%d  I=%s TCNTI=%s\n",
           cpu->timer,
           cpu->timer_run == 1 ? "(running)" : (cpu->timer_run == 2 ? "(counter)" : "(stopped)"),
           cpu->timer_flag,
           cpu->int_enabled ? "EN" : "DIS", cpu->tcnti_enabled ? "EN" : "DIS");
    printf("  Ports: BUS=0x%02X P1=0x%02X P2=0x%02X\n", cpu->bus, cpu->p1, cpu->p2);
    printf("  Registers (RB%d):\n", cpu->bs);
    for (int i = 0; i < 8; ++i) {
        printf("    R%d: 0x%02X%s", i, get_r(cpu, (uint8_t)i), (i == 3 || i == 7) ? "\n" : "  ");
    }
}

void i8048_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    I8048CPU *cpu = (I8048CPU*)context;

    uint16_t pc = cpu->pc & 0x0FFF;
    uint8_t op = cpu->rom[pc];
    uint8_t len = instr_len(op);
    uint8_t byte2 = (len == 2) ? cpu->rom[(pc + 1) & 0x0FFF] : 0;
    uint16_t next_pc = (pc + len) & 0x0FFF;
    uint16_t page_target = (uint16_t)((next_pc & 0x0F00) | byte2);

    if ((op & 0x1F) == 0x04) {
        uint16_t addr = (uint16_t)((((uint16_t)op & 0xE0) << 3) | byte2 | ((uint16_t)cpu->mb << 11));
        snprintf(buf, buf_len, "JMP   0x%03X", addr);
    }
    else if ((op & 0x1F) == 0x14) {
        uint16_t addr = (uint16_t)((((uint16_t)op & 0xE0) << 3) | byte2 | ((uint16_t)cpu->mb << 11));
        snprintf(buf, buf_len, "CALL  0x%03X", addr);
    }
    else if ((op & 0x1F) == 0x12) {
        snprintf(buf, buf_len, "JB%d   0x%03X", (op >> 5) & 7, page_target);
    }
    else if ((op & 0xF8) == 0xE8) {
        snprintf(buf, buf_len, "DJNZ  R%d, 0x%03X", op & 7, page_target);
    }
    else if ((op & 0xF8) == 0x68) snprintf(buf, buf_len, "ADD   A, R%d", op & 7);
    else if ((op & 0xFE) == 0x60) snprintf(buf, buf_len, "ADD   A, @R%d", op & 1);
    else if ((op & 0xF8) == 0x78) snprintf(buf, buf_len, "ADDC  A, R%d", op & 7);
    else if ((op & 0xFE) == 0x70) snprintf(buf, buf_len, "ADDC  A, @R%d", op & 1);
    else if ((op & 0xF8) == 0x58) snprintf(buf, buf_len, "ANL   A, R%d", op & 7);
    else if ((op & 0xFE) == 0x50) snprintf(buf, buf_len, "ANL   A, @R%d", op & 1);
    else if ((op & 0xF8) == 0x48) snprintf(buf, buf_len, "ORL   A, R%d", op & 7);
    else if ((op & 0xFE) == 0x40) snprintf(buf, buf_len, "ORL   A, @R%d", op & 1);
    else if ((op & 0xF8) == 0xD8) snprintf(buf, buf_len, "XRL   A, R%d", op & 7);
    else if ((op & 0xFE) == 0xD0) snprintf(buf, buf_len, "XRL   A, @R%d", op & 1);
    else if ((op & 0xF8) == 0x18) snprintf(buf, buf_len, "INC   R%d", op & 7);
    else if ((op & 0xFE) == 0x10) snprintf(buf, buf_len, "INC   @R%d", op & 1);
    else if ((op & 0xF8) == 0xC8) snprintf(buf, buf_len, "DEC   R%d", op & 7);
    else if ((op & 0xF8) == 0x28) snprintf(buf, buf_len, "XCH   A, R%d", op & 7);
    else if ((op & 0xFE) == 0x20) snprintf(buf, buf_len, "XCH   A, @R%d", op & 1);
    else if ((op & 0xFE) == 0x30) snprintf(buf, buf_len, "XCHD  A, @R%d", op & 1);
    else if ((op & 0xF8) == 0xF8) snprintf(buf, buf_len, "MOV   A, R%d", op & 7);
    else if ((op & 0xFE) == 0xF0) snprintf(buf, buf_len, "MOV   A, @R%d", op & 1);
    else if ((op & 0xF8) == 0xA8) snprintf(buf, buf_len, "MOV   R%d, A", op & 7);
    else if ((op & 0xFE) == 0xA0) snprintf(buf, buf_len, "MOV   @R%d, A", op & 1);
    else if ((op & 0xF8) == 0xB8) snprintf(buf, buf_len, "MOV   R%d, #0x%02X", op & 7, byte2);
    else if ((op & 0xFE) == 0xB0) snprintf(buf, buf_len, "MOV   @R%d, #0x%02X", op & 1, byte2);
    else {
        switch (op) {
            case 0x00: snprintf(buf, buf_len, "NOP"); break;
            case 0x03: snprintf(buf, buf_len, "ADD   A, #0x%02X", byte2); break;
            case 0x13: snprintf(buf, buf_len, "ADDC  A, #0x%02X", byte2); break;
            case 0x23: snprintf(buf, buf_len, "MOV   A, #0x%02X", byte2); break;
            case 0x43: snprintf(buf, buf_len, "ORL   A, #0x%02X", byte2); break;
            case 0x53: snprintf(buf, buf_len, "ANL   A, #0x%02X", byte2); break;
            case 0xD3: snprintf(buf, buf_len, "XRL   A, #0x%02X", byte2); break;
            case 0x07: snprintf(buf, buf_len, "DEC   A"); break;
            case 0x17: snprintf(buf, buf_len, "INC   A"); break;
            case 0x27: snprintf(buf, buf_len, "CLR   A"); break;
            case 0x37: snprintf(buf, buf_len, "CPL   A"); break;
            case 0x47: snprintf(buf, buf_len, "SWAP  A"); break;
            case 0x57: snprintf(buf, buf_len, "DA    A"); break;
            case 0x77: snprintf(buf, buf_len, "RR    A"); break;
            case 0xE7: snprintf(buf, buf_len, "RL    A"); break;
            case 0x67: snprintf(buf, buf_len, "RRC   A"); break;
            case 0xF7: snprintf(buf, buf_len, "RLC   A"); break;
            case 0x97: snprintf(buf, buf_len, "CLR   C"); break;
            case 0xA7: snprintf(buf, buf_len, "CPL   C"); break;
            case 0x85: snprintf(buf, buf_len, "CLR   F0"); break;
            case 0x95: snprintf(buf, buf_len, "CPL   F0"); break;
            case 0xA5: snprintf(buf, buf_len, "CLR   F1"); break;
            case 0xB5: snprintf(buf, buf_len, "CPL   F1"); break;
            case 0xC7: snprintf(buf, buf_len, "MOV   A, PSW"); break;
            case 0xD7: snprintf(buf, buf_len, "MOV   PSW, A"); break;
            case 0x42: snprintf(buf, buf_len, "MOV   A, T"); break;
            case 0x62: snprintf(buf, buf_len, "MOV   T, A"); break;
            case 0x45: snprintf(buf, buf_len, "STRT  CNT"); break;
            case 0x55: snprintf(buf, buf_len, "STRT  T"); break;
            case 0x65: snprintf(buf, buf_len, "STOP  TCNT"); break;
            case 0x05: snprintf(buf, buf_len, "EN    I"); break;
            case 0x15: snprintf(buf, buf_len, "DIS   I"); break;
            case 0x25: snprintf(buf, buf_len, "EN    TCNTI"); break;
            case 0x35: snprintf(buf, buf_len, "DIS   TCNTI"); break;
            case 0x75: snprintf(buf, buf_len, "ENT0  CLK"); break;
            case 0xC5: snprintf(buf, buf_len, "SEL   RB0"); break;
            case 0xD5: snprintf(buf, buf_len, "SEL   RB1"); break;
            case 0xE5: snprintf(buf, buf_len, "SEL   MB0"); break;
            case 0xF5: snprintf(buf, buf_len, "SEL   MB1"); break;
            case 0xA3: snprintf(buf, buf_len, "MOVP  A, @A"); break;
            case 0xE3: snprintf(buf, buf_len, "MOVP3 A, @A"); break;
            case 0xB3: snprintf(buf, buf_len, "JMPP  @A"); break;
            case 0x83: snprintf(buf, buf_len, "RET"); break;
            case 0x93: snprintf(buf, buf_len, "RETR"); break;
            case 0x16: snprintf(buf, buf_len, "JTF   0x%03X", page_target); break;
            case 0x26: snprintf(buf, buf_len, "JNT0  0x%03X", page_target); break;
            case 0x36: snprintf(buf, buf_len, "JT0   0x%03X", page_target); break;
            case 0x46: snprintf(buf, buf_len, "JNT1  0x%03X", page_target); break;
            case 0x56: snprintf(buf, buf_len, "JT1   0x%03X", page_target); break;
            case 0x76: snprintf(buf, buf_len, "JF1   0x%03X", page_target); break;
            case 0x86: snprintf(buf, buf_len, "JNI   0x%03X", page_target); break;
            case 0x96: snprintf(buf, buf_len, "JNZ   0x%03X", page_target); break;
            case 0xB6: snprintf(buf, buf_len, "JF0   0x%03X", page_target); break;
            case 0xC6: snprintf(buf, buf_len, "JZ    0x%03X", page_target); break;
            case 0xE6: snprintf(buf, buf_len, "JNC   0x%03X", page_target); break;
            case 0xF6: snprintf(buf, buf_len, "JC    0x%03X", page_target); break;
            case 0x08: snprintf(buf, buf_len, "INS   A, BUS"); break;
            case 0x09: snprintf(buf, buf_len, "IN    A, P1"); break;
            case 0x0A: snprintf(buf, buf_len, "IN    A, P2"); break;
            case 0x02: snprintf(buf, buf_len, "OUTL  BUS, A"); break;
            case 0x39: snprintf(buf, buf_len, "OUTL  P1, A"); break;
            case 0x3A: snprintf(buf, buf_len, "OUTL  P2, A"); break;
            case 0x88: snprintf(buf, buf_len, "ORL   BUS, #0x%02X", byte2); break;
            case 0x89: snprintf(buf, buf_len, "ORL   P1, #0x%02X", byte2); break;
            case 0x8A: snprintf(buf, buf_len, "ORL   P2, #0x%02X", byte2); break;
            case 0x98: snprintf(buf, buf_len, "ANL   BUS, #0x%02X", byte2); break;
            case 0x99: snprintf(buf, buf_len, "ANL   P1, #0x%02X", byte2); break;
            case 0x9A: snprintf(buf, buf_len, "ANL   P2, #0x%02X", byte2); break;
            default:   snprintf(buf, buf_len, "INV   0x%02X", op); break;
        }
    }
}
