#include "pic16.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PROG_WORDS 1024   // 1K x 14-bit program memory (PIC16F84-class)
#define DATA_SIZE 256     // Flat 256-byte data memory (both banks)
#define STACK_DEPTH 8     // 8-level hardware stack

// Special Function Register offsets (mirrored across both banks)
#define REG_INDF   0x00
#define REG_PCL    0x02
#define REG_STATUS 0x03
#define REG_FSR    0x04
#define REG_PCLATH 0x0A

// STATUS bits
#define FLAG_C  0
#define FLAG_DC 1
#define FLAG_Z  2

typedef struct PIC16_CPU {
    uint16_t prog[PROG_WORDS]; // 14-bit instruction words
    uint8_t data[DATA_SIZE];   // general data memory (SFRs handled separately)
    uint16_t pc;               // word address (10 bits used)
    uint8_t w;                 // working register
    uint8_t status;            // C, DC, Z + bank bits (RP0/RP1/IRP writable)
    uint8_t fsr;               // indirect address pointer
    uint8_t pclath;            // PC latch high (5 bits)
    uint16_t stack[STACK_DEPTH];
    uint8_t sp;                // stack pointer (circular, 0..7)
    uint8_t stack_used;        // depth for display (saturates at 8)
    int pc_written;            // set when an instruction writes PCL
    uint32_t ticks;
    int halted;
} PIC16_CPU;

#define GET_FLAG(bit) ((cpu->status >> (bit)) & 1)
#define SET_FLAG(bit, val) do { if (val) cpu->status |= (uint8_t)(1 << (bit)); else cpu->status &= (uint8_t)~(1 << (bit)); } while(0)

void* pic16_create(void) {
    PIC16_CPU *cpu = (PIC16_CPU*)calloc(1, sizeof(PIC16_CPU));
    return cpu;
}

void pic16_destroy(void *context) {
    free(context);
}

int pic16_init(void *context) {
    if (!context) return -1;
    PIC16_CPU *cpu = (PIC16_CPU*)context;

    memset(cpu, 0, sizeof(PIC16_CPU));
    cpu->status = 0x18; // power-on: TO and PD set
    return 0;
}

int pic16_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    PIC16_CPU *cpu = (PIC16_CPU*)context;

    // Bytes are little-endian 16-bit words, each holding a 14-bit instruction.
    // The address parameter is a word address.
    if (address >= PROG_WORDS) return -2;

    size_t words = size / 2;
    if (address + words > PROG_WORDS) {
        words = PROG_WORDS - address;
    }

    for (size_t i = 0; i < words; ++i) {
        uint16_t word = (uint16_t)(data[i * 2] | ((uint16_t)data[i * 2 + 1] << 8));
        cpu->prog[address + i] = word & 0x3FFF;
    }
    return 0;
}

// Effective 8-bit data address for a 7-bit file operand (RP0 selects the bank)
static inline uint8_t eff_addr(PIC16_CPU *cpu, uint8_t f) {
    return (uint8_t)((f & 0x7F) | ((cpu->status & 0x20) << 2));
}

static uint8_t data_read(PIC16_CPU *cpu, uint8_t addr) {
    switch (addr & 0x7F) {
        case REG_INDF:
            // Indirect read through FSR; reading INDF indirectly yields 0
            if ((cpu->fsr & 0x7F) == REG_INDF) return 0;
            return data_read(cpu, cpu->fsr);
        case REG_PCL:    return (uint8_t)(cpu->pc & 0xFF);
        case REG_STATUS: return cpu->status;
        case REG_FSR:    return cpu->fsr;
        case REG_PCLATH: return cpu->pclath;
        default:         return cpu->data[addr];
    }
}

static void data_write(PIC16_CPU *cpu, uint8_t addr, uint8_t val) {
    switch (addr & 0x7F) {
        case REG_INDF:
            // Indirect write through FSR; writing INDF indirectly is a no-op
            if ((cpu->fsr & 0x7F) != REG_INDF) {
                data_write(cpu, cpu->fsr, val);
            }
            break;
        case REG_PCL:
            // Computed GOTO: PC = PCLATH<4:0>:val (masked to program memory)
            cpu->pc = (uint16_t)((((uint16_t)(cpu->pclath & 0x1F) << 8) | val) % PROG_WORDS);
            cpu->pc_written = 1;
            break;
        case REG_STATUS: cpu->status = val; break;
        case REG_FSR:    cpu->fsr = val; break;
        case REG_PCLATH: cpu->pclath = val & 0x1F; break;
        default:         cpu->data[addr] = val; break;
    }
}

static inline void push_stack(PIC16_CPU *cpu, uint16_t addr) {
    cpu->stack[cpu->sp] = addr;
    cpu->sp = (uint8_t)((cpu->sp + 1) % STACK_DEPTH);
    if (cpu->stack_used < STACK_DEPTH) cpu->stack_used++;
}

static inline uint16_t pop_stack(PIC16_CPU *cpu) {
    cpu->sp = (uint8_t)((cpu->sp + STACK_DEPTH - 1) % STACK_DEPTH);
    if (cpu->stack_used > 0) cpu->stack_used--;
    return cpu->stack[cpu->sp];
}

static inline void set_z(PIC16_CPU *cpu, uint8_t res) {
    SET_FLAG(FLAG_Z, res == 0);
}

// result of a + b (+ optional borrow logic handled by callers)
static uint8_t do_add(PIC16_CPU *cpu, uint8_t a, uint8_t b) {
    uint16_t sum = (uint16_t)a + b;
    SET_FLAG(FLAG_C, sum > 0xFF);
    SET_FLAG(FLAG_DC, ((a & 0x0F) + (b & 0x0F)) > 0x0F);
    set_z(cpu, (uint8_t)(sum & 0xFF));
    return (uint8_t)(sum & 0xFF);
}

// result of a - b; C set means no borrow (PIC convention)
static uint8_t do_sub(PIC16_CPU *cpu, uint8_t a, uint8_t b) {
    uint16_t diff = (uint16_t)a - b;
    SET_FLAG(FLAG_C, a >= b);
    SET_FLAG(FLAG_DC, (a & 0x0F) >= (b & 0x0F));
    set_z(cpu, (uint8_t)(diff & 0xFF));
    return (uint8_t)(diff & 0xFF);
}

int pic16_step(void *context) {
    if (!context) return -1;
    PIC16_CPU *cpu = (PIC16_CPU*)context;

    if (cpu->halted) return 1;

    uint16_t instr_pc = cpu->pc;
    uint16_t op = cpu->prog[cpu->pc % PROG_WORDS];
    uint16_t next_pc = (uint16_t)((cpu->pc + 1) % PROG_WORDS);
    cpu->ticks++;
    cpu->pc_written = 0;

    uint8_t f = (uint8_t)(op & 0x7F);
    uint8_t d = (uint8_t)((op >> 7) & 1);
    uint8_t bit = (uint8_t)((op >> 7) & 7);
    uint8_t lit = (uint8_t)(op & 0xFF);
    uint8_t addr = eff_addr(cpu, f);

    if ((op & 0x3F9F) == 0x0000) {
        // NOP (00 0000 0xx0 0000)
    }
    else if (op == 0x0008) {
        // RETURN
        next_pc = (uint16_t)(pop_stack(cpu) % PROG_WORDS);
    }
    else if (op == 0x0009) {
        // RETFIE (no interrupt controller: behaves as return)
        next_pc = (uint16_t)(pop_stack(cpu) % PROG_WORDS);
    }
    else if (op == 0x0063) {
        // SLEEP
        cpu->pc = next_pc;
        cpu->halted = 1;
        return 1;
    }
    else if (op == 0x0064) {
        // CLRWDT (no watchdog: treated as NOP)
    }
    else if ((op & 0x3F80) == 0x0080) {
        // MOVWF f
        data_write(cpu, addr, cpu->w);
    }
    else if ((op & 0x3F80) == 0x0100) {
        // CLRW
        cpu->w = 0;
        SET_FLAG(FLAG_Z, 1);
    }
    else if ((op & 0x3F80) == 0x0180) {
        // CLRF f
        data_write(cpu, addr, 0);
        SET_FLAG(FLAG_Z, 1);
    }
    else if ((op & 0x3F00) == 0x0200) {
        // SUBWF f, d (f - W)
        uint8_t res = do_sub(cpu, data_read(cpu, addr), cpu->w);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0300) {
        // DECF f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) - 1);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0400) {
        // IORWF f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) | cpu->w);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0500) {
        // ANDWF f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) & cpu->w);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0600) {
        // XORWF f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) ^ cpu->w);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0700) {
        // ADDWF f, d
        uint8_t res = do_add(cpu, data_read(cpu, addr), cpu->w);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0800) {
        // MOVF f, d
        uint8_t res = data_read(cpu, addr);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0900) {
        // COMF f, d
        uint8_t res = (uint8_t)~data_read(cpu, addr);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0A00) {
        // INCF f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) + 1);
        set_z(cpu, res);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0B00) {
        // DECFSZ f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) - 1);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
        if (res == 0) next_pc = (uint16_t)((instr_pc + 2) % PROG_WORDS);
    }
    else if ((op & 0x3F00) == 0x0C00) {
        // RRF f, d
        uint8_t val = data_read(cpu, addr);
        uint8_t res = (uint8_t)((val >> 1) | (GET_FLAG(FLAG_C) << 7));
        SET_FLAG(FLAG_C, val & 1);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0D00) {
        // RLF f, d
        uint8_t val = data_read(cpu, addr);
        uint8_t res = (uint8_t)((val << 1) | GET_FLAG(FLAG_C));
        SET_FLAG(FLAG_C, (val & 0x80) != 0);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0E00) {
        // SWAPF f, d
        uint8_t val = data_read(cpu, addr);
        uint8_t res = (uint8_t)((val << 4) | (val >> 4));
        if (d) data_write(cpu, addr, res); else cpu->w = res;
    }
    else if ((op & 0x3F00) == 0x0F00) {
        // INCFSZ f, d
        uint8_t res = (uint8_t)(data_read(cpu, addr) + 1);
        if (d) data_write(cpu, addr, res); else cpu->w = res;
        if (res == 0) next_pc = (uint16_t)((instr_pc + 2) % PROG_WORDS);
    }
    else if ((op & 0x3C00) == 0x1000) {
        // BCF f, b
        data_write(cpu, addr, (uint8_t)(data_read(cpu, addr) & ~(1 << bit)));
    }
    else if ((op & 0x3C00) == 0x1400) {
        // BSF f, b
        data_write(cpu, addr, (uint8_t)(data_read(cpu, addr) | (1 << bit)));
    }
    else if ((op & 0x3C00) == 0x1800) {
        // BTFSC f, b
        if (!((data_read(cpu, addr) >> bit) & 1)) {
            next_pc = (uint16_t)((instr_pc + 2) % PROG_WORDS);
        }
    }
    else if ((op & 0x3C00) == 0x1C00) {
        // BTFSS f, b
        if ((data_read(cpu, addr) >> bit) & 1) {
            next_pc = (uint16_t)((instr_pc + 2) % PROG_WORDS);
        }
    }
    else if ((op & 0x3800) == 0x2000) {
        // CALL k (target = PCLATH<4:3>:k11)
        uint16_t target = (uint16_t)(((((uint16_t)cpu->pclath & 0x18) << 8) | (op & 0x7FF)) % PROG_WORDS);
        push_stack(cpu, next_pc);
        next_pc = target;
    }
    else if ((op & 0x3800) == 0x2800) {
        // GOTO k (target = PCLATH<4:3>:k11)
        uint16_t target = (uint16_t)(((((uint16_t)cpu->pclath & 0x18) << 8) | (op & 0x7FF)) % PROG_WORDS);
        if (target == instr_pc) {
            // GOTO-to-self: interpreted as intentional halt
            cpu->halted = 1;
            return 1;
        }
        next_pc = target;
    }
    else if ((op & 0x3C00) == 0x3000) {
        // MOVLW k
        cpu->w = lit;
    }
    else if ((op & 0x3C00) == 0x3400) {
        // RETLW k
        cpu->w = lit;
        next_pc = (uint16_t)(pop_stack(cpu) % PROG_WORDS);
    }
    else if ((op & 0x3F00) == 0x3800) {
        // IORLW k
        cpu->w |= lit;
        set_z(cpu, cpu->w);
    }
    else if ((op & 0x3F00) == 0x3900) {
        // ANDLW k
        cpu->w &= lit;
        set_z(cpu, cpu->w);
    }
    else if ((op & 0x3F00) == 0x3A00) {
        // XORLW k
        cpu->w ^= lit;
        set_z(cpu, cpu->w);
    }
    else if ((op & 0x3E00) == 0x3C00) {
        // SUBLW k (k - W)
        cpu->w = do_sub(cpu, lit, cpu->w);
    }
    else if ((op & 0x3E00) == 0x3E00) {
        // ADDLW k
        cpu->w = do_add(cpu, lit, cpu->w);
    }
    // Unknown opcodes execute as NOP

    if (cpu->pc_written) {
        // A write to PCL redirected the program counter (computed GOTO)
        next_pc = cpu->pc;
        cpu->pc_written = 0;
    }

    cpu->pc = next_pc;
    return 0;
}

void pic16_print_state(void *context) {
    if (!context) return;
    PIC16_CPU *cpu = (PIC16_CPU*)context;

    printf("PIC16 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%03X  Halted: %s\n", cpu->pc, cpu->halted ? "Yes" : "No");
    printf("  W: 0x%02X\n", cpu->w);
    printf("  STATUS: 0x%02X (C=%d, DC=%d, Z=%d, RP0=%d)\n",
           cpu->status, GET_FLAG(FLAG_C), GET_FLAG(FLAG_DC), GET_FLAG(FLAG_Z),
           (cpu->status >> 5) & 1);
    printf("  FSR: 0x%02X  PCLATH: 0x%02X\n", cpu->fsr, cpu->pclath);
    printf("  Stack depth: %u/%u\n", cpu->stack_used, STACK_DEPTH);
}

void pic16_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    PIC16_CPU *cpu = (PIC16_CPU*)context;

    uint16_t op = cpu->prog[cpu->pc % PROG_WORDS];
    uint8_t f = (uint8_t)(op & 0x7F);
    char d = ((op >> 7) & 1) ? 'f' : 'w';
    uint8_t bit = (uint8_t)((op >> 7) & 7);
    uint8_t lit = (uint8_t)(op & 0xFF);
    uint16_t k11 = (uint16_t)(op & 0x7FF);

    if ((op & 0x3F9F) == 0x0000) snprintf(buf, buf_len, "nop");
    else if (op == 0x0008) snprintf(buf, buf_len, "return");
    else if (op == 0x0009) snprintf(buf, buf_len, "retfie");
    else if (op == 0x0063) snprintf(buf, buf_len, "sleep");
    else if (op == 0x0064) snprintf(buf, buf_len, "clrwdt");
    else if ((op & 0x3F80) == 0x0080) snprintf(buf, buf_len, "movwf  0x%02X", f);
    else if ((op & 0x3F80) == 0x0100) snprintf(buf, buf_len, "clrw");
    else if ((op & 0x3F80) == 0x0180) snprintf(buf, buf_len, "clrf   0x%02X", f);
    else if ((op & 0x3F00) == 0x0200) snprintf(buf, buf_len, "subwf  0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0300) snprintf(buf, buf_len, "decf   0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0400) snprintf(buf, buf_len, "iorwf  0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0500) snprintf(buf, buf_len, "andwf  0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0600) snprintf(buf, buf_len, "xorwf  0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0700) snprintf(buf, buf_len, "addwf  0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0800) snprintf(buf, buf_len, "movf   0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0900) snprintf(buf, buf_len, "comf   0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0A00) snprintf(buf, buf_len, "incf   0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0B00) snprintf(buf, buf_len, "decfsz 0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0C00) snprintf(buf, buf_len, "rrf    0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0D00) snprintf(buf, buf_len, "rlf    0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0E00) snprintf(buf, buf_len, "swapf  0x%02X, %c", f, d);
    else if ((op & 0x3F00) == 0x0F00) snprintf(buf, buf_len, "incfsz 0x%02X, %c", f, d);
    else if ((op & 0x3C00) == 0x1000) snprintf(buf, buf_len, "bcf    0x%02X, %d", f, bit);
    else if ((op & 0x3C00) == 0x1400) snprintf(buf, buf_len, "bsf    0x%02X, %d", f, bit);
    else if ((op & 0x3C00) == 0x1800) snprintf(buf, buf_len, "btfsc  0x%02X, %d", f, bit);
    else if ((op & 0x3C00) == 0x1C00) snprintf(buf, buf_len, "btfss  0x%02X, %d", f, bit);
    else if ((op & 0x3800) == 0x2000) snprintf(buf, buf_len, "call   0x%03X", k11);
    else if ((op & 0x3800) == 0x2800) snprintf(buf, buf_len, "goto   0x%03X", k11);
    else if ((op & 0x3C00) == 0x3000) snprintf(buf, buf_len, "movlw  0x%02X", lit);
    else if ((op & 0x3C00) == 0x3400) snprintf(buf, buf_len, "retlw  0x%02X", lit);
    else if ((op & 0x3F00) == 0x3800) snprintf(buf, buf_len, "iorlw  0x%02X", lit);
    else if ((op & 0x3F00) == 0x3900) snprintf(buf, buf_len, "andlw  0x%02X", lit);
    else if ((op & 0x3F00) == 0x3A00) snprintf(buf, buf_len, "xorlw  0x%02X", lit);
    else if ((op & 0x3E00) == 0x3C00) snprintf(buf, buf_len, "sublw  0x%02X", lit);
    else if ((op & 0x3E00) == 0x3E00) snprintf(buf, buf_len, "addlw  0x%02X", lit);
    else snprintf(buf, buf_len, "unknown (0x%04X)", op);
}
