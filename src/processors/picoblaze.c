#include "picoblaze.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PB_IMEM_SIZE   4096
#define PB_STACK_DEPTH 30
#define PB_HWBUILD     0x41

typedef struct PicoBlazeCPU {
    uint16_t pc;                     // 12-bit program counter (instruction index)
    uint32_t imem[PB_IMEM_SIZE];     // 18-bit instructions
    uint8_t reg[2][16];              // Register banks A (0) and B (1)
    uint8_t bank;                    // Active register bank (0 = A, 1 = B)
    uint8_t scratchpad[64];          // Scratchpad RAM
    uint16_t stack[PB_STACK_DEPTH];  // Call/return stack
    uint8_t sp;                      // Stack depth (0..30)
    uint8_t flag_z;                  // Zero flag
    uint8_t flag_c;                  // Carry flag
    uint8_t shadow_z;                // Preserved Z across interrupts
    uint8_t shadow_c;                // Preserved C across interrupts
    uint8_t int_enable;              // Interrupt enable flag
    uint8_t in_ports[256];           // Input port values
    uint8_t out_ports[256];          // Output port latches
    uint32_t ticks;
} PicoBlazeCPU;

void* picoblaze_create(void) {
    return calloc(1, sizeof(PicoBlazeCPU));
}

void picoblaze_destroy(void *context) {
    free(context);
}

int picoblaze_init(void *context) {
    if (!context) return -1;
    PicoBlazeCPU *cpu = (PicoBlazeCPU*)context;

    cpu->pc = 0;
    memset(cpu->reg, 0, sizeof(cpu->reg));
    cpu->bank = 0;
    memset(cpu->scratchpad, 0, sizeof(cpu->scratchpad));
    memset(cpu->stack, 0, sizeof(cpu->stack));
    cpu->sp = 0;
    cpu->flag_z = 0;
    cpu->flag_c = 0;
    cpu->shadow_z = 0;
    cpu->shadow_c = 0;
    cpu->int_enable = 0;
    memset(cpu->in_ports, 0, sizeof(cpu->in_ports));
    memset(cpu->out_ports, 0, sizeof(cpu->out_ports));
    memset(cpu->imem, 0, sizeof(cpu->imem));
    cpu->ticks = 0;
    return 0;
}

int picoblaze_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    PicoBlazeCPU *cpu = (PicoBlazeCPU*)context;
    if (address >= PB_IMEM_SIZE) return -2;

    size_t count = size / 3; // 3 bytes per 18-bit instruction, big-endian
    for (size_t i = 0; i < count; ++i) {
        if (address + i >= PB_IMEM_SIZE) break;
        uint32_t instr = ((uint32_t)(data[i * 3] & 0x03) << 16) |
                         ((uint32_t)data[i * 3 + 1] << 8) |
                         (uint32_t)data[i * 3 + 2];
        cpu->imem[address + i] = instr;
    }
    return 0;
}

static uint8_t pb_parity(uint8_t v) {
    v ^= (uint8_t)(v >> 4);
    v ^= (uint8_t)(v >> 2);
    v ^= (uint8_t)(v >> 1);
    return (uint8_t)(v & 1);
}

static int pb_push(PicoBlazeCPU *cpu, uint16_t addr) {
    if (cpu->sp >= PB_STACK_DEPTH) return -5; // Stack overflow
    cpu->stack[cpu->sp++] = addr;
    return 0;
}

static int pb_pop(PicoBlazeCPU *cpu, uint16_t *addr) {
    if (cpu->sp == 0) return -6; // Stack underflow
    *addr = cpu->stack[--cpu->sp];
    return 0;
}

int picoblaze_step(void *context) {
    if (!context) return -1;
    PicoBlazeCPU *cpu = (PicoBlazeCPU*)context;

    uint16_t instr_pc = cpu->pc & 0xFFF;
    uint32_t instr = cpu->imem[instr_pc];
    uint8_t op = (uint8_t)((instr >> 12) & 0x3F);
    uint8_t x = (uint8_t)((instr >> 8) & 0x0F);
    uint8_t y = (uint8_t)((instr >> 4) & 0x0F);
    uint8_t kk = (uint8_t)(instr & 0xFF);
    uint16_t aaa = (uint16_t)(instr & 0xFFF);

    uint8_t *r = cpu->reg[cpu->bank];
    uint8_t sx = r[x];
    uint8_t operand = (op & 1) ? kk : r[y]; // Odd opcodes use the constant form

    cpu->pc = (uint16_t)((instr_pc + 1) & 0xFFF);
    cpu->ticks += 2; // Every KCPSM6 instruction takes 2 clock cycles

    switch (op) {
        case 0x00: case 0x01: // LOAD sX, sY|kk
            r[x] = operand;
            break;
        case 0x16: case 0x17: // STAR sX, sY|kk (write to inactive bank)
            cpu->reg[cpu->bank ^ 1][x] = operand;
            break;
        case 0x02: case 0x03: // AND
            r[x] = (uint8_t)(sx & operand);
            cpu->flag_z = (r[x] == 0);
            cpu->flag_c = 0;
            break;
        case 0x04: case 0x05: // OR
            r[x] = (uint8_t)(sx | operand);
            cpu->flag_z = (r[x] == 0);
            cpu->flag_c = 0;
            break;
        case 0x06: case 0x07: // XOR
            r[x] = (uint8_t)(sx ^ operand);
            cpu->flag_z = (r[x] == 0);
            cpu->flag_c = 0;
            break;
        case 0x10: case 0x11: { // ADD
            uint16_t sum = (uint16_t)(sx + operand);
            r[x] = (uint8_t)sum;
            cpu->flag_c = (sum > 0xFF);
            cpu->flag_z = (r[x] == 0);
            break;
        }
        case 0x12: case 0x13: { // ADDCY
            uint16_t sum = (uint16_t)(sx + operand + cpu->flag_c);
            r[x] = (uint8_t)sum;
            cpu->flag_c = (sum > 0xFF);
            cpu->flag_z = (r[x] == 0) && cpu->flag_z;
            break;
        }
        case 0x18: case 0x19: { // SUB
            uint16_t diff = (uint16_t)(sx - operand);
            r[x] = (uint8_t)diff;
            cpu->flag_c = (diff > 0xFF);
            cpu->flag_z = (r[x] == 0);
            break;
        }
        case 0x1A: case 0x1B: { // SUBCY
            uint16_t diff = (uint16_t)(sx - operand - cpu->flag_c);
            r[x] = (uint8_t)diff;
            cpu->flag_c = (diff > 0xFF);
            cpu->flag_z = (r[x] == 0) && cpu->flag_z;
            break;
        }
        case 0x1C: case 0x1D: { // COMPARE
            uint16_t diff = (uint16_t)(sx - operand);
            cpu->flag_c = (diff > 0xFF);
            cpu->flag_z = ((diff & 0xFF) == 0);
            break;
        }
        case 0x1E: case 0x1F: { // COMPARECY
            uint16_t diff = (uint16_t)(sx - operand - cpu->flag_c);
            cpu->flag_c = (diff > 0xFF);
            cpu->flag_z = ((diff & 0xFF) == 0) && cpu->flag_z;
            break;
        }
        case 0x0C: case 0x0D: { // TEST
            uint8_t res = (uint8_t)(sx & operand);
            cpu->flag_z = (res == 0);
            cpu->flag_c = pb_parity(res);
            break;
        }
        case 0x0E: case 0x0F: { // TESTCY
            uint8_t res = (uint8_t)(sx & operand);
            cpu->flag_z = (res == 0) && cpu->flag_z;
            cpu->flag_c = (uint8_t)(pb_parity(res) ^ cpu->flag_c);
            break;
        }
        case 0x14: // Shift/rotate group and HWBUILD
            if (kk & 0x80) { // HWBUILD sX
                r[x] = PB_HWBUILD;
                cpu->flag_c = 1;
                cpu->flag_z = (r[x] == 0);
            } else {
                uint8_t res = sx;
                uint8_t old_c = cpu->flag_c;
                switch (kk & 0x0F) {
                    case 0x00: // SLA
                        cpu->flag_c = (sx >> 7) & 1;
                        res = (uint8_t)((sx << 1) | old_c);
                        break;
                    case 0x02: // RL
                        cpu->flag_c = (sx >> 7) & 1;
                        res = (uint8_t)((sx << 1) | (sx >> 7));
                        break;
                    case 0x04: // SLX
                        cpu->flag_c = (sx >> 7) & 1;
                        res = (uint8_t)((sx << 1) | (sx & 1));
                        break;
                    case 0x06: // SL0
                        cpu->flag_c = (sx >> 7) & 1;
                        res = (uint8_t)(sx << 1);
                        break;
                    case 0x07: // SL1
                        cpu->flag_c = (sx >> 7) & 1;
                        res = (uint8_t)((sx << 1) | 1);
                        break;
                    case 0x08: // SRA
                        cpu->flag_c = sx & 1;
                        res = (uint8_t)((sx >> 1) | (old_c << 7));
                        break;
                    case 0x0A: // SRX
                        cpu->flag_c = sx & 1;
                        res = (uint8_t)((sx >> 1) | (sx & 0x80));
                        break;
                    case 0x0C: // RR
                        cpu->flag_c = sx & 1;
                        res = (uint8_t)((sx >> 1) | (sx << 7));
                        break;
                    case 0x0E: // SR0
                        cpu->flag_c = sx & 1;
                        res = (uint8_t)(sx >> 1);
                        break;
                    case 0x0F: // SR1
                        cpu->flag_c = sx & 1;
                        res = (uint8_t)((sx >> 1) | 0x80);
                        break;
                    default:
                        return -4; // Invalid shift code
                }
                r[x] = res;
                cpu->flag_z = (res == 0);
            }
            break;
        case 0x37: // REGBANK A/B
            cpu->bank = (uint8_t)(instr & 1);
            break;
        case 0x08: // INPUT sX, (sY)
            r[x] = cpu->in_ports[r[y]];
            break;
        case 0x09: // INPUT sX, pp
            r[x] = cpu->in_ports[kk];
            break;
        case 0x2C: // OUTPUT sX, (sY)
            cpu->out_ports[r[y]] = sx;
            break;
        case 0x2D: // OUTPUT sX, pp
            cpu->out_ports[kk] = sx;
            break;
        case 0x2B: // OUTPUTK kk, p (constant to constant-optimized port 0..15)
            cpu->out_ports[instr & 0x0F] = (uint8_t)((instr >> 4) & 0xFF);
            break;
        case 0x2E: // STORE sX, (sY)
            cpu->scratchpad[r[y] & 0x3F] = sx;
            break;
        case 0x2F: // STORE sX, ss
            cpu->scratchpad[kk & 0x3F] = sx;
            break;
        case 0x0A: // FETCH sX, (sY)
            r[x] = cpu->scratchpad[r[y] & 0x3F];
            break;
        case 0x0B: // FETCH sX, ss
            r[x] = cpu->scratchpad[kk & 0x3F];
            break;
        case 0x22: // JUMP aaa
            cpu->pc = aaa;
            break;
        case 0x32: // JUMP Z, aaa
            if (cpu->flag_z) cpu->pc = aaa;
            break;
        case 0x36: // JUMP NZ, aaa
            if (!cpu->flag_z) cpu->pc = aaa;
            break;
        case 0x3A: // JUMP C, aaa
            if (cpu->flag_c) cpu->pc = aaa;
            break;
        case 0x3E: // JUMP NC, aaa
            if (!cpu->flag_c) cpu->pc = aaa;
            break;
        case 0x26: // JUMP@ (sX, sY)
            cpu->pc = (uint16_t)(((sx & 0x0F) << 8) | r[y]);
            break;
        case 0x20: { // CALL aaa
            int rc = pb_push(cpu, cpu->pc);
            if (rc) return rc;
            cpu->pc = aaa;
            break;
        }
        case 0x30: case 0x34: case 0x38: case 0x3C: { // CALL Z/NZ/C/NC, aaa
            int taken = 0;
            if (op == 0x30) taken = cpu->flag_z;
            else if (op == 0x34) taken = !cpu->flag_z;
            else if (op == 0x38) taken = cpu->flag_c;
            else taken = !cpu->flag_c;
            if (taken) {
                int rc = pb_push(cpu, cpu->pc);
                if (rc) return rc;
                cpu->pc = aaa;
            }
            break;
        }
        case 0x24: { // CALL@ (sX, sY)
            int rc = pb_push(cpu, cpu->pc);
            if (rc) return rc;
            cpu->pc = (uint16_t)(((sx & 0x0F) << 8) | r[y]);
            break;
        }
        case 0x25: { // RETURN
            uint16_t ret;
            int rc = pb_pop(cpu, &ret);
            if (rc) return rc;
            cpu->pc = ret;
            break;
        }
        case 0x31: case 0x35: case 0x39: case 0x3D: { // RETURN Z/NZ/C/NC
            int taken = 0;
            if (op == 0x31) taken = cpu->flag_z;
            else if (op == 0x35) taken = !cpu->flag_z;
            else if (op == 0x39) taken = cpu->flag_c;
            else taken = !cpu->flag_c;
            if (taken) {
                uint16_t ret;
                int rc = pb_pop(cpu, &ret);
                if (rc) return rc;
                cpu->pc = ret;
            }
            break;
        }
        case 0x21: { // LOAD&RETURN sX, kk
            uint16_t ret;
            int rc = pb_pop(cpu, &ret);
            if (rc) return rc;
            r[x] = kk;
            cpu->pc = ret;
            break;
        }
        case 0x28: // ENABLE/DISABLE INTERRUPT
            cpu->int_enable = (uint8_t)(instr & 1);
            break;
        case 0x29: { // RETURNI ENABLE/DISABLE
            uint16_t ret;
            int rc = pb_pop(cpu, &ret);
            if (rc) return rc;
            cpu->pc = ret;
            cpu->flag_z = cpu->shadow_z;
            cpu->flag_c = cpu->shadow_c;
            cpu->int_enable = (uint8_t)(instr & 1);
            break;
        }
        default:
            return -3; // Invalid opcode
    }

    // JUMP-to-self (or equivalent) means the program has halted
    if (cpu->pc == instr_pc) {
        return 1;
    }

    return 0;
}

void picoblaze_print_state(void *context) {
    if (!context) return;
    PicoBlazeCPU *cpu = (PicoBlazeCPU*)context;
    const uint8_t *r = cpu->reg[cpu->bank];

    printf("PicoBlaze (KCPSM6) State:\n");
    printf("  PC: 0x%03X    Z: %d    C: %d    IE: %d    Bank: %c    Ticks: %u\n",
           cpu->pc, cpu->flag_z, cpu->flag_c, cpu->int_enable,
           cpu->bank ? 'B' : 'A', cpu->ticks);
    printf("  Registers (bank %c):\n", cpu->bank ? 'B' : 'A');
    for (int i = 0; i < 16; ++i) {
        printf("    s%X: 0x%02X%s", i, r[i], (i % 4 == 3) ? "\n" : "  ");
    }
    printf("  Stack depth: %d/%d", cpu->sp, PB_STACK_DEPTH);
    if (cpu->sp > 0) {
        printf("  (top: 0x%03X)", cpu->stack[cpu->sp - 1]);
    }
    printf("\n");
}

void picoblaze_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    PicoBlazeCPU *cpu = (PicoBlazeCPU*)context;

    uint32_t instr = cpu->imem[cpu->pc & 0xFFF];
    uint8_t op = (uint8_t)((instr >> 12) & 0x3F);
    uint8_t x = (uint8_t)((instr >> 8) & 0x0F);
    uint8_t y = (uint8_t)((instr >> 4) & 0x0F);
    uint8_t kk = (uint8_t)(instr & 0xFF);
    uint16_t aaa = (uint16_t)(instr & 0xFFF);

    switch (op) {
        case 0x00: snprintf(buf, buf_len, "LOAD s%X, s%X", x, y); break;
        case 0x01: snprintf(buf, buf_len, "LOAD s%X, 0x%02X", x, kk); break;
        case 0x16: snprintf(buf, buf_len, "STAR s%X, s%X", x, y); break;
        case 0x17: snprintf(buf, buf_len, "STAR s%X, 0x%02X", x, kk); break;
        case 0x02: snprintf(buf, buf_len, "AND s%X, s%X", x, y); break;
        case 0x03: snprintf(buf, buf_len, "AND s%X, 0x%02X", x, kk); break;
        case 0x04: snprintf(buf, buf_len, "OR s%X, s%X", x, y); break;
        case 0x05: snprintf(buf, buf_len, "OR s%X, 0x%02X", x, kk); break;
        case 0x06: snprintf(buf, buf_len, "XOR s%X, s%X", x, y); break;
        case 0x07: snprintf(buf, buf_len, "XOR s%X, 0x%02X", x, kk); break;
        case 0x10: snprintf(buf, buf_len, "ADD s%X, s%X", x, y); break;
        case 0x11: snprintf(buf, buf_len, "ADD s%X, 0x%02X", x, kk); break;
        case 0x12: snprintf(buf, buf_len, "ADDCY s%X, s%X", x, y); break;
        case 0x13: snprintf(buf, buf_len, "ADDCY s%X, 0x%02X", x, kk); break;
        case 0x18: snprintf(buf, buf_len, "SUB s%X, s%X", x, y); break;
        case 0x19: snprintf(buf, buf_len, "SUB s%X, 0x%02X", x, kk); break;
        case 0x1A: snprintf(buf, buf_len, "SUBCY s%X, s%X", x, y); break;
        case 0x1B: snprintf(buf, buf_len, "SUBCY s%X, 0x%02X", x, kk); break;
        case 0x1C: snprintf(buf, buf_len, "COMPARE s%X, s%X", x, y); break;
        case 0x1D: snprintf(buf, buf_len, "COMPARE s%X, 0x%02X", x, kk); break;
        case 0x1E: snprintf(buf, buf_len, "COMPARECY s%X, s%X", x, y); break;
        case 0x1F: snprintf(buf, buf_len, "COMPARECY s%X, 0x%02X", x, kk); break;
        case 0x0C: snprintf(buf, buf_len, "TEST s%X, s%X", x, y); break;
        case 0x0D: snprintf(buf, buf_len, "TEST s%X, 0x%02X", x, kk); break;
        case 0x0E: snprintf(buf, buf_len, "TESTCY s%X, s%X", x, y); break;
        case 0x0F: snprintf(buf, buf_len, "TESTCY s%X, 0x%02X", x, kk); break;
        case 0x14:
            if (kk & 0x80) {
                snprintf(buf, buf_len, "HWBUILD s%X", x);
            } else {
                switch (kk & 0x0F) {
                    case 0x00: snprintf(buf, buf_len, "SLA s%X", x); break;
                    case 0x02: snprintf(buf, buf_len, "RL s%X", x); break;
                    case 0x04: snprintf(buf, buf_len, "SLX s%X", x); break;
                    case 0x06: snprintf(buf, buf_len, "SL0 s%X", x); break;
                    case 0x07: snprintf(buf, buf_len, "SL1 s%X", x); break;
                    case 0x08: snprintf(buf, buf_len, "SRA s%X", x); break;
                    case 0x0A: snprintf(buf, buf_len, "SRX s%X", x); break;
                    case 0x0C: snprintf(buf, buf_len, "RR s%X", x); break;
                    case 0x0E: snprintf(buf, buf_len, "SR0 s%X", x); break;
                    case 0x0F: snprintf(buf, buf_len, "SR1 s%X", x); break;
                    default: snprintf(buf, buf_len, "INV 0x%05X", instr); break;
                }
            }
            break;
        case 0x37: snprintf(buf, buf_len, "REGBANK %c", (instr & 1) ? 'B' : 'A'); break;
        case 0x08: snprintf(buf, buf_len, "INPUT s%X, (s%X)", x, y); break;
        case 0x09: snprintf(buf, buf_len, "INPUT s%X, 0x%02X", x, kk); break;
        case 0x2C: snprintf(buf, buf_len, "OUTPUT s%X, (s%X)", x, y); break;
        case 0x2D: snprintf(buf, buf_len, "OUTPUT s%X, 0x%02X", x, kk); break;
        case 0x2B: snprintf(buf, buf_len, "OUTPUTK 0x%02X, %u",
                            (unsigned)((instr >> 4) & 0xFF), (unsigned)(instr & 0x0F));
                   break;
        case 0x2E: snprintf(buf, buf_len, "STORE s%X, (s%X)", x, y); break;
        case 0x2F: snprintf(buf, buf_len, "STORE s%X, 0x%02X", x, kk & 0x3F); break;
        case 0x0A: snprintf(buf, buf_len, "FETCH s%X, (s%X)", x, y); break;
        case 0x0B: snprintf(buf, buf_len, "FETCH s%X, 0x%02X", x, kk & 0x3F); break;
        case 0x22: snprintf(buf, buf_len, "JUMP 0x%03X", aaa); break;
        case 0x32: snprintf(buf, buf_len, "JUMP Z, 0x%03X", aaa); break;
        case 0x36: snprintf(buf, buf_len, "JUMP NZ, 0x%03X", aaa); break;
        case 0x3A: snprintf(buf, buf_len, "JUMP C, 0x%03X", aaa); break;
        case 0x3E: snprintf(buf, buf_len, "JUMP NC, 0x%03X", aaa); break;
        case 0x26: snprintf(buf, buf_len, "JUMP@ (s%X, s%X)", x, y); break;
        case 0x20: snprintf(buf, buf_len, "CALL 0x%03X", aaa); break;
        case 0x30: snprintf(buf, buf_len, "CALL Z, 0x%03X", aaa); break;
        case 0x34: snprintf(buf, buf_len, "CALL NZ, 0x%03X", aaa); break;
        case 0x38: snprintf(buf, buf_len, "CALL C, 0x%03X", aaa); break;
        case 0x3C: snprintf(buf, buf_len, "CALL NC, 0x%03X", aaa); break;
        case 0x24: snprintf(buf, buf_len, "CALL@ (s%X, s%X)", x, y); break;
        case 0x25: snprintf(buf, buf_len, "RETURN"); break;
        case 0x31: snprintf(buf, buf_len, "RETURN Z"); break;
        case 0x35: snprintf(buf, buf_len, "RETURN NZ"); break;
        case 0x39: snprintf(buf, buf_len, "RETURN C"); break;
        case 0x3D: snprintf(buf, buf_len, "RETURN NC"); break;
        case 0x21: snprintf(buf, buf_len, "LOAD&RETURN s%X, 0x%02X", x, kk); break;
        case 0x28: snprintf(buf, buf_len, "%s INTERRUPT",
                            (instr & 1) ? "ENABLE" : "DISABLE");
                   break;
        case 0x29: snprintf(buf, buf_len, "RETURNI %s",
                            (instr & 1) ? "ENABLE" : "DISABLE");
                   break;
        default: snprintf(buf, buf_len, "INV 0x%05X", instr); break;
    }
}
