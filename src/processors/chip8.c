#include "chip8.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 4096
#define FONT_BASE 0x50
#define PROG_BASE 0x200
#define SCREEN_W 64
#define SCREEN_H 32
#define STACK_DEPTH 16
#define TIMER_DIVIDER 8 // ~500Hz step rate / 8 ~= 60Hz timer tick
#define RNG_SEED 0x2A6D3E51u

typedef struct Chip8CPU {
    uint8_t v[16];       // V0-VF registers
    uint16_t i;          // Index register
    uint16_t pc;         // Program counter
    uint16_t stack[STACK_DEPTH];
    uint8_t sp;          // Stack pointer (points to next free slot)
    uint8_t delay_timer;
    uint8_t sound_timer;
    uint8_t keys[16];    // 1 = pressed
    uint8_t framebuffer[SCREEN_W * SCREEN_H]; // 1 = pixel on
    uint8_t memory[MEM_SIZE];
    uint32_t rng_state;
    uint32_t ticks;
    int halted;
} Chip8CPU;

static const uint8_t fontset[80] = {
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0x80, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80  // F
};

static uint8_t next_random(Chip8CPU *cpu) {
    // xorshift32 PRNG
    uint32_t x = cpu->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    cpu->rng_state = x;
    return (uint8_t)(x & 0xFF);
}

static uint16_t fetch_opcode(const Chip8CPU *cpu, uint16_t addr) {
    return (uint16_t)((cpu->memory[addr & (MEM_SIZE - 1)] << 8) |
                      cpu->memory[(addr + 1) & (MEM_SIZE - 1)]);
}

static void draw_sprite(Chip8CPU *cpu, uint8_t vx, uint8_t vy, uint8_t n) {
    cpu->v[0xF] = 0;
    for (uint8_t row = 0; row < n; ++row) {
        uint8_t sprite = cpu->memory[(cpu->i + row) & (MEM_SIZE - 1)];
        for (uint8_t col = 0; col < 8; ++col) {
            if (!(sprite & (0x80 >> col))) continue;
            uint8_t px = (uint8_t)((vx + col) % SCREEN_W);
            uint8_t py = (uint8_t)((vy + row) % SCREEN_H);
            uint8_t *pixel = &cpu->framebuffer[py * SCREEN_W + px];
            if (*pixel) cpu->v[0xF] = 1;
            *pixel ^= 1;
        }
    }
}

void* chip8_create(void) {
    Chip8CPU *cpu = (Chip8CPU*)calloc(1, sizeof(Chip8CPU));
    return cpu;
}

void chip8_destroy(void *context) {
    free(context);
}

int chip8_init(void *context) {
    if (!context) return -1;
    Chip8CPU *cpu = (Chip8CPU*)context;
    memset(cpu, 0, sizeof(Chip8CPU));
    memcpy(&cpu->memory[FONT_BASE], fontset, sizeof(fontset));
    cpu->pc = PROG_BASE;
    cpu->rng_state = RNG_SEED;
    return 0;
}

int chip8_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context || !data) return -1;
    Chip8CPU *cpu = (Chip8CPU*)context;
    if (address == 0) address = PROG_BASE; // ROMs load at 0x200 by convention
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int chip8_step(void *context) {
    if (!context) return -1;
    Chip8CPU *cpu = (Chip8CPU*)context;
    if (cpu->halted) return 1;

    uint16_t opcode = fetch_opcode(cpu, cpu->pc);
    uint8_t x = (opcode >> 8) & 0x0F;
    uint8_t y = (opcode >> 4) & 0x0F;
    uint8_t n = opcode & 0x0F;
    uint8_t nn = opcode & 0xFF;
    uint16_t nnn = opcode & 0x0FFF;
    uint16_t opcode_addr = cpu->pc;

    cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
    cpu->ticks++;

    // Approximate 60Hz timers at ~500Hz step rate
    if (cpu->ticks % TIMER_DIVIDER == 0) {
        if (cpu->delay_timer > 0) cpu->delay_timer--;
        if (cpu->sound_timer > 0) cpu->sound_timer--;
    }

    switch (opcode >> 12) {
        case 0x0:
            if (opcode == 0x00E0) { // CLS
                memset(cpu->framebuffer, 0, sizeof(cpu->framebuffer));
            } else if (opcode == 0x00EE) { // RET
                if (cpu->sp == 0) return -3; // Stack underflow
                cpu->sp--;
                cpu->pc = cpu->stack[cpu->sp];
            } else {
                return -2; // 0NNN (RCA 1802 call) unsupported
            }
            break;
        case 0x1: // JP nnn
            if (nnn == opcode_addr) { // Infinite jump-to-self: halt
                cpu->halted = 1;
                return 1;
            }
            cpu->pc = nnn;
            break;
        case 0x2: // CALL nnn
            if (cpu->sp >= STACK_DEPTH) return -3; // Stack overflow
            cpu->stack[cpu->sp++] = cpu->pc;
            cpu->pc = nnn;
            break;
        case 0x3: // SE Vx, nn
            if (cpu->v[x] == nn) cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
            break;
        case 0x4: // SNE Vx, nn
            if (cpu->v[x] != nn) cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
            break;
        case 0x5: // SE Vx, Vy
            if (n != 0) return -2;
            if (cpu->v[x] == cpu->v[y]) cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
            break;
        case 0x6: // LD Vx, nn
            cpu->v[x] = nn;
            break;
        case 0x7: // ADD Vx, nn
            cpu->v[x] = (uint8_t)(cpu->v[x] + nn);
            break;
        case 0x8: {
            uint8_t vx = cpu->v[x];
            uint8_t vy = cpu->v[y];
            switch (n) {
                case 0x0: cpu->v[x] = vy; break;        // LD Vx, Vy
                case 0x1: cpu->v[x] = vx | vy; break;   // OR
                case 0x2: cpu->v[x] = vx & vy; break;   // AND
                case 0x3: cpu->v[x] = vx ^ vy; break;   // XOR
                case 0x4: // ADD with carry
                    cpu->v[x] = (uint8_t)(vx + vy);
                    cpu->v[0xF] = ((uint16_t)vx + vy > 0xFF) ? 1 : 0;
                    break;
                case 0x5: // SUB Vx, Vy
                    cpu->v[x] = (uint8_t)(vx - vy);
                    cpu->v[0xF] = (vx >= vy) ? 1 : 0;
                    break;
                case 0x6: // SHR Vx
                    cpu->v[x] = vx >> 1;
                    cpu->v[0xF] = vx & 1;
                    break;
                case 0x7: // SUBN Vx, Vy
                    cpu->v[x] = (uint8_t)(vy - vx);
                    cpu->v[0xF] = (vy >= vx) ? 1 : 0;
                    break;
                case 0xE: // SHL Vx
                    cpu->v[x] = (uint8_t)(vx << 1);
                    cpu->v[0xF] = (vx >> 7) & 1;
                    break;
                default: return -2;
            }
            break;
        }
        case 0x9: // SNE Vx, Vy
            if (n != 0) return -2;
            if (cpu->v[x] != cpu->v[y]) cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
            break;
        case 0xA: // LD I, nnn
            cpu->i = nnn;
            break;
        case 0xB: // JP V0, nnn
            cpu->pc = (nnn + cpu->v[0]) & (MEM_SIZE - 1);
            break;
        case 0xC: // RND Vx, nn
            cpu->v[x] = next_random(cpu) & nn;
            break;
        case 0xD: // DRW Vx, Vy, n
            draw_sprite(cpu, cpu->v[x], cpu->v[y], n);
            break;
        case 0xE:
            if (nn == 0x9E) { // SKP Vx
                if (cpu->keys[cpu->v[x] & 0x0F]) cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
            } else if (nn == 0xA1) { // SKNP Vx
                if (!cpu->keys[cpu->v[x] & 0x0F]) cpu->pc = (cpu->pc + 2) & (MEM_SIZE - 1);
            } else {
                return -2;
            }
            break;
        case 0xF:
            switch (nn) {
                case 0x07: // LD Vx, DT
                    cpu->v[x] = cpu->delay_timer;
                    break;
                case 0x0A: { // LD Vx, K (wait for keypress)
                    int pressed = -1;
                    for (int k = 0; k < 16; ++k) {
                        if (cpu->keys[k]) { pressed = k; break; }
                    }
                    if (pressed < 0) {
                        cpu->pc = opcode_addr; // Repeat this instruction
                    } else {
                        cpu->v[x] = (uint8_t)pressed;
                    }
                    break;
                }
                case 0x15: // LD DT, Vx
                    cpu->delay_timer = cpu->v[x];
                    break;
                case 0x18: // LD ST, Vx
                    cpu->sound_timer = cpu->v[x];
                    break;
                case 0x1E: // ADD I, Vx
                    cpu->i = (cpu->i + cpu->v[x]) & 0x0FFF;
                    break;
                case 0x29: // LD F, Vx (font sprite address)
                    cpu->i = FONT_BASE + (uint16_t)(cpu->v[x] & 0x0F) * 5;
                    break;
                case 0x33: { // BCD Vx
                    uint8_t val = cpu->v[x];
                    cpu->memory[cpu->i & (MEM_SIZE - 1)] = val / 100;
                    cpu->memory[(cpu->i + 1) & (MEM_SIZE - 1)] = (val / 10) % 10;
                    cpu->memory[(cpu->i + 2) & (MEM_SIZE - 1)] = val % 10;
                    break;
                }
                case 0x55: // LD [I], V0..Vx
                    for (uint8_t r = 0; r <= x; ++r) {
                        cpu->memory[(cpu->i + r) & (MEM_SIZE - 1)] = cpu->v[r];
                    }
                    break;
                case 0x65: // LD V0..Vx, [I]
                    for (uint8_t r = 0; r <= x; ++r) {
                        cpu->v[r] = cpu->memory[(cpu->i + r) & (MEM_SIZE - 1)];
                    }
                    break;
                default: return -2;
            }
            break;
        default:
            return -2; // Unreachable, but keeps the compiler happy
    }

    return 0;
}

void chip8_print_state(void *context) {
    if (!context) return;
    Chip8CPU *cpu = (Chip8CPU*)context;

    printf("CHIP-8 State: PC=0x%03X I=0x%03X SP=%u DT=%u ST=%u Ticks=%u%s\n",
           cpu->pc, cpu->i, cpu->sp, cpu->delay_timer, cpu->sound_timer,
           cpu->ticks, cpu->halted ? " [HALTED]" : "");
    for (int r = 0; r < 16; ++r) {
        printf("V%X=0x%02X%s", r, cpu->v[r], (r % 8 == 7) ? "\n" : " ");
    }
    printf("Display:\n");
    for (int row = 0; row < SCREEN_H; ++row) {
        char line[SCREEN_W + 1];
        for (int col = 0; col < SCREEN_W; ++col) {
            line[col] = cpu->framebuffer[row * SCREEN_W + col] ? '#' : '.';
        }
        line[SCREEN_W] = '\0';
        printf("%s\n", line);
    }
}

void chip8_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    Chip8CPU *cpu = (Chip8CPU*)context;

    uint16_t opcode = fetch_opcode(cpu, cpu->pc);
    uint8_t x = (opcode >> 8) & 0x0F;
    uint8_t y = (opcode >> 4) & 0x0F;
    uint8_t n = opcode & 0x0F;
    uint8_t nn = opcode & 0xFF;
    uint16_t nnn = opcode & 0x0FFF;

    switch (opcode >> 12) {
        case 0x0:
            if (opcode == 0x00E0) snprintf(buf, buf_len, "CLS");
            else if (opcode == 0x00EE) snprintf(buf, buf_len, "RET");
            else snprintf(buf, buf_len, "SYS 0x%03X", nnn);
            break;
        case 0x1: snprintf(buf, buf_len, "JP 0x%03X", nnn); break;
        case 0x2: snprintf(buf, buf_len, "CALL 0x%03X", nnn); break;
        case 0x3: snprintf(buf, buf_len, "SE V%X, 0x%02X", x, nn); break;
        case 0x4: snprintf(buf, buf_len, "SNE V%X, 0x%02X", x, nn); break;
        case 0x5:
            if (n == 0) snprintf(buf, buf_len, "SE V%X, V%X", x, y);
            else snprintf(buf, buf_len, "DB 0x%04X", opcode);
            break;
        case 0x6: snprintf(buf, buf_len, "LD V%X, 0x%02X", x, nn); break;
        case 0x7: snprintf(buf, buf_len, "ADD V%X, 0x%02X", x, nn); break;
        case 0x8:
            switch (n) {
                case 0x0: snprintf(buf, buf_len, "LD V%X, V%X", x, y); break;
                case 0x1: snprintf(buf, buf_len, "OR V%X, V%X", x, y); break;
                case 0x2: snprintf(buf, buf_len, "AND V%X, V%X", x, y); break;
                case 0x3: snprintf(buf, buf_len, "XOR V%X, V%X", x, y); break;
                case 0x4: snprintf(buf, buf_len, "ADD V%X, V%X", x, y); break;
                case 0x5: snprintf(buf, buf_len, "SUB V%X, V%X", x, y); break;
                case 0x6: snprintf(buf, buf_len, "SHR V%X", x); break;
                case 0x7: snprintf(buf, buf_len, "SUBN V%X, V%X", x, y); break;
                case 0xE: snprintf(buf, buf_len, "SHL V%X", x); break;
                default: snprintf(buf, buf_len, "DB 0x%04X", opcode); break;
            }
            break;
        case 0x9:
            if (n == 0) snprintf(buf, buf_len, "SNE V%X, V%X", x, y);
            else snprintf(buf, buf_len, "DB 0x%04X", opcode);
            break;
        case 0xA: snprintf(buf, buf_len, "LD I, 0x%03X", nnn); break;
        case 0xB: snprintf(buf, buf_len, "JP V0, 0x%03X", nnn); break;
        case 0xC: snprintf(buf, buf_len, "RND V%X, 0x%02X", x, nn); break;
        case 0xD: snprintf(buf, buf_len, "DRW V%X, V%X, %u", x, y, n); break;
        case 0xE:
            if (nn == 0x9E) snprintf(buf, buf_len, "SKP V%X", x);
            else if (nn == 0xA1) snprintf(buf, buf_len, "SKNP V%X", x);
            else snprintf(buf, buf_len, "DB 0x%04X", opcode);
            break;
        case 0xF:
            switch (nn) {
                case 0x07: snprintf(buf, buf_len, "LD V%X, DT", x); break;
                case 0x0A: snprintf(buf, buf_len, "LD V%X, K", x); break;
                case 0x15: snprintf(buf, buf_len, "LD DT, V%X", x); break;
                case 0x18: snprintf(buf, buf_len, "LD ST, V%X", x); break;
                case 0x1E: snprintf(buf, buf_len, "ADD I, V%X", x); break;
                case 0x29: snprintf(buf, buf_len, "LD F, V%X", x); break;
                case 0x33: snprintf(buf, buf_len, "BCD V%X", x); break;
                case 0x55: snprintf(buf, buf_len, "LD [I], V0-V%X", x); break;
                case 0x65: snprintf(buf, buf_len, "LD V0-V%X, [I]", x); break;
                default: snprintf(buf, buf_len, "DB 0x%04X", opcode); break;
            }
            break;
        default:
            snprintf(buf, buf_len, "DB 0x%04X", opcode);
            break;
    }
}
