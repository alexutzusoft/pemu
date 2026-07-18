#include "dummy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct DummyCPU {
    uint32_t pc;
    uint32_t ticks;
    uint8_t memory[256];
} DummyCPU;

void* dummy_create(void) {
    DummyCPU *cpu = (DummyCPU*)calloc(1, sizeof(DummyCPU));
    return cpu;
}

void dummy_destroy(void *context) {
    free(context);
}

int dummy_init(void *context) {
    if (!context) return -1;
    DummyCPU *cpu = (DummyCPU*)context;
    cpu->pc = 0;
    cpu->ticks = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    return 0;
}

int dummy_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    DummyCPU *cpu = (DummyCPU*)context;
    if (address >= sizeof(cpu->memory)) return -2;
    size_t copy_len = size;
    if (address + size > sizeof(cpu->memory)) {
        copy_len = sizeof(cpu->memory) - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int dummy_step(void *context) {
    if (!context) return -1;
    DummyCPU *cpu = (DummyCPU*)context;
    
    if (cpu->pc >= sizeof(cpu->memory)) {
        return -3; // Out of bounds execution
    }
    
    uint8_t opcode = cpu->memory[cpu->pc];
    cpu->ticks++;
    
    if (opcode == 0xFF) {
        // Halt opcode
        return 1; 
    }
    
    // Normal step, increment PC
    cpu->pc = (cpu->pc + 1) % sizeof(cpu->memory);
    return 0;
}

void dummy_print_state(void *context) {
    if (!context) return;
    DummyCPU *cpu = (DummyCPU*)context;
    printf("Dummy CPU State: PC=0x%02X, Ticks=%u, Opcode=0x%02X\n", 
           cpu->pc, cpu->ticks, cpu->memory[cpu->pc]);
}

void dummy_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    DummyCPU *cpu = (DummyCPU*)context;
    
    uint8_t opcode = cpu->memory[cpu->pc];
    if (opcode == 0xFF) {
        snprintf(buf, buf_len, "HALT (0xFF)");
    } else {
        snprintf(buf, buf_len, "STEP (0x%02X)", opcode);
    }
}
