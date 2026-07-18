#ifndef CHIP8_H
#define CHIP8_H

#include <stdint.h>
#include <stddef.h>

void* chip8_create(void);
void chip8_destroy(void *context);
int chip8_init(void *context);
int chip8_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int chip8_step(void *context);
void chip8_print_state(void *context);
void chip8_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // CHIP8_H
