#ifndef Z80_H
#define Z80_H

#include <stdint.h>
#include <stddef.h>

void* z80_create(void);
void z80_destroy(void *context);
int z80_init(void *context);
int z80_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int z80_step(void *context);
void z80_print_state(void *context);
void z80_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // Z80_H
