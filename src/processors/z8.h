#ifndef Z8_H
#define Z8_H

#include <stdint.h>
#include <stddef.h>

void* z8_create(void);
void z8_destroy(void *context);
int z8_init(void *context);
int z8_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int z8_step(void *context);
void z8_print_state(void *context);
void z8_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // Z8_H
