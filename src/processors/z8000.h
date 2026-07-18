#ifndef Z8000_H
#define Z8000_H

#include <stdint.h>
#include <stddef.h>

void* z8000_create(void);
void z8000_destroy(void *context);
int z8000_init(void *context);
int z8000_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int z8000_step(void *context);
void z8000_print_state(void *context);
void z8000_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // Z8000_H
