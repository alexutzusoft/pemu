#ifndef Z180_H
#define Z180_H

#include <stdint.h>
#include <stddef.h>

void* z180_create(void);
void z180_destroy(void *context);
int z180_init(void *context);
int z180_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int z180_step(void *context);
void z180_print_state(void *context);
void z180_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // Z180_H
