#ifndef I4040_H
#define I4040_H

#include <stdint.h>
#include <stddef.h>

void* i4040_create(void);
void i4040_destroy(void *context);
int i4040_init(void *context);
int i4040_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i4040_step(void *context);
void i4040_print_state(void *context);
void i4040_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I4040_H
