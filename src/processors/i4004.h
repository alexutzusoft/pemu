#ifndef I4004_H
#define I4004_H

#include <stdint.h>
#include <stddef.h>

void* i4004_create(void);
void i4004_destroy(void *context);
int i4004_init(void *context);
int i4004_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i4004_step(void *context);
void i4004_print_state(void *context);
void i4004_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I4004_H
