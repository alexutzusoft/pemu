#ifndef I8048_H
#define I8048_H

#include <stdint.h>
#include <stddef.h>

void* i8048_create(void);
void i8048_destroy(void *context);
int i8048_init(void *context);
int i8048_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i8048_step(void *context);
void i8048_print_state(void *context);
void i8048_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I8048_H
