#ifndef I8086_H
#define I8086_H

#include <stdint.h>
#include <stddef.h>

void* i8086_create(void);
void i8086_destroy(void *context);
int i8086_init(void *context);
int i8086_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i8086_step(void *context);
void i8086_print_state(void *context);
void i8086_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I8086_H
