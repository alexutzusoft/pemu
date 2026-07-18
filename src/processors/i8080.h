#ifndef I8080_H
#define I8080_H

#include <stdint.h>
#include <stddef.h>

void* i8080_create(void);
void i8080_destroy(void *context);
int i8080_init(void *context);
int i8080_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i8080_step(void *context);
void i8080_print_state(void *context);
void i8080_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I8080_H
