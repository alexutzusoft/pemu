#ifndef I8008_H
#define I8008_H

#include <stdint.h>
#include <stddef.h>

void* i8008_create(void);
void i8008_destroy(void *context);
int i8008_init(void *context);
int i8008_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i8008_step(void *context);
void i8008_print_state(void *context);
void i8008_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I8008_H
