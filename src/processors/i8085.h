#ifndef I8085_H
#define I8085_H

#include <stdint.h>
#include <stddef.h>

void* i8085_create(void);
void i8085_destroy(void *context);
int i8085_init(void *context);
int i8085_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i8085_step(void *context);
void i8085_print_state(void *context);
void i8085_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I8085_H
