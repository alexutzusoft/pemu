#ifndef I8051_H
#define I8051_H

#include <stdint.h>
#include <stddef.h>

void* i8051_create(void);
void i8051_destroy(void *context);
int i8051_init(void *context);
int i8051_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i8051_step(void *context);
void i8051_print_state(void *context);
void i8051_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I8051_H
