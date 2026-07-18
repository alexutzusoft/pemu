#ifndef F8_H
#define F8_H

#include <stdint.h>
#include <stddef.h>

void* f8_create(void);
void f8_destroy(void *context);
int f8_init(void *context);
int f8_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int f8_step(void *context);
void f8_print_state(void *context);
void f8_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // F8_H
