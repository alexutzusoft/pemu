#ifndef W65C02_H
#define W65C02_H

#include <stdint.h>
#include <stddef.h>

void* w65c02_create(void);
void w65c02_destroy(void *context);
int w65c02_init(void *context);
int w65c02_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int w65c02_step(void *context);
void w65c02_print_state(void *context);
void w65c02_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
