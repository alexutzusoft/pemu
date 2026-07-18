#ifndef W65816_H
#define W65816_H

#include <stdint.h>
#include <stddef.h>

void* w65816_create(void);
void w65816_destroy(void *context);
int w65816_init(void *context);
int w65816_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int w65816_step(void *context);
void w65816_print_state(void *context);
void w65816_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
