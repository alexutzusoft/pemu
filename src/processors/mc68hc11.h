#ifndef MC68HC11_H
#define MC68HC11_H

#include <stdint.h>
#include <stddef.h>

void* mc68hc11_create(void);
void mc68hc11_destroy(void *context);
int mc68hc11_init(void *context);
int mc68hc11_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mc68hc11_step(void *context);
void mc68hc11_print_state(void *context);
void mc68hc11_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
