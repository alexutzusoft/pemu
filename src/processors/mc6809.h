#ifndef MC6809_H
#define MC6809_H

#include <stdint.h>
#include <stddef.h>

void* mc6809_create(void);
void mc6809_destroy(void *context);
int mc6809_init(void *context);
int mc6809_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mc6809_step(void *context);
void mc6809_print_state(void *context);
void mc6809_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
