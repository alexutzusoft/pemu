#ifndef MC6800_H
#define MC6800_H

#include <stdint.h>
#include <stddef.h>

void* mc6800_create(void);
void mc6800_destroy(void *context);
int mc6800_init(void *context);
int mc6800_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mc6800_step(void *context);
void mc6800_print_state(void *context);
void mc6800_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
