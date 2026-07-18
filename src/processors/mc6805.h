#ifndef MC6805_H
#define MC6805_H

#include <stdint.h>
#include <stddef.h>

void* mc6805_create(void);
void mc6805_destroy(void *context);
int mc6805_init(void *context);
int mc6805_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mc6805_step(void *context);
void mc6805_print_state(void *context);
void mc6805_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
