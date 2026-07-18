#ifndef PDP8_H
#define PDP8_H

#include <stdint.h>
#include <stddef.h>

void* pdp8_create(void);
void pdp8_destroy(void *context);
int pdp8_init(void *context);
int pdp8_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int pdp8_step(void *context);
void pdp8_print_state(void *context);
void pdp8_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // PDP8_H
