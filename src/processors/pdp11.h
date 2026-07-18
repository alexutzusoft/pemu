#ifndef PDP11_H
#define PDP11_H

#include <stdint.h>
#include <stddef.h>

void* pdp11_create(void);
void pdp11_destroy(void *context);
int pdp11_init(void *context);
int pdp11_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int pdp11_step(void *context);
void pdp11_print_state(void *context);
void pdp11_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // PDP11_H
