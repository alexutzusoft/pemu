#ifndef HD6309_H
#define HD6309_H

#include <stdint.h>
#include <stddef.h>

void* hd6309_create(void);
void hd6309_destroy(void *context);
int hd6309_init(void *context);
int hd6309_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int hd6309_step(void *context);
void hd6309_print_state(void *context);
void hd6309_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
