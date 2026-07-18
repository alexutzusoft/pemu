#ifndef NECV20_H
#define NECV20_H

#include <stdint.h>
#include <stddef.h>

void* necv20_create(void);
void necv20_destroy(void *context);
int necv20_init(void *context);
int necv20_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int necv20_step(void *context);
void necv20_print_state(void *context);
void necv20_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // NECV20_H
