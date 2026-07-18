#ifndef SM83_H
#define SM83_H

#include <stdint.h>
#include <stddef.h>

void* sm83_create(void);
void sm83_destroy(void *context);
int sm83_init(void *context);
int sm83_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int sm83_step(void *context);
void sm83_print_state(void *context);
void sm83_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // SM83_H
