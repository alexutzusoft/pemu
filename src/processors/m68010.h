#ifndef M68010_H
#define M68010_H

#include <stdint.h>
#include <stddef.h>

void* m68010_create(void);
void m68010_destroy(void *context);
int m68010_init(void *context);
int m68010_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int m68010_step(void *context);
void m68010_print_state(void *context);
void m68010_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // M68010_H
