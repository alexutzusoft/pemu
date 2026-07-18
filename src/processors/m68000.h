#ifndef M68000_H
#define M68000_H

#include <stdint.h>
#include <stddef.h>

void* m68000_create(void);
void m68000_destroy(void *context);
int m68000_init(void *context);
int m68000_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int m68000_step(void *context);
void m68000_print_state(void *context);
void m68000_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // M68000_H
