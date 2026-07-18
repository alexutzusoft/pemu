#ifndef S2650_H
#define S2650_H

#include <stdint.h>
#include <stddef.h>

void* s2650_create(void);
void s2650_destroy(void *context);
int s2650_init(void *context);
int s2650_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int s2650_step(void *context);
void s2650_print_state(void *context);
void s2650_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // S2650_H
