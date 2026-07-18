#ifndef LC3_H
#define LC3_H

#include <stdint.h>
#include <stddef.h>

void* lc3_create(void);
void lc3_destroy(void *context);
int lc3_init(void *context);
int lc3_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int lc3_step(void *context);
void lc3_print_state(void *context);
void lc3_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // LC3_H
