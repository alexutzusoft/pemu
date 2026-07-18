#ifndef MOS6510_H
#define MOS6510_H

#include <stdint.h>
#include <stddef.h>

void* mos6510_create(void);
void mos6510_destroy(void *context);
int mos6510_init(void *context);
int mos6510_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mos6510_step(void *context);
void mos6510_print_state(void *context);
void mos6510_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
