#ifndef MIPS32_H
#define MIPS32_H

#include <stdint.h>
#include <stddef.h>

void* mips32_create(void);
void mips32_destroy(void *context);
int mips32_init(void *context);
int mips32_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mips32_step(void *context);
void mips32_print_state(void *context);
void mips32_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // MIPS32_H
