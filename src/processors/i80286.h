#ifndef I80286_H
#define I80286_H

#include <stdint.h>
#include <stddef.h>

void* i80286_create(void);
void i80286_destroy(void *context);
int i80286_init(void *context);
int i80286_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i80286_step(void *context);
void i80286_print_state(void *context);
void i80286_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I80286_H
