#ifndef I80186_H
#define I80186_H

#include <stdint.h>
#include <stddef.h>

void* i80186_create(void);
void i80186_destroy(void *context);
int i80186_init(void *context);
int i80186_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int i80186_step(void *context);
void i80186_print_state(void *context);
void i80186_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // I80186_H
