#ifndef NOVA_H
#define NOVA_H

#include <stdint.h>
#include <stddef.h>

void* nova_create(void);
void nova_destroy(void *context);
int nova_init(void *context);
int nova_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int nova_step(void *context);
void nova_print_state(void *context);
void nova_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // NOVA_H
