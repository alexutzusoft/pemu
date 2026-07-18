#ifndef DUMMY_H
#define DUMMY_H

#include <stdint.h>
#include <stddef.h>

void* dummy_create(void);
void dummy_destroy(void *context);
int dummy_init(void *context);
int dummy_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int dummy_step(void *context);
void dummy_print_state(void *context);
void dummy_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // DUMMY_H
