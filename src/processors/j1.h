#ifndef J1_H
#define J1_H

#include <stdint.h>
#include <stddef.h>

void* j1_create(void);
void j1_destroy(void *context);
int j1_init(void *context);
int j1_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int j1_step(void *context);
void j1_print_state(void *context);
void j1_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // J1_H
