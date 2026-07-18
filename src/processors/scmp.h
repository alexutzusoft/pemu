#ifndef SCMP_H
#define SCMP_H

#include <stdint.h>
#include <stddef.h>

void* scmp_create(void);
void scmp_destroy(void *context);
int scmp_init(void *context);
int scmp_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int scmp_step(void *context);
void scmp_print_state(void *context);
void scmp_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // SCMP_H
