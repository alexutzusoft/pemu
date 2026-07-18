#ifndef SH2_H
#define SH2_H

#include <stdint.h>
#include <stddef.h>

void* sh2_create(void);
void sh2_destroy(void *context);
int sh2_init(void *context);
int sh2_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int sh2_step(void *context);
void sh2_print_state(void *context);
void sh2_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // SH2_H
