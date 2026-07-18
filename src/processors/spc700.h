#ifndef SPC700_H
#define SPC700_H

#include <stdint.h>
#include <stddef.h>

void* spc700_create(void);
void spc700_destroy(void *context);
int spc700_init(void *context);
int spc700_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int spc700_step(void *context);
void spc700_print_state(void *context);
void spc700_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
