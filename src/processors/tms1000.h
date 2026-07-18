#ifndef TMS1000_H
#define TMS1000_H

#include <stdint.h>
#include <stddef.h>

void* tms1000_create(void);
void tms1000_destroy(void *context);
int tms1000_init(void *context);
int tms1000_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int tms1000_step(void *context);
void tms1000_print_state(void *context);
void tms1000_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // TMS1000_H
