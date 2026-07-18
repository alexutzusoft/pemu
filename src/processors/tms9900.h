#ifndef TMS9900_H
#define TMS9900_H

#include <stdint.h>
#include <stddef.h>

void* tms9900_create(void);
void tms9900_destroy(void *context);
int tms9900_init(void *context);
int tms9900_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int tms9900_step(void *context);
void tms9900_print_state(void *context);
void tms9900_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // TMS9900_H
