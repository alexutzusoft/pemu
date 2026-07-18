#ifndef CP1600_H
#define CP1600_H

#include <stdint.h>
#include <stddef.h>

void* cp1600_create(void);
void cp1600_destroy(void *context);
int cp1600_init(void *context);
int cp1600_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int cp1600_step(void *context);
void cp1600_print_state(void *context);
void cp1600_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // CP1600_H
