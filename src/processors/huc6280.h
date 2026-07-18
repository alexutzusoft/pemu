#ifndef HUC6280_H
#define HUC6280_H

#include <stdint.h>
#include <stddef.h>

void* huc6280_create(void);
void huc6280_destroy(void *context);
int huc6280_init(void *context);
int huc6280_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int huc6280_step(void *context);
void huc6280_print_state(void *context);
void huc6280_get_disassembly(void *context, char *buf, size_t buf_len);

#endif
