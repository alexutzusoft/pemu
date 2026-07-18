#ifndef CDP1802_H
#define CDP1802_H

#include <stdint.h>
#include <stddef.h>

void* cdp1802_create(void);
void cdp1802_destroy(void *context);
int cdp1802_init(void *context);
int cdp1802_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int cdp1802_step(void *context);
void cdp1802_print_state(void *context);
void cdp1802_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // CDP1802_H
