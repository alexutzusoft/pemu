#ifndef ARM7TDMI_H
#define ARM7TDMI_H

#include <stdint.h>
#include <stddef.h>

void* arm7tdmi_create(void);
void arm7tdmi_destroy(void *context);
int arm7tdmi_init(void *context);
int arm7tdmi_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int arm7tdmi_step(void *context);
void arm7tdmi_print_state(void *context);
void arm7tdmi_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // ARM7TDMI_H
