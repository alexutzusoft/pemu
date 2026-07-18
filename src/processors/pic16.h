#ifndef PIC16_H
#define PIC16_H

#include <stdint.h>
#include <stddef.h>

void* pic16_create(void);
void pic16_destroy(void *context);
int pic16_init(void *context);
int pic16_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int pic16_step(void *context);
void pic16_print_state(void *context);
void pic16_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // PIC16_H
