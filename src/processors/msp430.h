#ifndef MSP430_H
#define MSP430_H

#include <stdint.h>
#include <stddef.h>

void* msp430_create(void);
void msp430_destroy(void *context);
int msp430_init(void *context);
int msp430_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int msp430_step(void *context);
void msp430_print_state(void *context);
void msp430_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // MSP430_H
