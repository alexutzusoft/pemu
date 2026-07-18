#ifndef ATMEGA328P_H
#define ATMEGA328P_H

#include <stdint.h>
#include <stddef.h>

void* atmega328p_create(void);
void atmega328p_destroy(void *context);
int atmega328p_init(void *context);
int atmega328p_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int atmega328p_step(void *context);
void atmega328p_print_state(void *context);
void atmega328p_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // ATMEGA328P_H
