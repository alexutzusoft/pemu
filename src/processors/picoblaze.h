#ifndef PICOBLAZE_H
#define PICOBLAZE_H

#include <stdint.h>
#include <stddef.h>

void* picoblaze_create(void);
void picoblaze_destroy(void *context);
int picoblaze_init(void *context);
int picoblaze_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int picoblaze_step(void *context);
void picoblaze_print_state(void *context);
void picoblaze_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // PICOBLAZE_H
