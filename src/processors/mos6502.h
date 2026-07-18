#ifndef MOS6502_H
#define MOS6502_H

#include <stdint.h>
#include <stddef.h>

void* mos6502_create(void);
void mos6502_destroy(void *context);
int mos6502_init(void *context);
int mos6502_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int mos6502_step(void *context);
void mos6502_print_state(void *context);
void mos6502_get_disassembly(void *context, char *buf, size_t buf_len);

// Screen rendering interface
void mos6502_render_screen(void *context, uint32_t *display_buffer);

#endif
