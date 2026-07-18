#ifndef RV32I_H
#define RV32I_H

#include <stdint.h>
#include <stddef.h>

void* rv32i_create(void);
void rv32i_destroy(void *context);
int rv32i_init(void *context);
int rv32i_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int rv32i_step(void *context);
void rv32i_print_state(void *context);
void rv32i_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // RV32I_H
