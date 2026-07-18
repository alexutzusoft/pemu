#ifndef RV64I_H
#define RV64I_H

#include <stdint.h>
#include <stddef.h>

void* rv64i_create(void);
void rv64i_destroy(void *context);
int rv64i_init(void *context);
int rv64i_load(void *context, const uint8_t *data, size_t size, uint32_t address);
int rv64i_step(void *context);
void rv64i_print_state(void *context);
void rv64i_get_disassembly(void *context, char *buf, size_t buf_len);

#endif // RV64I_H
