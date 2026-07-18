#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <stdint.h>
#include <stddef.h>

typedef struct ProcessorInfo {
    const char *name;
    const char *description;
    uint32_t default_speed_hz;
    
    // Lifecycle
    void* (*create)(void);
    void (*destroy)(void *context);
    
    // Operation
    int (*init)(void *context);
    int (*load)(void *context, const uint8_t *data, size_t size, uint32_t address);
    int (*step)(void *context); // Returns 0 on success, 1 on halt, negative on error
    void (*print_state)(void *context);
    void (*get_disassembly)(void *context, char *buf, size_t buf_len);
} ProcessorInfo;

// Registry functions
const ProcessorInfo* processor_find(const char *name);
void processor_list_all(void);

#endif // PROCESSOR_H
