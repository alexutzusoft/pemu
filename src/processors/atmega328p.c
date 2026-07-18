#include "atmega328p.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLASH_WORDS 16384 // 32 KB Flash (16K words of 16 bits)
#define SRAM_SIZE 2304    // 32 GP registers + 64 IO + 160 Ext IO + 2048 Internal SRAM

typedef struct ATmega328P_CPU {
    uint16_t flash[FLASH_WORDS];
    uint8_t data_space[SRAM_SIZE];
    uint16_t pc; // word address
    uint32_t ticks;
    int halted;
    
    // Timer helper
    uint32_t timer_tick_counter;
} ATmega328P_CPU;

#define SREG cpu->data_space[0x5F]
#define GET_FLAG(bit) ((SREG >> bit) & 1)
#define SET_FLAG(bit, val) do { if (val) SREG |= (1 << bit); else SREG &= ~(1 << bit); } while(0)

static const char* br_names_set[] = { "brcs", "breq", "brmi", "brvs", "brlt", "brhs", "brts", "brie" };
static const char* br_names_clr[] = { "brcc", "brne", "brpl", "brvc", "brge", "brlo", "brtc", "brid" };

void* atmega328p_create(void) {
    ATmega328P_CPU *cpu = (ATmega328P_CPU*)calloc(1, sizeof(ATmega328P_CPU));
    return cpu;
}

void atmega328p_destroy(void *context) {
    free(context);
}

int atmega328p_init(void *context) {
    if (!context) return -1;
    ATmega328P_CPU *cpu = (ATmega328P_CPU*)context;
    
    memset(cpu->flash, 0, sizeof(cpu->flash));
    memset(cpu->data_space, 0, sizeof(cpu->data_space));
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->timer_tick_counter = 0;
    
    // Stack Pointer SP initialized to top of SRAM (0x08FF)
    // SPH is at data address 0x5E, SPL at 0x5D
    cpu->data_space[0x5D] = 0xFF;
    cpu->data_space[0x5E] = 0x08;
    
    // UART state: UCSR0A UDRE0 is set by default
    cpu->data_space[0xC0] = 0x20;
    
    return 0;
}

int atmega328p_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    ATmega328P_CPU *cpu = (ATmega328P_CPU*)context;
    
    uint32_t byte_limit = FLASH_WORDS * 2;
    if (address >= byte_limit) return -2;
    
    size_t copy_len = size;
    if (address + size > byte_limit) {
        copy_len = byte_limit - address;
    }
    
    memcpy(((uint8_t*)cpu->flash) + address, data, copy_len);
    return 0;
}

static inline void push_stack(ATmega328P_CPU *cpu, uint16_t addr) {
    uint16_t sp = ((uint16_t)cpu->data_space[0x5E] << 8) | cpu->data_space[0x5D];
    
    if (sp < SRAM_SIZE) cpu->data_space[sp] = addr & 0xFF;
    sp--;
    if (sp < SRAM_SIZE) cpu->data_space[sp] = (addr >> 8) & 0xFF;
    sp--;
    
    cpu->data_space[0x5E] = (sp >> 8) & 0xFF;
    cpu->data_space[0x5D] = sp & 0xFF;
}

static inline uint16_t pop_stack(ATmega328P_CPU *cpu) {
    uint16_t sp = ((uint16_t)cpu->data_space[0x5E] << 8) | cpu->data_space[0x5D];
    
    sp++;
    uint8_t high = 0;
    if (sp < SRAM_SIZE) high = cpu->data_space[sp];
    
    sp++;
    uint8_t low = 0;
    if (sp < SRAM_SIZE) low = cpu->data_space[sp];
    
    cpu->data_space[0x5E] = (sp >> 8) & 0xFF;
    cpu->data_space[0x5D] = sp & 0xFF;
    
    return ((uint16_t)high << 8) | low;
}

static inline void check_led_toggle(ATmega328P_CPU *cpu, uint8_t old_portb) {
    uint8_t ddrb = cpu->data_space[0x24];
    uint8_t new_portb = cpu->data_space[0x25];
    
    if (ddrb & 0x20) { // PB5 configured as output
        uint8_t old_val = (old_portb >> 5) & 1;
        uint8_t new_val = (new_portb >> 5) & 1;
        if (old_val == 0 && new_val == 1) {
            printf("\033[1;32m[LED ON]\033[0m\n");
        } else if (old_val == 1 && new_val == 0) {
            printf("\033[1;31m[LED OFF]\033[0m\n");
        }
    }
}

int atmega328p_step(void *context) {
    if (!context) return -1;
    ATmega328P_CPU *cpu = (ATmega328P_CPU*)context;
    
    if (cpu->halted) return 1;
    if (cpu->pc >= FLASH_WORDS) return -3;
    
    uint16_t instr_pc = cpu->pc;
    uint16_t op = cpu->flash[cpu->pc];
    uint16_t next_pc = (cpu->pc + 1) % FLASH_WORDS;
    cpu->ticks++;
    
    // UART intercept for outputs (UDR0 writes)
    // We check writes to address 0xC6 (UDR0). Since the instruction does the write,
    // we let it execute, and we can check if it writes, OR intercept it at the I/O write logic.
    // To do it cleanly, we record the old value of UDR0 or check the instruction executed.
    // Intercepting at the end of instruction step is easiest:
    uint8_t old_udr0 = cpu->data_space[0xC6];
    
    // Decode and Execute
    if ((op & 0xF000) == 0xE000) {
        // LDI Rd, K (1110 KKKK dddd KKKK)
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        cpu->data_space[d] = k;
    }
    else if ((op & 0xFE0F) == 0x9000) {
        // LDS Rd, k (Word 1: 1001 000d dddd 0000, Word 2: k)
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t k = cpu->flash[(instr_pc + 1) % FLASH_WORDS];
        cpu->data_space[d] = (k < SRAM_SIZE) ? cpu->data_space[k] : 0;
        next_pc = (instr_pc + 2) % FLASH_WORDS;
    }
    else if ((op & 0xFE0F) == 0x9200) {
        // STS k, Rr (Word 1: 1001 001r rrrr 0000, Word 2: k)
        uint8_t r = (op >> 4) & 0x1F;
        uint16_t k = cpu->flash[(instr_pc + 1) % FLASH_WORDS];
        if (k < SRAM_SIZE) {
            uint8_t old_portb = cpu->data_space[0x25];
            cpu->data_space[k] = cpu->data_space[r];
            check_led_toggle(cpu, old_portb);
        }
        next_pc = (instr_pc + 2) % FLASH_WORDS;
    }
    else if ((op & 0xF800) == 0xB800) {
        // OUT A, Rr (1011 1AAr rrrr AAAA)
        uint8_t a = ((op >> 5) & 0x30) | (op & 0x0F);
        uint8_t r = (op >> 4) & 0x1F;
        uint8_t old_portb = cpu->data_space[0x25];
        cpu->data_space[a + 0x20] = cpu->data_space[r];
        check_led_toggle(cpu, old_portb);
    }
    else if ((op & 0xF800) == 0xB000) {
        // IN Rd, A (1011 0AAd dddd AAAA)
        uint8_t a = ((op >> 5) & 0x30) | (op & 0x0F);
        uint8_t d = (op >> 4) & 0x1F;
        cpu->data_space[d] = cpu->data_space[a + 0x20];
    }
    else if ((op & 0xFF00) == 0x9A00) {
        // SBI A, b (1001 1010 AAAA Abbb)
        uint8_t a = (op >> 3) & 0x1F;
        uint8_t b = op & 0x07;
        uint8_t old_portb = cpu->data_space[0x25];
        cpu->data_space[a + 0x20] |= (1 << b);
        check_led_toggle(cpu, old_portb);
    }
    else if ((op & 0xFF00) == 0x9800) {
        // CBI A, b (1001 1000 AAAA Abbb)
        uint8_t a = (op >> 3) & 0x1F;
        uint8_t b = op & 0x07;
        uint8_t old_portb = cpu->data_space[0x25];
        cpu->data_space[a + 0x20] &= ~(1 << b);
        check_led_toggle(cpu, old_portb);
    }
    else if ((op & 0xFC00) == 0x0C00) {
        // ADD Rd, Rr (0000 11rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t d_val = cpu->data_space[d];
        uint8_t r_val = cpu->data_space[r];
        uint16_t sum = (uint16_t)d_val + r_val;
        cpu->data_space[d] = sum & 0xFF;
        
        SET_FLAG(0, (sum > 0xFF) ? 1 : 0);
        SET_FLAG(1, ((sum & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((sum & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((d_val & 0x80) && (r_val & 0x80) && !(sum & 0x80)) ||
                     (!(d_val & 0x80) && !(r_val & 0x80) && (sum & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((sum & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x1C00) {
        // ADC Rd, Rr (0001 11rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t d_val = cpu->data_space[d];
        uint8_t r_val = cpu->data_space[r];
        uint8_t cy = GET_FLAG(0);
        uint16_t sum = (uint16_t)d_val + r_val + cy;
        cpu->data_space[d] = sum & 0xFF;
        
        SET_FLAG(0, (sum > 0xFF) ? 1 : 0);
        SET_FLAG(1, ((sum & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((sum & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((d_val & 0x80) && (r_val & 0x80) && !(sum & 0x80)) ||
                     (!(d_val & 0x80) && !(r_val & 0x80) && (sum & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((sum & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x1800) {
        // SUB Rd, Rr (0001 10rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t d_val = cpu->data_space[d];
        uint8_t r_val = cpu->data_space[r];
        uint16_t diff = (uint16_t)d_val - r_val;
        cpu->data_space[d] = diff & 0xFF;
        
        SET_FLAG(0, (d_val < r_val) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((d_val & 0x80) && !(r_val & 0x80) && !(diff & 0x80)) ||
                     (!(d_val & 0x80) && (r_val & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x0800) {
        // SBC Rd, Rr (0000 10rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t d_val = cpu->data_space[d];
        uint8_t r_val = cpu->data_space[r];
        uint8_t cy = GET_FLAG(0);
        uint16_t diff = (uint16_t)d_val - r_val - cy;
        cpu->data_space[d] = diff & 0xFF;
        
        SET_FLAG(0, (d_val < (uint16_t)r_val + cy) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((d_val & 0x80) && !(r_val & 0x80) && !(diff & 0x80)) ||
                     (!(d_val & 0x80) && (r_val & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x1400) {
        // CP Rd, Rr (0001 01rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t d_val = cpu->data_space[d];
        uint8_t r_val = cpu->data_space[r];
        uint16_t diff = (uint16_t)d_val - r_val;
        
        SET_FLAG(0, (d_val < r_val) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((d_val & 0x80) && !(r_val & 0x80) && !(diff & 0x80)) ||
                     (!(d_val & 0x80) && (r_val & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x0400) {
        // CPC Rd, Rr (0000 01rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t d_val = cpu->data_space[d];
        uint8_t r_val = cpu->data_space[r];
        uint8_t cy = GET_FLAG(0);
        uint16_t diff = (uint16_t)d_val - r_val - cy;
        
        SET_FLAG(0, (d_val < (uint16_t)r_val + cy) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((d_val & 0x80) && !(r_val & 0x80) && !(diff & 0x80)) ||
                     (!(d_val & 0x80) && (r_val & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x2400) {
        // EOR Rd, Rr (0010 01rd dddd rrrr) -> XOR
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t res = cpu->data_space[d] ^ cpu->data_space[r];
        cpu->data_space[d] = res;
        
        SET_FLAG(0, 0);
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        SET_FLAG(3, 0);
        SET_FLAG(4, (res & 0x80) ? 1 : 0);
    }
    else if ((op & 0xFC00) == 0x2000) {
        // AND Rd, Rr (0010 00rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t res = cpu->data_space[d] & cpu->data_space[r];
        cpu->data_space[d] = res;
        
        SET_FLAG(0, 0);
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        SET_FLAG(3, 0);
        SET_FLAG(4, (res & 0x80) ? 1 : 0);
    }
    else if ((op & 0xFC00) == 0x2A00) {
        // OR Rd, Rr (0010 10rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint8_t res = cpu->data_space[d] | cpu->data_space[r];
        cpu->data_space[d] = res;
        
        SET_FLAG(0, 0);
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        SET_FLAG(3, 0);
        SET_FLAG(4, (res & 0x80) ? 1 : 0);
    }
    else if ((op & 0xFC00) == 0x2C00) {
        // MOV Rd, Rr (0010 11rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        cpu->data_space[d] = cpu->data_space[r];
    }
    else if ((op & 0xF000) == 0x7000) {
        // ANDI Rd, K (0111 KKKK dddd KKKK)
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        uint8_t res = cpu->data_space[d] & k;
        cpu->data_space[d] = res;
        
        SET_FLAG(0, 0);
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        SET_FLAG(3, 0);
        SET_FLAG(4, (res & 0x80) ? 1 : 0);
    }
    else if ((op & 0xF000) == 0x6000) {
        // ORI Rd, K (0110 KKKK dddd KKKK)
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        uint8_t res = cpu->data_space[d] | k;
        cpu->data_space[d] = res;
        
        SET_FLAG(0, 0);
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        SET_FLAG(3, 0);
        SET_FLAG(4, (res & 0x80) ? 1 : 0);
    }
    else if ((op & 0xF000) == 0x5000) {
        // SUBI Rd, K (0101 KKKK dddd KKKK)
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        uint8_t val = cpu->data_space[d];
        uint16_t diff = (uint16_t)val - k;
        cpu->data_space[d] = diff & 0xFF;
        
        SET_FLAG(0, (val < k) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((val & 0x80) && !(k & 0x80) && !(diff & 0x80)) ||
                     (!(val & 0x80) && (k & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xF000) == 0x4000) {
        // SBCI Rd, K (0100 KKKK dddd KKKK)
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        uint8_t val = cpu->data_space[d];
        uint8_t cy = GET_FLAG(0);
        uint16_t diff = (uint16_t)val - k - cy;
        cpu->data_space[d] = diff & 0xFF;
        
        SET_FLAG(0, (val < (uint16_t)k + cy) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((val & 0x80) && !(k & 0x80) && !(diff & 0x80)) ||
                     (!(val & 0x80) && (k & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xF000) == 0x3000) {
        // CPI Rd, K (0011 KKKK dddd KKKK)
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        uint8_t val = cpu->data_space[d];
        uint16_t diff = (uint16_t)val - k;
        
        SET_FLAG(0, (val < k) ? 1 : 0);
        SET_FLAG(1, ((diff & 0xFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((diff & 0x80) != 0) ? 1 : 0);
        uint8_t v = (((val & 0x80) && !(k & 0x80) && !(diff & 0x80)) ||
                     (!(val & 0x80) && (k & 0x80) && (diff & 0x80))) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((diff & 0x80) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFF00) == 0x9600) {
        // ADIW Rd, K (1001 0110 KKdd KKKK)
        uint8_t d_reg = 24 + ((op >> 4) & 0x03) * 2;
        uint8_t k = (op & 0x0F) | ((op >> 2) & 0x30);
        uint16_t word_val = ((uint16_t)cpu->data_space[d_reg + 1] << 8) | cpu->data_space[d_reg];
        uint32_t result = (uint32_t)word_val + k;
        cpu->data_space[d_reg] = result & 0xFF;
        cpu->data_space[d_reg + 1] = (result >> 8) & 0xFF;
        
        SET_FLAG(0, (result > 0xFFFF) ? 1 : 0);
        SET_FLAG(1, ((result & 0xFFFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((result & 0x8000) != 0) ? 1 : 0);
        uint8_t v = (!(word_val & 0x8000) && (result & 0x8000)) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((result & 0x8000) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFF00) == 0x9700) {
        // SBIW Rd, K (1001 0111 KKdd KKKK)
        uint8_t d_reg = 24 + ((op >> 4) & 0x03) * 2;
        uint8_t k = (op & 0x0F) | ((op >> 2) & 0x30);
        uint16_t word_val = ((uint16_t)cpu->data_space[d_reg + 1] << 8) | cpu->data_space[d_reg];
        uint32_t result = (uint32_t)word_val - k;
        cpu->data_space[d_reg] = result & 0xFF;
        cpu->data_space[d_reg + 1] = (result >> 8) & 0xFF;
        
        SET_FLAG(0, (word_val < k) ? 1 : 0);
        SET_FLAG(1, ((result & 0xFFFF) == 0) ? 1 : 0);
        SET_FLAG(2, ((result & 0x8000) != 0) ? 1 : 0);
        uint8_t v = ((word_val & 0x8000) && !(result & 0x8000)) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, (((result & 0x8000) != 0) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFC00) == 0x0200) {
        // MUL Rd, Rr (0000 0010 rrrr dddd)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        uint16_t product = (uint16_t)cpu->data_space[d] * cpu->data_space[r];
        cpu->data_space[0] = product & 0xFF;
        cpu->data_space[1] = (product >> 8) & 0xFF;
        SET_FLAG(0, (product & 0x8000) ? 1 : 0);
        SET_FLAG(1, (product == 0) ? 1 : 0);
    }
    else if ((op & 0xFE0F) == 0x920F) {
        // PUSH Rr (1001 001r rrrr 1111)
        uint8_t r = (op >> 4) & 0x1F;
        uint16_t sp = ((uint16_t)cpu->data_space[0x5E] << 8) | cpu->data_space[0x5D];
        if (sp < SRAM_SIZE) cpu->data_space[sp] = cpu->data_space[r];
        sp--;
        cpu->data_space[0x5E] = (sp >> 8) & 0xFF;
        cpu->data_space[0x5D] = sp & 0xFF;
    }
    else if ((op & 0xFE0F) == 0x900F) {
        // POP Rd (1001 000d dddd 1111)
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t sp = ((uint16_t)cpu->data_space[0x5E] << 8) | cpu->data_space[0x5D];
        sp++;
        if (sp < SRAM_SIZE) cpu->data_space[d] = cpu->data_space[sp];
        cpu->data_space[0x5E] = (sp >> 8) & 0xFF;
        cpu->data_space[0x5D] = sp & 0xFF;
    }
    else if ((op & 0xD000) == 0x8000) {
        // LDD / STD with displacement: Y or Z pointer
        // LDD Rd, Y/Z+q (10q0 q00d dddd Yqqq)
        // STD Y/Z+q, Rr (10q0 q01r rrrr Yqqq)
        uint8_t reg = (op >> 4) & 0x1F;
        uint8_t is_store = (op >> 9) & 1;
        uint8_t is_z = (op >> 3) & 1;
        uint8_t q = (op & 0x07) | ((op >> 7) & 0x18) | ((op >> 8) & 0x20);
        
        uint8_t base_reg = is_z ? 30 : 28;
        uint16_t addr = ((uint16_t)cpu->data_space[base_reg + 1] << 8) | cpu->data_space[base_reg];
        uint16_t target_addr = addr + q;
        
        if (is_store) {
            if (target_addr < SRAM_SIZE) {
                uint8_t old_portb = cpu->data_space[0x25];
                cpu->data_space[target_addr] = cpu->data_space[reg];
                check_led_toggle(cpu, old_portb);
            }
        } else {
            cpu->data_space[reg] = (target_addr < SRAM_SIZE) ? cpu->data_space[target_addr] : 0;
        }
    }
    else if ((op & 0xFE0F) == 0x9001) {
        // LD Rd, Z+ (1001 000d dddd 0001)
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t addr = ((uint16_t)cpu->data_space[31] << 8) | cpu->data_space[30];
        cpu->data_space[d] = (addr < SRAM_SIZE) ? cpu->data_space[addr] : 0;
        addr++;
        cpu->data_space[31] = (addr >> 8) & 0xFF;
        cpu->data_space[30] = addr & 0xFF;
    }
    else if ((op & 0xFE0F) == 0x9002) {
        // LD Rd, -Z (1001 000d dddd 0010)
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t addr = ((uint16_t)cpu->data_space[31] << 8) | cpu->data_space[30];
        addr--;
        cpu->data_space[31] = (addr >> 8) & 0xFF;
        cpu->data_space[30] = addr & 0xFF;
        cpu->data_space[d] = (addr < SRAM_SIZE) ? cpu->data_space[addr] : 0;
    }
    else if ((op & 0xFE0F) == 0x9201) {
        // ST Z+, Rr (1001 001r rrrr 0001)
        uint8_t r = (op >> 4) & 0x1F;
        uint16_t addr = ((uint16_t)cpu->data_space[31] << 8) | cpu->data_space[30];
        if (addr < SRAM_SIZE) {
            uint8_t old_portb = cpu->data_space[0x25];
            cpu->data_space[addr] = cpu->data_space[r];
            check_led_toggle(cpu, old_portb);
        }
        addr++;
        cpu->data_space[31] = (addr >> 8) & 0xFF;
        cpu->data_space[30] = addr & 0xFF;
    }
    else if ((op & 0xFE0F) == 0x9202) {
        // ST -Z, Rr (1001 001r rrrr 0010)
        uint8_t r = (op >> 4) & 0x1F;
        uint16_t addr = ((uint16_t)cpu->data_space[31] << 8) | cpu->data_space[30];
        addr--;
        cpu->data_space[31] = (addr >> 8) & 0xFF;
        cpu->data_space[30] = addr & 0xFF;
        if (addr < SRAM_SIZE) {
            uint8_t old_portb = cpu->data_space[0x25];
            cpu->data_space[addr] = cpu->data_space[r];
            check_led_toggle(cpu, old_portb);
        }
    }
    else if ((op & 0xFE0E) == 0x940E) {
        // CALL k (32-bit: Word 1: 1001 010k kkkk 111k, Word 2: k)
        uint16_t k = cpu->flash[(instr_pc + 1) % FLASH_WORDS];
        push_stack(cpu, (instr_pc + 2) % FLASH_WORDS);
        next_pc = k % FLASH_WORDS;
    }
    else if ((op & 0xFE0E) == 0x940C) {
        // JMP k (32-bit: Word 1: 1001 010k kkkk 110k, Word 2: k)
        uint16_t k = cpu->flash[(instr_pc + 1) % FLASH_WORDS];
        next_pc = k % FLASH_WORDS;
    }
    else if ((op & 0xFC00) == 0x1000) {
        // CPSE Rd, Rr (Compare, Skip if Equal: 0001 00rd dddd rrrr)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        if (cpu->data_space[d] == cpu->data_space[r]) {
            uint16_t next_op = cpu->flash[next_pc];
            int is_2word = ((next_op & 0xFE0E) == 0x940C) || ((next_op & 0xFE0E) == 0x940E) ||
                           ((next_op & 0xFE0F) == 0x9000) || ((next_op & 0xFE0F) == 0x9200);
            next_pc = (next_pc + (is_2word ? 2 : 1)) % FLASH_WORDS;
        }
    }
    else if ((op & 0xFE0F) == 0x9403) {
        // INC Rd (1001 010d dddd 0011)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t res = cpu->data_space[d] + 1;
        cpu->data_space[d] = res;
        
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        uint8_t v = (res == 0x80) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, ((res & 0x80) ? 1 : 0) ^ v);
    }
    else if ((op & 0xFE0F) == 0x940A) {
        // DEC Rd (1001 010d dddd 1010)
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t res = cpu->data_space[d] - 1;
        cpu->data_space[d] = res;
        
        SET_FLAG(1, (res == 0) ? 1 : 0);
        SET_FLAG(2, (res & 0x80) ? 1 : 0);
        uint8_t v = (res == 0x7F) ? 1 : 0;
        SET_FLAG(3, v);
        SET_FLAG(4, ((res & 0x80) ? 1 : 0) ^ v);
    }
    else if ((op & 0xF000) == 0xC000) {
        // RJMP k (1100 kkkk kkkk kkkk)
        int16_t k = op & 0x0FFF;
        if (k & 0x0800) k |= 0xF000;
        next_pc = (instr_pc + 1 + k) % FLASH_WORDS;
    }
    else if ((op & 0xF000) == 0xD000) {
        // RCALL k (1101 kkkk kkkk kkkk)
        int16_t k = op & 0x0FFF;
        if (k & 0x0800) k |= 0xF000;
        push_stack(cpu, (instr_pc + 1) % FLASH_WORDS);
        next_pc = (instr_pc + 1 + k) % FLASH_WORDS;
    }
    else if (op == 0x9508 || op == 0x9518) {
        // RET (0x9508) or RETI (0x9518)
        next_pc = pop_stack(cpu);
        if (op == 0x9518) {
            SET_FLAG(7, 1);
        }
    }
    else if ((op & 0xFC00) == 0xF000) {
        // BRBS s, k (1111 00kk kkkk ksss)
        uint8_t s = op & 0x07;
        int8_t k = (op >> 3) & 0x7F;
        if (k & 0x40) k |= 0x80;
        if (GET_FLAG(s)) {
            next_pc = (instr_pc + 1 + k) % FLASH_WORDS;
        }
    }
    else if ((op & 0xFC00) == 0xF400) {
        // BRBC s, k (1111 01kk kkkk ksss)
        uint8_t s = op & 0x07;
        int8_t k = (op >> 3) & 0x7F;
        if (k & 0x40) k |= 0x80;
        if (!GET_FLAG(s)) {
            next_pc = (instr_pc + 1 + k) % FLASH_WORDS;
        }
    }
    else if ((op & 0xFE08) == 0xFC08) {
        // SBRS Rd, b (1111 110d dddd 0bbb) -> Skip if Bit in Register Set
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t b = op & 0x07;
        if ((cpu->data_space[d] >> b) & 1) {
            uint16_t next_op = cpu->flash[next_pc];
            int is_2word = ((next_op & 0xFE0E) == 0x940C) || ((next_op & 0xFE0E) == 0x940E) ||
                           ((next_op & 0xFE0F) == 0x9000) || ((next_op & 0xFE0F) == 0x9200);
            next_pc = (next_pc + (is_2word ? 2 : 1)) % FLASH_WORDS;
        }
    }
    else if ((op & 0xFE08) == 0xFC00) {
        // SBRC Rd, b (1111 110d dddd 1bbb) -> Skip if Bit in Register Cleared
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t b = op & 0x07;
        if (!((cpu->data_space[d] >> b) & 1)) {
            uint16_t next_op = cpu->flash[next_pc];
            int is_2word = ((next_op & 0xFE0E) == 0x940C) || ((next_op & 0xFE0E) == 0x940E) ||
                           ((next_op & 0xFE0F) == 0x9000) || ((next_op & 0xFE0F) == 0x9200);
            next_pc = (next_pc + (is_2word ? 2 : 1)) % FLASH_WORDS;
        }
    }
    else if ((op & 0xFE0F) == 0x9004) {
        // LPM Rd, Z (1001 000d dddd 0100)
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t z = ((uint16_t)cpu->data_space[31] << 8) | cpu->data_space[30]; // R31:R30
        uint16_t w_addr = z >> 1;
        uint16_t w_val = cpu->flash[w_addr % FLASH_WORDS];
        cpu->data_space[d] = (z & 1) ? (w_val >> 8) : (w_val & 0xFF);
    }
    else if ((op & 0xFE0F) == 0x900C) {
        // LD Rd, X (1001 000d dddd 1100)
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t addr = ((uint16_t)cpu->data_space[27] << 8) | cpu->data_space[26];
        cpu->data_space[d] = (addr < SRAM_SIZE) ? cpu->data_space[addr] : 0;
    }
    else if ((op & 0xFE0F) == 0x920C) {
        // ST X, Rr (1001 001r rrrr 1100)
        uint8_t r = (op >> 4) & 0x1F;
        uint16_t addr = ((uint16_t)cpu->data_space[27] << 8) | cpu->data_space[26];
        if (addr < SRAM_SIZE) {
            uint8_t old_portb = cpu->data_space[0x25];
            cpu->data_space[addr] = cpu->data_space[r];
            check_led_toggle(cpu, old_portb);
        }
    }
    
    cpu->pc = next_pc;
    
    // UART Output Capture (detecting write changes to UDR0 at 0xC6)
    if (cpu->data_space[0xC6] != old_udr0) {
        putchar(cpu->data_space[0xC6]);
        fflush(stdout);
        // Reset/keep UDRE0 set in UCSR0A (0xC0) so the software knows it is ready to write
        cpu->data_space[0xC0] = 0x20;
    }
    
    // Hardware Timer 0 Tick
    cpu->timer_tick_counter++;
    if (cpu->timer_tick_counter >= 64) { // prescaler of 64
        cpu->timer_tick_counter = 0;
        uint8_t tcnt0 = cpu->data_space[0x46];
        tcnt0++;
        cpu->data_space[0x46] = tcnt0;
        if (tcnt0 == 0) { // Overflow occurred!
            // Set TOV0 (bit 0 of TIFR0 at 0x35)
            cpu->data_space[0x35] |= 0x01;
        }
    }
    
    // AVR Interrupt Execution
    // Check if global interrupts are enabled (I-flag, bit 7 of SREG)
    if (GET_FLAG(7)) {
        // Check Timer0 Overflow interrupt enabled (TOIE0, bit 0 of TIMSK0 at 0x6E)
        // and Timer0 Overflow flag is set (TOV0, bit 0 of TIFR0 at 0x35)
        if ((cpu->data_space[0x6E] & 0x01) && (cpu->data_space[0x35] & 0x01)) {
            // Push current PC onto stack
            push_stack(cpu, cpu->pc);
            
            // Clear I-flag in SREG to disable nested interrupts
            SET_FLAG(7, 0);
            
            // Clear TOV0 flag in TIFR0
            cpu->data_space[0x35] &= ~0x01;
            
            // Jump to TIMER0_OVF interrupt vector at word address 0x0020
            cpu->pc = 0x0020;
        }
    }
    
    // Check if PC self-looped (interpreted as hardware spin/halt)
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }
    
    return 0;
}

void atmega328p_print_state(void *context) {
    if (!context) return;
    ATmega328P_CPU *cpu = (ATmega328P_CPU*)context;
    
    printf("ATmega328P State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  Halted: %s\n", cpu->pc, cpu->halted ? "Yes" : "No");
    printf("  SREG: Carry=%d, Zero=%d, Neg=%d, Ovf=%d, Sign=%d, Interrupts=%d\n",
           GET_FLAG(0), GET_FLAG(1), GET_FLAG(2), GET_FLAG(3), GET_FLAG(4), GET_FLAG(7));
    
    uint16_t sp = ((uint16_t)cpu->data_space[0x5E] << 8) | cpu->data_space[0x5D];
    printf("  SP: 0x%04X\n", sp);
    
    printf("  Registers:\n");
    for (int i = 0; i < 32; ++i) {
        printf("    R%02d: 0x%02X%s", i, cpu->data_space[i], (i % 8 == 7) ? "\n" : "  ");
    }
    
    printf("  Port B Registers: DDRB=0x%02X  PORTB=0x%02X\n",
           cpu->data_space[0x24], cpu->data_space[0x25]);
    printf("  Timer0 Registers: TCNT0=0x%02X  TIMSK0=0x%02X  TIFR0=0x%02X\n",
           cpu->data_space[0x46], cpu->data_space[0x6E], cpu->data_space[0x35]);
}

void atmega328p_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    ATmega328P_CPU *cpu = (ATmega328P_CPU*)context;
    
    if (cpu->pc >= FLASH_WORDS) {
        snprintf(buf, buf_len, "<out of Flash>");
        return;
    }
    
    uint16_t op = cpu->flash[cpu->pc];
    
    if ((op & 0xF000) == 0xE000) {
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        snprintf(buf, buf_len, "ldi   R%d, 0x%02X", d, k);
    }
    else if ((op & 0xFE0F) == 0x9000) {
        uint8_t d = (op >> 4) & 0x1F;
        uint16_t k = cpu->flash[(cpu->pc + 1) % FLASH_WORDS];
        snprintf(buf, buf_len, "lds   R%d, 0x%04X", d, k);
    }
    else if ((op & 0xFE0F) == 0x9200) {
        uint8_t r = (op >> 4) & 0x1F;
        uint16_t k = cpu->flash[(cpu->pc + 1) % FLASH_WORDS];
        snprintf(buf, buf_len, "sts   0x%04X, R%d", k, r);
    }
    else if ((op & 0xF800) == 0xB800) {
        uint8_t a = ((op >> 5) & 0x30) | (op & 0x0F);
        uint8_t r = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "out   0x%02X, R%d", a, r);
    }
    else if ((op & 0xF800) == 0xB000) {
        uint8_t a = ((op >> 5) & 0x30) | (op & 0x0F);
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "in    R%d, 0x%02X", d, a);
    }
    else if ((op & 0xFF00) == 0x9A00) {
        uint8_t a = (op >> 3) & 0x1F;
        uint8_t b = op & 0x07;
        snprintf(buf, buf_len, "sbi   0x%02X, %d", a, b);
    }
    else if ((op & 0xFF00) == 0x9800) {
        uint8_t a = (op >> 3) & 0x1F;
        uint8_t b = op & 0x07;
        snprintf(buf, buf_len, "cbi   0x%02X, %d", a, b);
    }
    else if ((op & 0xFC00) == 0x0C00) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "add   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x1C00) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "adc   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x1800) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "sub   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x0800) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "sbc   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x1400) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "cp    R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x0400) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "cpc   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x2400) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "eor   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x2000) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "and   R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x2A00) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "or    R%d, R%d", d, r);
    }
    else if ((op & 0xFC00) == 0x2C00) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "mov   R%d, R%d", d, r);
    }
    else if ((op & 0xF000) == 0x7000) {
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        snprintf(buf, buf_len, "andi  R%d, 0x%02X", d, k);
    }
    else if ((op & 0xF000) == 0x6000) {
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        snprintf(buf, buf_len, "ori   R%d, 0x%02X", d, k);
    }
    else if ((op & 0xF000) == 0x5000) {
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        snprintf(buf, buf_len, "subi  R%d, 0x%02X", d, k);
    }
    else if ((op & 0xF000) == 0x4000) {
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        snprintf(buf, buf_len, "sbci  R%d, 0x%02X", d, k);
    }
    else if ((op & 0xF000) == 0x3000) {
        uint8_t d = 16 + ((op >> 4) & 0x0F);
        uint8_t k = (op & 0x0F) | ((op >> 4) & 0xF0);
        snprintf(buf, buf_len, "cpi   R%d, 0x%02X", d, k);
    }
    else if ((op & 0xFF00) == 0x9600) {
        uint8_t d_reg = 24 + ((op >> 4) & 0x03) * 2;
        uint8_t k = (op & 0x0F) | ((op >> 2) & 0x30);
        snprintf(buf, buf_len, "adiw  R%d, %d", d_reg, k);
    }
    else if ((op & 0xFF00) == 0x9700) {
        uint8_t d_reg = 24 + ((op >> 4) & 0x03) * 2;
        uint8_t k = (op & 0x0F) | ((op >> 2) & 0x30);
        snprintf(buf, buf_len, "sbiw  R%d, %d", d_reg, k);
    }
    else if ((op & 0xFC00) == 0x0200) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "mul   R%d, R%d", d, r);
    }
    else if ((op & 0xFE0F) == 0x920F) {
        uint8_t r = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "push  R%d", r);
    }
    else if ((op & 0xFE0F) == 0x900F) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "pop   R%d", d);
    }
    else if ((op & 0xD000) == 0x8000) {
        uint8_t reg = (op >> 4) & 0x1F;
        uint8_t is_store = (op >> 9) & 1;
        uint8_t is_z = (op >> 3) & 1;
        uint8_t q = (op & 0x07) | ((op >> 7) & 0x18) | ((op >> 8) & 0x20);
        if (is_store) {
            snprintf(buf, buf_len, "std   %c+%d, R%d", is_z ? 'Z' : 'Y', q, reg);
        } else {
            snprintf(buf, buf_len, "ldd   R%d, %c+%d", reg, is_z ? 'Z' : 'Y', q);
        }
    }
    else if ((op & 0xFE0F) == 0x9001) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "ld    R%d, Z+", d);
    }
    else if ((op & 0xFE0F) == 0x9002) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "ld    R%d, -Z", d);
    }
    else if ((op & 0xFE0F) == 0x9201) {
        uint8_t r = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "st    Z+, R%d", r);
    }
    else if ((op & 0xFE0F) == 0x9202) {
        uint8_t r = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "st    -Z, R%d", r);
    }
    else if ((op & 0xFE0E) == 0x940E) {
        uint16_t k = cpu->flash[(cpu->pc + 1) % FLASH_WORDS];
        snprintf(buf, buf_len, "call  0x%04X", k);
    }
    else if ((op & 0xFE0E) == 0x940C) {
        uint16_t k = cpu->flash[(cpu->pc + 1) % FLASH_WORDS];
        snprintf(buf, buf_len, "jmp   0x%04X", k);
    }
    else if ((op & 0xFC00) == 0x1000) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t r = (op & 0x0F) | ((op >> 5) & 0x10);
        snprintf(buf, buf_len, "cpse  R%d, R%d", d, r);
    }
    else if ((op & 0xFE0F) == 0x9403) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "inc   R%d", d);
    }
    else if ((op & 0xFE0F) == 0x940A) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "dec   R%d", d);
    }
    else if ((op & 0xF000) == 0xC000) {
        int16_t k = op & 0x0FFF;
        if (k & 0x0800) k |= 0xF000;
        snprintf(buf, buf_len, "rjmp  pc%+d (0x%04X)", k + 1, (cpu->pc + 1 + k) % FLASH_WORDS);
    }
    else if ((op & 0xF000) == 0xD000) {
        int16_t k = op & 0x0FFF;
        if (k & 0x0800) k |= 0xF000;
        snprintf(buf, buf_len, "rcall pc%+d (0x%04X)", k + 1, (cpu->pc + 1 + k) % FLASH_WORDS);
    }
    else if (op == 0x9508) {
        snprintf(buf, buf_len, "ret");
    }
    else if (op == 0x9518) {
        snprintf(buf, buf_len, "reti");
    }
    else if ((op & 0xFC00) == 0xF000) {
        uint8_t s = op & 0x07;
        int8_t k = (op >> 3) & 0x7F;
        if (k & 0x40) k |= 0x80;
        snprintf(buf, buf_len, "%s  pc%+d (0x%04X)", br_names_set[s], k + 1, (cpu->pc + 1 + k) % FLASH_WORDS);
    }
    else if ((op & 0xFC00) == 0xF400) {
        uint8_t s = op & 0x07;
        int8_t k = (op >> 3) & 0x7F;
        if (k & 0x40) k |= 0x80;
        snprintf(buf, buf_len, "%s  pc%+d (0x%04X)", br_names_clr[s], k + 1, (cpu->pc + 1 + k) % FLASH_WORDS);
    }
    else if ((op & 0xFE08) == 0xFC08) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t b = op & 0x07;
        snprintf(buf, buf_len, "sbrs  R%d, %d", d, b);
    }
    else if ((op & 0xFE08) == 0xFC00) {
        uint8_t d = (op >> 4) & 0x1F;
        uint8_t b = op & 0x07;
        snprintf(buf, buf_len, "sbrc  R%d, %d", d, b);
    }
    else if ((op & 0xFE0F) == 0x9004) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "lpm   R%d, Z", d);
    }
    else if ((op & 0xFE0F) == 0x900C) {
        uint8_t d = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "ld    R%d, X", d);
    }
    else if ((op & 0xFE0F) == 0x920C) {
        uint8_t r = (op >> 4) & 0x1F;
        snprintf(buf, buf_len, "st    X, R%d", r);
    }
    else {
        snprintf(buf, buf_len, "unknown (0x%04X)", op);
    }
}
