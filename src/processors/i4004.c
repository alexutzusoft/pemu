#include "i4004.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct I4004CPU {
    uint16_t pc;
    uint16_t stack[3];
    uint8_t acc;
    uint8_t carry;
    uint8_t test_signal;
    uint8_t r[16];
    
    // Selection state
    uint8_t current_bank;
    uint8_t selected_chip;
    uint8_t selected_register;
    uint8_t selected_char;
    
    // Memory
    uint8_t rom[4096];
    uint8_t ram_data[8][4][4][16]; // 8 banks, 4 chips, 4 registers, 16 characters
    uint8_t ram_status[8][4][4][4]; // 8 banks, 4 chips, 4 registers, 4 status characters
    uint8_t ram_ports[8][4];        // RAM output ports
    uint8_t rom_ports[16];          // ROM ports
    
    uint32_t ticks;
} I4004CPU;

void* i4004_create(void) {
    I4004CPU *cpu = (I4004CPU*)calloc(1, sizeof(I4004CPU));
    if (cpu) {
        cpu->test_signal = 1; // Default to high/inactive
    }
    return cpu;
}

void i4004_destroy(void *context) {
    free(context);
}

int i4004_init(void *context) {
    if (!context) return -1;
    I4004CPU *cpu = (I4004CPU*)context;
    
    cpu->pc = 0;
    memset(cpu->stack, 0, sizeof(cpu->stack));
    cpu->acc = 0;
    cpu->carry = 0;
    cpu->test_signal = 1;
    memset(cpu->r, 0, sizeof(cpu->r));
    
    cpu->current_bank = 0;
    cpu->selected_chip = 0;
    cpu->selected_register = 0;
    cpu->selected_char = 0;
    
    memset(cpu->rom, 0, sizeof(cpu->rom));
    memset(cpu->ram_data, 0, sizeof(cpu->ram_data));
    memset(cpu->ram_status, 0, sizeof(cpu->ram_status));
    memset(cpu->ram_ports, 0, sizeof(cpu->ram_ports));
    memset(cpu->rom_ports, 0, sizeof(cpu->rom_ports));
    
    cpu->ticks = 0;
    return 0;
}

int i4004_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    I4004CPU *cpu = (I4004CPU*)context;
    if (address >= sizeof(cpu->rom)) return -2;
    
    size_t copy_len = size;
    if (address + size > sizeof(cpu->rom)) {
        copy_len = sizeof(cpu->rom) - address;
    }
    memcpy(&cpu->rom[address], data, copy_len);
    return 0;
}

int i4004_step(void *context) {
    if (!context) return -1;
    I4004CPU *cpu = (I4004CPU*)context;
    
    if (cpu->pc >= 4096) {
        return -3; // PC out of ROM bounds
    }
    
    uint16_t instr_pc = cpu->pc; // Save start address to check for infinite loop
    uint8_t op = cpu->rom[cpu->pc];
    uint8_t is_2byte = 0;
    uint8_t data_byte = 0;
    
    // Determine if instruction is 2 bytes
    if ((op >= 0x10 && op <= 0x1F) ||
        (op >= 0x20 && op <= 0x2E && (op & 1) == 0) ||
        (op >= 0x40 && op <= 0x4F) ||
        (op >= 0x50 && op <= 0x5F) ||
        (op >= 0x70 && op <= 0x7F)) {
        is_2byte = 1;
        if (cpu->pc + 1 >= 4096) {
            return -4; // Second byte out of bounds
        }
        data_byte = cpu->rom[cpu->pc + 1];
    }
    
    // Advance PC
    cpu->pc = (cpu->pc + (is_2byte ? 2 : 1)) & 0xFFF;
    cpu->ticks += is_2byte ? 2 : 1; // 2-byte instructions take 2 cycles, 1-byte take 1 cycle
    
    // Execute
    if (op == 0x00) {
        // NOP
    } 
    else if (op >= 0x10 && op <= 0x1F) {
        // JCN
        int cond = 0;
        if ((op & 4) && (cpu->acc == 0)) cond = 1;
        if ((op & 2) && (cpu->carry == 1)) cond = 1;
        if ((op & 1) && (cpu->test_signal == 0)) cond = 1;
        if (op & 8) cond = !cond;
        
        if (cond) {
            cpu->pc = (instr_pc & 0xF00) | data_byte;
        }
    }
    else if (op >= 0x20 && op <= 0x2E && (op & 1) == 0) {
        // FIM
        uint8_t pair = (op >> 1) & 7;
        cpu->r[pair * 2] = (data_byte >> 4) & 0x0F;
        cpu->r[pair * 2 + 1] = data_byte & 0x0F;
    }
    else if (op >= 0x21 && op <= 0x2F && (op & 1) != 0) {
        // SRC
        uint8_t pair = (op >> 1) & 7;
        uint8_t val = (cpu->r[pair * 2] << 4) | cpu->r[pair * 2 + 1];
        cpu->selected_chip = (val >> 6) & 3;
        cpu->selected_register = (val >> 4) & 3;
        cpu->selected_char = val & 0x0F;
    }
    else if (op >= 0x30 && op <= 0x3E && (op & 1) == 0) {
        // FIN
        uint8_t pair = (op >> 1) & 7;
        uint16_t fetch_addr = (instr_pc & 0xF00) | ((cpu->r[0] << 4) | cpu->r[1]);
        if (fetch_addr < 4096) {
            uint8_t val = cpu->rom[fetch_addr];
            cpu->r[pair * 2] = (val >> 4) & 0x0F;
            cpu->r[pair * 2 + 1] = val & 0x0F;
        }
    }
    else if (op >= 0x31 && op <= 0x3F && (op & 1) != 0) {
        // JIN
        uint8_t pair = (op >> 1) & 7;
        cpu->pc = (instr_pc & 0xF00) | ((cpu->r[pair * 2] << 4) | cpu->r[pair * 2 + 1]);
    }
    else if (op >= 0x40 && op <= 0x4F) {
        // JUN
        cpu->pc = ((op & 0x0F) << 8) | data_byte;
    }
    else if (op >= 0x50 && op <= 0x5F) {
        // JMS
        // Push next PC to stack
        cpu->stack[2] = cpu->stack[1];
        cpu->stack[1] = cpu->stack[0];
        cpu->stack[0] = cpu->pc;
        
        cpu->pc = ((op & 0x0F) << 8) | data_byte;
    }
    else if (op >= 0x60 && op <= 0x6F) {
        // INC
        uint8_t reg = op & 0x0F;
        cpu->r[reg] = (cpu->r[reg] + 1) & 0x0F;
    }
    else if (op >= 0x70 && op <= 0x7F) {
        // ISZ
        uint8_t reg = op & 0x0F;
        cpu->r[reg] = (cpu->r[reg] + 1) & 0x0F;
        if (cpu->r[reg] != 0) {
            cpu->pc = (instr_pc & 0xF00) | data_byte;
        }
    }
    else if (op >= 0x80 && op <= 0x8F) {
        // ADD
        uint8_t reg = op & 0x0F;
        uint8_t sum = cpu->acc + cpu->r[reg] + cpu->carry;
        cpu->acc = sum & 0x0F;
        cpu->carry = (sum > 15) ? 1 : 0;
    }
    else if (op >= 0x90 && op <= 0x9F) {
        // SUB
        uint8_t reg = op & 0x0F;
        uint8_t sum = cpu->acc + (~cpu->r[reg] & 0x0F) + cpu->carry;
        cpu->acc = sum & 0x0F;
        cpu->carry = (sum > 15) ? 1 : 0;
    }
    else if (op >= 0xA0 && op <= 0xAF) {
        // LD
        cpu->acc = cpu->r[op & 0x0F];
    }
    else if (op >= 0xB0 && op <= 0xBF) {
        // XCH
        uint8_t reg = op & 0x0F;
        uint8_t tmp = cpu->acc;
        cpu->acc = cpu->r[reg];
        cpu->r[reg] = tmp;
    }
    else if (op >= 0xC0 && op <= 0xCF) {
        // BBL
        cpu->acc = op & 0x0F;
        // Pop stack
        cpu->pc = cpu->stack[0];
        cpu->stack[0] = cpu->stack[1];
        cpu->stack[1] = cpu->stack[2];
        cpu->stack[2] = 0;
    }
    else if (op >= 0xD0 && op <= 0xDF) {
        // LDM
        cpu->acc = op & 0x0F;
    }
    else if (op >= 0xE0 && op <= 0xEF) {
        // RAM/ROM I/O
        switch (op) {
            case 0xE0: // WRM
                cpu->ram_data[cpu->current_bank][cpu->selected_chip][cpu->selected_register][cpu->selected_char] = cpu->acc;
                break;
            case 0xE1: // WMP
                cpu->ram_ports[cpu->current_bank][cpu->selected_chip] = cpu->acc;
                break;
            case 0xE4: // WR0
            case 0xE5:
            case 0xE6:
            case 0xE7:
                cpu->ram_status[cpu->current_bank][cpu->selected_chip][cpu->selected_register][op - 0xE4] = cpu->acc;
                break;
            case 0xE8: { // SBM
                uint8_t ram_val = cpu->ram_data[cpu->current_bank][cpu->selected_chip][cpu->selected_register][cpu->selected_char];
                uint8_t sum = cpu->acc + (~ram_val & 0x0F) + cpu->carry;
                cpu->acc = sum & 0x0F;
                cpu->carry = (sum > 15) ? 1 : 0;
                break;
            }
            case 0xE9: // RDM
                cpu->acc = cpu->ram_data[cpu->current_bank][cpu->selected_chip][cpu->selected_register][cpu->selected_char];
                break;
            case 0xEA: // RDR
                cpu->acc = cpu->rom_ports[cpu->selected_chip] & 0x0F;
                break;
            case 0xEB: { // ADM
                uint8_t ram_val = cpu->ram_data[cpu->current_bank][cpu->selected_chip][cpu->selected_register][cpu->selected_char];
                uint8_t sum = cpu->acc + ram_val + cpu->carry;
                cpu->acc = sum & 0x0F;
                cpu->carry = (sum > 15) ? 1 : 0;
                break;
            }
            case 0xEC: // RD0
            case 0xED:
            case 0xEE:
            case 0xEF:
                cpu->acc = cpu->ram_status[cpu->current_bank][cpu->selected_chip][cpu->selected_register][op - 0xEC];
                break;
            default:
                break;
        }
    }
    else if (op >= 0xF0 && op <= 0xFD) {
        // Accumulator Group
        switch (op) {
            case 0xF0: // CLB
                cpu->acc = 0;
                cpu->carry = 0;
                break;
            case 0xF1: // CLC
                cpu->carry = 0;
                break;
            case 0xF2: { // IAC
                uint8_t sum = cpu->acc + 1;
                cpu->acc = sum & 0x0F;
                cpu->carry = (sum > 15) ? 1 : 0;
                break;
            }
            case 0xF3: // CMC
                cpu->carry = cpu->carry ? 0 : 1;
                break;
            case 0xF4: // CMA
                cpu->acc = ~cpu->acc & 0x0F;
                break;
            case 0xF5: { // RAL
                uint8_t temp = (cpu->acc << 1) | cpu->carry;
                cpu->acc = temp & 0x0F;
                cpu->carry = (temp >> 4) & 1;
                break;
            }
            case 0xF6: { // RAR
                uint8_t temp = cpu->acc | (cpu->carry << 4);
                cpu->acc = (temp >> 1) & 0x0F;
                cpu->carry = temp & 1;
                break;
            }
            case 0xF7: // TCC
                cpu->acc = cpu->carry;
                cpu->carry = 0;
                break;
            case 0xF8: { // DAC
                uint8_t sum = cpu->acc + 15;
                cpu->acc = sum & 0x0F;
                cpu->carry = (sum > 15) ? 1 : 0;
                break;
            }
            case 0xF9: // TCS
                cpu->acc = cpu->carry ? 10 : 9;
                cpu->carry = 0;
                break;
            case 0xFA: // STC
                cpu->carry = 1;
                break;
            case 0xFB: // DAA
                if (cpu->acc > 9 || cpu->carry) {
                    uint8_t sum = cpu->acc + 6;
                    cpu->acc = sum & 0x0F;
                    if (sum > 15) cpu->carry = 1;
                }
                break;
            case 0xFC: // KBP
                if (cpu->acc == 0) cpu->acc = 0;
                else if (cpu->acc == 1) cpu->acc = 1;
                else if (cpu->acc == 2) cpu->acc = 2;
                else if (cpu->acc == 4) cpu->acc = 3;
                else if (cpu->acc == 8) cpu->acc = 4;
                else cpu->acc = 15;
                break;
            case 0xFD: // DCL
                cpu->current_bank = cpu->acc & 7;
                break;
            default:
                break;
        }
    }
    
    // Infinite loop check: if JUN or other instructions result in jumping to the exact same instruction
    if (cpu->pc == instr_pc) {
        return 1; // Loop detected, treat as HALT
    }
    
    return 0;
}

void i4004_print_state(void *context) {
    if (!context) return;
    I4004CPU *cpu = (I4004CPU*)context;
    
    printf("Intel 4004 State:\n");
    printf("  PC:  0x%03X    Acc: 0x%X    CY: %d    Ticks: %u\n", 
           cpu->pc, cpu->acc, cpu->carry, cpu->ticks);
    printf("  Stack: [0x%03X, 0x%03X, 0x%03X]\n", 
           cpu->stack[0], cpu->stack[1], cpu->stack[2]);
    printf("  Index Pairs:\n");
    for (int i = 0; i < 8; ++i) {
        printf("    P%d (R%d,R%d): 0x%02X (0x%X, 0x%X)%s", 
               i, i*2, i*2+1,
               (cpu->r[i*2] << 4) | cpu->r[i*2+1],
               cpu->r[i*2], cpu->r[i*2+1],
               (i % 4 == 3) ? "\n" : "  ");
    }
    printf("  RAM Select: Bank=%d, Chip=%d, Reg=%d, Char=%d\n",
           cpu->current_bank, cpu->selected_chip, 
           cpu->selected_register, cpu->selected_char);
}

void i4004_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    I4004CPU *cpu = (I4004CPU*)context;
    uint16_t pc = cpu->pc;
    if (pc >= 4096) {
        snprintf(buf, buf_len, "<out of ROM>");
        return;
    }
    uint8_t op = cpu->rom[pc];
    
    uint8_t data_byte = 0;
    if (pc + 1 < 4096) {
        data_byte = cpu->rom[pc + 1];
    }
    
    if (op == 0x00) {
        snprintf(buf, buf_len, "NOP");
    }
    else if (op >= 0x10 && op <= 0x1F) {
        snprintf(buf, buf_len, "JCN  %c%c%c%c, 0x%02X", 
                 (op & 8) ? 'I' : '_', // Invert condition bit
                 (op & 4) ? 'Z' : '_', // Zero check
                 (op & 2) ? 'C' : '_', // Carry check
                 (op & 1) ? 'T' : '_', // Test signal check
                 data_byte);
    }
    else if (op >= 0x20 && op <= 0x2E && (op & 1) == 0) {
        snprintf(buf, buf_len, "FIM  P%d, 0x%02X", (op >> 1) & 7, data_byte);
    }
    else if (op >= 0x21 && op <= 0x2F && (op & 1) != 0) {
        snprintf(buf, buf_len, "SRC  P%d", (op >> 1) & 7);
    }
    else if (op >= 0x30 && op <= 0x3E && (op & 1) == 0) {
        snprintf(buf, buf_len, "FIN  P%d", (op >> 1) & 7);
    }
    else if (op >= 0x31 && op <= 0x3F && (op & 1) != 0) {
        snprintf(buf, buf_len, "JIN  P%d", (op >> 1) & 7);
    }
    else if (op >= 0x40 && op <= 0x4F) {
        snprintf(buf, buf_len, "JUN  0x%03X", ((op & 0x0F) << 8) | data_byte);
    }
    else if (op >= 0x50 && op <= 0x5F) {
        snprintf(buf, buf_len, "JMS  0x%03X", ((op & 0x0F) << 8) | data_byte);
    }
    else if (op >= 0x60 && op <= 0x6F) {
        snprintf(buf, buf_len, "INC  R%d", op & 0x0F);
    }
    else if (op >= 0x70 && op <= 0x7F) {
        snprintf(buf, buf_len, "ISZ  R%d, 0x%02X", op & 0x0F, data_byte);
    }
    else if (op >= 0x80 && op <= 0x8F) {
        snprintf(buf, buf_len, "ADD  R%d", op & 0x0F);
    }
    else if (op >= 0x90 && op <= 0x9F) {
        snprintf(buf, buf_len, "SUB  R%d", op & 0x0F);
    }
    else if (op >= 0xA0 && op <= 0xAF) {
        snprintf(buf, buf_len, "LD   R%d", op & 0x0F);
    }
    else if (op >= 0xB0 && op <= 0xBF) {
        snprintf(buf, buf_len, "XCH  R%d", op & 0x0F);
    }
    else if (op >= 0xC0 && op <= 0xCF) {
        snprintf(buf, buf_len, "BBL  0x%X", op & 0x0F);
    }
    else if (op >= 0xD0 && op <= 0xDF) {
        snprintf(buf, buf_len, "LDM  0x%X", op & 0x0F);
    }
    else if (op >= 0xE0 && op <= 0xEF) {
        switch (op) {
            case 0xE0: snprintf(buf, buf_len, "WRM"); break;
            case 0xE1: snprintf(buf, buf_len, "WMP"); break;
            case 0xE4: case 0xE5: case 0xE6: case 0xE7:
                snprintf(buf, buf_len, "WR%d", op - 0xE4); break;
            case 0xE8: snprintf(buf, buf_len, "SBM"); break;
            case 0xE9: snprintf(buf, buf_len, "RDM"); break;
            case 0xEA: snprintf(buf, buf_len, "RDR"); break;
            case 0xEB: snprintf(buf, buf_len, "ADM"); break;
            case 0xEC: case 0xED: case 0xEE: case 0xEF:
                snprintf(buf, buf_len, "RD%d", op - 0xEC); break;
            default: snprintf(buf, buf_len, "IO_UNSPEC (0x%02X)", op); break;
        }
    }
    else if (op >= 0xF0 && op <= 0xFD) {
        switch (op) {
            case 0xF0: snprintf(buf, buf_len, "CLB"); break;
            case 0xF1: snprintf(buf, buf_len, "CLC"); break;
            case 0xF2: snprintf(buf, buf_len, "IAC"); break;
            case 0xF3: snprintf(buf, buf_len, "CMC"); break;
            case 0xF4: snprintf(buf, buf_len, "CMA"); break;
            case 0xF5: snprintf(buf, buf_len, "RAL"); break;
            case 0xF6: snprintf(buf, buf_len, "RAR"); break;
            case 0xF7: snprintf(buf, buf_len, "TCC"); break;
            case 0xF8: snprintf(buf, buf_len, "DAC"); break;
            case 0xF9: snprintf(buf, buf_len, "TCS"); break;
            case 0xFA: snprintf(buf, buf_len, "STC"); break;
            case 0xFB: snprintf(buf, buf_len, "DAA"); break;
            case 0xFC: snprintf(buf, buf_len, "KBP"); break;
            case 0xFD: snprintf(buf, buf_len, "DCL"); break;
            default: snprintf(buf, buf_len, "ACC_UNSPEC (0x%02X)", op); break;
        }
    }
    else {
        snprintf(buf, buf_len, "INV  0x%02X", op);
    }
}
