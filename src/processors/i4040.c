#include "i4040.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define I4040_ROM_BANK_SIZE 4096
#define I4040_ROM_BANKS     2
#define I4040_ROM_SIZE      (I4040_ROM_BANK_SIZE * I4040_ROM_BANKS)
#define I4040_STACK_DEPTH   7

typedef struct I4040CPU {
    uint16_t pc;
    uint16_t stack[I4040_STACK_DEPTH];
    uint8_t acc;
    uint8_t carry;
    uint8_t test_signal;
    uint8_t r[24];              // 24 index registers: R0-R7 in two banks + shared R8-R15
    uint8_t reg_bank;           // Index register bank (SB0/SB1)
    uint8_t rom_bank;           // ROM bank (DB0/DB1)
    uint8_t int_enable;         // Interrupt enable flag (EIN/DIN)
    uint8_t halted;             // Set by HLT

    // Selection state
    uint8_t current_bank;       // RAM bank (DCL / command register)
    uint8_t selected_chip;
    uint8_t selected_register;
    uint8_t selected_char;
    uint8_t src_value;          // Full 8-bit SRC address (for RPM / BBS restore)
    uint8_t saved_src;          // SRC value saved on interrupt, restored by BBS
    uint8_t rpm_half;           // RPM half-byte toggle (0 = high nibble first)

    // Memory
    uint8_t rom[I4040_ROM_SIZE];
    uint8_t pram[256];          // 4008/4009 program RAM (read via RPM), byte-wide
    uint8_t ram_data[8][4][4][16];  // 8 banks, 4 chips, 4 registers, 16 characters
    uint8_t ram_status[8][4][4][4]; // 8 banks, 4 chips, 4 registers, 4 status characters
    uint8_t ram_ports[8][4];        // RAM output ports
    uint8_t rom_ports[16];          // ROM ports

    uint32_t ticks;
} I4040CPU;

// R0-R7 are banked (SB0/SB1); R8-R15 are shared between banks.
static uint8_t* i4040_reg(I4040CPU *cpu, uint8_t reg) {
    reg &= 0x0F;
    if (reg < 8 && cpu->reg_bank) {
        return &cpu->r[16 + reg];
    }
    return &cpu->r[reg];
}

static uint8_t i4040_pair_value(I4040CPU *cpu, uint8_t pair) {
    return (uint8_t)((*i4040_reg(cpu, (uint8_t)(pair * 2)) << 4) |
                      *i4040_reg(cpu, (uint8_t)(pair * 2 + 1)));
}

static void i4040_stack_push(I4040CPU *cpu, uint16_t value) {
    int i;
    for (i = I4040_STACK_DEPTH - 1; i > 0; --i) {
        cpu->stack[i] = cpu->stack[i - 1];
    }
    cpu->stack[0] = value;
}

static uint16_t i4040_stack_pop(I4040CPU *cpu) {
    uint16_t value = cpu->stack[0];
    int i;
    for (i = 0; i < I4040_STACK_DEPTH - 1; ++i) {
        cpu->stack[i] = cpu->stack[i + 1];
    }
    cpu->stack[I4040_STACK_DEPTH - 1] = 0;
    return value;
}

static void i4040_apply_src(I4040CPU *cpu, uint8_t val) {
    cpu->src_value = val;
    cpu->selected_chip = (val >> 6) & 3;
    cpu->selected_register = (val >> 4) & 3;
    cpu->selected_char = val & 0x0F;
}

static uint8_t i4040_fetch(I4040CPU *cpu, uint16_t addr) {
    return cpu->rom[((uint16_t)cpu->rom_bank << 12) | (addr & 0xFFF)];
}

void* i4040_create(void) {
    I4040CPU *cpu = (I4040CPU*)calloc(1, sizeof(I4040CPU));
    if (cpu) {
        cpu->test_signal = 1; // Default to high/inactive
    }
    return cpu;
}

void i4040_destroy(void *context) {
    free(context);
}

int i4040_init(void *context) {
    if (!context) return -1;
    I4040CPU *cpu = (I4040CPU*)context;

    cpu->pc = 0;
    memset(cpu->stack, 0, sizeof(cpu->stack));
    cpu->acc = 0;
    cpu->carry = 0;
    cpu->test_signal = 1;
    memset(cpu->r, 0, sizeof(cpu->r));
    cpu->reg_bank = 0;
    cpu->rom_bank = 0;
    cpu->int_enable = 0;
    cpu->halted = 0;

    cpu->current_bank = 0;
    cpu->selected_chip = 0;
    cpu->selected_register = 0;
    cpu->selected_char = 0;
    cpu->src_value = 0;
    cpu->saved_src = 0;
    cpu->rpm_half = 0;

    memset(cpu->rom, 0, sizeof(cpu->rom));
    memset(cpu->pram, 0, sizeof(cpu->pram));
    memset(cpu->ram_data, 0, sizeof(cpu->ram_data));
    memset(cpu->ram_status, 0, sizeof(cpu->ram_status));
    memset(cpu->ram_ports, 0, sizeof(cpu->ram_ports));
    memset(cpu->rom_ports, 0, sizeof(cpu->rom_ports));

    cpu->ticks = 0;
    return 0;
}

int i4040_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    I4040CPU *cpu = (I4040CPU*)context;
    if (address >= sizeof(cpu->rom)) return -2;

    size_t copy_len = size;
    if (address + size > sizeof(cpu->rom)) {
        copy_len = sizeof(cpu->rom) - address;
    }
    memcpy(&cpu->rom[address], data, copy_len);
    return 0;
}

int i4040_step(void *context) {
    if (!context) return -1;
    I4040CPU *cpu = (I4040CPU*)context;

    if (cpu->halted) {
        return 1;
    }

    if (cpu->pc >= I4040_ROM_BANK_SIZE) {
        return -3; // PC out of ROM bounds
    }

    uint16_t instr_pc = cpu->pc; // Save start address to check for infinite loop
    uint8_t op = i4040_fetch(cpu, cpu->pc);
    uint8_t is_2byte = 0;
    uint8_t data_byte = 0;

    // Determine if instruction is 2 bytes
    if ((op >= 0x10 && op <= 0x1F) ||
        (op >= 0x20 && op <= 0x2E && (op & 1) == 0) ||
        (op >= 0x40 && op <= 0x4F) ||
        (op >= 0x50 && op <= 0x5F) ||
        (op >= 0x70 && op <= 0x7F)) {
        is_2byte = 1;
        if (cpu->pc + 1 >= I4040_ROM_BANK_SIZE) {
            return -4; // Second byte out of bounds
        }
        data_byte = i4040_fetch(cpu, (uint16_t)(cpu->pc + 1));
    }

    // Advance PC
    cpu->pc = (cpu->pc + (is_2byte ? 2 : 1)) & 0xFFF;
    cpu->ticks += is_2byte ? 2 : 1; // 2-byte instructions take 2 cycles, 1-byte take 1 cycle

    // Execute
    if (op == 0x00) {
        // NOP
    }
    else if (op >= 0x01 && op <= 0x0F) {
        // 4040 extended instruction group
        switch (op) {
            case 0x01: // HLT
                cpu->halted = 1;
                return 1;
            case 0x02: // BBS - branch back from interrupt, restore SRC
                cpu->pc = i4040_stack_pop(cpu);
                i4040_apply_src(cpu, cpu->saved_src);
                cpu->int_enable = 1;
                break;
            case 0x03: // LCR - load command register into accumulator
                cpu->acc = (uint8_t)(((cpu->rom_bank & 1) << 3) | (cpu->current_bank & 7));
                break;
            case 0x04: // OR4 - OR register 4 with accumulator
                cpu->acc = (cpu->acc | *i4040_reg(cpu, 4)) & 0x0F;
                break;
            case 0x05: // OR5 - OR register 5 with accumulator
                cpu->acc = (cpu->acc | *i4040_reg(cpu, 5)) & 0x0F;
                break;
            case 0x06: // AN6 - AND register 6 with accumulator
                cpu->acc = (cpu->acc & *i4040_reg(cpu, 6)) & 0x0F;
                break;
            case 0x07: // AN7 - AND register 7 with accumulator
                cpu->acc = (cpu->acc & *i4040_reg(cpu, 7)) & 0x0F;
                break;
            case 0x08: // DB0 - designate ROM bank 0
                cpu->rom_bank = 0;
                break;
            case 0x09: // DB1 - designate ROM bank 1
                cpu->rom_bank = 1;
                break;
            case 0x0A: // SB0 - select index register bank 0
                cpu->reg_bank = 0;
                break;
            case 0x0B: // SB1 - select index register bank 1
                cpu->reg_bank = 1;
                break;
            case 0x0C: // EIN - enable interrupts
                cpu->int_enable = 1;
                break;
            case 0x0D: // DIN - disable interrupts
                cpu->int_enable = 0;
                break;
            case 0x0E: { // RPM - read program memory (4008/4009), half-byte at a time
                uint8_t byte_val = cpu->pram[cpu->src_value];
                cpu->acc = cpu->rpm_half ? (byte_val & 0x0F) : ((byte_val >> 4) & 0x0F);
                cpu->rpm_half ^= 1;
                break;
            }
            default: // 0x0F unassigned
                break;
        }
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
        *i4040_reg(cpu, (uint8_t)(pair * 2)) = (data_byte >> 4) & 0x0F;
        *i4040_reg(cpu, (uint8_t)(pair * 2 + 1)) = data_byte & 0x0F;
    }
    else if (op >= 0x21 && op <= 0x2F && (op & 1) != 0) {
        // SRC
        uint8_t pair = (op >> 1) & 7;
        i4040_apply_src(cpu, i4040_pair_value(cpu, pair));
    }
    else if (op >= 0x30 && op <= 0x3E && (op & 1) == 0) {
        // FIN
        uint8_t pair = (op >> 1) & 7;
        uint16_t fetch_addr = (instr_pc & 0xF00) | i4040_pair_value(cpu, 0);
        uint8_t val = i4040_fetch(cpu, fetch_addr);
        *i4040_reg(cpu, (uint8_t)(pair * 2)) = (val >> 4) & 0x0F;
        *i4040_reg(cpu, (uint8_t)(pair * 2 + 1)) = val & 0x0F;
    }
    else if (op >= 0x31 && op <= 0x3F && (op & 1) != 0) {
        // JIN
        uint8_t pair = (op >> 1) & 7;
        cpu->pc = (instr_pc & 0xF00) | i4040_pair_value(cpu, pair);
    }
    else if (op >= 0x40 && op <= 0x4F) {
        // JUN
        cpu->pc = (uint16_t)(((op & 0x0F) << 8) | data_byte);
    }
    else if (op >= 0x50 && op <= 0x5F) {
        // JMS
        i4040_stack_push(cpu, cpu->pc);
        cpu->pc = (uint16_t)(((op & 0x0F) << 8) | data_byte);
    }
    else if (op >= 0x60 && op <= 0x6F) {
        // INC
        uint8_t *reg = i4040_reg(cpu, op & 0x0F);
        *reg = (*reg + 1) & 0x0F;
    }
    else if (op >= 0x70 && op <= 0x7F) {
        // ISZ
        uint8_t *reg = i4040_reg(cpu, op & 0x0F);
        *reg = (*reg + 1) & 0x0F;
        if (*reg != 0) {
            cpu->pc = (instr_pc & 0xF00) | data_byte;
        }
    }
    else if (op >= 0x80 && op <= 0x8F) {
        // ADD
        uint8_t sum = cpu->acc + *i4040_reg(cpu, op & 0x0F) + cpu->carry;
        cpu->acc = sum & 0x0F;
        cpu->carry = (sum > 15) ? 1 : 0;
    }
    else if (op >= 0x90 && op <= 0x9F) {
        // SUB
        uint8_t sum = cpu->acc + (~(*i4040_reg(cpu, op & 0x0F)) & 0x0F) + cpu->carry;
        cpu->acc = sum & 0x0F;
        cpu->carry = (sum > 15) ? 1 : 0;
    }
    else if (op >= 0xA0 && op <= 0xAF) {
        // LD
        cpu->acc = *i4040_reg(cpu, op & 0x0F);
    }
    else if (op >= 0xB0 && op <= 0xBF) {
        // XCH
        uint8_t *reg = i4040_reg(cpu, op & 0x0F);
        uint8_t tmp = cpu->acc;
        cpu->acc = *reg;
        *reg = tmp;
    }
    else if (op >= 0xC0 && op <= 0xCF) {
        // BBL
        cpu->acc = op & 0x0F;
        cpu->pc = i4040_stack_pop(cpu);
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
                uint8_t temp = (uint8_t)((cpu->acc << 1) | cpu->carry);
                cpu->acc = temp & 0x0F;
                cpu->carry = (temp >> 4) & 1;
                break;
            }
            case 0xF6: { // RAR
                uint8_t temp = (uint8_t)(cpu->acc | (cpu->carry << 4));
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

void i4040_print_state(void *context) {
    if (!context) return;
    I4040CPU *cpu = (I4040CPU*)context;
    int i;

    printf("Intel 4040 State:\n");
    printf("  PC:  0x%03X    Acc: 0x%X    CY: %d    Ticks: %u\n",
           cpu->pc, cpu->acc, cpu->carry, cpu->ticks);
    printf("  Halted: %d    IntEnable: %d    RegBank: %d    ROMBank: %d\n",
           cpu->halted, cpu->int_enable, cpu->reg_bank, cpu->rom_bank);
    printf("  Stack: [0x%03X, 0x%03X, 0x%03X, 0x%03X, 0x%03X, 0x%03X, 0x%03X]\n",
           cpu->stack[0], cpu->stack[1], cpu->stack[2], cpu->stack[3],
           cpu->stack[4], cpu->stack[5], cpu->stack[6]);
    printf("  Index Pairs (bank %d for P0-P3):\n", cpu->reg_bank);
    for (i = 0; i < 8; ++i) {
        printf("    P%d (R%d,R%d): 0x%02X (0x%X, 0x%X)%s",
               i, i*2, i*2+1,
               (*i4040_reg(cpu, (uint8_t)(i*2)) << 4) | *i4040_reg(cpu, (uint8_t)(i*2+1)),
               *i4040_reg(cpu, (uint8_t)(i*2)), *i4040_reg(cpu, (uint8_t)(i*2+1)),
               (i % 4 == 3) ? "\n" : "  ");
    }
    printf("  RAM Select: Bank=%d, Chip=%d, Reg=%d, Char=%d\n",
           cpu->current_bank, cpu->selected_chip,
           cpu->selected_register, cpu->selected_char);
}

void i4040_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    I4040CPU *cpu = (I4040CPU*)context;
    uint16_t pc = cpu->pc;
    if (pc >= I4040_ROM_BANK_SIZE) {
        snprintf(buf, buf_len, "<out of ROM>");
        return;
    }
    uint8_t op = i4040_fetch(cpu, pc);

    uint8_t data_byte = 0;
    if (pc + 1 < I4040_ROM_BANK_SIZE) {
        data_byte = i4040_fetch(cpu, (uint16_t)(pc + 1));
    }

    if (op == 0x00) {
        snprintf(buf, buf_len, "NOP");
    }
    else if (op >= 0x01 && op <= 0x0F) {
        switch (op) {
            case 0x01: snprintf(buf, buf_len, "HLT"); break;
            case 0x02: snprintf(buf, buf_len, "BBS"); break;
            case 0x03: snprintf(buf, buf_len, "LCR"); break;
            case 0x04: snprintf(buf, buf_len, "OR4"); break;
            case 0x05: snprintf(buf, buf_len, "OR5"); break;
            case 0x06: snprintf(buf, buf_len, "AN6"); break;
            case 0x07: snprintf(buf, buf_len, "AN7"); break;
            case 0x08: snprintf(buf, buf_len, "DB0"); break;
            case 0x09: snprintf(buf, buf_len, "DB1"); break;
            case 0x0A: snprintf(buf, buf_len, "SB0"); break;
            case 0x0B: snprintf(buf, buf_len, "SB1"); break;
            case 0x0C: snprintf(buf, buf_len, "EIN"); break;
            case 0x0D: snprintf(buf, buf_len, "DIN"); break;
            case 0x0E: snprintf(buf, buf_len, "RPM"); break;
            default:   snprintf(buf, buf_len, "INV  0x%02X", op); break;
        }
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
