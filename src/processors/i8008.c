#include "i8008.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE 16384 // 16 KB (14-bit address space)

typedef struct I8008CPU {
    uint8_t regs[7]; // A, B, C, D, E, H, L (M is virtual register index 7)
    uint16_t pc;     // 14-bit Program Counter
    uint16_t stack[8]; // 8-level address stack
    
    // Flags
    uint8_t flags_c; // Carry
    uint8_t flags_z; // Zero
    uint8_t flags_s; // Sign
    uint8_t flags_p; // Parity
    
    uint8_t memory[MEM_SIZE];
    uint32_t ticks;
    int halted;
} I8008CPU;

static const char* r_names[] = { "A", "B", "C", "D", "E", "H", "L", "M" };
static const char* c_names[] = { "NZ", "Z", "NC", "C", "PO", "PE", "P", "M" };

static uint8_t calculate_parity(uint8_t val) {
    uint8_t count = 0;
    for (int i = 0; i < 8; ++i) {
        if ((val >> i) & 1) count++;
    }
    return (count % 2 == 0) ? 1 : 0;
}

static inline void update_flags_nzsp(I8008CPU *cpu, uint8_t res) {
    cpu->flags_z = (res == 0) ? 1 : 0;
    cpu->flags_s = (res & 0x80) ? 1 : 0;
    cpu->flags_p = calculate_parity(res);
}

static inline uint8_t get_reg(I8008CPU *cpu, uint8_t reg_idx) {
    if (reg_idx == 7) {
        uint16_t addr = (((uint16_t)cpu->regs[5] & 0x3F) << 8) | cpu->regs[6];
        return cpu->memory[addr];
    }
    return cpu->regs[reg_idx];
}

static inline void set_reg(I8008CPU *cpu, uint8_t reg_idx, uint8_t val) {
    if (reg_idx == 7) {
        uint16_t addr = (((uint16_t)cpu->regs[5] & 0x3F) << 8) | cpu->regs[6];
        cpu->memory[addr] = val;
    } else {
        cpu->regs[reg_idx] = val;
    }
}

static inline int check_cond(I8008CPU *cpu, uint8_t cond) {
    switch (cond) {
        case 0: return cpu->flags_z == 0; // JNZ / CNZ / RNZ
        case 1: return cpu->flags_z == 1; // JZ / CZ / RZ
        case 2: return cpu->flags_c == 0; // JNC / CNC / RNC
        case 3: return cpu->flags_c == 1; // JC / CC / RC
        case 4: return cpu->flags_p == 0; // JPO / CPO / RPO (Odd)
        case 5: return cpu->flags_p == 1; // JPE / CPE / RPE (Even)
        case 6: return cpu->flags_s == 0; // JP / CP / RP (Positive)
        case 7: return cpu->flags_s == 1; // JM / CM / RM (Minus)
    }
    return 0;
}

static inline void push_stack(I8008CPU *cpu, uint16_t addr) {
    for (int i = 7; i > 0; --i) {
        cpu->stack[i] = cpu->stack[i - 1];
    }
    cpu->stack[0] = addr & 0x3FFF;
}

static inline uint16_t pop_stack(I8008CPU *cpu) {
    uint16_t addr = cpu->stack[0];
    for (int i = 0; i < 7; ++i) {
        cpu->stack[i] = cpu->stack[i + 1];
    }
    cpu->stack[7] = 0;
    return addr;
}

void* i8008_create(void) {
    I8008CPU *cpu = (I8008CPU*)calloc(1, sizeof(I8008CPU));
    return cpu;
}

void i8008_destroy(void *context) {
    free(context);
}

int i8008_init(void *context) {
    if (!context) return -1;
    I8008CPU *cpu = (I8008CPU*)context;
    
    memset(cpu->regs, 0, sizeof(cpu->regs));
    cpu->pc = 0;
    memset(cpu->stack, 0, sizeof(cpu->stack));
    cpu->flags_c = 0;
    cpu->flags_z = 0;
    cpu->flags_s = 0;
    cpu->flags_p = 0;
    memset(cpu->memory, 0, sizeof(cpu->memory));
    cpu->ticks = 0;
    cpu->halted = 0;
    return 0;
}

int i8008_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    I8008CPU *cpu = (I8008CPU*)context;
    
    if (address >= MEM_SIZE) return -2;
    size_t copy_len = size;
    if (address + size > MEM_SIZE) {
        copy_len = MEM_SIZE - address;
    }
    memcpy(&cpu->memory[address], data, copy_len);
    return 0;
}

int i8008_step(void *context) {
    if (!context) return -1;
    I8008CPU *cpu = (I8008CPU*)context;
    
    if (cpu->halted) return 1;
    if (cpu->pc >= MEM_SIZE) return -3;
    
    uint16_t instr_pc = cpu->pc;
    uint8_t op = cpu->memory[cpu->pc];
    
    // Determine instruction size:
    // 3 bytes: JMP/Jcond, CAL/Ccond (pattern 01xxx100 or 01xxx110)
    // 2 bytes: MVI (pattern 00ddd110 except MVI M which is 00111110), ALU Immediate (pattern 00xxx100)
    // 1 byte: all others
    uint8_t is_3byte = 0;
    uint8_t is_2byte = 0;
    uint8_t byte2 = 0;
    uint8_t byte3 = 0;
    
    if ((op & 0xC7) == 0x44 || (op & 0xC7) == 0x46) {
        is_3byte = 1;
        byte2 = cpu->memory[(cpu->pc + 1) % MEM_SIZE];
        byte3 = cpu->memory[(cpu->pc + 2) % MEM_SIZE];
    } else if ((op & 0xC7) == 0x06 || (op & 0xC7) == 0x04) {
        is_2byte = 1;
        byte2 = cpu->memory[(cpu->pc + 1) % MEM_SIZE];
    }
    
    // Advance PC
    uint16_t next_pc = (cpu->pc + (is_3byte ? 3 : (is_2byte ? 2 : 1))) % MEM_SIZE;
    cpu->pc = next_pc;
    cpu->ticks++;
    
    // Decode and Execute
    if (op == 0xFF) {
        // HLT (represented by MOV M, M)
        cpu->halted = 1;
        return 1;
    }
    else if ((op & 0xC0) == 0xC0) {
        // MOV r1, r2 (11dddsss)
        uint8_t dest = (op >> 3) & 0x07;
        uint8_t src = op & 0x07;
        set_reg(cpu, dest, get_reg(cpu, src));
    }
    else if ((op & 0xC7) == 0x06) {
        // MVI r, data (00ddd110)
        uint8_t dest = (op >> 3) & 0x07;
        set_reg(cpu, dest, byte2);
    }
    else if ((op & 0xC7) == 0x00) {
        // INR r (00ddd000)
        uint8_t reg = (op >> 3) & 0x07;
        if (reg < 7) {
            uint8_t val = get_reg(cpu, reg) + 1;
            set_reg(cpu, reg, val);
            update_flags_nzsp(cpu, val);
        }
    }
    else if ((op & 0xC7) == 0x01) {
        // DCR r (00ddd001)
        uint8_t reg = (op >> 3) & 0x07;
        if (reg < 7) {
            uint8_t val = get_reg(cpu, reg) - 1;
            set_reg(cpu, reg, val);
            update_flags_nzsp(cpu, val);
        }
    }
    else if ((op & 0xC0) == 0x80) {
        // ALU operations (10aaasss)
        uint8_t alu_op = (op >> 3) & 0x07;
        uint8_t src = op & 0x07;
        uint8_t val = get_reg(cpu, src);
        uint8_t acc = cpu->regs[0];
        
        switch (alu_op) {
            case 0: { // ADD
                uint16_t sum = (uint16_t)acc + val;
                cpu->regs[0] = sum & 0xFF;
                cpu->flags_c = (sum > 0xFF) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 1: { // ADC (Add with Carry)
                uint16_t sum = (uint16_t)acc + val + cpu->flags_c;
                cpu->regs[0] = sum & 0xFF;
                cpu->flags_c = (sum > 0xFF) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 2: { // SUB
                uint16_t diff = (uint16_t)acc - val;
                cpu->regs[0] = diff & 0xFF;
                cpu->flags_c = (acc < val) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 3: { // SBB (Subtract with Borrow)
                uint16_t borrow = val + cpu->flags_c;
                uint16_t diff = (uint16_t)acc - borrow;
                cpu->regs[0] = diff & 0xFF;
                cpu->flags_c = (acc < borrow) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 4: { // ANA (AND)
                cpu->regs[0] = acc & val;
                cpu->flags_c = 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 5: { // XRA (XOR)
                cpu->regs[0] = acc ^ val;
                cpu->flags_c = 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 6: { // ORA (OR)
                cpu->regs[0] = acc | val;
                cpu->flags_c = 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 7: { // CMP (Compare)
                uint16_t diff = (uint16_t)acc - val;
                cpu->flags_c = (acc < val) ? 1 : 0;
                update_flags_nzsp(cpu, diff & 0xFF);
                break;
            }
        }
    }
    else if ((op & 0xC7) == 0x04) {
        // ALU Immediate operations (00aaacal) followed by data byte
        uint8_t alu_op = (op >> 3) & 0x07;
        uint8_t val = byte2;
        uint8_t acc = cpu->regs[0];
        
        switch (alu_op) {
            case 0: { // ADI
                uint16_t sum = (uint16_t)acc + val;
                cpu->regs[0] = sum & 0xFF;
                cpu->flags_c = (sum > 0xFF) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 1: { // ACI
                uint16_t sum = (uint16_t)acc + val + cpu->flags_c;
                cpu->regs[0] = sum & 0xFF;
                cpu->flags_c = (sum > 0xFF) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 2: { // SUI
                uint16_t diff = (uint16_t)acc - val;
                cpu->regs[0] = diff & 0xFF;
                cpu->flags_c = (acc < val) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 3: { // SBI
                uint16_t borrow = val + cpu->flags_c;
                uint16_t diff = (uint16_t)acc - borrow;
                cpu->regs[0] = diff & 0xFF;
                cpu->flags_c = (acc < borrow) ? 1 : 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 4: { // ANI
                cpu->regs[0] = acc & val;
                cpu->flags_c = 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 5: { // XRI
                cpu->regs[0] = acc ^ val;
                cpu->flags_c = 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 6: { // ORI
                cpu->regs[0] = acc | val;
                cpu->flags_c = 0;
                update_flags_nzsp(cpu, cpu->regs[0]);
                break;
            }
            case 7: { // CPI
                uint16_t diff = (uint16_t)acc - val;
                cpu->flags_c = (acc < val) ? 1 : 0;
                update_flags_nzsp(cpu, diff & 0xFF);
                break;
            }
        }
    }
    else if ((op & 0xC7) == 0x02) {
        // Rotates (00aaacal) where cal=010
        uint8_t rot_op = (op >> 3) & 0x03;
        uint8_t acc = cpu->regs[0];
        
        switch (rot_op) {
            case 0: { // RLC
                uint8_t msb = (acc >> 7) & 1;
                cpu->regs[0] = (acc << 1) | msb;
                cpu->flags_c = msb;
                break;
            }
            case 1: { // RRC
                uint8_t lsb = acc & 1;
                cpu->regs[0] = (acc >> 1) | (lsb << 7);
                cpu->flags_c = lsb;
                break;
            }
            case 2: { // RAL
                uint8_t msb = (acc >> 7) & 1;
                cpu->regs[0] = (acc << 1) | cpu->flags_c;
                cpu->flags_c = msb;
                break;
            }
            case 3: { // RAR
                uint8_t lsb = acc & 1;
                cpu->regs[0] = (acc >> 1) | (cpu->flags_c << 7);
                cpu->flags_c = lsb;
                break;
            }
        }
    }
    else if ((op & 0xC7) == 0x44) {
        // JMP / Jcond (01ccc100)
        uint16_t addr = (((uint16_t)byte3 & 0x3F) << 8) | byte2;
        uint8_t cond = (op >> 3) & 0x07;
        
        if (op == 0x44) { // Unconditional JMP (ccc = 000 with pattern 01000100)
            cpu->pc = addr;
        } else { // Conditional Jcond
            if (check_cond(cpu, cond)) {
                cpu->pc = addr;
            }
        }
    }
    else if ((op & 0xC7) == 0x46) {
        // CAL / Ccond (01ccc110)
        uint16_t addr = (((uint16_t)byte3 & 0x3F) << 8) | byte2;
        uint8_t cond = (op >> 3) & 0x07;
        
        if (op == 0x46) { // Unconditional CAL
            push_stack(cpu, next_pc);
            cpu->pc = addr;
        } else { // Conditional Ccond
            if (check_cond(cpu, cond)) {
                push_stack(cpu, next_pc);
                cpu->pc = addr;
            }
        }
    }
    else if ((op & 0xC7) == 0x07) {
        // RET (00xxx111) -> Unconditional
        cpu->pc = pop_stack(cpu);
    }
    else if ((op & 0xC7) == 0x03) {
        // Rcond (00ccc011)
        uint8_t cond = (op >> 3) & 0x07;
        if (check_cond(cpu, cond)) {
            cpu->pc = pop_stack(cpu);
        }
    }
    else if ((op & 0xC7) == 0x05) {
        // RST (00nnn101)
        uint8_t vec = (op >> 3) & 0x07;
        push_stack(cpu, next_pc);
        cpu->pc = (uint16_t)vec * 8;
    }
    
    // Checked if PC self-looped (interpreted as software halt)
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }
    
    return 0;
}

void i8008_print_state(void *context) {
    if (!context) return;
    I8008CPU *cpu = (I8008CPU*)context;
    
    printf("Intel 8008 CPU State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  Halted: %s\n", cpu->pc, cpu->halted ? "Yes" : "No");
    printf("  Flags: Carry=%d, Zero=%d, Sign=%d, Parity=%d\n",
           cpu->flags_c, cpu->flags_z, cpu->flags_s, cpu->flags_p);
    printf("  Registers:\n");
    for (int i = 0; i < 7; ++i) {
        printf("    %s: 0x%02X%s", r_names[i], cpu->regs[i], (i == 3 || i == 6) ? "\n" : "  ");
    }
    uint16_t m_addr = (((uint16_t)cpu->regs[5] & 0x3F) << 8) | cpu->regs[6];
    printf("  M (at 0x%04X): 0x%02X\n", m_addr, cpu->memory[m_addr]);
    printf("  Stack levels: [0x%04X, 0x%04X, 0x%04X, 0x%04X, 0x%04X, 0x%04X, 0x%04X, 0x%04X]\n",
           cpu->stack[0], cpu->stack[1], cpu->stack[2], cpu->stack[3],
           cpu->stack[4], cpu->stack[5], cpu->stack[6], cpu->stack[7]);
}

void i8008_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    I8008CPU *cpu = (I8008CPU*)context;
    
    if (cpu->pc >= MEM_SIZE) {
        snprintf(buf, buf_len, "<out of RAM>");
        return;
    }
    
    uint8_t op = cpu->memory[cpu->pc];
    uint8_t byte2 = 0;
    uint8_t byte3 = 0;
    if ((cpu->pc + 1) < MEM_SIZE) byte2 = cpu->memory[cpu->pc + 1];
    if ((cpu->pc + 2) < MEM_SIZE) byte3 = cpu->memory[cpu->pc + 2];
    
    if (op == 0xFF) {
        snprintf(buf, buf_len, "HLT (MOV M,M)");
    }
    else if ((op & 0xC0) == 0xC0) {
        snprintf(buf, buf_len, "MOV   %s, %s", r_names[(op >> 3) & 7], r_names[op & 7]);
    }
    else if ((op & 0xC7) == 0x06) {
        snprintf(buf, buf_len, "MVI   %s, 0x%02X", r_names[(op >> 3) & 7], byte2);
    }
    else if ((op & 0xC7) == 0x00) {
        snprintf(buf, buf_len, "INR   %s", r_names[(op >> 3) & 7]);
    }
    else if ((op & 0xC7) == 0x01) {
        snprintf(buf, buf_len, "DCR   %s", r_names[(op >> 3) & 7]);
    }
    else if ((op & 0xC0) == 0x80) {
        const char* alu_names[] = { "ADD", "ADC", "SUB", "SBB", "ANA", "XRA", "ORA", "CMP" };
        snprintf(buf, buf_len, "%-5s %s", alu_names[(op >> 3) & 7], r_names[op & 7]);
    }
    else if ((op & 0xC7) == 0x04) {
        const char* alu_imm_names[] = { "ADI", "ACI", "SUI", "SBI", "ANI", "XRI", "ORI", "CPI" };
        snprintf(buf, buf_len, "%-5s 0x%02X", alu_imm_names[(op >> 3) & 7], byte2);
    }
    else if ((op & 0xC7) == 0x02) {
        const char* rot_names[] = { "RLC", "RRC", "RAL", "RAR" };
        snprintf(buf, buf_len, "%s", rot_names[(op >> 3) & 3]);
    }
    else if ((op & 0xC7) == 0x44) {
        uint16_t addr = (((uint16_t)byte3 & 0x3F) << 8) | byte2;
        if (op == 0x44) snprintf(buf, buf_len, "JMP   0x%04X", addr);
        else snprintf(buf, buf_len, "J%-4s 0x%04X", c_names[(op >> 3) & 7], addr);
    }
    else if ((op & 0xC7) == 0x46) {
        uint16_t addr = (((uint16_t)byte3 & 0x3F) << 8) | byte2;
        if (op == 0x46) snprintf(buf, buf_len, "CAL   0x%04X", addr);
        else snprintf(buf, buf_len, "C%-4s 0x%04X", c_names[(op >> 3) & 7], addr);
    }
    else if ((op & 0xC7) == 0x07) {
        snprintf(buf, buf_len, "RET");
    }
    else if ((op & 0xC7) == 0x03) {
        snprintf(buf, buf_len, "R%-4s", c_names[(op >> 3) & 7]);
    }
    else if ((op & 0xC7) == 0x05) {
        snprintf(buf, buf_len, "RST   %d (0x%02X)", (op >> 3) & 7, ((op >> 3) & 7) * 8);
    }
    else {
        snprintf(buf, buf_len, "INV   0x%02X", op);
    }
}
