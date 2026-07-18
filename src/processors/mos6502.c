#include "mos6502.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLAG_C 0x01
#define FLAG_Z 0x02
#define FLAG_I 0x04
#define FLAG_D 0x08
#define FLAG_B 0x10
#define FLAG_U 0x20
#define FLAG_V 0x40
#define FLAG_N 0x80

typedef struct MOS6502_CPU {
    uint8_t ram[65536];
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t sp;
    uint8_t p;
    uint16_t pc;
    uint32_t ticks;
    int halted;
    
    // iNES Cartridge details
    int is_nes_rom;
    uint8_t prg_rom[262144]; // supports up to 256 KB
    uint8_t chr_rom[262144]; // supports up to 256 KB
    uint32_t prg_rom_size;
    uint32_t chr_rom_size;
    uint8_t mapper_num;
    
    // MMC1 Mapper Registers
    uint8_t mmc1_shift;
    uint8_t mmc1_ctrl;
    uint8_t mmc1_chr0;
    uint8_t mmc1_chr1;
    uint8_t mmc1_prg;
    
    // Active bank pointers
    uint8_t *prg_bank0;
    uint8_t *prg_bank1;
    uint8_t *chr_bank0;
    uint8_t *chr_bank1;
    
    // PPU Memory Map & registers
    uint8_t ppu_vram[2048];   // Nametables (Vertical mirroring base)
    uint8_t ppu_palette[32];  // Palettes
    uint8_t oam[256];         // Sprite info
    uint16_t ppu_addr;
    uint8_t ppu_addr_latch_phase;
    uint8_t ppu_read_buffer;
    uint8_t ppu_ctrl;
    uint8_t ppu_mask;
    uint8_t ppu_status;
    uint8_t ppu_scroll_x;
    uint8_t ppu_scroll_y;
    uint8_t ppu_scroll_phase;
    
    // Controller Latch
    uint8_t controller_latched;
    uint8_t controller_shift;
    
    // Frame cycle counter for NMI timing
    uint32_t frame_cycles;
} MOS6502_CPU;

#define SET_FLAG_C(cond) do { if (cond) cpu->p |= FLAG_C; else cpu->p &= ~FLAG_C; } while(0)
#define SET_FLAG_Z(val) do { if ((val) == 0) cpu->p |= FLAG_Z; else cpu->p &= ~FLAG_Z; } while(0)
#define SET_FLAG_N(val) do { if ((val) & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N; } while(0)
#define GET_FLAG(flag) ((cpu->p & (flag)) ? 1 : 0)

// Standard NES Palette (GDI compatible 0x00BBGGRR colors)
static const uint32_t g_nes_palette[64] = {
    0x007C7C7C, 0x00FC1800, 0x00BC0018, 0x00443858, 0x0094007C, 0x00B800B8, 0x00E810AC, 0x00B02C5C,
    0x000030E4, 0x000078A8, 0x0000684C, 0x00005800, 0x00407000, 0x00000000, 0x00000000, 0x00000000,
    0x00BCBCBC, 0x00F87800, 0x00F85800, 0x00AC44D8, 0x00F800B8, 0x00F838F8, 0x00F83CE4, 0x00CF4C00,
    0x005C8200, 0x0000BC00, 0x0000B43C, 0x0000A85C, 0x0000A0A0, 0x00000000, 0x00000000, 0x00000000,
    0x00F8F8F8, 0x00FCBC3C, 0x00FCB868, 0x00FCB0B8, 0x00FCA0F8, 0x00FCB0FC, 0x00FCBCF8, 0x00D4D03C,
    0x0080D000, 0x0000F800, 0x0040F878, 0x0000E8BC, 0x0000E0E0, 0x00000000, 0x00000000, 0x00000000,
    0x00F8F8F8, 0x00FCE0A4, 0x00FCD8B8, 0x00FCD0D8, 0x00FCC8F8, 0x00FCD0F8, 0x00FCD8FC, 0x00ECE4A4,
    0x00B0E43C, 0x0040F800, 0x00B0F8B8, 0x0000F0D8, 0x0000E8E8, 0x00000000, 0x00000000, 0x00000000
};

// GDI compatible retro C64 palette (0x00BBGGRR) for raw 6502 modes
static const uint32_t g_c64_palette[16] = {
    0x00000000, // Black
    0x00FFFFFF, // White
    0x000000FF, // Red
    0x00FFFF00, // Cyan
    0x00FF00FF, // Violet
    0x0000FF00, // Green
    0x00FF0000, // Blue
    0x0000FFFF, // Yellow
    0x000088FF, // Orange
    0x00004488, // Brown
    0x008888FF, // Light Red
    0x00333333, // Dark Grey
    0x00777777, // Grey
    0x0088FF88, // Light Green
    0x00FF8888, // Light Blue
    0x00BBBBBB  // Light Grey
};

uint8_t ppu_read(MOS6502_CPU *cpu, uint16_t addr) {
    addr &= 0x3FFF;
    if (addr < 0x2000) {
        if (addr < 0x1000) {
            return cpu->chr_bank0[addr];
        } else {
            return cpu->chr_bank1[addr - 0x1000];
        }
    } else if (addr < 0x3F00) {
        uint16_t nt_addr = addr & 0x0FFF;
        if (nt_addr < 0x0800) {
            return cpu->ppu_vram[nt_addr];
        } else {
            return cpu->ppu_vram[nt_addr - 0x0800];
        }
    } else {
        uint16_t pal_addr = addr & 0x001F;
        if ((pal_addr & 0x03) == 0) {
            pal_addr &= 0x0F;
        }
        return cpu->ppu_palette[pal_addr];
    }
}

void ppu_write(MOS6502_CPU *cpu, uint16_t addr, uint8_t val) {
    addr &= 0x3FFF;
    if (addr < 0x2000) {
        if (addr < 0x1000) {
            cpu->chr_bank0[addr] = val;
        } else {
            cpu->chr_bank1[addr - 0x1000] = val;
        }
    } else if (addr < 0x3F00) {
        uint16_t nt_addr = addr & 0x0FFF;
        if (nt_addr < 0x0800) {
            cpu->ppu_vram[nt_addr] = val;
        } else {
            cpu->ppu_vram[nt_addr - 0x0800] = val;
        }
    } else {
        uint16_t pal_addr = addr & 0x001F;
        if ((pal_addr & 0x03) == 0) {
            pal_addr &= 0x0F;
        }
        cpu->ppu_palette[pal_addr] = val;
    }
}

uint8_t cpu_read(MOS6502_CPU *cpu, uint16_t addr) {
    if (cpu->is_nes_rom) {
        if (addr < 0x2000) {
            return cpu->ram[addr & 0x07FF];
        } else if (addr < 0x4000) {
            uint16_t ppu_reg = addr & 0x2007;
            if (ppu_reg == 0x2002) {
                uint8_t status = cpu->ppu_status;
                cpu->ppu_status &= ~0x80; // clear VBLANK
                cpu->ppu_addr_latch_phase = 0;
                cpu->ppu_scroll_phase = 0;
                return status;
            } else if (ppu_reg == 0x2007) {
                uint8_t val = cpu->ppu_read_buffer;
                cpu->ppu_read_buffer = ppu_read(cpu, cpu->ppu_addr);
                if ((cpu->ppu_addr & 0x3F00) == 0x3F00) {
                    val = cpu->ppu_read_buffer; // palette is immediate
                }
                cpu->ppu_addr += (cpu->ppu_ctrl & 0x04) ? 32 : 1;
                return val;
            }
            return 0;
        } else if (addr == 0x4016) {
            uint8_t val = 0;
            if (cpu->controller_shift < 8) {
                val = (cpu->controller_latched >> cpu->controller_shift) & 1;
                cpu->controller_shift++;
            } else {
                val = 1;
            }
            return val;
        } else if (addr >= 0x8000) {
            if (addr < 0xC000) {
                return cpu->prg_bank0[addr - 0x8000];
            } else {
                return cpu->prg_bank1[addr - 0xC000];
            }
        }
    }
    return cpu->ram[addr];
}

void cpu_write(MOS6502_CPU *cpu, uint16_t addr, uint8_t val) {
    if (cpu->is_nes_rom) {
        if (addr < 0x2000) {
            cpu->ram[addr & 0x07FF] = val;
        } else if (addr < 0x4000) {
            uint16_t ppu_reg = addr & 0x2007;
            if (ppu_reg == 0x2000) {
                cpu->ppu_ctrl = val;
            } else if (ppu_reg == 0x2001) {
                cpu->ppu_mask = val;
            } else if (ppu_reg == 0x2005) {
                if (cpu->ppu_scroll_phase == 0) {
                    cpu->ppu_scroll_x = val;
                    cpu->ppu_scroll_phase = 1;
                } else {
                    cpu->ppu_scroll_y = val;
                    cpu->ppu_scroll_phase = 0;
                }
            } else if (ppu_reg == 0x2006) {
                if (cpu->ppu_addr_latch_phase == 0) {
                    cpu->ppu_addr = (cpu->ppu_addr & 0x00FF) | ((uint16_t)val << 8);
                    cpu->ppu_addr_latch_phase = 1;
                } else {
                    cpu->ppu_addr = (cpu->ppu_addr & 0xFF00) | val;
                    cpu->ppu_addr_latch_phase = 0;
                }
            } else if (ppu_reg == 0x2007) {
                ppu_write(cpu, cpu->ppu_addr, val);
                cpu->ppu_addr += (cpu->ppu_ctrl & 0x04) ? 32 : 1;
            }
        } else if (addr == 0x4014) {
            // OAM DMA
            uint16_t src = (uint16_t)val << 8;
            for (int i = 0; i < 256; ++i) {
                cpu->oam[i] = cpu_read(cpu, (uint16_t)(src + i));
            }
            cpu->ticks += 513;
        } else if (addr == 0x4016) {
            if (val & 1) {
                extern uint8_t g_keyboard_state;
                cpu->controller_latched = g_keyboard_state;
                cpu->controller_shift = 0;
            }
        } else if (addr >= 0x8000) {
            // MMC1 serial write
            if (val & 0x80) {
                cpu->mmc1_shift = 0x10;
                cpu->mmc1_ctrl |= 0x0C;
                cpu->prg_bank0 = cpu->prg_rom;
                cpu->prg_bank1 = cpu->prg_rom + cpu->prg_rom_size - 16384;
            } else {
                uint8_t complete = cpu->mmc1_shift & 1;
                cpu->mmc1_shift = (cpu->mmc1_shift >> 1) | ((val & 1) << 4);
                if (complete) {
                    uint8_t load_val = cpu->mmc1_shift;
                    uint16_t reg_addr = addr & 0xE000;
                    
                    if (reg_addr == 0x8000) {
                        cpu->mmc1_ctrl = load_val;
                    } else if (reg_addr == 0xA000) {
                        cpu->mmc1_chr0 = load_val;
                    } else if (reg_addr == 0xC000) {
                        cpu->mmc1_chr1 = load_val;
                    } else if (reg_addr == 0xE000) {
                        cpu->mmc1_prg = load_val;
                    }
                    
                    uint32_t num_prg_banks = cpu->prg_rom_size / 16384;
                    if (num_prg_banks == 0) num_prg_banks = 1;
                    uint32_t num_chr_banks = cpu->chr_rom_size / 4096;
                    if (num_chr_banks == 0) num_chr_banks = 1;
                    
                    uint8_t prg_mode = (cpu->mmc1_ctrl >> 2) & 0x03;
                    uint8_t bank_prg = (cpu->mmc1_prg & 0x0F) % num_prg_banks;
                    if (prg_mode <= 1) {
                        cpu->prg_bank0 = cpu->prg_rom + (bank_prg & 0x0E) * 16384;
                        cpu->prg_bank1 = cpu->prg_bank0 + 16384;
                    } else if (prg_mode == 2) {
                        cpu->prg_bank0 = cpu->prg_rom;
                        cpu->prg_bank1 = cpu->prg_rom + bank_prg * 16384;
                    } else if (prg_mode == 3) {
                        cpu->prg_bank0 = cpu->prg_rom + bank_prg * 16384;
                        cpu->prg_bank1 = cpu->prg_rom + cpu->prg_rom_size - 16384;
                    }
                    
                    if (cpu->mmc1_ctrl & 0x10) {
                        uint8_t bank_chr0 = cpu->mmc1_chr0 % num_chr_banks;
                        uint8_t bank_chr1 = cpu->mmc1_chr1 % num_chr_banks;
                        cpu->chr_bank0 = cpu->chr_rom + bank_chr0 * 4096;
                        cpu->chr_bank1 = cpu->chr_rom + bank_chr1 * 4096;
                    } else {
                        uint8_t bank_chr0 = (cpu->mmc1_chr0 & 0x1E) % num_chr_banks;
                        cpu->chr_bank0 = cpu->chr_rom + bank_chr0 * 4096;
                        cpu->chr_bank1 = cpu->chr_bank0 + 4096;
                    }
                    
                    cpu->mmc1_shift = 0x10;
                }
            }
        }
    } else {
        cpu->ram[addr] = val;
    }
}

void* mos6502_create(void) {
    MOS6502_CPU *cpu = (MOS6502_CPU*)calloc(1, sizeof(MOS6502_CPU));
    return cpu;
}

void mos6502_destroy(void *context) {
    free(context);
}

int mos6502_init(void *context) {
    if (!context) return -1;
    MOS6502_CPU *cpu = (MOS6502_CPU*)context;
    
    memset(cpu->ram, 0, sizeof(cpu->ram));
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->p = FLAG_U | FLAG_I;
    cpu->pc = 0;
    cpu->ticks = 0;
    cpu->halted = 0;
    cpu->is_nes_rom = 0;
    
    // Clear PPU states
    memset(cpu->ppu_vram, 0, sizeof(cpu->ppu_vram));
    memset(cpu->ppu_palette, 0, sizeof(cpu->ppu_palette));
    memset(cpu->oam, 0, sizeof(cpu->oam));
    cpu->ppu_addr = 0;
    cpu->ppu_addr_latch_phase = 0;
    cpu->ppu_read_buffer = 0;
    cpu->ppu_ctrl = 0;
    cpu->ppu_mask = 0;
    cpu->ppu_status = 0;
    cpu->ppu_scroll_x = 0;
    cpu->ppu_scroll_y = 0;
    cpu->ppu_scroll_phase = 0;
    cpu->frame_cycles = 0;
    
    return 0;
}

int mos6502_load(void *context, const uint8_t *data, size_t size, uint32_t address) {
    if (!context) return -1;
    MOS6502_CPU *cpu = (MOS6502_CPU*)context;
    
    if (size > 16 && data[0] == 'N' && data[1] == 'E' && data[2] == 'S' && data[3] == 0x1A) {
        uint8_t prg_banks = data[4];
        uint8_t chr_banks = data[5];
        
        cpu->prg_rom_size = prg_banks * 16384;
        cpu->chr_rom_size = chr_banks * 8192;
        cpu->is_nes_rom = 1;
        
        if (cpu->prg_rom_size > 262144) cpu->prg_rom_size = 262144;
        if (cpu->chr_rom_size > 262144) cpu->chr_rom_size = 262144;
        
        memcpy(cpu->prg_rom, data + 16, cpu->prg_rom_size);
        if (cpu->chr_rom_size > 0) {
            memcpy(cpu->chr_rom, data + 16 + cpu->prg_rom_size, cpu->chr_rom_size);
        }
        
        cpu->mapper_num = 1;
        cpu->mmc1_shift = 0x10;
        cpu->mmc1_ctrl = 0x0C;
        
        cpu->prg_bank0 = cpu->prg_rom;
        cpu->prg_bank1 = cpu->prg_rom + cpu->prg_rom_size - 16384;
        cpu->chr_bank0 = cpu->chr_rom;
        cpu->chr_bank1 = cpu->chr_rom + 4096;
        
        // Copy to standard ranges so vector lookups succeed in flat space
        memcpy(cpu->ram + 0x8000, cpu->prg_bank0, 16384);
        memcpy(cpu->ram + 0xC000, cpu->prg_bank1, 16384);
        
        uint16_t vector = cpu->ram[0xFFFC] | ((uint16_t)cpu->ram[0xFFFD] << 8);
        cpu->pc = vector;
    } else {
        cpu->is_nes_rom = 0;
        if (address >= 65536) return -2;
        size_t copy_len = size;
        if (address + size > 65536) {
            copy_len = 65536 - address;
        }
        memcpy(cpu->ram + address, data, copy_len);
        cpu->pc = (uint16_t)address;
    }
    
    return 0;
}

static inline void push_byte(MOS6502_CPU *cpu, uint8_t val) {
    cpu_write(cpu, 0x0100 + cpu->sp, val);
    cpu->sp--;
}

static inline uint8_t pop_byte(MOS6502_CPU *cpu) {
    cpu->sp++;
    return cpu_read(cpu, 0x0100 + cpu->sp);
}

int mos6502_step(void *context) {
    if (!context) return -1;
    MOS6502_CPU *cpu = (MOS6502_CPU*)context;
    
    if (cpu->halted) return 1;
    
    uint16_t instr_pc = cpu->pc;
    uint32_t prev_ticks = cpu->ticks;
    uint8_t op = cpu_read(cpu, cpu->pc++);
    cpu->ticks++;
    
    switch (op) {
        // --- Added standard 6502 Instructions ---
        case 0x94: // STY ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu_write(cpu, zp, cpu->y);
            }
            break;
        case 0x96: // STX ZeroPage, Y
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->y) & 0xFF;
                cpu_write(cpu, zp, cpu->x);
            }
            break;
        case 0x09: // ORA Immediate
            cpu->a |= cpu_read(cpu, cpu->pc++);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0x05: // ORA ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu->a |= cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x0D: // ORA Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a |= cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x15: // ORA ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu->a |= cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x1D: // ORA Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a |= cpu_read(cpu, (uint16_t)(addr + cpu->x));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x19: // ORA Absolute, Y
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a |= cpu_read(cpu, (uint16_t)(addr + cpu->y));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x25: // AND ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu->a &= cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x2D: // AND Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a &= cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x35: // AND ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu->a &= cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x3D: // AND Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a &= cpu_read(cpu, (uint16_t)(addr + cpu->x));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x39: // AND Absolute, Y
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a &= cpu_read(cpu, (uint16_t)(addr + cpu->y));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x49: // EOR Immediate
            cpu->a ^= cpu_read(cpu, cpu->pc++);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0x45: // EOR ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu->a ^= cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x4D: // EOR Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a ^= cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x55: // EOR ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu->a ^= cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x5D: // EOR Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a ^= cpu_read(cpu, (uint16_t)(addr + cpu->x));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x59: // EOR Absolute, Y
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a ^= cpu_read(cpu, (uint16_t)(addr + cpu->y));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x24: // BIT ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a & val);
                if (val & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N;
                if (val & 0x40) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
            }
            break;
        case 0x2C: // BIT Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->a & val);
                if (val & 0x80) cpu->p |= FLAG_N; else cpu->p &= ~FLAG_N;
                if (val & 0x40) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
            }
            break;
        case 0x2A: // ROL A
            {
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(cpu->a & 0x80);
                cpu->a = (cpu->a << 1) | old_c;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x26: // ROL ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x80);
                val = (val << 1) | old_c;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x2E: // ROL Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x80);
                val = (val << 1) | old_c;
                cpu_write(cpu, addr, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x36: // ROL ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x80);
                val = (val << 1) | old_c;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x3E: // ROL Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target = (uint16_t)(addr + cpu->x);
                uint8_t val = cpu_read(cpu, target);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x80);
                val = (val << 1) | old_c;
                cpu_write(cpu, target, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x6A: // ROR A
            {
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(cpu->a & 0x01);
                cpu->a = (cpu->a >> 1) | (old_c << 7);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x66: // ROR ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x01);
                val = (val >> 1) | (old_c << 7);
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x6E: // ROR Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x01);
                val = (val >> 1) | (old_c << 7);
                cpu_write(cpu, addr, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x76: // ROR ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x01);
                val = (val >> 1) | (old_c << 7);
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x7E: // ROR Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target = (uint16_t)(addr + cpu->x);
                uint8_t val = cpu_read(cpu, target);
                uint8_t old_c = GET_FLAG(FLAG_C);
                SET_FLAG_C(val & 0x01);
                val = (val >> 1) | (old_c << 7);
                cpu_write(cpu, target, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x06: // ASL ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                SET_FLAG_C(val & 0x80);
                val <<= 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x0E: // ASL Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                SET_FLAG_C(val & 0x80);
                val <<= 1;
                cpu_write(cpu, addr, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x16: // ASL ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp);
                SET_FLAG_C(val & 0x80);
                val <<= 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x1E: // ASL Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target = (uint16_t)(addr + cpu->x);
                uint8_t val = cpu_read(cpu, target);
                SET_FLAG_C(val & 0x80);
                val <<= 1;
                cpu_write(cpu, target, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x46: // LSR ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                SET_FLAG_C(val & 0x01);
                val >>= 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x4E: // LSR Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                SET_FLAG_C(val & 0x01);
                val >>= 1;
                cpu_write(cpu, addr, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x56: // LSR ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp);
                SET_FLAG_C(val & 0x01);
                val >>= 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x5E: // LSR Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target = (uint16_t)(addr + cpu->x);
                uint8_t val = cpu_read(cpu, target);
                SET_FLAG_C(val & 0x01);
                val >>= 1;
                cpu_write(cpu, target, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xEE: // INC Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr) + 1;
                cpu_write(cpu, addr, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xF6: // INC ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp) + 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xFE: // INC Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target = (uint16_t)(addr + cpu->x);
                uint8_t val = cpu_read(cpu, target) + 1;
                cpu_write(cpu, target, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xC6: // DEC ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp) - 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xCE: // DEC Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr) - 1;
                cpu_write(cpu, addr, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xD6: // DEC ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp) - 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xDE: // DEC Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target = (uint16_t)(addr + cpu->x);
                uint8_t val = cpu_read(cpu, target) - 1;
                cpu_write(cpu, target, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0xEC: // CPX Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint16_t diff = (uint16_t)cpu->x - val;
                SET_FLAG_C(cpu->x >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xCC: // CPY Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint16_t diff = (uint16_t)cpu->y - val;
                SET_FLAG_C(cpu->y >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xC4: // CPY ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint16_t diff = (uint16_t)cpu->y - val;
                SET_FLAG_C(cpu->y >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xE4: // CPX ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint16_t diff = (uint16_t)cpu->x - val;
                SET_FLAG_C(cpu->x >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xD5: // CMP ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = cpu_read(cpu, zp);
                uint16_t diff = (uint16_t)cpu->a - val;
                SET_FLAG_C(cpu->a >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xDD: // CMP Absolute, X
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, (uint16_t)(addr + cpu->x));
                uint16_t diff = (uint16_t)cpu->a - val;
                SET_FLAG_C(cpu->a >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xD9: // CMP Absolute, Y
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, (uint16_t)(addr + cpu->y));
                uint16_t diff = (uint16_t)cpu->a - val;
                SET_FLAG_C(cpu->a >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xC5: // CMP ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint16_t diff = (uint16_t)cpu->a - val;
                SET_FLAG_C(cpu->a >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xCD: // CMP Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint16_t diff = (uint16_t)cpu->a - val;
                SET_FLAG_C(cpu->a >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xB5: // LDA ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu->a = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xBD: // LDA Absolute, X
            {
                uint16_t base = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a = cpu_read(cpu, (uint16_t)(base + cpu->x));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xB9: // LDA Absolute, Y
            {
                uint16_t base = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a = cpu_read(cpu, (uint16_t)(base + cpu->y));
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x95: // STA ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu_write(cpu, zp, cpu->a);
            }
            break;
        case 0x9D: // STA Absolute, X
            {
                uint16_t base = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu_write(cpu, (uint16_t)(base + cpu->x), cpu->a);
            }
            break;
        case 0x99: // STA Absolute, Y
            {
                uint16_t base = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu_write(cpu, (uint16_t)(base + cpu->y), cpu->a);
            }
            break;
        case 0xB6: // LDX ZeroPage, Y
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->y) & 0xFF;
                cpu->x = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->x);
                SET_FLAG_N(cpu->x);
            }
            break;
        case 0xBE: // LDX Absolute, Y
            {
                uint16_t base = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->x = cpu_read(cpu, (uint16_t)(base + cpu->y));
                SET_FLAG_Z(cpu->x);
                SET_FLAG_N(cpu->x);
            }
            break;
        case 0xB4: // LDY ZeroPage, X
            {
                uint8_t zp = (cpu_read(cpu, cpu->pc++) + cpu->x) & 0xFF;
                cpu->y = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->y);
                SET_FLAG_N(cpu->y);
            }
            break;
        case 0xBC: // LDY Absolute, X
            {
                uint16_t base = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->y = cpu_read(cpu, (uint16_t)(base + cpu->x));
                SET_FLAG_Z(cpu->y);
                SET_FLAG_N(cpu->y);
            }
            break;
        case 0x6C: // JMP Indirect
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t target;
                if ((addr & 0x00FF) == 0x00FF) {
                    target = cpu_read(cpu, addr) | ((uint16_t)cpu_read(cpu, addr & 0xFF00) << 8);
                } else {
                    target = cpu_read(cpu, addr) | ((uint16_t)cpu_read(cpu, addr + 1) << 8);
                }
                cpu->pc = target;
            }
            break;
            
        case 0x0A: // ASL A
            SET_FLAG_C(cpu->a & 0x80);
            cpu->a <<= 1;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0x4A: // LSR A
            SET_FLAG_C(cpu->a & 0x01);
            cpu->a >>= 1;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0xE6: // INC zp
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp) + 1;
                cpu_write(cpu, zp, val);
                SET_FLAG_Z(val);
                SET_FLAG_N(val);
            }
            break;
        case 0x29: // AND immediate
            cpu->a &= cpu_read(cpu, cpu->pc++);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
            
        // --- NOP ---
        case 0xEA:
            break;
            
        // --- BRK ---
        case 0x00:
            if (cpu->is_nes_rom) {
                // NES games shouldn't halt on BRK, just a standard software break vector lookup
                uint16_t dest = cpu_read(cpu, 0xFFFE) | ((uint16_t)cpu_read(cpu, 0xFFFF) << 8);
                push_byte(cpu, (cpu->pc >> 8) & 0xFF);
                push_byte(cpu, cpu->pc & 0xFF);
                push_byte(cpu, cpu->p | FLAG_B);
                cpu->pc = dest;
            } else {
                cpu->halted = 1;
                return 1;
            }
            break;
            
        // --- LDA ---
        case 0xA9: // LDA Immediate
            cpu->a = cpu_read(cpu, cpu->pc++);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0xA5: // LDA ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu->a = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xAD: // LDA Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->a = cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xB1: // LDA (Indirect), Y
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint16_t base = cpu_read(cpu, zp) | ((uint16_t)cpu_read(cpu, (zp + 1) & 0xFF) << 8);
                uint16_t addr = base + cpu->y;
                cpu->a = cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
            
        // --- LDX ---
        case 0xA2: // LDX Immediate
            cpu->x = cpu_read(cpu, cpu->pc++);
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case 0xA6: // LDX ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu->x = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->x);
                SET_FLAG_N(cpu->x);
            }
            break;
        case 0xAE: // LDX Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->x = cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->x);
                SET_FLAG_N(cpu->x);
            }
            break;
            
        // --- LDY ---
        case 0xA0: // LDY Immediate
            cpu->y = cpu_read(cpu, cpu->pc++);
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case 0xA4: // LDY ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu->y = cpu_read(cpu, zp);
                SET_FLAG_Z(cpu->y);
                SET_FLAG_N(cpu->y);
            }
            break;
        case 0xAC: // LDY Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu->y = cpu_read(cpu, addr);
                SET_FLAG_Z(cpu->y);
                SET_FLAG_N(cpu->y);
            }
            break;
            
        // --- STA ---
        case 0x85: // STA ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu_write(cpu, zp, cpu->a);
            }
            break;
        case 0x8D: // STA Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu_write(cpu, addr, cpu->a);
            }
            break;
        case 0x91: // STA (Indirect), Y
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint16_t base = cpu_read(cpu, zp) | ((uint16_t)cpu_read(cpu, (zp + 1) & 0xFF) << 8);
                uint16_t addr = base + cpu->y;
                cpu_write(cpu, addr, cpu->a);
            }
            break;
            
        // --- STX ---
        case 0x86: // STX ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu_write(cpu, zp, cpu->x);
            }
            break;
        case 0x8E: // STX Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu_write(cpu, addr, cpu->x);
            }
            break;
            
        // --- STY ---
        case 0x84: // STY ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                cpu_write(cpu, zp, cpu->y);
            }
            break;
        case 0x8C: // STY Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                cpu_write(cpu, addr, cpu->y);
            }
            break;
            
        // --- Transfers ---
        case 0xAA: // TAX
            cpu->x = cpu->a;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case 0x8A: // TXA
            cpu->a = cpu->x;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0xA8: // TAY
            cpu->y = cpu->a;
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case 0x98: // TYA
            cpu->a = cpu->y;
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0x9A: // TXS
            cpu->sp = cpu->x;
            break;
        case 0xBA: // TSX
            cpu->x = cpu->sp;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
            
        // --- Increments/Decrements ---
        case 0xE8: // INX
            cpu->x++;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case 0xC8: // INY
            cpu->y++;
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
        case 0xCA: // DEX
            cpu->x--;
            SET_FLAG_Z(cpu->x);
            SET_FLAG_N(cpu->x);
            break;
        case 0x88: // DEY
            cpu->y--;
            SET_FLAG_Z(cpu->y);
            SET_FLAG_N(cpu->y);
            break;
            
        // --- Comparisons ---
        case 0xC9: // CMP Immediate
            {
                uint8_t val = cpu_read(cpu, cpu->pc++);
                uint16_t diff = (uint16_t)cpu->a - val;
                SET_FLAG_C(cpu->a >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xE0: // CPX Immediate
            {
                uint8_t val = cpu_read(cpu, cpu->pc++);
                uint16_t diff = (uint16_t)cpu->x - val;
                SET_FLAG_C(cpu->x >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
        case 0xC0: // CPY Immediate
            {
                uint8_t val = cpu_read(cpu, cpu->pc++);
                uint16_t diff = (uint16_t)cpu->y - val;
                SET_FLAG_C(cpu->y >= val);
                SET_FLAG_Z(diff & 0xFF);
                SET_FLAG_N(diff & 0xFF);
            }
            break;
            
        // --- Jumps & Branches ---
        case 0x4C: // JMP Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc = addr;
            }
            break;
        case 0xD0: // BNE
            {
                int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
                if (!GET_FLAG(FLAG_Z)) {
                    cpu->pc = (uint16_t)(cpu->pc + offset);
                }
            }
            break;
        case 0xF0: // BEQ
            {
                int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
                if (GET_FLAG(FLAG_Z)) {
                    cpu->pc = (uint16_t)(cpu->pc + offset);
                }
            }
            break;
        case 0x90: // BCC
            {
                int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
                if (!GET_FLAG(FLAG_C)) {
                    cpu->pc = (uint16_t)(cpu->pc + offset);
                }
            }
            break;
        case 0xB0: // BCS
            {
                int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
                if (GET_FLAG(FLAG_C)) {
                    cpu->pc = (uint16_t)(cpu->pc + offset);
                }
            }
            break;
        case 0x10: // BPL
            {
                int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
                if (!GET_FLAG(FLAG_N)) {
                    cpu->pc = (uint16_t)(cpu->pc + offset);
                }
            }
            break;
        case 0x30: // BMI
            {
                int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
                if (GET_FLAG(FLAG_N)) {
                    cpu->pc = (uint16_t)(cpu->pc + offset);
                }
            }
            break;
            
        // --- Subroutines & Stack ---
        case 0x20: // JSR absolute
            {
                uint16_t dest = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t ret_pc = cpu->pc - 1;
                push_byte(cpu, (ret_pc >> 8) & 0xFF);
                push_byte(cpu, ret_pc & 0xFF);
                cpu->pc = dest;
            }
            break;
        case 0x40: // RTI
            {
                cpu->p = pop_byte(cpu) | FLAG_U;
                uint8_t low = pop_byte(cpu);
                uint8_t high = pop_byte(cpu);
                cpu->pc = ((uint16_t)high << 8) | low;
            }
            break;
        case 0x60: // RTS
            {
                uint8_t low = pop_byte(cpu);
                uint8_t high = pop_byte(cpu);
                cpu->pc = (((uint16_t)high << 8) | low) + 1;
            }
            break;
        case 0x48: // PHA
            push_byte(cpu, cpu->a);
            break;
        case 0x68: // PLA
            cpu->a = pop_byte(cpu);
            SET_FLAG_Z(cpu->a);
            SET_FLAG_N(cpu->a);
            break;
        case 0x08: // PHP
            push_byte(cpu, cpu->p | FLAG_B);
            break;
        case 0x28: // PLP
            cpu->p = pop_byte(cpu) | FLAG_U;
            break;
            
        // --- Add/Subtract ---
        case 0x69: // ADC Immediate
            {
                uint8_t val = cpu_read(cpu, cpu->pc++);
                uint16_t sum = (uint16_t)cpu->a + val + GET_FLAG(FLAG_C);
                SET_FLAG_C(sum > 0xFF);
                uint8_t v = (!((cpu->a ^ val) & 0x80) && ((cpu->a ^ sum) & 0x80)) ? 1 : 0;
                if (v) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
                cpu->a = sum & 0xFF;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x65: // ADC ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint16_t sum = (uint16_t)cpu->a + val + GET_FLAG(FLAG_C);
                SET_FLAG_C(sum > 0xFF);
                uint8_t v = (!((cpu->a ^ val) & 0x80) && ((cpu->a ^ sum) & 0x80)) ? 1 : 0;
                if (v) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
                cpu->a = sum & 0xFF;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0x6D: // ADC Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint16_t sum = (uint16_t)cpu->a + val + GET_FLAG(FLAG_C);
                SET_FLAG_C(sum > 0xFF);
                uint8_t v = (!((cpu->a ^ val) & 0x80) && ((cpu->a ^ sum) & 0x80)) ? 1 : 0;
                if (v) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
                cpu->a = sum & 0xFF;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xE9: // SBC Immediate
            {
                uint8_t val = cpu_read(cpu, cpu->pc++);
                uint16_t val16 = (uint16_t)val ^ 0xFF;
                uint16_t sum = (uint16_t)cpu->a + val16 + GET_FLAG(FLAG_C);
                SET_FLAG_C(sum > 0xFF);
                uint8_t v = (!((cpu->a ^ val16) & 0x80) && ((cpu->a ^ sum) & 0x80)) ? 1 : 0;
                if (v) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
                cpu->a = sum & 0xFF;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xE5: // SBC ZeroPage
            {
                uint8_t zp = cpu_read(cpu, cpu->pc++);
                uint8_t val = cpu_read(cpu, zp);
                uint16_t val16 = (uint16_t)val ^ 0xFF;
                uint16_t sum = (uint16_t)cpu->a + val16 + GET_FLAG(FLAG_C);
                SET_FLAG_C(sum > 0xFF);
                uint8_t v = (!((cpu->a ^ val16) & 0x80) && ((cpu->a ^ sum) & 0x80)) ? 1 : 0;
                if (v) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
                cpu->a = sum & 0xFF;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
        case 0xED: // SBC Absolute
            {
                uint16_t addr = cpu_read(cpu, cpu->pc) | ((uint16_t)cpu_read(cpu, cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint8_t val = cpu_read(cpu, addr);
                uint16_t val16 = (uint16_t)val ^ 0xFF;
                uint16_t sum = (uint16_t)cpu->a + val16 + GET_FLAG(FLAG_C);
                SET_FLAG_C(sum > 0xFF);
                uint8_t v = (!((cpu->a ^ val16) & 0x80) && ((cpu->a ^ sum) & 0x80)) ? 1 : 0;
                if (v) cpu->p |= FLAG_V; else cpu->p &= ~FLAG_V;
                cpu->a = sum & 0xFF;
                SET_FLAG_Z(cpu->a);
                SET_FLAG_N(cpu->a);
            }
            break;
            
        // --- Clear/Set Flags ---
        case 0x18: // CLC
            cpu->p &= ~FLAG_C;
            break;
        case 0x38: // SEC
            cpu->p |= FLAG_C;
            break;
        case 0x58: // CLI
            cpu->p &= ~FLAG_I;
            break;
        case 0x78: // SEI
            cpu->p |= FLAG_I;
            break;
        case 0xD8: // CLD
            cpu->p &= ~FLAG_D;
            break;
        case 0xF8: // SED
            cpu->p |= FLAG_D;
            break;
            
        default:
            cpu->halted = 1;
            return 1;
    }
    
    // Account for correct execution cycle increments
    uint32_t elapsed_ticks = cpu->ticks - prev_ticks;
    
    // --- NES Interrupt & Timing Frame Sync ---
    if (cpu->is_nes_rom) {
        cpu->frame_cycles += elapsed_ticks;
        // NES triggers NMI every 29780 ticks (corresponding to 60 FPS)
        if (cpu->frame_cycles >= 29780) {
            cpu->frame_cycles -= 29780;
            // Set VBLANK active (bit 7 of PPUSTATUS at 0x2002)
            cpu->ppu_status |= 0x80;
            
            // If NMIs are enabled (bit 7 of PPUCTRL at 0x2000)
            if (cpu->ppu_ctrl & 0x80) {
                // Execute NMI interrupt
                push_byte(cpu, (cpu->pc >> 8) & 0xFF);
                push_byte(cpu, cpu->pc & 0xFF);
                push_byte(cpu, cpu->p & ~FLAG_B); // Push status without B flag
                
                // Get vector at $FFFA-$FFFB
                uint16_t nmi_vector = cpu_read(cpu, 0xFFFA) | ((uint16_t)cpu_read(cpu, 0xFFFB) << 8);
                cpu->pc = nmi_vector;
                
                cpu->p |= FLAG_I; // Disable normal IRQs
            }
        }
    }
    
    if (cpu->pc == instr_pc) {
        cpu->halted = 1;
        return 1;
    }
    
    return 0;
}

void mos6502_print_state(void *context) {
    if (!context) return;
    MOS6502_CPU *cpu = (MOS6502_CPU*)context;
    
    printf("MOS 6502 State (Ticks: %u):\n", cpu->ticks);
    printf("  PC: 0x%04X  SP: 0x%02X  Halted: %s\n", cpu->pc, cpu->sp, cpu->halted ? "Yes" : "No");
    printf("  Registers: A=0x%02X  X=0x%02X  Y=0x%02X\n", cpu->a, cpu->x, cpu->y);
    printf("  Flags: N=%d  V=%d  U=%d  B=%d  D=%d  I=%d  Z=%d  C=%d\n",
           GET_FLAG(FLAG_N), GET_FLAG(FLAG_V), GET_FLAG(FLAG_U), GET_FLAG(FLAG_B),
           GET_FLAG(FLAG_D), GET_FLAG(FLAG_I), GET_FLAG(FLAG_Z), GET_FLAG(FLAG_C));
    if (cpu->is_nes_rom) {
        printf("  NES Core active: PRG-ROM=%u KB  CHR-ROM=%u KB\n",
               cpu->prg_rom_size / 1024, cpu->chr_rom_size / 1024);
        printf("  PPU registers: CTRL=0x%02X  MASK=0x%02X  STATUS=0x%02X  ADDR=0x%04X\n",
               cpu->ppu_ctrl, cpu->ppu_mask, cpu->ppu_status, cpu->ppu_addr);
        
        int non_zero_vram = 0;
        for (int i = 0; i < 2048; ++i) {
            if (cpu->ppu_vram[i] != 0) non_zero_vram++;
        }
        int non_zero_pal = 0;
        for (int i = 0; i < 32; ++i) {
            if (cpu->ppu_palette[i] != 0) non_zero_pal++;
        }
        printf("  DEBUG: Non-zero VRAM: %d / 2048  Non-zero Palettes: %d / 32\n",
               non_zero_vram, non_zero_pal);
    }
}

void mos6502_get_disassembly(void *context, char *buf, size_t buf_len) {
    if (!context || !buf || buf_len == 0) return;
    MOS6502_CPU *cpu = (MOS6502_CPU*)context;
    
    uint8_t op = cpu_read(cpu, cpu->pc);
    
    switch (op) {
        case 0x0A: snprintf(buf, buf_len, "asl   a"); break;
        case 0x4A: snprintf(buf, buf_len, "lsr   a"); break;
        case 0xE6: snprintf(buf, buf_len, "inc   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0x29: snprintf(buf, buf_len, "and   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xEA: snprintf(buf, buf_len, "nop"); break;
        case 0x00: snprintf(buf, buf_len, "brk"); break;
        
        case 0xA9: snprintf(buf, buf_len, "lda   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xA5: snprintf(buf, buf_len, "lda   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xAD: snprintf(buf, buf_len, "lda   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        case 0xB1: snprintf(buf, buf_len, "lda   ($0x%02X),Y", cpu_read(cpu, cpu->pc + 1)); break;
        
        case 0xA2: snprintf(buf, buf_len, "ldx   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xA6: snprintf(buf, buf_len, "ldx   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xAE: snprintf(buf, buf_len, "ldx   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0xA0: snprintf(buf, buf_len, "ldy   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xA4: snprintf(buf, buf_len, "ldy   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xAC: snprintf(buf, buf_len, "ldy   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0x85: snprintf(buf, buf_len, "sta   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0x8D: snprintf(buf, buf_len, "sta   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        case 0x91: snprintf(buf, buf_len, "sta   ($0x%02X),Y", cpu_read(cpu, cpu->pc + 1)); break;
        
        case 0x86: snprintf(buf, buf_len, "stx   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0x8E: snprintf(buf, buf_len, "stx   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0x84: snprintf(buf, buf_len, "sty   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0x8C: snprintf(buf, buf_len, "sty   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0xAA: snprintf(buf, buf_len, "tax"); break;
        case 0x8A: snprintf(buf, buf_len, "txa"); break;
        case 0xA8: snprintf(buf, buf_len, "tay"); break;
        case 0x98: snprintf(buf, buf_len, "tya"); break;
        case 0x9A: snprintf(buf, buf_len, "txs"); break;
        case 0xBA: snprintf(buf, buf_len, "tsx"); break;
        
        case 0xE8: snprintf(buf, buf_len, "inx"); break;
        case 0xC8: snprintf(buf, buf_len, "iny"); break;
        case 0xCA: snprintf(buf, buf_len, "dex"); break;
        case 0x88: snprintf(buf, buf_len, "dey"); break;
        
        case 0xC9: snprintf(buf, buf_len, "cmp   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xC5: snprintf(buf, buf_len, "cmp   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xCD: snprintf(buf, buf_len, "cmp   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0xE0: snprintf(buf, buf_len, "cpx   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xC0: snprintf(buf, buf_len, "cpy   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        
        case 0x4C: snprintf(buf, buf_len, "jmp   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0xD0: snprintf(buf, buf_len, "bne   %+d", (int8_t)cpu_read(cpu, cpu->pc + 1)); break;
        case 0xF0: snprintf(buf, buf_len, "beq   %+d", (int8_t)cpu_read(cpu, cpu->pc + 1)); break;
        case 0x90: snprintf(buf, buf_len, "bcc   %+d", (int8_t)cpu_read(cpu, cpu->pc + 1)); break;
        case 0xB0: snprintf(buf, buf_len, "bcs   %+d", (int8_t)cpu_read(cpu, cpu->pc + 1)); break;
        case 0x10: snprintf(buf, buf_len, "bpl   %+d", (int8_t)cpu_read(cpu, cpu->pc + 1)); break;
        case 0x30: snprintf(buf, buf_len, "bmi   %+d", (int8_t)cpu_read(cpu, cpu->pc + 1)); break;
        
        case 0x20: snprintf(buf, buf_len, "jsr   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        case 0x60: snprintf(buf, buf_len, "rts"); break;
        
        case 0x48: snprintf(buf, buf_len, "pha"); break;
        case 0x68: snprintf(buf, buf_len, "pla"); break;
        case 0x08: snprintf(buf, buf_len, "php"); break;
        case 0x28: snprintf(buf, buf_len, "plp"); break;
        
        case 0x69: snprintf(buf, buf_len, "adc   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0x65: snprintf(buf, buf_len, "adc   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0x6D: snprintf(buf, buf_len, "adc   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        case 0xE9: snprintf(buf, buf_len, "sbc   #$0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xE5: snprintf(buf, buf_len, "sbc   $0x%02X", cpu_read(cpu, cpu->pc + 1)); break;
        case 0xED: snprintf(buf, buf_len, "sbc   $0x%04X", cpu_read(cpu, cpu->pc + 1) | ((uint16_t)cpu_read(cpu, cpu->pc + 2) << 8)); break;
        
        case 0x18: snprintf(buf, buf_len, "clc"); break;
        case 0x38: snprintf(buf, buf_len, "sec"); break;
        case 0x58: snprintf(buf, buf_len, "cli"); break;
        case 0x78: snprintf(buf, buf_len, "sei"); break;
        case 0xD8: snprintf(buf, buf_len, "cld"); break;
        case 0xF8: snprintf(buf, buf_len, "sed"); break;
        
        default:
            snprintf(buf, buf_len, "unknown (0x%02X)", op);
            break;
    }
}

// Full background & sprite rendering pipeline for GDI window output
void mos6502_render_screen(void *context, uint32_t *display_buffer) {
    if (!context || !display_buffer) return;
    MOS6502_CPU *cpu = (MOS6502_CPU*)context;
    
    if (cpu->is_nes_rom) {
        // --- 1. RENDER BACKGROUNDS ---
        // NES Nametable is 32 x 30 tiles. Tile size is 8x8 pixels. Screen is 256 x 240.
        // Background Pattern Table base (0 or 0x1000 in CHR-ROM)
        uint16_t bg_pattern_base = (cpu->ppu_ctrl & 0x10) ? 0x1000 : 0x0000;
        
        for (int ty = 0; ty < 30; ++ty) {
            for (int tx = 0; tx < 32; ++tx) {
                // Address of tile index in Nametable 0 (Tetris uses vertical mirroring NT0 at 0x2000)
                uint16_t nt_addr = (uint16_t)(0x2000 + ty * 32 + tx);
                uint8_t tile_idx = ppu_read(cpu, nt_addr);
                
                // Address of Attribute Table byte for 4x4 tile blocks
                uint16_t attr_addr = (uint16_t)(0x23C0 + (ty / 4) * 8 + (tx / 4));
                uint8_t attr_byte = ppu_read(cpu, attr_addr);
                uint8_t shift = ((tx & 2) ? 2 : 0) + ((ty & 2) ? 4 : 0);
                uint8_t palette_idx = (attr_byte >> shift) & 0x03;
                
                // Read 8x8 tile pattern (2 bytes per row * 8 rows = 16 bytes per tile)
                uint16_t tile_pattern_addr = (uint16_t)(bg_pattern_base + tile_idx * 16);
                for (int py = 0; py < 8; ++py) {
                    uint8_t low = ppu_read(cpu, (uint16_t)(tile_pattern_addr + py));
                    uint8_t high = ppu_read(cpu, (uint16_t)(tile_pattern_addr + py + 8));
                    
                    int screen_y = ty * 8 + py;
                    for (int px = 0; px < 8; ++px) {
                        uint8_t bit0 = (low >> (7 - px)) & 1;
                        uint8_t bit1 = (high >> (7 - px)) & 1;
                        uint8_t color_idx = bit0 | (bit1 << 1);
                        
                        // Look up color in background palette
                        uint16_t pal_addr = 0x3F00 + (palette_idx * 4) + color_idx;
                        if (color_idx == 0) pal_addr = 0x3F00; // background transparency transparent
                        
                        uint8_t system_color = ppu_read(cpu, pal_addr) & 0x3F;
                        
                        int screen_x = tx * 8 + px;
                        display_buffer[screen_y * 256 + screen_x] = g_nes_palette[system_color];
                    }
                }
            }
        }
        
        // --- 2. RENDER SPRITES ---
        uint16_t sprite_pattern_base = (cpu->ppu_ctrl & 0x08) ? 0x1000 : 0x0000;
        // Loop from sprite 63 down to 0 to respect sprite priority
        for (int i = 63; i >= 0; --i) {
            uint8_t sy = cpu->oam[i * 4];
            uint8_t tile_idx = cpu->oam[i * 4 + 1];
            uint8_t attr = cpu->oam[i * 4 + 2];
            uint8_t sx = cpu->oam[i * 4 + 3];
            
            // NES sprite Y coordinate is delayed by 1 scanline
            sy += 1;
            if (sy >= 240) continue;
            
            uint8_t palette_idx = (attr & 0x03) + 4; // sprite palettes start at 0x3F10
            uint8_t flip_h = (attr & 0x40) ? 1 : 0;
            uint8_t flip_v = (attr & 0x80) ? 1 : 0;
            
            uint16_t tile_pattern_addr = (uint16_t)(sprite_pattern_base + tile_idx * 16);
            
            for (int py = 0; py < 8; ++py) {
                int row = flip_v ? (7 - py) : py;
                uint8_t low = ppu_read(cpu, (uint16_t)(tile_pattern_addr + row));
                uint8_t high = ppu_read(cpu, (uint16_t)(tile_pattern_addr + row + 8));
                
                int screen_y = sy + py;
                if (screen_y >= 240) continue;
                
                for (int px = 0; px < 8; ++px) {
                    int col = flip_h ? px : (7 - px);
                    uint8_t bit0 = (low >> col) & 1;
                    uint8_t bit1 = (high >> col) & 1;
                    uint8_t color_idx = bit0 | (bit1 << 1);
                    
                    if (color_idx == 0) continue; // Transparency
                    
                    uint16_t pal_addr = 0x3F00 + (palette_idx * 4) + color_idx;
                    uint8_t system_color = ppu_read(cpu, pal_addr) & 0x3F;
                    
                    int screen_x = sx + px;
                    if (screen_x >= 256) continue;
                    
                    display_buffer[screen_y * 256 + screen_x] = g_nes_palette[system_color];
                }
            }
        }
        
    } else {
        // Fallback: translate 64x64 raw framebuffer RAM (0x2000-0x2FFF) to 256x240 screen
        for (int y = 0; y < 240; ++y) {
            int src_y = y * 64 / 240;
            for (int x = 0; x < 256; ++x) {
                int src_x = x * 64 / 256;
                uint8_t color_idx = cpu->ram[0x2000 + src_y * 64 + src_x] & 0x0F;
                display_buffer[y * 256 + x] = g_c64_palette[color_idx];
            }
        }
    }
}
