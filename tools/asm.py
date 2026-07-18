#!/usr/bin/env python3
import sys
import os
import re

# Intel 4004 Instruction Size and Opcode encoding rules
# 2-Byte Instructions: JCN, FIM, JUN, JMS, ISZ
# 1-Byte Instructions: all others

CONDITIONS = {
    'T': 1,   # Test signal is 0
    'C': 2,   # Carry is 1
    'Z': 4,   # Accumulator is 0
    'IT': 9,  # Invert Test (Test signal is 1)
    'NC': 10, # Invert Carry (Carry is 0)
    'NZ': 12, # Invert Zero (Accumulator is non-zero)
}

def parse_int(val_str):
    val_str = val_str.strip()
    if val_str.lower().startswith('0x'):
        return int(val_str, 16)
    return int(val_str, 10)

def parse_reg(reg_str):
    # e.g., "R15" or "15"
    reg_str = reg_str.strip().upper()
    if reg_str.startswith('R'):
        return int(reg_str[1:])
    return int(reg_str)

def parse_pair(pair_str):
    # e.g., "P7" or "7"
    pair_str = pair_str.strip().upper()
    if pair_str.startswith('P'):
        return int(pair_str[1:])
    return int(pair_str)

def parse_cond(cond_str):
    cond_str = cond_str.strip().upper()
    if cond_str in CONDITIONS:
        return CONDITIONS[cond_str]
    return parse_int(cond_str)

def print_help():
    help_text = """
Intel 4004 Assembler Tool
Usage:
    python tools/asm.py <source.asm> <output.bin>

Syntax rules:
    - Labels must end with a colon (e.g. "LOOP:")
    - Comments start with a semicolon (e.g. "; this is a comment")
    - Operands can be decimal or hex (prefixed with 0x)
    - Registers are prefixed with R (e.g., R0 - R15)
    - Register pairs are prefixed with P (e.g., P0 - P7)

Supported Mnemonics & Syntax:
    NOP
    LDM <imm4>                (Load Immediate)
    LD  <reg>                 (Load Register to Acc)
    XCH <reg>                 (Exchange Acc and Register)
    ADD <reg>                 (Add Register to Acc)
    SUB <reg>                 (Subtract Register from Acc)
    INC <reg>                 (Increment Register)
    FIM <pair>, <imm8>        (Fetch Immediate to Pair)
    SRC <pair>                (Send Register Control)
    FIN <pair>                (Fetch Indirect from ROM)
    JIN <pair>                (Jump Indirect)
    JUN <addr12>              (Jump Unconditional to label or address)
    JMS <addr12>              (Jump to Subroutine)
    JCN <cond>, <addr8>       (Jump Conditional on Z, C, T, NZ, NC, IT)
    ISZ <reg>, <addr8>        (Increment register and jump if non-zero)
    BBL <imm4>                (Branch Back and Load Immediate)
    WRM                       (Write RAM Character)
    WMP                       (Write RAM Output Port)
    WR0, WR1, WR2, WR3        (Write RAM Status Character 0-3)
    SBM                       (Subtract RAM from Acc with Borrow)
    RDM                       (Read RAM Character to Acc)
    RDR                       (Read ROM Port to Acc)
    ADM                       (Add RAM to Acc with Carry)
    RD0, RD1, RD2, RD3        (Read RAM Status Character 0-3 to Acc)
    CLB                       (Clear Acc and Carry)
    CLC                       (Clear Carry)
    IAC                       (Increment Acc)
    CMC                       (Complement Carry)
    CMA                       (Complement Acc)
    RAL                       (Rotate Acc Left through Carry)
    RAR                       (Rotate Acc Right through Carry)
    TCC                       (Transmit Carry and Clear)
    DAC                       (Decrement Acc)
    TCS                       (Transfer Carry Subtract)
    STC                       (Set Carry)
    DAA                       (Decimal Adjust Acc)
    KBP                       (Keyboard Process)
    DCL                       (Designate Command Line)
"""
    print(help_text)

class Instruction:
    def __init__(self, mnemonic, args, line_num, original_line):
        self.mnemonic = mnemonic.upper()
        self.args = args
        self.line_num = line_num
        self.original_line = original_line
        self.size = self._determine_size()
        
    def _determine_size(self):
        two_byte_mnemonics = {'JCN', 'FIM', 'JUN', 'JMS', 'ISZ'}
        return 2 if self.mnemonic in two_byte_mnemonics else 1

    def encode(self, pc, labels):
        m = self.mnemonic
        args = self.args
        
        # 1-byte simple instructions
        no_arg_ops = {
            'NOP': 0x00, 'WRM': 0xE0, 'WMP': 0xE1,
            'WR0': 0xE4, 'WR1': 0xE5, 'WR2': 0xE6, 'WR3': 0xE7,
            'SBM': 0xE8, 'RDM': 0xE9, 'RDR': 0xEA, 'ADM': 0xEB,
            'RD0': 0xEC, 'RD1': 0xED, 'RD2': 0xEE, 'RD3': 0xEF,
            'CLB': 0xF0, 'CLC': 0xF1, 'IAC': 0xF2, 'CMC': 0xF3,
            'CMA': 0xF4, 'RAL': 0xF5, 'RAR': 0xF6, 'TCC': 0xF7,
            'DAC': 0xF8, 'TCS': 0xF9, 'STC': 0xFA, 'DAA': 0xFB,
            'KBP': 0xFC, 'DCL': 0xFD
        }
        
        if m in no_arg_ops:
            return [no_arg_ops[m]]
            
        # 1-byte register instructions
        reg_ops = {
            'INC': 0x60, 'ADD': 0x80, 'SUB': 0x90, 'LD': 0xA0, 'XCH': 0xB0
        }
        if m in reg_ops:
            reg = parse_reg(args[0])
            return [reg_ops[m] | (reg & 0x0F)]
            
        # 1-byte immediate / pair instructions
        if m == 'LDM':
            imm = parse_int(args[0])
            return [0xD0 | (imm & 0x0F)]
        if m == 'BBL':
            imm = parse_int(args[0])
            return [0xC0 | (imm & 0x0F)]
        if m == 'SRC':
            pair = parse_pair(args[0])
            return [0x21 | ((pair & 7) << 1)]
        if m == 'FIN':
            pair = parse_pair(args[0])
            return [0x30 | ((pair & 7) << 1)]
        if m == 'JIN':
            pair = parse_pair(args[0])
            return [0x31 | ((pair & 7) << 1)]
            
        # 2-byte instructions
        if m == 'FIM':
            pair = parse_pair(args[0])
            imm = parse_int(args[1])
            return [0x20 | ((pair & 7) << 1), imm & 0xFF]
            
        if m == 'JUN':
            target = args[0]
            addr = labels[target] if target in labels else parse_int(target)
            return [0x40 | ((addr >> 8) & 0x0F), addr & 0xFF]
            
        if m == 'JMS':
            target = args[0]
            addr = labels[target] if target in labels else parse_int(target)
            return [0x50 | ((addr >> 8) & 0x0F), addr & 0xFF]
            
        if m == 'JCN':
            cond = parse_cond(args[0])
            target = args[1]
            addr = labels[target] if target in labels else parse_int(target)
            # JCN address is 8-bit page offset, warn if it crosses page boundary
            if (pc & 0xF00) != (addr & 0xF00):
                print(f"Warning Line {self.line_num}: JCN target 0x{addr:03X} is on a different ROM page from current PC 0x{pc:03X}!")
            return [0x10 | (cond & 0x0F), addr & 0xFF]
            
        if m == 'ISZ':
            reg = parse_reg(args[0])
            target = args[1]
            addr = labels[target] if target in labels else parse_int(target)
            if (pc & 0xF00) != (addr & 0xF00):
                print(f"Warning Line {self.line_num}: ISZ target 0x{addr:03X} is on a different ROM page from current PC 0x{pc:03X}!")
            return [0x70 | (reg & 0x0F), addr & 0xFF]
            
        raise ValueError(f"Unknown instruction '{m}'")

def assemble(source_path):
    with open(source_path, 'r') as f:
        lines = f.readlines()
        
    labels = {}
    instructions = []
    current_address = 0
    
    # Pass 1: Parse syntax, calculate addresses, record labels
    for idx, line in enumerate(lines):
        line_num = idx + 1
        clean_line = line.split(';')[0].strip() # strip comments
        if not clean_line:
            continue
            
        # Check if line is a label
        label_match = re.match(r'^([a-zA-Z_][a-zA-Z0-9_]*)\s*:$', clean_line)
        if label_match:
            label_name = label_match.group(1)
            if label_name in labels:
                raise ValueError(f"Line {line_num}: Duplicate label '{label_name}'")
            labels[label_name] = current_address
            continue
            
        # Parse instruction
        parts = re.split(r'\s+', clean_line, maxsplit=1)
        mnemonic = parts[0]
        args = []
        if len(parts) > 1:
            # split args by comma
            args = [a.strip() for a in parts[1].split(',') if a.strip()]
            
        try:
            instr = Instruction(mnemonic, args, line_num, line.strip())
            instructions.append((current_address, instr))
            current_address += instr.size
        except Exception as e:
            print(f"Error parsing line {line_num}: '{line.strip()}'")
            print(e)
            sys.exit(1)
            
    # Pass 2: Resolve labels and encode opcodes
    binary_bytes = bytearray()
    for pc, instr in instructions:
        try:
            encoded_bytes = instr.encode(pc, labels)
            binary_bytes.extend(encoded_bytes)
        except Exception as e:
            print(f"Assembly Error on line {instr.line_num}: '{instr.original_line}'")
            print(e)
            sys.exit(1)
            
    return binary_bytes

def main():
    if len(sys.argv) < 3:
        print_help()
        sys.exit(1)
        
    source = sys.argv[1]
    output = sys.argv[2]
    
    if not os.path.exists(source):
        print(f"Error: Source file '{source}' does not exist.")
        sys.exit(1)
        
    print(f"Assembling '{source}'...")
    try:
        data = assemble(source)
        with open(output, 'wb') as f:
            f.write(data)
        print(f"Successfully assembled: {output} ({len(data)} bytes)")
    except Exception as e:
        print("Assembly failed:")
        print(e)
        sys.exit(1)

if __name__ == '__main__':
    main()
