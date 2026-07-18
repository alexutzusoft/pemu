# Pemu

Pemu is a simple, modular processor emulator CLI written in C. It provides a generic interface for loading and simulating different processor architectures.

## Registered Processors

- `dummy`: A skeleton mock processor for bare-bones testing.
- `i4004`: A detailed software emulator for the historical Intel 4004 4-bit microprocessor.
- `rv32i`: A detailed software emulator for the 32-bit RISC-V Base Integer Instruction Set.
- `i8008`: A detailed software emulator for the historical Intel 8008 8-bit microprocessor.
- `i8080`: A detailed software emulator for the Intel 8080 8-bit microprocessor (full 64 KB address space, 16-bit stack pointer).
- `atmega328p`: A full-on AVR microcontroller (Arduino Nano) emulator with UART Serial output and Timer0 interrupts.
- `mos6502`: An 8-bit MOS Technology 6502 CPU core and NES console emulator with native Win32/SDL2 graphics and keyboard input mapping.

## Getting Started

### 1. Requirements
- **C Compiler**: MSVC compiler (`cl.exe`) set up in your path.
- **Build Tool**: NMake (`nmake.exe`).
- **Python**: Python 3 (only required to use the optional assembler tool).

### 2. Building Pemu
Run the build command from the Developer Command Prompt:
```cmd
nmake
```
This compiles the code and places the executable inside the `build/` directory (`build/pemu.exe`).

To clean the build directory:
```cmd
nmake clean
```

---

## Usage CLI Flags

Run `build/pemu.exe` without options to see usage details:
```
Options:
  -l, --list           List available modular processors
  -p <name>            Select processor to emulate (e.g. i4004, rv32i, i8008, i8080, atmega328p)
  -b <file>            Load binary program file into processor
  -a <addr>            Set start load address (dec or hex starting with 0x, default 0)
  -s, --step           Enable interactive step-by-step mode
  -d, --delay <ms>     Set delay (in milliseconds) between execution steps
  -m, --max-steps <n>  Limit maximum number of execution steps (default: 10000)
  -r, --realtime       Run emulation at real-time clock speed
  -f, --frequency <hz> Override the clock speed in Hz (implies --realtime)
  -h, --help           Show this help message
```

---

## Example: Writing & Running your own program

We provide an assembler tool located in `tools/asm.py` to compile Intel 4004 and Intel 8008 assembly.

### Intel 4004 Example
1. Create a text file `test_4004.asm` with the following code:
   ```assembly
   LDM 5       ; Load 5 into Accumulator
   XCH R1      ; Store it in register R1
   LDM 5       ; Load 5 into Accumulator
   CLC         ; Clear carry
   ADD R1      ; Add R1 (5) to Accumulator -> Acc = 10 (0xA)
   IAC         ; Increment Accumulator -> Acc = 11 (0xB)
   XCH R2      ; Store result in register R2
   HALT:
   JUN HALT    ; Jump to self to halt
   ```

2. Compile the assembly code into a binary file:
   ```cmd
   python tools/asm.py test_4004.asm test_4004.bin
   ```

3. Run the binary on Pemu:
   ```cmd
   build\pemu.exe -p i4004 -b test_4004.bin
   ```

### Intel 8008 Example
1. Create a text file `test_8008.asm` with the following code:
   ```assembly
   MVI B, 7    ; Load 7 into register B
   MVI C, 4    ; Load 4 into register C
   MOV A, B    ; Move B to Accumulator A (A = 7)
   ADD C       ; Add C to Acc (A = 7 + 4 = 11 / 0x0B)
   DCR A       ; Decrement Acc (A = 11 - 1 = 10 / 0x0A)
   MVI H, 0    ; Set memory address high byte H = 0
   MVI L, 100  ; Set memory address low byte L = 100 (0x64)
   MOV M, A    ; Write Acc (10) to memory address 100
   MVI A, 0    ; Clear Acc (A = 0)
   MOV D, M    ; Read memory address 100 to D (D = 10)
   HALT:
   JMP HALT    ; Jump to self to halt
   ```

2. Compile the assembly code:
   ```cmd
   python tools/asm.py test_8008.asm test_8008.bin
   ```

3. Run the binary:
   ```cmd
   build\pemu.exe -p i8008 -b test_8008.bin
   ```

### AVR ATmega328P (Arduino Nano) Example
To run the pre-built ATmega328P simulation which tests stack manipulation, UART Serial prints, and hardware Timer0 overflow interrupts:

1. Build the binaries using the python generator script:
   ```cmd
   python tests/generate_test_bin.py
   ```

2. Run the emulation at a custom clock speed (e.g. 1000 Hz) to watch Serial printing and LED blinking in sequence:
   ```cmd
   build\pemu.exe -p atmega328p -b tests\atmega328p_test.bin -f 1000 -m 100
   ```

### MOS 6502 / NES Emulator Example
To play commercial NES games (like *Tetris*):

1. Place your ROM in the project folder (e.g., `Tetris (Europe).nes`).
2. Run on Windows (native Win32 display, zero dependencies):
   ```cmd
   build\pemu.exe -p mos6502 -b "Tetris (Europe).nes" -r
   ```

---

## Cross-Platform Build (Linux & Raspberry Pi 3)

Pemu is fully compatible with Linux and can render graphics using the **SDL2** library.

### 1. Install SDL2
On Raspberry Pi OS / Debian / Ubuntu, run:
```bash
sudo apt update
sudo apt install libsdl2-dev
```

### 2. Compile Pemu
Compile the project with the following command:
```bash
gcc -O2 -Wall -DUSE_SDL2 -Isrc src/main.c src/processor.c src/processors/*.c -o pemu -lSDL2 -lpthread
```

### 3. Run Emulation
Run your emulation on Linux:
```bash
./pemu -p mos6502 -b "Tetris (Europe).nes" -r
```
*Controls:* Use Arrow keys to move, Z/X to rotate blocks, Enter to Start, Space to Select.

---
