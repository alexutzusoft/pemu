CC = cl.exe
CFLAGS = /nologo /W4 /O2 /D_CRT_SECURE_NO_WARNINGS /Isrc

OBJS = build\main.obj build\processor.obj build\dummy.obj build\i4004.obj build\rv32i.obj build\i8008.obj build\i8080.obj build\atmega328p.obj build\mos6502.obj \
	build\z80.obj build\i8085.obj build\i8086.obj build\i4040.obj build\mc6800.obj build\mc6809.obj build\m68000.obj build\cdp1802.obj \
	build\sm83.obj build\w65c02.obj build\chip8.obj build\lc3.obj build\pdp8.obj build\mips32.obj build\tms9900.obj \
	build\i80186.obj build\i80286.obj build\necv20.obj build\z180.obj build\z8.obj build\z8000.obj build\m68010.obj build\mc6805.obj build\mc68hc11.obj build\hd6309.obj \
	build\mos6510.obj build\w65816.obj build\huc6280.obj build\spc700.obj build\i8051.obj build\i8048.obj build\pic16.obj build\msp430.obj build\arm7tdmi.obj build\sh2.obj \
	build\tms1000.obj build\scmp.obj build\f8.obj build\s2650.obj build\cp1600.obj build\rv64i.obj build\pdp11.obj build\nova.obj build\picoblaze.obj build\j1.obj

NEW_HDRS = src\processors\z80.h src\processors\i8085.h src\processors\i8086.h src\processors\i4040.h src\processors\mc6800.h src\processors\mc6809.h src\processors\m68000.h src\processors\cdp1802.h src\processors\sm83.h src\processors\w65c02.h src\processors\chip8.h src\processors\lc3.h src\processors\pdp8.h src\processors\mips32.h src\processors\tms9900.h \
	src\processors\i80186.h src\processors\i80286.h src\processors\necv20.h src\processors\z180.h src\processors\z8.h src\processors\z8000.h src\processors\m68010.h src\processors\mc6805.h src\processors\mc68hc11.h src\processors\hd6309.h \
	src\processors\mos6510.h src\processors\w65816.h src\processors\huc6280.h src\processors\spc700.h src\processors\i8051.h src\processors\i8048.h src\processors\pic16.h src\processors\msp430.h src\processors\arm7tdmi.h src\processors\sh2.h \
	src\processors\tms1000.h src\processors\scmp.h src\processors\f8.h src\processors\s2650.h src\processors\cp1600.h src\processors\rv64i.h src\processors\pdp11.h src\processors\nova.h src\processors\picoblaze.h src\processors\j1.h

build\pemu.exe: $(OBJS)
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /Fe:build\pemu.exe $(OBJS) gdi32.lib user32.lib

build\main.obj: src\main.c src\processor.h src\processors\i8080.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\main.c /Fobuild\main.obj

build\processor.obj: src\processor.c src\processor.h src\processors\dummy.h src\processors\i4004.h src\processors\rv32i.h src\processors\i8008.h src\processors\i8080.h src\processors\atmega328p.h src\processors\mos6502.h $(NEW_HDRS)
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processor.c /Fobuild\processor.obj

build\dummy.obj: src\processors\dummy.c src\processors\dummy.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\dummy.c /Fobuild\dummy.obj

build\i4004.obj: src\processors\i4004.c src\processors\i4004.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i4004.c /Fobuild\i4004.obj

build\rv32i.obj: src\processors\rv32i.c src\processors\rv32i.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\rv32i.c /Fobuild\rv32i.obj

build\i8008.obj: src\processors\i8008.c src\processors\i8008.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i8008.c /Fobuild\i8008.obj

build\i8080.obj: src\processors\i8080.c src\processors\i8080.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i8080.c /Fobuild\i8080.obj

build\atmega328p.obj: src\processors\atmega328p.c src\processors\atmega328p.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\atmega328p.c /Fobuild\atmega328p.obj

build\mos6502.obj: src\processors\mos6502.c src\processors\mos6502.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mos6502.c /Fobuild\mos6502.obj

build\z80.obj: src\processors\z80.c src\processors\z80.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\z80.c /Fobuild\z80.obj

build\i8085.obj: src\processors\i8085.c src\processors\i8085.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i8085.c /Fobuild\i8085.obj

build\i8086.obj: src\processors\i8086.c src\processors\i8086.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i8086.c /Fobuild\i8086.obj

build\i4040.obj: src\processors\i4040.c src\processors\i4040.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i4040.c /Fobuild\i4040.obj

build\mc6800.obj: src\processors\mc6800.c src\processors\mc6800.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mc6800.c /Fobuild\mc6800.obj

build\mc6809.obj: src\processors\mc6809.c src\processors\mc6809.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mc6809.c /Fobuild\mc6809.obj

build\m68000.obj: src\processors\m68000.c src\processors\m68000.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\m68000.c /Fobuild\m68000.obj

build\cdp1802.obj: src\processors\cdp1802.c src\processors\cdp1802.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\cdp1802.c /Fobuild\cdp1802.obj

build\sm83.obj: src\processors\sm83.c src\processors\sm83.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\sm83.c /Fobuild\sm83.obj

build\w65c02.obj: src\processors\w65c02.c src\processors\w65c02.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\w65c02.c /Fobuild\w65c02.obj

build\chip8.obj: src\processors\chip8.c src\processors\chip8.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\chip8.c /Fobuild\chip8.obj

build\lc3.obj: src\processors\lc3.c src\processors\lc3.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\lc3.c /Fobuild\lc3.obj

build\pdp8.obj: src\processors\pdp8.c src\processors\pdp8.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\pdp8.c /Fobuild\pdp8.obj

build\mips32.obj: src\processors\mips32.c src\processors\mips32.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mips32.c /Fobuild\mips32.obj

build\tms9900.obj: src\processors\tms9900.c src\processors\tms9900.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\tms9900.c /Fobuild\tms9900.obj

build\i80186.obj: src\processors\i80186.c src\processors\i80186.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i80186.c /Fobuild\i80186.obj

build\i80286.obj: src\processors\i80286.c src\processors\i80286.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i80286.c /Fobuild\i80286.obj

build\necv20.obj: src\processors\necv20.c src\processors\necv20.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\necv20.c /Fobuild\necv20.obj

build\z180.obj: src\processors\z180.c src\processors\z180.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\z180.c /Fobuild\z180.obj

build\z8.obj: src\processors\z8.c src\processors\z8.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\z8.c /Fobuild\z8.obj

build\z8000.obj: src\processors\z8000.c src\processors\z8000.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\z8000.c /Fobuild\z8000.obj

build\m68010.obj: src\processors\m68010.c src\processors\m68010.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\m68010.c /Fobuild\m68010.obj

build\mc6805.obj: src\processors\mc6805.c src\processors\mc6805.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mc6805.c /Fobuild\mc6805.obj

build\mc68hc11.obj: src\processors\mc68hc11.c src\processors\mc68hc11.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mc68hc11.c /Fobuild\mc68hc11.obj

build\hd6309.obj: src\processors\hd6309.c src\processors\hd6309.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\hd6309.c /Fobuild\hd6309.obj

build\mos6510.obj: src\processors\mos6510.c src\processors\mos6510.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\mos6510.c /Fobuild\mos6510.obj

build\w65816.obj: src\processors\w65816.c src\processors\w65816.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\w65816.c /Fobuild\w65816.obj

build\huc6280.obj: src\processors\huc6280.c src\processors\huc6280.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\huc6280.c /Fobuild\huc6280.obj

build\spc700.obj: src\processors\spc700.c src\processors\spc700.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\spc700.c /Fobuild\spc700.obj

build\i8051.obj: src\processors\i8051.c src\processors\i8051.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i8051.c /Fobuild\i8051.obj

build\i8048.obj: src\processors\i8048.c src\processors\i8048.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\i8048.c /Fobuild\i8048.obj

build\pic16.obj: src\processors\pic16.c src\processors\pic16.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\pic16.c /Fobuild\pic16.obj

build\msp430.obj: src\processors\msp430.c src\processors\msp430.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\msp430.c /Fobuild\msp430.obj

build\arm7tdmi.obj: src\processors\arm7tdmi.c src\processors\arm7tdmi.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\arm7tdmi.c /Fobuild\arm7tdmi.obj

build\sh2.obj: src\processors\sh2.c src\processors\sh2.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\sh2.c /Fobuild\sh2.obj

build\tms1000.obj: src\processors\tms1000.c src\processors\tms1000.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\tms1000.c /Fobuild\tms1000.obj

build\scmp.obj: src\processors\scmp.c src\processors\scmp.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\scmp.c /Fobuild\scmp.obj

build\f8.obj: src\processors\f8.c src\processors\f8.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\f8.c /Fobuild\f8.obj

build\s2650.obj: src\processors\s2650.c src\processors\s2650.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\s2650.c /Fobuild\s2650.obj

build\cp1600.obj: src\processors\cp1600.c src\processors\cp1600.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\cp1600.c /Fobuild\cp1600.obj

build\rv64i.obj: src\processors\rv64i.c src\processors\rv64i.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\rv64i.c /Fobuild\rv64i.obj

build\pdp11.obj: src\processors\pdp11.c src\processors\pdp11.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\pdp11.c /Fobuild\pdp11.obj

build\nova.obj: src\processors\nova.c src\processors\nova.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\nova.c /Fobuild\nova.obj

build\picoblaze.obj: src\processors\picoblaze.c src\processors\picoblaze.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\picoblaze.c /Fobuild\picoblaze.obj

build\j1.obj: src\processors\j1.c src\processors\j1.h src\processor.h
	@if not exist build mkdir build
	$(CC) $(CFLAGS) /c src\processors\j1.c /Fobuild\j1.obj

clean:
	@if exist build del /q build\* 2>NUL
	@if exist build rmdir build 2>NUL
