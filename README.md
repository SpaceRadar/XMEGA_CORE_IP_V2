# XMEGA_CORE_IP_V2

This is an optimized IP of the Atmel MEGA and XMEGA processor, that is very simple to use.

It is a fully asynchronnous core only with some parts that are sinchronized with the clock and it is synchronized only on positive edge of the clock.

The posedge to posedge clock latency must be bigger than total latency of the ROM memory and core logic.

Utilization report from implementation for CORE_TYPE_CLASSIC_128K:

* Slice LUTs = 1241
* Slice Registers = 50
* F7 Muxes = 38
* F8 Muxes = 0
* Slices = 399
* LUT as logic = 1209
* LUT as memory = 32
* LUT as Flop Flops Pairs = 33

More details about instruction set implementation of different core type you can read here: https://en.wikipedia.org/wiki/Atmel_AVR_instruction_set

The core is configurable from minimal core to XMEGA core.

At this moment has implemented the next instructions:


* NOP (1 clock)
* MOVW (1 clock)
* MULS (not implemented yet)
* MULSU (not implemented yet)
* FMUL (not implemented yet)
* FMULSU (not implemented yet)
* CPC, CP (1 clock)
* SBC, SUB (1 clock)
* ADD, ADC, ROL, LSL (1 clock)
* CPSE (2 clock)
* AND (1 clock)
* EOR (1 clock)
* OR (1 clock)
* MOV (1 clock)
* CPI (1 clock)
* SBCI, SUBI (1 clock)
* ORI, SBR (1 clock)
* ANDI, CBR (1 clock)
* LDD, STD (1 clock) (original 2 clock)
* LDS, STS (1 clock) (original 2/3 clock)
* LD Y+, LD Z+, ST Y+, ST Z+ (2 clock)
* LD -Y, LD -Z, ST -Y, ST -Z (2 clock)
* LPM_Z (not implemented yet)
* LPM_ZP (not implemented yet)
* XCH, LAS, LAC, LAT (1 clock) (original 2 clock) (from second revision silicon - AU,B,C parts)
* LD_X, ST_X (1 clock) (original 2 clock)
* LD X+, ST X+ (2 clock)
* LD -X, ST -X (2 clock)
* POP (1 clock) (original 2 clock)
* PUSH (1 clock)
* COM (1 clock)
* NEG (1 clock)
* SWAP (1 clock)
* INC (1 clock)
* ASR (1 clock)
* LSR (1 clock)
* ROR (1 clock)
* SEx, CLx(1 clock)
* RET (2 clock)
* RETI (2 clock)
* IJMP (1 clock) (original 2 clock)
* ICALL (2 clock) (original 3 clock)
* DEC (1 clock)
* JMP, CALL (2 clock) (original 3 clock)
* ADIW (1 clock)
* SBIW (1 clock)
* CBI, SBI (1 clock)
* SBIC, SBIS (2 clock)
* MUL (2 clock) (original 1 clock) One extra clock because the multiply unit on FPGA has ~3ms latency.
* IN, OUT (1 clock)
* RJMP, RCALL (1 clock, 2 clock)
* LDI (1 clock)
* BRxx (1 clock)
* BLD, BST (1 clock)
* SBRC, SBRS (2 clock)

All instruction that is executed on more than 1 core clock it will be optimized. 

The implementation is made for Arty Artix-7 from Digilent.
On the implementation and simulation it run this bounch of code that will togle and xor the eight LEDs from board with the four switches and four buttons from the board every ~ 1s.

The core work at 66.66Mhz without timing violations, but in reality can work at more than 100Mhz.

This sequency is made to test the implemented instructions.


```asm

.org 0
	rjmp start
start3:
	rcall start2
	ret
start2:
	nop
	ret
start1:
	call start2
	rcall start3
	nop
	nop

start:
	rjmp start4
start4:
	clr r0
	clr r28
	clr r29

	clr r24
	clr r25
	ldi r26,0xff
	ldi r27,0x00
loop1:
	sbiw r24:r25, 0x01
	sbc r26, r0
	sbci r27, 0
	brmi continue1
	rjmp loop1
continue1:
	in r16,0x00
	ldi r17,0xFF
	eor r16,r17
	out 0, r16
	clr r24
	clr r25
	ldi r26,0xff
	ldi r27,0x00
loop2:
	sbiw r24:r25, 0x01
	sbc r26, r0
	sbci r27, 0
	brmi continue2
	rjmp loop2
continue2:
	in r16,0x00
	std y+2, R16
	ldd R17,y+2
	ldi r16,0xff
	eor r17,r16
	eor r17,r16
	push R17
	pop r0
	eor r0, r16
	sts 0x08, r0
	lds r1, 0x08
	eor r1, r16
	out 0, r1
	add r1, r16
	sub r1, r16
	ldi ZL, LOW(_ijmp)
	ldi ZH, HIGH(_ijmp)
	ijmp
_icall:
	ret
_ijmp:
	sbiw Z, 1
	adiw Z, 1
	subi zl, LOW(0x40)
	sbci zh, HIGH(0x40)
	subi zl, LOW(-0x40)
	sbci zh, HIGH(-0x40)
	neg zl
	neg zl
	com zl
	com zl
	lsr zl
	lsl zl
	ror zl
	rol zl

	cpse zl, zl
	nop
	cpse zl, zl
	jmp cpse_jmp
	nop

	st -z, r16
	ld r17,z+
	ldi xl, LOW(0x35)
	ldi xh, HIGH(0x35)
	ldi r16, 0x22
	st -x, r16
	inc r16
	st -x, r16
	inc r16
	st -x, r16
	ld r17,x+
	ld r17,x+
	ld r17,x+

cpse_jmp:
	ldi ZL, LOW(_icall)
	ldi ZH, HIGH(_icall)
	icall
	jmp start1

```
