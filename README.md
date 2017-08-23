# XMEGA_CORE_IP_V2
This is a easy to use Atmel MEGA core IP

This is an optimized IP of the Atmel XMEGA processor, that is very simple to use.

Utilization report from implementation:

* Slice LUTs = 1271
* Slice Registers = 119
* F7 Muxes = 8
* F8 Muxes = 0
* Slices = 364
* LUT as logic = 1239
* LUT as memory = 32
* LUT as Flop Flops Pairs = 82


At this moment has implemented the next instructions:

* NOP(1 clock)
* MOVW(1 clock)
* MULS (not implemented yet)
* MULSU (not implemented yet)
* FMUL (not implemented yet)
* FMULSU (not implemented yet)
* CPC, CP(1 clock)
* SBC, SUB(1 clock)
* ADD, ADC, ROL, LSL(1 clock)
* CPSE(2 clock)
* AND(1 clock)
* EOR(1 clock)
* OR(1 clock)
* MOV(1 clock)
* CPI(1 clock)
* SBCI, SUBI(1 clock)
* ORI, SBR(1 clock)
* ANDI, CBR(1 clock)
* LDD, STD(3 clock)
* LDS, STS (3 clock)
* LD Y+, LD Z+, ST Y+, ST Z+(3 clock)
* LD -Y, LD -Z, ST -Y, ST -Z(3 clock)
* LPM_Z (not implemented yet)
* LPM_ZP (not implemented yet)
* LD_X, ST_X(3 clock)
* LD X+, ST X+(3 clock)
* LD -X, ST -X(3 clock)
* POP, PUSH(3 clock/ 2 clock)
* COM(1 clock)
* NEG(1 clock)
* SWAP(1 clock)
* INC(1 clock)
* ASR(1 clock)
* LSR(1 clock)
* ROR(1 clock)
* SEx, CLx(1 clock)
* RET(4 clock)
* RETI (4 clock)
* IJMP, ICALL (1 clock, 3 clock)
* DEC(1 clock)
* JMP, CALL(2 clock, 3 clock)
* ADIW(1 clock)
* SBIW(1 clock)
* CBI, SBI (1 clock)
* SBIC, SBIS (2 clock)
* MUL (1 clock) Optional, has 3 ms latency, I will put them on 2 clock to rise working frequency of the core.
* IN, OUT(1 clock)
* RJMP, RCALL(1 clock, 3 clock)
* LDI(1 clock)
* BRxx(1 clock)
* BLD, BST (1 clock)
* SBRC, SBRS (2 clock)


The implementation is made for Arty Artix-7 from Digilent.
On the implementation and simulation it run this bounch of code that will togle and xor the eight LEDs from board with the four switches and four buttons from the board every ~ 1s.
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

cpse_jmp:
	ldi ZL, LOW(_icall)
	ldi ZH, HIGH(_icall)
	icall
	jmp start1
```
