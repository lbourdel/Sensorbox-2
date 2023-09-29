#include <xc.inc>

; When assembly code is placed in a psect, it can be manipulated as a
; whole by the linker and placed in memory.  
;
; In this example, barfunc is the program section (psect) name, 'local' means
; that the section will not be combined with other sections even if they have
; the same name.  class=CODE means the barfunc must go in the CODE container.
; PIC18's should have a delta (addressible unit size) of 1 (default) since they
; are byte addressible.  PIC10/12/16's have a delta of 2 since they are word
; addressible.  PIC18's should have a reloc (alignment) flag of 2 for any
; psect which contains executable code.  PIC10/12/16's can use the default
; reloc value of 1.  Use one of the psects below for the device you use:

psect   barfunc,local,class=CODE,delta=2 ; PIC10/12/16
; psect   barfunc,local,class=CODE,reloc=2 ; PIC18


; Square 10bit unsigned
;  by Martin Sturm  2010
; Tested over full 10bit input range
;
;   a*a --> r
;   a = aH:aL		(10bit, right justified) (not modified)
;   r = rH:rM:rL	(20bit result)
;
; algorithm
;  r = 2^16*(aH*aH) + 2^8*(2*aH*aL) + aL*aL
;             2x2            2x8       8x8
; all multiplications are unrolled
;
;   62 instructions, 49-62 cycles, 56 avg
;
; incorrect result if A has non-zero bits above the 10th
;  use optional ANDLW 0x03 to correct for this if necessary
;

; helper macro
mmac MACRO A,bit, uH,uL
	BTFSC	A,bit
	 ADDWF	uH,F
	RRF	uH,F
	RRF	uL,F
	ENDM

	
global SQR_10
SQR_10 ;MACRO aH,aL,rH,rM,rL
 ;LOCAL g1, g2

	; rM:rL = aL*aL  [8b x 8b mult] (36 instr, 36 cyc)
	CLRF	rM
	CLRF	rL
	BCF	    3,0		;CLRC
	MOVF	aL
	mmac	aL,0, rM,rL
	mmac	aL,1, rM,rL
	mmac	aL,2, rM,rL
	mmac	aL,3, rM,rL
	mmac	aL,4, rM,rL
	mmac	aL,5, rM,rL
	mmac	aL,6, rM,rL
	mmac	aL,7, rM,rL

	; rH = aH*aH   [2b x 2b square] (8 instr, 8 cyc)
	CLRF	rH
	MOVF	aH	; multiplicand in W
;	ANDLW	0x03	; prevent errors if aH non-zero above 10th bit
	BTFSC	aH,1
	 ADDWF	rH,F	; never sets carry
	RLF	rH,F
	BTFSC	aH,0
	 ADDWF	rH,F	; never sets carry

	; rH:rM += 2*aH*aL  [2b x 8b mult] (19 instr, 7-19 cyc, avg. 13)
	RLF	aL,W	; W = 2*aL   (carry is always clear before here)
	BTFSS	aH,0
	 GOTO	g1
	 BTFSC 3,0
	 INCF	rH,F	; add upper bit of 2*aL
	ADDWF   rM,F	; add lower byte of 2*aL
	 BTFSC 3,0
	 INCF	rH,F
g1:
	BTFSS	aH,1
	 GOTO	g2
	; W still holds (2*aL & 0xFF)
	ADDWF   rM,F	; add W to rM (twice)
	 BTFSC 3,0
	 INCF	rH,F
	ADDWF   rM,F	; 
	 BTFSC 3,0
	 INCF	rH,F
	MOVLW	0x02
	BTFSC	aL,7	;
	 ADDWF	rH,F	; add twice the upper bit of 2*aL
g2:
	return