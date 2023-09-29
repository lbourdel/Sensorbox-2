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

; Multiply 10bit by 10bit unsigned
;  by Martin Sturm  2010
; Tested over full range of 10bit input combinations
;
;   a (10bit) = aH:aL   (not modified)
;   b (10bit) = bH:bL   (not modified)
;   aH:aL * bH:bL --> rH:rM:rL
;
; incorrect result if a or b has non-zero bits above the 10th
;  unless optional ANDLW 0x03 is used
;
; Algorithm
;  r = a*b
;  r = 2^16*(aH*bH) + 2^8*(aH*bL + bH*aL) + aL*bL
;             2x2           2x8     2x8      8x8
; all multiplications are unrolled
;
;  73 instructions, 57-73 cycles, 65 avg
;    
    

GLOBAL _Umul10
;SIGNAT _Umul10,4217
GLOBAL _aH,_aL,_bH,_bL,_rH,_rM,_rL
PSECT mytext,local,class=CODE,delta=2 ; PIC10/12/16


; helper macro
mmac MACRO _A,_bit, _uH,_uL
    BTFSC   _A,_bit
    ADDWF   _uH,F
    RRF	    _uH,F
    RRF	    _uL,F
    ENDM

_Umul10:

; rM:rL = aL*bL   [8x8 bit multiply] (36 cycles)
    CLRF    _rM
    CLRF    _rL
    BCF	    3,0		;CLRC
    MOVF   _bL,W
    mmac    _aL,0, _rM,_rL
    mmac    _aL,1, _rM,_rL
    mmac    _aL,2, _rM,_rL
    mmac    _aL,3, _rM,_rL
    mmac    _aL,4, _rM,_rL
    mmac    _aL,5, _rM,_rL
    mmac    _aL,6, _rM,_rL
    mmac    _aL,7, _rM,_rL

; rH = aH*bH   [2x2 bit multiply] (8 cycles)
    CLRF    _rH
    MOVF   _bH,W	; multiplicand in W
;   ANDLW   0x03	; prevent errors if bH non-zero above 10th bit
    BTFSC   _aH,1	; carry always clear by here
    ADDWF   _rH,F	; never sets carry
    RLF	    _rH,F
    BTFSC   _aH,0
    ADDWF   _rH,F

; rH:rM += aH*bL  [2-bit x 8-bit] (7-15 cycles, avg 11)
    MOVF   _bL,W	; multiplicand in W
    BTFSS   _aH,0
     BRA    g1
    ADDWF   _rM,F	; add bL
    BTFSC   3,0		;SKPNC
    INCF    _rH,F
g1:
    BTFSS   _aH,1
     BRA    g2
    BCF	    3,0		;CLRC
    RLF     _bL,W	;  W now holds (2*bL & 0xFF)
    ADDWF   _rM,F	; add W to rM
    BTFSC   3,0		;SKPNC
     INCF    _rH,F
    BTFSC   _bL,7	;
     INCF    _rH,F	; add the upper bit of 2*bL
g2:

; rH:rM += bH*aL  [2-bit x 8-bit] (7-15 cycles, avg 11)
    MOVF   _aL,W	; multiplicand in W
    BTFSS   _bH,0
     BRA   g3
    ADDWF   _rM,F	; add aL
    BTFSC   3,0		;SKPNC
     INCF    _rH,F
g3:
    BTFSS   _bH,1
     BRA   g4
    BCF	    3,0		;CLRC
    RLF     _aL,W	;  W now holds (2*aL & 0xFF)
    ADDWF   _rM,F	; add W to rM
    BTFSC   3,0		;SKPNC
     INCF   _rH,F
    BTFSC   _aL,7	;
     INCF   _rH,F	; add the upper bit of 2*aL
g4:
    return
     
