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

global _mul16s	; extern of function goes in the C source file
_mul16s:
    movf PORTA,w    ; here we use a symbol defined via xc.inc
    return

    
    
    ;malin@onspec.co.uk
;signed multiply of a2:a1 with b2:b1 leaving result in res4:res3:res2:res1
;	These 8 variables need to be defined, as does neg_flag
;
;	Program length	64 lines
;	time 134 to 248 cycles
;	This program looks at the lsb of a1 to decide whether to add b1 to res2
;
;   and b2 to res3, with appropriate carrys
;	It then looks at the lsb of a2 to decide whether to add b1 to res3 and 
;	b2 to res4, again with appropriate carrys.
;	The rotates then only have to be done 8 times
;
;	This is uses slightly more program but takes a little less time than 
;	a routine that performs one 16 bit addition per rotate and 16 rotates
;
;	Multiple byte addition routine from Microchip AN617
;	Result registers used as loop counter from Bob Fehrenbach & Scott Dattalo
;


	clrf    res4
	clrf	res3
	clrf    res2
	movlw	0x80
	movwf	res1		

	clrf	neg_flag

	btfss   a2,7
	goto    a_pos
	comf    a2,f
	comf	a1,f
	incf	a1,f
	btfsc	status, zero
	incf    a2,f
	incf	neg_flag, f
a_pos
	btfss   b2,7
	goto    nextbit
	comf    b2,f
	comf    b1,f 
	incf    b1,f 
	btfsc   status, zero
	incf    b2,f
	incf	neg_flag, f

nextbit
	rrf		a2,f
	rrf		a1,f

	btfss	status, carry
	goto	nobit_l
	movf	b1,w
	addwf	res2,f

	movf	b2, w
	btfsc	status, carry
	incfsz	b2, w	
	addwf	res3, f	
	btfsc	status, carry
	incf	res4, f
	bcf		status, carry
	
nobit_l	
	btfss	a1, 7
	goto	nobit_h
	movf	b1,w
	addwf	res3,f
	btfsc	status, carry
	incf	res4,f
	movf	b2,w		
	addwf	res4,f
nobit_h
	rrf		res4,f
	rrf		res3,f
	rrf		res2,f
	rrf		res1,f

	btfss   status, carry
	goto	nextbit
	btfss   neg_flag, 0
	goto 	no_invert		

	comf    res4,f
	comf    res3,f
	comf    res2,f
	comf	res1,f

	incf    res1,f
	btfsc   status,zero
	incf	res2,f
	btfsc   status,zero
	incf    res3,f
	btfsc   status,zero
	incf    res4,f
no_invert
	return