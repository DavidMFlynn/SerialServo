;===================================================================================
;
;  FileName: DMFMath.inc
;  Date: 10/20/2018
;  File Version: 1.0.1
;  
;  Author: David M. Flynn
;  Company: Oxford V.U.E., Inc.
;
;======================================================================================
; Notes:
;
;  These are generic math routines for the 16F1847
;  Bank must be correct for math variables and is not changed.
;
;======================================================================================
; Routines
;	(bank in/bank exit)
; Int32ToFixedBArg	(3/3)Int32 at (FSR0) to Fixed in BArg
; Int32ToFixedAArg	(3/3)Int32 at (FSR0) to Fixed in AArg
; AddInt2424	(3/3)Signed addition Int24 + Int24, AArg = AArg + BArg
; SubInt3232	(3/3)Signed sutraction Int32 - Int32, AArg = AArg - BArg
; AddInt3232	(3/3)Signed addition Int32 + Int32, AArg = AArg + BArg
; SwapAArgBArg	(3/3)Int32 swap, yes including the sign bits
; FXM1616U	(3/3)16x16 Bit Unsigned Fixed Point Multiply 16 x 16 -> 32
;	AccB3:AccB0 = AArgB1:AArgB0 * BArgB1:BArgB0
; AddAArgToAcc	(3/3)AccB3:AccB0 = AccB3:AccB0 + AArgB3:AArgB0
; MULWF	(3/3)macro FileReg, PRODH:PRODL = FileReg * W
; FXM0808U	(3/3)PRODH:PRODL = Param78 * Param79 Unsined UInt16 = UInt8 * UInt8
; MUL0808	(3/3)PRODH:PRODL = Param78 * Param79 Signed Int16 = Int8 * Int8
; MUL1616	(3/3)Int32 = Int16 * Int16, Acc = AArg * BArg
;
; _24_BitAdd	(3/3)AARGB2:AARGB0 = AARGB2:AARGB0 + BARGB2:BARGB0
; _32_BitAdd	(3/3)AARGB3:AARGB0 = AARGB3:AARGB0 + BARGB3:BARGB0
; _24_bit_sub	(3/3)AARGB2:AARGB0 = AARGB2:AARGB0 - BARGB2:BARGB0
; _32_bit_sub	(3/3)AARGB3:AARGB0 = AARGB3:AARGB0 - BARGB3:BARGB0
; FXD2416U	(3/3)Unsigned Integer division UInt24/UInt16 AArg=AArg/BArg 
; FXD3232U	(3/3)Unsigned Integer division UInt32/UInt32 AArg=AArg/BArg
;======================================================================================
; Revision History
;
; 1.0.1  10/20/2018	Fixed FXM0808U, Added MUL0808, MUL1616
; 1.0.0  9/21/2018	First code.
;
;======================================================================================
; Options
	ifndef UsesFXD3232U
	constant UsesFXD3232U=0
	endif
;
	ifndef UsesMUL1616
	constant UsesMUL1616=0
	endif
;	
;======================================================================================
; Int32 at (FSR0) to Fixed in BArg
;
Int32ToFixedBArg	moviw	3[FSR0]	;MSB W/ sign
	btfsc	WREG,7
	bra	Int32ToFixedBArg_1
; It's positive, just copy it.
	bsf	BArg_sign
	movwf	BArgB3
	moviw	2[FSR0]
	movwf	BArgB2
	moviw	1[FSR0]
	movwf	BArgB1
	moviw	0[FSR0]
	movwf	BArgB0
	return
; It's negative, 2's comp
Int32ToFixedBArg_1	bcf	BArg_sign
	clrf	BArgB0
	clrf	BArgB1
	clrf	BArgB2
	clrf	BArgB3
	moviw	0[FSR0]
	subwf	BArgB0,F
	moviw	1[FSR0]
	subwfb	BArgB1,F
	moviw	2[FSR0]
	subwfb	BArgB2,F
	moviw	3[FSR0]
	subwfb	BArgB3,F
	return
;
;=======================================================
; Int32 at (FSR0) to Fixed in AArg
Int32ToFixedAArg	moviw	3[FSR0]	;MSB W/ sign
	btfsc	WREG,7
	bra	Int32ToFixedAArg_1
; It's positive, just copy it.
	bsf	AArg_sign
	movwf	AArgB3
	moviw	2[FSR0]
	movwf	AArgB2
	moviw	1[FSR0]
	movwf	AArgB1
	moviw	0[FSR0]
	movwf	AArgB0
	return
; It's negative, 2's comp
Int32ToFixedAArg_1	bcf	AArg_sign
	clrf	AArgB0
	clrf	AArgB1
	clrf	AArgB2
	clrf	AArgB3
	moviw	0[FSR0]
	subwf	AArgB0,F
	moviw	1[FSR0]
	subwfb	AArgB1,F
	moviw	2[FSR0]
	subwfb	AArgB2,F
	moviw	3[FSR0]
	subwfb	AArgB3,F
	return
;
;***********************************************************************
; Function: AddInt2424
;
; Signed addition Int24 + Int24, AArg = AArg + BArg
;
; PreCondition: The sign bits in pidStat2 have been set or cleared
;    depending on the variables they represent.
;
; Overview: This routine takes the numbers loaded into the math
;    variables (AArgB, BArgB) and determines whether they 
;    need to be added or subtracted based on their sign 
;    which is located in the pidStat2 register.
;
; Input: AArg, AArg_sign, BArg, BArg_sign
;
; Output: AArgB, AArg_sign
;
;***********************************************************************
AddInt2424	btfsc	AArg_sign
	bra	AddInt2424_1	;AArg is Positive
	bra	AddInt2424_2
;
AddInt2424_1	btfsc	BArg_sign
	goto	_24_BitAdd	;both are POSITIVE (11), add them
	bra	AddInt2424_3
;
AddInt2424_2	btfss	BArg_sign
	goto	_24_BitAdd	;both are NEGATIVE (00), add them
; signs don't match
AddInt2424_3	movf	BArgB0,W
	subwf	AArgB0,W
	movf	BArgB1,W
	subwfb	AArgB1,W
	movf	BArgB2,W
	subwfb	AArgB2,W
	SKPB		;Is BArg > AArg?
	goto	_24_bit_sub	; No, AArg>=BArg
;
	call	SwapAArgBArg	;BArgB > AArgB
	goto	_24_bit_sub	
;
;***********************************************************************
; Signed fixed point sutraction Int32 - Int32, AArg = AArg - BArg
;  Invert the sign of BArg then add
; Input: AArg, AArg_sign, BArg, BArg_sign
;
; Output: AArgB, AArg_sign
;***********************************************************************
SubInt3232	btfsc	BArg_sign
	bra	SubInt3232_1	;BArg is Positive
	bsf	BArg_sign
	bra	AddInt3232
;
SubInt3232_1	bcf	BArg_sign
; fall through to AddInt3232
;***********************************************************************
; Signed fixed point addition Int32 + Int32, AArg = AArg + BArg
;
; Input: AArg, AArg_sign, BArg, BArg_sign
;
; Output: AArgB, AArg_sign
;***********************************************************************
AddInt3232	btfsc	AArg_sign
	bra	AddInt3232_1	;AArg is Positive
	bra	AddInt3232_2
;
AddInt3232_1	btfsc	BArg_sign
	goto	_32_BitAdd	;both are POSITIVE (11), add them
	bra	AddInt3232_3
;
AddInt3232_2	btfss	BArg_sign
	goto	_32_BitAdd	;both are NEGATIVE (00), add them
; signs don't match
AddInt3232_3	movf	BArgB0,W
	subwf	AArgB0,W
	movf	BArgB1,W
	subwfb	AArgB1,W
	movf	BArgB2,W
	subwfb	AArgB2,W
	movf	BArgB3,W
	subwfb	AArgB3,W
	SKPB		;Is BArg > AArg?
	goto	_32_bit_sub	; No, AArg>=BArg
;
	call	SwapAArgBArg	;BArgB > AArgB
	goto	_32_bit_sub	
;
;=========================================================================================
; Int32 swap, yes including the sign bits
;
; Param78 = BArg
; BArg = AArg
; AArg = Param78
;
SwapAArgBArg	movf	BArgB0,W
	movwf	Param78	;swap AArgB0 with BArgB0
	movf	AArgB0,W
	movwf	BArgB0
	movf	Param78,W
	movwf	AArgB0
;
	movf	BArgB1,W
	movwf	Param78	;swap AArgB1 with BArgB1
	movf	AArgB1,W
	movwf	BArgB1
	movf	Param78,W
	movwf	AArgB1
;
	movf	BArgB2,W
	movwf	Param78	;swap AArgB2 with BArgB2
	movf	AArgB2,W
	movwf	BArgB2
	movf	Param78,W
	movwf	AArgB2
;	
	movf	BArgB3,W
	movwf	Param78	;swap AArgB3 with BArgB3
	movf	AArgB3,W
	movwf	BArgB3
	movf	Param78,W
	movwf	AArgB3
;
	bcf	Param78,0	;swap the signs too
	btfsc	AArg_sign
	bsf	Param78,0
	bcf	AArg_sign
	btfsc	BArg_sign
	bsf	AArg_sign
	bcf	BArg_sign
	btfsc	Param78,0
	bsf	BArg_sign
	return
;
;=========================================================================================
;=========================================================================================
; 16x16 Bit Unsigned Fixed Point Multiply 16 x 16 -> 32
; AccB3:AccB0 = AArgB1:AArgB0 * BArgB1:BArgB0
;
; Entry: AArgB1:AArgB0, BArgB1:BArgB0
; Exit: Acc, _C, W changed
; Ram used: AccB0:AccB3, AArgB0:AArgB3, BArgB0:BArgB1
; Calls: 1/0 AddAArgToAcc
FXM1616U	clrf	AccB0	;Acc = 0
	clrf	AccB1
	clrf	AccB2
	clrf	AccB3
	movf	AArgB0,W	;if AArg = 0 RTN
	iorwf	AArgB1,W
	SKPNZ
	return
;
	clrf	AArgB2
	clrf	AArgB3
FXM1616U_L1	movf	BArgB0,W	;if BArg=0 RTN
	iorwf	BArgB1,W
	SKPNZ
	return
;
	btfsc	BArgB0,0	;if LSb add AArg to Acc
	call	AddAArgToAcc
	lslf	AArgB0,F	;AArg = AArg * 2
	rlf	AArgB1,F
	rlf	AArgB2,F
	rlf	AArgB3,F
	lsrf	BArgB1,F	;BArg = BArg / 2
	rrf	BArgB0,F
	bra	FXM1616U_L1
;
;=========================================================================================
; Add AArg to Accumulator
; AccB3:AccB0 = AccB3:AccB0 + AArgB3:AArgB0
; Entry: AArgB3:AArgB0, AccB3:AccB0
; Exit: AccB3:AccB0, _C, W changed
AddAArgToAcc	movf	AArgB0,W
	addwf	AccB0,F
	movf	AArgB1,W
	addwfc	AccB1,F
	movf	AArgB2,W
	addwfc	AccB2,F
	movf	AArgB3,W
	addwfc	AccB3,F
	return
;
;-------------------------------------------------------------------------
; 8x8 Bit Unsigned Fixed Point Multiply 8 * 8 -> 16
; PRODH:PRODL = FileReg * W
; uses: Param78, Param79, Param7A
;
; macro to implement PIC18F MULWF command
MULWF	macro	FileReg
	movwf	Param78
	movf	FileReg,W
	movwf	Param79
	call	FXM0808U
	endm
;
;=============================
; PRODH:PRODL = Param78 * Param79
; Uinsigned FF*FF=FE01
; Ram used: Param78,Param79,Param7A,PRODH,PRODL
;
FXM0808U	clrf	PRODH
	clrf	PRODL
	movf	Param79,F
	SKPNZ
	return
	clrf	Param7A
FXM0808U_L1	movf	Param78,F
	SKPNZ		;Param78=0?
	return		; Yes
;
	btfss	Param78,0
	bra	FXM0808U_1
	movf	Param79,W
	addwf	PRODL,F
	movf	Param7A,W
	addwfc	PRODH,F
FXM0808U_1	lsrf	Param78,F	;Param78 = Param78 / 2
	lslf	Param79,F	;Param7A:Param79 = Param7A:Param79 * 2
	rlf	Param7A,F
	bra	FXM0808U_L1	
;
;=============================
; PRODH:PRODL = Param78 * Param79
; Signed FF*FF=0001
; Ram used: Param78,Param79,Param7A,Param7B,PRODH,PRODL
;
MUL0808	clrf	PRODH
	clrf	PRODL
	movf	Param79,F	
	SKPNZ		;Param79=0?
	return		; Yes
;
	clrf	Param7A
	clrf	Param7B
	movlw	0xFF
	btfsc	Param79,7	;Param79 is negative?
	movwf	Param7A	; Yes, sign extend it
	btfsc	Param78,7	;Param78 is negative?
	movwf	Param7B	; Yes, sign extend it
; PRODH:PRODL = Param7B:Param78 * Param7A:Param79
MUL0808_L1	movf	Param78,W
	iorwf	Param7B,W
	SKPNZ		;Param78=0?
	return		; Yes
;
	btfss	Param78,0
	bra	MUL0808_1
	movf	Param79,W
	addwf	PRODL,F
	movf	Param7A,W
	addwfc	PRODH,F
MUL0808_1	lsrf	Param7B,F
	rrf	Param78,F	;Param78 = Param78 / 2
	lslf	Param79,F	;Param7A:Param79 = Param7A:Param79 * 2
	rlf	Param7A,F
	bra	MUL0808_L1	
;
	if UsesMUL1616
;=========================================================================================
; Acc = AArg * BArg
; Signed Int32 = Int16 * Int16
; Ram used: AccB3:AccB0, AArgB3:AArgB0, BArgB3:BArgB0
;
MUL1616	clrf	AccB0
	clrf	AccB1
	clrf	AccB2
	clrf	AccB3
	movf	AArgB0,W
	iorwf	AArgB1,W
	SKPNZ		;AArg = 0?
	return		; Yes
;
	clrf	BArgB2
	clrf	BArgB3
	clrf	AArgB2
	clrf	AArgB3
	movlw	0xFF
	btfss	AArgB1,7
	bra	MUL1616_1
	movwf	AArgB2	;Sign extend AArg
	movwf	AArgB3
;
MUL1616_1	btfss	BArgB1,7
	bra	MUL1616_L1
	movwf	BArgB2	;Sign extend BArg
	movwf	BArgB3
;
MUL1616_L1	movf	BArgB0,W
	iorwf	BArgB1,W
	iorwf	BArgB2,W
	iorwf	BArgB3,W
	SKPNZ		;BArg=0?
	return		; Yes
;
	btfss	BArgB0,0
	bra	MUL1616_2
	movf	AArgB0,W
	addwf	AccB0,F
	movf	AArgB1,W
	addwfc	AccB1,F
	movf	AArgB2,W
	addwfc	AccB2,F
	movf	AArgB3,W
	addwfc	AccB3,F
;
MUL1616_2	lsrf	BArgB3,F
	rrf	BArgB2,F
	rrf	BArgB1,F
	rrf	BArgB0,F
;
	lslf	AArgB0,F
	rlf	AArgB1,F
	rlf	AArgB2,F
	rlf	AArgB3,F
	bra	MUL1616_L1
;
	endif
;=========================================================================================
;---------------------------------------------------------------------
;24-BIT ADDITION
;  AARGB2:AARGB0 = AARGB2:AARGB0 + BARGB2:BARGB0
;
_24_BitAdd	movf	BArgB0,W
	addwf	AArgB0,F
	movf	BArgB1,W
	addwfc	AArgB1,F
	movf	BArgB2,W
	addwfc	AArgB2,F
	return
;
;---------------------------------------------------------------------
;32-BIT ADDITION
;  AARGB3:AARGB0 = AARGB3:AARGB0 + BARGB3:BARGB0
;
_32_BitAdd	movf	BArgB0,W
	addwf	AArgB0,F
	movf	BArgB1,W
	addwfc	AArgB1,F
	movf	BArgB2,W
	addwfc	AArgB2,F
	movf	BArgB3,W
	addwfc	AArgB3,F
	return
;
;---------------------------------------------------------------------
;24-BIT SUBTRACTION
;  AARGB2:AARGB0 = AARGB2:AARGB0 - BARGB2:BARGB0
;
_24_bit_sub	movf	BArgB0,W
	subwf	AArgB0,F
	movf	BArgB1,W
	subwfb	AArgB1,F
	movf	BArgB2,W
	subwfb	AArgB2,F
	return
;
;---------------------------------------------------------------------
;32-BIT SUBTRACTION
;  AARGB3:AARGB0 = AARGB3:AARGB0 - BARGB3:BARGB0
;
_32_bit_sub	movf	BArgB0,W
	subwf	AArgB0,F
	movf	BArgB1,W
	subwfb	AArgB1,F
	movf	BArgB2,W
	subwfb	AArgB2,F
	movf	BArgB3,W
	subwfb	AArgB3,F
	return
;
;==============================================================================
; Unsigned Integer division UInt24/UInt16 AArg=AArg/BArg 
;
;Inputs:
;   Dividend - AARGB2:AARGB1:AARGB0
;   Divisor  - BARGB1:BARGB0
;Temporary:
;   Counter  - LOOPCOUNT
;Output:
;   Quotient - AARGB2:AARGB1:AARGB0
;   Remainder- RemB1:RemB0
;
#Define	LOOPCOUNT	Param78
;
FXD2416U	CLRF	RemB0
	CLRF	RemB1
;
	MOVLW	.24
	MOVWF	LOOPCOUNT
;
FXD2416U_L1	RLF	AArgB0,F
	RLF	AArgB1,F
	RLF	AArgB2,F
;
	RLF	RemB0,F
	RLF	RemB1,F
;
;test if Rem >= BArg
	MOVF	BArgB0,W
	subwf	RemB0,W
	MOVF	BArgB1,W
	subwfb	RemB1,W	;keep that byte in W untill we make sure about borrow
;
	SKPNB		;Rem >= BArg?
	bra	FXD2416U_1	;don't add to the result
;
;Yes, do the subtraction for real this time.
	movf	BArgB0,W
	subwf	RemB0,F
	movf	BArgB1,W
	subwfb	RemB1,F
;
	bsf	_C	;add to the result
	bra	FXD2416U_2
;
FXD2416U_1	bcf	_C
FXD2416U_2	DECFSZ	LOOPCOUNT,F	;decrement counter
	bra	FXD2416U_L1	;and repeat loop if not zero
;
	RLF	AArgB0,F	;shift in last bit of result
	RLF	AArgB1,F
	RLF	AArgB2,F
	RETURN
;
	if UsesFXD3232U
;============================================================================
; Unsigned Integer division UInt32/UInt32 AArg=AArg/BArg
;
;
FXD3232U	clrf	RemB0
	clrf	RemB1
	clrf	RemB2
	clrf	RemB3
;
	movlw	.32
	movwf	LOOPCOUNT
;
FXD3232U_L1	rlf	AArgB0,F
	rlf	AArgB1,F
	rlf	AArgB2,F
	rlf	AArgB3,F
;
	rlf	RemB0,F
	rlf	RemB1,F
	rlf	RemB2,F
	rlf	RemB3,F
;test if Rem >= BArg
	movf	BArgB0,W
	subwf	RemB0,W
	movf	BArgB1,W
	subwfb	RemB1,W
	movf	BArgB2,W
	subwfb	RemB2,W
	movf	BArgB3,W
	subwfb	RemB3,W
;
	SKPNB		;Rem >= BArg?
	bra	FXD3232U_1	; No
;Yes, do the subtraction for real this time.
	movf	BArgB0,W
	subwf	RemB0,F
	movf	BArgB1,W
	subwfb	RemB1,F
	movf	BArgB2,W
	subwfb	RemB2,F
	movf	BArgB3,W
	subwfb	RemB3,F
;	
	bsf	_C	;add to the result
	bra	FXD3232U_2
;
FXD3232U_1	bcf	_C	;don't add to the result
FXD3232U_2	decfsz	LOOPCOUNT,F
	bra	FXD3232U_L1
;
	rlf	AArgB0,F
	rlf	AArgB1,F
	rlf	AArgB2,F
	rlf	AArgB3,F
	return		
;
	endif
;=========================================================================================
	