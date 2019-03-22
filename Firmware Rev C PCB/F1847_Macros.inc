	subtitle	"F648A Macros"
	page
;===================================================================================
;
;  FileName: F1847 Macros.asm
;  Date: 5/29/2017
;  File Version: 1.4.0
;  
;  Author: David M. Flynn
;  Company: Oxford V.U.E., Inc.
;
;============================================================================================
; Notes:
;
;  These are generic macros for the 16F1847
;
;============================================================================================
; Revision History
;
; 1.4.0  5/29/2017     Added CPFSGT, SETF
; 1.3.1  11/16/13	Modified for 16F1847
; 1.3    4/28/04	Added mBankISel
; 1.2.4  5/19/03	Added BNE, BEQ
; 1.2.3  5/16/03	Added CMP, BMI and BPL.
; 1.2.2  4/23/03	Changed LCALL to be just CALL when same segment.
; 1.2.1  4/15/03	Added SKPB, SKPNB
; 1.2    2/26/03	Added Instruction set extentions for 14 bit core (16F###) (27 of them)
; 1.1    2/21/03	Added mLED_On, mLED_Off, mPCLGoto
;	Added 18F452 version of mCall#To#
; 1.0.1  2/12/03	Fixed mCallxTox so a return from a segment other than the one
;	called will not cause PCLATH,3/PCLATH,4 problems.
; 1.0    10/1/02	Moved macros to this file
;
;============================================================================================
; Conditionals
;
;============================================================================================
; Name	(additional stack words required), (Return Bank) Description
;============================================================================================
;TestTimerIsZero	(0),(0) pass Timer1Lo,Timer2Lo, etc. returns STATUS,Z set if zero
;
;mCall0To1	(1+n),(n/a) used to set/clr PCLATH bits
;mCall0To2	(1+n),(n/a) used to set/clr PCLATH bits
;mCall0To3	(1+n),(n/a) used to set/clr PCLATH bits
;
;mCall1To0	(1+n),(n/a) used to set/clr PCLATH bits
;mCall1To2	(1+n),(n/a) used to set/clr PCLATH bits
;mCall1To3	(1+n),(n/a) used to set/clr PCLATH bits
;
;mCall2To0	(1+n),(n/a) used to set/clr PCLATH bits
;mCall2To1	(1+n),(n/a) used to set/clr PCLATH bits
;mCall2To3	(1+n),(n/a) used to set/clr PCLATH bits
;
;mCall3To0	(1+n),(n/a) used to set/clr PCLATH bits
;mCall3To1	(1+n),(n/a) used to set/clr PCLATH bits
;mCall3To2	(1+n),(n/a) used to set/clr PCLATH bits
;
;mPCLGoto	(0),(n/a) Set PCLATH:0..2 then add W to PCL
;mBankISel	(0),(n/a) Set Status,IRP bit corect for the operand passed.
;LOADFSR0 	(0),(n/a) Add (16 bits) ADDRESS+INDEX >> FSR0
;LOADFSR1 	(0),(n/a) Add (16 bits) ADDRESS+INDEX >> FSR1
;LOADFSR0W 	(0),(n/a) Add (16 bits) ADDRESS+WReg >> FSR0
;LOADFSR1W	(0),(n/a) Add (16 bits) ADDRESS+WReg >> FSR1
;===================================================================
; Instruction set extentions for 14 bit core (16F###)
;
; opcode	operand	description
;---------------------------------------------------------------
;ADDCF	f,d	Add Carry to File
;ADDDCF	f,d	Add Digit Carry to File
;BC	k	Branch on Carry
;BDC	k	Branch on Digit Carry
;BEQ	k	Branch on Equal to Zero aka BZ
;BMI	k	Branch on W minus (bit 7 set) (W bits 0..6 are zerroed)
;BNC	k	Branch on No Carry
;BNDC	k	Branch on No Digit Carry
;BNE	k	Branch on Not Equal to Zero aka BNZ
;BNZ	k	Branch on No Zero
;BPL	k	Branch on W plus (bit 7 clr) (W bits 0..6 are zerroed)
;BZ	k	Branch on Zero
;CLRC		Clear Carry
;CLRDC		Clear Digit Carry
;CLRZ		Clear Zero flag
;CMP	f	Compare F to W (F-W)
;CPFSGT                f                      skip if (f) > (W), W changed
;LCALL	k	Long Call
;LGOTO	k	Long GOTO
;MOVFW	f	Move File to W
;NEGF	f,d	Negate File
;SETC		Set Carry
;SETDC		Set Digit Carry
;SETF                  f                      (f) = 0xFF, W changed
;SETZ		Set Zero
;SKPB		Skip on Borrow
;SKPC		Skip on Carry
;SKPDC		Skip on Digit Carry
;SKPNB		Skip on Not Borrow
;SKPNC		Skip on No Carry
;SKPNDC		Skip on No Digit Carry
;SKPNZ		Skip on Not Zero
;SKPZ		Skip on Zero
;SUBBF	f,d	Subtract Borrow from File (used after SUBWF)
;SUBCF	f,d	Subtract Carry from File
;SUBDCF	f,d	Subtract Digit Carry from File
;TSTF	f	Test File
;
;======================================================================================
; pass Timer1Lo,Timer2Lo, etc.
; returns STATUS,Z set if zero
;
TestTimerIsZero	macro	T_Number
	CLRF	BSR	;Bank0
	MOVF	T_Number,W
	IORWF	T_Number+1,W
	endm	
;
;=================================================================================
;=================================================================================
; Call macros, use only when stack space is short 16F877 version
;
;Call from segment 0 to segment 1
;
;
mCall0To1	macro	Destination
	BSF	PCLATH,3
	CALL	Destination
	CLRF	PCLATH
	endm
;
;Call from segment 0 to segment 2
;
mCall0To2	macro	Destination
	BSF	PCLATH,4
	CALL	Destination
	CLRF	PCLATH
	endm
;
;Call from segment 0 to segment 3
;
mCall0To3	macro	Destination
	BSF	PCLATH,3
	BSF	PCLATH,4
	CALL	Destination
	CLRF	PCLATH
	endm
;
;Call from segment 1 to segment 0
;
mCall1To0	macro	Destination
	BCF	PCLATH,3
	CALL	Destination
	BSF	PCLATH,3
	BCF	PCLATH,4
	endm
;
;Call from segment 1 to segment 2
;
mCall1To2	macro	Destination
	BSF	PCLATH,4
	BCF	PCLATH,3
	CALL	Destination
	BCF	PCLATH,4
	BSF	PCLATH,3
	endm
;
;Call from segment 1 to segment 3
;
mCall1To3	macro	Destination
	BSF	PCLATH,4
	CALL	Destination
	BSF	PCLATH,3
	BCF	PCLATH,4
	endm
;
;Call from segment 2 to segment 0
;
mCall2To0	macro	Destination
	BCF	PCLATH,4
	CALL	Destination
	BCF	PCLATH,3
	BSF	PCLATH,4
	endm
;
;Call from segment 2 to segment 1
;
mCall2To1	macro	Destination
	BSF	PCLATH,3
	BCF	PCLATH,4
	CALL	Destination
	BCF	PCLATH,3
	BSF	PCLATH,4
	endm
;
;Call from segment 2 to segment 3
;
mCall2To3	macro	Destination
	BSF	PCLATH,3
	CALL	Destination
	BCF	PCLATH,3
	BSF	PCLATH,4
	endm
;
;Call from segment 3 to segment 0
;
mCall3To0	macro	Destination
	BCF	PCLATH,3
	BCF	PCLATH,4
	CALL	Destination
	BSF	PCLATH,3
	BSF	PCLATH,4
	endm
;
;Call from segment 3 to segment 1
;
mCall3To1	macro	Destination
	BCF	PCLATH,4
	CALL	Destination
	BSF	PCLATH,3
	BSF	PCLATH,4
	endm
;
;Call from segment 3 to segment 2
;
mCall3To2	macro	Destination
	BCF	PCLATH,3
	CALL	Destination
	BSF	PCLATH,3
	BSF	PCLATH,4
	endm
;
;=================================================================================
; Set PCLATH:0..2 then add W to PCL
; CAUTION:The first BSF/BCF instruction and the location of the 
;  last branched to instruction must be in the same page
;
; Entry: W= and 8 bit offset for PCL
; Exit: branches forward W locations
; RAM used:none
; Calls:(0) none
;
mPCLGoto	macro	
	if $ & 0x0100
	BSF	PCLATH,0
	else
	BCF	PCLATH,0
	endif
	if $ & 0x0200
	BSF	PCLATH,1
	else
	BCF	PCLATH,1
	endif
	if $ & 0x0400
	BSF	PCLATH,2
	else
	BCF	PCLATH,2
	endif
;
	ADDWF	PCL,F
	endm
;
;-----------------------------------------------------------------------------------------------
; The macro LOADFSR1 loads FSR1 and makes code easier to read.
;------------------------------------- LOADFSR macro -------------------------------------------
;
LOADFSR0	macro 	ADDRESS,INDEX 				
	movlw 	low ADDRESS 	; load address 						
	addwf	INDEX,W	; add the index value to determine location in array
	movwf 	FSR0L	; load FSR1L with pointer info
	clrf	FSR0H
	movlw	high ADDRESS
	addwfc	FSR0H,F
	endm		
;
LOADFSR1	macro 	ADDRESS,INDEX 				
	movlw 	low ADDRESS 	; load address 						
	addwf	INDEX,W	; add the index value to determine location in array
	movwf 	FSR1L	; load FSR1L with pointer info
	clrf	FSR1H
	movlw	high ADDRESS
	addwfc	FSR1H,F
	endm		
;
LOADFSR0W	macro 	ADDRESS				
	addlw 	low ADDRESS						
	movwf 	FSR0L
	clrf	FSR0H
	movlw	high ADDRESS
	addwfc	FSR0H,F	;<< needed to work w/ contig ram
	endm		
;
LOADFSR1W	macro 	ADDRESS				
	addlw 	low ADDRESS						
	movwf 	FSR1L
	clrf	FSR1H
	movlw	high ADDRESS
	addwfc	FSR1H,F	;<< needed to work w/ contig ram
	endm		
;
;=======================================================================================
; Instruction set extentions for 14 bit core (16F###)
;=======================================================================================
; Add Carry to File
;
;ADDCF	macro	dest,W_or_F
;	BTFSC	STATUS,C
;	INCF	dest,W_or_F
;	endm
;
;==============================================================
; Add Digit Carry to File
;
;ADDDCF	macro	dest,W_or_F
;	BTFSC	STATUS,DC
;	INCF	dest,W_or_F
;	endm
;
;==============================================================
; Branch on Carry
;
;BC	macro	dest
;	BTFSC	STATUS,C
;	GOTO	dest
;	endm
;
;==============================================================
; Branch on Digit Carry
;
;BDC	macro	dest
;	BTFSC	STATUS,DC
;	GOTO	dest
;	endm
;
;==============================================================
; Branch on Equal to Zero aka BZ
;
BEQ	macro	dest
	BTFSC	STATUS,Z
	GOTO	dest
	endm
;
;==============================================================
; Branch on W minus (bit 7 set) (W bits 0..6 are zerroed)
;
BMI	macro	dest
	ANDLW	0x80
	BTFSC	STATUS,Z	;skip if Z
	GOTO	dest
	endm
;
;==============================================================
; Branch on No Carry
;
;BNC	macro	dest
;	BTFSS	STATUS,C
;	GOTO	dest
;	endm
;
;==============================================================
; Branch on No Digit Carry
;
;BNDC	macro	dest
;	BTFSS	STATUS,DC
;	GOTO	dest
;	endm
;
;==============================================================
; Branch on Not Equal to Zero aka BNZ
;
BNE	macro	dest
	BTFSS	STATUS,Z
	GOTO	dest
	endm
;
;==============================================================
; Branch on No Zero
;
;BNZ	macro	dest
;	BTFSS	STATUS,Z
;	GOTO	dest
;	endm
;
;==============================================================
; Branch on W plus (bit 7 clr) (W bits 0..6 are zerroed)
;
BPL	macro	dest
	ANDLW	0x80
	BTFSS	STATUS,Z	;skip if not Z
	GOTO	dest
	endm
;
;==============================================================
; Branch on Zero
;
;BZ	macro	dest
;	BTFSC	STATUS,Z
;	GOTO	dest
;	endm
;
;==============================================================
; Clear Carry
;
;CLRC	macro
;	BCF	STATUS,C
;	endm
;
;==============================================================
; Clear Digit Carry
;
;CLRDC	macro
;	BCF	STATUS,DC
;	endm
;
;==============================================================
; Clear Zero flag
;
;CLRZ	macro
;	BCF	STATUS,Z
;	endm
;
;==============================================================
; Compare F to W (F-W)
;
CMP	macro	f
	SUBWF	f,W
	endm
;
;=========================================================================================
; (f) - (W)
;skip if (f) > (W)
CPFSGT	macro	FileReg
	subwf	FileReg,W
	btfsc	_Z	;W = F?
	goto	$+2	; yes
	btfss	_C	;W > F?
			; yes, W>F or W=F
	endm
;
	if oldCode
;==============================================================
; Long Call
;
LCALL	macro	dest
	if ($ & 0x0800) <> (dest & 0x0800)
	if dest & 0x0800
	BSF	PCLATH,3
	else
	BCF	PCLATH,3
	endif
	endif
		
	if ($ & 0x1000) <> (dest & 0x1000)
	if dest & 0x1000
	BSF	PCLATH,4
	else
	BCF	PCLATH,4
	endif
	endif	

	CALL	dest
	if (($ & 0x0800) <> (dest & 0x0800)) | (($ & 0x1000) <> (dest & 0x1000))
	if $ & 0x0800
	BSF	PCLATH,3
	else
	BCF	PCLATH,3
	endif
	
	if $ & 0x1000
	BSF	PCLATH,4
	else
	BCF	PCLATH,4
	endif
	endif
	endm
;
;==============================================================
; Long GOTO
;
LGOTO	macro	dest
	if ($ & 0x0800) <> (dest & 0x0800)
	if dest & 0x0800
	BSF	PCLATH,3
	else
	BCF	PCLATH,3
	endif
	endif
		
	if ($ & 0x1000) <> (dest & 0x1000)
	if dest & 0x1000
	BSF	PCLATH,4
	else
	BCF	PCLATH,4
	endif
	endif	

	GOTO	dest
	endm
;
	endif
;==============================================================
; Move File to W
;
;MOVFW	macro	src
;	MOVF	src,W
;	endm
;
;==============================================================
; Negate File
;
;NEGF	macro	dest,W_or_F
;	COMF	dest,F
;	INCF	dest,W_or_F
;	endm
;
;==============================================================
; Set Carry
;
;SETC	macro
;	BSF	STATUS,C
;	endm
;
;==============================================================
; Set Digit Carry
;
;SETDC	macro
;	BSF	STATUS,DC
;	endm
;
;==============================================================
SETF	macro	FileReg
	movlw	0xFF
	movwf	FileReg
	endm
;
;==============================================================
; Set Zero
;
;SETZ	macro
;	BSF	STATUS,Z
;	endm
;
;==============================================================
; Skip on Borrow
;
SKPB	macro
	BTFSC	STATUS,C
	endm
;
;==============================================================
; Skip on Carry
;
;SKPC	macro
;	BTFSS	STATUS,C
;	endm
;
;==============================================================
; Skip on Digit Carry
;
;SKPDC	macro
;	BTFSS	STATUS,DC
;	endm
;
;==============================================================
; Skip on Not Borrow
;
SKPNB	macro
	BTFSS	STATUS,C
	endm
;
;==============================================================
; Skip on No Carry
;
;SKPNC	macro
;	BTFSC	STATUS,C
;	endm
;
;==============================================================
; Skip on No Digit Carry
;
;SKPNDC	macro
;	BTFSC	STATUS,DC
;	endm
;
;==============================================================
; Skip on Not Zero
;
;SKPNZ	macro
;	BTFSC	STATUS,Z
;	endm
;
;==============================================================
; Skip on Zero
;
;SKPZ	macro
;	BTFSS	STATUS,Z
;	endm
;
;==============================================================
; Subtract Borrow from File
;
SUBBF	macro	dest,W_or_F
	BTFSS	STATUS,C
	DECF	dest,W_or_F
	endm
;
;==============================================================
; Subtract Carry from File
;
;SUBCF	macro	dest,W_or_F
;	BTFSC	STATUS,C
;	DECF	dest,W_or_F
;	endm
;
;==============================================================
; Subtract Digit Carry from File
;
;SUBDCF	macro	dest,W_or_F
;	BTFSC	STATUS,DC
;	DECF	dest,W_or_F
;	endm
;
;==============================================================
; Test File
;
;TSTF	macro	src
;	MOVF	src,F
;	endm
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
;
