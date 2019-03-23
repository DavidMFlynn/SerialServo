;******************************************************************
; AS5047D Library for PIC16F1847
; Filename: AS5047D_Lib.inc
; Created: 1/12/2019
; Revision: 1.0d1 1/12/2019
;******************************************************************
; History:
;   1.0d1 1/12/2019	First Code
;******************************************************************
; Notes:
; SPI I/O for AS5047D Magnetic Shaft Angle Encoder.
; CKP=0, CKE=1, SMP=1
; RB0	SPI CSn
; RA6	SPI MOSI  aka SDO1
; RB1	SPI MISO  aka SDI1
; RB4	SPI CLK
;
;******************************************************************
; Rotines:	(Stack Words Used)
; Init_AS5047D	(0) Call once to setup SSP1
;
; AS5047D_WrtZPOSM	(1) call AS5047_Write with AS5047_ZPOSM and Prog_AS5047_ZPOSM
; AS5047D_WrtZPOSL	(1) call AS5047_Write with AS5047_ZPOSL and Prog_AS5047_ZPOSL
; AS5047D_WrtSettings2 (1) call AS5047_Write with AS5047_Settings2 and Prog_AS5047_Settings2
; AS5047D_WrtSettings1 (1) call AS5047_Write with AS5047_Settings1 and Prog_AS5047_Settings1
;
; AS5047_Write	(1) Write data to AS5047D
; AS5047_ReadPos	(1) AS5047_ANGLECOM >> SPI_SendRecWord
; AS5047_ReadANGLEUNCReg (1) AS5047_ANGLEUNC >> AS5047_Read
; AS5047_ReadDIAAGCReg (1) AS5047_DIAAGCReg >> AS5047_Read
; AS5047_ReadZPOSMReg	(1) AS5047_ZPOSM >> AS5047_Read
; AS5047_ReadZPOSLReg  (1) AS5047_ZPOSL >> AS5047_Read
; AS5047_ReadStgs1Reg	(1) AS5047_Settings1 >> AS5047_Read
; AS5047_ReadStgs2Reg	(1) AS5047_Settings2 >> AS5047_Read
; AS5047_ReadErrorReg  (1) AS5047_ErrReg >> AS5047_Read
; AS5047_Read	(1) Send a read command + NOP, returns data
; SPI_SendRecWord	(0) Send Param79:Param78, Receive Param79:Param78
;
; CalcParity	(0) Param79:Param78 = Parity Bit + 15 bit word
; CheckParity	(0) Param79:Param78 = unchanged, ParityErrFlag
;******************************************************************
;
	ifndef kStandAlone
	constant	kStandAlone=0
	endif
;
; Bank 0 persistant variables used by stand alone version
;	Prog_AS5047_ZPOSM:2
;	Prog_AS5047_ZPOSL:2
;	Prog_AS5047_Settings2:2
;	Prog_AS5047_Settings1:2
;
AS5047_ParityBit	EQU	0x8000	;P
AS5047_ReadBit	EQU	0x4000	; R
AS5047_NOP	EQU	0xC000	;PR+0x0000
; Volatile Registers
AS5047_ErrReg	EQU	0x0001	; R+0x0001
AS5047_ProgReg	EQU	0x0003
AS5047_DIAAGCReg	EQU	0x3FFC
AS5047_MAGReg	EQU	0x3FFD
AS5047_ANGLEUNC	EQU	0x3FFE+AS5047_ReadBit
AS5047_ANGLECOM	EQU	0x3FFF+AS5047_ReadBit
; Non-Volatile Registers
AS5047_ZPOSM	EQU	0x0016
AS5047_ZPOSL	EQU	0x0017
AS5047_Settings1	EQU	0x0018
AS5047_Settings2	EQU	0x0019
;
; ***** Add to global declarations ****
;AS5047D_Flags	EQU	Param70	;Check that Param70 is OK to use
;
;#Define	ParityErrFlag	AS5047D_Flags,0
;#Define	AngleReadFlag	AS5047D_Flags,1
;#define	ContinueReadFlag	AS5047D_Flags,2
;#Define	CmdErrorFlag	AS5047D_Flags,3
;
;Diagnostics register bits
MagFldLowErrBit	EQU	3
MagFldHighErrBit	EQU	2
;
;******************************************************************
; Initialize the MSSP
; *SPI: CKP=0 Clock Idle is Low State,
; SPI: CKE=1 Transmit occurs on transition from active to Idle clock state
; *SPI: CKE=0 Transmit occurs on transition from Idle to active clock state
; *SPI: SMP=1 Input data sampled at end of data output time
; SPI master clock=Fosc/64
;
SSP1CON1_Val	EQU	b'00100010'	;SSPEN=1, SPI Fosc/64
#Define	SPI_CSn	LATB,0
#Define	SPI_CSn_Tris	TRISB,0
#Define	SPI_MOSI_Tris	TRISA,6
#Define	SPI_MISO_Tris	TRISB,1
#Define	SPI_CLK_Tris	TRISB,4
;
Init_AS5047D	movlb	2	;bank 2
	bsf	SPI_CSn	;inactive
;
	movlb	1	;bank 1
	bcf	SPI_CSn_Tris	;output
	bsf	SPI_MISO_Tris	;input
	bcf	SPI_MOSI_Tris	;output
	bcf	SPI_CLK_Tris	;output
;
	movlb	4	;bank 4
	movlw	SSP1CON1_Val
	movwf	SSP1CON1
	bsf	SSP1STAT,SMP
;	bsf	SSP1STAT,CKE	;CKE=1
	bcf	SSP1STAT,CKE	;CKE=1
;
;	BANKSEL	PEI1
;	bsf	PIE1,SSP1	;enable interupt
	movlb	0	;bank 0
	return
;
	if kStandAlone
;*******************************************************************
; call AS5047_Write with AS5047_ZPOSM and Prog_AS5047_ZPOSM
;
; Entry: none
; Exit: Param79:Param78 = Data read back, Param7B:Param7A = old data
; Used: Param76..Param7A
; CalcParity, SPI_SendRecWord
;
AS5047D_WrtZPOSM	movlb	0	;bank0
	movlw	low AS5047_ZPOSM
	movwf	Param78
	movlw	high AS5047_ZPOSM
	movwf	Param79
	movf	Prog_AS5047_ZPOSM,W
	movwf	Param7A
	movf	Prog_AS5047_ZPOSM+1,W
	movwf	Param7B
	goto	AS5047_Write
;
;*******************************************************************
; call AS5047_Write with AS5047_ZPOSL and Prog_AS5047_ZPOSL
;
; Entry: none
; Exit: Param79:Param78 = Data read back, Param7B:Param7A = old data
; Used: Param76..Param7A
; CalcParity, SPI_SendRecWord
;
AS5047D_WrtZPOSL	movlb	0	;bank0
	movlw	low AS5047_ZPOSL
	movwf	Param78
	movlw	high AS5047_ZPOSL
	movwf	Param79
	movf	Prog_AS5047_ZPOSL,W
	movwf	Param7A
	movf	Prog_AS5047_ZPOSL+1,W
	movwf	Param7B
	goto	AS5047_Write
;
;*******************************************************************
; call AS5047_Write with AS5047_Settings2 and Prog_AS5047_Settings2
;
; Entry: none
; Exit: Param79:Param78 = Data read back, Param7B:Param7A = old data
; Used: Param76..Param7A
; CalcParity, SPI_SendRecWord
;
AS5047D_WrtSettings2	movlb	0	;bank0
	movlw	low AS5047_Settings2
	movwf	Param78
	movlw	high AS5047_Settings2
	movwf	Param79
	movf	Prog_AS5047_Settings2,W
	movwf	Param7A
	movf	Prog_AS5047_Settings2+1,W
	movwf	Param7B
	goto	AS5047_Write
;
;*******************************************************************
; call AS5047_Write with AS5047_Settings1 and Prog_AS5047_Settings1
;
; Entry: none
; Exit: Param79:Param78 = Data read back, Param7B:Param7A = old data
; Used: Param76..Param7A
; CalcParity, SPI_SendRecWord
;
AS5047D_WrtSettings1	movlb	0	;bank0
	movlw	low AS5047_Settings1
	movwf	Param78
	movlw	high AS5047_Settings1
	movwf	Param79
	movf	Prog_AS5047_Settings1,W
	movwf	Param7A
	movf	Prog_AS5047_Settings1+1,W
	movwf	Param7B
;
; fall through to AS5047_Write
	endif
;*******************************************************************
; Write data to AS5047D
;  Write address, dicard dummy data
;  Write data, save old data
;  Send NOP, save new data
;
; Entry: Param79:Param78 = Address, Param7B:Param7A = Data
; Exit: Param79:Param78 = Data read back 14 bit value, ParityErrFlag, CmdErrorFlag,
;   Param7B:Param7A = old data 16 bits including error and parity bits
; RAM Used: Param76..Param7A
; Calls:CalcParity, SPI_SendRecWord
;
AS5047_Write	bcf	AngleReadFlag
	bcf	ContinueReadFlag
;
	bcf	Param79,6	;Write Address, dicard dummy data
	call	CalcParity
	call	SPI_SendRecWord	;ignore rec'd word
;
	movf	Param7B,W	;Send data
	movwf	Param79
	movf	Param7A,W
	movwf	Param78
	bcf	Param79,6	;Write data, save old value
	call	CalcParity
	call	SPI_SendRecWord
	movf	Param79,W	;store old data
	movwf	Param7B
	movf	Param78,W
	movwf	Param7A
;
	movlw	low AS5047_NOP	;send NOP, save data for verify
	movwf	Param78
	movlw	high AS5047_NOP
	movwf	Param79
	call	SPI_SendRecWord
	call	CheckParity	;set ParityErrFlag
	bcf	CmdErrorFlag
	btfsc	Param79,6	;Cmd Error?
	bsf	CmdErrorFlag
	movlw	0x3F
	andwf	Param79,F
	return
;
;*******************************************************************
; Read the compensated position
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; Calls: SPI_SendRecWord, CheckParity
;
AS5047_ReadPos	movlw	low AS5047_ANGLECOM
	movwf	Param78
	movlw	high AS5047_ANGLECOM
	movwf	Param79
;
	call	CalcParity
	call	SPI_SendRecWord
;
	call	CheckParity
	bcf	CmdErrorFlag
	btfsc	Param79,6	;Cmd Error?
	bsf	CmdErrorFlag
	movlw	0x3F
	andwf	Param79,F
;
; if (!ContinueReadFlag || !AngleReadFlag) then ignore data and read again
	btfss	ContinueReadFlag
	bra	AS5047_ReadPos_1
	btfss	AngleReadFlag
	bra	AS5047_ReadPos_1
	return
;
AS5047_ReadPos_1	bsf	ContinueReadFlag
	bsf	AngleReadFlag
	bra	AS5047_ReadPos	;one more time
;
;*******************************************************************
; Read the Angle Uncompensated Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadANGLEUNCReg	movlw	low AS5047_ANGLEUNC
	movwf	Param78
	movlw	high AS5047_ANGLEUNC
	movwf	Param79
	goto	AS5047_ReadReg
;
;*******************************************************************
; Read the Diagnostics Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadDIAAGCReg	movlw	low AS5047_DIAAGCReg
	movwf	Param78
	movlw	high AS5047_DIAAGCReg
	movwf	Param79
	goto	AS5047_ReadReg
;
;*******************************************************************
; Read the Zero position MSB Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadZPOSMReg	movlw	low AS5047_ZPOSM
	movwf	Param78
	movlw	high AS5047_ZPOSM
	movwf	Param79
	goto	AS5047_ReadReg
;
;*******************************************************************
; Read the Zero position LSB Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadZPOSLReg	movlw	low AS5047_ZPOSL
	movwf	Param78
	movlw	high AS5047_ZPOSL
	movwf	Param79
	goto	AS5047_ReadReg
;
;*******************************************************************
; Read the Settings1 Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadStgs1Reg	movlw	low AS5047_Settings1
	movwf	Param78
	movlw	high AS5047_Settings1
	movwf	Param79
	goto	AS5047_ReadReg
;
;*******************************************************************
; Read the Settings2 Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadStgs2Reg	movlw	low AS5047_Settings2
	movwf	Param78
	movlw	high AS5047_Settings2
	movwf	Param79
	goto	AS5047_ReadReg
;
;*******************************************************************
; Read the Error Register
;
; Entry: none
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadErrorReg	movlw	low AS5047_ErrReg
	movwf	Param78
	movlw	high AS5047_ErrReg
	movwf	Param79
;
; fall through to AS5047_ReadReg
;*******************************************************************
; Read any Register
;
; Entry: Param79:Param78 = 14 bit address to read
; Exit: Param79:Param78 = 14 bit value, ParityErrFlag, CmdErrorFlag
; RAM Used: Param76, Param77, Param78, Param79
; calls: AS5047_Read
;
AS5047_ReadReg	call	AS5047_Read
;
	call	CheckParity
	bcf	CmdErrorFlag
	btfsc	Param79,6	;Cmd Error?
	bsf	CmdErrorFlag
	movlw	0x3F
	andwf	Param79,F
	return
;
;*******************************************************************
; Read Data
;  Send a read command, dicards dummy data
;  Send NOP, returns data
;
; Entry: Param79:Param78 = 14 bit Address
; Exit: Param79:Param78 = Parity+EF+data
; RAM Used: Param76,Param77,Param78, Param79
; Calls: CalcParity,SPI_SendRecWord
;
AS5047_Read	bcf	ContinueReadFlag
	bcf	AngleReadFlag
;
	bsf	Param79,6	;read bit
	call	CalcParity
	call	SPI_SendRecWord
	movlw	low AS5047_NOP
	movwf	Param78
	movlw	high AS5047_NOP
	movwf	Param79
;
; fall through to SPI_SendRecWord
;*******************************************************************
; Send and Receive
;
; Entry: Param79:Param78 = word to send
; Exit: Param79:Param78 = word received
; RAM Used: Param78, Param79
; Calls: none
;
SPI_SendRecWord	movlb	2	;bank 2
	bcf	SPI_CSn	;assert chip select
	movlb	4	;bank 4
	movf	Param79,W
	movwf	SSP1BUF
SPI_SendRecWord_L1	nop
	btfss	SSP1STAT,BF
	bra	SPI_SendRecWord_L1
	movf	SSP1BUF,W
	movwf	Param79
;
	movf	Param78,W
	movwf	SSP1BUF
SPI_SendRecWord_L2	nop
	btfss	SSP1STAT,BF
	bra	SPI_SendRecWord_L2
	movf	SSP1BUF,W
	movwf	Param78
;
	movlb	2
	bsf	SPI_CSn	;disable chip select
	movlb	0	;bank 0
	return
;
;*******************************************************************
; Calculate Even Parity Bit for Param79:Param78
;
; Entry: Param79:Param78 = 15 bit word
; Exit: Param79:Param78 = Parity Bit + 15 bit word
; RAM Used: Param76,Param77,Param78,Param79
; Calls: none
;
CalcParity	clrf	Param76
	movlw	.15	;bit count
	movwf	Param77
CalcParity_L1	rlf	Param78,F
	rlf	Param79,F
	movlw	0x80
	andwf	Param79,W
	xorwf	Param76,F
	decfsz	Param77,F
	bra	CalcParity_L1
	rlf	Param78,F
	rlf	Param79,F	;LSb to _C
	rlf	Param78,F	;_C to LSb
	rlf	Param79,F
	movlw	0x80
	andwf	Param76,W	;get parity bit
	bcf	Param79,7	;force to 0 (default)
	iorwf	Param79,F	;set parity bit
	return
;
;**********************************************************************
; Check Even Parity Bit for Param79:Param78
;
; Entry: Param79:Param78 = Parity Bit + 15 bit word
; Exit: Param79:Param78 = unchanged, ParityErrFlag
; RAM Used: Param76,Param77,Param78,Param79
; Calls: none
;
CheckParity	clrf	Param76
	movlw	.15	;bit count
	movwf	Param77
CheckParity_L1	rlf	Param78,F
	rlf	Param79,F
	movlw	0x80
	andwf	Param79,W
	xorwf	Param76,F
	decfsz	Param77,F
	bra	CheckParity_L1
	rlf	Param78,F
	rlf	Param79,F	;LSb to _C
	rlf	Param78,F	;_C to LSb
	rlf	Param79,F
	movf	Param76,W	;get calc'd parity bit
	xorwf	Param79,W	;xor w/ parity bit
	andlw	0x80
	bcf	ParityErrFlag
	btfsc	WREG,7
	bsf	ParityErrFlag
	return
;
;*************************************************************************
