;******************************************************************
; TMC4361A Library for PIC16F1847
; Filename: TMC4361A_Lib.inc
; Created: 4/18/2020
; Revision: 1.0d1 4/18/2020
;******************************************************************
; History:
;   1.0d1 4/18/2020	First Code
;******************************************************************
; Notes:
; SPI I/O for TMC4361A Motion Controller.
; CKP=0, CKE=1, SMP=1
; RB0	SPI CSn
; RA6	SPI MOSI  aka SDO1
; RB1	SPI MISO  aka SDI1
; RB4	SPI CLK
;
;******************************************************************
; Rotines:	(Stack Words Used)
;
; Init_TMC4361A	(0) Call once to setup SSP1
; SPI_SendRecWord	(0) Send SPI_OutBuff, Receive SPI_InBuff
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
#Define	SPI_CSn_Tris	TRISB,0                ;RB0 CS
#Define	SPI_MOSI_Tris	TRISA,6                ;RA6 MOSI
#Define	SPI_MISO_Tris	TRISB,1                ;RB1 MISO
#Define	SPI_CLK_Tris	TRISB,4                ;RB4 CLK
;
Init_TMC4361A	movlb	2	;bank 2
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
	bsf                    SSP1CON1,CKP           ;CKP=1, Idle CLK High
;	bcf                    SSP1CON1,CKP           ;CKP=0, Idle CLK Low
	bsf	SSP1STAT,SMP
;	bsf	SSP1STAT,CKE	;CKE=1,
	bcf	SSP1STAT,CKE	;CKE=0, 
;
;	BANKSEL	PEI1
;	bsf	PIE1,SSP1	;enable interupt
	movlb	0	;bank 0
	return
;
;*******************************************************************
; Send and Receive
;
; Entry: SPI_OutBuff = word to send
; Exit: SPI_InBuff = word received
; RAM Used: Param78, FSR0, FSR1
; Calls: none
;
SPI_SendRecWord	mMOVLF                 SPI_InBuff,FSR0
                       mMOVLF                 SPI_OutBuff,FSR1
                       movlw                  0x05                   ;Tx/Rx 5 bytes
                       movwf                  Param78
                       movlb	2	;bank 2
	bcf	SPI_CSn	;assert chip select
	movlb	4	;bank 4
SPI_SendRecWord_L2	moviw	FSR1++                 ;Out Buffer
	movwf	SSP1BUF
SPI_SendRecWord_L1	nop
	btfss	SSP1STAT,BF
	bra	SPI_SendRecWord_L1
	movf	SSP1BUF,W
	movwi	FSR0++                 ;In Buffer
	decfsz                 Param78,F
	bra                    SPI_SendRecWord_L2
;
	movlb	2
	bsf	SPI_CSn	;disable chip select
	movlb	0	;bank 0
	return
;
;*************************************************************************
