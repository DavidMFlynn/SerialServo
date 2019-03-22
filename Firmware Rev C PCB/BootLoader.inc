;************************************************
; BootLoader for PIC16F1847
; filename: Bootloader1847.inc
; Created: 3/21/2019
; Revision: 1.0d1 3/21/2019
;************************************************
; History
; 1.0d1 3/21/2019 first code
;************************************************
; Notes:
;  Reset vector:
;  	movlp	high BootLoader
;    	goto	BootLoader
;  ProgStartVector	CLRF	PCLATH
;	goto	Start
;
;  DataEEProm 255 = 255 = run bootloader
;
; What happens next:
;  If Data EEProm address 255 = 0 then goto ProgStartVector
;  Setup  UART for 38.4KBaud
;  Parse incoming text as .hex file
;   if program memory address > 3 and < BootLoader then
;    read block, update block, compare block, erase block, write block
;
;  reset
;************************************************
; .hex
;:020000040000FA 
;       ^^^^^^ Set high word of address to 0x0000
;:100010005A23582356235423422021000C16200033
;:100020003A100C1E3A14A10B053221000C122000CC
;^ Start code
; ^^ Bytes of data
;   ^^^^ Address low word (0x0020)
;       ^^ Record type 0x00 = Data
;         ^^^^ LSB MSB of data for Addr / 2
;                                         ^^ Checksum
; Checksum is 2's complement of all other bytes added together
; Valid address range 0000 0004 .. 0000 3DFE (bootloader is at 0000 3E00 .. 0000 3FFF, 256 words)
;************************************************
;
;
	cblock 0x20
	PROG_ADDR_LO
	PROG_ADDR_HI
	endc
;
;================================================
;
; Entry: DataEEProm255
; Exit: goto ProgStartVector
; Calls: none
; RAM used: ALL
;
BootLoader:
	call	InitializeBLIO
;
	call	ReadData
	iorlw	0x00
	SKPNZ
	bra	BootLoader_End
;
;
;
BootLoader_End	movlp	0x00
	goto	ProgStartVector
;
;==============================================
; Read EEProm(255)
;
ReadData:	
	BANKSEL	EEADRL
	MOVLW	0xFF
	MOVWF	EEADRL
;
;Data Memory
;Address to read ;Deselect Config space
	BCF	EECON1,CFGS
	BCF	EECON1,EEPGD
;Point to DATA memory
	BSF	EECON1,RD	;EE Read
	MOVF	EEDATL,W	;W = EEDATL
	movlb	0
	return
;
;==============================================
; Read Program memory block
;
; Entry: FSR1,PROG_ADDR_HI:PROG_ADDR_LO
; Exit: FSR1 changed, 32 words >> (FSR1++)
;
ReadProgBlock:
	BANKSEL EEADRL		; Select Bank for EEPROM registers
	movf	PROG_ADDR_HI,W
	MOVWF	EEADRH	; Store MSB of address
	movf	PROG_ADDR_LO,W	
	MOVWF	EEADRL	; Store LSB of address
	BCF	EECON1,CFGS	; Do not select Configuration Space
	BSF	EECON1,EEPGD	; Select Program Memory
	movlw	.32	;Block is .32 words
	movwf	Param79
ReadProgBlock_L1	BSF	EECON1,RD	; Initiate read
	NOP		; skipped
	NOP		; do nothing
	MOVF	EEDATL,W	; Get LSB of word
	movwi	FSR1++	; Store in user location
	MOVF	EEDATH,W	; Get MSB of word
	movwi	FSR1++	; Store in user location
	decfsz	Param79,F
	bra	ReadProgBlock_1
	movlb	0
	return
;
ReadProgBlock_1	incf	EEADRL,F
	bra	ReadProgBlock_L1
;
;=====================================================
; Erase Program memory block
;
; Entry: PROG_ADDR_HI:PROG_ADDR_LO
;
EraseProgBlock:
	BANKSEL	EEADRL
	movf	PROG_ADDR_HI,W
	MOVWF	EEADRH	;Load upper 6 bits of erase address boundary
	movf	PROG_ADDR_LO,W
	MOVWF	EEADRL	;Load lower 8 bits of erase address boundary
	BSF	EECON1,EEPGD	;Point to program memory
	BCF	EECON1,CFGS	;Not configuration space
	BSF	EECON1,FREE	;Specify an erase operation
	BSF	EECON1,WREN	;Enable writes
	MOVLW	55h	;Start of required sequence to initiate erase
	MOVWF	EECON2	;Write 55h
	MOVLW	0AAh
	MOVWF	EECON2	;Write AAh
	BSF	EECON1,WR	;Set WR bit to begin erase
	NOP		;Any instructions here are ignored as processor
	NOP		; halts to begin erase sequence
			;Processor will stop here and wait for erase complete.
			;after erase processor continues with 3rd instruction
	BCF	EECON1,WREN	;Disable writes
	movlb	0
	return
;
;==========================================================
; Write Program memory block
;
; Entry: PROG_ADDR_HI:PROG_ADDR_LO, FSR1
; Exit: FSR1 changed
; 
WriteProgBlock:
	BANKSEL	EEADRH
	movf	PROG_ADDR_HI,W
	MOVWF	EEADRH	; Store MSB of address
	movf	PROG_ADDR_LO,W	
	MOVWF	EEADRL	; Store LSB of address
	BSF	EECON1,EEPGD	;Point to program memory
	BCF	EECON1,CFGS	;Not configuration space
	BSF	EECON1,WREN	;Enable writes
	BSF	EECON1,LWLO	;Only Load Write Latches
	movlw	.32
	movwf	Param79
WriteProgBlock_L1	moviw	FSR1++
	movwf	EEDATL
	moviw	FSR1++
	movwf	EEDATH
;
	movf	EEADRL,W
	xorlw	0x1F
	andlw	0x1F
	btfsc	_Z
	bra	WriteProgBlock_Wr
;
	movlw	0x55	; Start of required write sequence:
	movwf	EECON2
	movlw	0xAA
	movwf	EECON2
	bsf	EECON1,WR	; Set WR bit to begin write
	nop
	nop
	incf	EEADRL,F
	bra	WriteProgBlock_L1
;
WriteProgBlock_Wr	bcf	EECON1,LWLO	; Write memory not latches
	movlw	0x55	; Start of required write sequence:
	movwf	EECON2
	movlw	0xAA
	movwf	EECON2
	bsf	EECON1,WR	; Set WR bit to begin write
	nop
	nop
;
	bcf	EECON1,WREN
;
	movlb	0
	return	
;
;============================================================
; call once
;
InitializeBLIO	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	bsf	WPUA,WPUA5	;Put a pull up on the MCLR unused pin.
	MOVLB	0x01	; bank 1
	MOVLW	OSCCON_Value
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON
;
	MOVLB	0x03	; bank 3
	movlw	ANSELA_Val
	movwf	ANSELA
	movlw	ANSELB_Val
	movwf	ANSELB
;
;Setup T2 for 100/s
;	movlb	0	; bank 0
;	MOVLW	T2CON_Value
;	MOVWF	T2CON
;	MOVLW	PR2_Value
;	MOVWF	PR2
;	movlb	1	; bank 1
;	bsf	PIE1,TMR2IE	; enable Timer 2 interupt
;
; setup timer 1 for 0.5uS/count
;
;	MOVLB	0x00	; bank 0
;	MOVLW	T1CON_Val
;	MOVWF	T1CON
;	bcf	T1GCON,TMR1GE	;always count
;
;SPI MISO >> SDI1 RB1, default
;SPI CLK >> RB4, default
;	movlb	2	;bank 2
;	bsf	APFCON0,RXDTSEL	;RX >> RB2
;	bsf	APFCON1,TXCKSEL	;TX >> RB5
;	bsf	APFCON0,SDO1SEL	;SPI MOSI >> SDO1 RA6
;	
; clear memory to zero
	CALL	ClearRam
	CLRWDT
	CALL	CopyToRam
;
; setup ccp1
;
;	BSF	ServoOff
;	BANKSEL	APFCON
;	BSF	APFCON,CCP1SEL	;CCP1 on RA5
;	BANKSEL	CCP1CON
;	CLRF	CCP1CON
;
;	MOVLB	0x01	;Bank 1
;	bsf	PIE1,CCP1IE
;
;
;	MOVLB	0x00	;Bank 0
; setup data ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA
	MOVLB	0x01	; bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
;
; setup serial I/O
	BANKSEL	BAUDCON	; bank 3
	movlw	BAUDCON_Value
	movwf	BAUDCON
	MOVLW	low BaudRate
	MOVWF	SPBRGL
	MOVLW	high BaudRate
	MOVWF	SPBRGH
	MOVLW	TXSTA_Value
	MOVWF	TXSTA
	MOVLW	RCSTA_Value
	MOVWF	RCSTA
	movlb	0x01	; bank 1
;	BSF	PIE1,RCIE	; Serial Receive interupt
	movlb	0x00	; bank 0
;
;
	CLRWDT
;-----------------------
;
;	MOVLB	0x00
;	MOVLW	LEDTIME
;	MOVWF	SysLED_Time
;	movlw	0x01
;	movwf	SysLEDCount	;start blinking right away
;	movlw	.100
;	movwf	Timer4Lo	;ignor buttons for 1st second
;
;	movf	SysMode,W
;	movwf	LED1_Blinks
;
;if mode 3 don't move
;	bsf	ssCmdPos+1,7
;
	CLRWDT
;
;	bsf	INTCON,PEIE	; enable periferal interupts
	bcf	INTCON,GIE	; disable interupts
;
	return
;
;
 



 



 

















