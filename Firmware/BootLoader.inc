;************************************************
; BootLoader for PIC16F1847
; filename: Bootloader1847.inc
; Created: 3/21/2019
; Revision: 1.0a1 4/7/2019
;************************************************
; History
; 1.0a1 4/7/2019	First working version.
; 1.0d1 3/21/2019	first code
;************************************************
; Notes:
;
;  Does not program data EEProm, or other than prog mem 0..BootLoaderStart-1
;
; Must be the last thing in prog mem, will not overwrite itself or anything after it.
;
;BootLoaderStart	EQU	0x1E00
;	org BootLoaderStart
;	include <Bootloader1847.inc>
;
;  Reset vector:
;  	movlp	high BootLoaderStart
;    	goto	BootLoaderStart
;  ProgStartVector	CLRF	PCLATH
;	goto	Start
;
;  DataEEProm (0xFF) = 0xFF = run bootloader
;
; What happens next:
;  If Data EEProm address (0xFF) <> 0xFF then goto ProgStartVector
;  Setup  UART for 38.4KBaud
;  Parse incoming text as .hex file
;   if program memory address > 0 and < BootLoader then
;    read block from prog mem
;    if compare blocks then
;      if prog mem is not blank then erase block
;    write block
;
;  DataEEProm (0xFF) = 0x00
;
;  reset
;************************************************
; .hex
;:020000040000FA 
;       ^^^^^^ Set high word of address to 0x0000
;:100010005A23582356235423422021000C16200033
;:100020003A100C1E3A14A10B053221000C122000CC
;^ Start code                             ^^ Checksum
; ^^ Bytes of data (0x10)
;   ^^^^ Address low word (0x0020)
;       ^^ Record type 0x00 = Data
;         ^^^^ LSB MSB of data for Addr / 2
;                                         
; Checksum is 2's complement of all other bytes added together
; Valid address range 0000 0000 .. 0000 3BFE (bootloader is at 0000 3C00 .. 0000 3FFF, 512 words)
;************************************************
;
;
	cblock 0x20
	PROG_ADDR_LO
	PROG_ADDR_HI
	ProgWordCount
	
	IH_Addr_Lo:2		;
	IH_Addr_Hi:2		;extended linear address, alway 0x0000 for Prog mem
	IH_Bytes
	IH_ByteCount
	IH_Type		;Record Type 0..5
	IH_State
	IH_TempVal
	IH_Flags
	ChkSum
	endc
;
#Define	MSN_Done	IH_Flags,0	;Most Significant Nibble
#Define	DataInBuffer	IH_Flags,1
;
;IH_Type Types
IHT_Data	EQU	0x00
IHT_End	EQU	0x01	;ie :00000001FF
IHT_XSA	EQU	0x02	;extended segment address, not supported
IHT_SSA	EQU	0x03	;start segment address, not supported
IHT_HighAddr	EQU	0x04	;extended linear address, :02000004FFFFFC
IHT_SLA	EQU	0x05	;start linear address, not supported, :04000005000000CD2A
;
;IH_State values
IHS_WaitForColon	EQU	0x00
IHS_Length	EQU	0x01
IHS_AddrMSB	EQU	0x02
IHS_AddrLSB	EQU	0x03
IHS_DataType	EQU	0x04
IHS_Data	EQU	0x05
IHS_CSum	EQU	0x06	;waiting for check sum
;
IH_ErrorBadLen	EQU	'+'	;>64 Too big
IH_ErrorBadCSum	EQU	'-'	;Checksum didn't match
IH_Ack	EQU	'*'	;We got a good Intel Hex line
;
BlockSize	EQU	0x40
BlockWords	EQU	BlockSize/2	;0x20
AddressMask	EQU	0xE0
;
;Contiguous ram 0x2000..0x23EF
	cblock 0x2100		;0x0100 after start of ram
	SourceBuffer:BlockSize
	DestBuffer:BlockSize
	SerialBuffer:BlockSize
	endc
;================================================
;
; Entry: DataEEProm255
; Exit: goto ProgStartVector
; Calls: none
; RAM used: ALL
;
BootLoader:
	call	InitializeBLIO	;returns w/ bank 0 selected
;
	call	ReadEEData	;Get Data from EEPROM (0xFF) 
	xorlw	0xFF
	SKPZ		;0xFF xor 0xFF = 0
	bra	BootLoader_End
;
; Bootloader is active send "**"
	movlw	IH_Ack
	call	SendChar
	movlw	IH_Ack
	call	SendChar
;
;Initialize
	clrf	PROG_ADDR_LO
	clrf	PROG_ADDR_HI
	clrf	IH_Addr_Hi
	clrf	IH_Addr_Hi+1
	call	EraseSourceBuffer
;
	call	BL_Parse
;	
;if successful then clear EEProm(255)
	call	WriteEEData
;
BootLoader_End	movlp	0x00
	goto	ProgStartVector
;
;==============================================
; Parse incoming data and do stuff
;
BL_Parse	bcf	DataInBuffer
BL_ParseNextLine	clrf	IH_State
	clrf	IH_ByteCount
	clrf	ChkSum
	movlw	low SerialBuffer
	movwf	FSR0L
	movlw	high SerialBuffer
	movwf	FSR0H
;
BLP_L1	bcf	MSN_Done
;
BLP_L2	call	GetChar
	movwf	Param78
	movlw	':'
	subwf	Param78,W
	SKPZ		;Char=":"
	bra	BLP_TestNum	; No
; It's a ":" so set state=1, wait for length
	movlw	IHS_Length
	movwf	IH_State
	bra	BLP_L1
;
BLP_TestNum	movf	IH_State,F
	SKPNZ		;Are we waiting for a number?
	bra	BLP_L2	; No
;
	movlw	0x30	;'0'
	subwf	Param78,W
	SKPNB		;Char>="0"?
	bra	BL_ParseNextLine	; No start over
	movlw	0x3A	;':'
	subwf	Param78,W
	SKPNB
	bra	BLP_TestNum_1
	movlw	0x41	;'A'
	subwf	Param78,W
	SKPNB		;Char>="A"?
	bra	BL_ParseNextLine	; No start over
	movlw	0x47	;'G'
	subwf	Param78,W
	SKPB		;Char<"G"
	bra	BL_ParseNextLine	; No, start over
	movlw	0x07	; Yes, A..F
	subwf	Param78,F
;
BLP_TestNum_1	movlw	0x30
	subwf	Param78,F	;'0' >> 0x00
	btfsc	MSN_Done
	bra	BLP_TestNum_2	;do low nibble
	swapf	Param78,W
	movwf	IH_TempVal
	bsf	MSN_Done
	bra	BLP_L2
;
BLP_TestNum_2	movf	Param78,W
	addwf	IH_TempVal,F
;
; We have a number in IH_TempVal
	movlw	IHS_Length
	subwf	IH_State,W
	SKPZ		;Waiting for length byte?
	bra	BLP_IHS_AddrMSB	; No
	movlw	.65	; Yes, is it in range 0..64?
	subwf	IH_TempVal,W
	SKPNB		;<=64?
	bra	BLP_SaveIH_Length
;
	movlw	IH_ErrorBadLen
	call	SendChar
	goto	BL_ParseNextLine
;
BLP_SaveIH_Length	movf	IH_TempVal,W
	movwf	IH_Bytes
	addwf	ChkSum,F
BLP_NextState	incf	IH_State,F
	goto	BLP_L1
;
BLP_IHS_AddrMSB	movlw	IHS_AddrMSB
	subwf	IH_State,W
	SKPZ		;Waiting for Address MSB?
	bra	BLP_IHS_AddrLSB	; No
;
	movf	IH_TempVal,W
	movwf	IH_Addr_Lo+1
	addwf	ChkSum,F
	bra	BLP_NextState
;
BLP_IHS_AddrLSB	movlw	IHS_AddrLSB
	subwf	IH_State,W
	SKPZ		;Waiting for Address LSB?
	bra	BLP_IHS_DataType	; No
;
	movf	IH_TempVal,W
	movwf	IH_Addr_Lo
	addwf	ChkSum,F
	bra	BLP_NextState
;
BLP_IHS_DataType	movlw	IHS_DataType
	subwf	IH_State,W
	SKPZ		;Waiting for Data Type?
	bra	BLP_IHS_Data	; No
;
	movf	IH_TempVal,W
	movwf	IH_Type
	addwf	ChkSum,F
; Ship IHS_Data if length = 0
	movf	IH_Bytes,F
	SKPNZ		;Expecting zero length data?
	incf	IH_State,F	; Yes
	bra	BLP_NextState
;
BLP_IHS_Data	movlw	IHS_Data
	subwf	IH_State,W
	SKPZ		;Waiting for Data?
	bra	BLP_IHS_CSum	; No
;
	movf	IH_TempVal,W
	movwi	FSR0++
	addwf	ChkSum,F
	incf	IH_ByteCount,F
; if IH_ByteCount=IH_Bytes then next state
	movf	IH_ByteCount,W
	subwf	IH_Bytes,W
	SKPZ		;Done with data?
	goto	BLP_L1	; No
	bra	BLP_NextState	; Yes
;
BLP_IHS_CSum	movlw	IHS_CSum
	subwf	IH_State,W
	SKPZ		;Waiting for Checksum?
	goto	BL_ParseNextLine	; No, we should never get here
; get 2's comp
	clrf	Param78
	movf	ChkSum,W
	subwf	Param78,W
	subwf	IH_TempVal,W
	SKPZ		;Checksum matches?
	bra	BLP_BadCSum
	bra	BLP_Validate
;
BLP_BadCSum	movlw	IH_ErrorBadCSum
	call	SendChar
	goto	BL_ParseNextLine
;
;===================
; We have received a complete Intel Hex command
BLP_Validate:
	movlw	IHT_End
	subwf	IH_Type,W
	SKPZ		;End?
	bra	BLPV_Data	; No
; handle end command
	movlw	IH_Ack
	call	SendChar
	btfsc	DataInBuffer
	call	BLP_WriteBuffer
	return
;
BLPV_Data	movlw	IHT_Data
	subwf	IH_Type,W
	SKPZ		;Data?
	bra	BLPV_Addr	; No
; handle data
	call	BufferIHData
;
	movlw	IH_Ack
	call	SendChar
	goto	BL_ParseNextLine
;
BLPV_Addr	movlw	IHT_HighAddr
	subwf	IH_Type,W
	SKPZ		;High Address?
	goto	BL_ParseNextLine	; No
; handle high address change
	movlw	low SerialBuffer
	movwf	FSR0L
	movlw	high SerialBuffer
	movwf	FSR0H
	moviw	FSR0++
	movwf	IH_Addr_Hi+1
	moviw	FSR0++
	movwf	IH_Addr_Hi
	movlw	IH_Ack
	call	SendChar
	goto	BL_ParseNextLine
;
;===========================================
; Validate address is in range and move to SourceBuffer
;
; Entry: DataInBuffer, IH_ByteCount, Data in SerialBuffer, 32bit address IH_Addr_Hi:IH_Addr_Lo
; Exit:
; 
BufferIHData:
; check address range
	movf	IH_Addr_Hi,W
	iorwf	IH_Addr_Hi+1,W
	SKPZ		;High address = 0x0000 ?
	return		; No
; convert to words, IH_Addr_Lo = IH_Addr_Lo / 2
	lsrf	IH_Addr_Lo+1,F
	rrf	IH_Addr_Lo,F
	movlw	high BootLoaderStart
	subwf	IH_Addr_Lo+1,W
	SKPB		;BootLoaderStart>IH_Addr_Lo?
	return		; No
;
	btfss	DataInBuffer	;Any old data in the buffer?
	bra	BIHD_PutInBuff	; No
;
; Continue filling buffer or flush and add to new buffer?
;
	movf	IH_Addr_Lo+1,W
	subwf	PROG_ADDR_HI,W
	SKPZ		;Matches?
	bra	BIHD_FlushBuff	; No
;
	movlw	AddressMask
	andwf	PROG_ADDR_LO,W
	movwf	Param78
	movlw	AddressMask
	andwf	IH_Addr_Lo+1,W
	subwf	Param78,W
	SKPNZ		;Matches?
	bra	BIHD_PutInBuff_E2	; Yes
;
BIHD_FlushBuff	call	BLP_WriteBuffer
;
; Entry: IH_Addr_Lo=Word Address
;
BIHD_PutInBuff	movf	IH_Addr_Lo,W
	andlw	AddressMask
	movwf	PROG_ADDR_LO
	movf	IH_Addr_Lo+1,W
	movwf	PROG_ADDR_HI
	movlw	0xFF
	xorlw	AddressMask	;0x1F
	andwf	IH_Addr_Lo,W	;Offset within block
	movwf	ProgWordCount
;
BIHD_PutInBuff_E2	movlw	low SerialBuffer
	movwf	FSR0L
	movlw	high SerialBuffer
	movwf	FSR0H
;
; FSR1 = SourceBuffer + ProgWordCount * 2
	clrf	FSR1H
	lslf	ProgWordCount,W	;ProgWordCount * 2
	addlw	low SourceBuffer
	movwf	FSR1L
	movlw	high SourceBuffer
	addwfc	FSR1H,F
;
;Param79 = IH_ByteCount
	movf	IH_ByteCount,W
	movwf	Param79
;ProgWordCount += IH_ByteCount / 2
	lsrf	WREG,F	; divide by 2
	addwf	ProgWordCount,F
;
BIHD_PutInBuff_L1	moviw	FSR0++
	movwi	FSR1++
	decfsz	Param79,F
	bra	BIHD_PutInBuff_L1
;
	bsf	DataInBuffer
	movlw	BlockWords
	subwf	ProgWordCount,W
	SKPNZ		;buffer full?
	bra	BLP_WriteBuffer	; Yes
	SKPNB		;Overflow? This should not happen.
	return		; No
;
; fall thru to BLP_WriteBuffer
;=============================================
; Write SourceBuffer to Prog memory
;
; Entry: SourceBuffer,
;
BLP_WriteBuffer:
; copy old prog mem to DestBuffer
	movlw	low DestBuffer
	movwf	FSR1L
	movlw	high DestBuffer
	movwf	FSR1H
	call	ReadProgBlock
;
	movlw	low SourceBuffer	;new data
	movwf	FSR1L
	movlw	high SourceBuffer
	movwf	FSR1H	
	movlw	low DestBuffer	;old data
	movwf	FSR1L
	movlw	high DestBuffer
	movwf	FSR1H
	call	CompareBlock
;
	btfsc	WREG,0	;Matches?
	bra	BLP_WriteBuffer_end	; Yes
;	
	movlw	low DestBuffer
	movwf	FSR1L
	movlw	high DestBuffer
	movwf	FSR1H
	call	TestErased
	btfss	WREG,0	;Erased?
;
	call	EraseProgBlock	; No, Erase it.
;
	movlw	low SourceBuffer
	movwf	FSR1L
	movlw	high SourceBuffer
	movwf	FSR1H	
	call	WriteProgBlock
;
BLP_WriteBuffer_end	bcf	DataInBuffer
	call	EraseSourceBuffer
;
	return
;==============================================
; Get a byte from UART, waits forever
;
; Entry: none
; Exit: W=Char
;
GetChar	MOVLB	0x00
	BTFSS	PIR1,RCIF	;RX has a byte?
	BRA	GetChar
;
	BANKSEL	RCSTA	; bank 3
	MOVF	RCSTA,W
	ANDLW	0x06
	SKPZ		;Frame or Overrun Err?
	BRA	GetChar_Err	; yes
	MOVF	RCREG,W
;
	movlb	0x00	; bank 0
;	goto	SendChar	; tc
	RETURN
;
GetChar_Err	MOVF	RCREG,W	;bad byte so drop it
	BCF	RCSTA,CREN
	BCF	RCSTA,FERR
	BCF	RCSTA,OERR
	BSF	RCSTA,CREN
	bra	GetChar
;
;==============================================
; Send a byte to UART
;
; Entry: W=Char to send
SendChar:
	BTFSS	PIR1,TXIF	;TX done?
	bra	SendChar
	BANKSEL	TXREG
	MOVWF	TXREG
	MOVLB	0
	RETURN
;	
;==============================================
; Read EEProm(255)
;
ReadEEData:	
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
; Clear EEProm(255)
;
WriteEEData:
	BANKSEL	EEADRL
	MOVLW	0xFF
	MOVWF	EEADRL
	movlw	0x00
	movwf	EEDATL
	BCF	EECON1,CFGS	;not config mem
	BCF	EECON1,EEPGD	;not prog mem
	BSF	EECON1,WREN
	movlw	0x55
	movwf	EECON2
	movlw	0xAA
	movwf	EECON2
	bsf	EECON1,WR
WriteEEData_L1	btfsc	EECON1,WR
	bra	WriteEEData_L1
	bcf	EECON1,WREN
	movlb	0
	return
;
;==============================================
; Test if block is erased
;
; Entry: FSR1 -> current data
; Exit: W=1 if new = blank else W=0
;
TestErased	movlw	BlockWords
	movwf	Param79
TestErased_L1	moviw	FSR1++	;LSB
	sublw	0xFF
	SKPZ		;=0xFF?
	retlw	0x00	; No
;
	moviw	FSR1++	;MSB
	sublw	0x3F
	SKPZ		;=0x3F?
	retlw	0x00	; No
;
	decfsz	Param79,F
	bra	TestErased_L1
	retlw	0x01
;
;==============================================
; Compare Block
;
; Entry: FSR0 -> new data, FSR1 -> current data
; Exit: W=1 if new = current else W=0
;
CompareBlock	movlw	BlockWords
	movwf	Param79
CompareBlock_L1	moviw	FSR0++	;LSB
	movwf	Param77
	moviw	FSR0++	;MSB
	movwf	Param78
; skip?
	sublw	0x3F
	SKPZ
	bra	CB_NotBlank
	incf	Param77,W
	SKPZ
	bra	CB_NotBlank
	moviw	FSR1++
	moviw	FSR1++
	bra	CB_Next
;
CB_NotBlank	moviw	FSR1++
	subwf	Param77,W
	SKPZ		;Matches?
	retlw	0x00	; No
	moviw	FSR1++
	subwf	Param78,W
	SKPZ		;Matches?
	retlw	0x00	; No
;
CB_Next	decfsz	Param79,F
	bra	CompareBlock_L1
	retlw	0x01	;It's a match!
;
;==============================================
EraseSourceBuffer	movlw	low SourceBuffer
	movwf	FSR1L
	movlw	high SourceBuffer
	movwf	FSR1H
;
; Fall thru to EraseBlock
;==============================================
; Erase Block
;
; Entry: FSR1
;
EraseBlock	movlw	BlockWords
	movwf	Param79
EraseBlock_L1	movlw	0xFF
	movwi	FSR1++
	movlw	0x3F
	movwi	FSR1++
	decfsz	Param79,F
	bra	EraseBlock_L1
	return
;	
;==============================================
; Read Program memory block
;
; Entry: FSR1,PROG_ADDR_HI:PROG_ADDR_LO
; Exit: FSR1 changed, 32 words >> (FSR1++)
;
ReadProgBlock	call	LoadProgAddr
	BCF	EECON1,CFGS	; Do not select Configuration Space
	BSF	EECON1,EEPGD	; Select Program Memory
	movlw	BlockWords	;Block is .32 words
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
EraseProgBlock	call	LoadProgAddr
	BSF	EECON1,EEPGD	;Point to program memory
	BCF	EECON1,CFGS	;Not configuration space
	BSF	EECON1,FREE	;Specify an erase operation, automaticaly cleared
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
;=========================================================
; Load the program address PROG_ADDR_HI:PROG_ADDR_LO into EEADRH:EEADRL
LoadProgAddr	movf	PROG_ADDR_HI,W
	BANKSEL	EEADRH	;bank ?
	MOVWF	EEADRH	; Store MSB of address
	movlb	0	;bank 0
	movf	PROG_ADDR_LO,W
	BANKSEL	EEADRH	;bank ?
	MOVWF	EEADRL	; Store LSB of address
	return
;
;==========================================================
; Write Program memory block
;
; Entry: PROG_ADDR_HI:PROG_ADDR_LO, FSR1
; Exit: FSR1 changed
; 
WriteProgBlock	call	LoadProgAddr
	BSF	EECON1,EEPGD	;Point to program memory
	BCF	EECON1,CFGS	;Not configuration space
	BSF	EECON1,WREN	;Enable writes
	BSF	EECON1,LWLO	;Only Load Write Latches
	movlw	BlockWords
	movwf	Param79
;
WriteProgBlock_L1	moviw	FSR1++
	movwf	EEDATL
	moviw	FSR1++
	movwf	EEDATH
;
	movf	EEADRL,W
	xorlw	0x1F
	andlw	0x1F
	btfsc	_Z	;EEADRL = xxx1 1111?
	bra	WriteProgBlock_Wr	; Yes
;
	movlw	0x55	; Start of required write sequence:
	movwf	EECON2
	movlw	0xAA
	movwf	EECON2
	bsf	EECON1,WR	; Set WR bit to begin write to latches
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
WriteProgBlock_end	movlb	0
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
	MOVLW	OSCCON_Value
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON
;
	movlb	4	; bank 4
	bsf	WPUA,WPUA5	;Put a pull up on the MCLR unused pin.
;
	MOVLB	0x03	; bank 3
	movlw	ANSELA_Val
	movwf	ANSELA
	movlw	ANSELB_Val
	movwf	ANSELB
;
;
;SPI MISO >> SDI1 RB1, default
;SPI CLK >> RB4, default
	movlb	2	;bank 2
	bsf	APFCON0,RXDTSEL	;RX >> RB2
	bsf	APFCON1,TXCKSEL	;TX >> RB5
	bsf	APFCON0,SDO1SEL	;SPI MOSI >> SDO1 RA6
;
	MOVLB	0x00	;Bank 0
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
;
;	bsf	INTCON,PEIE	; enable periferal interupts
	bcf	INTCON,GIE	; disable interupts
;
	return
;
;
 



 



 

















