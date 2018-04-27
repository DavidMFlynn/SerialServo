;====================================================================================================
;
;    Filename:      RS232_Parse.inc
;    Date:          1/12/2016                                    
;    File Version:  1.0d1
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;
;====================================================================================================
;====================================================================================================
; Notes:
;
; History:
; 1.0d1  1/12/2016	Modified for I2C to RS-232 8 byte packets.
; 1.0d1  11/9/2015	Adapted for QD Tester
; 1.0    12/6/2013	First Rev'd version
;
; RS232_Parse	(X/0) Parse any bytes received by the serial port
; Packet format: FF FF SA0 SA1 DA0 DA1 D0 D1 D2 D3 D4 D5 D6 D7 CS
;  FF = Sync byte, SAn = Source Address, DA = Destination Address,
;       Dn = Data, CS Check Sum (xor of Address and Data bytes)
;  SA must be RS232_RAddr ( the boss )
;  DA must be RS232_MyAddr ( this program )
;  If the checksum matches put the data bytes in the I2C Tx buffer.
;=========================================================================================
; Bank0 variable used by RS232_Parse:
;
;	cblock
;	RX_ParseFlags
;	RX_Flags
;	RX_DataCount
;	RX_CSUM
;	RX_TempData:8
;	RX_Data:8
;	endc
;RX_ParseFlags Bits
;#Define	SyncByte1RXd	RX_ParseFlags,0
;#Define	SyncByte2RXd	RX_ParseFlags,1
;#Define	SourceAddLoRXd	RX_ParseFlags,2
;#Define	SourceAddHiRXd	RX_ParseFlags,3
;#Define	DestAddLoRXd	RX_ParseFlags,4
;#Define	DestAddHiRXd	RX_ParseFlags,5
;#Define	AllDataRXd	RX_ParseFlags,6
;
;RX_Flags Bits
;#Define	RXDataValidFlag	RX_Flags,0
;#Define	RXDataIsNew	RX_Flags,1
;
;
; *** put in main loop
;	CALL	RS232_Parse
; if RXDataValidFlag is set data may be used
;
;=========================================================================================
; Parse any bytes received by the serial port
;
; Receive 0xFF,Cmd,Data0,Data1,0xFF
; 
;
; Entry: none
; Exit: none
; Ram Used: Param78, FSR
; Calls (1+0): GetSerIn
;
RS232_Parse	CALL	GetSerIn
	BTFSS	Param78,0	;Got a byte?
	RETURN		; No
;
	MOVWF	Param78	;Data Byte RXd
; Command inturpreter
;
RS232_Parse_Sync	MOVF	RX_ParseFlags,F
	SKPZ		;Beginning new packet?
	BRA	RS232_Parse_Sync2	; No
	MOVLW	0xFF
	SUBWF	Param78,W
	SKPNZ		;Sync byte OK?
	BSF	SyncByte1RXd	; Yes
	return
;
RS232_Parse_Sync2	btfsc	SyncByte2RXd
	bra	RS232_Parse_SALo
	movlw	0xFF
	subwf	Param78,W
	SKPZ
	goto	RS232_Parse_Error
	bsf	SyncByte2RXd
	clrf	RX_CSUM
	clrf	RX_DataCount
	return
;	
RS232_Parse_SALo	btfsc	SourceAddLoRXd
	bra	RS232_Parse_SAHi
	movlw	low RS232_RAddr
	subwf	Param78,W
	SKPZ
	goto	RS232_Parse_Error
	bsf	SourceAddLoRXd
	movf	Param78,W
	xorwf	RX_CSUM,F
	return
;
RS232_Parse_SAHi	btfsc	SourceAddHiRXd
	bra	RS232_Parse_DALo
	movlw	high RS232_RAddr
	subwf	Param78,W
	SKPZ
	goto	RS232_Parse_Error
	bsf	SourceAddHiRXd
	movf	Param78,W
	xorwf	RX_CSUM,F
	return
;
RS232_Parse_DALo	btfsc	DestAddLoRXd
	bra	RS232_Parse_DAHi
	movlw	low RS232_MyAddr
	subwf	Param78,W
	SKPZ
	goto	RS232_Parse_Error
	bsf	DestAddLoRXd
	movf	Param78,W
	xorwf	RX_CSUM,F
	return
;
RS232_Parse_DAHi	btfsc	DestAddHiRXd
	bra	RS232_Parse_Data
	movlw	high RS232_MyAddr
	subwf	Param78,W
	SKPZ
	goto	RS232_Parse_Error
	bsf	DestAddHiRXd
	movf	Param78,W
	xorwf	RX_CSUM,F
	return
;
RS232_Parse_Data	btfsc	AllDataRXd
	bra	RS232_Parse_CSUM
	movf	RX_DataCount,W
	addlw	low RX_TempData
	movwf	FSR0L
	movlw	high RX_TempData
	movwf	FSR0H
	movf	Param78,W
	xorwf	RX_CSUM,F
	movwf	INDF0
	incf	RX_DataCount,F
	movlw	0x08
	subwf	RX_DataCount,W
	SKPNZ
	bsf	AllDataRXd
	return
;
RS232_Parse_CSUM	movf	Param78,W
	subwf	RX_CSUM,W
	SKPZ		;Checksum matches?
	bra	RS232_Parse_Error	; No
;
	movlw	low RX_TempData
	movwf	FSR0L
	movlw	high RX_TempData
	movwf	FSR0H
;
	movlw	low RX_Data
	movwf	FSR1L
	movlw	high RX_Data
	movwf	FSR1H
;
	movlw	0x08
	movwf	Param78
RS232_Parse_L1	moviw	FSR0++
	movwi	FSR1++
	decfsz	Param78,F
	bra	RS232_Parse_L1
;
	bsf	RXDataValidFlag
	bsf	RXDataIsNew
	clrf	RX_ParseFlags
	return
;
;
; We didn't get the sync byte we were expecting so restart the parser
RS232_Parse_Error	CLRF	RX_ParseFlags
	GOTO	RS232_Parse
;
;