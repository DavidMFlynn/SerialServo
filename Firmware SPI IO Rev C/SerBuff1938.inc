;=========================================================================================
;
;    Filename:      SerBuff.inc
;    Date:          5/24/2018                                    
;    File Version:  1.0.5
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;
;=========================================================================================
;=========================================================================================
; Notes:
;
; History:
; 1.0.5    5/24/2018   Added GetSerInBytes.
; 1.0.4    5/23/2018	Added FlushInBuff.
; 1.0.3    12/15/2015	Optimized some code to be segment independent.
; 1.0.2    10/23/2015	Modified for 'F1938
; 1.0.1    12/6/2013	Modified for 'F1847
; 1.0      5/25/2008	First Rev'd version
;
;=========================================================================================
; Bank0 Variables:
;
;	cblock	0x20
;	TXByte		;Next byte to send
;	RXByte		;Last byte received
;	SerFlags
;	endc
;
;#Define	DataReceivedFlag	SerFlags,1
;#Define	DataSentFlag	SerFlags,2
;
; Bank2 Variables:
;#Define	Ser_Buff_Bank	2
;	cblock	0x120
;	Ser_In_Bytes		;Bytes in Ser_In_Buff
;	Ser_Out_Bytes		;Bytes in Ser_Out_Buff
;	Ser_In_InPtr
;	Ser_In_OutPtr
;	Ser_Out_InPtr
;	Ser_Out_OutPtr
;	Ser_In_Buff:20
;	Ser_Out_Buff:20
;	endc
;
;=========================================================================================
; Routines	(Bank In/Bank Out)
;
; TX_TheByte	(0+0) Send one byte to UART
; RX_TheByte	(0+0) Receive one byte from UART
; StoreSerIn	(X/0) Put the byte in W into the serial input buffer, FSR0
; GetSerIn	(X/0) Get a byte from the serial input buffer, FSR0
; FlushInBuff	(X/0) Flush the serial incoming buffer.
; GetSerInBytes	(X/0) Get the number of bytes in the serial input buffer
; GetSerOutBytes	(X/0) Get the number of bytes in the serial ouput buffer
; StoreSerOut	(X/0) Put the byte in W into the serial output buffer, FSR0
; POP_SerOut	(X/0) Remove the last char stored in the output buffer
; GetSerOut	(X/0) Get a byte from the serial Output buffer, FSR0
;	       W=Byte, Param78:0=ValidFlag
;=========================================================================================
;=========================================================================================
; *** Put in main loop ***
;
; Handle Serial Communications
;	BTFSC	PIR1,TXIF	;TX done?
;	CALL	TX_TheByte	; Yes
;
; move any serial data received into the 32 byte input buffer
;	BTFSS	DataReceivedFlag
;	BRA	ML_Ser_Out
;	MOVF	RXByte,W
;	BCF	DataReceivedFlag
;	CALL	StoreSerIn
;
;=========================================================================================
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
;ML_Ser_Out	BTFSS	DataSentFlag
;	BRA	ML_Ser_End
;	CALL	GetSerOut
;	BTFSS	Param78,0
;	BRA	ML_Ser_End
;	MOVWF	TXByte
;	BCF	DataSentFlag
;ML_Ser_End
;
;=========================================================================================
; *** Put in Init section ***
;BAUDCON_Value	EQU	b'00001000'
;TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, High speed
;RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
;; 8MHz clock low speed (BRGH=0,BRG16=1)
;;Baud_300	EQU	d'1666'	;0.299, -0.02%
;;Baud_1200	EQU	d'416'	;1.199, -0.08%
;;Baud_2400	EQU	d'207'	;2.404, +0.16%
;;Baud_9600	EQU	d'51'	;9.615, +0.16%
;; 8MHz clock high speed (BRGH=1,BRG16=1)
;Baud_300	EQU	d'6666'	;0.299, -0.02%
;Baud_1200	EQU	d'1666'	;1.199, -0.08%
;Baud_2400	EQU	d'832'	;2.404, +0.16%
;Baud_9600	EQU	d'207'	;9.615, +0.16%
;Baud_19_2	EQU	d'103'	;19.23k, +0.16
;Baud_57_6	EQU	d'34'	;57.14k, -0.79
;BaudRate	EQU	Baud_9600
;-------------
;
; setup serial I/O
;	movlb	0x03	; bank 3
;	MOVLW	TXSTA_Value
;	MOVWF	TXSTA
;	MOVLW	low BaudRate
;	MOVWF	SPBRGL
;	MOVLW	high BaudRate
;	MOVWF	SPBRGH
;	MOVLW	RCSTA_Value
;	MOVWF	RCSTA
;
;	movlb	0x01	; bank 1
;	BSF	PIE1,RCIE	; Serial Receive interupt
;	movlb	0x00	; bank 0
;	BSF	INTCON,PEIE	;Peripheral IRQ Enable
;	BSF	INTCON,GIE
;
;=========================================================================================
; *** Put in ISR ***
;-----------------------------------------------------------------------------------------
;AUSART Serial ISR
;
;IRQ_Ser	BTFSS	PIR1,RCIF	;RX has a byte?
;	BRA	IRQ_Ser_End
;	CALL	RX_TheByte
;
;IRQ_Ser_End
;
;=========================================================================================
; Send one byte
; Entry: Bank0
; Exit: Bank0
;
;
TX_TheByte	BTFSC	DataSentFlag
	RETURN
	MOVF	TXByte,W	
	BANKSEL	TXREG
	MOVWF	TXREG
	MOVLB	0
	BSF	DataSentFlag
	RETURN
;
;=========================================================================================
;
RX_TheByte:
	BANKSEL	RCSTA	; bank 3
	MOVF	RCSTA,W
	ANDLW	0x06
	SKPZ		;Frame or Overrun Err?
	BRA	RX_TheByte_Err	; yes
	MOVF	RCREG,W
;
	movlb	0x00	; bank 0
	MOVWF	RXByte
	BSF	DataReceivedFlag
	RETURN
;
RX_TheByte_Err	MOVF	RCREG,W	;bad byte so drop it
	CLRF	RXByte
	BCF	RCSTA,CREN
	BCF	RCSTA,FERR
	BCF	RCSTA,OERR
	BSF	RCSTA,CREN
	movlb	0x00
	RETURN
;
;=========================================================================================
;=========================================================================================
; Put the byte in W into the serial input buffer
;
; Entry: W = byte to store
; Exit: none
; Ram Used: Param78, FSR
;
StoreSerIn	MOVWF	Param78
	MOVLB	Ser_Buff_Bank
	INCF	Ser_In_Bytes,F
;	
	MOVF	Ser_In_InPtr,W
	ANDLW	0x1F
	INCF	Ser_In_InPtr,F
	ADDLW	low Ser_In_Buff
	MOVWF	FSR0
	MOVLW	high Ser_In_Buff
	MOVWF	FSR0H
	MOVF	Param78,W
	MOVWF	INDF0
	MOVLB	0x00
	RETURN
;
;=========================================================================================
; Get a byte from the serial input buffer
;
; Entry: none
; Exit: W=byte from buffer, Param78,0=valid
;
GetSerIn	MOVLB	Ser_Buff_Bank
	BCF	Param78,0	;no data
	MOVF	Ser_In_Bytes,F
	SKPNZ
	bra	GetSerIn_End
	BSF	Param78,0
	DECF	Ser_In_Bytes,F
	MOVF	Ser_In_OutPtr,W
	ANDLW	0x1F
	INCF	Ser_In_OutPtr,F
	ADDLW	low Ser_In_Buff
	MOVWF	FSR0L
	MOVLW	high Ser_In_Buff
	MOVWF	FSR0H
	MOVF	INDF0,W
GetSerIn_End	movlb	0
	return
;
;=========================================================================================
; Flush the serial incoming buffer
; Entry: none
; Exit: bank 0
;
FlushInBuff	movlb	Ser_Buff_Bank
	clrf	Ser_In_Bytes
	clrf	Ser_In_OutPtr
	clrf	Ser_In_InPtr
	movlb	0
	return
;
;=========================================================================================
; Get the number of bytes in the serial input buffer
;
; Entry: none
; Exit: W=bytes in buffer, Z
;
GetSerInBytes	MOVLB	Ser_Buff_Bank
	MOVF	Ser_In_Bytes,W
	movlb	0
	return
;
;=========================================================================================
; Get the number of bytes in the serial ouput buffer
;
; Entry: none
; Exit: W=bytes in buffer, Z
;
GetSerOutBytes	MOVLB	Ser_Buff_Bank
	MOVF	Ser_Out_Bytes,W
	movlb	0
	return
;
;=========================================================================================
; Put the byte in W into the serial output buffer
;
; Entry: W = byte to store
; Exit: Param78 & W = byte stored, Bank 0
; Ram used: Param78, FSR0
; Calls: (0) none
;
StoreSerOut	MOVWF	Param78
	MOVLB	Ser_Buff_Bank
	INCF	Ser_Out_Bytes,F
;	
	MOVF	Ser_Out_InPtr,W
	ANDLW	0x1F
	INCF	Ser_Out_InPtr,F
	ADDLW	low Ser_Out_Buff
	MOVWF	FSR0
	MOVLW	high Ser_In_Buff
	MOVWF	FSR0H
	MOVF	Param78,W
	MOVWF	INDF0
	movlb	0
	return
;
;=========================================================================================
; Remove the last char stored in the output buffer
;
POP_SerOut	MOVLB	Ser_Buff_Bank
	MOVF	Ser_Out_Bytes,F
	SKPNZ
	bra	POP_SerOut_End
	DECF	Ser_Out_Bytes,F
	DECF	Ser_Out_InPtr,F
POP_SerOut_End	movlb	0
	return
;
;=========================================================================================
; Get a byte from the serial Output buffer
;
; Entry: none
; Exit: W=byte from buffer, Param78,0=valid
;
GetSerOut	MOVLB	Ser_Buff_Bank
	BCF	Param78,0	;no data
	MOVF	Ser_Out_Bytes,F
	SKPNZ
	bra	GetSerOut_End
	BSF	Param78,0
	DECF	Ser_Out_Bytes,F
	MOVF	Ser_Out_OutPtr,W
	ANDLW	0x1F
	INCF	Ser_Out_OutPtr,F
	ADDLW	low Ser_Out_Buff
	MOVWF	FSR0L
	MOVLW	high Ser_In_Buff
	MOVWF	FSR0H
	MOVF	INDF0,W
GetSerOut_End	movlb	0
	return
;
;=========================================================================================
;


















