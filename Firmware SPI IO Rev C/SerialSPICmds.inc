;=========================================================================================
;
;    Filename:      SerialServoCmds.inc
;    Date:          6/1/2018
;    File Version:  1.1
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
; Notes:
;  Command interpreter for SerialServo
;
; History
; 1.1    6/1/2018	All commands are here now.
; 1.0    5/25/2018	Moved out of SerialServo.asm
;=========================================================================================
;
kCmd_SetMode	EQU	0x81	;+1 data (SysMode), return ACK
kCmd_GetMode	EQU	0x01
kCmd_SPIInOut          EQU                    0x82
;
;
;
kCmd_SaveParams	EQU	0x94	;Save all eeprom params, return ACK
kCmd_RestoreParams	EQU	0x95	;Copy to ram, return ACK
kCmd_GetStatus	EQU	0x98	;return ssStatus
kCmd_ReBoot	EQU	0x99	;ReBoot the controller
kCmd_RunBootloader	EQU	0x9A	;Set EEPROM (0xFF) = 0xFF and reboot
;
; these commands save params and return a 0,0,0,0 packet with the new address
kCmd_SetMasterAddr	EQU	0xA1	;+1 data, return a zero packet
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;Special packet (0xDD, 0xDD, 0x01, 0x02, 0xAA, 0x00, 0x00, 0x00, checksum)
kCmd_SendIDString	EQU	0xAA	; returns "DMFE SPI\n"
;
;=========================================================================================
;
RS232_Send_S0	mLongCall	RS232_Send
	return
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRXData	movlb	1	;bank 1
	bcf	RXDataIsNew
	btfsc	RXDataValidFlag	;from master to me?
	bra	HandleRXData_1
HandleRXData_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;---kCmd_SetMode-------------------
HandleRXData_1	movlb	0	;bank 0
                       mMOVLF	RX_Data,FSR1
	mMOVLF	TX_Data,FSR0
;Dispatcher
;
	moviw	FSR1++
	movwf	Param78	;aka RX_Data
	
;
	movlw	kCmd_SetMode
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMode_end
; Set Mode
	movlw	kMaxMode+1
	subwf	INDF1,W
	SKPB		;kMaxMode+1>Data
	bra	HandleRXData_Rtn
;
	moviw	FSR1++
	movwf	SysMode
	goto	TX_ACK
;
Cmd_SetMode_end:
;---kCmd_GetMode------------------
	movlw	kCmd_GetMode
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetMode_end
; Get Mode
	movf	SysMode,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetMode_end:
;---kCmd_SPIInOut-------------------
                       movlw                  kCmd_SPIInOut
                       subwf                  Param78,W
                       SKPZ
                       bra                    Cmd_SPIInOut_end
;
                       mMOVLF                 SPI_OutBuff,FSR0
                       moviw                  0[FSR1]                ;Address
                       movwi                  0[FSR0]
                       moviw                  4[FSR1]                ;MSB
                       movwi                  1[FSR0]
                       moviw                  3[FSR1]
                       movwi                  2[FSR0]
                       moviw                  2[FSR1]
                       movwi                  3[FSR0]
                       moviw                  1[FSR1]                ;LSB
                       movwi                  4[FSR0]
                       call                   SPI_SendRecWord
                       mMOVLF	TX_Data,FSR0
                       mMOVLF                 SPI_InBuff,FSR1
                       moviw                  0[FSR1]                ;Address
                       movwi                  0[FSR0]
                       moviw                  4[FSR1]                ;LSB
                       movwi                  1[FSR0]
                       moviw                  3[FSR1]
                       movwi                  2[FSR0]
                       moviw                  2[FSR1]
                       movwi                  3[FSR0]
                       moviw                  1[FSR1]                ;MSB
                       movwi                  4[FSR0]
                       goto	RS232_Send_S0
Cmd_SPIInOut_end:
;---kCmd_SaveParams-----------------
	movlw	kCmd_SaveParams
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SaveParams_end
	mLongCall	SaveParams
	goto	TX_ACK
Cmd_SaveParams_end:
;---kCmd_RestoreParams--------------
	movlw	kCmd_RestoreParams
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_RestoreParams_end
	mLongCall	CopyToRam
	goto	TX_ACK
Cmd_RestoreParams_end:
;---kCmd_GetStatus----------------
	movlw	kCmd_GetStatus
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetStatus_end
	movf	ssStatus,W
	movwi	FSR0++
	movf	ssStatus+1,W
	movwi	FSR0++
	movf	ssStatus+2,W
	movwi	FSR0++
	movf	ssStatus+3,W
	movwi	FSR0++
;reset flags
	bcf	ssRX_Timeout
	bcf	ssGripMCur
	bcf	ssGripOCur
;
	clrf	ssStatus+1
;	
	goto	RS232_Send_S0
Cmd_GetStatus_end:
;---kCmd_ReBoot---------------------
	movlw	kCmd_ReBoot
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_ReBoot_end
	reset		;Does not return anything
Cmd_ReBoot_end:
;---kCmd_RunBootloader--------------
	movlw	kCmd_RunBootloader
	subwf	Param78,W
	SKPZ
	bra	Cmd_RunBootloader_end
	movlw	0xFF
	movwf	EEDataTemp
	mLongCall	EEWriteW	;Write EEPROM address in W
	reset
Cmd_RunBootloader_end:
;---kCmd_SetMasterAddr------------
	movlw	kCmd_SetMasterAddr
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMasterAddr_end
	moviw	FSR1++
	movwf	RS232_MasterAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
Cmd_SetMasterAddr_end:
;---kCmd_SetSlaveAddr-------------
	movlw	kCmd_SetSlaveAddr
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetSlaveAddr_end
	moviw	FSR1++
	movwf	RS232_SlaveAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
Cmd_SetSlaveAddr_end:
;---kCmd_SendIDString------------
	movlw	kCmd_SendIDString
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SendIDString_end
	clrf	Param79
Cmd_SendIDString_L1	movf	Param79,W
	call	IDString
	iorlw	0x00
	SKPNZ
	return
	call	StoreSerOut_S0
	incf	Param79,F
	bra	Cmd_SendIDString_L1
;
IDString	brw
	retlw	'D'
	retlw	'M'
	retlw	'F'
	retlw	'E'
	retlw	' '
	retlw	'S'
	retlw	'P'
	retlw	'I'
	retlw	'\n'
	retlw	0x00
;
Cmd_SendIDString_end:
;--------------------------------
	return
;
TX_ACK	movlw	0xFF
StoreSerOut_S0	mLongCall	StoreSerOut
	return
;
;=========================================================================================
