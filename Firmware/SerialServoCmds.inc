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
kCmd_SetCmdPos	EQU	0x82	;+2 data (ssCmdPos), return ACK
kCmd_GetCmdPos	EQU	0x02
kCmd_SetMaxI	EQU	0x83	;+1 data (ssMaxI), return ACK
kCmd_GetMaxI	EQU	0x03
kCmd_SetFFwd	EQU	0x84	;+2 data (ServoFastForward), return ACK
kCmd_GetFFwd	EQU	0x04
kCmd_SetFRev	EQU	0x85	;+2 data (ServoFastReverse), return ACK
kCmd_GetFRev	EQU	0x05
kCmd_SetMin_uS	EQU	0x86	;+2 data (ServoMin_uS), return ACK
kCmd_GetMin_uS	EQU	0x06
kCmd_SetMax_uS	EQU	0x87	;+2 data (ServoMax_uS), return ACK
kCmd_GetMax_uS	EQU	0x07
kCmd_SetRevDir	EQU	0x88	;+1 data (ssReverseDir), return ACK
kCmd_GetRevDir	EQU	0x08
kCmd_SetEnaOvrCur	EQU	0x89	;+1 data (ssEnableOverCur), return ACK
kCmd_GetEnaOvrCur	EQU	0x09
kCmd_Set_HighZ_TX	EQU	0x8A	;+1 data (ssEnableHighZTZ), return ACK
kCmd_Get_HighZ_TX	EQU	0x0A
kCmd_SetStopCenter	EQU	0x8B	;+2 data (ServoStopCenter), return ACK
kCmd_GetStopCenter	EQU	0x0B
kCmd_SetSpeed	EQU	0x8C	;+1 data (ServoSpeed), return ACK
kCmd_GetSpeed	EQU	0x0C
kCmd_SetUseIdleCenter	EQU	0x8D	;+1 data (ssMode3IdleCenter), return ACK
kCmd_GetUseIdleCenter	EQU	0x0D
kCmd_SetDeadBand	EQU	0x8E	;+1 data (DeadBand), return ACK
kCmd_GetDeadBand	EQU	0x0E
;
kCmd_GetI	EQU	0x91	;return Cur_AN0
kCmd_GetEnc	EQU	0x92	;return EncoderVal
kCmd_GetEncAbs	EQU	0x93	;return EncoderAccum
kCmd_SaveParams	EQU	0x94	;Save all eeprom params, return ACK
kCmd_RestoreParams	EQU	0x95	;Copy to ram, return ACK
kCmd_GetBattVolts	EQU	0x96	;return Cur_AN7
kCmd_GetCalPot	EQU	0x97	;return Cur_AN4
;
; these commands save params and return a 0,0,0,0 packet with the new address
kCmd_SetMasterAddr	EQU	0xA1	;+1 data, return a zero packet
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRXData	bcf	RXDataIsNew
	btfss	RXDataValidFlag	;from master to me?
	return		; no, ignore this packet
;---kCmd_SetMode-------------------
	movlw	kCmd_SetMode
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetMode_end
; Set Mode
	movlw	kMaxMode+1
	subwf	RX_Data+1,W
	SKPB		;kMaxMode+1>Data
	return
;
	movf	RX_Data+1,W
	movwf	SysMode
	movwf	LED1_Blinks
	goto	TX_ACK
;
Cmd_SetMode_end:
;---kCmd_GetMode------------------
	movlw	kCmd_GetMode
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetMode_end
; Get Mode
	movf	SysMode,W
	movwf	TX_Data
	goto	RS232_Send
;
Cmd_GetMode_end:
;---kCmd_SetCmdPos-------------------
	movlw	kCmd_SetCmdPos
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetCmdPos_end
; Set Command Position
	movf	RX_Data+1,W
	movwf	ssCmdPos
	movf	RX_Data+2,W
	movwf	ssCmdPos+1
	goto	TX_ACK
;
Cmd_SetCmdPos_end:
;---kCmd_GetCmdPos------------------
	movlw	kCmd_GetCmdPos
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetCmdPos_end
; Get Command Position
	movf	ssCmdPos,W
	movwf	TX_Data
	movf	ssCmdPos+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetCmdPos_end:
;---kCmd_SetMaxI-------------------
	movlw	kCmd_SetMaxI
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetMaxI_end
; Set Max Current
	movf	RX_Data+1,W
	movwf	ssMaxI
	goto	TX_ACK
;
Cmd_SetMaxI_end:
;---kCmd_GetMaxI-------------------
	movlw	kCmd_GetMaxI
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetMaxI_end
; Get Mode
	movf	ssMaxI,W
	movwf	TX_Data
	goto	RS232_Send
;
Cmd_GetMaxI_end:
;---kCmd_SetFFwd-------------------
	movlw	kCmd_SetFFwd
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetFFwd_end
; Set ServoFastForward
	movf	RX_Data+1,W
	movwf	ServoFastForward
	movf	RX_Data+2,W
	movwf	ServoFastForward+1
	goto	TX_ACK
;
Cmd_SetFFwd_end:
;---kCmd_GetFFwd-------------------
	movlw	kCmd_GetFFwd
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetFFwd_end
; Get ServoFastForward
	movf	ServoFastForward,W
	movwf	TX_Data
	movf	ServoFastForward+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetFFwd_end:
;---kCmd_SetFRev-------------------
	movlw	kCmd_SetFRev
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetFRev_end
; Set ServoFastReverse
	movf	RX_Data+1,W
	movwf	ServoFastReverse
	movf	RX_Data+2,W
	movwf	ServoFastReverse+1
	goto	TX_ACK
;
Cmd_SetFRev_end:
;---kCmd_GetFRev------------------
	movlw	kCmd_GetFRev
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetFRev_end
; Get ServoFastReverse
	movf	ServoFastReverse,W
	movwf	TX_Data
	movf	ServoFastReverse+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetFRev_end:
;---kCmd_SetMin_uS-------------------
	movlw	kCmd_SetMin_uS
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetMin_uS_end
; Set ServoMin_uS
	movf	RX_Data+1,W
	movwf	ServoMin_uS
	movf	RX_Data+2,W
	movwf	ServoMin_uS+1
	goto	TX_ACK
;
Cmd_SetMin_uS_end:
;---kCmd_GetMin_uS----------------
	movlw	kCmd_GetMin_uS
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetMin_uS_end
; Get ServoMin_uS
	movf	ServoMin_uS,W
	movwf	TX_Data
	movf	ServoMin_uS+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetMin_uS_end:
;---kCmd_SetMax_uS-------------------
	movlw	kCmd_SetMax_uS
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetMax_uS_end
; Set ServoMax_uS
	movf	RX_Data+1,W
	movwf	ServoMax_uS
	movf	RX_Data+2,W
	movwf	ServoMax_uS+1
	goto	TX_ACK
;
Cmd_SetMax_uS_end:
;---kCmd_GetMax_uS-----------------
	movlw	kCmd_GetMax_uS
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetMax_uS_end
; Get ServoMax_uS
	movf	ServoMax_uS,W
	movwf	TX_Data
	movf	ServoMax_uS+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetMax_uS_end:
;---kCmd_SetRevDir---------------
	movlw	kCmd_SetRevDir
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetRevDir_End
	bcf	ssReverseDir
	movlw	0x01
	xorwf	RX_Data+1,W
	SKPNZ
	bsf	ssReverseDir
	goto	TX_ACK
Cmd_SetRevDir_End:
;---kCmd_GetRevDir---------------
	movlw	kCmd_GetRevDir
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetRevDir_End
	btfsc	ssReverseDir
	bsf	TX_Data,0
	goto	RS232_Send
Cmd_GetRevDir_End:
;---kCmd_SetEnaOvrCur------------
	movlw	kCmd_SetEnaOvrCur
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetEnaOvrCur_End
	bcf	ssEnableOverCur
	movlw	0x01
	xorwf	RX_Data+1,W
	SKPNZ
	bsf	ssEnableOverCur
	goto	TX_ACK
Cmd_SetEnaOvrCur_End:
;---kCmd_GetEnaOvrCur------------
	movlw	kCmd_GetEnaOvrCur
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetEnaOvrCur_End
	btfsc	ssEnableOverCur
	bsf	TX_Data,0
	goto	RS232_Send
Cmd_GetEnaOvrCur_End:
;---kCmd_Set_HighZ_TX-------------
	movlw	kCmd_Set_HighZ_TX
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_Set_HighZ_TX_End
	bcf	ssEnableHighZTZ
	movlw	0x01
	xorwf	RX_Data+1,W
	SKPNZ
	bsf	ssEnableHighZTZ
	goto	TX_ACK
Cmd_Set_HighZ_TX_End:
;---kCmd_Get_HighZ_TX-------------
	movlw	kCmd_Get_HighZ_TX
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_Get_HighZ_TX_End
	btfsc	ssEnableHighZTZ
	bsf	TX_Data,0
	goto	RS232_Send
Cmd_Get_HighZ_TX_End:
;---kCmd_SetStopCenter---------
	movlw	kCmd_SetStopCenter
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetStopCenter_End
	movf	RX_Data+1,W
	movwf	ServoStopCenter
	movf	RX_Data+2,W
	movwf	ServoStopCenter+1
	goto	TX_ACK
Cmd_SetStopCenter_End:
;---kCmd_GetStopCenter---------
	movlw	kCmd_GetStopCenter
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetStopCenter_End
	movf	ServoStopCenter,W
	movwf	TX_Data
	movf	ServoStopCenter+1,W
	movwf	TX_Data+1
	goto	RS232_Send
Cmd_GetStopCenter_End:
;---kCmd_SetSpeed--------------
	movlw	kCmd_SetSpeed
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetSpeed_End
	movf	RX_Data+1,W
	andlw	0x3F	;0..63
	movwf	ServoSpeed
	goto	TX_ACK
Cmd_SetSpeed_End:
;---kCmd_GetSpeed--------------
	movlw	kCmd_GetSpeed
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetSpeed_End
	movf	ServoSpeed,W
	movwf	TX_Data
	goto	RS232_Send
Cmd_GetSpeed_End:
;---kCmd_SetUseIdleCenter------
	movlw	kCmd_SetUseIdleCenter
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetUseIdleCenter_End
	bcf	ssMode3IdleCenter
	movlw	0x01
	xorwf	RX_Data+1,W
	SKPNZ
	bsf	ssMode3IdleCenter
	goto	TX_ACK
Cmd_SetUseIdleCenter_End:
;---kCmd_GetUseIdleCenter------
	movlw	kCmd_GetUseIdleCenter
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetUseIdleCenter_End
	btfsc	ssMode3IdleCenter
	bsf	TX_Data,0
	goto	RS232_Send
Cmd_GetUseIdleCenter_End:
;---kCmd_SetDeadBand------------
	movlw	kCmd_SetDeadBand
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetDeadBand_End
	movf	RX_Data+1,W
	movwf	DeadBand
	goto	TX_ACK	
Cmd_SetDeadBand_End:
;---kCmd_GetDeadBand------------
	movlw	kCmd_GetDeadBand
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetDeadBand_End
	movf	DeadBand,W
	movwf	TX_Data
	goto	RS232_Send
Cmd_GetDeadBand_End:
;---kCmd_GetI-------------------
	movlw	kCmd_GetI
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetI_end
; Get servo current
	movf	Cur_AN0,W
	movwf	TX_Data
	movf	Cur_AN0+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetI_end:
;---kCmd_GetEnc-------------------
	movlw	kCmd_GetEnc
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetEnc_end
; Get Encoder Raw Position
	movf	EncoderVal,W
	movwf	TX_Data
	movf	EncoderVal+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetEnc_end:
;---kCmd_GetEncAbs-------------------
	movlw	kCmd_GetEncAbs
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetEncAbs_end
; Get Encoder Accumulated Position
	movf	EncoderAccum,W
	movwf	TX_Data
	movf	EncoderAccum+1,W
	movwf	TX_Data+1
	movf	EncoderAccum+2,W
	movwf	TX_Data+2
	goto	RS232_Send
;
Cmd_GetEncAbs_end:
;---kCmd_SaveParams-----------------
	movlw	kCmd_SaveParams
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SaveParams_end
	call	SaveParams
	goto	TX_ACK
Cmd_SaveParams_end:
;---kCmd_RestoreParams--------------
	movlw	kCmd_RestoreParams
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_RestoreParams_end
	call	CopyToRam
	goto	TX_ACK
Cmd_RestoreParams_end:
;---kCmd_GetBattVolts---------------
	movlw	kCmd_GetBattVolts
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetBattVolts_end
; Get battery volts
	movf	Cur_AN7,W
	movwf	TX_Data
	movf	Cur_AN7+1,W
	movwf	TX_Data+1
	goto	RS232_Send
Cmd_GetBattVolts_end:
;---kCmd_GetCalPot------------------
	movlw	kCmd_GetCalPot
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetCalPot_end
; Get calibration pot
	movf	Cur_AN4,W
	movwf	TX_Data
	movf	Cur_AN4+1,W
	movwf	TX_Data+1
	goto	RS232_Send
Cmd_GetCalPot_end:
;---kCmd_SetMasterAddr------------
	movlw	kCmd_SetMasterAddr
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetMasterAddr_end
	movf	RX_Data+1,W
	movwf	RS232_MasterAddr
	call	SaveParams
	goto	RS232_Send
Cmd_SetMasterAddr_end:
;---kCmd_SetSlaveAddr-------------
	movlw	kCmd_SetSlaveAddr
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_SetSlaveAddr_end
	movf	RX_Data+1,W
	movwf	RS232_SlaveAddr
	call	SaveParams
	goto	RS232_Send
Cmd_SetSlaveAddr_end:
;
	return
;
TX_ACK	movlw	0xFF
	goto	StoreSerOut
;
;=========================================================================================
