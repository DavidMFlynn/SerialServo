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
;
kCmd_SetStopCenter	EQU	0x8B	;+2 data (ServoStopCenter), return ACK
kCmd_GetStopCenter	EQU	0x0B
kCmd_SetSpeed	EQU	0x8C	;+1 data (ServoSpeed), return ACK
kCmd_GetSpeed	EQU	0x0C
kCmd_SetUseIdleCenter	EQU	0x8D	;+1 data (ssMode3IdleCenter), return ACK
kCmd_GetUseIdleCenter	EQU	0x0D
kCmd_SetDeadBand	EQU	0x8E	;+1 data (DeadBand), return ACK
kCmd_GetDeadBand	EQU	0x0E
kCmd_SetUseFastPWM	EQU	0x8F	;+1 data (ssEnableFastPWM), return ACK
kCmd_GetUseFastPWM	EQU	0x0F
kCmd_SetGripI	EQU	0xB1	;+1 data (ssGripI), return ACK
kCmd_GetGripI	EQU	0x21
kCmd_SetAuxConfig	EQU	0xB2	;+2 data (AuxNum,Value), return ACK
kCmd_GetAuxConfig	EQU	0x22	;+1 data (AuxNum)
kCmd_SetAuxNValue	EQU	0xB3	;+2 data (AuxNum,Value), return ACK
kCmd_GetAuxNValue	EQU	0x23	;(AuxN,ValueL,ValueH)
kCmd_SetEncoderOffset	EQU	0xB4	;+2 data (EncoderOffset), return ACK
kCmd_GetEncoderOffset	EQU	0x24
;
kCmd_GetI	EQU	0x91	;return Cur_AN0
kCmd_GetEnc	EQU	0x92	;return EncoderVal
kCmd_GetEncAbs	EQU	0x93	;return EncoderAccum
kCmd_SaveParams	EQU	0x94	;Save all eeprom params, return ACK
kCmd_RestoreParams	EQU	0x95	;Copy to ram, return ACK
kCmd_GetBattVolts	EQU	0x96	;return Cur_AN7
kCmd_GetCalPot	EQU	0x97	;return Cur_AN4
kCmd_GetStatus	EQU	0x98	;return ssStatus
kCmd_ReBoot	EQU	0x99	;ReBoot the controller
kCmd_RunBootloader	EQU	0x9A	;Set EEPROM (0xFF) = 0xFF and reboot
;
; these commands save params and return a 0,0,0,0 packet with the new address
kCmd_SetMasterAddr	EQU	0xA1	;+1 data, return a zero packet
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;Special packet (0xDD, 0xDD, 0x01, 0x02, 0xAA, 0x00, 0x00, 0x00, checksum)
kCmd_SendIDString	EQU	0xAA	; returns "DMFE Serial Servo\n"
;
;=========================================================================================
;
RS232_Send_S0	mCall1To0	RS232_Send
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
	movlw	low RX_Data
	movwf	FSR1L
	movlw	high RX_Data
	movwf	FSR1H
;
	movlw	low TX_Data
	movwf	FSR0L
	movlw	high TX_Data
	movwf	FSR0H
;
	moviw	FSR1++
	movwf	Param78	;aka RX_Data
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
	movwf	LED1_Blinks
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
;---kCmd_SetCmdPos-------------------
	movlw	kCmd_SetCmdPos
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetCmdPos_end
; Set Command Position
	moviw	FSR1++
	movwf	ssCmdPos
	moviw	FSR1++
	movwf	ssCmdPos+1
	goto	TX_ACK
;
Cmd_SetCmdPos_end:
;---kCmd_GetCmdPos------------------
	movlw	kCmd_GetCmdPos
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetCmdPos_end
; Get Command Position
	movf	ssCmdPos,W
	movwi	FSR0++
	movf	ssCmdPos+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetCmdPos_end:
;---kCmd_SetMaxI-------------------
	movlw	kCmd_SetMaxI
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMaxI_end
; Set Max Current
	moviw	FSR1++
	movwf	ssMaxI
	goto	TX_ACK
;
Cmd_SetMaxI_end:
;---kCmd_GetMaxI-------------------
	movlw	kCmd_GetMaxI
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetMaxI_end
; Get Max Current
	movf	ssMaxI,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetMaxI_end:
;---kCmd_SetFFwd-------------------
	movlw	kCmd_SetFFwd
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetFFwd_end
; Set ServoFastForward
	moviw	FSR1++
	movwf	ServoFastForward
	moviw	FSR1++
	movwf	ServoFastForward+1
	goto	TX_ACK
;
Cmd_SetFFwd_end:
;---kCmd_GetFFwd-------------------
	movlw	kCmd_GetFFwd
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetFFwd_end
; Get ServoFastForward
	movf	ServoFastForward,W
	movwi	FSR0++
	movf	ServoFastForward+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetFFwd_end:
;---kCmd_SetFRev-------------------
	movlw	kCmd_SetFRev
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetFRev_end
; Set ServoFastReverse
	moviw	FSR1++
	movwf	ServoFastReverse
	moviw	FSR1++
	movwf	ServoFastReverse+1
	goto	TX_ACK
;
Cmd_SetFRev_end:
;---kCmd_GetFRev------------------
	movlw	kCmd_GetFRev
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetFRev_end
; Get ServoFastReverse
	movf	ServoFastReverse,W
	movwi	FSR0++
	movf	ServoFastReverse+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetFRev_end:
;---kCmd_SetMin_uS-------------------
	movlw	kCmd_SetMin_uS
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMin_uS_end
; Set ServoMin_uS
	moviw	FSR1++
	movwf	ServoMin_uS
	moviw	FSR1++
	movwf	ServoMin_uS+1
	goto	TX_ACK
;
Cmd_SetMin_uS_end:
;---kCmd_GetMin_uS----------------
	movlw	kCmd_GetMin_uS
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetMin_uS_end
; Get ServoMin_uS
	movf	ServoMin_uS,W
	movwi	FSR0++
	movf	ServoMin_uS+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetMin_uS_end:
;---kCmd_SetMax_uS-------------------
	movlw	kCmd_SetMax_uS
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMax_uS_end
; Set ServoMax_uS
	moviw	FSR1++
	movwf	ServoMax_uS
	moviw	FSR1++
	movwf	ServoMax_uS+1
	goto	TX_ACK
;
Cmd_SetMax_uS_end:
;---kCmd_GetMax_uS-----------------
	movlw	kCmd_GetMax_uS
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetMax_uS_end
; Get ServoMax_uS
	movf	ServoMax_uS,W
	movwi	FSR0++
	movf	ServoMax_uS+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetMax_uS_end:
;---kCmd_SetRevDir---------------
	movlw	kCmd_SetRevDir
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetRevDir_End
	bcf	ssReverseDir
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ		;01h?
	bsf	ssReverseDir	; Yes
	goto	TX_ACK
Cmd_SetRevDir_End:
;---kCmd_GetRevDir---------------
	movlw	kCmd_GetRevDir
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetRevDir_End
	btfsc	ssReverseDir
	bsf	INDF0,0
	goto	RS232_Send_S0
Cmd_GetRevDir_End:
;---kCmd_SetEnaOvrCur------------
	movlw	kCmd_SetEnaOvrCur
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetEnaOvrCur_End
	bcf	ssEnableOverCur
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ
	bsf	ssEnableOverCur
	goto	TX_ACK
Cmd_SetEnaOvrCur_End:
;---kCmd_GetEnaOvrCur------------
	movlw	kCmd_GetEnaOvrCur
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetEnaOvrCur_End
	btfsc	ssEnableOverCur
	bsf	INDF0,0
	goto	RS232_Send_S0
Cmd_GetEnaOvrCur_End:
;
;---kCmd_SetStopCenter---------
	movlw	kCmd_SetStopCenter
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetStopCenter_End
	moviw	FSR1++
	movwf	ServoStopCenter
	moviw	FSR1++
	movwf	ServoStopCenter+1
	goto	TX_ACK
Cmd_SetStopCenter_End:
;---kCmd_GetStopCenter---------
	movlw	kCmd_GetStopCenter
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetStopCenter_End
	movf	ServoStopCenter,W
	movwi	FSR0++
	movf	ServoStopCenter+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
Cmd_GetStopCenter_End:
;---kCmd_SetSpeed--------------
	movlw	kCmd_SetSpeed
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetSpeed_End
	moviw	FSR1++
	andlw	0x3F	;0..63
	movwf	ServoSpeed
	goto	TX_ACK
Cmd_SetSpeed_End:
;---kCmd_GetSpeed--------------
	movlw	kCmd_GetSpeed
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetSpeed_End
	movf	ServoSpeed,W
	movwi	FSR0++
	goto	RS232_Send_S0
Cmd_GetSpeed_End:
;---kCmd_SetUseIdleCenter------
	movlw	kCmd_SetUseIdleCenter
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetUseIdleCenter_End
	bcf	ssMode3IdleCenter
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ
	bsf	ssMode3IdleCenter
	goto	TX_ACK
Cmd_SetUseIdleCenter_End:
;---kCmd_GetUseIdleCenter------
	movlw	kCmd_GetUseIdleCenter
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetUseIdleCenter_End
	btfsc	ssMode3IdleCenter
	bsf	INDF0,0
	goto	RS232_Send_S0
Cmd_GetUseIdleCenter_End:
;---kCmd_SetDeadBand------------
	movlw	kCmd_SetDeadBand
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetDeadBand_End
	moviw	FSR1++
	movwf	DeadBand
	goto	TX_ACK	
Cmd_SetDeadBand_End:
;---kCmd_GetDeadBand------------
	movlw	kCmd_GetDeadBand
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetDeadBand_End
	movf	DeadBand,W
	movwi	FSR0++
	goto	RS232_Send_S0
Cmd_GetDeadBand_End:
;---kCmd_SetUseFastPWM----------	;+1 data (ssEnableFastPWM), return ACK
	movlw	kCmd_SetUseFastPWM
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetUseFastPWM_End
	bcf	ssEnableFastPWM
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ
	bsf	ssEnableFastPWM
	goto	TX_ACK
Cmd_SetUseFastPWM_End:
;---kCmd_GetUseFastPWM----------
	movlw	kCmd_GetUseFastPWM
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetUseFastPWM_End
	btfsc	ssEnableFastPWM
	bsf	INDF0,0
	goto	RS232_Send_S0
Cmd_GetUseFastPWM_End:
;---kCmd_SetGripI-------------------
	movlw	kCmd_SetGripI
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetGripI_end
; Set Grip Current
	moviw	FSR1++
	movwf	ssGripI
	goto	TX_ACK
;
Cmd_SetGripI_end:
;---kCmd_GetGripI-------------------
	movlw	kCmd_GetGripI
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetGripI_end
; Get Grip Current
	movf	ssGripI,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetGripI_end:
;---kCmd_SetAuxConfig-----------	;+3 data (Aux0Config,Aux1Config,Aux2Config), return ACK
	movlw	kCmd_SetAuxConfig
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetAuxConfig_end
; Set Aux N Config
	movf	INDF1,W
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	moviw	FSR1++
	addlw	low ssAux0Config
	movwf	FSR0L
	movlw	high ssAux0Config
	movwf	FSR0H
	moviw	FSR1++	;New Config Value
	movwi	FSR0++
	goto	TX_ACK
;
Cmd_SetAuxConfig_end:
;---kCmd_GetAuxConfig-----------
	movlw	kCmd_GetAuxConfig
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetAuxConfig_end
; Get Aux N Config
	movf	INDF1,W
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	movf	INDF1,W
	addlw	low ssAux0Config
	movwf	FSR1L
	movlw	high ssAux0Config
	movwf	FSR1H
;
	movf	INDF1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxConfig_end:	
;---kCmd_SetAuxNValue-----------	;+2 data (AuxNum,Value), return ACK
	movlw	kCmd_SetAuxNValue
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetAuxNValue_end
; Set Aux N Value
	moviw	FSR1++
	movwf	Param78	;AuxNum
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	movf	Param78,W	;AuxNum
	addlw	low ssAux0Config
	movwf	FSR0L
	movlw	high ssAux0Config
	movwf	FSR0H
;
	movf	INDF0,W	;ssAuxXConfig
	andlw	0x07
	brw
	goto	TX_ACK	;kAuxIOnone
	bra	Cmd_SetAuxNLED	;kAuxIOLEDBtn
	goto	TX_ACK	;kAuxIODigitalIn
	bra	Cmd_SetAuxNDigOut	;kAuxIODigitalOut
	goto	TX_ACK	;kAuxIOAnalogIn
	goto	TX_ACK	;kAuxIOHomeSw
	goto	TX_ACK	;kAuxIOFwdLimit
	goto	TX_ACK	;kAuxIORevLimit
;
Cmd_SetAuxNDigOut:
Cmd_SetAuxNLED	movf	Param78,W	;AuxNum
	addlw	low LED1_Blinks
	movwf	FSR0L
	movlw	high LED1_Blinks
	movwf	FSR0H
	moviw	FSR1++	;AuxVal
	movwi	FSR0++
	goto	TX_ACK
;
Cmd_SetAuxNValue_end:
;---kCmd_GetAuxNValue-----------	;(AuxN,ValueL,ValueH)
	movlw	kCmd_GetAuxNValue
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetAuxNValue_end
; Get Aux N Value
	movf	INDF1,W
	movwf	Param78	;AuxNum
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	movf	Param78,W	;AuxNum
	addlw	low ssAux0Config
	movwf	FSR1L
	movlw	high ssAux0Config
	movwf	FSR1H
;
	movf	INDF1,W	;ssAuxXConfig
	andlw	0x07
	brw
	goto	HandleRXData_Rtn	;kAuxIOnone, not valid
	bra	Cmd_GetAuxNValue_Btn	;kAuxIOLEDBtn
	bra	Cmd_GetAuxNValue_Dig	;kAuxIODigitalIn
	bra	Cmd_GetAuxNValue_Dig	;kAuxIODigitalOut
	bra	Cmd_GetAuxNValue_Ana	;kAuxIOAnalogIn
	bra	Cmd_GetAuxNValue_Dig	;kAuxIOHomeSw
	bra	Cmd_GetAuxNValue_Dig	;kAuxIOFwdLimit
	bra	Cmd_GetAuxNValue_Dig	;kAuxIORevLimit
;
Cmd_GetAuxNValue_Ana	lslf	Param78,W	;AuxNum X2
	addlw	low Cur_AN1
	movwf	FSR1L
	movlw	high Cur_AN1
	movwf	FSR1H
	moviw	FSR1++
	movwi	FSR0++
	moviw	FSR1++
	movwi	FSR0++
	goto	RS232_Send_S0
;
; Send the value of the port bit 0x00 or 0x01
Cmd_GetAuxNValue_Dig	movf	Param78,W	;AuxNum
	SKPZ
	bra	Cmd_GetAuxNValue_Dig1
	btfsc	SW1_In
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Dig1	clrw
	btfsc	SW2_In
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
; Send the Button value
Cmd_GetAuxNValue_Btn	movf	Param78,W	;AuxNum
	SKPZ
	bra	Cmd_GetAuxNValue_Btn1
	btfsc	SW1_Flag
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Btn1	sublw	0x01
	SKPZ
	bra	Cmd_GetAuxNValue_Btn2
	btfsc	SW2_Flag
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Btn2	clrw
;	btfsc	SW3_Flag
;	movlw	0x01
;	movwi	FSR0++
	goto	RS232_Send_S0
;
;
Cmd_GetAuxNValue_end:
;---kCmd_SetEncoderOffset-------  +2 data (EncoderOffset), return ACK
	movlw	kCmd_SetEncoderOffset
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetEncoderOffset_End
;
	moviw	FSR1++
	movwf	EncoderOffset
	moviw	FSR1++
	movwf	EncoderOffset+1
	goto	TX_ACK
Cmd_SetEncoderOffset_End:
;---kCmd_GetEncoderOffset-------
	movlw	kCmd_GetEncoderOffset
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetEncoderOffset_End
;
	movf	EncoderOffset,W
	movwi	FSR0++
	movf	EncoderOffset+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
Cmd_GetEncoderOffset_End:
;---kCmd_GetI-------------------
	movlw	kCmd_GetI
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetI_end
; Get servo current
	BankSel	Cur_AN0
	movf	Cur_AN0,W
	movwi	FSR0++
	movf	Cur_AN0+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetI_end:
;---kCmd_GetEnc-------------------
	movlw	kCmd_GetEnc
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetEnc_end
; Get Encoder Raw Position
	movf	EncoderVal,W
	movwi	FSR0++
	movf	EncoderVal+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetEnc_end:
;---kCmd_GetEncAbs-------------------
	movlw	kCmd_GetEncAbs
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetEncAbs_end
; Get Encoder Accumulated Position
	movf	EncoderAccum,W
	movwi	FSR0++
	movf	EncoderAccum+1,W
	movwi	FSR0++
	movf	EncoderAccum+2,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetEncAbs_end:
;---kCmd_SaveParams-----------------
	movlw	kCmd_SaveParams
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SaveParams_end
	mCall1To0	SaveParams
	goto	TX_ACK
Cmd_SaveParams_end:
;---kCmd_RestoreParams--------------
	movlw	kCmd_RestoreParams
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_RestoreParams_end
	mCall1To0	CopyToRam
	goto	TX_ACK
Cmd_RestoreParams_end:
;---kCmd_GetBattVolts---------------
	movlw	kCmd_GetBattVolts
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetBattVolts_end
; Get battery volts
	BankSel	BattVolts
	movf	BattVolts,W
	movwi	FSR0++
	movf	BattVolts+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
Cmd_GetBattVolts_end:
;---kCmd_GetCalPot------------------
;	movlw	kCmd_GetCalPot
;	subwf	Param78,W	;aka RX_Data
;	SKPZ
;	bra	Cmd_GetCalPot_end
; Get calibration pot
;	BankSel	Cur_AN4
;	movf	Cur_AN4,W
;	movwi	FSR0++
;	movf	Cur_AN4+1,W
;	movwi	FSR0++
;	goto	RS232_Send_S0
Cmd_GetCalPot_end:
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
	movf	ssStatus+2,W
	movwi	FSR0++
;reset flags
	bcf	ssRX_Timeout
	bcf	ssGripMCur
	bcf	ssGripOCur
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
	mCall1To0	EEWriteW	;Write EEPROM address in W
	reset
Cmd_RunBootloader_end:
;---kCmd_SetMasterAddr------------
	movlw	kCmd_SetMasterAddr
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMasterAddr_end
	moviw	FSR1++
	movwf	RS232_MasterAddr
	mCall1To0	SaveParams
	goto	RS232_Send_S0
Cmd_SetMasterAddr_end:
;---kCmd_SetSlaveAddr-------------
	movlw	kCmd_SetSlaveAddr
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetSlaveAddr_end
	moviw	FSR1++
	movwf	RS232_SlaveAddr
	mCall1To0	SaveParams
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
	retlw	'e'
	retlw	'r'
	retlw	'i'
	retlw	'a'
	retlw	'l'
	retlw	' '
	retlw	'S'
	retlw	'e'
	retlw	'r'
	retlw	'v'
	retlw	'o'
	retlw	'\n'
	retlw	0x00
;
Cmd_SendIDString_end:
;--------------------------------
	return
;
TX_ACK	movlw	0xFF
StoreSerOut_S0	mCall1To0	StoreSerOut
	return
;
;=========================================================================================
