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
;
kCmd_SetGripI	EQU	0xB1	;+1 data (ssGripI), return ACK
kCmd_GetGripI	EQU	0x21
kCmd_SetAuxConfig	EQU	0xB2	;+2 data (AuxNum,Value), return ACK
kCmd_GetAuxConfig	EQU	0x22	;+1 data (AuxNum)
kCmd_SetAuxNValue	EQU	0xB3	;+2 data (AuxNum,Value), return ACK
kCmd_GetAuxNValue	EQU	0x23	;(AuxN,ValueL,ValueH)
kCmd_SetEncoderOffset	EQU	0xB4	;+2 data (EncoderOffset), return ACK
kCmd_GetEncoderOffset	EQU	0x24
                       if UsePID
kCmd_SetKp	EQU	0xB5	;+1 data (Kp), return ACK
kCmd_GetKp	EQU	0x25
kCmd_SetKi	EQU	0xB6	;+1 data (Ki), return ACK
kCmd_GetKi	EQU	0x26
kCmd_SetKd	EQU	0xB7	;+1 data (Kd), return ACK
kCmd_GetKd	EQU	0x27
                       endif
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
RS232_Send_S0	mLongCall	RS232_Send
	return
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRXData	movlb	RX_Flags	;bank 1
	bcf	RXDataIsNew
	btfsc	RXDataValidFlag	;from master to me?
	bra	HandleRXData_1
HandleRXData_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;
HandleRXData_1	movlb	0	;bank 0
                       mMOVLF	RX_Data,FSR1
	mMOVLF	TX_Data,FSR0
;Dispatcher
;
	moviw	FSR1++
	movwf	Param78	;aka RX_Data
	andlw                  0xF0
	SKPZ
	bra                    Dispatcher_1
	movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x00 to 0x0F
                       brw
                       return                                        ;0x00 not used
                       bra                    Cmd_GetMode
                       bra                    Cmd_GetCmdPos
                       goto                   Cmd_GetMaxI
                       goto                   Cmd_GetFFwd
                       goto                   Cmd_GetFRev
                       goto                   Cmd_GetMin_uS
                       goto                   Cmd_GetMax_uS
                       goto                   Cmd_GetRevDir
                       goto                   Cmd_GetEnaOvrCur       ;9
                       return
                       goto                   Cmd_GetStopCenter
                       goto                   Cmd_GetSpeed
                       goto                   Cmd_GetUseIdleCenter
                       goto                   Cmd_GetDeadBand
                       goto                   Cmd_GetUseFastPWM
;
Dispatcher_1           sublw                  0x80
                       SKPZ
                       bra                    Dispatcher_2
                       movf                   Param78,W
	andlw                  0x0F
;Jump table for 0x80 to 0x8F
                       brw
                       return                                        ;0x80 not used
                       bra                    Cmd_SetMode
                       bra                    Cmd_SetCmdPos
                       goto                   Cmd_SetMaxI
                       goto                   Cmd_SetFFwd
                       goto                   Cmd_SetFRev
                       goto                   Cmd_SetMin_uS
                       goto                   Cmd_SetMax_uS
                       goto                   Cmd_SetRevDir
                       goto                   Cmd_SetEnaOvrCur
                       return
                       goto                   Cmd_SetStopCenter
                       goto                   Cmd_SetSpeed
                       goto                   Cmd_SetUseIdleCenter
                       goto                   Cmd_SetDeadBand
                       goto                   Cmd_SetUseFastPWM
;
;Jump table for 0x20 to 0x27
Dispatcher_2           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x20
                       SKPZ
                       bra                    Dispatcher_3
                       btfsc                  Param78,3              ;>7?
                       return                                        ; Yes, oops
                       movlw                  0x07
                       andwf                  Param78,W
                       brw
                       return
                       goto                   Cmd_GetGripI
                       goto                   Cmd_GetAuxConfig
                       goto                   Cmd_GetAuxNValue
                       goto                   Cmd_GetEncoderOffset
                       if UsePID
                       goto                   Cmd_GetKp
                       goto                   Cmd_GetKi
                       goto                   Cmd_GetKd
                       else
                       return
                       return
                       return
                       endif
;
Dispatcher_3           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xB0
                       SKPZ
                       bra                    Dispatcher_4
                       btfsc                  Param78,3              ;>7?
                       return                                        ; Yes, oops
                       movlw                  0x07
                       andwf                  Param78,W
                       brw
                       return
                       goto                   Cmd_SetGripI
                       goto                   Cmd_SetAuxConfig
                       goto                   Cmd_SetAuxNValue
                       goto                   Cmd_SetEncoderOffset
                       if UsePID
                       goto                   Cmd_SetKp
                       goto                   Cmd_SetKi
                       goto                   Cmd_SetKd
                       else
                       return
                       return
                       return
                       endif
;
;Jump table for 0x90 to 0x9F
Dispatcher_4           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0x90
                       SKPZ
                       bra                    Dispatcher_5
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
                       return                                        ;0x90 not used
                       goto                   Cmd_GetI
                       goto                   Cmd_GetEnc
                       goto                   Cmd_GetEncAbs
                       goto                   Cmd_SaveParams
                       goto                   Cmd_RestoreParams
                       goto                   Cmd_GetBattVolts
                       return                                        ;goto                   Cmd_GetCalPot
                       goto                   Cmd_GetStatus
                       reset                                         ;Cmd_ReBoot
                       goto                   Cmd_RunBootloader
                       return
                       return
                       return
                       return
                       return
;
;Jump table for 0xA0 to 0xAF
Dispatcher_5           movf                   Param78,W
                       andlw                  0xF0
                       sublw                  0xA0
                       SKPZ
                       bra                    Dispatcher_6
                       movlw                  0x0F
                       andwf                  Param78,W
                       brw
Dispatcher_6           return                                        ;0xA0 not used
                       goto                   Cmd_SetMasterAddr
                       goto                   Cmd_SetSlaveAddr
                       return
                       return
                       return
                       return
                       return
                       return
                       return
                       goto                   Cmd_SendIDString       ;0xAA
                       return
                       return
                       return
                       return
                       return
;
;---kCmd_SetMode----------------
; Set Mode
Cmd_SetMode	movlw	kMaxMode+1
	subwf	INDF1,W
	SKPB		;kMaxMode+1>Data
	bra	HandleRXData_Rtn
;
	moviw	FSR1++
	movwf	SysMode
	goto	TX_ACK
;
;---kCmd_GetMode------------------
Cmd_GetMode	movf	SysMode,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetCmdPos-------------------
; Set Command Position
Cmd_SetCmdPos	moviw	FSR1++
	movwf	ssCmdPos
	moviw	FSR1++
	movwf	ssCmdPos+1
	goto	TX_ACK
;
;---kCmd_GetCmdPos------------------
; Get Command Position
Cmd_GetCmdPos	movf	ssCmdPos,W
	movwi	FSR0++
	movf	ssCmdPos+1,W
	movwi	FSR0++
	bcf                    ssCmdPosVerified
	goto	RS232_Send_S0
;
Cmd_GetCmdPos_end:
;---kCmd_SetMaxI-------------------
; Set Max Current
Cmd_SetMaxI	moviw	FSR1++
	movwf	ssMaxI
	goto	TX_ACK
;
;---kCmd_GetMaxI-------------------
; Get Max Current
Cmd_GetMaxI	movf	ssMaxI,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetFFwd-------------------
; Set ServoFastForward
Cmd_SetFFwd	moviw	FSR1++
	movwf	ServoFastForward
	moviw	FSR1++
	movwf	ServoFastForward+1
	goto	TX_ACK
;
;---kCmd_GetFFwd-------------------
; Get ServoFastForward
Cmd_GetFFwd	movf	ServoFastForward,W
	movwi	FSR0++
	movf	ServoFastForward+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetFRev-------------------
; Set ServoFastReverse
Cmd_SetFRev	moviw	FSR1++
	movwf	ServoFastReverse
	moviw	FSR1++
	movwf	ServoFastReverse+1
	goto	TX_ACK
;
;---kCmd_GetFRev------------------
; Get ServoFastReverse
Cmd_GetFRev	movf	ServoFastReverse,W
	movwi	FSR0++
	movf	ServoFastReverse+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetMin_uS-------------------
; Set ServoMin_uS
Cmd_SetMin_uS	moviw	FSR1++
	movwf	ServoMin_uS
	moviw	FSR1++
	movwf	ServoMin_uS+1
	goto	TX_ACK
;
;---kCmd_GetMin_uS----------------
; Get ServoMin_uS
Cmd_GetMin_uS	movf	ServoMin_uS,W
	movwi	FSR0++
	movf	ServoMin_uS+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetMax_uS-------------------
; Set ServoMax_uS
Cmd_SetMax_uS	moviw	FSR1++
	movwf	ServoMax_uS
	moviw	FSR1++
	movwf	ServoMax_uS+1
	goto	TX_ACK
;
;---kCmd_GetMax_uS-----------------
; Get ServoMax_uS
Cmd_GetMax_uS	movf	ServoMax_uS,W
	movwi	FSR0++
	movf	ServoMax_uS+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetRevDir---------------
Cmd_SetRevDir	bcf	ssReverseDir
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ		;01h?
	bsf	ssReverseDir	; Yes
	goto	TX_ACK
;
;---kCmd_GetRevDir---------------
Cmd_GetRevDir	btfsc	ssReverseDir
	bsf	INDF0,0
	goto	RS232_Send_S0
;
;---kCmd_SetEnaOvrCur------------
Cmd_SetEnaOvrCur	bcf	ssEnableOverCur
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ
	bsf	ssEnableOverCur
	goto	TX_ACK
;
;---kCmd_GetEnaOvrCur------------
Cmd_GetEnaOvrCur	btfsc	ssEnableOverCur
	bsf	INDF0,0
	goto	RS232_Send_S0
;
;---kCmd_SetStopCenter---------
Cmd_SetStopCenter	moviw	FSR1++
	movwf	ServoStopCenter
	moviw	FSR1++
	movwf	ServoStopCenter+1
	goto	TX_ACK
;
;---kCmd_GetStopCenter---------
Cmd_GetStopCenter	movf	ServoStopCenter,W
	movwi	FSR0++
	movf	ServoStopCenter+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetSpeed--------------
Cmd_SetSpeed	moviw	FSR1++
	andlw	0x3F	;0..63
	movwf	ServoSpeed
	goto	TX_ACK
;
;---kCmd_GetSpeed--------------
Cmd_GetSpeed	movf	ServoSpeed,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetUseIdleCenter------
Cmd_SetUseIdleCenter	bcf	ssMode3IdleCenter
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ
	bsf	ssMode3IdleCenter
	goto	TX_ACK
;
;---kCmd_GetUseIdleCenter------
Cmd_GetUseIdleCenter	btfsc	ssMode3IdleCenter
	bsf	INDF0,0
	goto	RS232_Send_S0
;
;---kCmd_SetDeadBand------------
Cmd_SetDeadBand	moviw	FSR1++
	movwf	DeadBand
	goto	TX_ACK	
;
;---kCmd_GetDeadBand------------
Cmd_GetDeadBand	movf	DeadBand,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetUseFastPWM----------	;+1 data (ssEnableFastPWM), return ACK
Cmd_SetUseFastPWM	bcf	ssEnableFastPWM
	movlw	0x01
	xorwf	INDF1,W
	SKPNZ
	bsf	ssEnableFastPWM
	goto	TX_ACK
;
;---kCmd_GetUseFastPWM----------
Cmd_GetUseFastPWM	btfsc	ssEnableFastPWM
	bsf	INDF0,0
	goto	RS232_Send_S0
;
;---kCmd_SetGripI-------------------
; Set Grip Current
Cmd_SetGripI	moviw	FSR1++
	movwf	ssGripI
	goto	TX_ACK
;
;---kCmd_GetGripI-------------------
; Get Grip Current
Cmd_GetGripI	movf	ssGripI,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetAuxConfig-----------	;+3 data (Aux0Config,Aux1Config,Aux2Config), return ACK
; Set Aux N Config
Cmd_SetAuxConfig	movf	INDF1,W
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
	moviw	FSR1++
	movwf	Param78	;AuxNum
	sublw	0x02	;2-AuxNum
	SKPNB		;<=2?
	goto	HandleRXData_Rtn	; no
;
                       movf                   Param78,W              ;AuxNum
	addlw	low ssAux0Config
	movwf	FSR0L
	movlw	high ssAux0Config
	movwf	FSR0H
	moviw	FSR1++	;New Config Value
	andlw                  0x07
	movwi	FSR0++
;
; Configure I/O Port
                       brw
                       bra                    Cmd_SetAuxNConfigNone
                       bra                    Cmd_SetAuxNConfigLEDBtn
                       bra                    Cmd_SetAuxNConfigDigIn
                       bra                    Cmd_SetAuxNConfigDigOut
                       bra                    Cmd_SetAuxNConfigAnalogIn
                       bra                    Cmd_SetAuxNConfigHomeSw
                       bra                    Cmd_SetAuxNConfigFwdLimit
                       bra                    Cmd_SetAuxNConfigRevLimit
;
Cmd_SetAuxNConfigNone:
Cmd_SetAuxNConfigLEDBtn:
Cmd_SetAuxNConfigDigIn:
Cmd_SetAuxNConfigHomeSw:
Cmd_SetAuxNConfigFwdLimit:
Cmd_SetAuxNConfigRevLimit:
                       movf                   Param78,W              ;AuxNum
                       SKPZ
                       bra                    Cmd_SetAux1ConfigDigIn
;
Cmd_SetAux0ConfigDigIn:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux0_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux0_ActiveBit      ;Don't Read AN
                       bsf                    Aux0_LED1_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux0_LED1_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAux1ConfigDigIn:
                       sublw                  0x01
                       SKPZ
                       bra                    Cmd_SetAux2ConfigDigIn
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux1_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux1_ActiveBit      ;Don't Read AN
                       bsf                    Aux1_LED2_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux1_LED2_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAux2ConfigDigIn:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux2_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux2_ActiveBit      ;Don't Read AN
                       bsf                    Aux2_LED3_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux2_LED3_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAuxNConfigDigOut:
                       movf                   Param78,W              ;AuxNum
                       SKPZ
                       bra                    Cmd_SetAux1ConfigDigOut
;
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux0_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux0_ActiveBit      ;Don't Read AN
                       bcf                    Aux0_LED1_TRIS         ;Output
                       movlb                  2                      ;bank 2
                       bcf                    Aux0_LED1_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAux1ConfigDigOut:
                       sublw                  0x01
                       SKPZ
                       bra                    Cmd_SetAux2ConfigDigOut
;
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux1_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux1_ActiveBit      ;Don't Read AN
                       bcf                    Aux1_LED2_TRIS         ;Output
                       movlb                  2                      ;bank 2
                       bcf                    Aux1_LED2_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;        
Cmd_SetAux2ConfigDigOut:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux2_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux2_ActiveBit      ;Don't Read AN
                       bcf                    Aux2_LED3_TRIS         ;Output
                       movlb                  2                      ;bank 2
                       bcf                    Aux2_LED3_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAuxNConfigAnalogIn:
                       movf                   Param78,W              ;AuxNum
                       SKPZ
                       bra                    Cmd_SetAux1ConfigAnalogIn
;
                       movlb                  ANSELA                 ;bank 3
                       bsf                    Aux0_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bsf                    AN_Aux0_ActiveBit      ;Don't Read AN
                       bsf                    Aux0_LED1_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux0_LED1_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAux1ConfigAnalogIn:
                       sublw                  0x01
                       SKPZ
                       bra                    Cmd_SetAux2ConfigAnalogIn
;
                       movlb                  ANSELA                 ;bank 3
                       bsf                    Aux1_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bsf                    AN_Aux1_ActiveBit      ;Don't Read AN
                       bsf                    Aux1_LED2_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux1_LED2_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
Cmd_SetAux2ConfigAnalogIn:
                       movlb                  ANSELA                 ;bank 3
                       bsf                    Aux2_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bsf                    AN_Aux2_ActiveBit      ;Don't Read AN
                       bsf                    Aux2_LED3_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux2_LED3_Lat          ;0
                       movlb                  0                      ;bank 0
                       goto	TX_ACK
;
;
;---kCmd_GetAuxConfig-----------
; Get Aux N Config
Cmd_GetAuxConfig	movf	INDF1,W
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
;---kCmd_SetAuxNValue-----------	;+2 data (AuxNum,Value), return ACK
; Set Aux N Value
Cmd_SetAuxNValue	moviw	FSR1++
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
                       movf	Param78,W	;AuxNum
                       SKPZ
	bra	Cmd_SetAuxNValue_Dig1
;
	moviw	FSR1++	;AuxVal
                       bcf                    Aux0_SW1_Active        ;default to 0
	btfsc                  WREG,0
	bsf	Aux0_SW1_Active
	goto	TX_ACK
;
Cmd_SetAuxNValue_Dig1:
                       sublw	0x01                   ;AuxNum=1?
	SKPZ
	bra	Cmd_SetAuxNValue_Dig2  ; No
;
	moviw	FSR1++	;AuxVal
                       bcf                    Aux1_SW2_Active        ;default to 0
	btfsc                  WREG,0
	bsf	Aux1_SW2_Active
	goto	TX_ACK
;
Cmd_SetAuxNValue_Dig2:
                       movf	Param78,W	;AuxNum
                       sublw	0x02                   ;AuxNum=2?
	SKPZ
	goto	TX_ACK                 ; No
;                       
	moviw	FSR1++	;AuxVal
                       bcf                    Aux2_SW3_Active        ;default to 0
	btfsc                  WREG,0
	bsf	Aux2_SW3_Active
	goto	TX_ACK
;
Cmd_SetAuxNLED:
	if UseAuxLEDBlinking
	movf	Param78,W	;AuxNum
	addlw	low LED1_Blinks
	movwf	FSR0L
	movlw	high LED1_Blinks
	movwf	FSR0H
	moviw	FSR1++	;AuxVal
	movwi	FSR0++
	endif
	goto	TX_ACK
;
;---kCmd_GetAuxNValue-----------	;(AuxN,ValueL,ValueH)
; Get Aux N Value
Cmd_GetAuxNValue	movf	INDF1,W
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
	addlw	low Cur_AN2            ;Aux0 = AN2
	movwf	FSR1L
	movlw	high Cur_AN2
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
	btfsc	Aux0_SW1_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Dig1	sublw	0x01
	SKPZ
	bra	Cmd_GetAuxNValue_Dig2
	btfsc	Aux1_SW2_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Dig2	clrw
	btfsc	Aux2_SW3_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
; Send the Button value
Cmd_GetAuxNValue_Btn	movf	Param78,W	;AuxNum
	SKPZ
	bra	Cmd_GetAuxNValue_Btn1
	btfsc	Aux0_SW1_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Btn1	sublw	0x01
	SKPZ
	bra	Cmd_GetAuxNValue_Btn2
	btfsc	Aux1_SW2_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetAuxNValue_Btn2	clrw
	btfsc	Aux2_SW3_Active
	movlw	0x01
	movwi	FSR0++
	goto	RS232_Send_S0
;
;
;---kCmd_SetEncoderOffset-------  +2 data (EncoderOffset), return ACK
;
Cmd_SetEncoderOffset	moviw	FSR1++
	movwf	EncoderOffset
	moviw	FSR1++
	movwf	EncoderOffset+1
	goto	TX_ACK
;
;---kCmd_GetEncoderOffset-------
;
Cmd_GetEncoderOffset	movf	EncoderOffset,W
	movwi	FSR0++
	movf	EncoderOffset+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
                       if UsePID
;---kCmd_SetKp-------------------	;+1 data (Kp), return ACK
Cmd_SetKp	moviw	FSR1++
	movwf	Kp
	goto	TX_ACK
;
;---kCmd_GetKp-------------------
Cmd_GetKp	movf	Kp,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetKi-------------------	;+1 data (Ki), return ACK
Cmd_SetKi	moviw	FSR1++
	movwf	Ki
	goto	TX_ACK
;
;---kCmd_GetKi-------------------
Cmd_GetKi	movf	Ki,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_SetKd-------------------	;+1 data (Kd), return ACK
Cmd_SetKd	moviw	FSR1++
	movwf	Kd
	goto	TX_ACK
;---kCmd_GetKd-------------------
Cmd_GetKd	movf	Kd,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
                       endif
;---kCmd_GetI-------------------
; Get servo current
Cmd_GetI	BankSel	Cur_AN0
	movf	Cur_AN0,W
	movwi	FSR0++
	movf	Cur_AN0+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_GetEnc-------------------
; Get Encoder Raw Position
Cmd_GetEnc	movf	EncoderVal,W
	movwi	FSR0++
	movf	EncoderVal+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_GetEncAbs-------------------
; Get Encoder Accumulated Position
Cmd_GetEncAbs	movf	EncoderAccum,W
	movwi	FSR0++
	movf	EncoderAccum+1,W
	movwi	FSR0++
	movf	EncoderAccum+2,W
	movwi	FSR0++
	movlw	0x00
	btfsc	EncoderAccum+2,7
	movlw	0xFF
	movwi	FSR0++	;sign extrnded Int24
	goto	RS232_Send_S0
;
;---kCmd_SaveParams-----------------
Cmd_SaveParams	mLongCall	SaveParams
	goto	TX_ACK
;
;---kCmd_RestoreParams--------------
Cmd_RestoreParams	mLongCall	CopyToRam
	goto	TX_ACK
;
;---kCmd_GetBattVolts---------------
; Get battery volts
Cmd_GetBattVolts	BankSel	BattVolts
	movf	BattVolts,W
	movwi	FSR0++
	movf	BattVolts+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_GetCalPot------------------
; Get calibration pot
Cmd_GetCalPot	movlb	ModeZeroPot
	movf	ModeZeroPot,W
	movwi	FSR0++
	movf	ModeZeroPot+1,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
;---kCmd_GetStatus----------------
Cmd_GetStatus	movf	ssStatus,W
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
;
;---kCmd_RunBootloader--------------
Cmd_RunBootloader	movlw	0xFF
	movwf	EEDataTemp
	mLongCall	EEWriteW	;Write EEPROM address in W
	reset
;
;---kCmd_SetMasterAddr------------
Cmd_SetMasterAddr	moviw	FSR1++
	movwf	RS232_MasterAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
;
;---kCmd_SetSlaveAddr-------------
Cmd_SetSlaveAddr	moviw	FSR1++
	movwf	RS232_SlaveAddr
	mLongCall	SaveParams
	goto	RS232_Send_S0
;
;---kCmd_SendIDString------------
Cmd_SendIDString	clrf	Param79
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
	retlw	' '
	retlw	'r'
	retlw	'C'
	retlw	'\n'
	retlw	0x00
;
;--------------------------------
	return
;
TX_ACK	movlw	0xFF
StoreSerOut_S0	mLongCall	StoreSerOut
	return
;
;=========================================================================================
