;====================================================================================================
;
;    Filename:      SerialServo.asm
;    Created:       4/26/2018
;    File Version:  1.1b4   8/26/2020
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    SerialServo is sample code.
;    Controls a single robot arm joint using an R/C servo (SG90) modified
;    for continuous rotation.  Features and configurations will be added as needed.
;
;    Features: 	TTL Packet Serial
;	R/C Servo PWM output
;	Current sensing.
;	3 Buttons/LEDs for config
;	Absolute magnetic encoder
;
;Mode 0: (LED 1 = off) servo test mode, copy AN4 Pot value x 2 + 1976 to servo PWM.
;Mode 1: (LED 1 = 1 flash) servo and encoder test mode, AN4 Pot value + 950 - EncoderVal to servo dir.
;Mode 2: Basic Serial Servo, output servo pulse of ssCmdPos x 0.5uS.
;Mode 3: Absolute encoder position control. ssCmdPos = 0..4095
;Mode 4: Gripper force control.
;
;    History:
; 1.1b4   8/26/2020    Addded kCmd_SetKp..., Fixed Batt Volts (AN1), ClampInt bug fixed.
; 1.1b3   4/10/2020    Improved Mode 3
; 1.1b2   8/11/2019	Continue fixes for 14bit encoder. New defaults Mode 3 (2950 ±100, fast, Idle center)
; 1.1b1   3/21/2019	Port for Rev C PCB
; 1.0b7   10/3/2018	Mode 3 is working for 4-wheel rover corner pivot motors.
; 1.0b6   8/18/2018	Moved analog variables to bank 1. Fast blink on error. EncoderOffset for mode3
; 1.0b5   7/23/2018	Aux IO
; 1.0b4   7/14/2018	Better defaults. Gripper mode (4).
; 1.0b3   6/19/2018	Added ssEnableFastPWM
; 1.0b2   6/3/2018	Servo current is averaged, DD DD Sync bytes and checksum.
; 1.0b1   6/1/2018	Modes 2 and 3 are working. No current limit yet.
; 1.0a3   5/31/2018    Added Speed, StopCenter.
; 1.0a2   5/25/2018	Added some more commands.
; 1.0a1   5/24/2018	It begins to work.
; 1.0d1   4/26/2018	First code.
;
;====================================================================================================
; ToDo:
;
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;   Mode 0: (LED 1 = off) servo test mode, copy AN4 Pot value to servo.
;   Mode 1: (LED 1 = 1 flash) servo  and encoder test mode, AN4 Pot value - EncoderVal to servo dir.
;   Mode 2: Basic Serial Servo, output servo pulse of CmdPos * 0.5uS.
;   Mode 3: Absolute encoder position control, Single rotation of encoder, Continuous rotation servo.
;   Mode 4: Gripper force control.
;====================================================================================================
;
;   Pin 1 (RA2/AN2) SW1/LED1 (Active Low Input/Output)
;   Pin 2 (RA3/AN3) SW2/LED2 (Active Low Input/Output)
;   Pin 3 (RA4/AN4) n/c
;   Pin 4 (RA5/MCLR*) VPP/MCLR*
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) MagEnc_CSBit (Active Low Output)
;   Pin 7 (RB1/AN11/SDA1) MISO MagEnc_DataBit (Digital Input)
;   Pin 8 (RB2/AN10/TX) TTL Serial RX
;   Pin 9 (RB3/CCP1) Pulse output for Servo
;
;   Pin 10 (RB4/AN8/SLC1) SCL1 MagEnc_CLKBit
;   Pin 11 (RB5/AN7) TTL Serial TX
;   Pin 12 (RB6/AN5/CCP2) ICSPCLK
;   Pin 13 (RB7/AN6) ICSPDAT
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) MOSI MagEnc_DataBit (Digital Output)
;   Pin 16 (RA7/CCP2) LED3 (Active Low Output)(System LED)
;   Pin 17 (RA0/AN0) Current sensing analog input
;   Pin 18 (RA1/AN1) Battery voltage sensing analog input
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,W=1	; list directive to define processor
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_ON & _LVP_OFF
;
; Write protection off
; 4x PLL Enabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
	constant	useRS232=1
	constant	UseEEParams=1
;
	constant	UseAltSerialPort=1
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
	constant               UsePID=0
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x02	;This Slave's Address
kSysMode	EQU	.3	;Default Mode
Default_Kp	EQU	.32	;Fxd4.4 10*16
Default_Ki	EQU	0	; max gain is 255 = 15 15/16
Default_Kd	EQU	0
kGripperHC	EQU	0x04	;Gripper hysteresis
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'10111111'
PortAValue	EQU	b'00000000'
ANSELA_Val	EQU	b'00000011'	;RA0/AN0, RA4/AN4
;
#Define	RA0_In	PORTA,0	;Current, Analog Input
#Define	RA1_In	PORTA,1	;Battery Volts, Analog Input
#Define	SW1_In	PORTA,2	;SW1/LED1
#Define	SW2_In	PORTA,3	;SW2/LED2
#Define	SW3_In	PORTA,4	;n/c on Rev C
#Define	RA5_In	PORTA,5	;VPP/MCLR*
#Define	RA6_Out	PORTA,6	;MagEnc_DataBit Encoder MOSI (SPI, Digital Output)
#Define	RA7_In	PORTA,7	;LED3 (Active Low Output)(System LED)
LED1_Bit	EQU	2	;LED1 (Active Low Output)
LED2_Bit	EQU	3	;LED2 (Active Low Output)
SysLED_Bit	EQU	7	;LED3 (Active Low Output)
#Define	LED1_Tris	TRISA,LED1_Bit	;LED1 (Active Low Output)
#Define	LED1_Lat	LATA,LED1_Bit	;LED1 (Active Low Output)
#Define	LED2_Tris	TRISA,LED2_Bit	;LED2 (Active Low Output)
#Define	LED2_Lat	LATA,LED2_Bit	;LED2 (Active Low Output)
#Define	SysLED_Tris	TRISA,SysLED_Bit	;LED3 (Active Low Output)
;
Servo_AddrDataMask	EQU	0xF8
;
;
;    Port B bits
PortBDDRBits	EQU	b'11000110'	;MagEnc_CSBit, CCP1, MagEnc_CLKBit
PortBValue	EQU	b'00010001'
ANSELB_Val	EQU	b'00000000'	;RB5/AN7
;
#Define	RB0_Out	LATB,0	;MagEnc_CSBit (Active Low Output)
#Define	RB1_In	PORTB,1	;MISO MagEnc_DataBit (Digital Input)
#Define	RB2_In	PORTB,2	;RX Serial Data
#Define	RB3_Out	PORTB,3	;CCP1 Output
#Define	RB4_In	PORTB,4	;SCL1 MagEnc_CLKBit
#Define	RB5_In	PORTB,5	;TX Serial Data
#Define	RB6_In	PORTB,6	;ICSPCLK
#Define	RB7_In	PORTB,7	;ICSPDAT
;
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
CCP1CON_Clr	EQU	b'00001001'
CCP1CON_Set	EQU	b'00001000'
CCP1CON_Idle	EQU	b'00001010'
;
;OSCCON_Value	EQU	b'01110010'	; 8 MHz
OSCCON_Value	EQU	b'11110000'	;32MHz
;
;T2CON_Value	EQU	b'01001110'	;T2 On, /16 pre, /10 post
T2CON_Value	EQU	b'01001111'	;T2 On, /64 pre, /10 post
PR2_Value	EQU	.125
;
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
LEDFastTime	EQU	d'20'
;
;T1CON_Val	EQU	b'00000001'	;Fosc=8MHz, PreScale=1,Fosc/4,Timer ON
T1CON_Val	EQU	b'00100001'	;Fosc=32MHz, PreScale=4,Fosc/4,Timer ON
;
;TXSTA_Value	EQU	b'00100000'	;8 bit, TX enabled, Async, low speed
TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, high speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
BAUDCON_Value	EQU	b'00001000'	;BRG16=1
; 8MHz clock low speed (BRGH=0,BRG16=1)
;Baud_300	EQU	d'1666'	;0.299, -0.02%
;Baud_1200	EQU	d'416'	;1.199, -0.08%
;Baud_2400	EQU	d'207'	;2.404, +0.16%
;Baud_9600	EQU	d'51'	;9.615, +0.16%
; 32MHz clock low speed (BRGH=1,BRG16=1)
Baud_300	EQU	.26666	;300, 0.00%
Baud_1200	EQU	.6666	;1200, 0.00%
Baud_2400	EQU	.3332	;2400, +0.01%
Baud_9600	EQU	.832	;9604, +0.04%
Baud_19200	EQU	.416	;19.18k, -0.08%
Baud_38400	EQU	.207	;38.46k, +0.16%
Baud_57600	EQU	.138	;57.55k, -0.08%
BaudRate	EQU	Baud_38400
;
kServoDwellTime	EQU	.40000	;20mS
kServoFastDwellTime	EQU	.20000	;10mS
kServoSpeed	EQU	.10	;Slow 5uS/Update
kssFlags	EQU	b'00011001'	;ssEnableFastPWM,ssMode3IdleCenter,ssEnableOverCur=true
kssMaxI	EQU	.50	;Low
kMidPulseWidth	EQU	.3000	;1500uS
;
                       if kSysMode==3
kMinPulseWidth	EQU	.100	;100 encoder counts
kMaxPulseWidth	EQU	.16280	;Max encoder value for ssCmdPos
                       else
kMinPulseWidth	EQU	.1800	;900uS
kMaxPulseWidth	EQU	.4200	;2100uS
                       endif
;
kServoCenterStop	EQU	.2945                  ;test value
kServoFastForward	EQU	kServoCenterStop+.100
kServoFastReverse	EQU	kServoCenterStop-.100
kDeadBand	EQU	.100	;100 encoder counts
kSysFlags	EQU	.0
kGripI	EQU	.40
;
DebounceTime	EQU	.10
kMaxMode	EQU	.4
;
; AuxIO modes
kAuxIOnone	EQU	0x00
kAuxIOLEDBtn	EQU	0x01
kAuxIODigitalIn	EQU	0x02
kAuxIODigitalOut	EQU	0x03
kAuxIOAnalogIn	EQU	0x04
kAuxIOHomeSw	EQU	0x05
kAuxIOFwdLimit	EQU	0x06
kAuxIORevLimit	EQU	0x07
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	SysLED_Time		;sys LED time
	SysLED_Blinks		;0=1 flash,1,2,3
	SysLED_BlinkCount
	SysLEDCount		;sys LED Timer tick count
;
	LED1_Blinks		;0=off,1,2,3
	LED2_Blinks
	LED1_BlinkCount		;LED1_Blinks..0
	LED2_BlinkCount
	LED1_Count		;tick count
	LED2_Count
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; 50 mS RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
;
;
	ssCmdPos:2		;Commanded position, 0=not used
	ssCurPos:2                                    ;Servo signal in 1/2 microseconds
	                                              ;Mode 3: Target Position
	ssTempFlags
	ssStatus:4		;Status and condition flags for the user.
;
	EncoderAccum:3		;Accumulated distance
	EncoderVal:2		;Value last read, raw 12 bit data
;-----------------------
;Below here are saved in eprom
	EncoderFlags
                       EncoderHome:2                                 ;Absolute Home
                       EncoderOffset:2		;Used in mode 2 for single rotation
;
	ServoFastReverse:2                            ;ServoFastReverse is less than
	ServoFastForward:2                            ; ServoFastForward
	ServoStopCenter:2		;Mode 3 Idle position
	ServoMin_uS:2
	ServoMax_uS:2
	ServoSpeed		;0 = off, 1..63 position change per cycle
;
                       if UsePID
	Kp		;8-bit proportional Gain
	Ki		;8-bit integral Gain
	Kd		;8-bit derivative Gain
	endif
;
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
	ssFlags		;Serial Servo flags
	ssMaxI		;Max Current 0=off
	DeadBand		;Used by Mode 2
	ssGripI		;Gripper tension
	ssAux0Config
	ssAux1Config
	ssAux2Config
	SysFlags		;saved in eprom 0x64 must
			; move something to another
			; bank before adding anything new
;
	endc
;--------------------------------------------------------------
;
MD3_MinCmd             equ                    ServoMin_uS
MD3_MaxCmd             equ                    ServoMax_uS
;
;---SerFlags bits---
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
;
;---ssTempFlags bits---
#Define	PulseSent	ssTempFlags,0
#Define	ServoOff	ssTempFlags,1
#Define	ServoIdle	ssTempFlags,2
#Define	OverCurrentFlag	ssTempFlags,3
#Define	GripIMet	ssTempFlags,4
#Define	GripIOver	ssTempFlags,5
;
;----ssStatus bits
#Define	MD3_FFwd	ssStatus,0
#Define	MD3_FRev	ssStatus,1
#Define	ssio_OverCurSD	ssStatus,2	;Servo stopped for over-current
#Define	ssRX_Timeout	ssStatus,3	;cleared by host read
#Define	ssGripOCur	ssStatus,4	;cleared by host read
#Define	ssGripMCur	ssStatus,5	;cleared by host read
;
; all bits of ssStatus+1 are cleared by a host kCmd_GetStatus command.
#Define	ssEncParityError	ssStatus+1,0	;cleared by host read
#Define	ssEncCmdError	ssStatus+1,1	;cleared by host read	
#Define                ssCmdPosVerified       ssStatus+1,2
;
;---------------
#Define	FirstRAMParam	EncoderFlags
#Define	LastRAMParam	SysFlags
;
;---ssFlags bits---
#Define	ssEnableOverCur	ssFlags,0	;disable if current is too high
#Define	ssReverseDir	ssFlags,1	;if set ServoFastForward<=>ServoFastReverse
;
#Define	ssMode3IdleCenter	ssFlags,3	;0= Disable PWM, 1= output ServoStopCenter
#Define	ssEnableFastPWM	ssFlags,4	;0= 20mS PWM, 1= 10mS PWM
#Define	ssEnableAN4	ssFlags,5	;0= Mode 0,1 disabled; 1= Enabled;
;
#Define	SW1_Flag	SysFlags,0
#Define	SW2_Flag	SysFlags,1
#Define	SW3_Flag	SysFlags,2
#Define	SW4_Flag	SysFlags,3
;
;================================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes
	cblock	0x0A0
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_SrcAdd:RP_AddressBytes
	RX_DstAdd:RP_AddressBytes
	RX_TempData:RP_DataBytes
	RX_Data:RP_DataBytes
	TX_Data:RP_DataBytes
;
	ANFlags
	Cur_AN0:2		;IServo
	Cur_AN1:2		;Battery Volts
	Cur_AN2:2		;SW1_LED1
	Cur_AN3:2		;SW2_LED2
	Cur_AN4:2                                     ;SW3_LED3, n/c on Rev C
;
	OldAN0Value:2
	endc
;
#Define	ServoCurrent	Cur_AN0
#Define	BattVolts	Cur_AN1
#Define	ModeZeroPot	Cur_AN2
;
;---ANFlags bits---
#Define	NewDataAN0	ANFlags,0
#Define	NewDataAN1	ANFlags,1
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
#Define	Ser_Buff_Bank	2
;
	cblock	0x120
	Ser_In_Bytes		;Bytes in Ser_In_Buff
	Ser_Out_Bytes		;Bytes in Ser_Out_Buff
	Ser_In_InPtr
	Ser_In_OutPtr
	Ser_Out_InPtr
	Ser_Out_OutPtr
	Ser_In_Buff:20
	Ser_Out_Buff:20
	endc
;
;================================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
; PID vars
	cblock	0x1A0
	derivCount		;This value determins how many times the Derivative term is
			;calculated based on each Integral term.
	pidOut0		;24-bit Final Result of PID for the "Plant"
	pidOut1
	pidOut2
	error0		;16-bit error, passed to the PID
	error1
	a_Error0		;24-bit accumulated error, used for Integral term
	a_Error1
	a_Error2
	p_Error0		;16-bit previous error, used for Derivative term
	p_Error1
	d_Error0		;16-bit delta error (error - previous error)
	d_Error1
;
	prop0		;24-bit proportional value
	prop1
	prop2
	integ0		;24-bit Integral value
	integ1
	integ2
	deriv0		;24-bit Derivative value
	deriv1
	deriv2
;
	pidStat1		;PID bit-status register
	pidStat2		;PID bit-status register2
;
; PIDMath
	PRODL
	PRODH
	AccB0		;LSB
	AccB1
	AccB2
	AccB3		;MSB
	AArgB0
	AArgB1
	AArgB2
	AArgB3
	BArgB0
	BArgB1
	BArgB2
	BArgB3
	RemB0
	RemB1
	RemB2
	RemB3
	endc
;
;___________________________ pidStat1 register ________________________________________________
;|  bit 7   |   bit 6    |  bit 5 |    bit 4   |   bit 3    |  bit 2   |   bit 1    |  bit 0   |
;| pid_sign | d_err_sign |        | p_err_sign | a_err_sign | err_sign |  a_err_z   |  err_z   |
;|__________|____________|________|____________|____________|__________|____________|__________|
;
#Define	err_z	pidStat1,0	;error zero flag, Zero = set
#Define	a_err_z	pidStat1,1	;a_error zero flag, Zero = set
#Define	err_sign	pidStat1,2	;error sign flag, Pos = set/ Neg = clear
#Define	a_err_sign	pidStat1,3	;a_error sign flag, Pos = set/ Neg = clear
#Define	p_err_sign	pidStat1,4	;p_error sign flag, Pos = set/ Neg = clear
;
#Define	d_err_sign	pidStat1,6	;d_error sign flag, Pos = set/ Neg = clear
#Define	pid_sign	pidStat1,7	;PID result sign flag, Pos = set/ Neg = clear
;
;________________________________ pidStat2 register______________________________________
;| bit 7 |  bit 6  |  bit 5   |    bit 4   |   bit 3    |  bit 2    |   bit 1    |  bit 0   |
;|       |         |          | error_limit| deriv_sign | BArg_sign | AArg_Sign  | d_err_z  |
;|_______|_________|__________|____________|____________|___________|____________|__________|
;
#Define	d_err_z	pidStat2,0	;d_error zero flag, Zero = set
#Define	AArg_sign	pidStat2,1	;AArg sign flag, Pos = set/ Neg = clear
#Define	BArg_sign	pidStat2,2	;BArg sign flag, Pos = set/ Neg = clear
#Define	deriv_sign	pidStat2,3	;deriv sign flag, Pos = set/ Neg = clear
#Define	error_limit	pidStat2,4	;Error limit exceeded flag, error = set/ no error = clear
;
;=========================================================================================
;  Bank4 Ram 220h-26Fh 80 Bytes
;=========================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
	cblock	0x2A0
	SigOutTime
	SigOutTimeH
	CalcdDwell
	CalcdDwellH
	endc
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
AS5047D_Flags	EQU	Param70	;Check that Param70 is OK to use
;
#Define	ParityErrFlag	AS5047D_Flags,0
#Define	AngleReadFlag	AS5047D_Flags,1
#define	ContinueReadFlag	AS5047D_Flags,2
#Define	CmdErrorFlag	AS5047D_Flags,3
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10b4
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;
; default values
	ORG	0xF000
	de	0x00	;nvEncoderFlags
	de	0x00,0x00	;nvEncoderHome
	de	0x00,0x00	;nvEncoderOffset
	de	low kServoFastReverse
	de	high kServoFastReverse
	de	low kServoFastForward
	de	high kServoFastForward
	de	low kServoCenterStop	;nvServoStopCenter
	de	high kServoCenterStop
	de	low kMinPulseWidth	;nvServoMin_uS
	de	high kMinPulseWidth
	de	low kMaxPulseWidth	;nvServoMax_uS
	de	high kMaxPulseWidth
	de	kServoSpeed	;nvServoSpeed
	if UsePID
	de	Default_Kp	;8-bit proportional Gain
	de	Default_Ki	;8-bit integral Gain
	de	Default_Kd	;8-bit derivative Gain
	endif
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr, 0x0F
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr, 0x10
	de	kssFlags	;nvssFlags
	de	kssMaxI	;nvssMaxI
	de	kDeadBand	;nvDeadBand
	de	kGripI
	de	0x00	;ssAux0Config
	de	0x00	;ssAux1Config
	de	0x00	;ssAux2Config
	de	kSysFlags	;nvSysFlags
;
	ORG	0xF0FF
	de	0x00	;Skip BootLoader
;
	cblock	0x0000
;
	nvEncoderFlags
                       nvEncoderHome:2
                       nvEncoderOffset:2
;
	nvServoFastReverse:2
	nvServoFastForward:2
	nvServoStopCenter:2
	nvServoMin_uS:2
	nvServoMax_uS:2
	nvServoSpeed
;
	if UsePID
                       nvKp
                       nvKi
                       nvKd
                       endif
;
	nvSysMode
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
	nvssFlags
	nvssMaxI
	nvDeadBand
	nvssGripI
	nvssAux0Config
	nvssAux1Config
	nvssAux2Config
	nvSysFlags
	endc
;
#Define	nvFirstParamByte	nvEncoderFlags
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
BootLoaderStart	EQU	0x1E00
;
	ORG	0x000	; processor reset vector
	movlp	BootLoaderStart
	goto	BootLoaderStart
ProgStartVector	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	PCLATH
	CLRF	BSR	; bank0
;
;
	BTFSS	PIR1,TMR2IF
	goto	SystemTick_end
;
	BCF	PIR1,TMR2IF	; reset interupt flag bit
;------------------
; These routines run 100 times per second
;
;------------------
;Decrement timers until they are zero
;
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
;
; All LEDs off
	movlb	0x01	;bank 1
	bsf	SysLED_Tris
;
	BSF	LED1_Tris
	BSF	LED2_Tris
;
; Read Switches
	movlb	0x00	;bank 0
;--------------------
; Sys LED time
	DECFSZ	SysLEDCount,F	;Is it time?
	bra	SystemBlink_end	; No, not yet
;
	movf	SysLED_Blinks,F
	SKPNZ		;Standard Blinking?
	bra	SystemBlink_Std	; Yes
;
; custom blinking
;
SystemBlink_Std	CLRF	SysLED_BlinkCount
	MOVF	SysLED_Time,W
SystemBlink_DoIt	MOVWF	SysLEDCount
	movlb	0x01	;bank 1
	bcf	SysLED_Tris	;LED ON
SystemBlink_end:
;--------------------
; Flash LEDs
	movlb	0x00	;bank 0
	movf	ssAux0Config,W
	andlw	0x0F
	sublw	kAuxIOLEDBtn
	SKPZ
	bra	LED1_Blink_end
; Get Button Value
	movlb	0x01	;bank 1
	BSF	LED1_Tris
	movlb	0x00	;bank 0
	BCF	SW1_Flag
	BTFSS	SW1_In
	BSF	SW1_Flag
;
	movf	LED1_Blinks,F
	SKPZ		;LED1 active?
	bra	LED1_Blinking	; Yes
	clrf	LED1_BlinkCount
	clrf	LED1_Count
	bra	LED1_Blink_end
;
LED1_Blinking	movf	LED1_Count,W
	iorwf	LED1_BlinkCount,W
	SKPNZ		;Startup?
	bra	LED1_Start
;
	decfsz	LED1_Count,F	;Done w/ blink
	bra	LED1_Blink_end	; no
;
	movf	LED1_BlinkCount,F
	SKPNZ		;Done w/ cycle?
	bra	LED1_Start	; Yes
;
	decfsz	LED1_BlinkCount,F
	bra	LED1_NextBlink
	movlw	LEDTIME	;long off time
	movwf	LED1_Count
	bra	LED1_Blink_end
;
LED1_Start	movf	LED1_Blinks,W
	movwf	LED1_BlinkCount
LED1_NextBlink	movlw	LEDFastTime
	movwf	LED1_Count
;
	movlb	0x01
	BCF	LED1_Tris
LED1_Blink_end:
;-------------
;kAuxIODigitalOut
	movlb	0x00	;bank 0
	movf	ssAux0Config,W
	andlw	0x0F
	sublw	kAuxIODigitalOut
	SKPZ
	bra	Aux0DigOut_end
;
	btfss	LED1_Blinks,0
	bra	Aux0DigOut_1
	movlb	0x02	;bank 2
	bsf	LED1_Lat
	bra	Aux0DigOut_2
;
Aux0DigOut_1	movlb	0x02	;bank 2
	bcf	LED1_Lat
	bra	Aux0DigOut_2
;
Aux0DigOut_2	movlb	0x01	;bank 1
	BCF	LED1_Tris
Aux0DigOut_end:
;-------------
	movlb	0x00	;bank 0
	movf	ssAux1Config,W
	andlw	0x0F
	sublw	kAuxIOLEDBtn
	SKPZ
	bra	LED2_Blink_end
; Get Button Value
	movlb	0x01	;bank 1
	BSF	LED2_Tris
	movlb	0x00	;bank 0
	BCF	SW2_Flag
	BTFSS	SW2_In
	BSF	SW2_Flag
;
	movf	LED2_Blinks,F
	SKPZ		;LED2 active?
	bra	LED2_Blinking	; Yes
	clrf	LED2_BlinkCount
	clrf	LED2_Count
	bra	LED2_Blink_end
;
LED2_Blinking	movf	LED2_Count,W
	iorwf	LED2_BlinkCount,W
	SKPNZ		;Startup?
	bra	LED2_Start
;
	decfsz	LED2_Count,F	;Done w/ blink
	bra	LED2_Blink_end	; no
;
	movf	LED2_BlinkCount,F
	SKPNZ		;Done w/ cycle?
	bra	LED2_Start	; Yes
;
	decfsz	LED2_BlinkCount,F
	bra	LED2_NextBlink
	movlw	LEDTIME	;long off time
	movwf	LED2_Count
	bra	LED2_Blink_end
;
LED2_Start	movf	LED2_Blinks,W
	movwf	LED2_BlinkCount
LED2_NextBlink	movlw	LEDFastTime
	movwf	LED2_Count
;
	movlb	0x01
	BCF	LED2_Tris
LED2_Blink_end:
;-------------
;kAuxIODigitalOut
	movlb	0x00	;bank 0
	movf	ssAux1Config,W
	andlw	0x0F
	sublw	kAuxIODigitalOut
	SKPZ
	bra	Aux1DigOut_end
;
	btfss	LED2_Blinks,0
	bra	Aux1DigOut_1
	movlb	0x02	;bank 2
	bsf	LED2_Lat
	bra	Aux1DigOut_2
;
Aux1DigOut_1	movlb	0x02	;bank 2
	bcf	LED2_Lat
	bra	Aux1DigOut_2
;
Aux1DigOut_2	movlb	0x01	;bank 1
	BCF	LED2_Tris
Aux1DigOut_end:
;-------------
;
SystemTick_end:
;
;==================================================================================
;
; Handle CCP1 Interupt Flag, Enter w/ bank 0 selected
;
IRQ_Servo1	MOVLB	0	;bank 0
	BTFSS	PIR1,CCP1IF
	bra	IRQ_Servo1_End
;
	bsf	PulseSent	;ok to update CurPos
;
	BTFSS	ServoOff	;Are we sending a pulse?
	bra	IRQ_Servo1_1	; Yes
;
;Servo is off, idle CCP1 and keep output low
	MOVLB	0x05	;Bank 5
	movlw	CCP1CON_Idle
	movwf	CCP1CON
	bra	IRQ_Servo1_Dwell
;
IRQ_Servo1_1	btfsc	ServoIdle
	bra	IRQ_Servo1_Idle
	MOVLB	0x05	;Bank 5
	BTFSC	CCP1CON,CCP1M1	;Idling?
	bra	IRQ_Servo1_OL	; Yes, go high after dwell
	BTFSC	CCP1CON,CCP1M0	;Cleared output on match?
	bra	IRQ_Servo1_OL	; No
; An output just went high
;
IRQ_Servo1_OH	MOVF	SigOutTime,W	;Put the pulse into the CCP reg.
	ADDWF	CCPR1L,F
	MOVF	SigOutTime+1,W
	ADDWFC	CCPR1H,F
	MOVLW	CCP1CON_Clr	;Clear output on match
	MOVWF	CCP1CON	;CCP1 clr on match
;Calculate dwell time
	movlb	0	;bank 0
	btfss	ssEnableFastPWM
	bra	IRQ_Servo1_20mS
	movlb	5	;Bank 5
	MOVLW	LOW kServoFastDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoFastDwellTime
	MOVWF	CalcdDwellH
	bra	IRQ_Servo1_CalcDwell
;
IRQ_Servo1_20mS	movlb	5	;Bank 5
	MOVLW	LOW kServoDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTime
	MOVWF	CalcdDwellH
;
IRQ_Servo1_CalcDwell	MOVF	SigOutTime,W
	SUBWF	CalcdDwell,F
	MOVF	SigOutTime+1,W
	SUBWFB	CalcdDwellH,F
	bra	IRQ_Servo1_X
;
IRQ_Servo1_Idle	MOVLB	0x05
	BTFSC	CCP1CON,CCP1M1	;Idling?
	bra	IRQ_Servo1_Dwell	; yes, continue idling.
	BTFSS	CCP1CON,CCP1M0	;Just went low?
	bra	IRQ_Servo1_OH	; No, finish pulse
	movlw	CCP1CON_Idle	; Yes, start idling
	movwf	CCP1CON
	MOVLW	LOW kServoDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTime
	MOVWF	CalcdDwellH
	bra	IRQ_Servo1_Dwell
;
; output went low so this cycle is done
IRQ_Servo1_OL	MOVLW	CCP1CON_Set	;Set output on match
	MOVWF	CCP1CON
;
IRQ_Servo1_Dwell	MOVF	CalcdDwell,W
	ADDWF	CCPR1L,F
	MOVF	CalcdDwellH,W
	ADDWFC	CCPR1H,F
;
IRQ_Servo1_X	MOVLB	0x00
	BCF	PIR1,CCP1IF
IRQ_Servo1_End:
;-----------------------------------------------------------------------------------------
;AUSART Serial ISR
;
IRQ_Ser	BTFSS	PIR1,RCIF	;RX has a byte?
	BRA	IRQ_Ser_End
	CALL	RX_TheByte
;
IRQ_Ser_End:
;-----------------------------------------------------------------------------------------
	retfie		; return from interrupt
;
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	include <F1847_Common.inc>
	include <SerBuff1938.inc>
	include <RS232_Parse.inc>
;
;=========================================================================================
;
start	mLongCall	InitializeIO
;
	CALL	StartServo
	CALL	ReadAN0_ColdStart
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
MainLoop	CLRWDT
;
	call	GetSerInBytes
	SKPZ		;Any data?
	CALL	RS232_Parse	; yes
;
	movlb	1
	btfss	RXDataIsNew
	bra	ML_1
	mLongCall	HandleRXData
ML_1:
;
; Fast blink the system LED is the servo is stopped because of an error
	MOVLB	0x00
	MOVLW	LEDTIME
	btfsc	ssio_OverCurSD
	movlw	LEDErrorTime
	MOVWF	SysLED_Time
;
	CALL	ReadAN
;
; Average AN0
	BankSel	Cur_AN0
	btfss	NewDataAN0
	bra	No_NewDataAN0
	bcf	NewDataAN0
	movf	OldAN0Value,W
	addwf	Cur_AN0,F
	movf	OldAN0Value+1,W
	addwfc	Cur_AN0+1,F
	lsrf	Cur_AN0+1,W
	movwf	Cur_AN0+1
	movwf	OldAN0Value+1
	rrf	Cur_AN0,W
	movwf	Cur_AN0
	movwf	OldAN0Value
;
No_NewDataAN0:
	mLongCall	ReadEncoder
;
	call	HandleButtons
;
;---------------------
; Handle Serial Communications
	BTFSC	PIR1,TXIF	;TX done?
	CALL	TX_TheByte	; Yes
;
; move any serial data received into the 32 byte input buffer
	BTFSS	DataReceivedFlag
	BRA	ML_Ser_Out
	MOVF	RXByte,W
	BCF	DataReceivedFlag
	CALL	StoreSerIn
;
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
ML_Ser_Out	BTFSS	DataSentFlag
	BRA	ML_Ser_End
	CALL	GetSerOut
	BTFSS	Param78,0
	BRA	ML_Ser_End
	MOVWF	TXByte
	BCF	DataSentFlag
ML_Ser_End:
;----------------------
;
	movlb	0x00	;bank 0
	movf	SysMode,W
	brw
	goto	DoModeZero
	goto	DoModeOne
	goto	DoModeTwo	
	goto	DoModeThree
	goto	DoMode4
;
ModeReturn:
;
	goto	MainLoop
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;Simple servo testing
; copy AN4 value x2 + .1976 to servo value
;
DoModeZero:
	BankSel	ModeZeroPot
	lslf	ModeZeroPot,W
	movwf	Param7C
	rlf	ModeZeroPot+1,W
	movwf	Param7D
	movlw	low .1976
	addwf	Param7C,F
	movlw	high .1976
	addwfc	Param7D,F
;
	call	ClampInt
	call	Copy7CToSig
;
	goto	ModeReturn
;=========================================================================================
;Testing servo and encoder
; if AN4 + .950 > EncoderVal set servo to ServoFastForward
; elseif AN4 + .1050 < EncoderVal set servo to ServoFastReverse
; else Set ServoIdle
;
DoModeOne:
	BankSel	ModeZeroPot
;
;Param7A:Param79 = Cur_AN4 + .950
	movlw	low .950
	addwf	ModeZeroPot,W
	movwf	Param79
	movlw	high .950
	addwfc	ModeZeroPot+1,W
	movwf	Param7A
;
;Param7A:Param79 = Param7A:Param79 - EncoderVal
	BankSel	EncoderVal
	movf	EncoderVal,W
	subwf	Param79,F
	movf	EncoderVal+1,W
	subwfb	Param7A,F
;
	btfss	Param7A,7	;Param7A:Param79 < 0?
	bra	DM1_FF	; No, EncoderVal <= (AN4 + .950)
;
;Param7A:Param79 = Cur_AN4 + .1050
	BankSel	ModeZeroPot
	movlw	low .1050
	addwf	ModeZeroPot,W
	movwf	Param79
	movlw	high .1050
	addwfc	ModeZeroPot+1,W
	movwf	Param7A
;
;Param7A:Param79 = Param7A:Param79 - EncoderVal
	BankSel	EncoderVal
	movf	EncoderVal,W
	subwf	Param79,F
	movf	EncoderVal+1,W
	subwfb	Param7A,F
;
	btfsc	Param7A,7	;Param7A:Param79 < 0?
	bra	DM1_FR	; Yes, EncoderVal > (AN4 + .1050)
;
; EncoderVal > (ModeZeroPot + .950) && EncoderVal <= (ModeZeroPot + .1050)
	bsf	ServoIdle
	goto	ModeReturn
;
DM1_FF	movf	ServoFastForward,W
	movwf	Param7C
	movf	ServoFastForward+1,W
	movwf	Param7D
	call	Copy7CToSig
	goto	ModeReturn
;
DM1_FR	movf	ServoFastReverse,W
	movwf	Param7C
	movf	ServoFastReverse+1,W
	movwf	Param7D
	call	Copy7CToSig
	goto	ModeReturn
;
;=========================================================================================
; if ssEnableOverCur and Cur_AN0>ssMaxI*4 then
;   OverCurrentFlag=true
CheckCurrent	movlb	0x00	;Bank 0
	btfss	ssEnableOverCur
	return
;Param79:Param78 = ssMaxI * 4
	clrf	Param79
	lslf	ssMaxI,W
	movwf	Param78
	rlf	Param79,F
	lslf	Param78,F
	rlf	Param79,F
;Param79:Param78 -= Cur_AN0
	movlb	ServoCurrent
	movf	ServoCurrent,W
	subwf	Param78,F
	movf	ServoCurrent+1,W
	subwfb	Param79,F
	movlb	0x00	;Bank 0
;
	btfsc	Param79,7	;Cur_AN0>ssMaxI*4?
	bsf	OverCurrentFlag
	return
;
;=========================================================================================
; if Cur_AN0>ssGripI*4 then
;   GripIMet=true
; if Cur_AN0>(ssGripI+0x10)*4 then
;   GripIOver=true
CheckGripCurrent	movlb	0x00	;Bank 0
;Param79:Param78 = ssGripI * 4
	clrf	Param79
	lslf	ssGripI,W
	movwf	Param78
	rlf	Param79,F
	lslf	Param78,F
	rlf	Param79,F
;Param79:Param78 -= Cur_AN0
	BankSel	ServoCurrent
	movf	ServoCurrent,W
	subwf	Param78,F
	movf	ServoCurrent+1,W
	subwfb	Param79,F
	movlb	0x00	;Bank 0
;
	btfsc	Param79,7	;Cur_AN0>ssGripI*4?
	bsf	GripIMet	; Yes
	btfsc	Param79,7	;Cur_AN0>ssGripI*4?
	bsf	ssGripMCur
;Param79:Param78 = (ssGripI+0x10) * 4
	clrf	Param79
	movlw	kGripperHC
	addwf	ssGripI,W
	movwf	Param78
	movlw	0x00
	addwfc	Param79,F
	lslf	Param78,F
	rlf	Param79,F
	lslf	Param78,F
	rlf	Param79,F
;Param79:Param78 -= Cur_AN0
	BankSel	ServoCurrent
	movf	ServoCurrent,W
	subwf	Param78,F
	movf	ServoCurrent+1,W
	subwfb	Param79,F
	movlb	0x00	;Bank 0
;
	btfsc	Param79,7	;Cur_AN0>(ssGripI+10)*4?
	bsf	GripIOver
	btfsc	Param79,7	;Cur_AN0>(ssGripI+10)*4?
	bsf	ssGripOCur
	return
;
;=========================================================================================
;Idle routine for Basic Serial Servo mode
;
DoModeTwo	movlb	0
	btfsc	ssCmdPos+1,7	;Any command issued?
	bra	DoModeTwo_1	; No, Idle the servo
;
;Check for over current, kill position command if over current is detected.
	call	CheckCurrent
	btfss	OverCurrentFlag
	bra	DM2_NotOverCurrent
	clrf	ssCmdPos
	clrf	ssCmdPos+1
	bsf	ssCmdPos+1,7
	bsf	ssio_OverCurSD
	bcf	OverCurrentFlag
	bra	DoModeTwo_1
;
DM2_NotOverCurrent:
	bcf	ssio_OverCurSD
	movf	ServoSpeed,F
	SKPNZ		;Speed = 0?
	bra	DoModeTwo_NoSpeed	; yes
	btfss	PulseSent	;Time to update?
	goto	ModeReturn	; No
	bcf	PulseSent
;Param7D:Param7C = Cmd-Cur
	movf	ssCurPos,W
	subwf	ssCmdPos,W
	movwf	Param7C
	movf	ssCurPos+1,W
	subwfb	ssCmdPos+1,W
	movwf	Param7D
; if Param7D:Param7C = 0 then we are In Position
	iorwf	Param7C,W
	SKPNZ
	bra	DoModeTwo_Go	; if Cmd = Cur Go
;
;
	BTFSS	Param7D,7	;Cmd<Cur? Set if Cur>Cmd
	GOTO	DoModeTwo_MovPlus	; Yes
;Move minus
	INCFSZ	Param7D,W	;Dist=0xFFxx?
	GOTO	DoModeTwo_Minus	; No
	MOVF	ServoSpeed,W
	ADDWF	Param7C,W
	BTFSC	_C	;Dist<Speed?
	bra	DoModeTwo_NoSpeed	; No
;
; Subtract speed from current position
DoModeTwo_Minus	MOVF	ServoSpeed,W
	SUBWF	ssCurPos,F	;SigOutTime
	MOVLW	0x00
	SUBWFB	ssCurPos+1,F	;SigOutTimeH
	bra	DoModeTwo_Go
;
;=============================
; 7D:7C = distance to go
;
DoModeTwo_MovPlus	MOVF	Param7D,F
	SKPZ		;Dist>255 to go?
	bra	DoModeTwo_Plus	; Yes
	MOVF	ServoSpeed,W
	SUBWF	Param7C,W	;Dist-Speed
	SKPNB		;Speed>Dist?
	bra	DoModeTwo_NoSpeed	; Yes
;
DoModeTwo_Plus	MOVF	ServoSpeed,W	;7D:7C = CurPos + Speed
	ADDWF	ssCurPos,F
	CLRW
	ADDWFC	ssCurPos+1,F
	bra	DoModeTwo_Go
;
;
; set current position at command position
DoModeTwo_NoSpeed	movf	ssCmdPos,W
	movwf	ssCurPos
	movf	ssCmdPos+1,W
	movwf	ssCurPos+1
; make it so
DoModeTwo_Go	movf	ssCurPos,W
	movwf	Param7C
	movf	ssCurPos+1,W
	movwf	Param7D
	call	ClampInt
	call	Copy7CToSig
	goto	ModeReturn
;
DoModeTwo_1:
	bsf	ServoIdle	;power down servo
	goto	ModeReturn
;
;=========================================================================================
;Idle routine for Absolute encoder position control w/ continuous rotation servo.
;
;Entry: ssCmdPos = user's command, ssCurPos
;
;error = ((EncoderVal + EncoderOffset) mod 16384)-ssCmdPos
;if DeadBand>0 then
;  if abs(error)<DeadBand then error=0
;
;if error=0 then
;  IdleServo
;else
;  if error<-128 then error= -128
;  if error>127 then error = 127
;  servo=ServoStopCenter-error
;
; Ram Used:Param79,Param7A,Param7C,Param7D
;
DoModeThree	movlb	0	;bank 0
	btfsc	ssCmdPos+1,7
	bra	DM3_ServoHere
;
;Check for over current, kill position command if over current is detected.
	call	CheckCurrent
	btfss	OverCurrentFlag
	bra	DM3_NotOverCurrent
;
	bsf	ssio_OverCurSD
	bcf	OverCurrentFlag
;Flag as no cmd pos.
	clrf	ssCmdPos
	clrf	ssCmdPos+1
	bsf	ssCmdPos+1,7
; Servo Here
DM3_ServoHere	movf	EncoderVal,W
	addwf	EncoderOffset,W
                       movwf                  ssCurPos
	movf	EncoderVal+1,W
	addwfc	EncoderOffset+1,W
                       movwf                  ssCurPos+1
;
	bra	DM3_IdleServo
;
DM3_NotOverCurrent	bcf	ssio_OverCurSD
                       btfsc                  ssCmdPosVerified       ;Has been verified?
                       bra                    DM3_CPV_End            ; Yes
                       movf                   ssCmdPos,W             ; No, Clamp and mark as verified.
                       movwf                  Param7C
                       movf                   ssCmdPos+1,W
                       movwf                  Param7D
                       call                   ClampInt               ;MD3_MinCmd<=ssCurPos<=MD3_MaxCmd
                       movf                   Param7C,W
                       movwf                  ssCmdPos
                       movf                   Param7D,W
                       movwf                  ssCmdPos+1
                       bsf                    ssCmdPosVerified
DM3_CPV_End:
;
; if speed = 0 then just be there
                       movf                   ServoSpeed,F
                       SKPZ
                       bra                    DM3_CalcCurPos
DM3_ServoThere         movf                   ssCmdPos,W
                       movwf                  ssCurPos
                       movf                   ssCmdPos+1,W
                       movwf                  ssCurPos+1
                       bra                    DM3_SetServoPWM
;
DM3_CalcCurPos         btfss	PulseSent	;Time to update?
	bra	DM3_SetServoPWM	; No
	bcf	PulseSent
;
;if ssCmdPos<>ssCurPos then
;  if ssCmdPos>ssCurPos then
;    if ssCmdPos>ssCurPos+ServoSpeed then
;      ssCurPos += ServoSpeed
;    else
;      ssCurPos = ssCmdPos
;  else
;    if ssCmdPos<ssCurPos-ServoSpeed then
;      ssCurPos -= ServoSpeed
;    else
;      ssCurPos = ssCmdPos
;
                       movf                   ssCmdPos,W             ;ssCurPos-ssCmdPos
                       subwf                  ssCurPos,W
                       movwf                  Param78
                       movf                   ssCmdPos+1,W
                       subwfb                 ssCurPos+1,W
                       iorwf                  Param78,W
                       SKPNZ                                         ;ssCmdPos=ssCurPos/
                       bra                    DM3_SetServoPWM        ; Yes
;
                       SKPB                                          ;ssCmdPos>ssCurPos?
                       bra                    DM3_GoRev              ; No
; ssCmdPos>ssCurPos forward
                       bsf                    MD3_FFwd
                       bcf                    MD3_FRev
;ssCurPos += ServoSpeed
                       movf                   ServoSpeed,W
                       addwf                  ssCurPos,F
                       movlw                  0x00
                       addwfc                 ssCurPos+1,F
;
                       movf                   ssCmdPos,W             ;(ssCurPos+Speed)-ssCmdPos
                       subwf                  ssCurPos,W
                       movf                   ssCmdPos+1,W
                       subwfb                 ssCurPos+1,W
                       SKPB
                       bra                    DM3_ServoThere
                       bra                    DM3_SetServoPWM
;                       
; ssCmdPos<ssCurPos reverse
DM3_GoRev              bcf                    MD3_FFwd
                       bsf                    MD3_FRev
;
                       movf                   ServoSpeed,W
                       subwf                  ssCurPos,F
                       movlw                  0x00
                       subwfb                 ssCurPos+1,F
;
                       movf                   ssCmdPos,W             ;(ssCurPos-Speed)-ssCmdPos
                       subwf                  ssCurPos,W
                       movf                   ssCmdPos+1,W
                       subwfb                 ssCurPos+1,W
                       SKPNB
                       bra                    DM3_ServoThere
;
;Param7A:Param79 = ((EncoderVal + EncoderOffset) mod 16384)
DM3_SetServoPWM	movf	EncoderVal,W
	addwf	EncoderOffset,W
	movwf	Param79
	movf	EncoderVal+1,W
	addwfc	EncoderOffset+1,W
	andlw	0x3F
	movwf	Param7A
; Calculate Error
;Param7A:Param79 = ((EncoderVal + EncoderOffset) mod 16384) - ssCurPos
	movf	ssCurPos,W	;(EncoderVal + EncoderOffset) mod 16384
	subwf	Param79,F
	movf	ssCurPos+1,W
	subwfb	Param7A,F
; if error = 0 then idle
                       movf                   Param79,W
                       iorwf                  Param7A,W
                       SKPNZ
                       bra                    DM3_IdleServo
; if DeadBand = 0 then skip DB check
                       movf                   DeadBand,F
                       SKPNZ
                       bra                    DM3_NoDB
; if error<0 then Error_a=abs(error), Param7D:Param7C=abs(Param7A:Param79)
                       movf                   Param79,W
                       movwf                  Param7C
                       movf                   Param7A,W
                       movwf                  Param7D
                       btfss                  Param7A,7
                       bra                    DM3_ErrIsPos
                       clrf                   Param7C
                       clrf                   Param7D
                       movf                   Param79,W
                       subwf                  Param7C,F
                       movf                   Param7A,W
                       subwfb                 Param7D,F
;
;if Error_a>255 then ignor DB
DM3_ErrIsPos           movf                   Param7D,F
                       SKPZ                                          ;Error>255?
                       bra                    DM3_NoDB               ; Yes
                       movf                   DeadBand,W
                       subwf                  Param7C,F              ;Param7C = Error - DB
                       SKPNB                                         ;DB>Error?
                       bra                    DM3_IdleServo          ; Yes
;
; if error<-128 then error = -128
DM3_NoDB               btfss                  Param7A,7              ;Error is negative?
                       bra                    DM3_PosLimit           ; No
                       movlw                  0x7F
                       iorwf                  Param79,W              ;high bit only
                       andwf                  Param7A,W
                       xorlw                  0xFF
                       SKPNZ                                         ;< -128?
                       bra                    DM3_CalcSCmd           ; No
                       movlw                  0x80                   ;-128
                       movwf                  Param79
                       bra                    DM3_CalcSCmd
;
; if error >= 128 then error = 127
DM3_PosLimit           movlw                  0x80
                       andwf                  Param79,W              ;hi bit only
                       iorwf                  Param7A,W              ;or w/ hi byte
                       SKPNZ                                         ;>= 128?
                       bra                    DM3_CalcSCmd           ; No
                       movlw                  0x7F
                       movwf                  Param79
;
DM3_CalcSCmd           btfss                  ssReverseDir           ;Moves reversed?
                       bra                    DM3_CalcSCmd_1         ; No
                       movf                   Param79,W
                       sublw                  0x80                   ;test for -128
                       SKPZ                                          ;Is -128?
                       bra                    DM3_CalcSCmd_2s
                       movlw                  0x81                   ; Yes, Make it -127
                       movwf                  Param79
DM3_CalcSCmd_2s        movf                   Param79,W
                       sublw                  0x00                   ; Yes, 2's comp
                       movwf                  Param79
DM3_CalcSCmd_1:      
;
;0.5 x gain
;                       asrf                   Param79,W
;
;
                       movf                   Param79,W
                       subwf                  ServoStopCenter,W
                       movwf                  Param7C
                       movlw                  0x00
                       btfsc                  Param79,7              ;is neg?
                       movlw                  0xFF                   ; yes, sign extend it
                       subwfb                 ServoStopCenter+1,W
                       movwf                  Param7D
                       bra	DM3_UpdatePos
;
;
; abs(Error) <= DeadBand
; if ssMode3IdleCenter then
;   servo=ServoStopCenter
; else
;   ServoIdle=true
;
DM3_IdleServo	btfss	ssMode3IdleCenter
	bra	DM3_IdleInactive
	movf	ServoStopCenter,W
	movwf	Param7C
	movf	ServoStopCenter+1,W
	movwf	Param7D
                       bcf                    MD3_FFwd
                       bcf                    MD3_FRev
	bcf	ServoIdle
;
; set current position at destination position
; Entry: Param7D:Param7C servo signal in 1/2 microseconds
;
DM3_UpdatePos	call	ClampIntMD3
	call	Copy7CToSig
	goto	ModeReturn
;
;
DM3_IdleInactive	bsf	ServoIdle
                       bcf                    MD3_FFwd
                       bcf                    MD3_FRev
	goto	ModeReturn
;
;=============================
;=========================================================================================
;Idle routine for Gripper Serial Servo mode
; Servo is set to idle only is no command or over current.
;
DoMode4	movlb	0
	btfsc	ssCmdPos+1,7	;Any command issued?
	bra	DoMode4_1	; No, Idle the servo
;
;Check for over current, kill position command if over current is detected.
	call	CheckCurrent
	call	CheckGripCurrent
	btfss	OverCurrentFlag
	bra	DM4_NotOverCurrent
	clrf	ssCmdPos	;kill the command
	clrf	ssCmdPos+1
	bsf	ssCmdPos+1,7
	bsf	ssio_OverCurSD
	bcf	OverCurrentFlag
	bra	DoMode4_1	;Idle the servo
;
DM4_NotOverCurrent:
; Speed cannot be 0, if 0 set to 1 (slow)
	bcf	ssio_OverCurSD
	movf	ServoSpeed,F
	SKPNZ		;Speed = 0?
	incf	ServoSpeed,F	; yes, make it 1
	btfss	PulseSent	;Time to update?
	goto	ModeReturn	; No
	bcf	PulseSent
;Param7D:Param7C = Cmd-Cur
	movf	ssCurPos,W
	subwf	ssCmdPos,W
	movwf	Param7C
	movf	ssCurPos+1,W
	subwfb	ssCmdPos+1,W
	movwf	Param7D
; if Param7D:Param7C = 0 then we are In Position
	iorwf	Param7C,W
	SKPNZ
	bra	DoMode4_Hold	; if Cmd = Cur Go
;
;Sign bit set if Cur>Cmd
	BTFSS	Param7D,7	;Cmd>Cur?
	bra	DoMode4_MovPlus	; Yes
;Move minus
	INCFSZ	Param7D,W	;Dist=0xFFxx?
	GOTO	DoMode4_Minus	; No
	MOVF	ServoSpeed,W
	ADDWF	Param7C,W
	BTFSS	_C	;Dist<Speed?
	bra	DoMode4_Minus	; Yes
	movlw	0x01	; No, use 1 as speed
	bra	DoMode4_Minus_1
;
; Subtract speed from current position
DoMode4_Minus	MOVF	ServoSpeed,W
DoMode4_Minus_1	SUBWF	ssCurPos,F	;SigOutTime
	MOVLW	0x00
	SUBWFB	ssCurPos+1,F	;SigOutTimeH
	bra	DoMode4_Go
;
;=============================
; if Cur_AN0>(ssGripI+0x10)*4 then move minus 1
DoMode4_Hold	btfss	GripIOver	;Gripping too hard?
	goto	DoMode4_Go	; No
	movlw	0x01	; No, use 2 as speed
	bra	DoMode4_Minus_1
;
;=============================
; 7D:7C = distance to go
;
DoMode4_MovPlus	btfsc	GripIMet	;Servo Current > ssGripI?
	bra	DoMode4_Hold	; Yes, don't move more closed.
;
	MOVF	Param7D,F
	SKPZ		;Dist>255 to go?
	bra	DoMode4_Plus	; Yes
	MOVF	ServoSpeed,W
	SUBWF	Param7C,W	;Dist-Speed
	SKPB		;Speed>Dist?
	bra	DoMode4_Plus	; No
	movlw	0x01	;Use 1 as speed
	bra	DoMode4_Plus_1
;
DoMode4_Plus	MOVF	ServoSpeed,W	;CurPos += Speed
DoMode4_Plus_1	ADDWF	ssCurPos,F
	CLRW
	ADDWFC	ssCurPos+1,F
;
; make it so
DoMode4_Go	movf	ssCurPos,W	;7D:7C = CurPos
	movwf	Param7C
	movf	ssCurPos+1,W
	movwf	Param7D
	call	ClampInt
	call	Copy7CToSig
	bcf	GripIMet
	bcf	GripIOver
	goto	ModeReturn
;
DoMode4_1:
	bsf	ServoIdle	;power down servo
	bcf	GripIMet
	bcf	GripIOver
	goto	ModeReturn
;
;=========================================================================================
;=========================================================================================
;DebounceTime,kMaxMode
;Timer4Lo,SysMode
HandleButtons	movlb	0x00	;bank 0
	movf	Timer4Lo,F
	SKPNZ		;Debounced?
	bra	HdlBtn_1	; Yes
;
	btfsc	SW1_Flag
	bra	HdlBtn_DB
	btfsc	SW2_Flag
	bra	HdlBtn_DB
	btfsc	SW3_Flag
	bra	HdlBtn_DB
	btfss	SW4_Flag
	return
;
HdlBtn_DB	movlw	DebounceTime
	movwf	Timer4Lo
	return
; we are de-bounced
HdlBtn_1	btfsc	SW1_Flag
	bra	HdlBtn_Btn1
	btfsc	SW2_Flag
	bra	HdlBtn_Btn2
	btfsc	SW3_Flag
	bra	HdlBtn_Btn3
	btfsc	SW4_Flag
	bra	HdlBtn_Btn4
	return
;
; Mode
HdlBtn_Btn1:
	goto	HdlBtn_DB
;
HdlBtn_Btn2:
	goto	HdlBtn_DB
;
HdlBtn_Btn3:
	goto	HdlBtn_DB
; not used
HdlBtn_Btn4:
	goto	HdlBtn_DB
;
;=========================================================================================
; Setup or Read AN0 or Read AN4
ANNumMask	EQU	0x7C
AN0_Val	EQU	0x00                   ;Current
AN1_Val	EQU	0x04                   ;Volts
AN2_Val	EQU	0x08                   ;SW1/LED1/Aux0
AN3_Val	EQU	0x0C                   ;SW2/LED2/Aux1
;AN4_Val	EQU	0x10
;AN7_Val	EQU	0x1C
;
ReadAN	MOVLB	1	;bank 1
	BTFSS	ADCON0,ADON	;Is the Analog input ON?
	BRA	ReadAN0_ColdStart	; No, go start it
;
	BTFSC	ADCON0,GO_NOT_DONE	;Conversion done?
	BRA	ReadAN_Rtn	; No
;
	movlw	HIGH Cur_AN0
	movwf	FSR0H
	movf	ADCON0,W
	movlb	0x00	;bank 0
	andlw	ANNumMask
	SKPNZ
	bra	ReadAN_AN0
;
	movwf	Param78	;AN select bits
	movlw                  AN1_Val
	subwf                  Param78,W
	SKPNZ
	bra                    ReadAN_AN1             ;Batt Volts
;
;Aux0 SW1_LED1
	movf	ssAux0Config,W
	andlw	0x0F
	sublw	kAuxIOAnalogIn
	SKPZ
	bra	ReadAN_TryAN2
	movlw	AN2_Val
	subwf	Param78,W	;AN select bits
	SKPNZ
	bra	ReadAN_AN2
;Aux1 SW2_LED2
ReadAN_TryAN2	movf	ssAux1Config,W
	andlw	0x0F
	sublw	kAuxIOAnalogIn
	SKPZ
	bra	ReadAN_TryAN0
	movlw	AN2_Val
	subwf	Param78,W	;AN select bits
	SKPNZ
	bra	ReadAN_AN2
;IServo
ReadAN_TryAN0	movlw	AN0_Val
	subwf	Param78,W
	SKPNZ
	bra	ReadAN_AN0
;
	movlw	AN0_Val	;next to read
	movwf	Param78
	movlw	LOW Cur_AN0
	movwf	FSR0L
	BankSel	Cur_AN0	;where the analog stuff is
	bsf	NewDataAN0
	bra	ReadAN_1
;
ReadAN_AN0	movlw	low Cur_AN0
	movwf	FSR0L
	BankSel	Cur_AN0	;where the analog stuff is
	bsf	NewDataAN0
	movlw	AN1_Val	;next to read
	movwf	Param78
;	movf	ssAux0Config,W
;	andlw	0x0F
;	sublw	kAuxIOAnalogIn
;	SKPNZ
	bra	ReadAN_1
;
ReadAN_AN0_1	movlw	AN2_Val	;next to read
	movwf	Param78
	movf	ssAux0Config,W
	andlw	0x0F
	sublw	kAuxIOAnalogIn
	SKPNZ
	bra	ReadAN_1
;
ReadAN_AN0_2	movlw	AN3_Val	;next to read
	movwf	Param78
	movf	ssAux1Config,W
	andlw	0x0F
	sublw	kAuxIOAnalogIn
	SKPNZ
	bra	ReadAN_1
;
ReadAN_AN0_3	movlw	AN0_Val	;next to read
	movwf	Param78
	bra	ReadAN_1
;
ReadAN_AN1	movlw	low Cur_AN1
	movwf	FSR0L
	bra	ReadAN_AN0_1
;
ReadAN_AN2	movlw	low Cur_AN2
	movwf	FSR0L
	bra	ReadAN_AN0_2
;
ReadAN_AN3	movlw	low Cur_AN3
	movwf	FSR0L
	bra	ReadAN_AN0_3
;
ReadAN_1	movlb	0x01	;bank 1
	MOVF	ADRESL,W
	MOVWI	FSR0++
	MOVF	ADRESH,W
	MOVWI	FSR0++
;
	movf	Param78,W
	BSF	WREG,0	;ADC ON
	MOVWF	ADCON0
	movlw	0x04	;Acquisition time 5uS
	call	DelayWuS
	BSF	ADCON0,ADGO	;Start next conversion.
	movlb	0x00	; bank 0
	return
;
ReadAN0_ColdStart	MOVLB	1
	MOVLW	b'11100000'	;Right Just, fosc/64
;	MOVLW	b'11110000'	;Right Just, Frc
	MOVWF	ADCON1
	MOVLW	AN0_Val	;Select AN0
	BSF	WREG,0	;ADC ON
	MOVWF	ADCON0
	movlw	0x04	;Acquisition time 5uS
	call	DelayWuS
ReadAN_3	BSF	ADCON0,GO
ReadAN_Rtn:
Bank0_Rtn	MOVLB	0
	Return
;
;=========================================================================================
;
; Don't disable interrupts if you don't need to...
Copy7CToSig	MOVLB	0x05	;bank 5
	MOVF	Param7C,W
	SUBWF	SigOutTime,W
	SKPZ
	bra	Copy7CToSig_1
	MOVF	Param7D,W
	SUBWF	SigOutTimeH,W
	SKPNZ
	bra	Copy7CToSig_Done
;
Copy7CToSig_1	bcf	INTCON,GIE
	btfsc	INTCON,GIE
	bra	Copy7CToSig_1
	MOVF	Param7C,W
	MOVWF	SigOutTime
	MOVF	Param7D,W
	MOVWF	SigOutTimeH
	bsf	INTCON,GIE
Copy7CToSig_Done	movlb	0	;bank 0
	BCF	ServoIdle
	return
;
;=========================================================================================
;=========================================================================================
; Set CCP1 to go high is 0x100 clocks
;
StartServo	MOVLB	0	;bank 0
	BTFSS	ServoOff
	RETURN
	BCF	ServoOff
;
	CALL	SetMiddlePosition
	CALL	Copy7CToSig
;
	movlb	0x05	;bank 5
	MOVLW	LOW kServoDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTime
	MOVWF	CalcdDwellH
	movlb	0	;bank 0
;
	MOVLW	0x00	;start in 0x100 clocks
	MOVWF	TMR1L
	MOVLW	0xFF
	MOVWF	TMR1H
;
	MOVLB	0x05
	CLRF	CCPR1H
	CLRF	CCPR1L
	MOVLW	CCP1CON_Set
	MOVWF	CCP1CON	;go high on match
	MOVLB	0x00	;Bank 0
	RETURN
;
;=========
;
SetMiddlePosition	MOVLW	LOW kMidPulseWidth
	MOVWF	Param7C
	MOVLW	HIGH kMidPulseWidth
	MOVWF	Param7D
	Return
;
;=========================================================================================
StopServo	movlb	0	;bank 0
	BTFSC	ServoOff
	RETURN
;
	movlb	0x05	;bank 5
	MOVLW	LOW kServoDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTime
	MOVWF	CalcdDwellH
	movlb	0	;bank 0
	BSF	ServoIdle
	BSF	ServoOff
	return
;
;=========================================================================================
; ClampIntMD3(Param7D:Param7C,ServoFastReverse,ServoFastForward)
;
ClampIntMD3            mMOVLF                 ServoFastReverse,FSR0
                       bra                    ClampInt_E2
;
;---------------------
; ClampInt(Param7D:Param7C,ServoMin_uS,ServoMax_uS)
;
; Entry: Param7D:Param7C
; Exit: Param7D:Param7C=ClampInt(Param7D:Param7C,ServoMin_uS,ServoMax_uS)
; Ram Used: FSR0
;
ClampInt	mMOVLF                 ServoMin_uS,FSR0
ClampInt_E2            movlb	0
;W = Cmd - Max
                       moviw                  2[FRS0]
                       subwf                  Param7C,W
                       moviw                  3[FRS0]
                       subwfb                 Param7D,W
                       SKPB                                          ;Cmd > Max?
                       bra                    ClampInt_tooHigh       ; Yes, Fix it
                       bra                    ClampInt_1             ; No, check for < Min
;
; W=Cmd - Min
ClampInt_1             moviw                  0[FRS0]
                       subwf                  Param7C,W
                       moviw                  1[FSR0]
                       subwfb                 Param7D,W
                       SKPB                                          ;Cmd > Min?
                       return                                        ; Yes
                       bra                    ClampInt_tooLow        ; No, Fix it
;
ClampInt_tooLow	moviw                  0[FRS0]
	MOVWF	Param7C
	moviw                  1[FRS0]
	MOVWF	Param7D
	RETURN
;
ClampInt_tooHigh	moviw                  2[FRS0]
	MOVWF	Param7C
	moviw                  3[FRS0]
	MOVWF	Param7D
	RETURN
;
;=========================================================================================
;=========================================================================================
;
                       if UsePID
	include <DMFMath.inc>
	include <PIDInt.inc>
	endif
;
;
;
	org 0x800
	include <SerialServoCmds.inc>
	include <MagEncoder.inc>
	include <AS5047D_Lib.inc>
	include <ssInit.inc>
;
	org BootLoaderStart
	include <BootLoader1847.inc>
;
;
	END
;
