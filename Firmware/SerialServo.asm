;====================================================================================================
;
;    Filename:      SerialServo.asm
;    Date:          7/10/2018
;    File Version:  1.0b4
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
;
;    History:
; 1.0b4   7/10/2018	Better defaults. Gripper mode (4).
; 1.0b3   6/19/2018	Added ssEnableFastPWM
; 1.0b2   6/3/2018	Servo current is averaged, DD DD Sync bytes and checksum.
; 1.0b1   6/1/2018	Modes 2 and 3 are working. No current limit yet.
; 1.0a3   5/31/2018    Added Speed, StopCenter.
; 1.0a2   5/25/2018	Added some more commands.
; 1.0a1   5/24/2018	It begins to work.
; 1.0d1   4/26/2018	First code.
;
;====================================================================================================
; Options
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;   Mode 0: (LED 1 = off) servo test mode, copy AN4 Pot value to servo.
;   Mode 1: (LED 1 = 1 flash) servo  and encoder test mode, AN4 Pot value - EncoderVal to servo dir.
;   Mode 2: Basic Serial Servo, output servo pulse of CmdPos * 0.5uS.
;   Mode 3: Absolute encoder position control.
;
;====================================================================================================
;
;   Pin 1 (RA2/AN2) SW2/LED2 (Active Low Input/Output)
;   Pin 2 (RA3/AN3) SW3/LED3 (Active Low Input/Output)
;   Pin 3 (RA4/AN4) Calibration Pot (Analog Input)
;   Pin 4 (RA5/MCLR*) VPP/MCLR*
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) MagEnc_CSBit (Active Low Output)
;   Pin 7 (RB1/AN11/SDA1) TTL Serial RX
;   Pin 8 (RB2/AN10/TX) TTL Serial TX
;   Pin 9 (RB3/CCP1) Pulse output for Servo
;
;   Pin 10 (RB4/AN8/SLC1)  MagEnc_CLKBit
;   Pin 11 (RB5/AN7) Battery voltage sensing analog input
;   Pin 12 (RB6/AN5/CCP2) ICSPCLK
;   Pin 13 (RB7/AN6) ICSPDAT
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) MagEnc_DataBit (Digital Input)
;   Pin 16 (RA7/CCP2) LED4 (Active Low Output)(System LED)
;   Pin 17 (RA0/AN0) Current sensing analog input
;   Pin 18 (RA1) SW1/LED1 (Active Low Input/Output)
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
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x02	;This Slave's Address
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
PortADDRBits	EQU	b'11111111'
PortAValue	EQU	b'00000000'
ANSELA_Val	EQU	b'00010001'	;RA0/AN0, RA4/AN4
;
#Define	RA0_In	PORTA,0	;Current, Analog Input
#Define	SW1_In	PORTA,1	;SW1/LED1
#Define	SW2_In	PORTA,2	;SW2/LED2
#Define	SW3_In	PORTA,3	;SW3/LED3
#Define	RA4_In	PORTA,4	;Pot, Analog Input
#Define	RA5_In	PORTA,5	;VPP/MCLR*
#Define	RA6_In	PORTA,6	;MagEnc_DataBit
#Define	SW4_In	PORTA,7	;LED4 System LED
LED1_Bit	EQU	1	;LED1 (Active Low Output)
LED2_Bit	EQU	2	;LED2 (Active Low Output)
LED3_Bit	EQU	3	;LED3 (Active Low Output)
SysLED_Bit	EQU	7	;LED4 (Active Low Output)
#Define	LED1_Tris	TRISA,LED1_Bit	;LED1 (Active Low Output)
#Define	LED2_Tris	TRISA,LED2_Bit	;LED2 (Active Low Output)
#Define	LED3_Tris	TRISA,LED3_Bit	;LED3 (Active Low Output)
#Define	SysLED_Tris	TRISA,SysLED_Bit	;LED4 (Active Low Output)
;
Servo_AddrDataMask	EQU	0xF8
;
;
;    Port B bits
PortBDDRBits	EQU	b'11100010'	;CCP1, MagEnc_CLKBit
PortBValue	EQU	b'00010101'
ANSELB_Val	EQU	b'00100000'	;RB5/AN7
;
#Define	RB0_In	PORTB,0	;MagEnc_CSBit (Active Low Output)
#Define	RB1_In	PORTB,1	;RX Serial Data
#Define	RB2_In	PORTB,2	;TX Serial Data
#Define	RB3_Out	PORTB,3	;CCP1 Output
#Define	RB4_In	PORTB,4	;MagEnc_CLKBit
#Define	RB5_In	PORTB,5	;Battery Volts, Analog Input
#Define	RB6_In	PORTB,6	;ICSPCLK
#Define	RB7_In	PORTB,7	;ICSPDAT
#Define	TX_TRIS	TRISB,2	;for High-Z control
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
kSysMode	EQU	.2	;Default Mode Basic Servo
kServoSpeed	EQU	.10	;Slow 5uS/Update
kssFlags	EQU	b'00000001'	;ssEnableOverCur=true
kssMaxI	EQU	.50	;Low
kMinPulseWidth	EQU	.1800	;900uS
kMidPulseWidth	EQU	.3000	;1500uS
kMaxPulseWidth	EQU	.4200	;2100uS
kServoFastForward	EQU	.3600	;1800uS
kServoFastReverse	EQU	.2400	;1200uS
kDeadBand	EQU	.100	;100 encoder counts
kSysFlags	EQU	.0
kGripI	EQU	.40
;
DebounceTime	EQU	.10
kMaxMode	EQU	.4
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
	SysLEDCount		;sys LED Timer tick count
;
	LED1_Blinks		;0=off,1,2,3
	LED2_Blinks
	LED3_Blinks
	LED1_BlinkCount		;LED1_Blinks..0
	LED2_BlinkCount
	LED3_BlinkCount
	LED1_Count		;tick count
	LED2_Count
	LED3_Count
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	ANFlags
	Cur_AN0:2		;IServo
	Cur_AN4:2		;Calibration Pot
	Cur_AN7:2		;Battery Volts
;
	OldAN0Value:2
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
	ssCurPos:2
	ssMD3_Dest:2
	ssTempFlags
	ssStatus:4		;Status and condition flags for the user.
;
	EncoderAccum:3		;Accumulated distance
	EncoderVal:2		;Value last read, raw 12 bit data
;-----------------------
;Below here are saved in eprom
	EncoderFlags
                       EncoderHome:2                                 ;Absolute Home
;
	ServoFastForward:2
	ServoFastReverse:2
	ServoStopCenter:2		;Mode 3 Idle position
	ServoMin_uS:2
	ServoMax_uS:2
	ServoSpeed		;0 = off, 1..63 position change per cycle
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
	ssFlags		;Serial Servo flags
	ssMaxI		;Max Current 0=off
	DeadBand		;Used by Mode 2
	ssGripI		;Gripper tension
	SysFlags		;saved in eprom
;
	endc
;--------------------------------------------------------------
;---ANFlags bits---
#Define	NewDataAN0	ANFlags,0
#Define	NewDataAN4	ANFlags,1
#Define	NewDataAN7	ANFlags,2
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
;
;----ssStatus bits
#Define	MD3_FFwd	ssStatus,0
#Define	MD3_FRev	ssStatus,1
#Define	ssio_OverCurSD	ssStatus,2
#Define	ssRX_Timeout	ssStatus,3
;
;---------------
#Define	FirstRAMParam	EncoderFlags
#Define	LastRAMParam	SysFlags
;
;---ssFlags bits---
#Define	ssEnableOverCur	ssFlags,0	;disable if current is too high
#Define	ssReverseDir	ssFlags,1	;if set ServoFastForward<=>ServoFastReverse
#Define	ssEnableHighZTZ	ssFlags,2	;if set TX is High-Z when not active
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
	endc
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
	de	low kServoFastForward
	de	high kServoFastForward
	de	low kServoFastReverse
	de	high kServoFastReverse
	de	low kMidPulseWidth	;nvServoStopCenter
	de	high kMidPulseWidth
	de	low kMinPulseWidth	;nvServoMin_uS
	de	high kMinPulseWidth
	de	low kMaxPulseWidth	;nvServoMax_uS
	de	high kMaxPulseWidth
	de	kServoSpeed	;nvServoSpeed
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr, 0x0F
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr, 0x10
	de	kssFlags	;nvssFlags
	de	kssMaxI	;nvssMaxI
	de	kDeadBand	;nvDeadBand
	de	kGripI
	de	kSysFlags	;nvSysFlags
;
	cblock	0x0000
;
	nvEncoderFlags
                       nvEncoderHome:2
;
	nvServoFastForward:2
	nvServoFastReverse:2
	nvServoStopCenter:2
	nvServoMin_uS:2
	nvServoMax_uS:2
	nvServoSpeed
	nvSysMode
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
	nvssFlags
	nvssMaxI
	nvDeadBand
	nvSysFlags
	nvssGripI
	endc
;
#Define	nvFirstParamByte	nvEncoderFlags
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
;
	ORG	0x000	; processor reset vector
	CLRF	STATUS
	CLRF	PCLATH
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
	BSF	LED1_Tris
	BSF	LED2_Tris
	BSF	LED3_Tris
;
; Read Switches
	movlb	0x00	;bank 0
	BCF	SW1_Flag
	BCF	SW2_Flag
	BCF	SW3_Flag
	BCF	SW4_Flag
;
	BTFSS	SW1_In
	BSF	SW1_Flag
	BTFSS	SW2_In
	BSF	SW2_Flag
	BTFSS	SW3_In
	BSF	SW3_Flag
;	BTFSS	SW4_In	;not used in rev n/c
;	BSF	SW4_Flag
;
; Sys LED time
	DECFSZ	SysLEDCount,F	;Is it time?
	bra	SystemBlink_end	; No, not yet
;
	MOVF	SysLED_Time,W
	MOVWF	SysLEDCount
	movlb	0x01	;bank 1
	bcf	SysLED_Tris
SystemBlink_end:
; Flash LEDs
	movlb	0x00	;bank 0
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
	movlb	0x00	;bank 0
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
	movlb	0x00	;bank 0
	movf	LED3_Blinks,F
	SKPZ		;LED3 active?
	bra	LED3_Blinking	; Yes
	clrf	LED3_BlinkCount
	clrf	LED3_Count
	bra	LED3_Blink_end
;
LED3_Blinking	movf	LED3_Count,W
	iorwf	LED3_BlinkCount,W
	SKPNZ		;Startup?
	bra	LED3_Start
;
	decfsz	LED3_Count,F	;Done w/ blink
	bra	LED3_Blink_end	; no
;
	movf	LED3_BlinkCount,F
	SKPNZ		;Done w/ cycle?
	bra	LED3_Start	; Yes
;
	decfsz	LED3_BlinkCount,F
	bra	LED3_NextBlink
	movlw	LEDTIME	;long off time
	movwf	LED3_Count
	bra	LED3_Blink_end
;
LED3_Start	movf	LED3_Blinks,W
	movwf	LED3_BlinkCount
LED3_NextBlink	movlw	LEDFastTime
	movwf	LED3_Count
;
	movlb	0x01
	BCF	LED3_Tris
	movlb	0x00
LED3_Blink_end:
;-------------
;
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
	include <MagEncoder.inc>
	include <SerBuff1938.inc>
	include <RS232_Parse.inc>
;
;=========================================================================================
;
start	call	InitializeIO
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
	mCall0To1	HandleRXData
ML_1:
;
	CALL	ReadAN
;
; Average AN0
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
	call	ReadEncoder
;
	call	HandleButtons
;
;---------------------
; Handle High-Z Serial control
; if TXIF and GetSerOutBytes=0 and ssEnableHighZTZ
;  set Tris
; else
;  clr Tris
	btfss	ssEnableHighZTZ	;High-Z Enabled?
	bra	ML_TXActive	; No
	btfsc	PIR1,TXIF	;TX in progress?
	bra	ML_TXActive	; Yes
	call	GetSerOutBytes
	SKPZ		;Any bytes to TX?
	bra	ML_TXActive	; Yes
	movlb	0x01	; bank 1
	bsf	TX_TRIS	;go High-Z
	bra	ML_HighZTX_End
ML_TXActive	movlb	0x01	; bank1
	bcf	TX_TRIS	;Output Active
ML_HighZTX_End	movlb	0x00	; bank 0
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
DoModeZero	lslf	Cur_AN4,W
	movwf	Param7C
	rlf	Cur_AN4+1,W
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
DoModeOne	movlb	0	;bank 0
;
;Param7A:Param79 = Cur_AN4 + .950
	movlw	low .950
	addwf	Cur_AN4,W
	movwf	Param79
	movlw	high .950
	addwfc	Cur_AN4+1,W
	movwf	Param7A
;
;Param7A:Param79 = Param7A:Param79 - EncoderVal
	movf	EncoderVal,W
	subwf	Param79,F
	movf	EncoderVal+1,W
	subwfb	Param7A,F
;
	btfss	Param7A,7	;Param7A:Param79 < 0?
	bra	DM1_FF	; No, EncoderVal <= (AN4 + .950)
;
;Param7A:Param79 = Cur_AN4 + .1050
	movlw	low .1050
	addwf	Cur_AN4,W
	movwf	Param79
	movlw	high .1050
	addwfc	Cur_AN4+1,W
	movwf	Param7A
;
;Param7A:Param79 = Param7A:Param79 - EncoderVal
	movf	EncoderVal,W
	subwf	Param79,F
	movf	EncoderVal+1,W
	subwfb	Param7A,F
;
	btfsc	Param7A,7	;Param7A:Param79 < 0?
	bra	DM1_FR	; Yes, EncoderVal > (AN4 + .1050)
;
; EncoderVal > (AN4 + .950) && EncoderVal <= (AN4 + .1050)
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
;
CheckCurrent	movlb	0x00	;Bank 0
	bcf	OverCurrentFlag
	btfss	ssEnableOverCur
	return
;Param79:Param78 = ssMaxI * 4
	clrf	Param79
	lslf	ssMaxI,W
	movwf	Param78
	rlf	Param79,F
	lslf	Param78,F
	rlf	Param79
;Param79:Param78 -= Cur_AN0
	movf	Cur_AN0,W
	subwf	Param78,F
	movf	Cur_AN0+1,W
	subwfb	Param79,F
;
	btfsc	Param79,7	;Cur_AN0>ssMaxI*4?
	bsf	OverCurrentFlag
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
;Idle routine for Absolute encoder position control.
; if ssCmdPos > EncoderVal set servo to ServoFastForward
; elseif ssCmdPos + DeadBand < EncoderVal set servo to ServoFastReverse
; else Set ServoIdle
;
DoModeThree	movlb	0	;bank 0
	btfsc	ssCmdPos+1,7
	bra	DM3_IdleServo
;
;Check for over current, kill position command if over current is detected.
	call	CheckCurrent
	btfss	OverCurrentFlag
	bra	DM3_NotOverCurrent
	clrf	ssCmdPos
	clrf	ssCmdPos+1
	bsf	ssCmdPos+1,7
	bsf	ssio_OverCurSD
	bra	DM3_IdleServo
;
DM3_NotOverCurrent:
;Param7A:Param79 = ssCmdPos
	bcf	ssio_OverCurSD
	movf	ssCmdPos,W
	movwf	Param79
	movf	ssCmdPos+1,W
	movwf	Param7A
;
;Param7A:Param79 = Param7A:Param79 - EncoderVal
	movf	EncoderVal,W
	subwf	Param79,F
	movf	EncoderVal+1,W
	subwfb	Param7A,F
;
	btfss	Param7A,7	;Param7A:Param79 < 0?
	bra	DM3_FF	; No, EncoderVal <= ssCmdPos
;
;Param7A:Param79 = ssCmdPos + DeadBand
	movf	DeadBand,W
	addwf	ssCmdPos,W
	movwf	Param79
	movlw	0x00
	addwfc	ssCmdPos+1,W
	movwf	Param7A
;
;Param7A:Param79 = Param7A:Param79 - EncoderVal
	movf	EncoderVal,W
	subwf	Param79,F
	movf	EncoderVal+1,W
	subwfb	Param7A,F
;
	btfsc	Param7A,7	;Param7A:Param79 < 0?
	bra	DM3_FR	; Yes, EncoderVal > (ssCmdPos + DeadBand)
;
; EncoderVal > ssCmdPos && EncoderVal <= (ssCmdPos + DeadBand)
DM3_IdleServo	bcf	MD3_FFwd
	bcf	MD3_FRev
	btfss	ssMode3IdleCenter
	bra	DM3_IdleInactive
	movf	ServoStopCenter,W
	movwf	ssMD3_Dest
	movwf	ssCurPos
	movf	ServoStopCenter+1,W
	movwf	ssMD3_Dest+1
	movwf	ssCurPos+1
	bra	DM3_UpdatePos
;
DM3_IdleInactive	bsf	ServoIdle
	goto	ModeReturn
;
DM3_FF	btfsc	ssReverseDir
	bra	DM3_FR_1
DM3_FF_1	btfsc	MD3_FRev	;Moving Reverse dir?
	bra	DM3_IdleServo	; Yes
	bsf	MD3_FFwd
	movf	ServoFastForward,W
	movwf	ssMD3_Dest
	movf	ServoFastForward+1,W
	movwf	ssMD3_Dest+1
	bra	DM3_UpdatePos
;
DM3_FR	btfsc	ssReverseDir
	bra	DM3_FF_1
DM3_FR_1	btfsc	MD3_FFwd	;Moving Forward dir?
	bra	DM3_IdleServo	; Yes
	bsf	MD3_FRev
	movf	ServoFastReverse,W
	movwf	ssMD3_Dest
	movf	ServoFastReverse+1,W
	movwf	ssMD3_Dest+1
;
DM3_UpdatePos	movf	ServoSpeed,F
	SKPNZ		;Speed = 0?
	bra	DM3_NoSpeed	; yes
	btfss	PulseSent	;Time to update?
	goto	ModeReturn	; No
	bcf	PulseSent
;Param7D:Param7C = Dest-Cur
	movf	ssCurPos,W
	subwf	ssMD3_Dest,W
	movwf	Param7C
	movf	ssCurPos+1,W
	subwfb	ssMD3_Dest+1,W
	movwf	Param7D
; if Param7D:Param7C = 0 then we are In Position
	iorwf	Param7C,W
	SKPNZ
	bra	DM3_Go	; if Cmd = Cur Go
;
;
	BTFSS	Param7D,7	;Cmd<Cur? Set if Cur>Cmd
	GOTO	DM3_MovPlus	; Yes
;Move minus
	INCFSZ	Param7D,W	;Dist=0xFFxx?
	GOTO	DM3_Minus	; No
	MOVF	ServoSpeed,W
	ADDWF	Param7C,W
	BTFSC	_C	;Dist<Speed?
	bra	DM3_NoSpeed	; No
;
; Subtract speed from current position
DM3_Minus	MOVF	ServoSpeed,W
	SUBWF	ssCurPos,F	;SigOutTime
	MOVLW	0x00
	SUBWFB	ssCurPos+1,F	;SigOutTimeH
	bra	DM3_Go
;
;=============================
; 7D:7C = distance to go
;
DM3_MovPlus	MOVF	Param7D,F
	SKPZ		;Dist>255 to go?
	bra	DM3_Plus	; Yes
	MOVF	ServoSpeed,W
	SUBWF	Param7C,W	;Dist-Speed
	SKPNB		;Speed>Dist?
	bra	DM3_NoSpeed	; Yes
;
DM3_Plus	MOVF	ServoSpeed,W	;7D:7C = CurPos + Speed
	ADDWF	ssCurPos,F
	CLRW
	ADDWFC	ssCurPos+1,F
	bra	DM3_Go
;
;
; set current position at destination position
DM3_NoSpeed	movf	ssMD3_Dest,W
	movwf	ssCurPos
	movf	ssMD3_Dest+1,W
	movwf	ssCurPos+1
;
DM3_Go	movf	ssCurPos,W
	movwf	Param7C
	movf	ssCurPos+1,W
	movwf	Param7D
	call	ClampInt
	call	Copy7CToSig
	goto	ModeReturn
;
;=========================================================================================
;Idle routine for Gripper Serial Servo mode
;
DoMode4	movlb	0
	btfsc	ssCmdPos+1,7	;Any command issued?
	bra	DoMode4_1	; No, Idle the servo
;
;Check for over current, kill position command if over current is detected.
	call	CheckCurrent
	btfss	OverCurrentFlag
	bra	DM2_NotOverCurrent
	clrf	ssCmdPos
	clrf	ssCmdPos+1
	bsf	ssCmdPos+1,7
	bsf	ssio_OverCurSD
	bra	DoMode4_1
;
DM4_NotOverCurrent:
	bcf	ssio_OverCurSD
	movf	ServoSpeed,F
	SKPNZ		;Speed = 0?
	bra	DoMode4_NoSpeed	; yes
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
	bra	DoMode4_Go	; if Cmd = Cur Go
;
;
	BTFSS	Param7D,7	;Cmd<Cur? Set if Cur>Cmd
	GOTO	DoMode4_MovPlus	; Yes
;Move minus
	INCFSZ	Param7D,W	;Dist=0xFFxx?
	GOTO	DoMode4_Minus	; No
	MOVF	ServoSpeed,W
	ADDWF	Param7C,W
	BTFSC	_C	;Dist<Speed?
	bra	DoMode4_NoSpeed	; No
;
; Subtract speed from current position
DoMode4_Minus	MOVF	ServoSpeed,W
	SUBWF	ssCurPos,F	;SigOutTime
	MOVLW	0x00
	SUBWFB	ssCurPos+1,F	;SigOutTimeH
	bra	DoMode4_Go
;
;=============================
; 7D:7C = distance to go
;
DoMode4_MovPlus	MOVF	Param7D,F
	SKPZ		;Dist>255 to go?
	bra	DoMode4_Plus	; Yes
	MOVF	ServoSpeed,W
	SUBWF	Param7C,W	;Dist-Speed
	SKPNB		;Speed>Dist?
	bra	DoMode4_NoSpeed	; Yes
;
DoMode4_Plus	MOVF	ServoSpeed,W	;7D:7C = CurPos + Speed
	ADDWF	ssCurPos,F
	CLRW
	ADDWFC	ssCurPos+1,F
	bra	DoMode4_Go
;
;
; set current position at command position
DoMode4_NoSpeed	movf	ssCmdPos,W
	movwf	ssCurPos
	movf	ssCmdPos+1,W
	movwf	ssCurPos+1
; make it so
DoMode4_Go	movf	ssCurPos,W
	movwf	Param7C
	movf	ssCurPos+1,W
	movwf	Param7D
	call	ClampInt
	call	Copy7CToSig
	goto	ModeReturn
;
DoMode4_1:
	bsf	ServoIdle	;power down servo
	goto	ModeReturn
;
;=========================================================================================	
;=========================================================================================
;DebounceTime,kMaxMode
;Timer4Lo,SysMode
HandleButtons	movlb	0x00	;bank 0
	movf	Timer4Lo,f
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
	movf	SysMode,W
	incf	SysMode,f
	sublw	kMaxMode
	SKPNZ		;SysMode==kMaxMode?
	clrf	SysMode	; Yes, set mode=0x00
;
	movf	SysMode,W
	movwf	LED1_Blinks
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
AN0_Val	EQU	0x00
AN4_Val	EQU	0x10
AN7_Val	EQU	0x1C
;
ReadAN	MOVLB	1	;bank 1
	BTFSS	ADCON0,ADON	;Is the Analog input ON?
	BRA	ReadAN0_ColdStart	; No, go start it
;
	BTFSC	ADCON0,GO_NOT_DONE	;Conversion done?
	BRA	ReadAN_Rtn	; No
;
	clrf	FSR0H
	movf	ADCON0,W
	movlb	0x00	;bank 0
	andlw	ANNumMask
	SKPNZ
	bra	ReadAN_AN0
;
	movwf	Param78
	movlw	AN4_Val
	subwf	Param78,W
	SKPNZ
	bra	ReadAN_AN4
;
	movlw	AN0_Val	;next to read
	movwf	Param78
	movlw	LOW Cur_AN7
	movwf	FSR0L
	bsf	NewDataAN7
	bra	ReadAN_1
;
ReadAN_AN4	movlw	AN7_Val	;next to read
	movwf	Param78
	movlw	low Cur_AN4
	movwf	FSR0L
	bsf	NewDataAN4
	bra	ReadAN_1
;
ReadAN_AN0	movlw	AN4_Val	;next to read
	movwf	Param78
	movlw	low Cur_AN0
	movwf	FSR0L
	bsf	NewDataAN0
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
; ClampInt(Param7D:Param7C,ServoMin_uS,kMaxPulseWidth)
;
; Entry: Param7D:Param7C
; Exit: Param7D:Param7C=ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
ClampInt	movlb	0
	MOVF	ServoMax_uS+1,W
	SUBWF	Param7D,W	;7D-kMaxPulseWidth
	SKPNB		;7D<Max?
	GOTO	ClampInt_1	; Yes
	SKPZ		;7D=Max?
	GOTO	ClampInt_tooHigh	; No, its greater.
	MOVF	ServoMax_uS,W	; Yes, MSB was equal check LSB
	SUBWF	Param7C,W	;7C-kMaxPulseWidth
	SKPNZ		;=kMaxPulseWidth
	RETURN		;Yes
	SKPB		;7C<Max?
	GOTO	ClampInt_tooHigh	; No
	RETURN		; Yes
;
ClampInt_1	MOVF	ServoMin_uS+1,W
	SUBWF	Param7D,W	;7D-kMinPulseWidth
	SKPNB		;7D<Min?
	GOTO	ClampInt_tooLow	; Yes
	SKPZ		;=Min?
	RETURN		; No, 7D>kMinPulseWidth
	MOVF	ServoMin_uS,F	; Yes, MSB is a match
	SUBWF	Param7C,W	;7C-kMinPulseWidth
	SKPB		;7C>=Min?
	RETURN		; Yes
;
ClampInt_tooLow	MOVF	ServoMin_uS,W
	MOVWF	Param7C
	MOVF	ServoMin_uS+1,W
	MOVWF	Param7D
	RETURN
;
ClampInt_tooHigh	MOVF	ServoMax_uS,W
	MOVWF	Param7C
	MOVF	ServoMax_uS+1,W
	MOVWF	Param7D
	RETURN
;
;=========================================================================================
; call once
;=========================================================================================
;
InitializeIO	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
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
	movlb	0	; bank 0
	MOVLW	T2CON_Value
	MOVWF	T2CON
	MOVLW	PR2_Value
	MOVWF	PR2
	movlb	1	; bank 1
	bsf	PIE1,TMR2IE	; enable Timer 2 interupt
;
; setup timer 1 for 0.5uS/count
;
	MOVLB	0x00	; bank 0
	MOVLW	T1CON_Val
	MOVWF	T1CON
	bcf	T1GCON,TMR1GE	;always count
;
; clear memory to zero
	CALL	ClearRam
	CLRWDT
	CALL	CopyToRam
;
; setup ccp1
;
	BSF	ServoOff
;	BANKSEL	APFCON
;	BSF	APFCON,CCP1SEL	;CCP1 on RA5
	BANKSEL	CCP1CON
	CLRF	CCP1CON
;
	MOVLB	0x01	;Bank 1
	bsf	PIE1,CCP1IE
;
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
	if useRS232
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
	BSF	PIE1,RCIE	; Serial Receive interupt
	movlb	0x00	; bank 0
;
	endif
;
	CLRWDT
;-----------------------
;
	MOVLB	0x00
	MOVLW	LEDTIME
	MOVWF	SysLED_Time
	movlw	0x01
	movwf	SysLEDCount	;start blinking right away
	movlw	.100
	movwf	Timer4Lo	;ignor buttons for 1st second
;
	movf	SysMode,W
	movwf	LED1_Blinks
;
;if mode 3 don't move
	bsf	ssCmdPos+1,7
;
	CLRWDT
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
	return
;
;=========================================================================================
;=========================================================================================
;
;
	org 0x800
	include <SerialServoCmds.inc>
;
;
;
	END
;
