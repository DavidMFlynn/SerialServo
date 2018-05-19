;====================================================================================================
;
;    Filename:      SerialServo.asm
;    Date:          4/26/2018
;    File Version:  1.0d1
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
;    History:
;
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
;   Mode 2: Basic Serial Servo, Do what the master says.
;   Mode 3: Absolute encoder position control.
;
;====================================================================================================
;
;   Pin 1 (RA2/AN2) SW2/LED2 (Active Low Input/Output)
;   Pin 2 (RA3/AN3) SW3/LED3 (Active Low Input/Output)
;   Pin 3 (RA4/AN4) Calibration Pot (Analog Input)
;   Pin 4 (RA5/MCLR*) VPP/MCLR*
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) MagEnc_CSBit
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
	constant	RP_DataBytes=4
RS232_RAddr	EQU	0x01	;Master's Address
RS232_MyAddr	EQU	0x02	;This Slave's Address
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
PortBDDRBits	EQU	b'11100111'	;CCP1, MagEnc_CLKBit
PortBValue	EQU	b'00010001'
ANSELB_Val	EQU	b'00100000'	;RB5/AN7
;
#Define	RB0_In	PORTB,0	;MagEnc_DataBit
#Define	RB1_In	PORTB,1	;RX Serial Data
#Define	RB2_In	PORTB,2	;TX Serial Data
#Define	RB3_Out	PORTB,3	;CCP1 Output
#Define	RB4_In	PORTB,4	;MagEnc_CLKBit
#Define	RB5_In	PORTB,5	;Battery Volts, Analog Input
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
kMinPulseWidth	EQU	.1800	;900uS
kMidPulseWidth	EQU	.3000	;1500uS
kMaxPulseWidth	EQU	.4200	;2100uS
kServoFastForward	EQU	.3600	;1800uS
kServoFastReverse	EQU	.2400	;1200uS
;
DebounceTime	EQU	.10
kMaxMode	EQU	.3
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
	Cur_AN0:2		;IServo
	Cur_AN4:2		;Calibration Pot
	Cur_AN7:2		;Battery Volts
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
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_TempData:RP_DataBytes
	RX_Data:RP_DataBytes
	TX_Data:RP_DataBytes
;
	ssCmdPos:2		;Commanded position, 0=not used
;
	EncoderAccum:3		;Accumulated distance
	EncoderVal:2		;Value last read, raw 12 bit data
;Below here are saved in eprom
	EncoderFlags
                       EncoderHome:2                                 ;Absolute Home
;
	ServoFastForward:2
	ServoFastReverse:2
	ServoMin_uS:2
	ServoMax_uS:2
	SysMode
	ssFlags		;Serial Servo flags
	ssMaxI		;Max Current 0=off
	SysFlags		;saved in eprom
;
	endc
;-----------------------
;RX_ParseFlags Bits
#Define	SyncByte1RXd	RX_ParseFlags,0
#Define	SyncByte2RXd	RX_ParseFlags,1
#Define	SourceAddLoRXd	RX_ParseFlags,2
#Define	SourceAddHiRXd	RX_ParseFlags,3
#Define	DestAddLoRXd	RX_ParseFlags,4
#Define	DestAddHiRXd	RX_ParseFlags,5
#Define	AllDataRXd	RX_ParseFlags,6
;
;RX_Flags Bits
#Define	RXDataValidFlag	RX_Flags,0
#Define	RXDataIsNew	RX_Flags,1
;
	if useRS232
	cblock
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
	endc
;
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
;
	endif
;
;
#Define	FirstRAMParam	EncoderFlags
#Define	LastRAMParam	SysFlags
;
#Define	ssEnableOverCur	ssFlags,0	;disable if current is too high
#Define	ssReverseDir	ssFlags,1	;if set ServoFastForward<=>ServoFastReverse
;
#Define	SW1_Flag	SysFlags,0
#Define	SW2_Flag	SysFlags,1
#Define	SW3_Flag	SysFlags,2
#Define	SW4_Flag	SysFlags,3
;
#Define	ServoOff	SysFlags,4
#Define	ServoIdle	SysFlags,5
;
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
;	ORG	0x2000
;	DE	'1','.','0','0'
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
	de	low kMinPulseWidth	;nvServoMin_uS
	de	high kMinPulseWidth
	de	low kMaxPulseWidth	;nvServoMax_uS
	de	high kMaxPulseWidth
	de	0x00	;nvSysMode
	de	0x00	;nvssFlags
	de	0x00	;nvssMaxI
	de	0x00	;nvSysFlags
;
	cblock	0x0000
;
	nvEncoderFlags
                       nvEncoderHome:2
;
	nvServoFastForward:2
	nvServoFastReverse:2
	nvServoMin_uS:2
	nvServoMax_uS:2
	nvSysMode
	nvssFlags
	nvssMaxI
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
	BTFSS	ServoOff	;Are we sending a pulse?
	bra	IRQ_Servo1_1	; Yes
;
;Servo is off, idle CCP1 and keep output low
	MOVLB	0x05
	movlw	CCP1CON_Idle
	movwf	CCP1CON
	bra	IRQ_Servo1_Dwell
;
IRQ_Servo1_1	btfsc	ServoIdle
	bra	IRQ_Servo1_Idle
	MOVLB	0x05
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
	MOVLW	LOW kServoDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTime
	MOVWF	CalcdDwellH
	MOVF	SigOutTime,W
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
;=========================================================================================
;
;=========================================================================================
;
	retfie		; return from interrupt
;
;
;==============================================================================================
;==============================================================================================
;
	include	<F1847_Common.inc>
;
;==============================================================================================
;
start	call	InitializeIO
;
	CALL	StartServo
	CALL	ReadAN0_ColdStart
;
;=========================================================================================
;=========================================================================================
;  Main Loop
;
;=========================================================================================
MainLoop	CLRWDT
;
;	goto	MainLoop	;tc
;
;	CALL	RS232_Parse
;	btfsc	RXDataIsNew
;	call	HandleRXData
;
	CALL	ReadAN
;
;	call	ReadEncoder
;
	call	HandleButtons
;
	if oldCode
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
	endif
;
	movlb	0x00	;bank 0
	movf	SysMode,W
	brw
	goto	DoModeZero
	goto	DoModeOne
	goto	DoModeTwo
	goto	DoModeThree
;
ModeReturn:
;
	goto	MainLoop
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
kCmd_SetMode	EQU	0x81	;+1 data (SysMode)
kCmd_GetMode	EQU	0x01
kCmd_SetCmdPos	EQU	0x82	;+2 data (ssCmdPos)
kCmd_GetCmdPos	EQU	0x02
kCmd_SetMaxI	EQU	0x83	;+1 data (ssMaxI)
kCmd_GetMaxI	EQU	0x03
kCmd_SetFFwd	EQU	0x84	;+2 data (ServoFastForward)
kCmd_GetFFwd	EQU	0x04
kCmd_SetFRev	EQU	0x85	;+2 data (ServoFastReverse)
kCmd_GetFRev	EQU	0x05
kCmd_SetMin_uS	EQU	0x86	;+2 data (ServoMin_uS)
kCmd_GetMin_uS	EQU	0x06
kCmd_SetMax_uS	EQU	0x87	;+2 data (ServoMax_uS)
kCmd_GetMax_uS	EQU	0x07
;
kCmd_GetI	EQU	0x91	;return Cur_AN0
kCmd_GetEnc	EQU	0x92	;return EncoderVal
kCmd_GetEncAbs	EQU	0x93	;return EncoderAccum
;
HandleRXData	bcf	RXDataIsNew
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
	goto	TX_ACK
;
Cmd_SetMode_end:
;
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
;----------------------
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
;
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
;----------------------
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
;
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
;----------------------
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
;
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
;----------------------
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
;
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
;----------------------
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
;
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
;----------------------
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
;
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
;----------------------
	movlw	kCmd_GetI
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetI_end
; Get servo current
	movf	EncoderVal,W
	movwf	TX_Data
	movf	EncoderVal+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetI_end:
;----------------------
	movlw	kCmd_GetEnc
	subwf	RX_Data,W
	SKPZ
	bra	Cmd_GetEnc_end
; Get Encoder Raw Position
	movf	Cur_AN0,W
	movwf	TX_Data
	movf	Cur_AN0+1,W
	movwf	TX_Data+1
	goto	RS232_Send
;
Cmd_GetEnc_end:
;----------------------
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
;
;
	return
;
TX_ACK	movlw	0xFF
	goto	StoreSerOut
;
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
;Idle routine for Basic Serial Servo mode
DoModeTwo	movlb	0
	movf	ssCmdPos,W
	iorwf	ssCmdPos+1,W
	SKPNZ
	bra	DoModeTwo_1
; set command position
	movf	ssCmdPos,W
	movwf	Param7C
	movf	ssCmdPos+1,W
	movwf	Param7D
	call	ClampInt
	call	Copy7CToSig
	goto	ModeReturn
;
DoModeTwo_1:
	bsf	ServoIdle	;power down servo
	goto	ModeReturn
;=========================================================================================
DeadBand	EQU	.100
;Idle routine for Absolute encoder position control.
; if ssCmdPos > EncoderVal set servo to ServoFastForward
; elseif ssCmdPos + DeadBand < EncoderVal set servo to ServoFastReverse
; else Set ServoIdle
;
DoModeThree	movlb	0	;bank 0
;
;Param7A:Param79 = ssCmdPos
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
	movlw	low DeadBand
	addwf	ssCmdPos,W
	movwf	Param79
	movlw	high DeadBand
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
	bsf	ServoIdle
	goto	ModeReturn
;
DM3_FF	btfsc	ssReverseDir
	bra	DM3_FR_1
DM3_FF_1	movf	ServoFastForward,W
	movwf	Param7C
	movf	ServoFastForward+1,W
	movwf	Param7D
	call	Copy7CToSig
	goto	ModeReturn
;
DM3_FR	btfsc	ssReverseDir
	bra	DM3_FF_1
DM3_FR_1	movf	ServoFastReverse,W
	movwf	Param7C
	movf	ServoFastReverse+1,W
	movwf	Param7D
	call	Copy7CToSig
	goto	ModeReturn
;
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
	bra	ReadAN_1
;
ReadAN_AN4	movlw	AN7_Val	;next to read
	movwf	Param78
	movlw	low Cur_AN4
	movwf	FSR0L
	bra	ReadAN_1
;
ReadAN_AN0	movlw	AN4_Val	;next to read
	movwf	Param78
	movlw	low Cur_AN0
	movwf	FSR0L
;
ReadAN_1	MOVF	ADRESL,W
	MOVWI	FSR0++
	MOVF	ADRESH,W
	MOVWI	FSR0++
;
	movf	Param78,W
	BSF	WREG,0	;ADC ON
	MOVWF	ADCON0
	BSF	ADCON0,ADGO	;Start next conversion.
	BRA	ReadAN_Rtn
;
ReadAN0_ColdStart	MOVLB	1
	MOVLW	0xA0	;Right Just, fosc/32
	MOVWF	ADCON1
	MOVLW	AN0_Val	;Select AN0
	BSF	WREG,0	;ADC ON
	MOVWF	ADCON0
	BSF	ADCON0,GO
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
; ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
; Entry: Param7D:Param7C
; Exit: Param7D:Param7C=ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
ClampInt	MOVLW	high kMaxPulseWidth
	SUBWF	Param7D,W	;7D-kMaxPulseWidth
	SKPNB		;7D<Max?
	GOTO	ClampInt_1	; Yes
	SKPZ		;7D=Max?
	GOTO	ClampInt_tooHigh	; No, its greater.
	MOVLW	low kMaxPulseWidth	; Yes, MSB was equal check LSB
	SUBWF	Param7C,W	;7C-kMaxPulseWidth
	SKPNZ		;=kMaxPulseWidth
	RETURN		;Yes
	SKPB		;7C<Max?
	GOTO	ClampInt_tooHigh	; No
	RETURN		; Yes
;
ClampInt_1	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,W	;7D-kMinPulseWidth
	SKPNB		;7D<Min?
	GOTO	ClampInt_tooLow	; Yes
	SKPZ		;=Min?
	RETURN		; No, 7D>kMinPulseWidth
	MOVLW	low kMinPulseWidth	; Yes, MSB is a match
	SUBWF	Param7C,W	;7C-kMinPulseWidth
	SKPB		;7C>=Min?
	RETURN		; Yes
;
ClampInt_tooLow	MOVLW	low kMinPulseWidth
	MOVWF	Param7C
	MOVLW	high kMinPulseWidth
	MOVWF	Param7D
	RETURN
;
ClampInt_tooHigh	MOVLW	low kMaxPulseWidth
	MOVWF	Param7C
	MOVLW	high kMaxPulseWidth
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
	MOVLW	TXSTA_Value
	MOVWF	TXSTA
	MOVLW	low BaudRate
	MOVWF	SPBRGL
	MOVLW	high BaudRate
	MOVWF	SPBRGH
	MOVLW	RCSTA_Value
	MOVWF	RCSTA
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
	CLRWDT
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
	return
;
;=========================================================================================
	include <MagEncoder.inc>
	include <SerBuff1938.inc>
	include <RS232_Parse.inc>
;=========================================================================================
;=========================================================================================
;
;
;
;
;
	END
;
