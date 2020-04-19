;====================================================================================================
;
;    Filename:      SerialSPI.asm
;    Created:       4/18/2020
;    File Version:  1.1d1   4/18/2020
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    SerialSPI is sample code and Testing tool for TMC4361A.
;    Features and configurations will be added as needed.
;
;    Features: 	TTL Packet Serial
;	
;
;Mode 0: TTL Packet Serial to SPI Master to TMC4361A
;
;    History:
; 1.1d1   4/18/2020	First code.
;
;====================================================================================================
; ToDo:
;
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;   Mode 0: TTL Packet Serial to SPI Master to TMC4361A
;
;====================================================================================================
;
;   Pin 1 (RA2/AN2)                           SW1/LED1 (Active Low Input/Output)
;   Pin 2 (RA3/AN3)                           SW2/LED2 (Active Low Input/Output)
;   Pin 3 (RA4/AN4)                           n/c
;   Pin 4 (RA5/MCLR*)                         VPP/MCLR*
;   Pin 5 (GND)                               Ground
;   Pin 6 (RB0)                               SPI CSBit (Active Low Output)
;   Pin 7 (RB1/AN11/SDA1)                     SPI MISO (Digital Input)
;   Pin 8 (RB2/AN10/TX)                       TTL Serial RX
;   Pin 9 (RB3/CCP1)                          Pulse output for Servo
;
;   Pin 10 (RB4/AN8/SLC1)                     SPI SCL1
;   Pin 11 (RB5/AN7)                          TTL Serial TX
;   Pin 12 (RB6/AN5/CCP2)                     ICSPCLK
;   Pin 13 (RB7/AN6)                          ICSPDAT
;   Pin 14 (Vcc)                              +5 volts
;   Pin 15 (RA6)                              SPI MOSI (Digital Output)
;   Pin 16 (RA7/CCP2)                         LED3 (Active Low Output)(System LED)
;   Pin 17 (RA0/AN0)                          Current sensing analog input
;   Pin 18 (RA1/AN1)                          Battery voltage sensing analog input
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
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x02	;This Slave's Address
kSysMode	EQU	.0	;Default Mode
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
ANSELA_Val	EQU	b'00000000'	;RA0/AN0, RA4/AN4
;
#Define	RA0_In	PORTA,0	;Current, Analog Input
#Define	RA1_In	PORTA,1	;Battery Volts, Analog Input
#Define	SW1_In	PORTA,2	;SW1/LED1
#Define	SW2_In	PORTA,3	;SW2/LED2
#Define	RA4_In	PORTA,4	;n/c
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
;
kSysFlags	EQU	.0
;
DebounceTime	EQU	.10
kMaxMode	EQU	.0
;
;=========================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;=========================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	SysLED_Time		;sys LED time
	SysLED_Blinks		;0=1 flash,1,2,3
	SysLED_BlinkCount
	SysLEDCount		;sys LED Timer tick count
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
	ssTempFlags
	ssStatus:4		;Status and condition flags for the user.
;
;-----------------------
;Below here are saved in eprom
;
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
	SysFlags		;saved in eprom 0x64 must
			; move something to another
			; bank before adding anything new
;
	endc
;--------------------------------------------------------------
;
;---SerFlags bits---
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
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
#Define	FirstRAMParam	SysMode
#Define	LastRAMParam	SysFlags
;
;=========================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes
;
	cblock	0x0A0
;
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
	endc
;
;
;=========================================================================================
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
;=========================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
;
                       cblock                 0x1A0
;
                       SPI_Flags
                       SPI_InBuff:5                                  ;Status,MSB..LSB
                       SPI_OutBuff:5                                 ;Address,MSB..LSB
;
                       endc
;
;=========================================================================================
;  Bank4 Ram 220h-26Fh 80 Bytes
;=========================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
;=========================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=========================================================================================
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
;=========================================================================================
; ID Locations
	__idlocs	0x10d1
;
;=========================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;
; default values
	ORG	0xF000
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr, 0x0F
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr, 0x10
	de	kSysFlags	;nvSysFlags
;
	ORG	0xF0FF
	de	0x00	;Skip BootLoader
;
	cblock	0x0000
;
;
	nvSysMode
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
	nvSysFlags
	endc
;
#Define	nvFirstParamByte	nvSysMode
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
;
;-------------
;
SystemTick_end:
;
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
;
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
;
	goto	MainLoop
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	org 0x800
	include <SerialSPICmds.inc>
	include <TMC4361A_Lib.inc>
	include <sSPIInit.inc>
;
	org BootLoaderStart
	include <BootLoader1847.inc>
;
;
	END
;
