;=========================================================================================
; Commonly used routines PIC16F1847 version
;
;    Filename:      F1847 Common.inc
;    Date:          4/6/2019
;    File Version:  1.0.2
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
;    History:
;
; 1.0.2  4/6/2019	Fixes to EERead/EEwrite
; 1.0.2 2/16/2019	Fix: added FSR0H to DecTimer.
; 1.0.1 11/21/2015	Updates DecTimer.
; 1.0   11/16/2013	Updated from F648A Common.inc
;
;=========================================================================================
; Routines:
;
; ClearRam	(2+0) Clears all RAM, call once before initializing variables, FSR0
; CopyToRam	(1+0) copy param memory (EEPROM) to ram, call once, FSR0
; SaveParams	(1+0) copy ram to param memory (EEPROM), FSR0
;
; DecTimer4	(0+0) Decrement routine for 16 bit timers, FSR0
; DecTimer3
; DecTimer2
; DecTimer1
; DecTimer	(0+0) High byte of counter address in W
;
; TestT4_Zero	Test for 16 bit timers = zero
; TestT3_Zero	If Timer is zero return Z flag,1 else Z=0
; TestT2_Zero
; TestT1_Zero
;
; Delay10uS	(0+0)Delay uS    1 cycle = 1uS, 8Mhz clock version
; Delay100uS
; Delay40uS
; DelayWuS
;
; EEReadW	(0+0) Read EEPROM address in W
; EERead	(0+0) Read EEPROM address in EEAddrTemp
; EEWriteW	(0+0) Write EEPROM address in W, Data in EEDataTemp
; EEWrite	(0+0) Write EEPROM address in EEAdrTemp, Data in EEDataTemp, FSR0
;
;=========================================================================================
	ifndef UseEEParams
	constant UseEEParams=0
	endif
;=========================================================================================
; Clears all RAM
; Entry: none
; Exit: none
; RAM used: All
; Calls:(2+0) ClearRam_L2
;
ClearRam	MOVLB	0x00
	MOVLW	0x5F	;Clear 20h-7Eh, 95 bytes
	MOVWF	Param7F
	MOVLW	0x20
	MOVWF	FSR0
	CLRF	FSR0H
	CALL	ClearRam_L2
;
	MOVLW	0x50	;Clear A0h-EFh, 80 bytes
	MOVWF	Param7F
	MOVLW	0xA0
	MOVWF	FSR0
	CALL	ClearRam_L2
;
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 2,3
	INCF	FSR0H,F	
	CALL	ClearRam_2	;Banks 4,5
	INCF	FSR0H,F	;Banks 6,7
;	
ClearRam_2	MOVLW	0x50	;Clear 120h-16Fh, 80 bytes
	MOVWF	Param7F
	MOVLW	0x20
	MOVWF	FSR0
	CALL	ClearRam_L2
;
	MOVLW	0x50	;Clear A0h-EFh, 80 bytes
	MOVWF	Param7F
	MOVLW	0xA0
	MOVWF	FSR0
;
ClearRam_L2	CLRF	INDF0
	INCF	FSR0,F
	DECFSZ	Param7F,F
	bra	ClearRam_L2
	RETURN
;
	if UseEEParams
;==========================================================================
; copy param memory to ram
;
CopyToRam	MOVLW	nvFirstParamByte
	MOVWF	EEAddrTemp
	MOVLW	FirstRAMParam
	MOVWF	FSR0L
	CLRF	FSR0H
CopyToRam_L1	CALL	EERead
	movwi	FSR0++
	INCF	EEAddrTemp,F
	MOVLW	LastRAMParam+1
	SUBWF	FSR0L,W
	SKPZ
	BRA	CopyToRam_L1
	RETURN
;
;===========================================================================
; copy ram to param memory
;
SaveParams	MOVLW	nvFirstParamByte
	MOVWF	EEAddrTemp
	MOVLW	FirstRAMParam
	MOVWF	FSR0L
	CLRF	FSR0H
SaveParams_L1	moviw	FSR0++
	MOVWF	EEDataTemp
	CALL	EEWrite
	INCF	EEAddrTemp,F
	MOVLW	LastRAMParam+1	;last byte
	SUBWF	FSR0L,W
	SKPZ
	BRA	SaveParams_L1
	RETURN
;
	endif
;=====================================================================================================
;=========================================================================================================
; Decrement routine for 16 bit timers
; Set FSR0H before calling these routines.
;
DecTimer4	movlw	Timer4Hi
	bra	DecTimer
DecTimer3	movlw	Timer3Hi
	bra	DecTimer
DecTimer2	movlw	Timer2Hi
	bra	DecTimer
DecTimer1	movlw	Timer1Hi
;DecTimer
; entry: FSR=Timer(n)Hi
DecTimer	MOVWF	FSR0L
	movlw	High Timer4Hi
	movwf	FSR0H
	MOVIW	FSR0--	;TimerNHi
	IORWF	INDF0,W	;TimerNLo
	SKPNZ
	RETURN
	MOVLW	0x01
	SUBWF	INDF0,F	;TimerNLo
	INCF	FSR0L,F
	CLRW
	SUBWFB	INDF0,F	;TimerNHi
	RETURN
;
;==============================================================================================
; Test for 16 bit timers = zero
;If Timer is zero return Z flag,1 else Z=0
;
TestT4_Zero	movlb	0
	movf	Timer4Lo,W
	iorwf	Timer4Hi,W
	return
;
TestT3_Zero	movlb	0
	movf	Timer3Lo,W
	iorwf	Timer3Hi,W
	return
;
TestT2_Zero	movlb	0
	movf	Timer2Lo,W
	iorwf	Timer2Hi,W
	return
;
TestT1_Zero	movlb	0
	movf	Timer1Lo,W
	iorwf	Timer1Hi,W
	return	
;
	if oldCode
;======================================================================================
;Delay uS    1 cycle = 1uS, 8MHz clock version
; RAM used: Param77
; Calls:(0) none
;
Delay10uS	MOVLW	0x05	;(2*3+5)/2=10
	bra	DelayWuS
Delay100uS	MOVLW	d'65'	;(28*3+5)/2=100
	bra	DelayWuS
Delay40uS	MOVLW	d'25'	;(11*3+5)=40
DelayWuS	MOVWF	Param77
DelayWuS_Loop	DECFSZ	Param77,F
	bra	DelayWuS_Loop
	RETURN
;
	endif
;======================================================================================
;Delay uS    1 cycle = .125uS, 32MHz clock version
; RAM used: Param77
; Calls:(0) none
;
Delay10uS	MOVLW	0x05	;(2*3+5)/2=10
	bra	DelayWuS
Delay100uS	MOVLW	d'65'	;(28*3+5)/2=100
	bra	DelayWuS
Delay40uS	MOVLW	d'25'	;(11*3+5)=40
DelayWuS	MOVWF	Param77
DelayWuS_Loop	nop
	nop
	DECFSZ	Param77,F
	bra	DelayWuS_Loop
	RETURN
;==============================================================================================
; Read EEPROM
; entry: EEPROM address to read in W
;        Bank 0 selected
; exit: W=EEDATA, Bank 0 selected
;
EEReadW	movwf	EEAddrTemp
;
;==============================================================================================
; Read EEPROM
; entry: EEPROM address to read in EEAddrTemp
;        Bank 0 selected
; exit: W=EEDATA, Bank 0 selected
;
EERead	movf	EEAddrTemp,W
	MOVLB	0x03	;Bank 3
	movwf	EEADR	;Address to read
	bcf	EECON1,CFGS	;not config mem
	bcf	EECON1,EEPGD	;Data memory
	bsf	EECON1,RD
	movf	EEDAT,W	;W=EEDAT
	MOVLB	0x00	;Bank 0
	return
;
;==============================================================================================
; Write EEPROM
; entry: EEPROM address to write in W
;        EEPROM data to write in EEDataTemp
;        Bank 0 selected
; exit: Bank 0 selected
;
EEWriteW	movwf	EEAddrTemp
;
;==============================================================================================
; Write EEPROM
; entry: EEPROM address to write in EEAdrTemp
;        EEPROM data to write in EEDataTemp
;        Bank 0 selected
; exit: Bank 0 selected
;
EEWrite	MOVF	EEAddrTemp,W
	MOVLB	0x03	;Bank 3
	movwf	EEADR	;Address to write
	movlb	0x00	; bank 0
	movf	EEDataTemp,W
	movlb	0x03
	movwf	EEDAT
	bcf	EECON1,CFGS	;not config mem
	bcf	EECON1,EEPGD	;Data memory
	bsf	EECON1,WREN
	bcf	INTCON,GIE	;Disale Ints
	btfsc	INTCON,GIE
	bra	$-2
	movlw	0x55
	movwf	EECON2	;write 55
	movlw	0xAA
	movwf	EECON2	;write AA
	bsf	EECON1,WR
EEWriteLoop	btfsc	EECON1,WR	
	bra	EEWriteLoop
	bcf	EECON1,WREN
	bsf	INTCON,GIE
	MOVLB	0x00	;Bank 0
	return
;
;	
;=========================================================================================
;=========================================================================================
;
;
;
