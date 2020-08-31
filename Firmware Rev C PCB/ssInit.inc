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
	movlb	4	; bank 4
	bsf	WPUA,WPUA5	;Put a pull up on the MCLR unused pin.
;
	MOVLB	ANSELA	; bank 3
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
;SPI MISO >> SDI1 RB1, default
;SPI CLK >> RB4, default
	movlb	2	;bank 2
	bsf	APFCON0,RXDTSEL	;RX >> RB2
	bsf	APFCON1,TXCKSEL	;TX >> RB5
	bsf	APFCON0,SDO1SEL	;SPI MOSI >> SDO1 RA6
;	
; clear memory to zero
	mLongCall	ClearRam
	CLRWDT
	mLongCall	CopyToRam
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
;
;if mode 3 don't move
	bsf	ssCmdPos+1,7
;
	CLRWDT
;
	call	Init_AS5047D	;initialize the SPI encoder I/O
;
	movlb	1	; bank 0
	bsf	AN0_ActiveBit
	bsf	AN1_ActiveBit
;
                       call                   DoAuxConfig
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
	return
;=============================================================
;
DoAuxConfig	movf                   ssAux0Config,W              ;AuxNum
                       brw
                       bra                    SetAux0ConfigNone
                       bra                    SetAux0ConfigLEDBtn
                       bra                    SetAux0ConfigDigIn
                       bra                    SetAux0ConfigDigOut
                       bra                    SetAux0ConfigAnalogIn
                       bra                    SetAux0ConfigHomeSw
                       bra                    SetAux0ConfigFwdLimit
                       bra                    SetAux0ConfigRevLimit
;
SetAux0ConfigNone:
SetAux0ConfigLEDBtn:
SetAux0ConfigDigIn:
SetAux0ConfigHomeSw:
SetAux0ConfigFwdLimit:
SetAux0ConfigRevLimit:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux0_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux0_ActiveBit      ;Don't Read AN
                       bsf                    Aux0_LED1_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux0_LED1_Lat          ;0
                       movlb                  0                      ;bank 0
                       bra                    SetAux1Config
;
SetAux0ConfigDigOut:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux0_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux0_ActiveBit      ;Don't Read AN
                       bcf                    Aux0_LED1_TRIS         ;Output
                       movlb                  2                      ;bank 2
                       bcf                    Aux0_LED1_Lat          ;0
                       movlb                  0                      ;bank 0
                       bra                    SetAux1Config
;
SetAux0ConfigAnalogIn:
                       movlb                  ANSELA                 ;bank 3
                       bsf                    Aux0_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bsf                    AN_Aux0_ActiveBit      ;Don't Read AN
                       bsf                    Aux0_LED1_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux0_LED1_Lat          ;0
                       movlb                  0                      ;bank 0
;
SetAux1Config	movf                   ssAux1Config,W              ;AuxNum
                       brw
                       bra                    SetAux1ConfigNone
                       bra                    SetAux1ConfigLEDBtn
                       bra                    SetAux1ConfigDigIn
                       bra                    SetAux1ConfigDigOut
                       bra                    SetAux1ConfigAnalogIn
                       bra                    SetAux1ConfigHomeSw
                       bra                    SetAux1ConfigFwdLimit
                       bra                    SetAux1ConfigRevLimit
;
SetAux1ConfigNone:
SetAux1ConfigLEDBtn:
SetAux1ConfigDigIn:
SetAux1ConfigHomeSw:
SetAux1ConfigFwdLimit:
SetAux1ConfigRevLimit:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux1_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux1_ActiveBit      ;Don't Read AN
                       bsf                    Aux1_LED2_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux1_LED2_Lat          ;0
                       movlb                  0                      ;bank 0
                       bra                    SetAux2Config
;
SetAux1ConfigDigOut:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux1_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux1_ActiveBit      ;Don't Read AN
                       bcf                    Aux1_LED2_TRIS         ;Output
                       movlb                  2                      ;bank 2
                       bcf                    Aux1_LED2_Lat          ;0
                       movlb                  0                      ;bank 0
                       bra                    SetAux2Config
;
SetAux1ConfigAnalogIn:
                       movlb                  ANSELA                 ;bank 3
                       bsf                    Aux1_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bsf                    AN_Aux1_ActiveBit      ;Don't Read AN
                       bsf                    Aux1_LED2_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux1_LED2_Lat          ;0
                       movlb                  0                      ;bank 0
;
SetAux2Config	movf                   ssAux2Config,W              ;AuxNum
                       brw
                       bra                    SetAux2ConfigNone
                       bra                    SetAux2ConfigLEDBtn
                       bra                    SetAux2ConfigDigIn
                       bra                    SetAux2ConfigDigOut
                       bra                    SetAux2ConfigAnalogIn
                       bra                    SetAux2ConfigHomeSw
                       bra                    SetAux2ConfigFwdLimit
                       bra                    SetAux2ConfigRevLimit
;
SetAux2ConfigNone:
SetAux2ConfigLEDBtn:
SetAux2ConfigDigIn:
SetAux2ConfigHomeSw:
SetAux2ConfigFwdLimit:
SetAux2ConfigRevLimit:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux2_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux2_ActiveBit      ;Don't Read AN
                       bsf                    Aux2_LED3_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux2_LED3_Lat          ;0
                       movlb                  0                      ;bank 0
                       return
;
SetAux2ConfigDigOut:
                       movlb                  ANSELA                 ;bank 3
                       bcf                    Aux2_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bcf                    AN_Aux2_ActiveBit      ;Don't Read AN
                       bcf                    Aux2_LED3_TRIS         ;Output
                       movlb                  2                      ;bank 2
                       bcf                    Aux2_LED3_Lat          ;0
                       movlb                  0                      ;bank 0
                       return
;
SetAux2ConfigAnalogIn:
                       movlb                  ANSELA                 ;bank 3
                       bsf                    Aux2_ANSEL_Bit         ;Digital
                       movlb                  1                      ;bank 1
                       bsf                    AN_Aux2_ActiveBit      ;Don't Read AN
                       bsf                    Aux2_LED3_TRIS         ;Input
                       movlb                  2                      ;bank 2
                       bcf                    Aux2_LED3_Lat          ;0
                       movlb                  0                      ;bank 0
                       return
;
;
