;*****************************************************************************************
;    Filename:      MagEncoder.inc
;    Date:          5/28/2017
;    File Version:  1.0.2   8/27/2020
;    
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;*****************************************************************************************
; AEAT-6012 Magnetic Encoder Reader
; Written for the 16F1847 running at 32MHz
; Delay 60mS before calling ReadEncoder the first time, power-up time.
; Add ReadEncoder to your main loop, must be call often enough so roll over/under
;  is seen.
; If the last ReadEncoder saw zero then MagEnc_Index is set.
; If a roll over/under was detected MagEnc_IndexLatch will be set, you must clear it
;  before calling ReadEncoder again.
; Works in any segment, requires no additional stack.
;
; Requires 2 output pins (CS* and CLK) and 1 input (Data)
;
; Constants for 12 bit encoder:
nDataBits             	EQU                    .12
kUpper2Bits            EQU                    0x0C
kEncHiBit	EQU	3
; Constants for 10 bit encoder:
;nDataBits             EQU                    .10
;kUpper2Bits           EQU                    0x03
;kEncHiBit	EQU	1
; 
; History:
;   1.0.2   8/27/2020  Fixed to asm w/ MacroPIC.
;   1.0.1   5/28/2017  Added Home routines.
;   1.0.0   5/27/2017  First Code and it WORKS!
;*****************************************************************************************
; Routines:
;
; ReadEncoder	Add change from last read to EncoderAccum,
;	 update MagEnc_Index and MagEnc_IndexLatch.
; Home_Absolute        Home absolute, Set home as encoder zero.
; Home_Here            Set Home as last read value.
;*****************************************************************************************
; Add the following to the Bank 0 variables.
;	cblock		;Variables bank0
;	EncoderAccum:3		;Accumulated distance
;	EncoderVal:2		;Value last read, raw 12 bit data
;	EncoderFlags                                  ;saved in eprom
;                      EncoderHome:2                                 ;Absolute Home, saved in eprom
;	endc
;
#Define	MagEnc_HighZone	EncoderFlags,0	
#Define	MagEnc_LowZone	EncoderFlags,1
#Define	MagEnc_Index	EncoderFlags,2
#Define	MagEnc_IndexLatch	EncoderFlags,3	;set if zero is crossed or Encode value = zero
#Define                MagEnc_Homed           EncoderFlags,4
;
; Change the following to use the available pins
MagEnc_CSBit	EQU	0	;RB0
MagEnc_CLKBit	EQU	4	;RB4
MagEnc_DataBit	EQU	6	;RA6
MagEnc_CSLAT	EQU	LATB
MagEnc_DataPort	EQU	PORTA
#Define	MagEnc_CS	MagEnc_CSLAT,MagEnc_CSBit	
#Define	MagEnc_CLK	MagEnc_CSLAT,MagEnc_CLKBit
#Define	MagEnc_Data	MagEnc_DataPort,MagEnc_DataBit
;=========================================
; ReadEncoder
; Entry: none
; Exit: EncoderValue,EncoderAccum,EncoderFlags
; Calls: none
; Uses: Param76,Param77,Param78,Param79,Param7A FSR0
;
ReadEncoder	movlb	0	;bank0
	movlw	nDataBits	;read 12 bits
	movwf	Param78
	movlw	low MagEnc_CSLAT
	movwf	FSR0L
	movlw	high MagEnc_CSLAT
	movwf	FSR0H
	bcf	INDF0,MagEnc_CSBit     ;assert CS*
;			;delay 500ns
	movf	EncoderVal,W
	movwf	Param79                ;retain old value
	movf	EncoderVal+1,W
	movwf	Param7A
	clrf	EncoderVal
	clrf	EncoderVal+1
;
;Loop to read the data.
ReadEncoder_L1	bcf	INDF0,MagEnc_CLKBit    ;Clock
	nop		;delay 500ns
	nop
	nop
	nop
	nop
	bsf	INDF0,MagEnc_CLKBit
	nop		;delay 375ns
	nop
	nop
	nop
	bcf	_C
	btfsc	MagEnc_Data
	bsf	_C
	rlf	EncoderVal,F
	rlf	EncoderVal+1,F
	decfsz	Param78,F
	bra	ReadEncoder_L1
;
	bsf	INDF0,MagEnc_CSBit
;copy to temp
	movf	EncoderVal,W
	movwf	Param76
	movf	EncoderVal+1,W
	movwf	Param77
;
; handle overflow, goes from >3072 to <1024
	btfss	MagEnc_HighZone
	bra	ReadEncoder_1
	movlw	kUpper2Bits
	andwf	Param77,W
	SKPNZ		;Less than 1024?
	bsf	Param77,4	; yes, set 13th bit
	bsf	MagEnc_IndexLatch	;crossed zero
	bra	ReadEncoder_2
;
; handle underflow, goes from <1024 to <=3072
ReadEncoder_1	btfss	MagEnc_LowZone
	bra	ReadEncoder_2
	movlw	kUpper2Bits
	andwf	Param77,W
	xorlw	kUpper2Bits
	SKPZ		;>=3072?
	bra	ReadEncoder_2	; No
;Val = 0 - value
	movf	Param76,W
	clrf                   Param76
	subwf	Param76,F
	movf                   Param77,W
	clrf                   Param77	
	subwfb	Param77,F
	bsf	MagEnc_IndexLatch	;crossed zero
;
; calculate new flag values
ReadEncoder_2	bcf	MagEnc_Index
	movf	EncoderVal,W
	iorwf	EncoderVal+1,W
	SKPNZ
	bsf	MagEnc_Index
;
	btfsc	MagEnc_Index
	bsf	MagEnc_IndexLatch
;
	bcf	MagEnc_LowZone
	movlw	kUpper2Bits
	andwf	EncoderVal+1,W
	SKPNZ		;Les than 1024?
	bsf	MagEnc_LowZone	; Yes
;
	bcf	MagEnc_HighZone
	movlw	kUpper2Bits
	andwf	EncoderVal+1,W	;2 high bits
	xorlw	kUpper2Bits
	SKPNZ		;both bits set, >=3072
	bsf	MagEnc_HighZone	; yes
;
; calc change = new - old
	movf	Param76,W
	subwf	Param79,W	;old value
	movwf	Param76
	movf	Param77,W
	subwfb	Param7A,W	;old value high byte
	movwf	Param77
	clrf	Param78	;sign extend to 24 bits
	movlw	0xFF
	btfsc	Param77,7	;sign bit set?
	movwf	Param78	; Yes
;
; add the change to the accumulated value
	movf	Param76,W
	addwf	EncoderAccum,F
	movf	Param77,W
	addwfc	EncoderAccum+1,F
	movf	Param78,W
	addwfc	EncoderAccum+2,F
;
	return
;
;=========================================================================================
; Home absolute, Set home as encoder HM absolute.
; Call ReadEncoder once first.
; Entry: HM = Absolute Home, EncoderVal = Current Position Absolute,
;        kMaxT = ±Max travel from HM,
;        EncoderAccum = 0 = Current Position Incremental
; Exit: EncoderAccum = Current Position Relitive
;
; The Math:
;  ' tests for Current Position > HM
;  if HM + kMaxT >= 4096 then 'Dual zone
;    if EncoderVal > HM then 
;      'Current Position is positive between HM and 4096
;      EncoderAccum = EncoderVal - HM
;    elseif EncoderVal < ((HM + kMaxT) mod 4096) then 
;      'Current Position is positive between 0 and HM + kMaxT
;      EncoderAccum = EncoderVal + 4096 - HM
;  else 'Single zone
;    if EncoderVal > HM and EncoderVal <= HM + kMaxT then
;      'Current Position is positive between HM and HM + kMAxT
;      EncoderAccum = EncoderVal - HM
;
;  ' test for Current Position < HM
;  if HM - kMaxT < 0 then 'Dual zone
;    if (EncoderVal < HM) then
;      'Current Position is between HM and 0
;      EncoderAccum = EncoderVal - HM
;    elseif (EncoderVal > 8192 - HM - kMaxT) then
;      'Current Position is between 0 and HM - kMaxT
;      EncoderAccum = EncoderVal - 4096 - HM
;  else 'Single zone
;    if EncoderVal < HM and EncoderVal > HM - kMaxT then
;      'Current Position is between HM and HM - kMaxT
;      EncoderAccum = EncoderVal - HM
;      
;
kMaxT                  EQU                    .1900                  ;Less than a half turn
;
Home_Absolute	call                   Home_Here
                       btfss                  MagEnc_Homed           ;First boot?
                       bra                    SaveHomePos            ; Yes
;
;  ' tests for Current Position > HM
;if HM + kMaxT >= 4096 then 'Dual zone
                       movlw                  low kMaxT
                       addwf                  EncoderHome,W
                       movwf                  Param78
                       movlw                  high kMaxT
                       addwfc                 EncoderHome+1,W
                       movwf                  Param79
                       andlw                  0xF0
                       SKPNZ
                       bra                    Home_Absolute_1
;    if EncoderVal > HM then 
;      'Current Position is positive between HM and 4096
;      EncoderAccum = EncoderVal - HM
                       movf                   EncoderVal,W
                       subwf                  EncoderHome,W
                       movf                   EncoderVal+1,W
                       subwfb                 EncoderHome+1,W
                       btfsc                  WREG,7                 ;EncoderVal > HM?
                       bra                    HA_SetIt               ; Yes
                       bra                    Home_Absolute_2
;        
;  else 'Single zone
;    if EncoderVal > HM and EncoderVal < HM + kMaxT then
;      'Current Position is positive between HM and HM + kMAxT
;      EncoderAccum = EncoderVal - HM
Home_Absolute_1        movf                   EncoderHome,W
                       subwf                  EncoderVal,W
                       movf                   EncoderHome+1,W
                       subwfb                 EncoderVal+1,W
                       btfss                  WREG,7
                       bra                    Home_Absolute_2
                       movf                   Param78,W              ;HM + kMaxT
                       subwf                  EncoderVal,W
                       movf                   Param79,W
                       subwfb                 EncoderVal+1,W
                       btfsc                  WREG,7                 ;EncoderVal < HM + kMaxT?
                       bra                    HA_SetIt               ; Yes
;
;  ' test for Current Position < HM
;  if HM - kMaxT < 0 then 'Dual zone
Home_Absolute_2        movlw                  low kMaxT
                       subwf                  EncoderHome,W
                       movlw                  high kMaxT
                       subwfb                 EncoderHome+1,W
                       btfss                  WREG,7                 ;HM - kMaxT < 0?
                       bra                    Home_Absolute_3        ; No, not dual zone
;    if (EncoderVal < HM) then
;      'Current Position is between HM and 0
;      EncoderAccum = EncoderVal - HM
                       movf                   EncoderHome,W
                       subwf                  EncoderVal,W
                       movf                   EncoderHome+1,W
                       subwfb                 EncoderVal+1,W
                       btfsc                  WREG,7                 ;EncoderVal < HM?
                       bra                    HA_SetIt               ; Yes
;    elseif (EncoderVal > 8192 - HM - kMaxT) then
;      'Current Position is between 0 and HM - kMaxT
;      EncoderAccum = EncoderVal - 4096 - HM
                       movlw                  low .8192              ;Param78:Param79 = .8192
                       movwf                  Param78
                       movlw                  high .8192
                       movwf                  Param79
                       movf                   EncoderHome,W          ;Param78:Param79 -= EncoderHome
                       subwf                  Param78,F
                       movf                   EncoderHome+1,W
                       subwfb                 Param79,F
                       movlw                  low kMaxT              ;Param78:Param79 -= kMaxT
                       subwf                  Param78,F
                       movlw                  high kMaxT
                       subwfb                 Param79,F
                       movf                   EncoderVal,W 
                       subwf                  Param78,W
                       movf                   EncoderVal+1,W
                       subwfb                 Param79,W
                       btfsc                  WREG,7                 ;EncoderVal > Param78:Param79?
                       bra                    HA_SetItNeg            ; Yes
                       return                                        ; No, not in this zone
;
;  else 'Single zone
;    if EncoderVal < HM and EncoderVal > HM - kMaxT then
;      'Current Position is between HM and HM - kMaxT
;      EncoderAccum = EncoderVal - HM
Home_Absolute_3        movf                   EncoderHome,W
                       subwf                  EncoderVal,W
                       movf                   EncoderHome+1,W
                       subwfb                 EncoderVal+1,W
                       btfss                  WREG,7                 ;EncoderVal < HM?
                       return                                        ; No
                       movlw                  low kMaxT              ;Param78:Param79 = EncoderHome - kMaxT
                       subwf                  EncoderHome,W
                       movwf                  Param78
                       movlw                  high kMaxT
                       subwfb                 EncoderHome+1,W
                       movwf                  Param79
                       movf                   EncoderVal,W
                       subwf                  Param78,W
                       movf                   EncoderVal+1,W
                       subwfb                 Param79,W
                       btfsc                  WREG,7                 ;EncoderVal > HM - kMaxT?
                       bra                    HA_SetIt               ; Yes
                       return                                        ; No, must be zero
;
;======================
SaveHomePos            bsf                    MagEnc_Homed
                       movf                   EncoderVal,W
                       movwf                  EncoderHome
                       movf                   EncoderVal+1,W
                       movwf                  EncoderHome+1
                       mLongCall              SaveParams
                       return
;======================
;EncoderAccum = EncoderVal + 4096 - HM
;                      movlw                  low .4096
;                      addwf                  EncoderVal,F
HA_SetItPos            movlw                  high .4096
                       addwfc                 EncoderVal+1,F
                       bra                    HA_SetIt
;======================
;EncoderAccum = EncoderVal - 4096 - HM
;                      movlw                  low .4096
;                      subwf                  EncoderVal,F
HA_SetItNeg            movlw                  high .4096
                       subwfb                 EncoderVal+1,F
;======================
;EncoderAccum = EncoderVal - HM
HA_SetIt               movf                   EncoderHome,W
                       subwf                  EncoderVal,W
                       movwf                  EncoderAccum
                       movf                   EncoderHome+1,W
                       subwfb                 EncoderVal+1,W
                       movwf                  EncoderAccum+1
                       movlw                  0xFF
                       btfss                  EncoderAccum+1,7
                       movlw                  0x00
                       movwf                  EncoderAccum+2
                       return
;
;                       
;=========================================================================================
; Set Home as last read value
; Call ReadEncoder once first.
;
Home_Here              movlb                  0
                       clrf                   EncoderAccum
                       clrf                   EncoderAccum+1
                       clrf                   EncoderAccum+2
                       return
;
;=========================================================================================

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
