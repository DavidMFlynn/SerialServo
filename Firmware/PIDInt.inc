;====================================================================================
; This file contains PID functions.
; File name:    PIDInt.inc
; Revision: 1.0b1 8/31/2019
;====================================================================================
; History:
; 1.0b1 8/31/2019	First rev'd version. Improved initialization.
;  2018 	derived from a Microchip Application Note
;====================================================================================
; Routines:
;
; public
;-----------------------------------------------------------
;	(bank in/bank exit)
; PID_Init	(-/0)Call to initialize variables as part of Servo Here
; PID_Main	(-/0)Call to get values, pidOut2:pidOut0, pid_sign
; Pid_Interrupt	(-/0)Call at loop frequency with error1:error0, err_z, err_sign
;
; private
;------------------------------------------------------------
; PID_Proportional	(3/3)gain(Kp) --> error0:error1 * Kp
; PID_Integral	gain(Ki)--> a_Error0:a_Error1 * Ki
; PID_Derivative	gain(Kd) --> d_Error0:d_Error1 * Kd
;	d_Error0:d_Error1 = error0:error1 - p_Error0:p_Error1
; PID_GetResult	pidOut0:pidOut2
; PID_GetA_Error	a_Error = a_Error + error (signed addition)
; ClampAArg	Clamp AArg to aErrLim
; PID_GetDeltaError	subtracts the previous error from the current error
; 
;*********************************************************************
;PID Notes:
;PROPORTIONAL = (system error * Pgain )
;System error = error0:error1
;	
;INTEGRAL = (ACUMULATED ERROR  * Igain)
;Accumulated error (a_error) = error0:error1 + a_Error0:a_Error2
;	
;DERIVATIVE = ((CURRENT ERROR  - PREVIOUS ERROR) * Dgain)
;delta error(d_error) = errro0:error1 - p_error0:p_error1
;	
;Integral & Derivative control will be based off sample periods of "x" time.
;The above sample period should be based off the PLANT response
;to control inputs.
;	SLOW Plant response = LONGER sample periods
;	FAST Plant response = SHORTER sample periods
;	
;If the error is equal to zero then no PID calculations are completed.
;	
;The PID routine is passed the 16 bit error data by the main application
; through the error1:error0, err_sign variables.
;
;The PID outputs a 24-bit vaule in pidOut2:pidOut0, pid_sign.
;-----------------------------------------------------------------------
;
;
;***** SYSTEM CONSTANTS
aErrLim	EQU	.4000	;accumulative error limits (4000d)
;
derivCountVal	EQU	.10	;determies how often the derivative term will be executed.
;
;	cblock
;
;	derivCount		;This value determins how many times the Derivative term is 
;			;calculated based on each Integral term.
;	pidOut0		;24-bit Final Result of PID for the "Plant"
;	pidOut1	
;	pidOut2	
;	error0		;16-bit error, passed to the PID
;	error1	
;	a_Error0		;24-bit accumulated error, used for Integral term
;	a_Error1	
;	a_Error2	
;	p_Error0		;16-bit previous error, used for Derivative term
;	p_Error1	
;	d_Error0		;16-bit delta error (error - previous error)
;	d_Error1	
;
;	prop0		;24-bit proportional value 
;	prop1		; = error * kp
;	prop2	
;	integ0		;24-bit Integral value 
;	integ1		; = a_Error * ki
;	integ2	
;	deriv0		;24-bit Derivative value 
;	deriv1		; = d_Error * kd
;	deriv2	
;
;	kp		;8-bit proportional Gain
;	ki		;8-bit integral Gain
;	kd		;8-bit derivative Gain
;
;	pidStat1		;PID status bits
;	pidStat2		;PID status bits
;	endc
;                               
;___________________________ pidStat1 register ________________________________________________
;|  bit 7   |   bit 6    |  bit 5 |    bit 4   |   bit 3    |  bit 2   |   bit 1    |  bit 0   |
;| pid_sign | d_err_sign |        | p_err_sign | a_err_sign | err_sign |  a_err_z   |  err_z   |
;|__________|____________|________|____________|____________|__________|____________|__________|
;
;#Define	err_z	pidStat1,0	;error zero flag, Zero = set
;#Define	a_err_z	pidStat1,1	;a_error zero flag, Zero = set
;#Define	err_sign	pidStat1,2	;error sign flag, Pos = set/ Neg = clear
;#Define	a_err_sign	pidStat1,3	;a_error sign flag, Pos = set/ Neg = clear
;#Define	p_err_sign	pidStat1,4	;p_error sign flag, Pos = set/ Neg = clear
;
;#Define	d_err_sign	pidStat1,6	;d_error sign flag, Pos = set/ Neg = clear
;#Define	pid_sign	pidStat1,7	;PID result sign flag, Pos = set/ Neg = clear
;
;________________________________ pidStat2 register______________________________________
;| bit 7 |  bit 6  |  bit 5   |    bit 4   |   bit 3    |  bit 2    |   bit 1    |  bit 0   |
;|       |         |          |error_limit | deriv_sign | BArg_sign | AArg_Sign  | d_err_z  |
;|_______|_________|__________|____________|____________|___________|____________|__________|
;
;#Define	d_err_z	pidStat2,0	;d_error zero flag, Zero = set
;#Define	AArg_sign	pidStat2,1	;AArg sign flag, Pos = set/ Neg = clear
;#Define	BArg_sign	pidStat2,2	;BArg sign flag, Pos = set/ Neg = clear
;#Define	deriv_sign	pidStat2,3	;deriv sign flag, Pos = set/ Neg = clear
;#Define	error_limit	pidStat2,4	;Error limit exceeded flag, error = set/ no error = clear
;
;
;***********************************************************************
; Function: PID_Init
;
; PreCondition: Called by the application code for PID initalization
;
; Overview: PID variables are cleared, PID gains are given values,
;	flags are initialized.
;
; Input:
;
; Output: none
;
;***********************************************************************
PID_Init	movlb	3
	movlw	derivCountVal
	movwf	derivCount	;derivative action = LoopFreq * derivCount
	bcf	err_z	;start w/ error not equal to zero
	bsf	a_err_z	;start w/ a_error equal to zero
	bsf	d_err_z	;start w/ d_error equal to zero
	bsf	p_err_sign	;start w/ previous error = positive
	bsf	a_err_sign	;start w/ accumulated error = positive
	bsf	deriv_sign
	clrf	pidOut0	;24-bit Final Result of PID for the "Plant"
	clrf	pidOut1
	clrf	pidOut2
	clrf	error0	;16-bit error, passed to the PID
	clrf	error1
	clrf	a_Error0	;24-bit accumulated error, used for Integral term
	clrf	a_Error1
	clrf	a_Error2
	clrf	p_Error0	;16-bit previous error, used for Derivative term
	clrf	p_Error1
	clrf	d_Error0	;16-bit delta error (error - previous error)
	clrf	d_Error1
;
	movlb	0
	return
;
;***********************************************************************
; Function: PID_Main
;
; PreCondition: error1:error0 are loaded with the latest system error
;
; Overview: This is the routine that the application code will call
;    to get a PID correction value. First, the error is checked
;    to determine if it is zero, if this is true, then the PID
;    code is complete.
;
; Input: error1:error0, err_sign, err_z
;
; Output: pidOut2:pidOut0, pid_sign
;
;***********************************************************************
PID_Main	movlb	3
;
;	movf	error1,W	;Is error = 00 ?
;	iorwf	error0,W
;	SKPZ
;	bra	call_pid_terms	;NO, done checking
;
;	bsf	err_z	;YES, set error zero flag
;	movlb	0
;	return		;return back to the main application code
;
;prop = kp * error
;integ = ki * a_error
;deriv = d_Error * kd
;
call_pid_terms	call	PID_Proportional	;NO, start with proportional term
	call	PID_Integral	;get Integral term	
	call	PID_Derivative	;get Derivative term
;
	call 	PID_GetResult	;get the final PID result that will go to the system
;
	movlb	0
	return
;	
;***********************************************************************
; Function: PID_Proportional
;
; PreCondition: error1:error0 are loaded with the latest system error
;
; Overview: This routine will multiply the system's 16-bit error by the
;           proportional gain(Kp) --> error1:error0 * Kp
;	
; Input: error1:error0, sign of the error: err_sign
;
; Output: prop2:prop0
;
; prop = kp * error
;
;***********************************************************************
PID_Proportional	movlw	low Kp
	movwf	FSR0L
	movlw	high Kp
	movwf	FSR0H
	moviw	0[FSR0]
	SKPNZ
	bra	PID_Proportional_Z
	movwf	BArgB0
	clrf	BArgB1
;
	movf	error0,W
	movwf	AArgB0
	movf	error1,W
	movwf	AArgB1
; Acc = AArgB * BArgB
	call	FXM1616U	;proportional gain * error	
;
;prop = Acc
	movf	AccB0,W
	movwf	prop0
	movf	AccB1,W
	movwf	prop1
	movf	AccB2,W
	movwf	prop2
	return
;
PID_Proportional_Z	clrf	prop0
	clrf	prop1
	clrf	prop2
	return
;
;***********************************************************************
; Function: PID_Integral
;
; PreCondition:  error1:error0 are loaded with the latest system error
;
; Overview: This routine will multiply the system's 16-bit accumulated
;           error by the integral gain(Ki)--> a_Error1:a_Error0 * Ki
;
; Input: a_Error1:a_Error0, sign of a_Error: a_err_sign
;
; Output: integ2:integ0
;
; integ = a_Error * Ki
;
;***********************************************************************
PID_Integral	btfsc	a_err_z	;Is a_error = 0
	bra	integral_zero	;Yes
; integ = ki * a_error
	movlw	low Ki	;No
	movwf	FSR0L
	movlw	high Ki
	movwf	FSR0H
	moviw	0[FSR0]
	SKPNZ
	bra	integral_zero
	movwf	BArgB0	;move the integral gain into BArgB0
	clrf	BArgB1
;
	movf	a_Error0,W
	movwf	AArgB0
	movf	a_Error1,W
	movwf	AArgB1
; Acc = AArgB * BArgB
	call	FXM1616U	;Integral gain * accumulated error
; integ = Acc
	movf	AccB0,W
	movwf	integ0
	movf	AccB1,W
	movwf	integ1
	movf	AccB2,W
	movwf	integ2
	return
;
integral_zero	clrf	integ0	;a_error = 0, clear Integral term
	clrf	integ1
	clrf	integ2
	return
;
;***********************************************************************
; Function: PID_Derivative
;
;
; Overview: This routine will multiply the system's 16-bit delta
;           error by the derivative gain(Kd) --> d_Error1:d_Error0 * Kd
;  deriv2:deriv0 = d_Error1:d_Error0 * Kd
;
; Input: d_Error1:d_Error0, d_err_z
;
; Output: deriv2:deriv0
;
;***********************************************************************
PID_Derivative	btfsc	d_err_z	;Is d_error = 0?
	bra	PID_Derivative_zero	;YES
; deriv = d_Error * kd
	movf	d_Error0,W
	movwf	BArgB0	;result ---> BArgB1
	movf	d_Error1,W
	movwf	BArgB1	;result ---> BArgB0
	movlw	low Kd
	movwf	FSR0L
	movlw	high Kd
	movwf	FSR0H
	moviw	0[FSR0]
	SKPNZ
	bra	PID_Derivative_zero
	movwf	AArgB0
	clrf	AArgB1
	call	FXM1616U	;Derivative gain * (error_l - prv_error1)
; deriv = Acc	
	movf	AccB0,W
	movwf	deriv0
	movf	AccB1,W
	movwf	deriv1
	movf	AccB2,W
	movwf	deriv2
	bcf	deriv_sign
	btfsc	d_err_sign
	bsf	deriv_sign
	return
;
PID_Derivative_zero	clrf	deriv0	;d_error = 0, clear Derivative term
	clrf	deriv1
	clrf	deriv2
	bsf	deriv_sign
	return
;
;***********************************************************************
; Function: PID_GetResult
;
; PreCondition: Proportional, Integral & Derivative terms have been
;	calculated. 
;
; Overview: This routine will add the PID terms and then scale down
;             the result by 16. This will be the final result that is
;             calcualted by the PID code.
;	
; Input: prop2:prop0, err_sign, integ2:integ0, a_err_sign, deriv2:deriv0, deriv_sign
;
; Output: pidOut2:pidOut0, pid_sign
;
; pidOut = (prop + integ + deriv)/16
;
;***********************************************************************
PID_GetResult	movf	prop0,W	;AArg <== prop
	movwf	AArgB0	;load Prop term & Integral term
	movf	prop1,W
	movwf	AArgB1
	movf	prop2,W
	movwf	AArgB2
	bcf	AArg_sign
	btfsc	err_sign
	bsf	AArg_sign
;
	movf	integ0,W	;BArg <== integ
	movwf	BArgB0
	movf	integ1,W
	movwf	BArgB1
	movf	integ2,W
	movwf	BArgB2
	bcf	BArg_sign
	btfsc	a_err_sign
	bsf	BArg_sign
;
	call	AddInt2424	;add signed numbers, AArg = AArg + BArg
;
;add_derivative
	movf	deriv0,W
	movwf	BArgB0	;AArg has result of Prop + Integ	
	movf	deriv1,W
	movwf	BArgB1	;load derivative term into BArg
	movf	deriv2,W
	movwf	BArgB2
	bcf	BArg_sign
	btfsc	deriv_sign
	bsf	BArg_sign
;
	call	AddInt2424	;add signed numbers
;
;scale_down
	clrf	BArgB1	;(Prop + Integ + Deriv) / 16 = FINAL PID RESULT to plant
	movlw	0x10
	movwf	BArgB0
	call	FXD2416U
;pidOut = AArg
	movf	AArgB2,W
	movwf	pidOut2
	movf	AArgB1,W
	movwf	pidOut1
	movf	AArgB0,W
	movwf	pidOut0
	bcf	pid_sign	;PID result is negative
	btfsc	AArg_sign
	bsf	pid_sign	;PID result is positive
	return
;
;***********************************************************************
; Function: PID_GetA_Error
;
; a_Error = a_Error + error (signed addition)
; if a_Error > .4000 then aError = .4000
;
;
; Overview: This routine will add the current error with all of the
;    previous errors. The sign of the accumulated error will
;    also be determined. After the accumulated error is
;    calculated then it is checked if it = 00 or as exceeded
;    the defined limits.
;
;
; Input: a_Error1:a_Error0, error1:error0
;
; Output: a_Error1:a_Error0, a_err_sign, a_err_z  (-4000 <= updated value <= 4000)
;
;***********************************************************************
PID_GetA_Error	movf	a_Error0,W	;BArg <== a_Error
	movwf	BArgB0	;load error & a_error 
	movf	a_Error1,W
	movwf	BArgB1
	movf	a_Error2,W
	movwf	BArgB2
	bcf	BArg_sign
	btfsc	a_err_sign
	bsf	BArg_sign
;
	clrf	AArgB2	;AArg <== error
	movf	error0,W
	movwf	AArgB0
	movf	error1,W
	movwf	AArgB1
	bcf	AArg_sign
	btfsc	err_sign
	bsf	AArg_sign
;
	call	AddInt2424	;call routine for add sign numbers
;
	movf	AArgB0,W	;a_Error <== AArg
	movwf	a_Error0	;store the a_error
	movf	AArgB1,W
	movwf	a_Error1
	movf	AArgB2,W
	movwf	a_Error2
	bcf	a_err_sign
	btfsc	AArg_sign
	bsf	a_err_sign
; set a_err_z?
	bcf	a_err_z	;clear a_error zero flag
	movf	a_Error0,W
	iorwf	a_Error1,W
	iorwf	a_Error2,W
	SKPZ
	bra	chk_a_Error2
	bsf	a_err_z
	return
;
;a_error reached limits?
chk_a_Error2	movlw	low aErrLim
	subwf	a_Error0,W
	movlw	high aErrLim
	subwfb	a_Error1,W
	movlw	0x00
	subwfb	a_Error2,W
	SKPNB		;a_Error>=aErrLim?
	return		; No
;restore_limit
	clrf	a_Error2	;YES, a_error limit has been exceeded
	movlw	high aErrLim
	movwf	a_Error1
	movlw	low aErrLim
	movwf	a_Error0
	return
;
;***********************************************************************
; Clamp AArg to aErrLim
ClampAArg	bcf	error_limit
	movlw	low aErrLim
	subwf	AArgB0,W
	movlw	high aErrLim
	subwfb	AArgB1,W
	movlw	0x00
	subwfb	AArgB2,W
	movlw	0x00
	subwfb	AArgB3,W
	SKPNB		;AArg>=aErrLim?
	return		; No
;restore_limit
	bsf	error_limit
	clrf	AArgB3
	clrf	AArgB2
	movlw	high aErrLim
	movwf	AArgB1
	movlw	low aErrLim
	movwf	AArgB0
	return
;
;***********************************************************************
; Function: PID_GetDeltaError
;
; Overview: This routine subtracts the previous error from the current
;	error. 
; d_Error = error - p_Error
; p_Error = error
;
; Input: P_Error1:p_Error0, p_err_sign, error1:error0, err_sign
;
; Output: d_Error1:d_Error0, d_err_sign, d_err_z
;
;***********************************************************************
PID_GetDeltaError	movf	error0,W	;load error and p_error
	movwf	AArgB0	; into AArg and BArg
	movf	error1,W
	movwf	AArgB1
	clrf	AArgB2	
	bcf	AArg_sign
	btfsc	err_sign
	bsf	AArg_sign
;
	movf	p_Error0,W
	movwf	BArgB0
	movf	p_Error1,W
	movwf	BArgB1
	clrf	BArgB2
	bsf	BArg_sign	;invert the sign
	btfsc	p_err_sign
	bcf	BArg_sign
; AArg = AArg - BArg
	call	AddInt2424	;call routine for add sign numbers
;
	movf	AArgB0,W
	movwf	d_Error0
	movf	AArgB1,W
	movwf	d_Error1
	bcf	d_err_sign	;d_error is negative
	btfsc	AArg_sign
	bsf	d_err_sign	;d_error is positive
;
; p_error = error
	movf	error0,W
	movwf	p_Error0	;load current error into previous for next deriavtive term
	movf	error1,W
	movwf	p_Error1	;load current error into previous for next deriavtive term
	bcf	p_err_sign	;make p_error negative
	btfsc	err_sign	;make p_error the same sign as error
	bsf	p_err_sign	;make p_error positive	
;
	bcf	d_err_z	;clear delta error zero bit	
	movf	d_Error0,W
	iorwf	d_Error1,W
	SKPNZ		;d_Error = 0?
	bsf	d_err_z	; Yes, set delta error zero bit
	return
;
;***********************************************************************
; Function: Pid_Interrupt
;
; PreCondition: This Routine will be called by the application's main
;	code.
;
; Overview: When Timer overflows, an updated value for the Integral
;    term will be calculated. An updated value for the derivative
;    term will be calculated if derivCount = 0. This routine
;    will check for error = 0, if this is true,then the routine
;    will return back to the main line code.
;
; Input: error1:error0 and sign in err_sign or err_z 
;
; Output: Integral & Derivative terms
;
;***********************************************************************
PID_Interrupt	movlb	3	;bank 3
	btfsc	err_z	;Is error = 00 ?
	bra	PID_Interrupt_end	;YES, done.
	call	PID_GetA_Error	;a_Error = a_Error + error (signed addition)
;
;derivative_ready?
	decfsz	derivCount,F 	;is it time to calculate d_error ?
	bra	PID_Interrupt_end	; NO
	call	PID_GetDeltaError	;error - p_error
;
	movlw	derivCountVal	;prepare for next delta error
	movwf	derivCount
;
PID_Interrupt_end	movlb	0	;bank 0
	return
;
