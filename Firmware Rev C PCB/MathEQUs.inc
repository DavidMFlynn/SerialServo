;===================================================================================
;  EUQ's for DMFMath.inc and PIDInt.inc
;  FileName: MathEQUs.inc
;  Date: 3/29/2020
;  File Version: 1.0.3 3/29/2020
;  
;  Author: David M. Flynn
;  Company: Oxford V.U.E., Inc.
;
;======================================================================================
; PID vars
	cblock	MathAddress
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