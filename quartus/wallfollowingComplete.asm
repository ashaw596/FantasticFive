;Code 
;
;
Init:
	; Always a good idea to make sure the robot
	; stops in the event of a reset.
	LOAD   Zero
	OUT    LVELCMD     ; Stop motors
	OUT    RVELCMD
	OUT    SONAREN     ; Disable sonar (optional)
	
	CALL   SetupI2C    ; Configure the I2C to read the battery voltage
	CALL   BattCheck   ; Get battery voltage (and end if too low).
	OUT    LCD         ; Display batt voltage on LCD
WaitForSafety:
	; Wait for safety switch to be toggled
	IN     XIO         ; XIO contains SAFETY signal
	AND    Mask4       ; SAFETY signal is bit 4
	JPOS   WaitForUser ; If ready, jump to wait for PB3
	IN     TIMER       ; We'll use the timer value to
	AND    Mask1       ;  blink LED17 as a reminder to toggle SW17
	SHIFT  8           ; Shift over to LED17
	OUT    XLEDS       ; LED17 blinks at 2.5Hz (10Hz/4)
	JUMP   WaitForSafety
WaitForUser:
	; Wait for user to press PB3
	IN     TIMER       ; We'll blink the LEDs above PB3
	AND    Mask1
	SHIFT  5           ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue
;---------------------------------------------------------------------------------
MAIN:
	OUT  RESETPOS
	CALL WALLFOLLOW
	JUMP DIE

DIE:
	LOADI 	0
	OUT 	LVELCMD
	OUT 	RVELCMD
	loadi	&hDEAD
	out		lcd

	JUMP DIE
	
;USER COMMANDS
;---------------------------------------------------------------------------------------
;Returns
;---------------------------------------------------------------------------------------------
;ASM CODE TO PERFORM WALL FOLLOWING ON THE COURSE
;WRITTEN BY TYLER ALLEN
WALL_SONAR_MASK: 			DW 	&B00101101	;THE CORRECT BINARY MASK???
WALL_DISTANCE_SIDE: 		DW 	&Hd5		;THE DISTANCE THE ROBOT WILL FOLLOW FROM THE WALL
WALL_DISTANCE_FORWARD: 		DW  &Hd5		;DISTANCE THE ROBOT IS INFRONT OF THE WALL AHEAD
WALL_SONAR_MIN_VALUE:		DW 	&H88		;MIN VALUE THAT THE SONAR CAN READ
WALL_SONAR_MAX_VALUE:		DW 	&HC20		;MAX VALUE THAT THE SONAR CAN READ
WALL_BASE_SPEED:			DW 	0			;BASE SPEED THE ROBOT IS TRAVELING WITHOUT ERROR
WALL_MOTOR_MIN:				DW	-500		;MINIMUM SPEED OF MOTOR
WALL_MOTOR_MAX:				DW	500			;MAX SPEED OF MOTOR
WALL_FORWARD_ERROR:			DW	000			;STORE THE FORWARD ERROR FOR LATER
WALL_END_BOUND:				dw	&H04F		;BOUND FOR SONAR ERROR MEASUREMENT TO END
WALL_START_SQUARE:			DW	000			;START SQUARE FOR THE WALLFOLLOW
WALL_SQUARE_DIST:			DW	&H22F		;SQUARE DIST TO MEASURE THE SQUARES INFRONT
WALL_CURRENT_SQUARE:		DW	000			;CURRENT SQUARE
WALL_BAD_SONAR_VALUE:		DW	&h7FFF		;BAD SONAR VALUE
WALL_BAD_VALUE:				DW	0000		;SIGNALS IF A BAD VALUE HAS OCCURED
WALL_IGNORE_BAD_VALUE:		DW	0000		;SIGNALS IF A BAD VALUE SHOULD BE CHECKED
WALL_FOLLOW_DO_FAR:			DW	0000		;SIGNALS IF THE WALL FOLLOWING SHOULD FOLLOW FAR, IE IT SHOULD FOLLOW FROM ONE SQUARE AWAY. A SOLUTION TO THE SINGLE EDGE
WAYPOINT1:					DW 	0000
WAYPOINT2:					DW 	0000
WAYPOINT3:					DW	0000
WALLFOLLOW: 
	LOAD 	WALL_SONAR_MASK 
	OUT  	SONAREN
WALL_LOOP: 
	;READ IN SONAR AND BOUND THE READINGS
	LOAD 	WALL_FOLLOW_DO_FAR
	JPOS	WALL_FOLLOW_FAR
	;If wall follow is close to the wall ie. the robot is on a tile near the outside wall.
	;Use the sonar sensor on the right of the robot
WALL_FOLLOW_NEAR:
	INA 	1, DIST5
	JUMP 	WALL_CALC
	;If the the robot is far from the wall use the sonar on the left 
WALL_FOLLOW_FAR:
	INA 	1, DIST0
	;Perform wall following calculations 
WALL_CALC:
	;Bound sonar reading, if WALL_IGNORE_BAD_VALUE is true then ignore the bad value in the bounds 
	LOADI	1
	STORE 	WALL_IGNORE_BAD_VALUE
	CALL 	WALL_BOUND_SOANR_READING
	;Read the two front sonars and select the min on them
	INA 	2, DIST3
	INA 	3, DIST2
	CALL WALL_MIN
	;When min is found bound it to the correct value, always want to take into account bad values here
	SWITCH  1,2 ;Switch the registers since WALL_BOUND_SOANR_READING expects the sonar reading to be in R1
	LOADI	0
	STORE 	WALL_IGNORE_BAD_VALUE
	CALL 	WALL_BOUND_SOANR_READING

	;CALCULATE ERROR VALUES 
	;SIDE=R2,FORWARD=R1
	CALL 		MeasureDIST0;Calculate squares infront of the robot
	COPY		7,0
	LOAD		WALL_START_POS
	SUBA		0,7,0
	;On clearer thinking this does not make sense on how to calculate the position
	ADDA		0,7,0
	OUT			LCD
	STORE		WALL_CURRENT_SQUARE;Add to start square to find the current position
	
	;If bad value is detected don't do the error calculations, just move at a constant speed straight 
	LOAD		WALL_BAD_VALUE
	JPOS		WALL_SKIP_ERROR
	;Calculate Error values 
	LOAD 		WALL_DISTANCE_SIDE
	SUBA		2,2,0
	
	LOAD		WALL_DISTANCE_FORWARD
	SUBA		1,1,0
	COPY		0,1
	STORE		WALL_FORWARD_ERROR ;Store this error for later to use in the end check
	OUTA		1, SSEG1

	;CALCULATE MOTOR VALUES
	;SIDE ERROR=R2, FORWARD ERROR=R1
	;Calculate error values 
	CALL	WALL_CALCULATE_MOTOR_VALUES
	;R3=LEFT,R2=RIGHT
	;APPLY MOTOR VALUES
	;If the we reverse the direction of wall following the motor values have to be reversed 
	LOAD 	WALL_FOLLOW_DO_FAR
	JPOS	WALL_CALC_FAR
WALL_CALC_NEAR:
	OUTA 	2,LVELCMD
	OUTA 	3,RVELCMD
	JUMP  	WALL_LOOP_CHECK
WALL_CALC_FAR:
	OUTA	3,LVELCMD
	OUTA	2,RVELCMD
	;Check end conditions simply check if the sonar value is within WALL_END_BOUND 
WALL_LOOP_CHECK:
	CALL 	WALL_CHECK_END_COND
	JPOS	WALL_LOOP
	JZERO	WALL_LOOP
	;Return from function call 
WALL_END_CALL:
	RETURN
	;Move at a set speed if a bad value on the sonar is detected to move past the error state
WALL_SKIP_ERROR:
	LOADI	&h1234
	
	LOADI	200
	OUT		LVELCMD
	OUT		RVELCMD
	JUMP 	WALL_LOOP
;BOUNDS THE SONAR VALUES TO BE BETWEEN THE MIN<R1<MAX
WALL_BOUND_SOANR_READING:
	LOAD 	WALL_SONAR_MIN_VALUE
	SUBA	0,1,0	;SUBTRACT THE MIN VALUE FROM SONAR
	JNEG	WALL_BOUND_SMALL
	LOAD 	WALL_IGNORE_BAD_VALUE
	;Check for bad sonar values 
	JPOS	WALL_CHECK_BOUND_LARGE
	LOAD	WALL_BAD_SONAR_VALUE
	SUBA	0,1,0
	JPOS	WALL_BOUND_BAD
WALL_CHECK_BOUND_LARGE:
	LOAD 	WALL_SONAR_MAX_VALUE
	SUBA	0,1,0 	;SUBTRACT THE MAX VALUE FROM SONAR 
	JPOS	WALL_BOUND_LARGE
	LOADI 	0
	STORE 	WALL_BAD_VALUE
	RETURN
WALL_BOUND_SMALL:
	LOAD	WALL_SONAR_MIN_VALUE
	COPY 1,0
	RETURN
WALL_BOUND_LARGE:
	LOAD	WALL_SONAR_MAX_VALUE
	COPY 1,0
	RETURN
WALL_BOUND_BAD:
	LOADI 1
	STORE WALL_BAD_VALUE
	RETURN
;CALCULATE MOTOR VALUES BASED ON SONAR ERROR VALUES 
;R2=SIDE ERROR,R1=FORWARD ERROR
;R6,R7=Free, returns on ->R2,R3
WALL_CALCULATE_MOTOR_VALUES:
	LOAD WALL_BASE_SPEED
	COPY 5,0
	COPY 0,1 ;load error into a(0) to apply gains
	SHIFT -1 ;adjust gains 
	ADDA 6,5,0 ;Add to motor value registers
	ADDA 7,5,0
	;Bound the forward motion
	COPY 	1,6 					;Copy the motor value into R1 to call WALL_BOUND_MOTOR_VALUES
	CALL 	WALL_BOUND_MOTOR_VALUES ;BOund motor values on to ensure that the motor values are good
	COPY 	6,1 					;Copy the motor value back int R6 for temp storage
	COPY 	1,7 
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	7,1
	;Begin calculating the motor the turns based on distance from side wall
	COPY 0,2
	SHIFT -1	;Apply gains 
	ADDA 6,6,0	;Add to motor values on the side facing  the wall
	SUBA 7,7,0	;Sub to motor values on the side away from the wall
	COPY 2,6	;Put into r2,r3 to output
	COPY 3,7
	;Bound motor values to ensure they are legal motor values
	COPY 	1,2
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	2,1
	COPY 	1,3
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	3,1
	

	RETURN 
;BOUND THE MOTOR VALUES TO BE BETWEEN MIN<MOTOR<MAX
;THE MOTOR VALUE IS IN R1
WALL_BOUND_MOTOR_VALUES:
	LOAD WALL_MOTOR_MIN
	SUBA 0,1,0
	JNEG WALL_MOTOR_LOW
	LOAD WALL_MOTOR_MAX
	SUBA 0,1,0
	JPOS WALL_MOTOR_LARGE
	RETURN
WALL_MOTOR_LOW:
	LOAD WALL_MOTOR_MIN
	COPY 1,0
	RETURN
WALL_MOTOR_LARGE:
	LOAD WALL_MOTOR_MAX
	COPY 1,0
	RETURN
;WALL CHECK IF END CONDITION IS MET 
WALL_CHECK_END_COND:
	LOAD	WALL_FORWARD_ERROR ;Load error 
	JNEG	WALL_END_NEG		;Check if error is negative
	COPY 1,0	
	;If not negative subtract WALL_END_BOUND from the error to see if the error is reasonably close to zero 
WALL_END_CHECK:
	LOAD 	WALL_END_BOUND
	SUBA	0,1,0
	;OUT 	LCD
	RETURN	
	;If negative invert the error reading to make the calculation work
WALL_END_NEG:
	COPY	1,0
	LOADI 	0
	SUBA		1,0,1
	JUMP WALL_END_CHECK
;Find min of R2 and R3 and returns on R2
;R2,R3->R2
WALL_MIN:
	SUBA 0,2,3
	JPOS WALL_MIN_3
	return
WALL_MIN_3:
	COPY 2,3
	return
;--------------------------------------------------------------------------------------	
; Motion thresholds
diff0:    DW &H22F
diff1:    DW &H482
diff2:    DW &H6ED
diff3:    DW &H929
diff4:    DW &HB94
MeasureDIST0:
tiles00:	IN 		DIST0
		SUB		diff0
		JPOS	tiles01
		LOADI   0
		RETURN
tiles01:	IN		DIST0
		SUB		diff1
		JPOS	tiles02
		LOADI	1
		RETURN
tiles02:	IN		DIST0
		SUB		diff2
		JPOS	tiles03
		LOADI	2
		RETURN
tiles03:	IN		DIST0
		SUB		diff3
		JPOS	tiles04
		LOADI	3
		RETURN
tiles04:	IN		DIST0
		SUB		diff4
		JPOS	tiles05
		LOADI	4
		RETURN
tiles05: LOADI   5
		RETURN

Wait1:
	OUT    TIMER	
BattCheck:
	CALL   GetBattLvl
	JZERO  BattCheck   ; A/D hasn't had time to initialize
	SUB    MinBatt
	JNEG   DeadBatt
	ADD    MinBatt     ; get original value back
	RETURN
; If the battery is too low, we want to make
; sure that the user realizes it...
DeadBatt:
	LOAD   Four
	OUT    BEEP        ; start beep sound
	CALL   GetBattLvl  ; get the battery level
	OUT    SSEG1       ; display it everywhere
	OUT    SSEG2
	OUT    LCD
	LOAD   Zero
	ADDI   -1          ; 0xFFFF
	OUT    LEDS        ; all LEDs on
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	Load   Zero
	OUT    BEEP        ; stop beeping
	LOAD   Zero
	OUT    LEDS        ; LEDs off
	OUT    XLEDS
	CALL   Wait1       ; 1 second
	JUMP   DeadBatt    ; repeat forever
	
; Subroutine to read the A/D (battery voltage)
; Assumes that SetupI2C has been run
GetBattLvl:
	LOAD   I2CRCmd     ; 0x0190 (write 0B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	IN     I2C_DATA    ; get the returned data
	RETURN

; Subroutine to configure the I2C for reading batt voltage
; Only needs to be done once after each reset.
SetupI2C:
	CALL   BlockI2C    ; wait for idle
	LOAD   I2CWCmd     ; 0x1190 (write 1B, read 1B, addr 0x90)
	OUT    I2C_CMD     ; to I2C_CMD register
	LOAD   Zero        ; 0x0000 (A/D port 0, no increment)
	OUT    I2C_DATA    ; to I2C_DATA register
	OUT    I2C_RDY     ; start the communication
	CALL   BlockI2C    ; wait for it to finish
	RETURN
	
; Subroutine to block until I2C device is idle
BlockI2C:
	LOAD   Zero
	STORE  Temp        ; Used to check for timeout
BI2CL:
	LOAD   Temp
	ADDI   1           ; this will result in ~0.1s timeout
	STORE  Temp
	JZERO  I2CError    ; Timeout occurred; error
	IN     I2C_RDY     ; Read busy signal
	JPOS   BI2CL       ; If not 0, try again
	RETURN             ; Else return
I2CError:
	LOAD   Zero
	ADDI   &H12C       ; "I2C"
	OUT    SSEG1
	OUT    SSEG2       ; display error message
	JUMP   I2CError

	
	
	
	
	
	
	
	
	
	
	
	
	
;Data MEM	
;-----------------------------------------------------------------------------------
Temp:     DW 0 ; "Temp" is not a great name, but can be useful

NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

OneMeter: DW 961       ; ~1m in 1.05mm units
HalfMeter: DW 481      ; ~0.5m in 1.05mm units
TwoFeet:  DW 586       ; ~2ft in 1.05mm units
Deg90:    DW 90        ; 90 degrees in odometry units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90
FOURFEET: DW	1172
SONARMASKWF:    DW &B00101000
;IO ADDRESSES
;------------------------------------------------------------------------------------
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H98  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0

