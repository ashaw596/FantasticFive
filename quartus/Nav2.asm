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
	JUMP DIE

DIE:
	
	JUMP DIE
	
;USER COMMANDS
;---------------------------------------------------------------------------------------
;Returns
;---------------------------------------------------------------------------------------------
WAYPOINT1:		DW 		000
WAYPOINT2:		DW		000
WAYPOINT3:		DW		000
;Turn right to theta, -180L-0-180R RELATIVE TO ROBOT
;R0->Theta to move to 
TURN_THETA:		DW		000
TURN_ANGLE:
	STORE	TURN_THETA
TURN_ANGLE_LOOP:
	IN		THETA
	CALL 	CORRECT_ANGLE
	COPY	1,0
	LOAD	TURN_THETA	;LOAD TARGET ANGLE
	SUBA	1,0,1		;CALC ERROR
	LOADI	5			;LOAD BOUNDS
	SWITCH	0,1			;SWITHC R0,R1 TO BE IN POSITION FOR CHECK_BOUND
	CALL	CHECK_BOUND
	JPOS	CMD_END
	;APPLY GAINS
	COPY 	0,1
	SHIFT	1
	;POSTIVE VALUE ON RIGHT 
	OUT		RVELCMD
	;NEGATIVE POWER ON LEFT WHEEL
	COPY	2,0
	LOADI	0
	SUBA	0,0,2
	OUT		LVELCMD
	JUMP	TURN_ANGLE_LOOP
CMD_END:
	LOADI	0
	OUT LVELCMD
	OUT RVELCMD
	RETURN
;GIVEN ROBOT ANGLE R0 IN GLOBAL REFERENCE,TURN TO GLOBAL ANGLE R1
;0 IS UP, ANGLES CLOCKWISE TO 359
TURN_GLOBAL:
	SUBA	0,0,1			;FIND RELATIVE ANGLE TO TURN
	CALL CORRECT_ANGLE		;CONVERT INTO ROBOT ANGLE FOR TURN ANGLE
	CALL TURN_ANGLE
;DRIVE STRAIGHT FOR X DISTANCE
;R0->DISTANCE TO TRAVEL
DRIVE_X:			DW		000
DRIVE_STRAIGHT:
	OUT RESETPOS
	STORE	DRIVE_X
DRIVE_LOOP:
	IN		XPOS
	COPY	1,0
	LOAD	DRIVE_X		;LOAD TARGET ANGLE
	SUBA	1,0,1		;CALC ERROR
	LOADI	5			;LOAD BOUNDS
	SWITCH	0,1			;SWITHC R0,R1 TO BE IN POSITION FOR CHECK_BOUND
	CALL	CHECK_BOUND
	JPOS	CMD_END	
	;APPLY GAINS
	COPY	0,1
	SHIFT	1
	CALL 	BOUND_MOTOR_VALUES
	COPY 	3,0
	IN		THETA
	CALL 	CORRECT_ANGLE
	SHIFT	-1
	ADDA	1,3,0	;ADD ANY ANGLUAR ERROR TO THE RIGHT WHEEL
	SUBA	2,3,0	;SUB ANY ANGULAR ERROR TO THE LEFT  WHEEL
	OUTA	1,RVELCMD
	OUTA	2,LVELCMD
	JUMP	DRIVE_LOOP
	
;RETURNS ANGLE -180-0-180 FROM THETA
;R0->IN,OUT->R0
CORRECT_ANGLE:
	ADDI	-180
	JPOS	CORRECT_ANGLE_NEG
	ADDI	180
	RETURN
CORRECT_ANGLE_NEG
	ADDI-180
	RETURN

;BOUNDS POS IF THE NUMBER IN IS WITHNIN THE BOUNDS -BOUND<IN<BOUND
;R0->IN,R1->BOUND
;OUT->R0,IN->R1
CHECK_BOUND:
	JNEG	CHECK_BOUND_NEG
CHECK_BOUND_CALC:
	SUBA	0,1,0
	RETURN
CHECK_BOUND_NEG:
	COPY 	3,0
	LOADI	0
	SUBA	0,0,3
	JUMP	CHECK_BOUND_CALC
;BOUND THE MOTOR VALUES TO BE BETWEEN MIN<MOTOR<MAX
;THE MOTOR VALUE IS IN R1
MOTOR_MIN:			DW			-500
MOTOR_MAX:			DW			 500
BOUND_MOTOR_VALUES:
	LOAD MOTOR_MIN
	SUBA 0,1,0
	JNEG MOTOR_LOW
	LOAD MOTOR_MAX
	SUBA 0,1,0
	JPOS MOTOR_LARGE
	RETURN
MOTOR_LOW:
	LOAD MOTOR_MIN
	COPY 1,0
	RETURN
MOTOR_LARGE:
	LOAD MOTOR_MAX
	COPY 1,0
	RETURN

;Read next target
READ_NEXT_TARGET
;--------------------------------------------------------------------------------------	
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

