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
	
	JUMP DIE
	
;USER COMMANDS
;---------------------------------------------------------------------------------------
;Returns
;---------------------------------------------------------------------------------------------
;ASM CODE TO PERFORM WALL FOLLOWING ON THE COURSE
;WRITTEN BY TYLER ALLEN
WALL_SONAR_MASK: 			DW 	&B00101000	;THE CORRECT BINARY MASK???
WALL_DISTANCE_SIDE: 		DW 	&Hd5		;THE DISTANCE THE ROBOT WILL FOLLOW FROM THE WALL
WALL_DISTANCE_FORWARD: 		DW  &Hd5		;DISTANCE THE ROBOT IS INFRONT OF THE WALL AHEAD
WALL_SONAR_MIN_VALUE:		DW 	&H88		;MIN VALUE THAT THE SONAR CAN READ
WALL_SONAR_MAX_VALUE:		DW 	&HC00		;MAX VALUE THAT THE SONAR CAN READ
WALL_BASE_SPEED:			DW 	300			;BASE SPEED THE ROBOT IS TRAVELING WITHOUT ERROR
WALL_MOTOR_MIN:				DW	-500		;MINIMUM SPEED OF MOTOR
WALL_MOTOR_MAX:				DW	500			;MAX SPEED OF MOTOR
WALL_TEMP:					DW  0
WALLFOLLOW: 
	LOAD 	WALL_SONAR_MASK
	OUT  	SONAREN
WALL_LOOP:
	LOAD WALL_TEMP
	OUT LCD
	ADDI 1
	STORE WALL_TEMP
	;READ IN SONAR AND BOUND THE READINGS
	INA 	1, DIST5
	CALL 	WALL_BOUND_SOANR_READING
	INA 	2, DIST3
	SWITCH  1,2
	CALL 	WALL_BOUND_SOANR_READING

	;CALCULATE ERROR VALUES 
	;SIDE=R2,FORWARD=R1
	LOAD 		WALL_DISTANCE_SIDE
	SUBA		2,0,2
	
	LOAD		WALL_DISTANCE_FORWARD
	SUBA		1,0,1
	;CALCULATE MOTOR VALUES
	;SIDE ERROR=R2, FORWARD ERROR=R1
	CALL	WALL_CALCULATE_MOTOR_VALUES
	;R3=LEFT,R2=RIGHT
	;APPLY MOTOR VALUES

	OUTA 	3,LVELCMD
	OUTA 	2,RVELCMD
	CALL 	WALL_CHECK_END_COND
	JPOS 	WALL_LOOP
	JZERO	WALL_LOOP

	RETURN
;BOUNDS THE SONAR VALUES TO BE BETWEEN THE MIN<R1<MAX
WALL_BOUND_SOANR_READING:
	LOAD 	WALL_SONAR_MIN_VALUE
	SUBA	0,1,0	;SUBTRACT THE MIN VALUE FROM SONAR
	JNEG	WALL_BOUND_SMALL
	LOAD 	WALL_SONAR_MAX_VALUE
	SUBA	0,1,0 	;SUBTRACT THE MAX VALUE FROM SONAR 
	JPOS	WALL_BOUND_LARGE
	RETURN
WALL_BOUND_SMALL:
	LOAD	WALL_SONAR_MIN_VALUE
	COPY 1,0
	RETURN
WALL_BOUND_LARGE:
	LOAD	WALL_SONAR_MAX_VALUE
	COPY 1,0
	RETURN
;CALCULATE MOTOR VALUES BASED ON SONAR ERROR VALUES 
;R2=SIDE ERROR,R1=FORWARD ERROR
WALL_CALCULATE_MOTOR_VALUES:
	LOAD WALL_BASE_SPEED
	COPY 5,0
	COPY 0,1
	ADDA 6,5,0
	ADDA 7,5,0

	COPY 	1,6
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	6,1
	COPY 	1,7
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	7,1
	
	COPY 0,2
	ADDA 6,6,0
	SUBA 7,7,0
	COPY 2,6
	COPY 3,7
	

	COPY 	1,2
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	2,1
	COPY 	1,3
	CALL 	WALL_BOUND_MOTOR_VALUES 
	COPY 	3,1
	
	OUTA	2, SSEG1
	OUTA	3, SSEG2

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
	LOADI 1
	RETURN	
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

