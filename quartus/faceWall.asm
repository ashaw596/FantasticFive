; SimpleRobotProgram.asm
; Created by Kevin Johnson
; (no copyright applied; edit freely, no attribution necessary)
; This program does basic initialization of the DE2Bot
; and provides an example of some peripherals.

; Section labels are for clarity only.


ORG        &H000       ;Begin program at x000
;***************************************************************
;* Initialization
;***************************************************************
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
	SHIFT  5          ; Both LEDG6 and LEDG7
	STORE  Temp        ; (overkill, but looks nice)
	SHIFT  1
	OR     Temp
	OUT    XLEDS
	IN     XIO         ; XIO contains KEYs
	AND    Mask2       ; KEY3 mask (KEY0 is reset and can't be read)
	JPOS   WaitForUser ; not ready (KEYs are active-low, hence JPOS)
	LOAD   Zero
	OUT    XLEDS       ; clear LEDs once ready to continue

;***************************************************************
;* Main code
;***************************************************************
Main: ; "Real" program starts here.
		OUT    RESETPOS    ; reset odometry in case wheels moved after programming
		LOADI  &B001100
		OUT    SONAREN     ; turn on sonars 2 and 3
		LOADI	5
		CALL	WaitAC
		;CALL	FACECLOSE
		;CALL	FACEWFUNC
		LOADI  	&B100001
		OUT    	SONAREN     ; turn on sonars 0 and 5
		LOADI	5
		CALL	WaitAC
		CALL	LOCATE
		JUMP 	die		
		OUT    RESETPOS 
		LOADI	&HFEED
		OUT		LCD
		LOADI	90
		CALL	ROTATE
		CALL	FACEWFUNC	
		JUMP die
		
TURNR90:	OUT    RESETPOS
			LOADI	90
			CALL	ROTATE
			RETURN
MAXSONAR:	DW		&H7FFF

FACECLOSE:	OUT    	RESETPOS	;Turns to Global Mininum
			LOAD	MAXSONAR
			COPY	4, 0		;AC(4) = min_dist
			LOADI	0
			COPY	2, 0		;AC(2) = theta of min_dist [0,360]
FCLOSELOOP:	LOAD	RSLOW
			OUT    	LVELCMD
			LOAD	FSlow
			OUT    	RVELCMD				
			INA		3, THETA
			COPY	0, 3
			
			ADDI	-350
			JPOS	FCLOSEST
			COPY	0, 4
			OUT		SSEG2
FCLOSEB:	INA		1, DIST2
			OUTA	1, SSEG1
			SUBA	0, 4, 1
			JNEG	FCLOSELOOP
			COPY	2, 3
			COPY	4, 1
			OUTA	2, LCD
			OUTA	1, SSEG1
			JUMP	FCLOSELOOP
			
FCLOSEST:	COPY	0, 3
			ADDI	-355
			JPOS	FCLOSEB
			LOADI	0
			OUT    	LVELCMD
			OUT    	RVELCMD	
			COPY	0, 2
			CALL	CENTER
			CALL	ROTATE
			RETURN
			
		
FACEWFUNC:	LOADI 	5
			CALL 	WAITAC
			OUT		TIMER
FACEWALL:	IN		TIMER
			ADDI	-50
			JNEG	FACESKIP
			CALL	TURNR90
			OUT		TIMER
			
			;LOADI	10
			;CALL	WNoTimer
FACESKIP:	INA		1, DIST2
			INA		2, DIST3
			;LOADI	10
			;COPY	1, 0
			;LOADI	15
			;COPY	2, 0
			OUTA 	1, LCD
			OUTA 	2, SSEG2
			
		
			;JUMP	FACEWALL
			SUBA	0, 1, 2
			
			COPY 	3, 0
			CALL	ABS
			OUTA	0, SSEG1
			ADDI	-80
			JNEG	CLOSE
			LOADI	0
			COPY	4, 0
			COPY	0, 3
			JNEG	TURNLEFT
			JUMP	TURNRIGHT
CLOSE:		LOADI	0
			OUT    	LVELCMD
			OUT    	RVELCMD
			ADDIA	4,	1
			COPY	0, 	4
			ADDI	-10
			JNEG	FACEWALL
			RETURN
			
TURNRIGHT:	
			LOAD	FSlow
			OUT    	LVELCMD
			LOAD	RSLOW
			OUT    	RVELCMD	
			JUMP	FACEWALL
TURNLEFT:			
			LOAD	RSlow
			OUT    	LVELCMD
			LOAD	FSLOW
			OUT    	RVELCMD	
			JUMP	FACEWALL

			
Locate:
	CALL	MeasureDIST0
	OUT		SSEG1
	STORE	TilesBehind
	CALL	MeasureDIST5
	OUT		SSEG2
	STORE	TilesForward
	OUT		RESETPOS
	
	LOADI	-90
	CALL	ROTATE
	OUT		LCD
	CALL	MeasureDIST0
	OUT		SSEG1
	STORE	TilesLeft
	CALL	MeasureDIST5
	OUT		SSEG2
	STORE	TilesRight
	
	;Pack Bits
	LOAD	TilesForward
	SHIFT	3
	OR		TilesBehind
	SHIFT	3
	OR		TilesRight
	SHIFT	3
	OR		TilesLeft
	LOC		;Completes Lookup and stores result to AC
	
	JZERO	Locate			;LOC returns zero if no match found
	
	;Unpack Bits
	COPY	1, 0
	ANDI	&B111
	STORE	LocY
	COPY	0, 1
	SHIFT 	-3
	ANDI	&B111
	STORE	LocX
	COPY	0, 1
	SHIFT 	-6
	ANDI	&B111
	STORE	LocTheta
	COPY	0, 1
	SHIFT 	-5
	SHIFT	-4
	ANDI	&B11111
	STORE	LocID
	
	;Display Location
	LOAD	LocX
	SHIFT	4
	SHIFT	4
	OR		LocY
	OUT		SSEG1
	
	
	CALL	StopBeep
	RETURN
				
;***************************************************************
;* Stop for 3 seconds and beep for 1 second
;***************************************************************
StopBeep:
		LOAD	Zero
		OUT		LVELCMD
		OUT		RVELCMD
		OUT		TIMER
StopLoop:
		IN		TIMER
		ADDI	-10 ;Wait 1 Sec
		JNEG	StopLoop
		
		LOAD	TWO
		OUT		BEEP
		OUT		TIMER
BeepLoop:
		IN		TIMER
		ADDI	-10 ;Wait 1 Sec
		JNEG	BeepLoop
		
		OUT		TIMER
StopLoop2:
		IN		TIMER
		ADDI	-10 ;Wait 1 Sec
		JNEG	StopLoop2
		
		RETURN


;***************************************************************
;* Measure Sonar Distance and Store the value, in tiles, to AC
;***************************************************************

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

		
MeasureDIST5:
tiles50:	IN 		DIST5
		SUB		diff0
		JPOS	tiles51
		LOADI   0
		RETURN
tiles51:	IN		DIST5
		SUB		diff1
		JPOS	tiles52
		LOADI	1
		RETURN
tiles52:	IN		DIST5
		SUB		diff2
		JPOS	tiles53
		LOADI	2
		RETURN
tiles53:	IN		DIST5
		SUB		diff3
		JPOS	tiles54
		LOADI	3
		RETURN
tiles54:	IN		DIST5
		SUB		diff4
		JPOS	tiles55
		LOADI	4
		RETURN
tiles55: LOADI   5
		RETURN
		
;***************************************************************
;* Ryan's Constants	
;***************************************************************
diff0:    DW &H22F
diff1:    DW &H482
diff2:    DW &H6ED
diff3:    DW &H929
diff4:    DW &HB94

; Data Storage Location
TilesRight:		DW -1
TilesLeft:		DW -1
TilesForward:	DW -1
TilesBehind:	DW -1
locy:			DW	0
locx:			DW	0
locTheta:		DW	0
locID:			DW	0

	
ABS:		JPOS	RET		;Function ABS uses R6&R0. R0 = abs(R0)
			COPY	6, 0
			LOADI	0
			SUBA	0, 0, 6
RET:		RETURN
			
LOOPDIST:			
		CALL CENTER
		Shift 2
		STORE ANGLE
		ADD   FMid
		OUT   lvelcmd
		
		LOAD  Zero
		SUB   ANGLE
		ADD   FMid
		OUT   rvelcmd		
		
		IN	THETA
		OUT SSEG2
		IN	XPOS
		OUT	SSEG1
		SUB FOURFEET
		JNEG LOOPDIST
		return
LOOPANGLE:
		CALL CENTER 
		OUT SSEG1
		SUB TARGET
		JNEG RET
		LOAD FSlow
		OUT lvelcmd
		LOAD ZERO
		SUB FSlow
		OUT rvelcmd
		JUMP LOOPANGLE

Rotate:		COPY	6, 0	
			LOADI	0
			COPY	5, 0
RotateCheck:
			IN 		THETA
			CALL	CENTER
			SUBA	0, 6, 0
			JNEG 	CTURN
			JPOS	CCTURN
			LOADI	0		
			OUT 	rvelcmd
			OUT 	lvelcmd
			RETURN
CTURN:
			LOADI	SlowP
			OUT 	lvelcmd
			SUBA	0, 5, 0
			OUT 	rvelcmd
			JUMP 	RotateCheck
CCTURN:
			LOADI	SlowP
			OUT 	rvelcmd
			SUBA	0, 5, 0
			OUT 	lvelcmd 	
			JUMP	RotateCheck
		
ANGLE:  DW  0
TARGET: DW  -90
TEMP1:	DW	2	
ERROR:  DW  0
; The following code ("Center" through "DeadZone") is purely for example.
; It attempts to gently keep the robot facing 0 degrees, showing how the
; odometer and motor controllers work.
Center:		JNEG	CenterNeg
			ADDI   	-180        ; test whether facing 0-179 or 180-359
			JPOS   	SUB180    ; robot facing 180-360; handle that separately
			JUMP	ADD180

CenterNeg:	ADDI	180
			JPOS	SUB180
			JUMP	ADD180
					 
SUB180:		ADDI	-180
			RETURN
ADD180:		ADDI	180
			RETURN
CheckAngle:
	; AC now contains the +/- angular error from 0, meaning that
	;  the discontinuity is at 179/-180 instead of 0/359
	OUT    LCD         ; Good data to display for debugging
	; As an example of math, multiply the error by 5 :
	; (AC + AC<<2) = AC*5
	STORE  Temp
	SHIFT  2          ; divide by two
	ADD    Temp        ; add original value
	
	; Cap velcmd at +/-100 (a slow speed)
	JPOS   CapPos      ; handle +/- separately
CapNeg:
	ADD    DeadZone    ; if close to 0, don't do anything
	JPOS   NoTurn      ; (don't do anything)
	SUB    DeadZone    ; restore original value
	ADDI   100         ; check for <-100
	JPOS   NegOK       ; it was not <-100, so carry on
	LOAD   Zero        ; it was <-100, so clear excess
NegOK:
	ADDI   -100        ; undo the previous addition
	return
CapPos:
	SUB    DeadZone    ; if close to 0, don't do anything
	JNEG   NoTurn
	ADD    DeadZone    ; restore original value
	ADDI   -100
	JNEG   PosOK       ; it was not >100, so carry on
	LOAD   Zero        ; it was >100, so clear excess
PosOK:
	ADDI   100         ; undo the previous subtraction
	return
NoTurn:
	LOAD   Zero
	return
	
	; The desired velocity (angular error * 1.5, capped at
	;  +/-100, and with a 2-degree dead zone) is now in AC
SendToMotors:
	; Since we want to spin in place, we need to send inverted
	;  velocities to the wheels.
	STORE  Temp        ; store calculated desired velocity
	; send the direct value to the left wheel
;	ADD    FMid        ; Could add an offset vel here to move forward
	OUT    LVELCMD
	OUT    SSEG1       ; for debugging purposes
	; send the negated number to the right wheel
	LOAD   Zero
	SUB    Temp        ; AC = 0 - AC
;	ADD    Fmid        ; Could add an offset vel here to move forward
	OUT    RVELCMD	
	OUT    SSEG2       ; debugging
	
	JUMP   Center      ; repeat forever
	
DeadZone:  DW 10       ; Actual deadzone will be /5 due to scaling above.
	                   ; Note that you can place data anywhere.
                       ; Just be careful that it doesn't get executed.
	
Die:
; Sometimes it's useful to permanently stop execution.
; This will also catch the execution if it accidentally
; falls through from above.
	LOAD   Zero         ; Stop everything.
	OUT    LVELCMD
	OUT    RVELCMD
	OUT    SONAREN
	LOAD   DEAD         ; An indication that we are dead
	OUT    SSEG2
Forever:
	JUMP   Forever      ; Do this forever.
DEAD: DW &HDEAD

	
;***************************************************************
;* Subroutines
;***************************************************************

WNoTimer:	ADDI	-1
			JPOS	WNoTimer
			RETURN

WaitAC:		OUT		TIMER
			COPY	1, 0
WACLOOP:	IN		TIMER
			SUBA	0, 0, 1
			JNEG	WACLOOP
			RETURN

; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second in 10Hz.
	JNEG   Wloop
	RETURN
WaitOne:
	OUT    TIMER
Wloop2:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -5         ; 1 second in 10Hz.
	JNEG   Wloop2
	RETURN
	
; This subroutine will get the battery voltage,
; and stop program execution if it is too low.
; SetupI2C must be executed prior to this.
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

; Subroutine to send AC value through the UART,
; formatted for default base station code:
; [ AC(15..8) | AC(7..0) | \lf ]
; Note that special characters such as \lf are
; escaped with the value 0x1B, thus the literal
; value 0x1B must be sent as 0x1B1B, should it occur.
UARTSend:
	STORE  UARTTemp
	SHIFT  -8
	ADDI   -27   ; escape character
	JZERO  UEsc1
	ADDI   27
	OUT    UART_DAT
	JUMP   USend2
UEsc1:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
USend2:
	LOAD   UARTTemp
	AND    LowByte
	ADDI   -27   ; escape character
	JZERO  UEsc2
	ADDI   27
	OUT    UART_DAT
	RETURN
UEsc2:
	ADDI   27
	OUT    UART_DAT
	OUT    UART_DAT
	RETURN
	UARTTemp: DW 0

UARTNL:
	LOAD   NL
	OUT    UART_DAT
	SHIFT  -8
	OUT    UART_DAT
	RETURN
	NL: DW &H0A1B

;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 0 ; "Temp" is not a great name, but can be useful

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
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

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
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

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.05mm units
HalfMeter: DW 481      ; ~0.5m in 1.05mm units
TwoFeet:  DW 586       ; ~2ft in 1.05mm units
Deg90:    DW 90        ; 90 degrees in odometry units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 150       ; 100 is about the lowest velocity value that will move
RSlow:    DW -150
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 130       ; 13.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90
FOURFEET: DW	1172

;***************************************************************
;* IO address space map
;***************************************************************
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

SlowP:    EQU 	120       ; 100 is about the lowest velocity value that will move
