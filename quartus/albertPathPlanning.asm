
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
				LOADI  &B00101101
				OUT    SONAREN     ; turn on sonars 2 and 3
				
				LOAD	SQUARESize
				ADD		SQUARESize
				STORE	FOLLOWDIST
				CALL 	WALLLOOP
				;OUT
				JUMP 	die

POS1V:			DW		1
POS2V:			DW		1
POS3V:			DW		1
CURPOS:			DW		0
POS1:			DW		1
POS2:			DW		5
POS3:			DW		10
	
SHOWLOC:		LOAD	CURPOS
				;CALL	SHOWLOCATION
				JUMP	SHOWDONE
PATHFOLLOWING:	;OUT		RESETPOS


PATHLOOP:		LOAD	CURPOS

				CALL	CHECKONGOAL
				JPOS	SHOWLOC
				
SHOWDONE:		JUMP	PATHCHECK
				
CHECKDONE:		COPY	1, 0
				ADDI	-6
				JNEG	PATH_1_5
				COPY	0, 1
				ADDI	-9
				JNEG	PATH_6_8
				COPY	0, 1
				ADDI	-12
				JNEG	PATH_9_11
				COPY	0, 1
				ADDI	-13
				JNEG	PATH_12_12
				COPY	0, 1
				ADDI	-14
				JNEG	PATH_13_13
				COPY	0, 1
				ADDI	-15
				JNEG	PATH_14_14
				COPY	0, 1
				ADDI	-16
				JNEG	PATH_15_15
				COPY	0, 1
				ADDI	-17
				JNEG	PATH_16_16
				COPY	0, 1
				ADDI	-19
				JNEG	PATH_17_18
				JUMP	PATH_19_19
				
PATH_1_5:		LOAD	CURPOS
				ADDI	1
				STORE	GOALMIN
				LOADI	5
				STORE	GOALMAX
				CALL	FOLLOWWALLTO
				LOAD	CURPOS
				ADDI	-6
				JZERO	TURN90
				RETURN
				
				
				
FOLLOWWALLTO:	CALL	CHECKGOALRANGE
				STORE	END
				JNEG	FWALLTOEND
				COPY	1, 0
				SUB		CURPOS
				CALL	FOLLOWWALLDIST
				LOAD	END
				STORE	CURPOS
				RETURN
				
FWALLTOEND:		LOAD	GOALMAX
				ADDI	1
				STORE	END
				SUB		CURPOS
				CALL	FOLLOWWALLDIST
				LOAD	END
				STORE	CURPOS
				RETURN
				
				
FOLLOWWALLDIST:	COPY	0, 1
				LOAD	SQUARESize
				MULTA	0, 0, 1
				STORE	FOLLOWDIST
				CALL	WALLFOLLOW
				RETURN
				
				

PATH_6_8:		LOAD	CURPOS
				ADDI	1
				STORE	GOALMIN
				LOADI	8
				STORE	GOALMAX
				CALL	FOLLOWWALLTO
				LOAD	CURPOS
				ADDI	-9
				JZERO	TURN90
				RETURN

PATH_9_11:		LOAD	CURPOS
				ADDI	1
				STORE	GOALMIN
				LOADI	11
				STORE	GOALMAX
				CALL	FOLLOWWALLTO
				LOAD	CURPOS
				ADDI	-12
				JZERO	TURN90
				RETURN

PATH_12_12:		LOAD	CURPOS
				ADDI	1
				STORE	GOALMIN
				LOADI	12
				STORE	GOALMAX
				CALL	FOLLOWWALLTO
				LOAD	CURPOS
				ADDI	-13
				JZERO	TURN90
				RETURN

PATH_13_13:		LOAD	CURPOS
				ADDI	1
				STORE	GOALMIN
				LOADI	13
				STORE	GOALMAX
				CALL	FOLLOWWALLTO
				RETURN
				
PATH_14_14:		LOADI	1
				CALL	GOSTRAIGHTSQ
				LOADI	15
				STORE	CURPOS
				JUMP	TURN-90
				RETURN


PATH_15_15:		LOADI	1
				CALL	GOSTRAIGHTSQ
				LOADI	16
				STORE	CURPOS
				JUMP	TURN-90
				RETURN
				
PATH_16_16:		LOADI	1
				CALL	GOSTRAIGHTSQ
				LOADI	17
				STORE	CURPOS
				RETURN
				
PATH_17_18:		LOAD	CURPOS
				ADDI	1
				STORE	GOALMIN
				LOADI	18
				STORE	GOALMAX
				CALL	FOLLOWWALLTO
				LOAD	CURPOS
				ADDI	-19
				JZERO	TURN90
				RETURN

PATH_19_19:		LOADI	1
				CALL	GOSTRAIGHTSQ
				CALL	TURN90
				CALL	GO1BACK
				LOADI	1
				STORE	CURPOS
				RETURN
				
END:			DW		0
GOALMIN:		DW		0
GOALMAX:		DW		19

CHECKGOALRANGE:	LOAD	POS1V
				JZERO	CGSKIP1
				LOAD	POS1
				CALL	CHECKINRANGE
				JPOS	RET
CGSKIP1:		LOAD	POS2V
				JZERO	CGSKIP2
				LOAD	POS2
				CALL	CHECKINRANGE
				JPOS	RET
CGSKIP2:		LOAD	POS3V
				JZERO	RET-1
				LOAD	POS3
				CALL	CHECKINRANGE
				RETURN

CHECKINRANGE:	COPY	1, 0
				SUB		GOALMIN
				JNEG	RET-1
				LOAD	GOALMAX
				SUB		0, 0, 1
				JNEG	RET-1
CHECKSKIP:		COPY	0, 1
				RETURN
				
				
				
CHECKONGOAL:	CALL	CHECKONE
				JPOS	RET
				CALL	CHECKTWO
				JPOS	RET
				CALL	CHECKTHREE
				RETURN
				
CHECKONE:		LOAD	POS1V
				JZERO	RET0
				LOAD	CURPOS
				SUB		POS1
				JZERO	MATCHONE
				JUMP	RET0
				
MATCHONE:		LOADI	0
				STORE	POS1V
				JUMP	RET1
				
				
CHECKTWO:		LOAD	POS2V
				JZERO	RET0
				LOAD	CURPOS
				SUB		POS2
				JZERO	MATCHTWO
				JUMP	RET0
				
MATCHTWO:		LOADI	0
				STORE	POS2V
				JUMP	RET1
				
				
CHECKTHREE:		LOAD	POS3V
				JZERO	RET0
				LOAD	CURPOS
				SUB		POS3
				JZERO	MATCHTHREE
				JUMP	RET0
				
MATCHTHREE:		LOADI	0
				STORE	POS3V
				JUMP	RET1

PATHCHECK:		LOAD	POS1V
				ADD		POS2V
				ADD		POS3V
				JZERO	RET
				JUMP	CHECKDONE
				
				
				
RET0:			LOADI	0
				RETURN
RET1:			LOADI	1
				RETURN	1
RET-1:			LOADI	-1
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
		LOADI	0
		OUT		BEEP
		OUT		TIMER
StopLoop2:
		IN		TIMER
		ADDI	-10 ;Wait 1 Sec
		JNEG	StopLoop2
		
		RETURN

GOSTRAIGHTSQ:	COPY	0, 1
				LOAD	SQUARESize
				MULTA	0, 0, 1
				STORE	FOLLOWDIST
				CALL	GOSTRAIGHTDIST
				RETURN
				
GO1BACK:		OUT		RESETPOS
				LOADI	SQUARESize
				STORE	FOLLOWDIST
GoBLOOP:		IN		THETA
				CALL 	CENTER
				Shift 	2
				STORE 	ANGLE
				ADD   	LOWSPEED
				CALL	LIMITWHEEL
				CALL	NEG
				OUT 	lvelcmd
				
				LOAD 	Zero
				SUB   	ANGLE
				ADD   	LOWSPEED
				CALL	LIMITWHEEL
				CALL	NEG
				OUT   	rvelcmd		
				
				IN		THETA
				OUT 	SSEG2
				IN		XPOS
				OUT		SSEG1
				ADD 	FOLLOWDIST
				JPOS 	GoBLOOP
				LOADI	0
				OUT 	lvelcmd
				OUT 	rvelcmd
				RETURN
				
NEG:			STORE 	TEMP
				LOADI	0
				SUB 	TEMP
				RETURN
LOWSPEED:		DW		150
ANGLE:			DW		0
GOSTRAIGHTDIST:	OUT		RESETPOS
LOOPSTRAIGHT:	IN		THETA
				CALL 	CENTER
				Shift 	2
				STORE 	ANGLE
				ADD   	LOWSPEED
				CALL	LIMITWHEEL
				OUT 	lvelcmd
				
				LOAD 	Zero
				SUB   	ANGLE
				ADD   	LOWSPEED
				CALL	LIMITWHEEL
				OUT   	rvelcmd		
				
				IN		THETA
				OUT 	SSEG2
				IN		XPOS
				OUT		SSEG1
				SUB 	FOLLOWDIST
				JNEG 	GOSTRAIGHTDIST
				LOADI	0
				OUT 	lvelcmd
				OUT 	rvelcmd
				RETURN
				
DEFAULTSPEED:	DW		350
CURW: 	dw 0000
CURF: 	dw 0000 
ERRW: 	dw 0000
WML:  	dw 0000
WMR:  	dw 0000
WDIST: 	dw &Hd5
FOLLOWDIST:		DW		100
WALLFOLLOW: 	OUT		RESETPOS
				LOADI	&B00101000		;Turn on Sensors 3 and 5
				OUT  	SONAREN
WALLLOOP:		IN 		DIST5
				STORE 	CURW
				SUB 	WDIST
				OUT 	SSEG1
				STORE 	ERRW
				CALL 	WCORR
				LOAD 	WML 
				OUT 	LVELCMD
				LOAD 	WMR
				OUT 	RVELCMD
				IN		XPOS
				OUT		LCD
				SUB		FOLLOWDIST
				ADDI	140				;STOP 140 SHORT. NEED TO GET BETTER WAY TO STOP OVERSHOOT
				JNEG	WALLLOOP
				LOADI	0
				OUT 	LVELCMD
				OUT 	RVELCMD
				RETURN
				
WCORR:			
				LOADI	-50
				STORE	LMIN
				LOADI	50
				STORE	LMAX
				LOAD  	ERRW 
				CALL	LIMIT
				STORE	ERRW
				
				LOAD 	DEFAULTSPEED	
				ADD		ERRW
				CALL	LIMITWHEEL
				STORE 	WML
				
				LOAD  	DEFAULTSPEED
				SUB 	ERRW
				CALL	LIMITWHEEL
				STORE 	WMR
				RETURN


LIMITWHEEL:		STORE	TEMP
				LOADI	100
				STORE	LMIN
				LOADI	500
				STORE	LMAX
				LOAD	TEMP
				CALL	LIMIT
				RETURN

LMIN:			DW		0
LMAX:			DW		360
LIMIT:			STORE   TEMP
				SUB 	LMIN
				JNEG	RETMIN
				LOAD	TEMP
				SUB		LMAX
				JPOS	RETMAX
				LOAD	TEMP
				return 
RETMIN:			LOAD	LMIN
				return
RETMAX:			LOAD	LMAX
				return
	
ABS:		JPOS	RET		;Function ABS uses R6&R0. R0 = abs(R0)
			COPY	6, 0
			LOADI	0
			SUBA	0, 0, 6
RET:		RETURN

TURN90:	OUT    RESETPOS
			LOADI	90
			CALL	ROTATE
			RETURN
			
TURN-90:	OUT    RESETPOS
			LOADI	-90
			CALL	ROTATE
			RETURN
			

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
SQUARESize:	  	DW	586
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
