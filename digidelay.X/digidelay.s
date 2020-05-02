;==============================================================================
;	Digital Delay - MAIN CODE
;==============================================================================
;	April 2016 - Carunchoso, waiting for Paula's e-bike!
;	From earlier work on the Idea Platform
;	Copyright 2016 Tom Wiltshire for Electric Druid.
;==============================================================================
; 25 Apr 2016: Got delay working with dsPIC's internal RAM. Also have CVs
; working and Level control.
; 10 May 2016: Working with 2 x external SRAM chips. Added fade in/out and
; momentary/latching footswitch functions, and selectable delay tails.
; 12 Nov 2016: Starting with this version to make code for the PCB. I've
; changed some pin assignments around since the stripboard prototype.
; 22 Nov 2016: Rewritten audio code to use block processing and increased
; block size to 64 samples. Rewritten filter routines to use 32-bit accumulators
; and fraction saving to reduce noise.
; 05 May 2020: Change tap behavior by Haroldo Gamal. The tap tempo  is computed  
; every time user press tap temp button. If the time is superior to the maximum
; time this tap is ignored.
	
; 33FJ dsPIC with on-chip Audio DAC
	.equ __33FJ64GP802, 1
	.include "p33FJ64GP802.inc"

;==============================================================================
;	Configuration bits for 33FJ64GP802
;==============================================================================
	; Boot segment
	config __FBS, RBS_NO_RAM & BSS_NO_FLASH & BWRP_WRPROTECT_OFF
	; Secure segment
	config __FSS, RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF
	; Code Protection	
	config __FGS, GSS_OFF & GCP_OFF & GWRP_OFF
	; Oscillator (7.37MHz int FRC)
	config __FOSCSEL, FNOSC_FRCPLL & IESO_OFF
	config __FOSC, FCKSM_CSDCMD & IOL1WAY_OFF & OSCIOFNC_ON
	; Watchdog
	config __FWDT, FWDTEN_OFF & WINDIS_OFF
	; Power up options (16ms)
	config __FPOR, ALTI2C_ON & FPWRT_PWR16
	; ICSP and JTAG
	config __FICD, ICS_PGD2 & JTAGEN_OFF


;==============================================================================
;	Program Specific Constants (literals used in code)
;==============================================================================

	; Indicator LEDs and Sync output (Port A)
	.equiv	LED0,	 	2			; Effect On/Off
	.equiv	LED1,	 	3			; Tempo
	.equiv	SYNC_OUT,	4			; Sync pulse output

	; Push buttons/footswitches (Port B)
	.equiv	BUTTON0,	 13			; Effect On/Off
	.equiv	BUTTON1,	 12			; Tap Tempo
	.equiv	BUTTON2,	 11			; Tails On/Off
	
	; I still have RB9 & RB10 free!! What should I do with it?!
	.equiv	UNUSED_OUT0,	 9			; Unused output
	.equiv	UNUSED_OUT1,	 10			; Unused output
	
	; System Flag byte definitions (FLAGS) 1 = Yes
	.equiv	INT_USING_BUFF1, 0		; Interrupt is read/writing Buffer1
	.equiv	BUFF_PROCESS_REQ, 1		; We need to process the In/Out buffers
	.equiv	NEW_CVS, 2				; We have a new set of CV values
	.equiv	DELAY_ON, 3				; Effect is switched in, not bypassed
	.equiv	TAILS, 4				; We maintain delay tails
	.equiv	FADING, 5				; We're fading in or out
	.equiv	TIMER_ON, 6				; We're timing a button press
	.equiv	LONG_PRESS, 7			; The button press took aaaggggeess!
	.equiv	TAP_TIMER_ON, 8			; We're timing between taps
	.equiv	SECOND_TAP, 9			; This is the second tap
	.equiv	DLY_XFADE_ON, 10		; We're Xfading two Read positions
	.equiv	DLY_XFADE_NEXT, 11		; There's another Xfade to do after this one
	
	; Delay RAM ~chipselect pins (Both Port B)
	.equiv	CS0_INHIBIT, 4
	.equiv	CS1_INHIBIT, 5
	
	; Delay Time Crossfade rate (Freq inc for 16-bit accumulator @ 32.4KHz)
	.equiv	DLY_XFADE_RATE,	16		; 64 mSecs
	; Fade time in seconds ~= 2 / DLY_XFADE_RATE
	; It is required that this rate gives a whole number of blocks, since the
	; overflow check is only done at the end of a block, in order to avoid
	; having conditionals inside DelayXfadeLoop.
	; E.g. 32768/DLY_XFADE_RATE *MUST* be divisable by 64.

	; Fade In/Out rate (Freq inc for 16-bit accumulator @ 32.4KHz)
	.equiv	FADE_RATE,	16			; 62 mSecs
	; Fade time in seconds ~= 1 / FADE_RATE

	; Button Press countdown timer - is this a long press?
	.equiv	TIMER_COUNT, 250		; 0.5 Secs
	; Button press time in seconds ~= TIMER_COUNT / 500
	; If this timer times out, the press is regarded as 'long' and the
	; LONG_PRESS flag is set.
	
	; Sync Pulse countdown timer
	.equiv	SYNC_COUNT, 4			; 8 msecs
	; Sync Pulse length in milliseconds ~= SYNC_COUNT * 2

;==============================================================================
;	Global Declarations:
;==============================================================================

	.global __reset					; The label for the first line of code
	.global __T2Interrupt			; Timer2 "Master Clock" interrupt
	;.global __DAC1LInterrupt		; DAC left channel

;==============================================================================
;	Uninitialized variables in Near data memory (Lower 8Kb of RAM)
;==============================================================================

.section mainstorage, bss, near	;, address(0x900)

; Main System flags
FLAGS:			.space 2			; See 'Constants' section for details

; Audio buffer positions
IN_LEFT:		.space 2			; Used to pass buffer address to
OUT_LEFT:		.space 2			; Interrupt routine
BUFFER_POS:		.space 2			; Position within buffer 0-30

; CV reading
CV_CHANNEL:		.space 2			; Counts through the CV sequence 0-18

; Delay memory variables
WRITE_BLOCK:	.space 2			; Where are we in the Delay RAM?
READ_BLOCK:		.space 2
NEW_RD_BLOCK:	.space 2			; This is the new required Read block
									; when the Delay Time is changed
NEXT_RD_BLOCK:	.space 2			; This is used if Delay Time changes
									; before we've cross-faded the two above.
									
; These block numbers are converted into a 24-bit address:
ADDR_HI:		.space 2			; Actual address sent to SRAM
ADDR_LO:		.space 2

; Input button debounce variables (From PORT B)
DEBOUNCE_LO:	.space 2			; 2-bit vertical debounce counter
DEBOUNCE_HI:	.space 2
IN_STATES:		.space 2			; Current debounced state
IN_CHANGES:		.space 2			; Have any buttons changed state?

; The Delay Time changes cross-fader
DLY_XFADE:		.space 2			; READ_BLOCK -> NEW_RD_BLCK
		
; The button press timer
BUTTON_TIMER:	.space 2			; Countdown timer
			
; The Tap Tempo timer
TAP_TIMER:		.space 2			; Counts in blocks of 64 samples (~2ms)	
		
; The Tempo LED
TEMPO:			.space 2			; The required tempo (in blocks)
TEMPO_TIMER:	.space 2			; Countdown timer
BRIGHTNESS:		.space 2			; The Tempo LED's brightness
BRIGHT_DEC:		.space 2			; The rate of brightness decrement
PWM_COUNT:		.space 2			; The LED PWM counter

; This is used to determine if the Delay Time has actually changed or not
PREV_TEMPO:	.space 2
	
; The Sync Pulse output
SYNC_TIMER:		.space 2			; Countdown timer


; Storage space for CV input values
.section cvvalues
RAW_CV0:		.space 2			; AN0
RAW_CV1:		.space 2			; AN1 
RAW_CV2:		.space 2			; AN2
RAW_CV3:		.space 2			; AN3
RAW_CV4:		.space 2			; AN4	(and AN5 is Audio Input)

; The RAW_CVs are filtered by 2 x one-pole IIRs
; The first pole
FILTER0_L:		.space 2
FILTER0_H:		.space 2
FILTER1_L:		.space 2
FILTER1_H:		.space 2
FILTER2_L:		.space 2
FILTER2_H:		.space 2
FILTER3_L:		.space 2
FILTER3_H:		.space 2
FILTER4_L:		.space 2
FILTER4_H:		.space 2

; The second pole - these are the values we actually use, although
; I only use the high word.
LEVEL_CV_L:		.space 2
LEVEL_CV:		.space 2
REPEATS_CV_L:	.space 2
REPEATS_CV:		.space 2
DELAY_CV_L:		.space 2
DELAY_CV:		.space 2
LOWPASS_CV_L:	.space 2
LOWPASS_CV:		.space 2
HIGHPASS_CV_L:	.space 2
HIGHPASS_CV:	.space 2

; A buffer of FADE_CV values for the In/Out switching.
FADE_BUF:		.space 128			; 64 values of FADE_CV	


; Feedback filtering
;--------------------
; There are three filters (DC blocker, LPF, HPF) with their various
; coefficients and stored values.
; These are all in order here to allow easy pointer incrementing

; The coefficients (Y memory)
.section coeffs, ymemory
; DC blocking filter
DC_A1:			.space 2			; Coefficients for DC blocker
DC_A0:			.space 2
DC_B1:			.space 2
; LP feedback filter
LP_B1:			.space 2
LP_A0:			.space 2
; We turn the LPF into a LP shelving by mixing the input in
LPS_X:			.space 2			; The dry path
LPS_Y:			.space 2			; The LPF 'wet' path
; HP feedback filter coefficients
HP_B1:			.space 2
HP_A0:			.space 2
; We turn the HPF into a HP shelving by mixing the input in
HPS_X:			.space 2			; The LPF 'wet' path
; The high pass case is simpler and only needs a single coefficient

; The Storage locations (X Memory)
.section storage, xmemory
DC_IN:			.space 2		; DC blocker previous input
DC_OUT_HI:		.space 2		; DC blocker previous output
DC_OUT_LO:		.space 2
LPF_OUT_HI:		.space 2		; Lowpass filter previous output
LPF_OUT_LO:		.space 2
HPF_OUT_HI:		.space 2		; Highpass filter previous output
HPF_OUT_LO:		.space 2
			
; Temporary crossfade buffers for delay time changes
.section xfade, xmemory
XFADE_X:		.space 128			; We interpolate between these two buffers
XFADE_Y:		.space 128
		
		
.section audiobuffers, address(0x1800)
; The audio input/output buffers
IN_L_BUF0:		.space 128			; Left First Input Buffer
IN_L_BUF1:		.space 128			; Left Second Input Buffer
OUT_L_BUF0:		.space 128			; Left First Output Buffer
OUT_L_BUF1:		.space 128			; Left Second Output Buffer
; These could be duplicated for the right channel (one day)
		
; The audio buffers for data on its way to/from the SRAM
SRAM_IN_BUF:		.space 128		; SRAM Input Buffer (Going in)
SRAM_OUT_BUF:		.space 128		; SRAM Output Buffer
; The output buffer is filled from the two crossfade buffers (XFADE_X/Y)


;==============================================================================
;	Mental Mickey's MiniMacros
;==============================================================================

; "Move Constant" Macro
.macro	movc	C, L				; Puts constant C into location L (via WREG)
	mov	\C, W0
	mov	W0, \L
.endm


;==============================================================================
;	Code Section in Program Memory
;==============================================================================
.section maincode code				; Start of Code section

__reset:
	mov		#__SP_init, W15			; Initalize the Stack Pointer
	mov		#__SPLIM_init, W0		; Initialize the Stack Pointer Limit Register
	mov		W0, SPLIM
	nop								; Add NOP to follow SPLIM initialization
        
	; Initialization of W registers to 0x0000
	clr		W0
	mov		W0, W14
	repeat	#12
	mov		W0, [++W14]
	clr		W14

;=======================================================
;	Set up Oscillator, PLL, DAC and all Pre/Postscalers
; We use the internal FRC osc at 7.37MHz
;=======================================================

	movc	#0x0000, CLKDIV
	movc	#0x0022, PLLFBD
	; FRC Internal Osc			= 7.3728MHz
	; PLL Prescaler (N1) = /2	= 3.6864MHz
	; PLL Multiplier  = x36		= 132.7104MHz (Fvco)
	; PLL Postscaler(N2) = /2	= 66.3552MHz (Fosc, for an Fcy of 33.1776MHz)

	; Fvco is  used as ACLK		= 132.7104MHz
	; DAC Prescaler = /16		= 8.2944MHz
	; DAC Clock division /256	= 32.4KHz sample output rate

	; Disable auxillary oscillator and use primary osc instead
	movc	#0x0700, ACLKCON
	; Aux Clk divide = /1
	; Primary osc is aux clock source

; DEBUG - Easier to work with if things are closer to the expected values
;	movc	#127, OSCTUN			; My chip runs a bit fast, so -0.375%


;=======================================================
;	Set up Timer 2
;=======================================================

	movc	#0x0010, T2CON			; Timer off, \8 prescale, 16-bit, Clock=Fcy
	; Timer 2 provides the 64.8KHz clock that runs everything, and
	; which triggers the ADC.
	; On every other interrupt, it feeds a sample to the DAC.
	; In between times, it samples the CV inputs.
	; This is more obvious if you look at the interrupt routine itself, and
	; the CVReadSequence.
	clr		TMR2
	movc	#63, PR2				; Match when timer=63, give /64 operation
	; With the 4.1472MHz Timer2 clock rate, this gives a 64.8KHz 
	; "Master clock" which generates the interrupts we need.

	; Set up Timer2 Interrupt
	movc	#0x7444, IPC1			; Make interrupt highest priority
	bclr	IFS0, #T2IF				; Clear interrupt flag
	bset	IEC0, #T2IE				; Enable interrupt

;=======================================================
;	Set up Audio DAC output
;=======================================================

	; Disable nested interrupts
	bset	INTCON1, #NSTDIS

	; Set up DAC for output sample-rate of 32.4KHz
	movc	#0x010F, DAC1CON		; Signed Int, Prescale /16
	movc	#0, DAC1DFLT			; Default data = zero midpoint value
	movc	#0x8400, DAC1STAT		; Enable L only, Interrupt on FIFO empty

	; No DAC interrupts are required. Timer2 provides the master clock


;=======================================================
;	Setup Ports for Input/Output
;=======================================================
	movc	#0x0003, TRISA			; RA<0:1> Input, others output
	clr		LATA					; Clear port A bits

	movc	#0x384F, TRISB			; RB<0:3>, RB<6>, RB<11:13> Input
	clr		LATB					; Clear port B bits

	; RA0, RA1, RB0, RB1, RB2, RB3 are the ADC input channels AN0-AN5

	; Disable SRAM chips until we need them
	bset	LATB, #CS0_INHIBIT		; ~CS0 High
	bset	LATB, #CS1_INHIBIT		; ~CS1 High


;=======================================================
;	Set up DSP features
;=======================================================

	bclr	CORCON, #0				; IF bit: Fractional Multiply

;=======================================================
;	Set up ADC for single-channel operation
;=======================================================

	;ADC1 Setup
	mov 	#0x14E0, W0				; Unsigned Int, 12-bit
	mov 	W0, AD1CON1
	mov 	#0x0000, W0				; No Scan, AVDD & AVSS
	mov 	W0, AD1CON2
	mov 	#0x0606, W0				; Tad=Tcy*(ADCS+1)= (1/33M)*6 = 180ns)
	mov 	W0, AD1CON3				; 6*Tad sample window
	mov 	#0x1, W0				; Start CHS0 on AN1
	mov 	W0, AD1CHS0
	mov 	#0x0, W0				; CHS123 unused in 12-bit mode
	mov 	W0, AD1CHS123
	mov 	#0xFFC0, W0				; AN0-AN5 as Analog Input
	mov 	W0, AD1PCFGL
	mov		#0x0000, W0				; No Scan AN0-AN5
	mov		W0, AD1CSSL
	bclr	IFS0, #AD1IF			; Clear the A/D interrupt flag bit
	bclr	IEC0, #AD1IE			; Disable A/D interrupt
	bset	AD1CON1, #ADON			; Turn on the A/D converter
	bset	AD1CON1, #SAMP			; Start inital conversion

;=======================================================
;	Set up SPI to talk to Serial RAMs
;=======================================================

	; Set up the remappable peripheral pins
	movc	#0x0806, RPINR20		; SCLK1 = RP8, SDI1 = RP6
	movc	#0x0700, RPOR3			; RP7 = SDO1, RP6 = N/A
	movc	#0x0008, RPOR4			; RP9 = N/A, RP8 = SCLK1

	; RB4 and RB5 are the chip selects (these are 'manual')

	; Set up the SPI1 module
	mov		#0x0000, W0				; Module off, clear flags
	mov		W0, SPI1STAT
	mov		#0x053E, W0				; SCK+SDO enabled, Word mode, Middle of data
	mov		W0, SPI1CON1			; SPRE=/1, PPRE = /4
	clr		SPI1CON2				; Disable SPI frames

	; Ok, start it up
	bset	SPI1STAT, #SPIEN


;=======================================================
;	Set up variables
;=======================================================

	; Clear the main system flags
	clr		FLAGS					; Int is using Buffer0

	; Ensure the LEDs/outputs match the associated flags
	bclr	LATA, #LED0
	bclr	LATA, #LED1
	bclr	LATA, #SYNC_OUT

	; Wipe the IN/OUT audio sample buffers
	mov		#IN_L_BUF0, W0
	repeat	#255					; 4x64 words (all Left channel buffers)
	clr		[W0++]
	
	; Wipe the IN/OUT delay line buffers
	mov		#SRAM_IN_BUF, W0
	repeat	#127					; 2x64 words
	clr		[W0++]
	
	; Set up the initial buffer addresses
	movc	#IN_L_BUF0, IN_LEFT		; Int is using buffer 0
	movc	#OUT_L_BUF0, OUT_LEFT
	clr		BUFFER_POS				; We start at zero in the buffer

	; Set up working variables
	clr		CV_CHANNEL				; Audio channel
	clr		BUTTON_TIMER			; Button timer reset and ready to go
	clr		DLY_XFADE				; Crossfade starts at zero
	
	; Set up the circular delay buffer
	movc	#0x0000, READ_BLOCK		; We start on Block 0
	movc	#0x3FE0, WRITE_BLOCK	; 1 second delay
	clr		ADDR_LO					; Working storage for SRAM address
	clr		ADDR_HI

	; Set the tempo to the difference between the pointers
	mov		#0x1FF, W0				; Set the starting tempo (as above /32)
	mov		W0, TEMPO
	mov		W0, TEMPO_TIMER			; Put the tempo in the counter
	mov		W0, PREV_TEMPO			; CV is unchanged
	mov		#0xFFFF, W0
	mov		W0, BRIGHTNESS			; Set the pulse brightness

	; Wipe the CV filters
	mov		#FILTER0_L, W0
	repeat	#19						; 20 words ( 2 x five 32-bit filters)
	clr		[W0++]

	; Set up the DC blocking filter (0.995)
	clr		DC_IN
	clr		DC_OUT_HI
	clr		DC_OUT_LO
	movc	#32767, DC_A1
	movc	#32767, DC_A0
	movc	#164, DC_B1

	; Set up a default state for the Feedback filters
	clr		LPF_OUT_HI
	clr		LPF_OUT_LO
	movc	#1, LP_B1
	movc	#65535, LP_A0
	
	movc	#9601, LPS_X		; Shelving parameters
	movc	#23167, LPS_Y
	
	clr		HPF_OUT_HI
	clr		HPF_OUT_LO
	movc	#65535, HP_B1
	movc	#1, HP_A0
	movc	#23167, HPS_X		; Shelving parameter
	
	clr		TAP_TIMER

;=======================================================
;	Done with setup, so start everything up
;=======================================================

	; Start the DAC running
	mov		#0, W0
	mov		W0, DAC1LDAT
	mov		W0, DAC1LDAT			; A couple of dummy samples to start with
	bset	DAC1CON, #DACEN			; Enable DAC

	; Start Timer2 providing interrupts
	bset	T2CON, #TON				; Enable Timer 2

	
;==============================================================================
;	Main Program Code starts here
;==============================================================================
Main:
	; Do we need to do a RAM transfer?
	btss	FLAGS, #BUFF_PROCESS_REQ
	goto	ReciteMantra			; No, so skip

	; We've got a buffer full of input data to send to the RAM
	call	ProcessBuffers
	
	; Do anything else that needs to happen at the 2msec block rate
	call	TempoMarking			; Tempo LED and Sync output pulse
	btsc	FLAGS, #TIMER_ON		; Are we timing a button press?
	call	ButtonTimer				; Yes, we are

	; Have we got new CV values to deal with?
	btss	FLAGS, #NEW_CVS
	goto	ReciteMantra			; No, so skip
	; Yes, so let's deal with the CV inputs and the buttons
	call	ProcessCVs
	call	ProcessButtons
	; All done!

; Recite Sacred Mantra into WREG in order to purify our code
; and invoke the blessings of Chenrezig, Buddha of compassion.
ReciteMantra:
	mov.b	#'O', W0
	mov.b	#'M', W0

	mov.b	#'M', W0
	mov.b	#'A', W0
	mov.b	#'N', W0
	mov.b	#'I', W0

	mov.b	#'P', W0
	mov.b	#'A', W0
	mov.b	#'D', W0
	mov.b	#'M', W0
	mov.b	#'E', W0

	mov.b	#'H', W0
	mov.b	#'U', W0	; A homage to Chet Wood and Sequential Circuits
	mov.b	#'M', W0	; Those crazy hippies were on to something!

	; Go back and do it all again
	goto	Main


;=======================================================
;	We need to process the audio buffers
;=======================================================
; The current IN_BUFx is full, and the current OUT_BUFx is empty.
; The interrupt has switched over to the other set (ping-pong), so now we
; have to generate a new set of data in the buffers it's not using.
; There are several steps:
;	Get the SRAM output
;		This could be crossfaded between two read positions if the delay is
;		changing, or not if not.
;	Process the Input
;		We use the contents of the IN_BUFx and the SRAM_OUT_BUF to
;		give us new SRAM_IN_BUF contents. We take the input, and add delay
;		feedback, basically.
;	Do the filtering
;		We need to block DC bias in the input and we also add high and low
;		shelving tone filters.
;	Process the Output
;		The delay output in SRM_OUT_BUF needs its level setting by LEVEL_CV.
;		This gives us new OUT_BUFx contents.
; Phew!
	
ProcessBuffers:
	bclr	FLAGS, #BUFF_PROCESS_REQ; Clear flag
	
;=======================================================
;	Get the RAM Output (output from the delayline)
;=======================================================
; Two options:
;	CrossfadeRAMOutput, for when delaytime is changing
;	SimpleRAMOutput, for when it's not.
	
GetRAMOutput:
	; Is the delaytime changing? Are we crossfading?
	btss	FLAGS, #DLY_XFADE_ON
	goto	SimpleRAMOutput		; No Xfade. We can use the output directly.


;	Crossfade RAM Output - Delay time is changing
;=======================================================	
CrossfadeRAMOutput:
	; First we get the latest output from the delayline into SRAM_OUT_BUF
	mov		READ_BLOCK, W0
	mov		#XFADE_X, W1
	call	FetchFromRAM			; Get READ_BLOCK into X buffer
	mov		NEW_RD_BLOCK, W0
	mov		#XFADE_Y, W1
	call	FetchFromRAM			; Get NEW_RD_BLOCK into Y buffer
	
	; Set up for the crossfade loop
	mov		#DLY_XFADE_RATE, W3		; Crossfade increment value
	mov		XFADE_X, W4				; First sample from READ_BLOCK
	mov		XFADE_Y, W5				; First sample from NEW_RD_BLOCK
	; W6 is the inverted index (1-index)
	mov		DLY_XFADE, W7			; Crossfade index 0-32767
	mov		#XFADE_X+2, W8			; Pointer to X buffer (READ_BLOCK)
	mov		#XFADE_Y+2, W9			; Pointer to Y buffer (NEW_RD_BLOCK)
	mov		#SRAM_OUT_BUF, W11		; Pointer to delay output buffer
	mov		#0x7FFF, W12			; Index inversion mask
	; Generate the inverted index
	xor		W7, W12, W6				; Crossfade (1-index)
	
; Loop through the buffers doing the crossfade and storing the result
CrossfadeLoop:
	do		#63, EndCrossfadeLoop-2	; 64 Words
	mpy		W4*W6, A, [W8]+=2, W4	; A = (X * (1-index))
	mac		W5*W7, A, [W9]+=2, W5	; A += (Y * index)
	sac.r	A, [W11++]				; Store result in the SRAM output buffer
	; Increment the crossfade
	add		W7, W3, W7				; Increment the index
	xor		W7, W12, W6				; Generate new (1-index)
EndCrossfadeLoop:
; At this point the SRAM_OUT_BUF has crossfaded data from the two read points
	mov		W7, DLY_XFADE			; Store state for next time
	; Has the crossfade finished?
	btss	W7, #15					; It's finished
	goto	FadeGeneration
	
CrossfadeComplete:
	clr		DLY_XFADE				; Reset to beginning for next time
	mov		NEW_RD_BLOCK, W0		; Get the next pointer
	mov		W0, READ_BLOCK
	bclr	FLAGS, #DLY_XFADE_ON	; Turn crossfade off
	
	; Do we have another crossfade to do?
	btss	FLAGS, #DLY_XFADE_NEXT
	goto	FadeGeneration
	
NextCrossfade:
	; Yes, we've got another crossfade lined up
	mov		NEXT_RD_BLOCK, W0		; Get the next pointer
	mov		W0, NEW_RD_BLOCK
	bset	FLAGS, #DLY_XFADE_ON	; Turn crossfade on
	bclr	FLAGS, #DLY_XFADE_NEXT	; No next crossfade any more
	goto	FadeGeneration

;	Simple RAM Output - Delay time is unchanging
;=======================================================	
SimpleRAMOutput:
	mov		READ_BLOCK, W0
	mov		#SRAM_OUT_BUF, W1
	call	FetchFromRAM			; Put RAM contents direct into buffer


;=======================================================
;	Fade In/Out Generation
;=======================================================
; This routine fills the FADE_BUFF with 'FADE CV' values
; that are used to soft-fade the effect In/Out switching.
FadeGeneration:
	; Set up regsters for fade generation
	mov		#FADE_BUF, W8			; Pointer to the FADE CV buffer
	mov		FADE_BUF+126, W9		; Get the previous state of the fade
	mov		#FADE_RATE, W10
FadeLoop:
	do		#63, EndFadeLoop-2		; 64 Words
	; Are we actually fading at all anyway?
	btss	FLAGS, #FADING	
	goto	FadeFinished
	; Add the FADE RATE to the FADE CV
	add		W9, W10, W9
	; Has it overflowed?
	btss	W9, #15
	goto	StoreFade				; No, so store the result
FadeFinished:
	bclr	FLAGS, #FADING			; Yes, it's overflowed
	mov		#32767, W9				; Fade over - use maximum level
StoreFade:
	mov		W9, [W8++]				; Store result in Fade CV buffer
EndFadeLoop:

	
;=======================================================
;	Process Input
;=======================================================
; Process the contents of the IN_BUFx and the SRAM_OUT_BUF.
; We use the delayline output as feedback to add to the input and
; generate the samples going into the delayline.
ProcessInput:
; Which buffers need processing?
	; Set up a pointer to the correct Input buffer
	mov		#IN_L_BUF0, W1			; Assume we use Buffer0
	btss	FLAGS, #INT_USING_BUFF1	; Is Interrupt is using Buffer1?
	mov		#IN_L_BUF1, W1			; No, so we use Buffer1
	
	mov		#SRAM_IN_BUF, W12		; Pointer to delay input buffer
	mov		#SRAM_OUT_BUF, W13		; Pointer to delay output buffer
	; Set up Fade CV pointer and inversion mask
	mov		#FADE_BUF, W8			; Pointer to Fade CV buffer
	mov		#0x0, W11				; Delay on = no inversion of Fade CV
	btss	FLAGS, #DELAY_ON	
	mov		#0x7FFF, W11			; Delay off = invert Fade CV
	
; Set up a loop to process all the samples in the buffer
ProcessInputLoop:
	do		#63, EndProcessInputLoop-2	; 64 Words
	; Get the input sample (IN_BUFx)
	mov		[W1++], W6
	; Get the output sample from the delayline (SRAM_OUT_BUF)
	mov		[W13++], W4
	; Input into the delay line is only turned on when DELAY_ON is true
	mov		[W8++], W5				; Get Fade CV from buffer
	xor		W5, W11, W5				; Invert it if required
	mpy		W6*W5, A				; A = (Input * Fade CV)
	mov		REPEATS_CV, W5
	mac		W4*W5, A				; + (Delay * Repeat CV)
	; Put incoming sample(Input) + echos into the SRAM_IN_BUF
	; (It'll be processed further by the filters from there)
	sac.r	A, [W12++]				; Store the sample
EndProcessInputLoop:

	
;=======================================================
;	Process Filters
;=======================================================
ProcessFilters:	
; The mixture of the incoming audio and the delay feedback are fed
; through a series of digital filters.
; First up is a DC blocker, then the LP shelving filter, then the
; HP shelving filter. The coefficients of the last two are under
; user control.
	
SetupDCBlocker:
	mov		DC_IN, W4				; Get previous input from last block
	mov		DC_A1, W7				; Current coefficient
	mov		#SRAM_IN_BUF, W8		; Pointer to sample buffer
	mov		#DC_A0, W10				; Pointer to next coefficient
	; Load the accumulator with the state from the previous block
	mov		DC_OUT_HI, W5			; Our first feedback signal
	lac		W5, A					
	mov		DC_OUT_LO, W0
	mov		W0, ACCAL				
ProcessDCBlockerLoop:
	do		#63, EndProcessDCBlockerLoop-2		; 64 Words
	msc		W4*W7, A, [W8], W4, [W10]+=2, W7	; Subtract prev DC_IN signal
	mac		W4*W7, A, [W10]-=4, W7				; Add IN signal
	msc		W5*W7, A, [W10]+=2, W7				; Subtract prev DC_OUT signal
	sac.r	a, W5
	mov		W5, [W8++]							; Put output into SRAM_IN_BUF
EndProcessDCBlockerLoop:
	; Store the DC_OUT value for the next block
	sac		A, W0
	mov		W0, DC_OUT_HI		; ACCAH -> DC_OUT_HI
	mov		ACCAL, W0
	mov		W0, DC_OUT_LO		; ACCAL -> DC_OUT_LO
	; Store the DC_IN value too
	mov		W4, DC_IN
	

SetupLPFilter:
	mov		LP_B1, W7				; Current coefficient
	mov		#SRAM_IN_BUF, W8		; Pointer to sample buffer
	mov		#LP_A0, W10				; Pointer to next coefficient
	; Load the accumulator with the state from the previous block
	mov		LPF_OUT_HI, W5			; Our first feedback signal
	lac		W5, A					
	mov		LPF_OUT_LO, W0
	mov		W0, ACCAL	
ProcessLPFilterLoop:
	do		#63, EndProcessLPFilterLoop-2		; 64 Words
	; Lowpass filter
	msc		W5*W7, A, [W8], W4, [W10]+=2, W7	; Last output * B1
	mac		W4*W7, A, [W10]+=2, W7				; Input * A0
	; Lowpass shelving
	sac.r	A, W5								; LPF output (Wet)
	mpy		W4*W7, B, [W10]-=6, W7				; Dry x LPS_X
	mac		W5*W7, B, [W10]+=2, W7				; Wet x LPS_Y
	sac.r	B, W6
	mov		W6, [W8++]							; Put output into SRAM_IN_BUF
EndProcessLPFilterLoop:
	; Store the LPF_OUT value for the next block
	sac		A, W0
	mov		W0, LPF_OUT_HI		; ACCAH -> LPF_OUT_HI
	mov		ACCAL, W0
	mov		W0, LPF_OUT_LO		; ACCAL -> LPF_OUT_LO
	
	
SetupHPFilter:
	mov		HP_B1, W7				; Current coefficient
	mov		#SRAM_IN_BUF, W8		; Pointer to sample buffer
	mov		#HP_A0, W10				; Pointer to next coefficient
	; Load the accumulator with the state from the previous block
	mov		HPF_OUT_HI, W5			; Our first feedback signal
	lac		W5, A					
	mov		HPF_OUT_LO, W0
	mov		W0, ACCAL	
ProcessHPFilterLoop:
	do		#63, EndProcessHPFilterLoop-2		; 64 Words	
	; Highpass filter (actually it's another lowpass!)
	msc		W5*W7, A, [W8], W4, [W10]+=2, W7	; Last output * B0
	mac		W4*W7, A, [W10]-=4, W7				; Input * A0
	; Highpass shelving (Take LP off input signal)
	sac.r	A, W5								; LPF output
	lac		W4, B								; Input
	msc		W5*W7, B, [W10]+=2, W7				; Take away LP
	sac.r	B, [W8++]							; Put output into SRAM_IN_BUF
EndProcessHPFilterLoop:
	; Store the HPF_OUT value for the next block
	sac		A, W0
	mov		W0, HPF_OUT_HI		; ACCAH -> HPF_OUT_HI
	mov		ACCAL, W0
	mov		W0, HPF_OUT_LO		; ACCAL -> HPF_OUT_LO
	
	
;=======================================================
;	Process Output
;=======================================================
; Process the contents of the SRAM_OUT_BUF, the delayline output.
; This generates the signal for the Output buffer.
ProcessOutput:
	; Which buffer needs processing?
	; Set up a pointer to the correct Output buffer
	mov		#OUT_L_BUF0, W2
	btss	FLAGS, #INT_USING_BUFF1
	mov		#OUT_L_BUF1, W2
	mov		#SRAM_OUT_BUF, W13		; Pointer to delay output buffer
	; Set up Fade CV pointer and inversion mask
	mov		#FADE_BUF, W8			; Pointer to Fade CV buffer
	mov		#0x0, W11				; Delay on = no inversion of Fade CV
	btss	FLAGS, #DELAY_ON	
	mov		#0x7FFF, W11			; Delay off = invert Fade CV
ProcessOutputLoop:
	do		#63, EndProcessOutputLoop-2	; 64 Words
	; Get the output sample from the delayline (SRAM_OUT_BUF)
	mov		[W13++], W4
	; If TAILS is off, we need to multiply LEVEL_CV by the FADE_CV
	; to generate a fade-in/fade-out signal
	mov		LEVEL_CV, W5
	mov		[W8++], W6				; Get the Fade CV from the buffer
	xor		W6, W11, W6				; Invert it if required
	mpy		W5*W6, A
	sac.r	A, W5					; Level = LEVEL_CV * FADE_CV
	btsc	FLAGS, #TAILS
	mov		LEVEL_CV, W5			; If TAILS is on, use LEVEL CV on its own
	mpy		W4*W5, A				; A = (Delay * Level)
	sac.r	A, [W2++]				; Put it in the OUT_BUFx
EndProcessOutputLoop:
	; Ok, so we can send all the processed samples to the delay
	call	SendToRAM				; Fill RAM from SRAM_IN_BUF buffer
	; Ok, so that's done. We're ready to move to the next block for next time.
	
	
;=======================================================
;	Move to the next SRAM read/write block
;=======================================================
NextRAMBlock:
	; Move to the next read/write block
	mov		#32, W0
	add		READ_BLOCK				; These can wraparound, no problem
	add		NEW_RD_BLOCK
	add		NEXT_RD_BLOCK
	add		WRITE_BLOCK
	
	
EndProcessBuffers:
	; Ok, we've generated another buffer's worth of audio for the output,
	; and processed the incoming audio ready to go into the RAM.
	; We're done here!
	return
	
	
;=======================================================
;	In/Out (Bypass) Button timer
;=======================================================
ButtonTimer:
	; We're timing a button press
	dec		BUTTON_TIMER			; Decrement 1 block from the count
	btss	SR, #Z					; Have we reached zero?
	return
ButtonTimerEnded:
	bclr	FLAGS, #TIMER_ON		; Stop the timer
	bset	FLAGS, #LONG_PRESS		; It must have been a long press
	return
	

;=======================================================
;	Tempo Marking
;=======================================================
; This routine deals with both the PWM-controlled Tempo LED
; and the simple SYNC_OUT pulses
TempoMarking:	
	; Is the Tap Tempo timer running?
	btst	FLAGS, #TAP_TIMER_ON
	bra		Z, notap

	inc		TAP_TIMER				; Add a block
	mov		TAP_TIMER, W0			; check for maximum delay range
	mov		#2047, W1
	cp		W0, W1
	bra		NC, notap
	
	clr		TAP_TIMER				; it has taken too much time between taps
	bclr	FLAGS, #TAP_TIMER_ON	; stop the timer
		
notap:
	; The Sync pulse is a downcounter
	dec		SYNC_TIMER
	btsc	SR, #Z					; Have we reached zero?
	bclr	LATA, #SYNC_OUT			; Yes, so the pulse is over
	; The tempo is a downcounter
	dec		TEMPO_TIMER
	btss	SR, #N					; Have we gone negative?
	return
TempoPulse:
	bclr	SR, #N					; IS THIS *REALLY* NECESSARY?!
	mov		TEMPO, W0
	mov		W0, TEMPO_TIMER			; Reset the counter
	mov		#SYNC_COUNT, W0
	mov		W0, SYNC_TIMER			; Reset the Sync Out pulse timer too
	bset	LATA, #SYNC_OUT
	mov		#0xFFFF, W0
	mov		W0, BRIGHTNESS			; Reset the pulse brightness
	return
	

;=======================================================
;	Do something with the input control values
;=======================================================
ProcessCVs:
;   +++   CV Filtering   +++
; We use single-pole IIR filters to smooth the CVs and reduce noise.
; filt = last_filt + (k * (input - last_filt))
		
	; Set up per-filter pointers and variables
	mov		#RAW_CV0, W1			; Raw CV input pointer
	mov		#FILTER0_L, W2			; First pole filter pointer
	mov		#LEVEL_CV_L, W3			; Second pole filter pointer
	; W4 - Temp storage
CVFilterLoop:
	do		#4, EndCVFilterLoop-2	; Five CV filters
; Do the first pole
	; First calculate (input - last_filt)
	mov		#0, W0					; W0 = InputLo = 0
	mov		[W1++], W4				; W4 = InputHi
	sub		W0, [W2++], W5			; W5 = TempLo = InputLo - FilterLo
	subb	W4, [W2--], W6			; W6 = TempHi = InputHi - FilterHi
	; Then do k*Temp ( e.g. shift W5:W6 down 7 places)
	mov		W6, W4					; Copy TempHi to W4
	asr		W6, #7, W6				; Shift W6 down (lose 8 bits off bottom)
	lsr		W5, #7, W5				; Shift W5 down
	and		#0x7F, W4				; Mask W4 to get lowest 8 'lost' bits
	sl		W4, #9, W4				; Shift W4 up to get 8 bits at top
	ior		W5, W4, W5				; Put 'lost' bits back into W5
	; Then add the shifted version onto the original
	add		W5, [W2], [W2++]		; FilterLo += TempLo
	addc	W6, [W2], [W2--]		; FilterHi += TempHi
; Do the second pole
	; First calculate (input - last_filt)
	mov		[W2++], W4				; This time, the output from the first..
	sub		W4, [W3++], W5			; ..filter is our input
	mov		[W2++], W4	
	subb	W4, [W3--], W6
	; Then do k*Temp ( e.g. shift W5:W6 down 8 places)
	mov		W6, W4			
	asr		W6, #7, W6	
	lsr		W5, #7, W5
	and		#0x7F, W4	
	sl		W4, #9, W4	
	ior		W5, W4, W5	
	; Then add the shifted version onto the original
	add		W5, [W3], [W3++]
	addc	W6, [W3], [W3++]
EndCVFilterLoop:
	; Ok, we've dealt with the new CV values
	bclr	FLAGS, #NEW_CVS
	

;   +++   Knob 0: Delay Level   +++
LevelKnob:
; Nothing needs doing to this

;   +++   Knob 1: Repeats (Delay Feedback)   +++
RepeatsKnob:
; Nothing needs doing to this

;   +++   Knob 2: Delay Time   +++
DelayKnob:
	; Input values from filtering are 0-32767, 15-bit
	; Make a lookup index from the CV value
	mov		DELAY_CV, W1
	lsr		W1, #4, W1				; Reduce to 0-511 range
	bclr	W1, #0					; Make into an index
	; Get the Delay Time for this control value from the table
	mov		#tblpage(DelayTime), W0
	mov		W0, TBLPAG							; Set up TBLPAG
	mov		#tbloffset(DelayTime), W0			; Get table base position
	add		W0, W1, W0				; Add index to base address
	tblrdl	[W0], W2				; Get required TEMPO into W2
;	
;	btsc	FLAGS, #ONE_SRAM		; Are we only using one SRAM?
;	lsr		W2, #1, W1				; Reduce range if so
;	
	; Get the Tempo LED decay time too
	mov		#tblpage(TempoLookup), W0
	mov		W0, TBLPAG							; Set up TBLPAG
	mov		#tbloffset(TempoLookup), W0			; Get table base position
	sl		W2, #1, W1				; Make TEMPO into an index
	add		W0, W1, W0				; Add index to base address
	tblrdl	[W0], W3				; Get required BRIGHT_DEC into W3
	; Has the value changed since last time we checked?
	mov		PREV_TEMPO, W1			; This is what it was
	xor		W2, W1, W1
	btsc	SR, #Z					; If zero set, it's the same
	goto	LowKnob
	; Ok, so it's not the same
	mov		W2, PREV_TEMPO			; For next time
	mov		W2, TEMPO				; Update the Tempo LED
	mov		W3, BRIGHT_DEC			; Update the Tempo decay rate
	; Generate a new Read pointer by taking it off the Write pointer
	sl		W2, #5, W2				; Shift up to make block address
	mov		WRITE_BLOCK, W1
	sub		W1, W2, W2				; This should wrap happily
	; Are we crossfading already?
	btss	FLAGS, #DLY_XFADE_ON
	goto	DelayTimeTweak

DelayTimeTweakDuringCrossfade:
	; Ok, so we're already crossfading to another delay block position
	; We just store this as the next one to happen
	mov		W2, NEXT_RD_BLOCK
	bset	FLAGS, #DLY_XFADE_NEXT
	; If another change comes in before this crossfade is over, this will
	; be replaced - no problem.
	goto	LowKnob

DelayTimeTweak:
	mov		W2, NEW_RD_BLOCK
	bset	FLAGS, #DLY_XFADE_ON


;   +++   Knob 3: Lowpass Filter   +++
LowKnob:
; This is the filter cutoff for the feedback single pole IIR
	mov		#tblpage(FeedbackFilter), W0
	mov		W0, TBLPAG							; Set up TBLPAG
	mov		#tbloffset(FeedbackFilter), W0		; Get table base position
	mov		LOWPASS_CV, W1
	lsr		W1, #6, W1				; Reduce to 0-511 range
	bclr	W1, #0					; Make into an index
	add		W0, W1, W0				; Add index to base address
	add		#304, W0				; Offset LPF to CV>=152
	tblrdl	[W0], W0				; Get result
	mov		W0, LP_A0				; Store a0 coefficient	
	mov		W0, LP_B1				; Store b1 coefficient


;   +++   Knob 4: Highpass Filter   +++
HighKnob:
; This is the filter cutoff for the feedback single pole IIR
	mov		#tblpage(FeedbackFilter), W0
	mov		W0, TBLPAG							; Set up TBLPAG
	mov		#tbloffset(FeedbackFilter), W0		; Get table base position
	mov		HIGHPASS_CV, W1
	lsr		W1, #6, W1				; Reduce to 0-511 range
	bclr	W1, #0					; Make into an index
	add		W0, W1, W0				; Add index to base address
	tblrdl	[W0], W0				; Get result
	mov		W0, HP_A0				; Store a0 coefficient	
	mov		W0, HP_B1				; Store b1 coefficient
	
	; That's all the CVs dealt with!
	return


;=======================================================
;	Process Buttons - Debounce and act on result
;=======================================================
; Do Scott Dattalo's vertical counter debounce
ProcessButtons:
	; Increment the vertical counter
	mov		DEBOUNCE_LO, W0
	xor		DEBOUNCE_HI				; Hi+1 = Hi XOR lo
	com		DEBOUNCE_LO				; Lo+1 = !Lo
	; See if any changes occured
	mov		IN_STATES, W0
	xor		PORTB, WREG
	; Reset any counters where there are no changes
	and		DEBOUNCE_LO
	and		DEBOUNCE_HI
	; Do any of these changes represent fully debounced input changes?
	com		W0, W0					; Invert (0=changed)
	ior		DEBOUNCE_LO, WREG		; If count=0, both HI & LO are 0
	ior		DEBOUNCE_HI, WREG
	com		W0, W0					; Invert (1=changed)
	mov		W0, IN_CHANGES			; Store the changes
	xor		IN_STATES				; Update the debounced states
	; Any bits set in IN_CHANGES represent debounced inputs that have changed


;=======================================================
;	Test the buttons
;=======================================================
; Note that the Bypass and Tempo switches should be momentary to get the full
; benefit. A latching footswitch *could* be used for the effect in/out, but
; will always give the 'long press', so the momentary/latching feature is lost.

; The Tails selection is done with a jumper, so this could be replaced with
; a latching switch or a toggleswitch.

;=======================================================
; Button 0 = Effect On/Off footswitch
;=======================================================
TestButton0:
	btss	IN_CHANGES, #BUTTON0	; Did the button change?
	goto	TestButton1
	btsc	IN_STATES, #BUTTON0		; Was it pressed down?
	goto	Button0Released
Button0Pressed:
	btsc	FLAGS, #FADING
	goto	TestButton1				; If we're fading already, ignore presses
	; Toggle the effect on/off
	btg		FLAGS, #DELAY_ON		; Toggle the effect
	btg		LATA, #LED0				; Toggle the indicator LED
	bset	FLAGS, #FADING			; Either way, we're fading
	clr		FADE_BUF+126			; Clear previous Fade CV
	; Start the timer to determine if this is a long press
	mov		#TIMER_COUNT, W0
	mov		W0, BUTTON_TIMER		; Reset the downcounter
	bclr	FLAGS, #LONG_PRESS		; Ensure this is clear (belt+braces)
	bset	FLAGS, #TIMER_ON		; Start it running
	goto	TestButton1
Button0Released:
; There are several possibilities:
; If the effect is off, do nothing
; If the effect is on, we check the button timer flag
	; If we had a long press, turn the effect off.
	; If it was a short press, do nothing
	; In either case, stop the timer
	bclr	FLAGS, #TIMER_ON		; Stop the button press timer
	; Is the effect on or off?
	btss	FLAGS, #DELAY_ON
	goto	TestButton1				; Effect off - ignore button release
	; The effect is on.
	; Do we want to leave it on (short press) or turn it off (long press)?
	btss	FLAGS, #LONG_PRESS
	goto	TestButton1				; Short press - ignore
	; It was a long press, so turn the effect off again
	bclr	FLAGS, #DELAY_ON		; Turn effect off
	bclr	LATA, #LED0				; Tutn indicator LED off
	bclr	FLAGS, #LONG_PRESS
	; Are we still fading?
	mov		#0xFFFF, W0
	btsc	FLAGS, #FADING
	xor		FADE_BUF+126			; Modify the Fade CV if so
	bset	FLAGS, #FADING			; Either way, we're fading now

	
;=======================================================
; Button 1 = Tap Tempo
;=======================================================
; The tempo is timed by TAP_TIMER. It is measured in 2msecs blocks, by the
; TempoMarking routine
TestButton1:
	btss	IN_CHANGES, #BUTTON1	; Did the button change?
	goto	TestButton2
	btsc	IN_STATES, #BUTTON1		; Was it pressed down?
	goto	Button1Released
Button1Pressed:
	; Which tap was it?
	btsc	FLAGS, #TAP_TIMER_ON
	goto	SecondTap
FirstTap:									; First Tap
	bset	FLAGS, #TAP_TIMER_ON	; Start the tap timer
	bset	LATA, #LED1				; Turn the Tempo LED on
	clr		TAP_TIMER
	goto	TestButton2
SecondTap:
	mov		TAP_TIMER, W2
	clr		TAP_TIMER
	
TapComputed:
	; Ok, we've got a new (tapped) tempo
	mov		W2, TEMPO				; Store the new tempo
	clr		TEMPO_TIMER
	; Get the Tempo LED decay time too
	mov		#tblpage(TempoLookup), W0
	mov		W0, TBLPAG							; Set up TBLPAG
	mov		#tbloffset(TempoLookup), W0			; Get table base position
	sl		W2, #1, W1				; Make TEMPO into an index
	add		W0, W1, W0				; Add index to base address
	tblrdl	[W0], W0				; Get required BRIGHT_DEC
	mov		W0, BRIGHT_DEC			; Store it
	; Generate a new Read pointer by taking Tempo off the Write pointer
	sl		W2, #5, W0				; Turn TEMPO into a block addres
	mov		WRITE_BLOCK, W1
	sub		W1, W0, W0				; This should wrap happily
	; Are we crossfading already?
	btss	FLAGS, #DLY_XFADE_ON
	goto	TapUpdate
TapUpdateDuringCrossfade:
	; Ok, so we're already crossfading to another delay block position
	; We just store this as the next one to happen
	mov		W0, NEXT_RD_BLOCK
	bset	FLAGS, #DLY_XFADE_NEXT
	goto	TestButton2
TapUpdate:
	mov		W0, NEW_RD_BLOCK
	bset	FLAGS, #DLY_XFADE_ON
	goto	TestButton2
Button1Released:
	; DOES NOTHING

	
;=======================================================
; Button 2 = Delay Tails On/Off switch (not momentary)
;=======================================================
TestButton2:
	; TAILS just follows the state of the input 0=On, 1=Off
;	btss	IN_CHANGES, #BUTTON2	; Did the button change?
;	return
	btsc	IN_STATES, #BUTTON2		; Is it pressed down?
	goto	Button2Released
Button2Pressed:
	bset	FLAGS, #TAILS			; Switch the delay tails on
;	bset	LATA, #LED2				; Turn the Tails LED on
	return
Button2Released:
	bclr	FLAGS, #TAILS			; Switch the delay tails off
;	bclr	LATA, #LED2				; Turn the Tails LED off
	return


;==============================================================================
;	Timer2 "Master Clock" Interrupt
;==============================================================================
; This runs at 64.8KHz, twice the audio rate.
; The Audio and CV inputs are interleaved according to the CVReadSequence
; table. This gives us an audio input/output sampled at 32.4KHz, and five CV
; inputs sampled at 6.48KHz.

__T2Interrupt:
	push.s							; Save W0-W3 & Status to shadow regs
	bclr	IFS0, #T2IF				; Clear interrupt flag
	; What type of CV is this one?
	btss	CV_CHANNEL, #1			; CVs=2,6,10,14,18, Audio=0,4,8,12,16
	goto	ReadAudioInput

	
;	***	Odd cycles - Read a CV input ***
;==========================================
ReadCVInput:
	; Get the last CV reading
	mov		ADC1BUF0, W2
	sl		W2, #3, W2				; Scale it up to 15-bit range 0-32767
	; Put it into the correct CV memory location
	mov		#RAW_CV0, W1			; Get base address
	lsr		CV_CHANNEL, WREG		; Channel 2,6,10,14,18 -> 1,3,5,7,9
	dec		W0, W0					; Channel 1,3,5,7,9 -> 0,2,4,6,8
	mov		W2, [W1+W0]				; Store CV
	; Ok, that's that.
	goto	NextCVChannel
	

;	***	Even cycles - Read the audio input ***
;================================================
ReadAudioInput:
	; Alternative clean input
	mov		ADC1BUF0, W2					; Get ADC reading
	sl		W2, #4, W2						; Shift up to 0-65520
	btg		W2, #15							; Convert to +/-32752
	
;	; Get the new audio sample (0-4095)
;	sl		ADC1BUF0, WREG					; Make ADC reading into an index
;	; Apply the input drive curve to the sample (+/-32611)
;	mov		#tblpage(InputCurve), W1
;	mov		W1, TBLPAG						; Set up TBLPAG
;	mov		#tbloffset(InputCurve), W1		; Get table base position
;	add		W1, W0, W0				; Add index to base address
;	tblrdl	[W0], W2				; Get result (Input)
	
	; Put the input into the In buffer
	mov		IN_LEFT, W0				; Get pointer
	mov		BUFFER_POS, W1			; Get offset
	mov		W2, [W0+W1]				; Send input to buffer
	; Fetch the output from the Out buffer
	mov		OUT_LEFT, W0			; Get pointer (we already have the offset)
	mov		[W0+W1], W2				; Get outgoing sample
	mov		W2, DAC1LDAT			; Send it to the DAC
; Move to the next audio buffer position
NextBufferPos:
	inc2	BUFFER_POS
	btss	BUFFER_POS, #7			; BUFFER_POS = 128?
	goto	NextCVChannel			; Not yet, so ignore
	
BufferOverflow:
	clr		BUFFER_POS				; Go back to beginning
	bset	FLAGS, #BUFF_PROCESS_REQ		; Buffer is full
	; Switch to the other buffer
	btg		FLAGS, #INT_USING_BUFF1
	btg		IN_LEFT, #7				; Toggle the IN pointer
	btg		OUT_LEFT, #7			; Toggle the OUT pointer
	
NextCVChannel:
	; Move to the next channel
	inc2	CV_CHANNEL				; Increment channel
	; Test for wrap-around
	mov		CV_CHANNEL, W0
	xor		#20, W0					; CV Channel = 20?
	btss	SR, #Z					; If zero, these match
	goto	ConfigADCForNewChannel
WrapCVReadSequence:
	clr		CV_CHANNEL				; CV Channel reset to zero
	bset	FLAGS, #NEW_CVS			; We have 5 new CV values now
ConfigADCForNewChannel:
	; Set up the ADC for the new channel
	mov		#tblpage(CVReadSequence), W0
	mov		W0, TBLPAG							; Set up TBLPAG
	mov		#tbloffset(CVReadSequence), W0		; Get table base position
	add		CV_CHANNEL, WREG		; Add channel offset
	tblrdl	[W0], W0				; Get next input
	mov		W0, AD1CHS0
	; Start the ADC sample/convert process for next time
	bset	AD1CON1, #SAMP
	
TempoPWM:
	; We control the brightness of the Tempo LED using PWM
	mov		#256, W0				; ~125Hz PWM
	add		PWM_COUNT
	; Brightness of Tempo LED decreases over time
	mov		BRIGHT_DEC, W0
	sub		BRIGHTNESS
	btss	SR, #C
	clr		BRIGHTNESS				; If we were at zero, stay there
	; if BRIGHTNESS > PWM_COUNT, turn LED on
	mov		PWM_COUNT, WREG
	cp		BRIGHTNESS
	bra		GTU, TempoLEDOn
TempoLEDOff:
	bclr	LATA, #LED1
	goto	InterruptExit
TempoLEDOn:
	bset	LATA, #LED1

InterruptExit:
	pop.s							; Pop registers W0-W3 & Status
	retfie


;==============================================================================
;	23L1024 1MBit (128KB) Serial RAM Read/Write routines
;==============================================================================
; There are two routines, each moves 64 words: FetchFromRAM, SendToRAM
; FetchFromRAM uses passed variables, since it is used twice for crossfades.
; SendToRAM uses a fixed SRAM_IN_BUF buffer.


;=======================================================
; Fetch from RAM - Read a 128 byte block from the SRAM
;=======================================================
; We're using 'Sequential mode' to read 128 bytes sequentially.
; This routine expects the following:
;	W0 - Block number
;	W1 - destination buffer pointer
	
; The full read of 128 bytes/64 words takes about 160usecs.
FetchFromRAM:	
	; Get block number and generate address
	mov		W0, ADDR_LO				; Block number is in W0
	bclr	ADDR_LO, #15			; Wipe the chip select bit
	clr		ADDR_HI
	sl		ADDR_LO					; Shift once
	sl		ADDR_LO					; Shift twice
	rlc		ADDR_HI
	; Select one of the two SRAMs
	btss	READ_BLOCK, #15
	bclr	LATB, #CS0_INHIBIT		; ~CS0 Low
	btsc	READ_BLOCK, #15
	bclr	LATB, #CS1_INHIBIT		; ~CS1 Low
	; Add the 'Read' instruction
	mov		#0x0300, W0
	ior		ADDR_HI
	; Send 'Read' instruction + highest byte of address
	mov		SPI1BUF, W0				; Do dummy read to clear flags
	mov		ADDR_HI, W0
	mov		W0, SPI1BUF				; Send it to the SRAM
ReadAddrHiWait:
	btss	SPI1STAT, #SPIRBF		; Wait while word is received
	goto	ReadAddrHiWait
	; Send lowest 16-bits of address
	mov		SPI1BUF, W0				; Do a dummy read
	mov		ADDR_LO, W0
	mov		W0, SPI1BUF				; Send it to the SRAM
ReadAddrLoWait:
	btss	SPI1STAT, #SPIRBF		; Wait while word is received
	goto	ReadAddrLoWait
	mov		SPI1BUF, W0				; Do a dummy read

; Read 128 bytes (64 words of data)
ReadRAMLoop:
	do		#63, EndReadRAMLoop-2	; 64 Words
	clr		W0
	mov		W0, SPI1BUF				; Do dummy write to ensure clock is sent
ReadRAMWait:
	btss	SPI1STAT, #SPIRBF		; Wait while word is received
	goto	ReadRAMWait
	; Ok, we've received a word from RAM
	mov		SPI1BUF, W0				; RAM -> OUT buffer
	mov		W0, [W1++]				; W1 is buffer pointer
EndReadRAMLoop:
	; That's it!
	bset	LATB, #CS0_INHIBIT		; Unselect both SRAMs
	bset	LATB, #CS1_INHIBIT
	return


;=======================================================
; Send to RAM - Write a 128 byte block to the SRAM
;=======================================================
SendToRAM:
	; Get block number and generate address
	mov		WRITE_BLOCK, W0
	mov		W0, ADDR_LO
	bclr	ADDR_LO, #15			; Wipe the chip select bit
	clr		ADDR_HI
	sl		ADDR_LO					; Shift once
	sl		ADDR_LO					; Shift twice
	rlc		ADDR_HI
	; Select one of the two SRAMs
	btss	WRITE_BLOCK, #15
	bclr	LATB, #CS0_INHIBIT		; ~CS0 Low
	btsc	WRITE_BLOCK, #15
	bclr	LATB, #CS1_INHIBIT		; ~CS1 Low
	; Add the 'Write' instruction
	mov		#0x0200, W0
	ior		ADDR_HI
	; Send 'Write' instruction + highest byte of address
	mov		SPI1BUF, W0				; Do dummy read to clear flags
	mov		ADDR_HI, W0
	mov		W0, SPI1BUF				; Send it to the SRAM
WriteAddrHiWait:
	btss	SPI1STAT, #SPIRBF		; Wait while it sends
	goto	WriteAddrHiWait
	; Send lowest 16-bits of address
	mov		SPI1BUF, W0				; Do dummy read
	mov		ADDR_LO, W0
	mov		W0, SPI1BUF				; Send it to the SRAM
WriteAddrLoWait:
	btss	SPI1STAT, #SPIRBF		; Wait while it sends
	goto	WriteAddrLoWait

; Write 128 bytes (64 words of data)
	mov		#SRAM_IN_BUF, W1		; Set up pointer from source buffer
WriteRAMLoop:
	do		#63, EndWriteRAMLoop-2	; 64 Words
	; Ok, we're ready to send a word to RAM
	mov		SPI1BUF, W0				; Do dummy read to clear flags
	mov		[W1++], W0				; IN buffer -> RAM
	mov		W0, SPI1BUF
WriteRAMWait:
	btss	SPI1STAT, #SPIRBF		; Wait while it sends
	goto	WriteRAMWait
	nop								; Avoid 'goto' as last loop instruction
EndWriteRAMLoop:
	; That's it!
	bset	LATB, #CS0_INHIBIT		; Unselect both SRAMs
	bset	LATB, #CS1_INHIBIT
	return


;==============================================================================
;	ADC Channel read table
;==============================================================================
; This table defines the order in which ADC inputs are read. It is organised
; so that audio is read every other interrupt (e.g. at 32.4KHz) while the CVs
; are read round-robin on the alternate interrupt.
; So it looks like this:
; Audio, CV0, Audio, CV1, Audio, CV2, Audio, CV3, Audio, CV4, etc
; Values given are for AD1CHS0
CVReadSequence:
	.hword	5,0,5,1,5,2,5,3,5,4

	
;==============================================================================
;	Audio input drive table
;==============================================================================
; The ADC gives us unsigned 12-bit data 0-4095. We need to convert to 1.15
; signed data. It would also be useful if we could avoid hard clipping on the
; input. This table is a way to achieve both those things.
; It converts from 0-4095 to 1.15, but also includes a soft tanh drive curve.
; It also inverts, since I need that too.
; This should soften the blow and give you some warning if you drive the input
; hard.
InputCurve:
	.hword	-32611, -32610, -32610, -32610, -32609, -32609, -32608, -32608
	.hword	-32607, -32607, -32606, -32606, -32605, -32605, -32604, -32604
	.hword	-32603, -32603, -32602, -32602, -32601, -32601, -32600, -32600
	.hword	-32599, -32599, -32598, -32598, -32597, -32596, -32596, -32595
	.hword	-32595, -32594, -32594, -32593, -32593, -32592, -32592, -32591
	.hword	-32591, -32590, -32590, -32589, -32588, -32588, -32587, -32587
	.hword	-32586, -32586, -32585, -32585, -32584, -32583, -32583, -32582
	.hword	-32582, -32581, -32581, -32580, -32579, -32579, -32578, -32578

	.hword	-32577, -32577, -32576, -32575, -32575, -32574, -32574, -32573
	.hword	-32572, -32572, -32571, -32571, -32570, -32569, -32569, -32568
	.hword	-32568, -32567, -32566, -32566, -32565, -32565, -32564, -32563
	.hword	-32563, -32562, -32561, -32561, -32560, -32560, -32559, -32558
	.hword	-32558, -32557, -32556, -32556, -32555, -32554, -32554, -32553
	.hword	-32552, -32552, -32551, -32551, -32550, -32549, -32549, -32548
	.hword	-32547, -32547, -32546, -32545, -32544, -32544, -32543, -32542
	.hword	-32542, -32541, -32540, -32540, -32539, -32538, -32538, -32537

	.hword	-32536, -32535, -32535, -32534, -32533, -32533, -32532, -32531
	.hword	-32530, -32530, -32529, -32528, -32528, -32527, -32526, -32525
	.hword	-32525, -32524, -32523, -32522, -32522, -32521, -32520, -32519
	.hword	-32519, -32518, -32517, -32516, -32516, -32515, -32514, -32513
	.hword	-32512, -32512, -32511, -32510, -32509, -32509, -32508, -32507
	.hword	-32506, -32505, -32505, -32504, -32503, -32502, -32501, -32500
	.hword	-32500, -32499, -32498, -32497, -32496, -32496, -32495, -32494
	.hword	-32493, -32492, -32491, -32491, -32490, -32489, -32488, -32487

	.hword	-32486, -32485, -32485, -32484, -32483, -32482, -32481, -32480
	.hword	-32479, -32478, -32478, -32477, -32476, -32475, -32474, -32473
	.hword	-32472, -32471, -32470, -32470, -32469, -32468, -32467, -32466
	.hword	-32465, -32464, -32463, -32462, -32461, -32460, -32459, -32458
	.hword	-32457, -32457, -32456, -32455, -32454, -32453, -32452, -32451
	.hword	-32450, -32449, -32448, -32447, -32446, -32445, -32444, -32443
	.hword	-32442, -32441, -32440, -32439, -32438, -32437, -32436, -32435
	.hword	-32434, -32433, -32432, -32431, -32430, -32429, -32428, -32427

	.hword	-32426, -32425, -32424, -32423, -32422, -32420, -32419, -32418
	.hword	-32417, -32416, -32415, -32414, -32413, -32412, -32411, -32410
	.hword	-32409, -32408, -32406, -32405, -32404, -32403, -32402, -32401
	.hword	-32400, -32399, -32398, -32396, -32395, -32394, -32393, -32392
	.hword	-32391, -32390, -32388, -32387, -32386, -32385, -32384, -32383
	.hword	-32381, -32380, -32379, -32378, -32377, -32375, -32374, -32373
	.hword	-32372, -32371, -32369, -32368, -32367, -32366, -32365, -32363
	.hword	-32362, -32361, -32360, -32358, -32357, -32356, -32355, -32353

	.hword	-32352, -32351, -32350, -32348, -32347, -32346, -32344, -32343
	.hword	-32342, -32341, -32339, -32338, -32337, -32335, -32334, -32333
	.hword	-32331, -32330, -32329, -32327, -32326, -32325, -32323, -32322
	.hword	-32321, -32319, -32318, -32317, -32315, -32314, -32312, -32311
	.hword	-32310, -32308, -32307, -32305, -32304, -32303, -32301, -32300
	.hword	-32298, -32297, -32295, -32294, -32293, -32291, -32290, -32288
	.hword	-32287, -32285, -32284, -32282, -32281, -32279, -32278, -32276
	.hword	-32275, -32273, -32272, -32270, -32269, -32267, -32266, -32264

	.hword	-32263, -32261, -32260, -32258, -32257, -32255, -32254, -32252
	.hword	-32250, -32249, -32247, -32246, -32244, -32242, -32241, -32239
	.hword	-32238, -32236, -32234, -32233, -32231, -32230, -32228, -32226
	.hword	-32225, -32223, -32221, -32220, -32218, -32216, -32215, -32213
	.hword	-32211, -32210, -32208, -32206, -32204, -32203, -32201, -32199
	.hword	-32198, -32196, -32194, -32192, -32191, -32189, -32187, -32185
	.hword	-32184, -32182, -32180, -32178, -32176, -32175, -32173, -32171
	.hword	-32169, -32167, -32166, -32164, -32162, -32160, -32158, -32156

	.hword	-32154, -32153, -32151, -32149, -32147, -32145, -32143, -32141
	.hword	-32139, -32137, -32136, -32134, -32132, -32130, -32128, -32126
	.hword	-32124, -32122, -32120, -32118, -32116, -32114, -32112, -32110
	.hword	-32108, -32106, -32104, -32102, -32100, -32098, -32096, -32094
	.hword	-32092, -32090, -32088, -32086, -32084, -32082, -32080, -32077
	.hword	-32075, -32073, -32071, -32069, -32067, -32065, -32063, -32060
	.hword	-32058, -32056, -32054, -32052, -32050, -32047, -32045, -32043
	.hword	-32041, -32039, -32036, -32034, -32032, -32030, -32028, -32025

	.hword	-32023, -32021, -32019, -32016, -32014, -32012, -32009, -32007
	.hword	-32005, -32002, -32000, -31998, -31995, -31993, -31991, -31988
	.hword	-31986, -31984, -31981, -31979, -31977, -31974, -31972, -31969
	.hword	-31967, -31964, -31962, -31960, -31957, -31955, -31952, -31950
	.hword	-31947, -31945, -31942, -31940, -31937, -31935, -31932, -31930
	.hword	-31927, -31925, -31922, -31920, -31917, -31914, -31912, -31909
	.hword	-31907, -31904, -31901, -31899, -31896, -31893, -31891, -31888
	.hword	-31885, -31883, -31880, -31877, -31875, -31872, -31869, -31867

	.hword	-31864, -31861, -31858, -31856, -31853, -31850, -31847, -31845
	.hword	-31842, -31839, -31836, -31833, -31831, -31828, -31825, -31822
	.hword	-31819, -31816, -31813, -31810, -31808, -31805, -31802, -31799
	.hword	-31796, -31793, -31790, -31787, -31784, -31781, -31778, -31775
	.hword	-31772, -31769, -31766, -31763, -31760, -31757, -31754, -31751
	.hword	-31748, -31745, -31742, -31739, -31735, -31732, -31729, -31726
	.hword	-31723, -31720, -31717, -31713, -31710, -31707, -31704, -31701
	.hword	-31697, -31694, -31691, -31688, -31684, -31681, -31678, -31675

	.hword	-31671, -31668, -31665, -31661, -31658, -31655, -31651, -31648
	.hword	-31644, -31641, -31638, -31634, -31631, -31627, -31624, -31621
	.hword	-31617, -31614, -31610, -31607, -31603, -31600, -31596, -31593
	.hword	-31589, -31585, -31582, -31578, -31575, -31571, -31568, -31564
	.hword	-31560, -31557, -31553, -31549, -31546, -31542, -31538, -31535
	.hword	-31531, -31527, -31523, -31520, -31516, -31512, -31508, -31505
	.hword	-31501, -31497, -31493, -31489, -31485, -31482, -31478, -31474
	.hword	-31470, -31466, -31462, -31458, -31454, -31450, -31446, -31442

	.hword	-31438, -31434, -31430, -31426, -31422, -31418, -31414, -31410
	.hword	-31406, -31402, -31398, -31394, -31390, -31385, -31381, -31377
	.hword	-31373, -31369, -31365, -31360, -31356, -31352, -31348, -31343
	.hword	-31339, -31335, -31330, -31326, -31322, -31317, -31313, -31309
	.hword	-31304, -31300, -31296, -31291, -31287, -31282, -31278, -31273
	.hword	-31269, -31264, -31260, -31255, -31251, -31246, -31242, -31237
	.hword	-31232, -31228, -31223, -31219, -31214, -31209, -31205, -31200
	.hword	-31195, -31191, -31186, -31181, -31176, -31172, -31167, -31162

	.hword	-31157, -31152, -31147, -31143, -31138, -31133, -31128, -31123
	.hword	-31118, -31113, -31108, -31103, -31098, -31093, -31088, -31083
	.hword	-31078, -31073, -31068, -31063, -31058, -31053, -31048, -31043
	.hword	-31037, -31032, -31027, -31022, -31017, -31011, -31006, -31001
	.hword	-30996, -30990, -30985, -30980, -30974, -30969, -30964, -30958
	.hword	-30953, -30947, -30942, -30936, -30931, -30926, -30920, -30915
	.hword	-30909, -30903, -30898, -30892, -30887, -30881, -30875, -30870
	.hword	-30864, -30858, -30853, -30847, -30841, -30836, -30830, -30824

	.hword	-30818, -30812, -30807, -30801, -30795, -30789, -30783, -30777
	.hword	-30771, -30765, -30759, -30753, -30747, -30741, -30735, -30729
	.hword	-30723, -30717, -30711, -30705, -30699, -30693, -30687, -30680
	.hword	-30674, -30668, -30662, -30655, -30649, -30643, -30637, -30630
	.hword	-30624, -30617, -30611, -30605, -30598, -30592, -30585, -30579
	.hword	-30572, -30566, -30559, -30553, -30546, -30540, -30533, -30526
	.hword	-30520, -30513, -30506, -30500, -30493, -30486, -30479, -30473
	.hword	-30466, -30459, -30452, -30445, -30438, -30432, -30425, -30418

	.hword	-30411, -30404, -30397, -30390, -30383, -30376, -30369, -30361
	.hword	-30354, -30347, -30340, -30333, -30326, -30318, -30311, -30304
	.hword	-30297, -30289, -30282, -30275, -30267, -30260, -30252, -30245
	.hword	-30238, -30230, -30223, -30215, -30208, -30200, -30192, -30185
	.hword	-30177, -30170, -30162, -30154, -30147, -30139, -30131, -30123
	.hword	-30115, -30108, -30100, -30092, -30084, -30076, -30068, -30060
	.hword	-30052, -30044, -30036, -30028, -30020, -30012, -30004, -29996
	.hword	-29988, -29980, -29971, -29963, -29955, -29947, -29938, -29930

	.hword	-29922, -29913, -29905, -29897, -29888, -29880, -29871, -29863
	.hword	-29854, -29846, -29837, -29828, -29820, -29811, -29802, -29794
	.hword	-29785, -29776, -29768, -29759, -29750, -29741, -29732, -29723
	.hword	-29714, -29705, -29696, -29687, -29678, -29669, -29660, -29651
	.hword	-29642, -29633, -29624, -29615, -29605, -29596, -29587, -29578
	.hword	-29568, -29559, -29550, -29540, -29531, -29521, -29512, -29502
	.hword	-29493, -29483, -29474, -29464, -29454, -29445, -29435, -29425
	.hword	-29416, -29406, -29396, -29386, -29376, -29366, -29357, -29347

	.hword	-29337, -29327, -29317, -29307, -29297, -29286, -29276, -29266
	.hword	-29256, -29246, -29236, -29225, -29215, -29205, -29194, -29184
	.hword	-29174, -29163, -29153, -29142, -29132, -29121, -29111, -29100
	.hword	-29089, -29079, -29068, -29057, -29046, -29036, -29025, -29014
	.hword	-29003, -28992, -28981, -28970, -28959, -28948, -28937, -28926
	.hword	-28915, -28904, -28893, -28882, -28870, -28859, -28848, -28837
	.hword	-28825, -28814, -28802, -28791, -28779, -28768, -28756, -28745
	.hword	-28733, -28722, -28710, -28698, -28687, -28675, -28663, -28651

	.hword	-28639, -28627, -28616, -28604, -28592, -28580, -28568, -28556
	.hword	-28543, -28531, -28519, -28507, -28495, -28482, -28470, -28458
	.hword	-28445, -28433, -28421, -28408, -28396, -28383, -28370, -28358
	.hword	-28345, -28333, -28320, -28307, -28294, -28282, -28269, -28256
	.hword	-28243, -28230, -28217, -28204, -28191, -28178, -28165, -28152
	.hword	-28138, -28125, -28112, -28099, -28085, -28072, -28059, -28045
	.hword	-28032, -28018, -28005, -27991, -27978, -27964, -27950, -27937
	.hword	-27923, -27909, -27895, -27881, -27867, -27854, -27840, -27826

	.hword	-27812, -27797, -27783, -27769, -27755, -27741, -27727, -27712
	.hword	-27698, -27684, -27669, -27655, -27640, -27626, -27611, -27597
	.hword	-27582, -27567, -27553, -27538, -27523, -27508, -27493, -27479
	.hword	-27464, -27449, -27434, -27419, -27403, -27388, -27373, -27358
	.hword	-27343, -27327, -27312, -27297, -27281, -27266, -27250, -27235
	.hword	-27219, -27204, -27188, -27172, -27157, -27141, -27125, -27109
	.hword	-27093, -27077, -27062, -27046, -27029, -27013, -26997, -26981
	.hword	-26965, -26949, -26932, -26916, -26900, -26883, -26867, -26850

	.hword	-26834, -26817, -26801, -26784, -26767, -26751, -26734, -26717
	.hword	-26700, -26683, -26666, -26649, -26632, -26615, -26598, -26581
	.hword	-26564, -26546, -26529, -26512, -26494, -26477, -26459, -26442
	.hword	-26424, -26407, -26389, -26371, -26354, -26336, -26318, -26300
	.hword	-26282, -26264, -26246, -26228, -26210, -26192, -26174, -26156
	.hword	-26138, -26119, -26101, -26082, -26064, -26046, -26027, -26008
	.hword	-25990, -25971, -25952, -25934, -25915, -25896, -25877, -25858
	.hword	-25839, -25820, -25801, -25782, -25763, -25744, -25724, -25705

	.hword	-25686, -25666, -25647, -25627, -25608, -25588, -25569, -25549
	.hword	-25529, -25509, -25490, -25470, -25450, -25430, -25410, -25390
	.hword	-25370, -25349, -25329, -25309, -25289, -25268, -25248, -25228
	.hword	-25207, -25186, -25166, -25145, -25125, -25104, -25083, -25062
	.hword	-25041, -25020, -24999, -24978, -24957, -24936, -24915, -24894
	.hword	-24872, -24851, -24830, -24808, -24787, -24765, -24744, -24722
	.hword	-24700, -24679, -24657, -24635, -24613, -24591, -24569, -24547
	.hword	-24525, -24503, -24481, -24459, -24436, -24414, -24392, -24369

	.hword	-24347, -24324, -24302, -24279, -24256, -24234, -24211, -24188
	.hword	-24165, -24142, -24119, -24096, -24073, -24050, -24027, -24003
	.hword	-23980, -23957, -23933, -23910, -23886, -23863, -23839, -23815
	.hword	-23792, -23768, -23744, -23720, -23696, -23672, -23648, -23624
	.hword	-23600, -23575, -23551, -23527, -23502, -23478, -23454, -23429
	.hword	-23404, -23380, -23355, -23330, -23306, -23281, -23256, -23231
	.hword	-23206, -23181, -23156, -23130, -23105, -23080, -23054, -23029
	.hword	-23004, -22978, -22952, -22927, -22901, -22875, -22850, -22824

	.hword	-22798, -22772, -22746, -22720, -22694, -22668, -22641, -22615
	.hword	-22589, -22562, -22536, -22509, -22483, -22456, -22429, -22403
	.hword	-22376, -22349, -22322, -22295, -22268, -22241, -22214, -22187
	.hword	-22159, -22132, -22105, -22077, -22050, -22022, -21995, -21967
	.hword	-21939, -21912, -21884, -21856, -21828, -21800, -21772, -21744
	.hword	-21716, -21688, -21659, -21631, -21603, -21574, -21546, -21517
	.hword	-21488, -21460, -21431, -21402, -21373, -21344, -21315, -21286
	.hword	-21257, -21228, -21199, -21170, -21140, -21111, -21082, -21052

	.hword	-21023, -20993, -20963, -20934, -20904, -20874, -20844, -20814
	.hword	-20784, -20754, -20724, -20694, -20664, -20633, -20603, -20572
	.hword	-20542, -20511, -20481, -20450, -20419, -20389, -20358, -20327
	.hword	-20296, -20265, -20234, -20203, -20172, -20140, -20109, -20078
	.hword	-20046, -20015, -19983, -19952, -19920, -19888, -19857, -19825
	.hword	-19793, -19761, -19729, -19697, -19665, -19632, -19600, -19568
	.hword	-19536, -19503, -19471, -19438, -19405, -19373, -19340, -19307
	.hword	-19274, -19242, -19209, -19176, -19142, -19109, -19076, -19043

	.hword	-19010, -18976, -18943, -18909, -18876, -18842, -18808, -18775
	.hword	-18741, -18707, -18673, -18639, -18605, -18571, -18537, -18503
	.hword	-18468, -18434, -18400, -18365, -18331, -18296, -18262, -18227
	.hword	-18192, -18157, -18123, -18088, -18053, -18018, -17982, -17947
	.hword	-17912, -17877, -17842, -17806, -17771, -17735, -17700, -17664
	.hword	-17628, -17593, -17557, -17521, -17485, -17449, -17413, -17377
	.hword	-17341, -17304, -17268, -17232, -17195, -17159, -17122, -17086
	.hword	-17049, -17013, -16976, -16939, -16902, -16865, -16828, -16791

	.hword	-16754, -16717, -16680, -16643, -16605, -16568, -16530, -16493
	.hword	-16455, -16418, -16380, -16342, -16305, -16267, -16229, -16191
	.hword	-16153, -16115, -16077, -16038, -16000, -15962, -15923, -15885
	.hword	-15847, -15808, -15769, -15731, -15692, -15653, -15614, -15576
	.hword	-15537, -15498, -15459, -15420, -15380, -15341, -15302, -15263
	.hword	-15223, -15184, -15144, -15105, -15065, -15025, -14986, -14946
	.hword	-14906, -14866, -14826, -14786, -14746, -14706, -14666, -14626
	.hword	-14585, -14545, -14505, -14464, -14424, -14383, -14343, -14302

	.hword	-14261, -14220, -14180, -14139, -14098, -14057, -14016, -13975
	.hword	-13933, -13892, -13851, -13810, -13768, -13727, -13685, -13644
	.hword	-13602, -13561, -13519, -13477, -13436, -13394, -13352, -13310
	.hword	-13268, -13226, -13184, -13142, -13099, -13057, -13015, -12972
	.hword	-12930, -12888, -12845, -12802, -12760, -12717, -12674, -12632
	.hword	-12589, -12546, -12503, -12460, -12417, -12374, -12331, -12288
	.hword	-12245, -12201, -12158, -12115, -12071, -12028, -11984, -11941
	.hword	-11897, -11853, -11810, -11766, -11722, -11678, -11634, -11590

	.hword	-11546, -11502, -11458, -11414, -11370, -11326, -11281, -11237
	.hword	-11193, -11148, -11104, -11059, -11015, -10970, -10925, -10881
	.hword	-10836, -10791, -10746, -10701, -10657, -10612, -10567, -10522
	.hword	-10476, -10431, -10386, -10341, -10296, -10250, -10205, -10159
	.hword	-10114, -10068, -10023, -9977, -9932, -9886, -9840, -9795
	.hword	-9749, -9703, -9657, -9611, -9565, -9519, -9473, -9427
	.hword	-9381, -9335, -9289, -9242, -9196, -9150, -9103, -9057
	.hword	-9011, -8964, -8918, -8871, -8824, -8778, -8731, -8684

	.hword	-8638, -8591, -8544, -8497, -8450, -8403, -8356, -8309
	.hword	-8262, -8215, -8168, -8121, -8074, -8026, -7979, -7932
	.hword	-7885, -7837, -7790, -7742, -7695, -7647, -7600, -7552
	.hword	-7505, -7457, -7409, -7362, -7314, -7266, -7218, -7170
	.hword	-7123, -7075, -7027, -6979, -6931, -6883, -6835, -6787
	.hword	-6738, -6690, -6642, -6594, -6546, -6497, -6449, -6401
	.hword	-6352, -6304, -6256, -6207, -6159, -6110, -6062, -6013
	.hword	-5964, -5916, -5867, -5819, -5770, -5721, -5672, -5624

	.hword	-5575, -5526, -5477, -5428, -5379, -5330, -5282, -5233
	.hword	-5184, -5135, -5085, -5036, -4987, -4938, -4889, -4840
	.hword	-4791, -4742, -4692, -4643, -4594, -4545, -4495, -4446
	.hword	-4397, -4347, -4298, -4248, -4199, -4150, -4100, -4051
	.hword	-4001, -3952, -3902, -3852, -3803, -3753, -3704, -3654
	.hword	-3604, -3555, -3505, -3455, -3406, -3356, -3306, -3256
	.hword	-3207, -3157, -3107, -3057, -3007, -2958, -2908, -2858
	.hword	-2808, -2758, -2708, -2658, -2608, -2558, -2508, -2458

	.hword	-2408, -2358, -2308, -2258, -2208, -2158, -2108, -2058
	.hword	-2008, -1958, -1908, -1858, -1808, -1758, -1707, -1657
	.hword	-1607, -1557, -1507, -1457, -1407, -1356, -1306, -1256
	.hword	-1206, -1156, -1105, -1055, -1005, -955, -905, -854
	.hword	-804, -754, -704, -653, -603, -553, -503, -452
	.hword	-402, -352, -302, -251, -201, -151, -101, -50
	.hword	0, 50, 101, 151, 201, 251, 302, 352
	.hword	402, 452, 503, 553, 603, 653, 704, 754

	.hword	804, 854, 905, 955, 1005, 1055, 1105, 1156
	.hword	1206, 1256, 1306, 1356, 1407, 1457, 1507, 1557
	.hword	1607, 1657, 1707, 1758, 1808, 1858, 1908, 1958
	.hword	2008, 2058, 2108, 2158, 2208, 2258, 2308, 2358
	.hword	2408, 2458, 2508, 2558, 2608, 2658, 2708, 2758
	.hword	2808, 2858, 2908, 2958, 3007, 3057, 3107, 3157
	.hword	3207, 3256, 3306, 3356, 3406, 3455, 3505, 3555
	.hword	3604, 3654, 3704, 3753, 3803, 3852, 3902, 3952

	.hword	4001, 4051, 4100, 4150, 4199, 4248, 4298, 4347
	.hword	4397, 4446, 4495, 4545, 4594, 4643, 4692, 4742
	.hword	4791, 4840, 4889, 4938, 4987, 5036, 5085, 5135
	.hword	5184, 5233, 5282, 5330, 5379, 5428, 5477, 5526
	.hword	5575, 5624, 5672, 5721, 5770, 5819, 5867, 5916
	.hword	5964, 6013, 6062, 6110, 6159, 6207, 6256, 6304
	.hword	6352, 6401, 6449, 6497, 6546, 6594, 6642, 6690
	.hword	6738, 6787, 6835, 6883, 6931, 6979, 7027, 7075

	.hword	7123, 7170, 7218, 7266, 7314, 7362, 7409, 7457
	.hword	7505, 7552, 7600, 7647, 7695, 7742, 7790, 7837
	.hword	7885, 7932, 7979, 8026, 8074, 8121, 8168, 8215
	.hword	8262, 8309, 8356, 8403, 8450, 8497, 8544, 8591
	.hword	8638, 8684, 8731, 8778, 8824, 8871, 8918, 8964
	.hword	9011, 9057, 9103, 9150, 9196, 9242, 9289, 9335
	.hword	9381, 9427, 9473, 9519, 9565, 9611, 9657, 9703
	.hword	9749, 9795, 9840, 9886, 9932, 9977, 10023, 10068

	.hword	10114, 10159, 10205, 10250, 10296, 10341, 10386, 10431
	.hword	10476, 10522, 10567, 10612, 10657, 10701, 10746, 10791
	.hword	10836, 10881, 10925, 10970, 11015, 11059, 11104, 11148
	.hword	11193, 11237, 11281, 11326, 11370, 11414, 11458, 11502
	.hword	11546, 11590, 11634, 11678, 11722, 11766, 11810, 11853
	.hword	11897, 11941, 11984, 12028, 12071, 12115, 12158, 12201
	.hword	12245, 12288, 12331, 12374, 12417, 12460, 12503, 12546
	.hword	12589, 12632, 12674, 12717, 12760, 12802, 12845, 12888

	.hword	12930, 12972, 13015, 13057, 13099, 13142, 13184, 13226
	.hword	13268, 13310, 13352, 13394, 13436, 13477, 13519, 13561
	.hword	13602, 13644, 13685, 13727, 13768, 13810, 13851, 13892
	.hword	13933, 13975, 14016, 14057, 14098, 14139, 14180, 14220
	.hword	14261, 14302, 14343, 14383, 14424, 14464, 14505, 14545
	.hword	14585, 14626, 14666, 14706, 14746, 14786, 14826, 14866
	.hword	14906, 14946, 14986, 15025, 15065, 15105, 15144, 15184
	.hword	15223, 15263, 15302, 15341, 15380, 15420, 15459, 15498

	.hword	15537, 15576, 15614, 15653, 15692, 15731, 15769, 15808
	.hword	15847, 15885, 15923, 15962, 16000, 16038, 16077, 16115
	.hword	16153, 16191, 16229, 16267, 16305, 16342, 16380, 16418
	.hword	16455, 16493, 16530, 16568, 16605, 16643, 16680, 16717
	.hword	16754, 16791, 16828, 16865, 16902, 16939, 16976, 17013
	.hword	17049, 17086, 17122, 17159, 17195, 17232, 17268, 17304
	.hword	17341, 17377, 17413, 17449, 17485, 17521, 17557, 17593
	.hword	17628, 17664, 17700, 17735, 17771, 17806, 17842, 17877

	.hword	17912, 17947, 17982, 18018, 18053, 18088, 18123, 18157
	.hword	18192, 18227, 18262, 18296, 18331, 18365, 18400, 18434
	.hword	18468, 18503, 18537, 18571, 18605, 18639, 18673, 18707
	.hword	18741, 18775, 18808, 18842, 18876, 18909, 18943, 18976
	.hword	19010, 19043, 19076, 19109, 19142, 19176, 19209, 19242
	.hword	19274, 19307, 19340, 19373, 19405, 19438, 19471, 19503
	.hword	19536, 19568, 19600, 19632, 19665, 19697, 19729, 19761
	.hword	19793, 19825, 19857, 19888, 19920, 19952, 19983, 20015

	.hword	20046, 20078, 20109, 20140, 20172, 20203, 20234, 20265
	.hword	20296, 20327, 20358, 20389, 20419, 20450, 20481, 20511
	.hword	20542, 20572, 20603, 20633, 20664, 20694, 20724, 20754
	.hword	20784, 20814, 20844, 20874, 20904, 20934, 20963, 20993
	.hword	21023, 21052, 21082, 21111, 21140, 21170, 21199, 21228
	.hword	21257, 21286, 21315, 21344, 21373, 21402, 21431, 21460
	.hword	21488, 21517, 21546, 21574, 21603, 21631, 21659, 21688
	.hword	21716, 21744, 21772, 21800, 21828, 21856, 21884, 21912

	.hword	21939, 21967, 21995, 22022, 22050, 22077, 22105, 22132
	.hword	22159, 22187, 22214, 22241, 22268, 22295, 22322, 22349
	.hword	22376, 22403, 22429, 22456, 22483, 22509, 22536, 22562
	.hword	22589, 22615, 22641, 22668, 22694, 22720, 22746, 22772
	.hword	22798, 22824, 22850, 22875, 22901, 22927, 22952, 22978
	.hword	23004, 23029, 23054, 23080, 23105, 23130, 23156, 23181
	.hword	23206, 23231, 23256, 23281, 23306, 23330, 23355, 23380
	.hword	23404, 23429, 23454, 23478, 23502, 23527, 23551, 23575

	.hword	23600, 23624, 23648, 23672, 23696, 23720, 23744, 23768
	.hword	23792, 23815, 23839, 23863, 23886, 23910, 23933, 23957
	.hword	23980, 24003, 24027, 24050, 24073, 24096, 24119, 24142
	.hword	24165, 24188, 24211, 24234, 24256, 24279, 24302, 24324
	.hword	24347, 24369, 24392, 24414, 24436, 24459, 24481, 24503
	.hword	24525, 24547, 24569, 24591, 24613, 24635, 24657, 24679
	.hword	24700, 24722, 24744, 24765, 24787, 24808, 24830, 24851
	.hword	24872, 24894, 24915, 24936, 24957, 24978, 24999, 25020

	.hword	25041, 25062, 25083, 25104, 25125, 25145, 25166, 25186
	.hword	25207, 25228, 25248, 25268, 25289, 25309, 25329, 25349
	.hword	25370, 25390, 25410, 25430, 25450, 25470, 25490, 25509
	.hword	25529, 25549, 25569, 25588, 25608, 25627, 25647, 25666
	.hword	25686, 25705, 25724, 25744, 25763, 25782, 25801, 25820
	.hword	25839, 25858, 25877, 25896, 25915, 25934, 25952, 25971
	.hword	25990, 26008, 26027, 26046, 26064, 26082, 26101, 26119
	.hword	26138, 26156, 26174, 26192, 26210, 26228, 26246, 26264

	.hword	26282, 26300, 26318, 26336, 26354, 26371, 26389, 26407
	.hword	26424, 26442, 26459, 26477, 26494, 26512, 26529, 26546
	.hword	26564, 26581, 26598, 26615, 26632, 26649, 26666, 26683
	.hword	26700, 26717, 26734, 26751, 26767, 26784, 26801, 26817
	.hword	26834, 26850, 26867, 26883, 26900, 26916, 26932, 26949
	.hword	26965, 26981, 26997, 27013, 27029, 27046, 27062, 27077
	.hword	27093, 27109, 27125, 27141, 27157, 27172, 27188, 27204
	.hword	27219, 27235, 27250, 27266, 27281, 27297, 27312, 27327

	.hword	27343, 27358, 27373, 27388, 27403, 27419, 27434, 27449
	.hword	27464, 27479, 27493, 27508, 27523, 27538, 27553, 27567
	.hword	27582, 27597, 27611, 27626, 27640, 27655, 27669, 27684
	.hword	27698, 27712, 27727, 27741, 27755, 27769, 27783, 27797
	.hword	27812, 27826, 27840, 27854, 27867, 27881, 27895, 27909
	.hword	27923, 27937, 27950, 27964, 27978, 27991, 28005, 28018
	.hword	28032, 28045, 28059, 28072, 28085, 28099, 28112, 28125
	.hword	28138, 28152, 28165, 28178, 28191, 28204, 28217, 28230

	.hword	28243, 28256, 28269, 28282, 28294, 28307, 28320, 28333
	.hword	28345, 28358, 28370, 28383, 28396, 28408, 28421, 28433
	.hword	28445, 28458, 28470, 28482, 28495, 28507, 28519, 28531
	.hword	28543, 28556, 28568, 28580, 28592, 28604, 28616, 28627
	.hword	28639, 28651, 28663, 28675, 28687, 28698, 28710, 28722
	.hword	28733, 28745, 28756, 28768, 28779, 28791, 28802, 28814
	.hword	28825, 28837, 28848, 28859, 28870, 28882, 28893, 28904
	.hword	28915, 28926, 28937, 28948, 28959, 28970, 28981, 28992

	.hword	29003, 29014, 29025, 29036, 29046, 29057, 29068, 29079
	.hword	29089, 29100, 29111, 29121, 29132, 29142, 29153, 29163
	.hword	29174, 29184, 29194, 29205, 29215, 29225, 29236, 29246
	.hword	29256, 29266, 29276, 29286, 29297, 29307, 29317, 29327
	.hword	29337, 29347, 29357, 29366, 29376, 29386, 29396, 29406
	.hword	29416, 29425, 29435, 29445, 29454, 29464, 29474, 29483
	.hword	29493, 29502, 29512, 29521, 29531, 29540, 29550, 29559
	.hword	29568, 29578, 29587, 29596, 29605, 29615, 29624, 29633

	.hword	29642, 29651, 29660, 29669, 29678, 29687, 29696, 29705
	.hword	29714, 29723, 29732, 29741, 29750, 29759, 29768, 29776
	.hword	29785, 29794, 29802, 29811, 29820, 29828, 29837, 29846
	.hword	29854, 29863, 29871, 29880, 29888, 29897, 29905, 29913
	.hword	29922, 29930, 29938, 29947, 29955, 29963, 29971, 29980
	.hword	29988, 29996, 30004, 30012, 30020, 30028, 30036, 30044
	.hword	30052, 30060, 30068, 30076, 30084, 30092, 30100, 30108
	.hword	30115, 30123, 30131, 30139, 30147, 30154, 30162, 30170

	.hword	30177, 30185, 30192, 30200, 30208, 30215, 30223, 30230
	.hword	30238, 30245, 30252, 30260, 30267, 30275, 30282, 30289
	.hword	30297, 30304, 30311, 30318, 30326, 30333, 30340, 30347
	.hword	30354, 30361, 30369, 30376, 30383, 30390, 30397, 30404
	.hword	30411, 30418, 30425, 30432, 30438, 30445, 30452, 30459
	.hword	30466, 30473, 30479, 30486, 30493, 30500, 30506, 30513
	.hword	30520, 30526, 30533, 30540, 30546, 30553, 30559, 30566
	.hword	30572, 30579, 30585, 30592, 30598, 30605, 30611, 30617

	.hword	30624, 30630, 30637, 30643, 30649, 30655, 30662, 30668
	.hword	30674, 30680, 30687, 30693, 30699, 30705, 30711, 30717
	.hword	30723, 30729, 30735, 30741, 30747, 30753, 30759, 30765
	.hword	30771, 30777, 30783, 30789, 30795, 30801, 30807, 30812
	.hword	30818, 30824, 30830, 30836, 30841, 30847, 30853, 30858
	.hword	30864, 30870, 30875, 30881, 30887, 30892, 30898, 30903
	.hword	30909, 30915, 30920, 30926, 30931, 30936, 30942, 30947
	.hword	30953, 30958, 30964, 30969, 30974, 30980, 30985, 30990

	.hword	30996, 31001, 31006, 31011, 31017, 31022, 31027, 31032
	.hword	31037, 31043, 31048, 31053, 31058, 31063, 31068, 31073
	.hword	31078, 31083, 31088, 31093, 31098, 31103, 31108, 31113
	.hword	31118, 31123, 31128, 31133, 31138, 31143, 31147, 31152
	.hword	31157, 31162, 31167, 31172, 31176, 31181, 31186, 31191
	.hword	31195, 31200, 31205, 31209, 31214, 31219, 31223, 31228
	.hword	31232, 31237, 31242, 31246, 31251, 31255, 31260, 31264
	.hword	31269, 31273, 31278, 31282, 31287, 31291, 31296, 31300

	.hword	31304, 31309, 31313, 31317, 31322, 31326, 31330, 31335
	.hword	31339, 31343, 31348, 31352, 31356, 31360, 31365, 31369
	.hword	31373, 31377, 31381, 31385, 31390, 31394, 31398, 31402
	.hword	31406, 31410, 31414, 31418, 31422, 31426, 31430, 31434
	.hword	31438, 31442, 31446, 31450, 31454, 31458, 31462, 31466
	.hword	31470, 31474, 31478, 31482, 31485, 31489, 31493, 31497
	.hword	31501, 31505, 31508, 31512, 31516, 31520, 31523, 31527
	.hword	31531, 31535, 31538, 31542, 31546, 31549, 31553, 31557

	.hword	31560, 31564, 31568, 31571, 31575, 31578, 31582, 31585
	.hword	31589, 31593, 31596, 31600, 31603, 31607, 31610, 31614
	.hword	31617, 31621, 31624, 31627, 31631, 31634, 31638, 31641
	.hword	31644, 31648, 31651, 31655, 31658, 31661, 31665, 31668
	.hword	31671, 31675, 31678, 31681, 31684, 31688, 31691, 31694
	.hword	31697, 31701, 31704, 31707, 31710, 31713, 31717, 31720
	.hword	31723, 31726, 31729, 31732, 31735, 31739, 31742, 31745
	.hword	31748, 31751, 31754, 31757, 31760, 31763, 31766, 31769

	.hword	31772, 31775, 31778, 31781, 31784, 31787, 31790, 31793
	.hword	31796, 31799, 31802, 31805, 31808, 31810, 31813, 31816
	.hword	31819, 31822, 31825, 31828, 31831, 31833, 31836, 31839
	.hword	31842, 31845, 31847, 31850, 31853, 31856, 31858, 31861
	.hword	31864, 31867, 31869, 31872, 31875, 31877, 31880, 31883
	.hword	31885, 31888, 31891, 31893, 31896, 31899, 31901, 31904
	.hword	31907, 31909, 31912, 31914, 31917, 31920, 31922, 31925
	.hword	31927, 31930, 31932, 31935, 31937, 31940, 31942, 31945

	.hword	31947, 31950, 31952, 31955, 31957, 31960, 31962, 31964
	.hword	31967, 31969, 31972, 31974, 31977, 31979, 31981, 31984
	.hword	31986, 31988, 31991, 31993, 31995, 31998, 32000, 32002
	.hword	32005, 32007, 32009, 32012, 32014, 32016, 32019, 32021
	.hword	32023, 32025, 32028, 32030, 32032, 32034, 32036, 32039
	.hword	32041, 32043, 32045, 32047, 32050, 32052, 32054, 32056
	.hword	32058, 32060, 32063, 32065, 32067, 32069, 32071, 32073
	.hword	32075, 32077, 32080, 32082, 32084, 32086, 32088, 32090

	.hword	32092, 32094, 32096, 32098, 32100, 32102, 32104, 32106
	.hword	32108, 32110, 32112, 32114, 32116, 32118, 32120, 32122
	.hword	32124, 32126, 32128, 32130, 32132, 32134, 32136, 32137
	.hword	32139, 32141, 32143, 32145, 32147, 32149, 32151, 32153
	.hword	32154, 32156, 32158, 32160, 32162, 32164, 32166, 32167
	.hword	32169, 32171, 32173, 32175, 32176, 32178, 32180, 32182
	.hword	32184, 32185, 32187, 32189, 32191, 32192, 32194, 32196
	.hword	32198, 32199, 32201, 32203, 32204, 32206, 32208, 32210

	.hword	32211, 32213, 32215, 32216, 32218, 32220, 32221, 32223
	.hword	32225, 32226, 32228, 32230, 32231, 32233, 32234, 32236
	.hword	32238, 32239, 32241, 32242, 32244, 32246, 32247, 32249
	.hword	32250, 32252, 32254, 32255, 32257, 32258, 32260, 32261
	.hword	32263, 32264, 32266, 32267, 32269, 32270, 32272, 32273
	.hword	32275, 32276, 32278, 32279, 32281, 32282, 32284, 32285
	.hword	32287, 32288, 32290, 32291, 32293, 32294, 32295, 32297
	.hword	32298, 32300, 32301, 32303, 32304, 32305, 32307, 32308

	.hword	32310, 32311, 32312, 32314, 32315, 32317, 32318, 32319
	.hword	32321, 32322, 32323, 32325, 32326, 32327, 32329, 32330
	.hword	32331, 32333, 32334, 32335, 32337, 32338, 32339, 32341
	.hword	32342, 32343, 32344, 32346, 32347, 32348, 32350, 32351
	.hword	32352, 32353, 32355, 32356, 32357, 32358, 32360, 32361
	.hword	32362, 32363, 32365, 32366, 32367, 32368, 32369, 32371
	.hword	32372, 32373, 32374, 32375, 32377, 32378, 32379, 32380
	.hword	32381, 32383, 32384, 32385, 32386, 32387, 32388, 32390

	.hword	32391, 32392, 32393, 32394, 32395, 32396, 32398, 32399
	.hword	32400, 32401, 32402, 32403, 32404, 32405, 32406, 32408
	.hword	32409, 32410, 32411, 32412, 32413, 32414, 32415, 32416
	.hword	32417, 32418, 32419, 32420, 32422, 32423, 32424, 32425
	.hword	32426, 32427, 32428, 32429, 32430, 32431, 32432, 32433
	.hword	32434, 32435, 32436, 32437, 32438, 32439, 32440, 32441
	.hword	32442, 32443, 32444, 32445, 32446, 32447, 32448, 32449
	.hword	32450, 32451, 32452, 32453, 32454, 32455, 32456, 32457

	.hword	32457, 32458, 32459, 32460, 32461, 32462, 32463, 32464
	.hword	32465, 32466, 32467, 32468, 32469, 32470, 32470, 32471
	.hword	32472, 32473, 32474, 32475, 32476, 32477, 32478, 32478
	.hword	32479, 32480, 32481, 32482, 32483, 32484, 32485, 32485
	.hword	32486, 32487, 32488, 32489, 32490, 32491, 32491, 32492
	.hword	32493, 32494, 32495, 32496, 32496, 32497, 32498, 32499
	.hword	32500, 32500, 32501, 32502, 32503, 32504, 32505, 32505
	.hword	32506, 32507, 32508, 32509, 32509, 32510, 32511, 32512

	.hword	32512, 32513, 32514, 32515, 32516, 32516, 32517, 32518
	.hword	32519, 32519, 32520, 32521, 32522, 32522, 32523, 32524
	.hword	32525, 32525, 32526, 32527, 32528, 32528, 32529, 32530
	.hword	32530, 32531, 32532, 32533, 32533, 32534, 32535, 32535
	.hword	32536, 32537, 32538, 32538, 32539, 32540, 32540, 32541
	.hword	32542, 32542, 32543, 32544, 32544, 32545, 32546, 32547
	.hword	32547, 32548, 32549, 32549, 32550, 32551, 32551, 32552
	.hword	32552, 32553, 32554, 32554, 32555, 32556, 32556, 32557

	.hword	32558, 32558, 32559, 32560, 32560, 32561, 32561, 32562
	.hword	32563, 32563, 32564, 32565, 32565, 32566, 32566, 32567
	.hword	32568, 32568, 32569, 32569, 32570, 32571, 32571, 32572
	.hword	32572, 32573, 32574, 32574, 32575, 32575, 32576, 32577
	.hword	32577, 32578, 32578, 32579, 32579, 32580, 32581, 32581
	.hword	32582, 32582, 32583, 32583, 32584, 32585, 32585, 32586
	.hword	32586, 32587, 32587, 32588, 32588, 32589, 32590, 32590
	.hword	32591, 32591, 32592, 32592, 32593, 32593, 32594, 32594

	.hword	32595, 32595, 32596, 32596, 32597, 32598, 32598, 32599
	.hword	32599, 32600, 32600, 32601, 32601, 32602, 32602, 32603
	.hword	32603, 32604, 32604, 32605, 32605, 32606, 32606, 32607
	.hword	32607, 32608, 32608, 32609, 32609, 32610, 32610, 32610
	.hword	32611, 32611, 32612, 32612, 32613, 32613, 32614, 32614
	.hword	32615, 32615, 32616, 32616, 32617, 32617, 32617, 32618
	.hword	32618, 32619, 32619, 32620, 32620, 32621, 32621, 32622
	.hword	32622, 32622, 32623, 32623, 32624, 32624, 32625, 32625

	.hword	32626, 32626, 32626, 32627, 32627, 32628, 32628, 32629
	.hword	32629, 32629, 32630, 32630, 32631, 32631, 32631, 32632
	.hword	32632, 32633, 32633, 32634, 32634, 32634, 32635, 32635
	.hword	32636, 32636, 32636, 32637, 32637, 32638, 32638, 32638
	.hword	32639, 32639, 32639, 32640, 32640, 32641, 32641, 32641
	.hword	32642, 32642, 32643, 32643, 32643, 32644, 32644, 32644
	.hword	32645, 32645, 32646, 32646, 32646, 32647, 32647, 32647
	.hword	32648, 32648, 32649, 32649, 32649, 32650, 32650, 32650

	.hword	32651, 32651, 32651, 32652, 32652, 32652, 32653, 32653
	.hword	32654, 32654, 32654, 32655, 32655, 32655, 32656, 32656
	.hword	32656, 32657, 32657, 32657, 32658, 32658, 32658, 32659
	.hword	32659, 32659, 32660, 32660, 32660, 32661, 32661, 32661
	.hword	32662, 32662, 32662, 32663, 32663, 32663, 32663, 32664
	.hword	32664, 32664, 32665, 32665, 32665, 32666, 32666, 32666
	.hword	32667, 32667, 32667, 32668, 32668, 32668, 32668, 32669
	.hword	32669, 32669, 32670, 32670, 32670, 32671, 32671, 32671

;==============================================================================
;	Feedback Filter IIR coefficient table
;==============================================================================
; The delays are fed through 6dB/oct IIR RC filters before being recirculated.
; The RC cutoffs are variable. This table maps from the LOWPASS_CV and
; HIGHPASS_CVs to the required coefficient.
FeedbackFilter:
	.hword	316, 320, 325, 329, 334, 338, 343, 348
	.hword	352, 357, 362, 367, 372, 377, 382, 387
	.hword	393, 398, 403, 409, 414, 420, 426, 431
	.hword	437, 443, 449, 455, 462, 468, 474, 481
	.hword	487, 494, 500, 507, 514, 521, 528, 535
	.hword	543, 550, 557, 565, 573, 580, 588, 596
	.hword	604, 613, 621, 629, 638, 647, 655, 664
	.hword	673, 682, 691, 701, 710, 720, 730, 740

	.hword	750, 760, 770, 780, 791, 802, 812, 823
	.hword	835, 846, 857, 869, 881, 892, 904, 917
	.hword	929, 942, 954, 967, 980, 993, 1007, 1020
	.hword	1034, 1048, 1062, 1076, 1091, 1106, 1120, 1135
	.hword	1151, 1166, 1182, 1198, 1214, 1230, 1247, 1263
	.hword	1280, 1297, 1315, 1332, 1350, 1368, 1387, 1405
	.hword	1424, 1443, 1462, 1482, 1502, 1522, 1542, 1563
	.hword	1584, 1605, 1626, 1648, 1670, 1692, 1715, 1737

	.hword	1761, 1784, 1808, 1832, 1856, 1881, 1906, 1931
	.hword	1957, 1983, 2009, 2035, 2062, 2090, 2117, 2145
	.hword	2174, 2202, 2232, 2261, 2291, 2321, 2352, 2383
	.hword	2414, 2446, 2478, 2510, 2543, 2577, 2611, 2645
	.hword	2680, 2715, 2750, 2786, 2823, 2860, 2897, 2935
	.hword	2973, 3012, 3051, 3091, 3131, 3172, 3213, 3254
	.hword	3297, 3339, 3383, 3426, 3471, 3516, 3561, 3607
	.hword	3653, 3701, 3748, 3796, 3845, 3894, 3944, 3995

	.hword	4046, 4098, 4150, 4203, 4257, 4311, 4366, 4421
	.hword	4478, 4534, 4592, 4650, 4709, 4768, 4829, 4890
	.hword	4951, 5013, 5076, 5140, 5204, 5270, 5336, 5402
	.hword	5470, 5538, 5607, 5676, 5747, 5818, 5890, 5963
	.hword	6036, 6111, 6186, 6262, 6339, 6416, 6495, 6574
	.hword	6654, 6735, 6817, 6900, 6983, 7068, 7153, 7239
	.hword	7326, 7414, 7503, 7593, 7683, 7775, 7867, 7961
	.hword	8055, 8150, 8246, 8343, 8441, 8540, 8640, 8741

	.hword	8843, 8946, 9049, 9154, 9260, 9366, 9474, 9582
	.hword	9692, 9802, 9914, 10026, 10140, 10254, 10369, 10486
	.hword	10603, 10721, 10840, 10960, 11082, 11204, 11327, 11451
	.hword	11576, 11702, 11829, 11957, 12086, 12216, 12346, 12478
	.hword	12611, 12744, 12879, 13014, 13151, 13288, 13426, 13565
	.hword	13705, 13846, 13987, 14130, 14273, 14417, 14562, 14708
	.hword	14855, 15002, 15151, 15300, 15449, 15600, 15751, 15903
	.hword	16055, 16209, 16363, 16517, 16672, 16828, 16985, 17142

	.hword	17299, 17457, 17616, 17775, 17934, 18094, 18255, 18416
	.hword	18577, 18738, 18900, 19063, 19225, 19388, 19551, 19714
	.hword	19877, 20041, 20205, 20368, 20532, 20696, 20860, 21024
	.hword	21187, 21351, 21515, 21678, 21841, 22004, 22167, 22329
	.hword	22491, 22653, 22815, 22976, 23136, 23296, 23456, 23615
	.hword	23773, 23931, 24088, 24244, 24400, 24555, 24709, 24862
	.hword	25014, 25166, 25316, 25466, 25614, 25761, 25908, 26053
	.hword	26197, 26340, 26482, 26622, 26761, 26899, 27036, 27171

	.hword	27305, 27437, 27568, 27697, 27825, 27951, 28076, 28199
	.hword	28321, 28440, 28559, 28675, 28790, 28903, 29015, 29124
	.hword	29232, 29338, 29443, 29545, 29646, 29745, 29842, 29937
	.hword	30030

	
;==============================================================================
;	Tempo LED decay rate table
;==============================================================================
; The Tempo LED fades out at a rate which varies along with the delay time.
; This table provides the mapping from TEMPO(0-2047) to BRIGHT_DEC.
	TempoLookup:
	.hword	2048, 1366, 683, 456, 342, 274, 228, 196
	.hword	171, 152, 137, 125, 114, 106, 98, 92
	.hword	86, 81, 76, 72, 69, 66, 63, 60
	.hword	57, 55, 53, 51, 49, 48, 46, 45
	.hword	43, 42, 41, 40, 38, 37, 36, 36
	.hword	35, 34, 33, 32, 32, 31, 30, 30
	.hword	29, 28, 28, 27, 27, 26, 26, 25
	.hword	25, 24, 24, 24, 23, 23, 23, 22

	.hword	22, 22, 21, 21, 21, 20, 20, 20
	.hword	19, 19, 19, 19, 18, 18, 18, 18
	.hword	18, 17, 17, 17, 17, 17, 16, 16
	.hword	16, 16, 16, 16, 15, 15, 15, 15
	.hword	15, 15, 14, 14, 14, 14, 14, 14
	.hword	14, 14, 13, 13, 13, 13, 13, 13
	.hword	13, 13, 12, 12, 12, 12, 12, 12
	.hword	12, 12, 12, 12, 12, 11, 11, 11

	.hword	11, 11, 11, 11, 11, 11, 11, 11
	.hword	11, 10, 10, 10, 10, 10, 10, 10
	.hword	10, 10, 10, 10, 10, 10, 10, 10
	.hword	9, 9, 9, 9, 9, 9, 9, 9
	.hword	9, 9, 9, 9, 9, 9, 9, 9
	.hword	9, 9, 9, 8, 8, 8, 8, 8
	.hword	8, 8, 8, 8, 8, 8, 8, 8
	.hword	8, 8, 8, 8, 8, 8, 8, 8

	.hword	8, 8, 8, 8, 7, 7, 7, 7
	.hword	7, 7, 7, 7, 7, 7, 7, 7
	.hword	7, 7, 7, 7, 7, 7, 7, 7
	.hword	7, 7, 7, 7, 7, 7, 7, 7
	.hword	7, 7, 7, 7, 6, 6, 6, 6
	.hword	6, 6, 6, 6, 6, 6, 6, 6
	.hword	6, 6, 6, 6, 6, 6, 6, 6
	.hword	6, 6, 6, 6, 6, 6, 6, 6

	.hword	6, 6, 6, 6, 6, 6, 6, 6
	.hword	6, 6, 6, 6, 6, 6, 6, 6
	.hword	6, 6, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 5, 5

	.hword	5, 5, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 5, 5
	.hword	5, 5, 5, 5, 5, 5, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4

	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	4, 4, 4, 4, 4, 4, 4, 4

	.hword	4, 4, 4, 4, 4, 4, 4, 4
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3

	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3

	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3

	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 3, 3, 3, 3, 3
	.hword	3, 3, 3, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2

	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 2, 2
	.hword	2, 2, 2, 2, 2, 2, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1

	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1
	.hword	1, 1, 1, 1, 1, 1, 1, 1


;==============================================================================
;	Delay Time control response table
;==============================================================================
; With a linear control it's hard to get good control of the shortest delays.
; Too much of the range is taken up by long delays (3/4rs is over 1 sec). This
; table bends the response closer to a octave-based exponential, and gives much
; more control over the short end.
DelayTime:
	.hword	2, 3, 3, 4, 4, 5, 5, 6
	.hword	6, 7, 7, 8, 8, 9, 9, 10
	.hword	10, 11, 11, 12, 12, 13, 13, 14
	.hword	14, 15, 15, 16, 16, 17, 17, 18
	.hword	18, 19, 19, 20, 20, 21, 21, 22
	.hword	22, 23, 23, 24, 24, 25, 25, 26
	.hword	26, 27, 27, 28, 28, 29, 29, 30
	.hword	30, 31, 32, 32, 33, 33, 34, 34

	.hword	35, 35, 36, 36, 37, 37, 38, 38
	.hword	39, 39, 40, 40, 41, 41, 42, 42
	.hword	43, 43, 44, 44, 45, 45, 46, 46
	.hword	47, 47, 48, 48, 49, 49, 50, 50
	.hword	51, 52, 52, 53, 53, 54, 54, 55
	.hword	55, 56, 56, 57, 57, 58, 58, 59
	.hword	59, 60, 60, 61, 61, 62, 62, 63
	.hword	63, 64, 64, 65, 65, 66, 66, 67

	.hword	68, 68, 69, 69, 70, 70, 71, 71
	.hword	72, 72, 73, 73, 74, 74, 75, 75
	.hword	76, 76, 77, 77, 78, 78, 79, 79
	.hword	80, 80, 81, 82, 82, 83, 83, 84
	.hword	84, 85, 85, 86, 86, 87, 87, 88
	.hword	88, 89, 89, 90, 90, 91, 91, 92
	.hword	93, 93, 94, 94, 95, 95, 96, 96
	.hword	97, 97, 98, 98, 99, 99, 100, 100

	.hword	101, 101, 102, 103, 103, 104, 104, 105
	.hword	105, 106, 106, 107, 107, 108, 108, 109
	.hword	109, 110, 111, 111, 112, 112, 113, 113
	.hword	114, 114, 115, 115, 116, 116, 117, 117
	.hword	118, 119, 119, 120, 120, 121, 121, 122
	.hword	122, 123, 123, 124, 124, 125, 126, 126
	.hword	127, 127, 128, 128, 129, 129, 130, 130
	.hword	131, 131, 132, 133, 133, 134, 134, 135

	.hword	135, 136, 136, 137, 137, 138, 139, 139
	.hword	140, 140, 141, 141, 142, 142, 143, 143
	.hword	144, 145, 145, 146, 146, 147, 147, 148
	.hword	148, 149, 150, 150, 151, 151, 152, 152
	.hword	153, 153, 154, 155, 155, 156, 156, 157
	.hword	157, 158, 158, 159, 160, 160, 161, 161
	.hword	162, 162, 163, 164, 164, 165, 165, 166
	.hword	166, 167, 168, 168, 169, 169, 170, 170

	.hword	171, 171, 172, 173, 173, 174, 174, 175
	.hword	176, 176, 177, 177, 178, 178, 179, 180
	.hword	180, 181, 181, 182, 182, 183, 184, 184
	.hword	185, 185, 186, 187, 187, 188, 188, 189
	.hword	189, 190, 191, 191, 192, 192, 193, 194
	.hword	194, 195, 195, 196, 197, 197, 198, 198
	.hword	199, 200, 200, 201, 201, 202, 203, 203
	.hword	204, 204, 205, 206, 206, 207, 207, 208

	.hword	209, 209, 210, 211, 211, 212, 212, 213
	.hword	214, 214, 215, 215, 216, 217, 217, 218
	.hword	219, 219, 220, 220, 221, 222, 222, 223
	.hword	224, 224, 225, 226, 226, 227, 227, 228
	.hword	229, 229, 230, 231, 231, 232, 233, 233
	.hword	234, 235, 235, 236, 237, 237, 238, 238
	.hword	239, 240, 240, 241, 242, 242, 243, 244
	.hword	244, 245, 246, 246, 247, 248, 248, 249

	.hword	250, 251, 251, 252, 253, 253, 254, 255
	.hword	255, 256, 257, 257, 258, 259, 259, 260
	.hword	261, 262, 262, 263, 264, 264, 265, 266
	.hword	267, 267, 268, 269, 269, 270, 271, 272
	.hword	272, 273, 274, 274, 275, 276, 277, 277
	.hword	278, 279, 280, 280, 281, 282, 283, 283
	.hword	284, 285, 286, 286, 287, 288, 289, 289
	.hword	290, 291, 292, 292, 293, 294, 295, 296

	.hword	296, 297, 298, 299, 299, 300, 301, 302
	.hword	303, 303, 304, 305, 306, 307, 307, 308
	.hword	309, 310, 311, 312, 312, 313, 314, 315
	.hword	316, 317, 317, 318, 319, 320, 321, 322
	.hword	322, 323, 324, 325, 326, 327, 328, 328
	.hword	329, 330, 331, 332, 333, 334, 335, 336
	.hword	336, 337, 338, 339, 340, 341, 342, 343
	.hword	344, 345, 346, 347, 347, 348, 349, 350

	.hword	351, 352, 353, 354, 355, 356, 357, 358
	.hword	359, 360, 361, 362, 363, 364, 365, 366
	.hword	367, 368, 369, 370, 371, 372, 373, 374
	.hword	375, 376, 377, 378, 379, 380, 381, 382
	.hword	383, 384, 385, 386, 388, 389, 390, 391
	.hword	392, 393, 394, 395, 396, 397, 399, 400
	.hword	401, 402, 403, 404, 405, 406, 408, 409
	.hword	410, 411, 412, 413, 415, 416, 417, 418

	.hword	419, 421, 422, 423, 424, 426, 427, 428
	.hword	429, 431, 432, 433, 434, 436, 437, 438
	.hword	439, 441, 442, 443, 445, 446, 447, 449
	.hword	450, 451, 453, 454, 455, 457, 458, 459
	.hword	461, 462, 464, 465, 466, 468, 469, 471
	.hword	472, 474, 475, 476, 478, 479, 481, 482
	.hword	484, 485, 487, 488, 490, 491, 493, 495
	.hword	496, 498, 499, 501, 502, 504, 506, 507

	.hword	509, 510, 512, 514, 515, 517, 519, 520
	.hword	522, 524, 525, 527, 529, 530, 532, 534
	.hword	536, 537, 539, 541, 543, 545, 546, 548
	.hword	550, 552, 554, 556, 557, 559, 561, 563
	.hword	565, 567, 569, 571, 573, 575, 577, 579
	.hword	581, 583, 585, 587, 589, 591, 593, 595
	.hword	597, 599, 601, 603, 605, 607, 609, 612
	.hword	614, 616, 618, 620, 623, 625, 627, 629

	.hword	631, 634, 636, 638, 641, 643, 645, 648
	.hword	650, 652, 655, 657, 660, 662, 665, 667
	.hword	669, 672, 674, 677, 680, 682, 685, 687
	.hword	690, 692, 695, 698, 700, 703, 706, 708
	.hword	711, 714, 717, 719, 722, 725, 728, 731
	.hword	733, 736, 739, 742, 745, 748, 751, 754
	.hword	757, 760, 763, 766, 769, 772, 775, 778
	.hword	781, 784, 788, 791, 794, 797, 800, 804

	.hword	807, 810, 814, 817, 820, 824, 827, 831
	.hword	834, 838, 841, 845, 848, 852, 855, 859
	.hword	862, 866, 870, 873, 877, 881, 885, 888
	.hword	892, 896, 900, 904, 908, 912, 916, 920
	.hword	924, 928, 932, 936, 940, 944, 948, 952
	.hword	957, 961, 965, 969, 974, 978, 982, 987
	.hword	991, 996, 1000, 1005, 1009, 1014, 1018, 1023
	.hword	1028, 1032, 1037, 1042, 1047, 1052, 1056, 1061

	.hword	1066, 1071, 1076, 1081, 1086, 1091, 1096, 1101
	.hword	1107, 1112, 1117, 1122, 1128, 1133, 1138, 1144
	.hword	1149, 1155, 1160, 1166, 1171, 1177, 1183, 1188
	.hword	1194, 1200, 1206, 1212, 1217, 1223, 1229, 1235
	.hword	1241, 1247, 1254, 1260, 1266, 1272, 1279, 1285
	.hword	1291, 1298, 1304, 1311, 1317, 1324, 1330, 1337
	.hword	1344, 1351, 1357, 1364, 1371, 1378, 1385, 1392
	.hword	1399, 1406, 1414, 1421, 1428, 1435, 1443, 1450

	.hword	1458, 1465, 1473, 1480, 1488, 1496, 1504, 1511
	.hword	1519, 1527, 1535, 1543, 1551, 1560, 1568, 1576
	.hword	1584, 1593, 1601, 1610, 1618, 1627, 1636, 1644
	.hword	1653, 1662, 1671, 1680, 1689, 1698, 1707, 1716
	.hword	1726, 1735, 1744, 1754, 1763, 1773, 1783, 1792
	.hword	1802, 1812, 1822, 1832, 1842, 1852, 1862, 1873
	.hword	1883, 1893, 1904, 1914, 1925, 1936, 1947, 1957
	.hword	1968, 1979, 1990, 2002, 2013, 2024, 2035, 2047
; We should never reach here.
.end
