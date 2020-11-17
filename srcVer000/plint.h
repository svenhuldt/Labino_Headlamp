/**
 * Pin and MACRO definitions for the LABINO project
 * 
 * 1: C1,2- from Vref
 * 2: C1+ from pin2 C2+ from pin1
 *
-
Pin	27: 	RA0	PLUSSW
		28:	RA1	DAC out (via OP1)
	 	1:		RA2	(AN2) UVL SW in
		2:		RA3	(AN3)	VVL SW in
		3:		RA4	L3 + Status
		4: 	RA5	(AN4)	battery_check CHECK
		6:		RA7 (sic)	POWER Lout ON
		7:		RA6 (sic)	BATT LOAD ON
TRISA =  0b00101111
ANSELA=  0b00101110
WPUA =	0b11000001
		18: 	RB0	(AN12) Charge detect
		19:	RB1	Current sense (AN10)
		20:	RB2	Temp lamphus (AN8)
		21:	RB3	L4 out (G.P.)
		22:	RB4	MINUSSW	
		23:	RB5	L1 + Status
		24:	RB6	PGM
		25:	RB7	PGM
TRISB =  0b11010111
ANSELB=	0b00000111
WPUB	=	0b00010000

		26:	MCLR	PGM
		28: 	RA1	AN1	DAC output to FB

		8: RC0 	MASTER ON
		9:	RC1	UVL ut
		10: RC2	WHL ut
		11: RC3	WLL ut
		12: RC4	SUMMER
		13: RC5	LoutSense (AN17)
		14,15: RC6,7 Serial + MEM switch
TRISC = 	0b11100000
ANSELC=	0b00100000
WPUC	=	0b00011111
**/
/* Logical macros: */
#define	LM_UVL_SW_ON	((CMOUT & 0x01) == 0)
#define	LM_WL_SW_ON		((CMOUT & 0x02) == 0)
#define	LM_PLUS_SW_ON	((PORTA & 0x01) == 0)
#define	LM_MINUS_SW_ON	((PORTB & 0x10) == 0)
#define	LM_MEM_SW_ON	((PORTC & 0x80) == 0)

/*Set macros: */
#define	SM_UVL_OFF	{LATC |= 0x02;}	// Pin 9, RC1
#define	SM_WLH_OFF	{LATC |= 0x04;}	// Pin 10, RC2
#define	SM_WLL_OFF	{LATC |= 0x08;}	// Pin 11, RC3

#define	SM_UVL_ON	{LATC &= ~0x02;}	// Pin 9, RC1
#define	SM_WLH_ON	{LATC &= ~0x04;}	// Pin 10, RC2
#define	SM_WLL_ON	{LATC &= ~0x08;}	// Pin 11, RC3

#define	SM_L1_ON		{LATB &= ~0x20;}	// Pin 23, RB5
#define	SM_L2_ON		{LATB &= ~0x08;}	// Pin 21, RB3
#define	SM_L3_ON		{LATA &= ~0x10;}	// Pin 3, RA4

#define	SM_L1_OFF	{LATB |=  0x20;}	// Pin 23, RB5
#define	SM_L2_OFF	{LATB |=  0x08;}	// Pin 21, RB3
#define	SM_L3_OFF	{LATA |=  0x10;}	// Pin 3, RA4

#define	SM_MASTER_ON	{LATC &= ~0x01;}	// Pin 8, RC0
#define	SM_MASTER_OFF	{LATC |=  0x01;}	// Pin 8

#define	SM_IC5_ON	{LATA &= ~0x80;}	// Pin 6, RA7
#define	SM_IC5_OFF	{LATA |=  0x80;}	//  

#define	SM_BATT_LOAD_ON	{LATA &= ~0x40;}	// Pin 7, RA6
#define	SM_BATT_LOAD_OFF	{LATA |=  0x40;}	// Pin 7
// Summer, AN16, RC4, PWM3
// #define	SM_SUMMER_ON	{PWM3CON |=  0x80;}
// #define	SM_SUMMER_OFF	{PWM3CON &= ~0x80;}
/** Obsolete - summer has its own oscillator!
*'	#define	SM_SUMMER_ON	{T4CON |=  0x80;}
*	#define	SM_SUMMER_OFF	{T4CON &= ~0x80;}
**/
#define	SM_SUMMER_ON	{LATC |=  0b00010000;}	// Pin 12 RC4
#define	SM_SUMMER_OFF	{LATC &= ~0b00010000;}

typedef union
{
	struct SWB
	{
		unsigned on			:1;
		unsigned make 		:1;
		unsigned brake 	:1;
		unsigned onlong 	:1;
		unsigned prev		:1;
		unsigned pad 		:3;
	};
	unsigned char SwitchByte;
} SWITCH_t;
