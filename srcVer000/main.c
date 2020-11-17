
/*******************************************************************************
 * File:  Labino main
 * Processor: PIC16LF1613
 * MACROS and constants in all CAPS, variables in mixed case.
 * 
 * Configuration:
 * - internal oscillator @ ? MHz
 * - BOR disabled (spikes may occur)
 * - WDT enavled but disabled in SLEEP
 * - ADC operated from ? clock
 * 
 * Manual switch input (to comparators) also act as on-switch
 * **************************************************************************/

#include <xc.h>     

#define DEBUG 1

// CONFIG
// #pragma config FOSC = INTOSC, WDTE = NSLEEP, PWRTE = OFF, MCLRE = OFF, CP = OFF, BOREN = OFF
#pragma config FOSC = INTOSC, WDTE = ON, WDTCPSE, PWRTE = OFF, MCLRE = OFF, CP = OFF, BOREN = ON
#pragma config CLKOUTEN = OFF, IESO = OFF, FCMEN = ON
#pragma config WRT = OFF, PPS1WAY = OFF, ZCDDIS = OFF, PLLEN = OFF, STVREN = ON, BORV = LO
#pragma config LPBOR = OFF, LVP = ON
#include <stdio.h>
#include <stdlib.h>
#include "plint.h"
#include "HEFlash.h"


typedef unsigned char byte;
typedef unsigned int uint_16;

#define _XTAL_FREQ 16000000
#define ADC_RESO 1024
#define RATED_VOLTAGE 5
#define RATED_SPEED 2400
#define SUPPLY_VOLTAGE 9

#define SW_TC 20    // delay for on long switch (high intensity white led, rapid +/-)

/* ADC channels: */
#define BATT    4
#define SHUNT  10
#define CHARGE 12
#define Vout   17
#define LAST   99

const byte  BATTix   = 0;
const byte  SHUNTix  = 1;
const byte  CHARGEix = 2;
const byte  Voutix   = 3;
const byte  LASTix   = 4;

const byte ADCnum[] = {BATT,SHUNT,CHARGE,Vout,LAST};
byte HEFbuffer[64];

void    ADC_Initialization (void);
void    DAC_Initialization (void);
void    PI_Calculation(void);
void    Pin_Manager(void);
void    FVR_Initialization(void);
void    Int_Initialization(void);
void    EUSART_Initialization(void);
void    sendChar(byte);
byte    getChar(void);
void    sendString(const char *);
void    sendStringV(char *);
void    sendInt(uint_16 );
void    TMR1_Initialization(void);
void    TMR1_Reload(void);
void    Check_Overcurrent (void);
int     battery_check(void);

void    sleep(void);
void    wake_up(void);
void    HEF_write(void);
void    HEF_read(void);

const byte SLEEP_TC = 50;
const uint_16 BATT_LEVEL_HIGH = 650;
const uint_16 BATT_LEVEL_MEDIUM = 500;
const byte DAC_MIN = 1;
const byte DAC_MAX = 255;

// Persisten data in HEF-memory:
byte UVL_max, LOL_max, HIL_max;
    
// Ram variables (non-persistent)
byte UVL_to, WL_to, PLUS_to, MINUS_to, MEM_to, sleep_to, sysState, old_sysState;
byte flag_100ms, flag_500ms, to_100ms, to_500ms;
byte beep_toggle, summer_beeps, killflag;
byte ADCindex;

uint_16 ADCvalue, Timer1ReloadVal, UVL_current, WLL_current, WLH_current;
uint_16 ADCdata[5], DAC_setting;
// #pragma section = HEF
uint_16 UVL_set_current, WLL_set_current, WLH_set_current;
uint_16 UVL_DAC_setting, WLL_DAC_setting, WLH_DAC_setting;
// #pragma section=code
const uint_16 UVL_CURRENT_MAX = 0x0fff, UVL_CURRENT_MIN = 10;
const uint_16 WLL_CURRENT_MAX = 0x0fff, WLL_CURRENT_MIN = 10;
const uint_16 WLH_CURRENT_MAX = 0x0fff, WLH_CURRENT_MIN = 10;

SWITCH_t UVL, WL, PLUS, MINUS, MEM;


/**
 * Interrupt from input switch pins are enabled during sleep.
 * During normal operation only timer and ADC interrupts are enabled.
 */

static byte savesize;


inline void    clrwdt(void)
{
#asm
    CLRWDT
#endasm
}

void    delay(uint_16 t)
{
    byte bb;
    int ii;

    while(t > 0)
    {
        bb = 255;
        ii = 7;
        --t;
           clrwdt();
        while(bb > 0)
        {
            if(+ii > --bb)ii--;            
        }
    }
}

static void __interrupt Int_(void);

enum STATUS
{
   ST_WAKEUP,  // Awake from sleep
   ST_SLEEP,   // System sleep
   ST_OFF,     // OFF, but awake
   ST_UVL_ON,    // UV LED on
   ST_WLL_ON,    // Low intensitu LED on
   ST_WLH_ON     // Hi intensity LED on
};

void    HEF_write(void)
{
    byte bix = 0;  
    HEFbuffer[bix++] = (UVL_set_current >> 8) & 0x00ff;
    HEFbuffer[bix++] = UVL_set_current & 0x00ff;
    HEFbuffer[bix++] = (WLL_set_current >> 8) & 0x00ff;
    HEFbuffer[bix++] = WLL_set_current & 0x00ff;
    HEFbuffer[bix++] = (WLH_set_current >> 8) & 0x00ff;
    HEFbuffer[bix++] = WLH_set_current & 0x00ff;
    HEFbuffer[bix++] = UVL_DAC_setting;   // DAC settings are 8 bit!
    HEFbuffer[bix++] = WLL_DAC_setting;   // DAC settings are 8 bit!
    HEFbuffer[bix++] = WLH_DAC_setting;   // DAC settings are 8 bit!
    savesize = bix;
 //     uint_16 UVL_set_current, WLL_set_current, WLH_set_current;
//      uint_16 UVL_DAC_setting, WLL_DAC_setting, WLH_DAC_setting;
//      char HEFLASH_writeBlock( char radd, char* data, char count)
    HEFLASH_writeBlock(0, HEFbuffer, bix); 
}

void    HEF_read()
{
//    char    HEFLASH_readBlock( char* buffer, char radd, char savesize);
    HEFLASH_readBlock( HEFbuffer, 0, savesize);
    byte bix = 0;   // Flash is 14-bit wide, so high locations are not used
    UVL_set_current =  *((uint_16 *)(&HEFbuffer[bix]));
    bix += 2;
    WLL_set_current =  *((uint_16 *)(&HEFbuffer[bix]));
    bix += 2;
    WLH_set_current = *((uint_16 *)(&HEFbuffer[bix]));
    bix += 2;
    UVL_DAC_setting = HEFbuffer[bix++];   // DAC settings are 8 bit!
    WLL_DAC_setting = HEFbuffer[bix++];   // DAC settings are 8 bit!
    WLH_DAC_setting = HEFbuffer[bix++];   // DAC settings are 8 bit!

 //     uint_16 UVL_set_current, WLL_set_current, WLH_set_current;
// uint_16 UVL_DAC_setting, WLL_DAC_setting, WLH_DAC_setting;
    HEFLASH_writeBlock(0,HEFbuffer,bix);
}

int    battery_check(void)
{
/* Load battery: */
    SM_BATT_LOAD_ON
    SM_MASTER_ON
    delay(1000); // ms
    ADCON0 = (byte)(BATT<<2 | 0x01);
    ADRESH = 0x00;
    ADRESL = 0x00;
    ADCON2 = 0; // No auto trigger
    PIR1bits.ADIF = 0;   // Clear ADC interrupt flag
    PIE1bits.ADIE = 0;    // Do not enable ADC interrupt
    ADCON0bits.GO = 1;
/* Wait for completetion */
    while (PIR1bits.ADIF == 0)continue;
    ADCvalue = (uint_16)((ADRESH << 8) | ADRESL);
    SM_BATT_LOAD_OFF
#if DEBUG
    sendChar(':');
    sendInt(ADCvalue);
    sendChar(0x0d);
#endif
    killflag = 0;
    if (ADCvalue > BATT_LEVEL_HIGH)summer_beeps = 2;
    else if (ADCvalue > BATT_LEVEL_MEDIUM)
        summer_beeps = 4;
    else 
    {
        killflag = 1;
        summer_beeps = 12;
    }
    SM_BATT_LOAD_OFF
    clrwdt();
    while(summer_beeps-- > 0)
    {
        delay(300);
        clrwdt();

        SM_SUMMER_ON
//        sendChar('b');
        delay(300);
        clrwdt();
        SM_SUMMER_OFF
    }
    if (killflag)
    {
        while(1)
        {
            SM_MASTER_OFF
            delay(200);
        }
    }
    PIR1bits.ADIF = 0;   // Clear ADC interrupt flag
    PIE1bits.ADIE = 1;    // Enable ADC interrupt
    ADCON0bits.GO = 0;     // Set every 100ms
    return(0);
}

static void __interrupt Int_(void)
{
    if (PIR1bits.TMR1IF)
    {
        if (++to_100ms > 99)
        {
            to_100ms = 0;
            flag_100ms = 1;
        }
//        TMR1_Reload();
        TMR1H = Timer1ReloadVal >> 8;
        TMR1L = Timer1ReloadVal;
//        Check_Overcurrent ();                // Check for an Overcurrent
        PIR1bits.TMR1IF = 0;
    }
 
 /** 
 * ADC channel 4, Pin4, RA5, Battery voltage
 * ADC channel 10, Pin19, RB1, Current measurement
 * ADC channel 12, Pin18, RB0, Charge voltage
 **/
    if (PIR1bits.ADIF)
    {
        ADCvalue = (uint_16)((ADRESH << 8) | ADRESL);
        PIR1bits.ADIF = 0;
        ADCdata[ADCindex] = ADCvalue;
        if(++ADCindex >= 3) // or: ADCnum[ADCindex] == LAST
        {
            ADCindex = 0;
            ADCON0bits.CHS = ADCnum[ADCindex];
            ADCON0bits.ADON = 1;
            ADCON0bits.GO = 0;        
        }
        else
        {
            ADCON0bits.CHS = ADCnum[ADCindex];
            ADCON0bits.ADON = 1;
            ADCON0bits.GO = 1;        
        }
        PIR1bits.ADIF = 0;
    }        
    PIR1 = 0;
    PIR2 = 0;
    PIR3 = 0;
}

#if 0
void    sleep(void)
{
//    Turn off GIE:
    GIE = 0;
// Turn off WDT, TMR1 and ADC:

// Switch to LFOSC (?):

    #asm 
        SLEEP
    #endasm
}
void     wake_up(void)
{
// Restore from HEF (obsolete?)
// Switch to HFOSC (?)
// Turn on WDT, TMR1, DAC and ADC:
    SLEEP_to = 0;
    GIE = 1;
}
#endif

/**
 * This is the place where buttons are checked and settings are updated
 * 
 **/
static byte tut, ttogg;

void    every_100ms(void)
{
    if(ttogg)
    {
        ttogg = 0;
        SM_L1_ON
    }
    else
    {
        ttogg = 0xff;
        SM_L1_OFF
        sendChar('*');
    }
/****************************
 * Set up switch structures: UVL WL PLUS MINUS MEM
 * Only strictly valid for the duration of this function!
 * **************************/

    UVL_to = 0;
    if(LM_UVL_SW_ON)
    {
        UVL.on = 1;
        if (UVL.prev == 0)UVL.make = 1;
        if (++UVL_to > SW_TC)UVL.onlong = 1;
        UVL.brake = 0;
    }
    else
    {
        UVL.on = 0;
        UVL_to = 0;
        UVL.onlong = 0;
        if (UVL.prev)UVL.brake = 1;
        UVL.make = 0;
    }
  
    UVL.prev = UVL.on;

    if(LM_WL_SW_ON)
    {
        WL.on = 1;
        if (WL.prev == 0)WL.make = 1;
        if (++WL_to > SW_TC)WL.onlong = 1;
        WL.brake = 0;
    }
    else
    {
        WL.on = 0;
        WL_to = 0;
        WL.onlong = 0;
        if (WL.prev)WL.brake = 1;
        WL.make = 0;
    }
    WL.prev = WL.on;
    
    PLUS.make = MINUS.make = MEM.make = 0;

    if(LM_PLUS_SW_ON)
    {
        PLUS.on = 1;
        if (PLUS.prev == 0)PLUS.make = 1;
        if (++PLUS_to > SW_TC)PLUS.onlong = 1;
        PLUS.brake = 0;
    }
    else
    {
        PLUS.on = 0;
        PLUS_to = 0;
        PLUS.onlong = 0;
        if (PLUS.prev)PLUS.brake = 1;
        PLUS.make = 0;
    }
    PLUS.prev = PLUS.on;

    if(LM_MINUS_SW_ON)
    {
        MINUS.on = 1;
        if (MINUS.prev == 0)MINUS.make = 1;
        if (++MINUS_to > SW_TC)MINUS.onlong = 1;
        MINUS.brake = 0;
    }
    else
    {
        MINUS.on = 0;
        MINUS_to = 0;
        MINUS.onlong = 0;
        if (MINUS.prev)MINUS.brake = 1;
        MINUS.make = 0;
    }
    MINUS.prev = MINUS.on;
    
    if(LM_MEM_SW_ON)
    {
        MEM.on = 1;
        if (MEM.prev == 0)MEM.make = 1;
        if (++MEM_to > SW_TC)MEM.onlong = 1;
        MEM.brake = 0;
    }
    else
    {
        MEM.on = 0;
        MEM_to = 0;
        MEM.onlong = 0;
        if (MEM.prev)MEM.brake = 1;
        MEM.make = 0;
    }
    MEM.prev = MEM.on;  

/*
    if (PLUS.make)sendChar('+');
    if (MINUS.make)sendChar('-');
    if (MEM.make)sendChar('M');
*/

#if 1
    switch (sysState)
    {
        case ST_UVL_ON:
            if (ADCdata[SHUNTix] < UVL_set_current)
            { 
                if (DAC_setting >= DAC_MIN)--DAC_setting;
            }
            else if (ADCdata[SHUNTix] > UVL_set_current)
            {
                if(DAC_setting <= DAC_MAX)++DAC_setting;
            }
// Adjustments:
            if ((PLUS.make) || (PLUS.onlong))
            {
#if DEBUG                
             sendInt(ADCdata[SHUNTix]);   sendChar('=');sendInt(UVL_set_current);
             sendChar('&');sendInt(DAC_setting); sendChar(0x0d);
#endif
                if (UVL_set_current < UVL_CURRENT_MAX)UVL_set_current++;
            }
            else if ((MINUS.make) || (MINUS.onlong))
            {
                if (UVL_set_current > UVL_CURRENT_MIN)UVL_set_current--;
            }
            else if (MEM.make)
            {
                UVL_DAC_setting = DAC_setting;
                HEF_write();
            }
            break;
        case ST_WLL_ON:
            if (ADCdata[SHUNTix] < WLL_set_current)
            { 
                if (DAC_setting >= DAC_MIN)--DAC_setting;
            }
            else if (ADCdata[SHUNTix] > WLL_set_current)
            {
                if(DAC_setting <= DAC_MAX)++DAC_setting;
            }
// Adjustments:
            if ((PLUS.make) || (PLUS.onlong))
            {
                if (WLL_set_current < WLL_CURRENT_MAX)WLL_set_current++;
            }
            else if ((MINUS.make) || (MINUS.onlong))
            {
                if (WLL_set_current > WLL_CURRENT_MIN)WLL_set_current--;
            }
            else if (MEM.make)
            {
                WLL_DAC_setting = DAC_setting;
                HEF_write();
            }
            break;
        case ST_WLH_ON:
            if (ADCdata[SHUNTix] < WLH_set_current)
            { 
                if (DAC_setting >= DAC_MIN)--DAC_setting;
            }
            else if (ADCdata[SHUNTix] > WLH_set_current)
            {
                if(DAC_setting <= DAC_MAX)++DAC_setting;
            }
// Adjustments:
            if ((PLUS.make) || (PLUS.onlong))
            {
                if (WLH_set_current < WLH_CURRENT_MAX)WLH_set_current++;
            }
            else if ((MINUS.make) || (MINUS.onlong))
            {
                if (WLH_set_current > WLH_CURRENT_MIN)WLL_set_current--;
            }
            else if (MEM.make)
            {
                WLH_DAC_setting = DAC_setting;
                HEF_write();
            }
            break;
        default:
            break;
    }
#endif
 /* Restart ADC:s */
    ADCindex = 0;
    PIR1bits.ADIF = 0;
    ADCON0bits.GO = 1;  // ADCCON0 |= 2;    // Set GO bit

/* Set DAC: */
//    tut += 4;  // +++
 //   DAC_setting = tut; // +++
    DAC1CON1 = DAC_setting;
}

static byte togg;
void    every_500ms(void)
{
#if DEBUG
    if (togg)
    {
        togg = 0;
    }
    else
    {
        togg = 0xff;
/*
        sendChar(sysState +'0');
        sendChar(':');
        sendInt(UVL_set_current);
        sendChar(' ');
        sendInt(WLL_set_current);
        sendChar(' ');
        sendInt(WLH_set_current);
*/
        sendChar(':');
        sendInt(ADCdata[SHUNTix]);
        sendChar(' ');
        sendInt(DAC_setting);
        sendChar(0x0d);

    }
#endif

}

void main(void)
{
    INTCON = 0;
    Pin_Manager(); // Init oscillator
    SM_MASTER_ON
    SM_BATT_LOAD_OFF
    clrwdt();
    delay(10);
    FVR_Initialization();
    DAC_Initialization ();
    ADC_Initialization();
#if DEBUG
        EUSART_Initialization();
        sendString("<Starting>");
        sendChar(0x0D);
        sendChar(0x0A);
#endif
        clrwdt();
       UVL.SwitchByte = UVL_to = 0;
        WL.SwitchByte = WL_to = 0;
        PLUS.SwitchByte = PLUS_to = 0;
        MINUS.SwitchByte = MINUS_to = 0;
        MEM.SwitchByte = MEM_to = 0;
/* nix
    if (LM_UVL_SW_ON)UVL.make = 1;
    else if (LM_WL_SW_ON)WL.make = 1;
*/
    battery_check(); // Set beep(s)
/*  variable init: */
    ADCindex = 0;
    sysState = ST_WAKEUP;
    sysState = ST_OFF;
    sleep_to = 0;
    clrwdt();
    TMR1_Initialization();

    HEF_read();
 /*
    UVL_set_current = 377; WLL_set_current = 1280; WLH_set_current = 350;
    UVL_DAC_setting = 150; WLL_DAC_setting = 180; WLH_DAC_setting = 160;
 */
    Int_Initialization();

/* ============================== MAIN LOOP =============================== */
    while(1)
    {
        clrwdt();   // Move later?

        if (flag_100ms)
        {
            flag_100ms = 0;
            ++sleep_to;
//                sendChar(CMOUT+'0');
#if DEBUG
            if (sysState != old_sysState)
            {
                sendChar(old_sysState+'0');
                sendChar('-');
                sendChar(sysState+'0');
                old_sysState = sysState;
            }
#endif

            every_100ms();
            if (++to_500ms > 4)
            {
                to_500ms = 0;
                every_500ms();
            }
        }

        switch (sysState)
        {
            case ST_WAKEUP:
                sleep_to = 0;
                SM_UVL_OFF
                SM_WLL_OFF
                SM_WLH_OFF
// Ramp ?
// Init Oscillator, disable sw interrupts

                sysState = ST_OFF;
                break;
            case ST_OFF:
                if (UVL.make)
                {
                    UVL.make = 0;
                    sysState = ST_UVL_ON;
                    DAC_setting = UVL_DAC_setting;
                }
                else if (WL.make)
                {
                    WL.make = 0;
                    sysState = ST_WLL_ON;
                    DAC_setting = WLL_DAC_setting;
                }
                else if (WL.onlong)
                {
                    WL.onlong = 0;
                    sysState = ST_WLH_ON;
                    DAC_setting = WLH_DAC_setting;
                }
                
                if (sleep_to > SLEEP_TC)
                {
#if 0
// +++              SW int on
// +++              disable _WCHAR_T_DEFINED
                    GIE = 0;
//                      Enter sleep mode
                    sysState = ST_SLEEP;
/**
 * Sleep not used - device is turned off instead
 *                     sleep();
 *                     wake_up();
 **/
#endif
// +++                    GIE = 0;
                    SM_MASTER_OFF
                    delay(100);
// +++                    while(1)clrwdt();
                }    
                break;
            case ST_UVL_ON:
                sleep_to = 0;
                SM_WLL_OFF
                SM_WLH_OFF
                if (UVL.make | WL.make)
                {
                    UVL.make = WL.make = 0;
                    SM_UVL_OFF
                    sysState = ST_OFF;
                }
                else
                {
                    SM_UVL_ON
                }
                break;

            case ST_WLL_ON:
                sleep_to = 0;
                SM_UVL_OFF
                SM_WLH_OFF
                if (UVL.make | WL.make)
                {
                    UVL.make = WL.make = 0;
                    SM_WLL_OFF
                    sysState = ST_OFF;
                }
                else if (WL.onlong)
                {
                    WL.onlong = 0;           
                    SM_WLL_OFF
                    sysState = ST_WLH_ON;
                }
                else
                {
                    SM_WLL_ON
                }
                break;
 
            case ST_WLH_ON:
                sleep_to = 0;
                SM_UVL_OFF
                SM_WLL_OFF
                if (UVL.make | WL.make)
                {
                    UVL.make = WL.make = 0;
                    SM_WLH_OFF
                    sysState = ST_OFF;
                }
                else
                {
                    SM_WLH_ON
                }
                break;
            default:
            sysState = ST_WAKEUP;
               sleep_to = 0;
        } // End switch statement
    }
}
/*******************************************************************************
* Pin_Manager, intializes the I/O Ports as follows:
* 1. 16MHz FOSC
*******************************************************************************/
void Pin_Manager()
{
    LATA=LATB=LATC=0xff;
    OSCCON = 0b01111000;                		// 16MHz Clock Speed
    while ((OSCSTAT & 0b0001000) == 0)continue; // Wait for oscillator start

    OPTION_REG = 0b01111111;

/* Switches: WL and UVL - - mem switch later! */
            // UVL  RA2
            // WL   RA3
/* See plint.h for details */    
    TRISA  = 0b00101111; // Obsolete : pins1,2 -> RA2,3 are analog by default     
    ANSELA = 0b00101110; // RA4 - L3
    WPUA   = 0b11000001;
    LATA   = 0b01111111;  // Turn on IC5 - pin 7 

    TRISB  = 0b11010111; // RB4 +SW , RB3 - L2, RB5 - L1
    WPUB   = 0b00010000;
    ANSELB = 0b00000111;
    LATB   = 0b11111111;

/* Comparator set-up: */
    CM1CON0 = 0b10001110;
    CM2CON0 = 0b10001110;
    CM1CON1 = 0b00001110; // pos input is C1IN1, neg is FVR
    CM2CON1 = 0b00000110; // pos input is C2IN0, neg is FVR
    
    /* OPA1 setup: */
    OPA1CON  = 0b11010010;   // Input fron DAC1, output on AN1, pin 28
    OPA2CON  = 0; // OPA2 disabled

/* Power on, LED out ports: */
            // Port RC0 - Power ON
            // Port RC3, White led low
            // Port RC2, White led high
            // Port RC1, UV led low
            // Port RC4, Summer

    TRISC   = 	0b11100000;
    ANSELC  =	0b00100000;
    WPUC	=	0b00011111;
    LATC    =   0b00111111;
    SM_SUMMER_OFF

//    ODCON is 0 by default
/* Timer 2: 
    RC4PPS = 0x0e;          // PWM3 connected to RC4
    T2CON = 0b00000011;     // FOSC/4, 1:64 Prescaler ger 62.5 kHz
    TMR2 = 71;          // Reload;
    SM_SUMMER_OFF
*/
    WDTCON = 0b00010111;    // Enable watch-dog
//    clrwdt();
}

/*******************************************************************************
*******************************************************************************/
void Int_Initialization()
{
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.TMR1IE = 1;
}

/*******************************************************************************
 * FVR_Initialization, intializes the FVR Module as follows:
 * 1. Enable FVR, FVR Output is internally connected to Comparator 1 and DAC
 *
 * FVR produces 1.024V that becomes the reference voltage for both
 * COMP1 And DAC1
*******************************************************************************/
void FVR_Initialization()
{
/*
    FVRCONbits.TSEN = 0;                	// Disable Temp Indicator
    FVRCONbits.TSRNG = 0;               	// Low temp Range Mode
    FVRCONbits.CDAFVR1 = 00;             	// FVR. gain 1x to COMP1 and DAC1
    FVRCONbits.CDAFVR0 = 01;
    FVRCONbits.ADFVR1 = 00;                  // Gain: 1x => to ADC
    FVRCONbits.ADFVR0 = 01;
    FVRCONbits.FVREN = 1;               	// Enable FVR
 */
    FVRCON = 0b11000100;

  }

/*******************************************************************************
 * DAC_Initialization
 * DAC1 is used to provide feedback to the LTC3119 DC out voltage.
 * DAC is set in a simple increment/decrement loop to match the set 
 * value in HEF (set at factory).
 * 
 * DAC Output = [(FVR - Vss)x(DAC1CON1/256)] + Vss
 * .021 = [(1.024 - 0)X(DAC1CON1/256)]+0
 *
 * DAC1CON1 = 5
 * 5 = 0.021
*******************************************************************************/
void DAC_Initialization ()
{
//    DAC1CON0 = 0b10001000;  // FVR buffer 2 is reference
    DAC1CON0 = 0b10000000;  // Vdd is reference
    DAC1CON1 = 0x05;        // Set value
}
/*******************************************************************************
 * ADC_Initialization, intializes the ADC Module as follows:
 * 1. AN10, RB1, Pin 19: LED current sense
 * 2. AN4, RA5, Pin4: Battery check
 * 3. AN12, RB0, Pin 18: CHarge detection - best digital? +++
 * 
 * #define BATT    4
 * #define SHUNT  10
 * #define CHARGE 12
 * #define  Vout  17
 * #define LAST   99
 * 
 *****************************************************************************/
 
void ADC_Initialization ()
{
/* Set pins to analog: */
    TRISA  |=  0b00100000;
    TRISB  |=  0b00000011;
    ANSELA |=  0b00100000;
    ANSELB |=  0b00000011;
    WPUA   &= ~0b00100000;
    WPUB   &= ~0b00000011;
//    ADCON1 = 0b11100011;    // FOSC/64, Right Justified, ref is FVR
    ADCON1 = 0b11100000;    // FOSC/64, Right Justified, ref is Vdd
    ADCindex = 0;
    ADCON0 = (byte)(ADCnum[ADCindex]<<2 | 0x01);
    ADRESH = 0x00;
    ADRESL = 0x00;
    ADCON2 = 0; // No auto trigger
    PIR1bits.ADIF = 0;   // Clear ADC interrupt flag
    PIE1bits.ADIE = 0;    // Do not enable ADC interrupts yet!
}
/*******************************************************************************
 * TMR1_Initialization, intializes the SMT Module as follows:
 * 1. Enable TMR1 with 1:8 Prescaler and FOSC/4 Clock Source
 * 2. 100mS TMR1 Flag Time
*******************************************************************************/
void TMR1_Initialization()
{
    T1CON = 0b00100000;     // FOSC/4, 1:4 Prescaler (gives 1MHz)
    T1GCON = 0x00;
    Timer1ReloadVal = 0xffff-999;
    TMR1_Reload();
    TMR1ON = 1;
    T1CONbits.TMR1ON = 1;
    PIR1bits.TMR1IF = 0;     // Reset interrupt flag
    PIE1bits.TMR1IE = 1;     // Enable timer 1 interrupt
}

/*******************************************************************************
 * TMR1_Reload, writes the Timer1ReloadVal to the Timer1 Register
*******************************************************************************/
void TMR1_Reload()
{
    TMR1H = Timer1ReloadVal >> 8;
    TMR1L = Timer1ReloadVal;
}

void    EUSART_Initialization()
{
/*  Pin 14, RC6, AN18, TX
    Pin 15, RC7, AN19, RX
*/
    ANSELCbits.ANSC6 = 0;
    ANSELCbits.ANSC7 = 0;
    RC6PPS = 0x14; // Source is TX
    RXPPS = 0b00010111; // Rx on Port C, pin 7
    TX1STA = 0b00100100;
    RC1STA = 0b10010000;
    BAUD1CON = 0b00000000;
    SP1BRGL =   25; // Gives 9600 (?)
    SP1BRGH =   0;
}

byte getChar(void)
{
    while(PIR1bits.RCIF == 0)continue;
    return(RC1REG);
}

void    sendChar(byte b)
{
    while ((TX1STA & 0x02) == 0)continue;
    TX1REG = b;
}

void    sendString(const char *p)
{
    while (*p != 0)
    {
        sendChar(*p);
        ++p;
    }
}

void    sendStringV(char *p)
{
    while (*p != 0)
    {
        sendChar(*p);
        ++p;
    }
}

void    sendInt(uint_16 num)
{
   uint_16 j;
   j=num/10000;
   num -= j*10000;
   sendChar(j+'0');
   j=num/1000;
   num -= j*1000;
   sendChar(j+'0');
   j=num/100;
   num -= j*100;
   sendChar(j+'0');
   j=num/10;
   num -= j*10;
   sendChar(j+'0');
  
   sendChar(num+'0');
}
