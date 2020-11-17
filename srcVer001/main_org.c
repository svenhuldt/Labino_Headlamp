
/*
 * File:   		AN1779_Source_Code.c
 * File Version: 	2.0
 * Date:		June 2014
 * Description:
 * Source Code for Application Note 1779
 *
 * Generation Information :
 *                          Device : PIC16F1613
 *                          Compiler : XC8 v1.30
 *                          MPLAB : MPLAB X 2.0
 *
 *
 * Motor Specification Used:
 * Nominal Speed/Rated Speed = 2400RPM +/- 10%
 * Nominal Voltage/Rated Voltage = 5Volts
 * Stall Current = 0.21A
 *
 */


/****************************************************************************
Copyright (c) 2014 released Microchip Technology Inc. All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).
You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
****************************************************************************/

#include <xc.h>

#pragma config FOSC = INOSC
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF

#pragma config WRT = OFF
#pragma config ZCDDIS = ON
#pragma config PLLEN = OFF
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LPBOR = OFF
#pragma config LVP = ON

#pragma config WDTCPS = WDTCPS1F
#pragma config WDTE = OFF
#pragma config WDTCWS = WDTCWSSW
#pragma config WDTCCS = SWC


#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 16000000
#define ADC_RESO 1024
#define RATED_VOLTAGE 5
#define RATED_SPEED 2400
#define SUPPLY_VOLTAGE 9
#define COUNT_CLOCKSOURCE 31000
#define TEMP_CALIBRATION 60			// Single-Point Calibration Value

#define  KSP1 0.1
#define  KSI1 0.05

void Pin_Manager(void);
void Int_Initialization(void);
void FVR_Initialization (void);
void ADC_Initialization (void);
void Comp_Initialization (void);
void DAC_Initialization (void);
void CCP_Initialization (void);
void CWG_Initialization (void);
void SMT_Initialization (void);
void PWM_Calculation(void);
void DesiredSpeed_Calculation(void);
void Get_ActualSpeed (void);
void PI_Calculation(void);
void TMR1_Initialization(void);
void TMR1_Reload(void);
void Check_Overcurrent (void);
static void __interrupt Int_(void);

int ADCValue;

float PWMResult;
int PWMValue;

int SpeedReference;
float SpeedCalculation;

float SpeedMeasurement;
int SpeedActual;

int SpeedError;
int SpeedIntegral;
int DutyCycle;
char SpeedCounter;
float PropOut;
float IntegOut;
int PropValue;
int IntegValue;
int PIValue;

unsigned int TempResult;
unsigned int Timer1ReloadVal;

static void __interrupt Int_(void)
{
    if (PIR1bits.TMR1IF == 1)
    {
        ADCON0bits.CHS = 0x1D;                  // Switch to Temp Sense Input
        __delay_us(300);
        ADCON0bits.ADON = 1;
        ADCON0bits.GO = 1;
        __delay_us(300);

        TempResult = ADRES;

// Check_TempFault, checks for a over temperature occurances in the system
// then initializes a shutdown to prevent damage to the controller
        TempResult += TEMP_CALIBRATION;
            if (TempResult > 571)               // Shutdown at 42Deg Celcius
            {
                CWG1AS0bits.SHUTDOWN = 1;
                PORTAbits.RA1 = 0;
                while(TempResult > 571)
                {
                    PORTAbits.RA2 = 1;
                    __delay_ms(100);
                    PORTAbits.RA2 = 0;
                    __delay_ms(100);
                }
            }
        ADC_Initialization ();              //Switch Back to AN3 Input
        ADCON0bits.ADON = 1;
        ADCON0bits.GO = 1;
        __delay_us(300);
        ADCValue = ADRES;

        TMR1_Reload();
        Check_Overcurrent ();                // Check for an Overcurrent
        INTCONbits.GIE = 1;
        PIR1bits.TMR1IF = 0;
    }
    INTCONbits.GIE = 1;                     // Enable Interrupt
    PIR1bits.TMR1IF = 0;                    // Reset TMR1 Flag
}

void main()
{
    Pin_Manager();
    FVR_Initialization();
    Comp_Initialization ();
    DAC_Initialization ();
    CCP_Initialization ();
    CWG_Initialization ();
    SMT_Initialization ();
    TMR1_Initialization();
    Int_Initialization();
    ADC_Initialization();

    DutyCycle = 0;
    PIValue = 0;
    SpeedError = 0;
    SpeedIntegral = 0;

    CWG1CON0bits.CWG1EN = 1;             // Turn ON CWG
    T2CONbits.TMR2ON = 1;                // Turn ON PWM
    T1CONbits.TMR1ON = 1;                // Start TMR1
    SMT1CON1bits.SMT1GO = 1;             // Acquire SMT Data

    while(1)
    {
        ADCON0bits.ADON = 1;
        ADCON0bits.GO = 1;
        __delay_us(300);

        while(ADCON0bits.GO)
        {
        }
        ADCValue = ADRES;

        PWM_Calculation();
        DesiredSpeed_Calculation();
        Get_ActualSpeed ();
        PI_Calculation();

        PORTAbits.RA3 = 0;
        PORTAbits.RA2 = 0;
        PORTAbits.RA1 = 1;                             

        if(TMR2IF)
            TMR2IF = 0;

        if (CMOUTbits.MC1OUT)
            CWG1CON0bits.CWG1MODE0 = 0;
        else
            CWG1CON0bits.CWG1MODE0 = 1;
    }
}
/*******************************************************************************
* Pin_Manager, intializes the I/O Ports as follows:
* 1. 16MHz FOSC
* 2. RA0 = (Input)Hall Sensor Output is input to COMP1
* (Positive Input Channel)
* 3. RC1 = (Input)Current Sensing Output is input to COMP2
* (Negative Input Channel)
* 4. RA4 = (Input)Analog Input Potentiometer
* 4. RA1 = (Output)LED Indicator (Green) - Normal Condition
* 5. RA2 = (Output)LED Indicator (Red) - Over Temperature
* 6. RA3 = (Output)LED Indicator (Red) - Over Current
*******************************************************************************/
void Pin_Manager()
{
    OSCCON = 0b01111000;                		// 16MHz Clock Speed
    LATA = 0b00000000;
    TRISA = 0b00010001;
    ANSELA = 0b00010000;
    TRISC = 0b00000010;
    ANSELC = 0b00000010;
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
* Comp_Initialization, intializes the Comparator Module as follows:
*
* COMP1:
* 1. Enable Comparator, Operate at Normal Power, Higher Speed Mode
* 2. Comparator Positive Input = C1IN+ Pin (RA0)
* 3. Comparator Negative Input = FVR_Buffer2 (Internally Connected to FVR
* Module)
*
* Check if the Hall Sensor will get high, if it is high it will send a high
* pulse to command that the next commutation must be initialized
*
* COMP2:
* 1. Comparator Positive Input Channel = Internally Connected to DAC_Output
* 2. Comparator Negative Input Channel = C2IN1- (RC1 = Shunt Resistor)
*
* COMP2 is used for auto-shutdown of CWG Module
* Auto-shutdown input sources are active low. Low Pulse will trigger an
* auto-shutdown sequence.
*
* DAC_Output = 0.021 Volts
* Rsense = RC1
*******************************************************************************/
void Comp_Initialization()
{
    CM1CON0 = 0b10000100;
    CM1CON1 = 0b00000110;
    CM2CON0 = 0b10000100;
    CM2CON1 = 0b00010001;
}

/*******************************************************************************
 * FVR_Initialization, intializes the FVR Module as follows:
 * 1. Enable FVR, FVR Output is internally connected to Comparator 1 and DAC
 *
 * FVR produces 1.024V that becomes the reference voltage for both
 * COMP1 And DAC1
*******************************************************************************/
void FVR_Initialization ()
{
    FVRCONbits.TSEN = 1;                	// Enable Temp Indicator
    FVRCONbits.TSRNG = 1;               	// High Range Mode
    FVRCONbits.FVREN = 1;               	// Enable FVR
    FVRCONbits.CDAFVR1 = 0;             	// FVR to COMP1 and DAC1
    FVRCONbits.CDAFVR0 = 1;
}

/*******************************************************************************
 * DAC_Initialization, intializes the DAC Module as follows:
 * 1. Enable DAC1 Module
 * 2. DAC is internally conected to the inverting input of the Comparator 2 Module
 * 3. DAC Source came from the FVR Buffer 2 Output (FVR Module)
 *
 * DAC is used to set the reference voltage for the Comp 2 Module.
 * The Comp 2 Module is used to detect an overcurrent flowing through the driver
 *
 * FVR = 1.024Volts
 * Motor Stall Current = 0.21
 * Driver Sense Resistor = .10 Ohms
 *
 * The voltage drop at Rsense is 21 mVolts at stall condition
 * so the DAC Output must equal to 21 mVolts
 *
 * DAC Output = [(FVR - Vss)x(DAC1CON1/256)] + Vss
 * .021 = [(1.024 - 0)X(DAC1CON1/256)]+0
 *
 * DAC1CON1 = 5
 * 5 = 0.021
*******************************************************************************/
void DAC_Initialization ()
{
    DAC1CON0 = 0b10001000;
    DAC1CON1 = 0x05;
}

/*******************************************************************************
 * CCP_Initialization, intializes the CCP Module as follows:
 * 1. Configure PWM Resoulution of 1024 (10Bit Resolution)
 * 2. PWM Period = 15625 Hz
*******************************************************************************/
void CCP_Initialization ()
{
    T2CLKCON = 0b00000000;
    T2CON = 0b00000111;
    PR2 = 0xFF;
    TMR2 = 0x00;
    PIR1bits.TMR2IF = 0;

    CCP1CON = 0b11001100;
    CCPR1 = 0x00;
}

/*******************************************************************************
 * ADC_Initialization, intializes the ADC Module as follows:
 * 1. Potentiometer = Input to ADC Module
 * 2. Vdd Voltage Reference, Right Justify
 * 3. 1uS Conversion Clock*11 TAD = 11uS ADC Conversion Time
*******************************************************************************/
void ADC_Initialization ()
{
    ADRESH = 0x00;
    ADRESL = 0x00;
    ADCON1 = 0b11010000;        		// FOSC/16, Right Justified

    ADCON0 = 0b00001100;        		// Analog Input (AN3)
    __delay_us(300);
}

/*******************************************************************************
 * CWG_Initialization, intializes the CWG Module as follows:
 * 1. CWG Output Operate in Forward/Reverse Full Bridge Mode
 * 2. Put some deadband timer to avoid current overshoot of FET
 * 3. All output is in Normal Polarity
 * 4. Enable All Output on CWGxA(RC5) ,CWGxB(RC4) ,CWGxC(RC3) ,CWGxD(RC2)
 * 5. CWG Clock = FOSC (16Mhz)
 * 6. Input Clock on CWGxB, CWGxD is the output of CCP1_Out (PWM Signal)
 * 7. Enable the shutdown state. Shutdown CWG when overcurrent occurs
 *
 * The CWG produces a Full Bridge waveform that will control the motor driver.
 * CWGxB and CWGxD can be modulated by the input signal (CCP1_Out)
*******************************************************************************/
void CWG_Initialization ()
{
    CWG1CON0 = 0x00;
    CWG1CON0 = 0b00000010;              	// Full Bridge Mode
    CWG1DBR = 0x20;                     	// Dead-band Timer
    CWG1DBF = 0x20;
    CWG1CON1 = 0b00000000;              	// Normal Polarity
    CWG1OCON1 = 0b00001111;             	// Output Enable
    CWG1CLKCON = 0b00000000;            	// FOSC Clock Source
    CWG1ISM = 0b00000011;               	// CCP1_Out

    CWG1AS0 = 0b00101000;               	// Shutdown All Ports of CWG
    CWG1AS1bits.C2AS = 1;               	// COMP2 Detection Shutdown
}

/*******************************************************************************
 * SMT_Initialization, intializes the SMT Module as follows:
 * 1. Enable SMT with 1:1 Prescaller of Clock Source
 * 2. Repeatable Period and Duty Cycle Acquisition Mode
 * 3. Acquire data from SMT Signal Source Comparator 1 Output
 * 4. SMT Clock is configured to be LFINTOSC 31kHz
 *
 * Count the numbers of SMT Clock present in 1 Period of SMT Signal Source and
 * display it in SMT1CPR Register
 *
 * SMT1CPR = SMTCLK/SMTSIGNAL
 * SMTCLK = 31KHz
 * SMTSIGNAL = Comparator Output
*******************************************************************************/
void SMT_Initialization ()
{
    SMT1CON0 = 0b10000000;              	// STM Enable
    SMT1CON1 = 0b01000010;              	// Single Acquisition
    SMT1SIG = 0b00000001;               	// Comp1 Signal
    SMT1CLK = 0b00000011;               	// LFINTOSC (31kHz Source)

    SMT1TMR = 0x000000;                 	// Clear all Counter Registers
    SMT1CPR = 0x000000;
    SMT1CPW = 0x000000;
    SMT1PR = 0x000000;
}

/*******************************************************************************
 * TMR1_Initialization, intializes the SMT Module as follows:
 * 1. Enable TMR1 with 1:8 Prescaler and FOSC/4 Clock Source
 * 2. 100mS TMR1 Flag Time
*******************************************************************************/
void TMR1_Initialization()
{
    T1CON = 0b00110000;                     // FOSC/4, 1:8 Prescaler
    T1GCON = 0x00;
    TMR1H = 0x3C;                           // 0x3CAF = 100mS to Flag
    TMR1L = 0xAF;
    Timer1ReloadVal=TMR1;                   // Load the TMR value to reload variable
}

/*******************************************************************************
 * TMR1_Reload, writes the Timer1ReloadVal to the Timer1 Register
*******************************************************************************/
void TMR1_Reload()
{
    TMR1H = Timer1ReloadVal>> 8;
    TMR1L = Timer1ReloadVal;
}


/*******************************************************************************
 * PWM_Calculation, Calculate the Duty Cycle needed based on the applied ADC Value
*******************************************************************************/
void PWM_Calculation ()
{
    PWMResult = (RATED_VOLTAGE/SUPPLY_VOLTAGE)*ADCValue;
    PWMValue = (int)PWMResult;

    DutyCycle = PWMValue + PIValue;
    CCPR1 = DutyCycle;

    if (DutyCycle > 1023)
    {
        CCPR1 = 1023;
    }
    if (DutyCycle < 110)
    {
        CCPR1 = 110;
    }
}

/*******************************************************************************
 * DesiredSpeed, determines the desired speed or speed reference based on the
 * applied ADC Value
*******************************************************************************/
void DesiredSpeed_Calculation ()
{
    SpeedCalculation = (RATED_SPEED/ADC_RESO)*ADCValue;
    SpeedReference = (int)SpeedCalculation;
}

/*******************************************************************************
 * Get_ActualSpeed, Determines the exact speed of the motor by using the value
 * in SMT1CPR for every electrical cycle
 *
 * 2 Electrical Revolution = 1 Mechanical Revolution
 * COUNTCLOCKSOURCE = 31kHz
 * No. of Poles = 4 Poles (2 Pair of North/South Pole - Single Phase Motor )
 * No. of Poles is also based on the Motor construction of your single phase motor,
 * typical construction of a single phase is consist of 2 pairs of N/S Poles
 *
 * RPM = 120*Frequency/No. of Poles
 * SMT1CPR = SMTCLK(Hz)/HALL SENSOR OUTPUT FREQUENCY(SMTSIGNAL SOURCE)
 *
 * RPM = 120*SMTSIGNAL(Hz)/4
 * ActualSpeed = 30*SMTCLK(Hz)/SMT1CPR
*******************************************************************************/
void Get_ActualSpeed ()
{
    SpeedMeasurement = (COUNT_CLOCKSOURCE/SMT1CPR)*30;
    SpeedActual = (int) SpeedMeasurement;
}

/*******************************************************************************
 * PI_Calculation, uses the PI algorith to calculate the PWM Duty Cycle that
 * will be loaded into the CCPR1 Register
*******************************************************************************/
void PI_Calculation()
{
    SpeedError = SpeedReference - SpeedActual;
    SpeedIntegral += SpeedError;

    if (SpeedIntegral > 5000)
        SpeedIntegral = 0;
    if (SpeedIntegral < -5000)
        SpeedIntegral = 0;

    PropOut = SpeedError*KSP1;
    PropValue = (int) PropOut;
    IntegOut = SpeedIntegral*KSI1;
    IntegValue = (int) IntegOut;
    PIValue = PropValue + IntegValue;
}

/*******************************************************************************
 *  Check_Overcurrent, monitor for an over current occurances.
*******************************************************************************/
void Check_Overcurrent()
{
    if (CWG1AS0bits.SHUTDOWN == 1)
    {
        while (CWG1AS0bits.SHUTDOWN)
        {
            PORTAbits.RA3 = 1;			// Fault LED
            __delay_ms(100);			// indicate over current
            PORTAbits.RA3 = 0;
            __delay_ms(100);
     	}
    }
}
