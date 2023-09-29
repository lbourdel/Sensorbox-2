/*
;   Project:       Sensorbox 1.5plus, for use with Smart EVSE
;   Date:          02 June 2021
;
;   Changes:
;   1.0  Initial release
;
;   (C) 2013-2023  Michael Stegen / Stegen Electronics
;
;	Current measurement calculations, from openenergymonitor.org
; 
;   set XC8 linker memory model settings to: double 32 bit, float 32 bit
;   extended instruction set is not used on XC8
;
;   XC8 compiler version 2.10 is used
;
;   Modbus RTU slave address is fixed to 0x0A, speed is 9600 bps 
:   
;   Input Registers (FC=04)(Read Only):
;  
;   Register  Register  
;   Address   length (16 bits)
;   0x0000      1       Sensorbox version 1.5+          = 0x001x (2 lsb mirror the 3/4 Wire and Rotation configuration data)
;                                                         0x0011 = 4 wire, CCW rotation
;                                                         0x0012 = 3 wire, CW rotation      
;                                                         0x0013 = 3 wire, CCW rotation
;                                                         0x0014 = no phase shifts  
;   0x0001      1       DSMR Version(MSB), CT's (LSB)   = 0x0003                 
;   0x0002      2       Volts L1 (32 bit floating point), Smartmeter P1 data. Unused on Sensorbox 1.5plus
;   0x0004      2       Volts L2 (32 bit floating point), Smartmeter P1 data. Unused on Sensorbox 1.5plus
;   0x0006      2       Volts L3 (32 bit floating point), Smartmeter P1 data. Unused on Sensorbox 1.5plus
;   0x0008      2       Amps L1 (32 bit floating point), Smartmeter P1 data. Unused on Sensorbox 1.5plus
;   0x000A      2       Amps L2 (32 bit floating point), Smartmeter P1 data. Unused on Sensorbox 1.5plus
;   0x000C      2       Amps L3 (32 bit floating point), Smartmeter P1 data. Unused on Sensorbox 1.5plus
;   0x000E      2       Amps L1 (32 bit floating point), CT imput 1
;   0x0010      2       Amps L2 (32 bit floating point), CT input 2
;   0x0012      2       Amps L3 (32 bit floating point), CT input 3
;
;   Holding Registers (FC=06)(Write):
;
;   Register  Register  
;   Address   length (16 bits) 
;   0x0800      1       Field rotation setting (bit 0)      00000000 0000000x -> 0= Rotation right 1= Rotation Left
;                       3/4 wire configuration (bit 1)      00000000 000000x0 -> 0= 4Wire, 1= 3Wire
;                       no phaseshifts (bit 2)              00000000 00000x00 -> 1= No phase shift
; 
; 
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include <xc.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "Sensorbox_plus.h"

// CONFIG1
#pragma config FOSC = INTOSC                                                    // Oscillator Selection Bits->INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = OFF                                                       // Watchdog Timer Enable->WDT disabled
#pragma config PWRTE = OFF                                                      // Power-up Timer Enable->PWRT disabled
#pragma config MCLRE = OFF                                                      // MCLR Pin Function Select->MCLR/VPP pin function is reset input (ignored if LVP is ON)
#ifdef CODEPROTECT
#pragma config CP = ON                                                          // Flash Program Memory Code Protection->Program memory code protection is enabled
#else
#pragma config CP = OFF                                                         // Flash Program Memory Code Protection->Program memory code protection is disabled
#endif
#pragma config BOREN = OFF                                                      // Brown-out Reset Disable->Brown-out Reset disabled
#pragma config CLKOUTEN = OFF                                                   // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config IESO = ON                                                        // Internal/External Switchover Mode->Internal/External Switchover Mode is enabled
#pragma config FCMEN = ON                                                       // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT = OFF                                                        // Flash Memory Self-Write Protection->Write protection off
#pragma config PPS1WAY = ON                                                     // Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config ZCDDIS = ON                                                      // Zero-cross detect disable->Zero-cross detect circuit is disabled at POR
#pragma config PLLEN = ON                                                       // Phase Lock Loop enable->4x PLL is always enabled
#pragma config STVREN = ON                                                      // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config BORV = LO                                                        // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LPBOR = OFF                                                      // Low-Power Brown Out Reset->Low-Power BOR is disabled

#ifdef ESP_DIRECT
#pragma config LVP = ON                                                         // Low-Voltage Programming Enable->High-voltage on MCLR/VPP is not needed for programming
#else
#pragma config LVP = OFF                                                        // Low-Voltage Programming Enable->High-voltage on MCLR/VPP is required for programming
#endif


// Global data

unsigned char Modbus_TX[100], Modbus_RX[50];
double IrmsCT[3];

#ifdef CALCULATE
double x1, x2, x3, y1, y2, y3;                                          // phase shift coefficients

double i1Lead = 16.00;     // degrees that the voltage input phase error leads the c.t.1 phase error by
double i2Lead = 16.00;     // degrees that the voltage input phase error leads the c.t.2 phase error by
double i3Lead = 16.00;     // degrees that the voltage input phase error leads the c.t.3 phase error by
double i1phaseshift, i2phaseshift, i3phaseshift;
#else

//const double yy[]= {-3.272029, -3.215480, -3.158804}; //60 samples 20.00 Ilead
//const double xx[]= {4.193797, 4.139688, 4.085416};

//const double yy[]= {-3.114637, -3.057741, -3.000724};   // 60 samples 19.00 Ilead     org optocoupler
//const double xx[]= {4.043093, 3.988535, 3.933821};

//const double yy[]= {-2.956295, -2.899069, -2.841730};  //60 samples 18.00 Ilead
//const double xx[]= {3.891157, 3.836167, 3.781027};
         
//const double yy[]= {-2.636960, -2.579127, -2.521192};   // 60 samples 16.00 Ilead       prev Alt optocoupler
//const double xx[]= {3.583776, 3.527973, 3.472030};

//const double yy[]= {-2.556609, -2.498635, -2.440563};   // 60 samples 15.50 Ilead
//const double xx[]= {3.506234, 3.450238, 3.394106};

//const double yy[]= {-2.476063, -2.417953, -2.359747};   // 60 samples 15.00 Ilead
//const double xx[]= {3.428424, 3.372240, 3.315922};

//const double yy[]= {-2.395329, -2.337086, -2.278752};   // 60 samples 14.50 Ilead
//const double xx[]= {3.350354, 3.293985, 3.237486};

const double yy[]= {-2.314412, -2.256042, -2.197583};     // 60 samples 14.00 Ilead     Alt optocoupler
const double xx[]= {3.272029, 3.215480, 3.158804};

//const double y1=-3.035582, x1=3.967276, y2=-2.978518, x2=3.912500, y3=-2.921338, x3=3.857570;  //60 samples 18.50 Ilead

#endif

unsigned char LedSeq[3] = {0,0,0};
unsigned long Timer, LedTimer, ModbusTimer=0;				// mS counter
unsigned char RXbyte, ADC_CH=0, dataready=0;
unsigned char ModbusIdx=0, ModbusTXlen=0, ModbusTXidx=0;
unsigned int T1Period=1, T1PLL=2652, sumPeriodSamples=0, PLLLock=0;          
unsigned char newsumCycle=0;
signed int sampleI[3];
unsigned char Phase[] = {PHASE0, PHASE120, PHASE240} , version = 0x10+WIRES+ROTATION;

volatile int oldsampleI1 = 0; 
//volatile unsigned long sumVsq, sumI1sq, sumI2sq, sumI3sq;
//volatile signed int sumVavg, sumI1avg, sumI2avg, sumI3avg;
volatile int24_t sumPowerA[3], sumPowerB[3];//, sumPower2A, sumPower2B, sumPower3A, sumPower3B; 
volatile unsigned char sumSamples, Samples;

//unsigned long sumPeriodVsq, sumPeriodI1sq, sumPeriodI2sq, sumPeriodI3sq;
//long sumPeriodVavg, sumPeriodI1avg, sumPeriodI2avg, sumPeriodI3avg;
//long sumPeriodPower1A, sumPeriodPower1B, sumPeriodPower2A, sumPeriodPower2B, sumPeriodPower3A, sumPeriodPower3B; 
long sumPeriodPowerA[3], sumPeriodPowerB[3]; 

const signed char voltage[]= {0,6,13,19,25,31,37,42,46,50,54,57,59,61,62,62,62,61,59,57,54,50,46,42,37,31,25,19,13,6,
                              0,-6,-13,-19,-25,-31,-37,-42,-46,-50,-54,-57,-59,-61,-62,-62,-62,-61,-59,-57,-54,-50,-46,-42,-37,-31,-25,-19,-13,-6,
                              0,6,13,19,25,31,37,42,46,50,54,57,59,61,62,62,62,61,59,57,54,50,46,42,37,31,25,19,13,6,
                              0,-6,-13,-19,-25,-31,-37,-42,-46,-50,-54,-57,-59,-61,-62,-62,-62,-61,-59,-57,-54,-50,-46,-42,-37,-31,-25,-19,-13,-6 };

volatile int lastSampleI[3], tempI;      // sample holds the raw analog read value, lastSample holds the last sample
volatile long filteredI[3], filtI_div4, tempL;
volatile unsigned long sqI, sumI[] = {0,0,0};
volatile unsigned int SumSamples2 = 0;
volatile unsigned char TotalSumReady = 0, NoLock = 0;
volatile unsigned long TotalSumI[3];

volatile uint8_t flag @ 0x70;

//uint24_t square10(uint16_t);


void __interrupt() ISR (void)
{
    while(PIR1bits.TMR1IF)                                                      // Timer 1 interrupt, called 60*50 times/sec for ADC measurements
	{
//      LED_RED_ON;
        ADCON0 = 0x0D;                                                          // Select first ADC channel (CT1)
        ADC_CH = 0;
        
        T1Period++;                                                             // count cycles   
        TMR1 = ~(T1PLL);                                                        // reset Timer 1
        
        // We also start sampling CT1 when there is no lock for alternative Irms calculations
        if (T1Period<=NUMSAMPLES || NoLock) {                                   // start ADC conversion on the selected channel.
            ADCON0bits.ADGO = 1;                                                // ADC will generate interrupt when finished.
        }

//      LED_RED_OFF;
		PIR1bits.TMR1IF = 0;                                                    // clear Timer 1 interrupt flag
	}
    
    while(IOCCFbits.IOCCF5)                                                     // External pin (RC5) interrupt triggered 50 times/sec (50Hz) (determined by line frequency)
    {
//      LED_RED_ON;
        ADCON0 = 0x0D;                                                          // Select first ADC channel (CT1)
        ADC_CH = 0;      
        if (T1Period>NUMSAMPLES) T1PLL+=10;                                     // Slow down timer interrupt
        else if (T1Period<NUMSAMPLES) T1PLL-=1;                                 // Speed up timer interrupt. to make sure we sample 'NUMSAMPLES' times per period (0.02sec)
        
        TMR1 = ~(T1PLL);                                                        // reset Timer 1

        if (sumSamples<=NUMSAMPLES) {
            ADCON0bits.ADGO = 1;                                                // start ADC conversion on the selected channel.        
        } 
        newPeriod();                                                            // Last sample in this period
                
        T1Period = 1;                                                           // Start with 1
        PLLLock = 0;
        NoLock = 0;                     // reset flag
        IOCCFbits.IOCCF5 = 0;                                                   // Clear external PIN interrupt flag
        PIR1bits.TMR1IF = 0;                                                    // also clear Timer 1 interrupt flag
//      LED_RED_OFF;
    }

 
     
#ifdef ESP_DIRECT    
    while(IOCAFbits.IOCAF1 )                                                    // External pin (RA1) interrupt
    {
        version ++;                                                             // increase version
        if (version == 0x15) version = 0x10;
        IOCAFbits.IOCAF1 = 0;                                                   // Clear Interrupt flag
    }
#endif
    
#ifndef CALCULATE   
    
    while(PIR1bits.ADIF)                                                        // ADC conversion done interrupt
    {
        if (ADC_CH == 0) {                                                      // CT1
            ADCON0 = 0x1D;                                                      // Select ADC channel CT2
//LED_GREEN_ON;
            lastSampleI[0] = sampleI[0];
            sampleI[0] = ADRES;                                                   // process CT1 sample
            ADC_CH = 1;
            ADCON0bits.ADGO = 1;                                                // start ADC conversion on CT2
            
        } else if (ADC_CH == 1) {                                               // CT2
            ADCON0 = 0x15;                                                      // Select ADC channel CT3
            lastSampleI[1] = sampleI[1];
            sampleI[1] = ADRES;
            ADC_CH = 2;
            ADCON0bits.ADGO = 1;                                                // start ADC conversion on CT3
            
        } else if (ADC_CH == 2) {                                               // CT 3
            ADCON0 = 0x0D;                                                      // Select first ADC channel (CT1)
            lastSampleI[2] = sampleI[2];
            sampleI[2] = ADRES;
            ADC_CH = 3;

            if (NoLock) {
                for (uint8_t i=0; i<3; i++) {
                
                    tempL = (long)(sampleI[i] - lastSampleI[i])<<8;             // re-scale the input change (x256)
                    tempL += filteredI[i];                                      // combine with the previous filtered value
                    filteredI[i] = tempL-(tempL>>8);                            // subtract 1/256, same as x255/256

                    filtI_div4 = filteredI[i]>>2;                               // now x64
                    // Root-mean-square method current
                    // 1) square current values
                    sqI = (unsigned long)(filtI_div4 * filtI_div4);
                    sqI = sqI >>12;                                             // scale back
                    // 2) sum
                    sumI[i] += sqI;
                }

                if (++SumSamples2 == SAMPLES) {
                
                    for (uint8_t i=0 ;i<3 ;i++) {
                        TotalSumI[i] = sumI[i];                                 // copy to variable to be used in main loop
                        sumI[i] = 0;  
                    }
                    SumSamples2 = 0;
                    TotalSumReady = 1;
                }
              
            } else {
                for (uint8_t i=0 ;i<3 ;i++) {
                    sumPowerA[i] += mul16x8s(sampleI[i]-512, voltage[T1Period-1+NUMSAMPLES-Phase[i]]); // Use stored & delayed voltage for power calculation phase 1 (0)
                    sumPowerB[i] += mul16x8s(sampleI[i]-512, voltage[T1Period-1+NUMSAMPLES-Phase[i]-1]);
                }
                sumSamples++;                                                   // counts samples per period (should be NUMSAMPLES)
            }
//            LED_GREEN_OFF; 
        }
        
        if (IOCCFbits.IOCCF5) T1PLL--;                                          // if external input was triggered already the sample loop took to much time, speed up the timer
        PIR1bits.ADIF = 0;                                                      // Clear ADC interrupt flag
    }
    
#ifndef ESP_DIRECT
    while (PIR1bits.RCIF)                                                       // Uart1 receive interrupt? RS485
    {
        RXbyte = RCREG;                                                         // copy received byte	

        if (Timer > (ModbusTimer + 3))                                          // last reception more then 3ms ago? 
        {
             ModbusIdx =0;                                                      // clear idx in RS485 RX handler
        }  
        if (ModbusIdx == 50) ModbusIdx--;                                       // max 50 bytes in buffer
  		Modbus_RX[ModbusIdx++] = RXbyte;                                        // Store received byte in buffer
        ModbusTimer=Timer;
	}
#endif
    
    if (PIR1bits.TXIF && PIE1bits.TXIE && TX1STAbits.TRMT)                      // Uart1 transmit interrupt? RS485
    {                                                                           // Also check if Transmit buffer is empty (see PIC16F170x errata)
        TXREG1 = Modbus_TX[ModbusTXidx++];
        if (ModbusTXidx == ModbusTXlen) PIE1bits.TXIE = 0;                      // disable interrupts
	}

    // Timer 4 interrupt, called 1000 times/sec
    while(PIR2bits.TMR4IF)                                      				// Timer 4 interrupt, called 1000 times/sec
	{
        Timer++;                                                                // mSec counter (overflows in 1193 hours)
        PLLLock++;
        if (LedTimer) LedTimer--;
		PIR2bits.TMR4IF = 0;                                                    // clear interrupt flag
	}
#endif //CALCULATE
}


void newPeriod(void)                                                            // end of period, calculate totals
{
    //LED_GREEN_ON;
    
    if (newsumCycle==0)                                                         // Reset by main loop
    {   
        //for (uint8_t i=0; i<3; i++) {
            sumPeriodPowerA[0] += sumPowerA[0];
            sumPeriodPowerB[0] += sumPowerB[0];
            sumPeriodPowerA[1] += sumPowerA[1];
            sumPeriodPowerB[1] += sumPowerB[1];
            sumPeriodPowerA[2] += sumPowerA[2];
            sumPeriodPowerB[2] += sumPowerB[2];
        //}
        sumPeriodSamples += sumSamples;                                         // add the sumsamples from this period (0.02sec) to the 1 second counter
                                                                                // sumPeriodCounter should be around 3000 (60 samples per period * 50 cycles) per second.
                                                                                // if a period ends early, it might be lower, this has no effect on the calculations
        if (++Samples >= SUPPLY_FREQUENCY)                                      // 50 Cycles = 1 sec
        {
            newsumCycle=1;                                                      // flag to main loop to process the data
            Samples=0;
        }
    }
    
    sumPowerA[0] = 0;
    sumPowerB[0] = 0;
    sumPowerA[1] = 0;
    sumPowerB[1] = 0;
    sumPowerA[2] = 0;
    sumPowerB[2] = 0;
          
    sumSamples=0;

    
}    

int24_t mul16x8s(int16_t a, int8_t b)
{
    volatile int24_t result=0;

#asm
_asm

    /* define access to bytes of the return value and parameters */
    r3 set mul16x8s@result+2
    r2 set mul16x8s@result+1
    r1 set mul16x8s@result+0
    aL set mul16x8s@a+0
    aH set mul16x8s@a+1   
    bL set mul16x8s@b+0
      
mmac MACRO A,bit, u2,u1
	BTFSC	A,bit
	 ADDWF	u2,F
	RRF	u2,F
	RRF	u1,F
	ENDM        
        
    clrf    r3    
	clrf    r2
    clrc        
	
	clrf	_flag

	btfss   aH,7    // msb set? then negative    
	goto    a_pos  
    comf    aH,f    
	comf	aL,f    // to convert it to a positive nr.
	incf	aL,f    
	skipnz      
     incf    aH,f
	incf	_flag, f
a_pos:
	btfss   bL,7
	goto    cal
	comf    bL,f 
	incf    bL,f 
    incf	_flag, f        
    
    // done with conversion,. now start the calculation
cal:
    movf    bL,W
    mmac	aL,0, r3,r1
	mmac	aL,1, r3,r1
	mmac	aL,2, r3,r1
	mmac	aL,3, r3,r1
	mmac	aL,4, r3,r1
	mmac	aL,5, r3,r1
	mmac	aL,6, r3,r1
	mmac	aL,7, r3,r1
	
	CLRF	r2
	// carry already clear from last RRF of mmac above
	// 8bit multiplicand still in W
	mmac	aH,0, r3,r2
	mmac	aH,1, r3,r2
	mmac	aH,2, r3,r2
	mmac	aH,3, r3,r2
	mmac	aH,4, r3,r2
	mmac	aH,5, r3,r2
	mmac	aH,6, r3,r2
	mmac	aH,7, r3,r2
              
    // done with the calculation. invert result? 
    btfss   _flag, 0
	goto 	no_invert		

    comf    r3,f
	comf    r2,f
	comf	r1,f
	incf    r1,f
	skipnz
	 incf	r2,f
    skipnz
	 incf	r3,f    
no_invert:

_endasm
#endasm

    return result;
}    


/*
double removeRMSOffset(unsigned long sumSquared, long sum, unsigned long numSamples)
{
    double x = ((double)sumSquared / numSamples) - ((double)sum * sum / numSamples / numSamples);
 
    //printf("sumsq: %lu sum: %ld samples %lu \r\n",sumSquared, sum, numSamples );
    
    return (x<0.0 ? 0.0 : sqrt(x));
}*/

#ifndef ESP_DIRECT
double removePowerOffset(long power, unsigned int numSamples)
{
    //return (((double)power / numSamples) - ((double)sumV * sumI / numSamples / numSamples));
    return ((double)power / numSamples);
}
#endif

#ifdef CALCULATE
double deg_rad(double a)
{
    return (0.01745329*a);
}

void calculateTiming(void)
{
  // Pre-calculate the constants for phase/timing correction
  i1phaseshift = (0 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i1Lead); // in degrees
  i2phaseshift = (1 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i2Lead);
  i3phaseshift = (2 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i3Lead);  
    
  y1 = sin(deg_rad(i1phaseshift)) / sin(deg_rad(SAMPLERATE));
  x1 = cos(deg_rad(i1phaseshift)) - y1 * cos(deg_rad(SAMPLERATE));
  y2 = sin(deg_rad(i2phaseshift)) / sin(deg_rad(SAMPLERATE));
  x2 = cos(deg_rad(i2phaseshift)) - y2 * cos(deg_rad(SAMPLERATE));
  y3 = sin(deg_rad(i3phaseshift)) / sin(deg_rad(SAMPLERATE));
  x3 = cos(deg_rad(i3phaseshift)) - y3 * cos(deg_rad(SAMPLERATE));
}
#endif


// Poly used is x^16+x^15+x^2+x
unsigned int CRC16(unsigned int crc, unsigned char *buf, unsigned char len)
{
    unsigned char i, pos;
    
	for (pos = 0; pos < len; pos++)     // char !! length of buffer max 255 bytes!!
	{
		crc ^= (unsigned int)buf[pos];  // XOR byte into least sig. byte of crc

		for (i = 8; i != 0; i--) {  // Loop over each bit
			if ((crc & 0x0001) != 0) {  // If the LSB is set
				crc >>= 1;              // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                        // Else LSB is not set
				crc >>= 1;              // Just shift right
		}
	}
	return crc;
}

void delay(unsigned int d)
{
	unsigned long x;
	x=Timer;                                                                    // read Timer value (increased every ms)
    while (Timer < (x+d)) { }                                                   // blocking delay
}
                                                                                
void RS485_Start(void)                                                          // Send buffer over RS485 line
{
	RS485_TRANSMIT;                                         					// set RS485 transceiver to transmit
	delay(1);
    
    PIE1bits.TXIE = 1;                                                          // Enable interrupts, this will enable transmission
}


void initialize(void) 
{
    LATA = 0x00;                                                                // LATx registers
    LATC = 0x00;
#ifdef ESP_DIRECT
    TRISA = 0x12;                                                               // Pins RA1 and RA4 are inputs, RA0 and RA5 are outputs
#else
    TRISA = 0x30;                                                               // Pins RA4-5 are inputs, RA0-1 are outputs 
#endif    
    TRISC = 0x2A;                                                               // Pins RC1,RC3,RC5 are inputs, RC0,RC2,RC4 are outputs
    ANSELA = 0x10;                                                              // Ansel enabled for all digital outputs, saves a bit of power
    ANSELC = 0x1F;                                                              // 
    WPUA = 0x00;                                                                // Weak Pull up registers
    WPUC = 0x20;                                                                // Pull up on RC5
    OPTION_REGbits.nWPUEN = 0;
    IOCCN = 0x20;                                                               // Interrupt on negative edge of RC5
#ifdef ESP_DIRECT    
    IOCAP = 0x02;                                                               // interrupt on positive edge of RA1
#endif
    
//    RXPPS = 0x01;                                                               // RA1->EUSART:RX;    
    RXPPS = 0x05;                                                               // RA5->EUSART:RX;    
    RA0PPS = 0x14;                                                              // RA0->EUSART:TX; 

    OSCCON = 0x70;                                                              // SCS FOSC; SPLLEN disabled; IRCF 8MHz_HF; 
                                                                                // setup DAC to half of VCC(3.3V) so 1.65V
    DAC1CON0 = 0x90;                                                            // DAC1EN enabled; DAC1NSS VSS; DAC1PSS VDD; DAC1OE1 disabled; DAC1OE2 enabled; 
    DAC1CON1 = 0x80;                                                            // DAC1R 128;

                                                                                // setup the ADC
    ADCON1 = 0xA0;                                                              // ADFM right; ADPREF VDD; ADCS FOSC/32; 
    ADCON0 = 0x0D;                                                              // GO_nDONE stop; ADON enabled; CHS AN3; 
    
    OPA1CON = 0xD2;                                                             // OPA1SP High_GBWP_mode; OPA1EN enabled; OPA1PCH DAC; OPA1UG OPA_Output; 
    
    // uart
    BAUD1CON = 0x08;                                                            // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE disabled; ABDEN disabled; 
    RC1STA = 0x90;                                                              // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled; 
    TX1STA = 0x24;                                                              // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave; 
    
#ifndef ESP_DIRECT
    SP1BRGL = 0x40;                                                             // set baudrate to 9600 bps (832)
    SP1BRGH = 0x03;                                                             // 
#else    
    SP1BRGL = 0x44;                                                             // set baudrate to 115200 bps (68)
    SP1BRGH = 0x00;                                                             // 
#endif    
    //Timer1
    T1GCON = 0;
    TMR1 = ~(T1PLL);                                                            // Timer 1 counter value. 8Mhz * 3200 = 400uS
                                                                                // 400us * 50 samples = 0.02s (50 samples per 50Hz period)
    T1CON = 0b00000001;                                                         // Clock source Fosc/4, prescale 1:1, Timer 1 ON
    
    //Timer4
    PR4 = 0x7C;                                                                 // Timer 4 frequency value -> 1Khz @ 32 Mhz
    T4CON = 0x07;                                                               // Timer 4 ON, prescaler 1:64
    
#ifndef ESP_DIRECT
    PIE1bits.RCIE = 1;                                                          // enable receive interrupt
#endif    
    PIE1bits.ADIE = 1;                                                          // enable ADC conversion done interrupt
    PIE1bits.TMR1IE = 1;                                                        // enable timer 1 interrupt
    PIE2bits.TMR4IE = 1;                                                        // enable timer 4 interrupt
    INTCONbits.IOCIE = 1;                                                       // enable interrupt on I/O pin 
    INTCONbits.PEIE = 1;                                                        // peripheral interrupts enabled
    INTCONbits.GIE = 1;                                                         // global interrupts enabled
}

#if defined (CALCULATE) //|| defined (ESP_DIRECT)
void putch(unsigned char byte)                                                  // user defined printf support on uart
{
	// output one byte on UART
	while(!PIR1bits.TXIF || !TX1STAbits.TRMT) {}                                // wait for Transmit register to be empty
    TX1REG = byte;

}
#endif

void main(void)
{
    char *pBytes;
	char x,n;
	unsigned int cs, crc, len;
    unsigned char LedState=0, LedCnt=0;
    initialize();                                       						// initialize ports, ADC, UART


#ifndef ESP_DIRECT
    LED_RED_OFF;
    LED_GREEN_ON;                                                               // Blink GREEN
    delay(1000);
    LED_RED_ON;                                                                 // Blink RED
    LED_GREEN_OFF;
    delay(1000);
    LED_RED_OFF;                                                                // LED's OFF
#endif

       
    //printf("PIC16F1704 powerup\n");
    
#ifdef CALCULATE    
    /* for (n=0;n<NUMSAMPLES*2;n++) {
        printf("%i,",(int)(sin(n*(360/NUMSAMPLES)*CONST)*63) );
    }*/
    
    calculateTiming();

    printf("\n i1phaseshift %f, i2phaseshift %f, i3phaseshift %f\n",i1phaseshift,i2phaseshift,i3phaseshift);
    printf("\nconst double y1=%f, x1=%f, y2=%f, x2=%f, y3=%f, x3=%f; \n",y1,x1,y2,x2,y3,x3);
    
    while(1) {}
#else   
    
    while (1)
    {

#ifndef ESP_DIRECT             

        while (!LedTimer && LedState) {                                         // start with state 6
            if ((LedState-- &1) ==0) {
                LED_GREEN_ON;
                if (LedSeq[LedCnt++]) LED_RED_ON;
                LedTimer = 200;
            } else {
                LED_GREEN_OFF;
                LED_RED_OFF;
                LedTimer = 200;    
            }
        }

#endif        
 
        if (PLLLock >= 75) {                                                    // Check if we have a lock on the 50 Hz mains input.
            PLLLock = 75;
                
            // set T1PLL to fixed value. 
            // This is to fast for the multiplications (which take more time)
            // If we speed up the large multiplications we can probably increase this..
            //
            T1PLL = 2500;                                                       // 312uS
            NoLock = 1;                                                         // indicates the MAINS connection is not connected,
                                                                                // switch to alternative Irms calculations
            if (TotalSumReady) {

#ifdef ESP_DIRECT
                sprintf(Modbus_TX,"/1R:%ld 2R:%ld 3R:%ld SA:%u!", TotalSumI[0], TotalSumI[1], TotalSumI[2], SAMPLES);
#else
                for (x=0 ;x<3; x++) {
                    IrmsCT[x] = sqrt((double)TotalSumI[x]/SAMPLES) * 0.3 ;
                }
                LedSeq[0] = 1;
#endif                
                dataready = 1;
                TotalSumReady = 0;
            }           
        }

    
        /*  while (!LedTimer) {
            printf("T1PLL %u sumsamples :%u sumPeriodSamples %u \n",T1PLL, sumsamprint, sumPeriodSamples );
            LedTimer=1000;
        }*/
        
        
        // sumPeriodxxx variables are updated by the ISR, and when ready the newsumCycle is set.
        // do not use the sumPeriodxxx variables anywhere else, as the data might be updated at any moment.
        if (newsumCycle) {
                                                                                // Calculate Real Power for all Phases
                                                                                // As the Vrms is based on a sine wave, and uncalibrated, the realPower
                                                                                // value is only used to calculate the real current (including phase shift).
#ifndef ESP_DIRECT
            for (x=0 ;x<3 ;x++) {
                IrmsCT[x] = (xx[x] * removePowerOffset(sumPeriodPowerA[x], sumPeriodSamples) 
                            + yy[x] * removePowerOffset(sumPeriodPowerB[x], sumPeriodSamples)) / CAL;
                
                // Set LED sequence. Orange = Import, Green = Export to grid
                if (IrmsCT[x] < -0.1 ) LedSeq[x]=0; else LedSeq[x]=1;
            }
#else
            sprintf(Modbus_TX,"/1A:%ld 1B:%ld 2A:%ld 2B:%ld 3A:%ld 3B:%ld SA:%u WI:%u!", sumPeriodPowerA[0], sumPeriodPowerB[0], 
                    sumPeriodPowerA[1], sumPeriodPowerB[1], sumPeriodPowerA[2], sumPeriodPowerB[2], sumPeriodSamples, version & 7u);
            
#endif
        //   printf("T1PLL %u sumsamples :%u sumPeriodSamples %u \n",T1PLL, sumsamprint, sumPeriodSamples );
        //   printf("Irms1 %2.1f A Irms2 %2.1f A Irms3 %2.1f A \n",IrmsCT[CT1],IrmsCT[CT2],IrmsCT[CT3]);

        
            sumPeriodPowerA[0] = 0;
            sumPeriodPowerB[0] = 0;
            sumPeriodPowerA[1] = 0;
            sumPeriodPowerB[1] = 0;
            sumPeriodPowerA[2] = 0;
            sumPeriodPowerB[2] = 0;
            sumPeriodSamples = 0;
            
          
            // here we set the phase shifts for the next cycles
            if (version & 4) {
                Phase[0] = PHASE0;                                              // no Phaseshifts
                Phase[1] = PHASE0;
                Phase[2] = PHASE0;
            } else if ( !(version & 2) ) {
                Phase[0] = PHASE0;                                              // 4 Wires, CW rotation
                Phase[1] = PHASE120;
                Phase[2] = PHASE240;
                if (version & 1) {                                              // 4 Wires, CCW rotation
                    Phase[1] = PHASE240;
                    Phase[2] = PHASE120; 
                }
            } else {    
                Phase[0] = PHASE30;                                             // 3 Wires, CW rotation
                Phase[1] = PHASE150;
                Phase[2] = PHASE270;
                if (version & 1) {                                              // 3 Wires, CCW rotation
                    Phase[1] = PHASE270;
                    Phase[2] = PHASE150;
                }    
            } 
            
            newsumCycle=0;                                                      // flag ready for next cycle
            dataready=1;
        }
        
        if (RC1STAbits.OERR) {                                                  // Uart1 Overrun Error?
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;                                                // Restart Uart
        }
        
        if (ModbusTXlen && TX1STAbits.TRMT && PIE1bits.TXIE==0) {           	// transmit register empty and interrupt disabled ?
            RS485_RECEIVE;                                          			// set RS485 transceiver to receive
            ModbusTXlen = 0;                                                    // reset length
        }
        
        // Transmit measurement data to master / host
        //
#ifdef ESP_DIRECT
        if (dataready) {
            // In direct mode we send the data over a serial line
            len = strlen(Modbus_TX);
            cs = CRC16(0, Modbus_TX, len);                                      // calculate CRC16 from data
            sprintf(Modbus_TX+len,"%04x\r\n", cs);                              // add crc to the end of the string

            ModbusTXlen = len+6;                                                // length of data to send
            ModbusTXidx = 0;                                                    // points to first character to send.

            RS485_Start();                                                      // send buffer to RS485 port, using interrupts
            dataready=0;
        }
#else   
        // We use modbus to send the data
        //
        if ( (ModbusIdx > 6) && (Timer>(ModbusTimer+3)) )                       // last reception more then 3ms ago? 
        {
            crc = CRC16(0xffff, Modbus_RX, ModbusIdx);                          // calculate checksum over all data (including crc16)
            
            // Write to Holding registers and set Wire and rotation settings (modbus register address 0x800)
            if (Modbus_RX[0]==0x0a && Modbus_RX[1]==0x06 && Modbus_RX[2]==0x08 && Modbus_RX[3]==0x00 && Modbus_RX[4]==0x00 && !crc)
            {
                version = 0x10 + (Modbus_RX[5] & 7u);                           // Set 3 or 4 Wire, and phase Rotation.
                for (n=0; n<8; n++) Modbus_TX[n] = Modbus_RX[n];                // echo back the Write request.
                ModbusTXlen = n;                                                // length of modbus packet
                ModbusTXidx = 0;                                                // points to first character to send.
                Timer = 0;
                RS485_Start();                                                  // send buffer to RS485 port, using interrupts
            }    
            
            // Request for a measurement from the SmartEVSE
            else if (Modbus_RX[0]==0x0a && Modbus_RX[1]==0x04 && Modbus_RX[5]<=0x14 && !crc && dataready )   // check CRC
            {                                                                   // we have received a valid request for sensorbox data
                                                                                // Setup Modbus data
                Modbus_TX[0]= 0x0a;                                             // Fixed Address 0x0A
                Modbus_TX[1]= Modbus_RX[1];                                     // function byte
                Modbus_TX[2]= Modbus_RX[5] * 2u;                                // takes the bytes from the request. 28h bytes will follow
                Modbus_TX[3]= 0x00;                                             // 
                Modbus_TX[4]= version;                                          // Sensorbox version 1.5+ = 0x10 plus bitmask for 3/4Wire and rotation

                //Modbus_TX[3]= 'I';                                            // Used as trigger for SmartEVSE TestIO
                //Modbus_TX[4]= 'O';

                Modbus_TX[5]= 0x00;                                             // DSMR version
                Modbus_TX[6]= 0x03;                                             // 0x03= 3CT's
                
                n=7;
                do {
                    Modbus_TX[n++] = 0;
                } while (n<31);

                for (x=0; x<3 ;x++) {	
                    pBytes = (char*)&IrmsCT[x];                                 // get raw 4 byte Double 
                    Modbus_TX[n++] = pBytes[3];
                    Modbus_TX[n++] = pBytes[2];
                    Modbus_TX[n++] = pBytes[1];
                    Modbus_TX[n++] = pBytes[0];
                }
                cs = CRC16(0xffff, Modbus_TX, n);                               // calculate CRC16 from data			
                Modbus_TX[n++] = ((unsigned char)(cs));
                Modbus_TX[n++] = ((unsigned char)(cs>>8));	

                ModbusTXlen = n;                                                // length of modbus packet
                ModbusTXidx = 0;                                                // points to first character to send.
                Timer = 0;
                
                RS485_Start();                                                  // send buffer to RS485 port, using interrupts
                dataready=0;
           
                LedCnt=0;                                                       // reset to first part of sequence
                if (NoLock) LedState = 2;
                else LedState=6;                                                     // 6 States, ON/OFF (3 CT's)
                
            }
            
            ModbusIdx=0;
        }       
#endif        
    } // while(1)
#endif  // CALCULATE
}
/*
 End of File
*/