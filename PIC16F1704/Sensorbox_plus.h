/*
;	 Project:       Smart EVSE Sensorbox 1.5plus
;    Date:          10 July 2019
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

#ifndef __EVSE_SENSORBOX
#define __EVSE_SENSORBOX

#include <xc.h>
#include <string.h>
#include <math.h>

//#define CALCULATE                                                             // uncomment to calculate constants

// LBR enable for sensorbox2
#define ESP_DIRECT                                                              // if defined, will -not- send data directly to SmartESVE controllers,
                                                                                // but only output CT measurement data over the Uart
#define CODEPROTECT                                                             // code protection enabled

#define _XTAL_FREQ  32000000

#define SUPPLY_VOLTS 3.3  
#define SUPPLY_FREQUENCY 50
                                                                                // A three-phase system may be arranged in delta (?) or star (Y)
#define WIRES WIRESMONO                                                            // 4-Wire star (Y)(L1,L2,L3 and Neutral) or delta (?) 3-Wire (L1,L2,L3, no Neutral)
#define ROTATION CW
#define CAL 151                                                                 // Current Sensor Calibration value
#define SAMPLES 3000 //2048

#define CT1 0                                                                   // the order in which the Phases are connected to the Sensorbox 
#define CT2 1                                                                   // When the mains voltage input is connected to Phase 2, the order is 2,0,1
#define CT3 2                                                                   // When the mains voltage input is connected to Phase 3, the order is 1,2,0

#define ADC_RATE 20                                                             // Time between successive ADC conversions in microseconds
#define NUMSAMPLES 60                                                           // Number of times to sample CT1, CT2, and CT3 each 50Hz cycle (should be a multiple of 12)
#define SAMPLERATE (360.0 / NUMSAMPLES)                                         // Sample Rate in degrees

#define PHASE0 0                                                                // No delay for the Phase 1 voltage
#define PHASE120 120/(360/NUMSAMPLES)                                           // Delay for the Phase120 voltage 360  /3 = 120 deg
#define PHASE240 240/(360/NUMSAMPLES)                                           // Delay for the Phase240 voltage 360*2/3 = 240 deg

#define PHASE30 30/(360/NUMSAMPLES)                                             // Delay for the Phase30 voltage 360 /12 =  30 deg
#define PHASE150 150/(360/NUMSAMPLES)
#define PHASE270 270/(360/NUMSAMPLES)


#define CW 0                                                                    // Rotation Right (CW)
#define CCW 1                                                                   // Rotation Left (CCW)
#define WIRES4 0                                                                //
#define WIRES3 2                                                                //
#define WIRESMONO 4                                                             // LBR for mono measurements

#define CONST 3.141592/180.0

#ifdef ESP_DIRECT
    #define RS485_RECEIVE 
    #define RS485_TRANSMIT

    #define LED_GREEN_ON
    #define LED_GREEN_OFF

    #define LED_RED_ON
    #define LED_RED_OFF
#else
    #define RS485_RECEIVE {LATAbits.LATA1 = 0;}
    #define RS485_TRANSMIT {LATAbits.LATA1 = 1;}

    #define LED_GREEN_ON { LATCbits.LATC0 = 0;}
    #define LED_GREEN_OFF { LATCbits.LATC0 = 1;}

    #define LED_RED_ON { LATCbits.LATC4 = 0;}
    #define LED_RED_OFF { LATCbits.LATC4 = 1;}
#endif

void newPeriod(void);
int24_t mul16x8s(int16_t,int8_t);

#endif	


