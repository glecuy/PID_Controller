/*****************************************************************************
*
*
*  File Name    : main.c
*  Version      : 1.0
*
*****************************************************************************/
#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include <avr/sleep.h>
#include <avr/wdt.h>

#include "uart_printf.h"
#include "timer0.h"
#include "ssd1306.h"
#include "analogIn.h"

#include "PID.h"

// On boad LED (Pro Mini board)
#define LED1        PINB5

// On main board LED
#define LED2        PINC2


#define TRIAC_GATE  PINB2


#define SoftReset() wdt_enable(WDTO_30MS); while(1) {}


uint8_t dbg_var;

/* Global variables */
static char TempString[16];  // Dangerous !

static volatile uint16_t zc_count;
static volatile uint16_t zc_width;


#define LED1_OFF()      (PORTB &= ~(1 << PINB5))
#define LED1_ON()       (PORTB |= (1 << PINB5))
#define LED1_TOGGLE()   { if ( PORTB & (1 << PINB5) ) LED1_OFF(); else LED1_ON(); }


#define LED2_OFF()      (PORTC &= ~(1 << PINC2))
#define LED2_ON()       (PORTC |= (1 << PINC2))
#define LED2_TOGGLE()   { if ( PORTC & (1 << PINC2) ) LED2_OFF(); else LED2_ON(); }


/* Initialise PID controller
 * Most values come from PID_Test.c */
static PIDController PID_Controller = {
                .Kp=2.0f, .Ki=0.5f, .Kd=0.25f,
                .tau=0.02f,
                .limMin=-1.0f, .limMax=+1.0f,
                .limMinInt=-5.0f, .limMaxInt=+5.0f,
                .T=(TIMER_TICK_MS/1000.0f)
};


/*
 * Zero crossing interrupt
 * Only rising edge is considered
 ************************************/
ISR(PCINT0_vect)
{
    // read PCINT0 (PB0): Rising edge
    if( PINB & (1 << PB0) ) {
        zc_count++;

        // Reset timer 1 counter
        TCNT1 = 0;

        // Make sure Gate is off
        PORTB &= ~(1<<TRIAC_GATE);
    }
    else{
        // Check width with timer 1 counter
        zc_width = TCNT1;
    }

}

ISR(TIMER1_COMPA_vect)
{
    // Turn triac gate signal ON
    PORTB |= (1<<TRIAC_GATE);
}


ISR(TIMER1_COMPB_vect)
{
    // Turn triac gate signal OFF
    PORTB &= ~(1<<TRIAC_GATE);
}



void io_ports_init( void ){
    DDRB = 0;
    DDRB |= (1 << LED1);    // Set LED as output

    DDRC = (1 << LED2);     // Set LED as output


    DDRB |= (1 << TRIAC_GATE);  // Set Triac output
    // Triac gate signal OFF
    PORTB &= ~(1<<TRIAC_GATE);

}


void zero_crossing_init(void){
    // Signal is mapped to PB0 PCINT0

    /* Define Pin change interrupt for PB0 (PCINT0) */
    PCICR |= (1 << PCIE0);  // Port B
    /* PCINT[7:0] */
    PCMSK0 |= (1 << PCINT0);  // Zero Crossing signal

}


/*
 * OCR1A/B should be in 0 to 1250 range
 * 0    : No delay after 0 crossing position
 * 1200 : Half wave period delay (10 mSec)
 *
 * POS  VOUT
 * 950   30V
 * 904   40V
 * 824   60V
 * 754   80V
 * 685  100V
 * 605  120V
 * 538  140V
 * 459  160V
 * 382  180V
 * 276  200V
 * 150  220V
 *  75  230V
 ***********************************************************/
void setTriacPulsePos(uint16_t pos){
    // VMIN => 1230
    // VMAX => 400
    if ( pos < 980 ){
        OCR1A = 300+pos;
        OCR1B = 300+pos+10;
        // Enable Compare A & B Match Interrupts
        TIMSK1  |= (1 << OCIE1A );
        TIMSK1  |= (1 << OCIE1B );
    }
    else{
        // Disable interrupts
        TIMSK1 = 0;
        // Triac gate signal OFF
        PORTB &= ~(1<<TRIAC_GATE);
    }
}


void ClockSetup(void){
/*
The CAL[6:0] bits are used to tune the frequency within the selected range.
  A setting of 0x00 gives the
lowest frequency in that range and a setting of 0x7F gives the highest frequency in the range.
*/

    // Value to fix fake chineese ATMEGA328 :
    //OSCCAL = 0x80 | 0x4F;
}




void timer1Setup(void){
    //Set up Timer1
    TCNT1 = 0;      //set timer to zero, may not be necessary
    TCCR1A = 0;     //normal counting up - output compare pins not used, initially zero so not necessary

    // Start timer at Fcpu/64
    // F = 8000000/64 = 125000.0
    // t = 64/8000000 = 8e-06 (8uSec)
    TCCR1B |= ((0 << CS12) | (1 << CS11) | (1 << CS10));

    //OCR1A = 500;
    //OCR1B = 510;
    //
    //// Enable Compare A & B Match Interrupts
    //TIMSK1  |= (1 << OCIE1A );
    //TIMSK1  |= (1 << OCIE1B );
}

uint8_t db_on;



int main(void)
{
    unsigned long TxTick=0;
    unsigned long NewTick=0;

    // Turn all pullup restistors ON
    PORTB = 0xFF;
    PORTC = 0xFF;
    PORTD = 0xFF;

    io_ports_init();
    LED1_OFF();

    ClockSetup();
    timer0_init();

    AnalogInInit();

    ssd1306_init();

    zero_crossing_init();

    timer1Setup();

    PIDController_Init(&PID_Controller);

    //  Enable global interrupts
    sei();

    // Initialize UART print
    uart_printf_init();

    printf("Hello AVR\n\n");

    LED1_ON();

    while( 1 ){
        /* Test Tx loop time (Tick = 50mSec) */
        if ( timer0_GetTicks() != NewTick )
        {
            NewTick = timer0_GetTicks();

            setTriacPulsePos( valVCtrl() );
#if 0
            if ( db_on ){
                db_on = 0;
                PORTB &= ~(1 << PINB0);
            }
            else{
                db_on=1;
                PORTB |= (1 << PINB0);
            }
#endif
        }
        if ( timer0_GetTicks() > TxTick + (1000/TIMER_TICK_MS) )
        {
            TxTick = timer0_GetTicks();
            LED1_TOGGLE();

            printf("VCTRL = %u\n", valVCtrl() );
            printf("zc_count = %u - %u\n", zc_count, zc_width );
            printf("TIMSK1 = %02X\n", TIMSK1 );

        }
    }


  return 0;
}


/* EOF  */

