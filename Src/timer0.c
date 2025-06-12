/*
    timer1.c

    Copyright 2014 GLecuyer <glecuyer207@gmail.com>


 */


#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>



volatile unsigned long TimerTicks;

volatile uint8_t led_on;

int timer0_init(void)
{
    TCCR0A = (1 << WGM01); // Configure timer 0 for CTC mode
    TCCR0B = 0; //  WGM02 = 0 Configure timer 0 for CTC mode

    // F = FOSC / (2 * 1024 * (1+OCR0A))
    // F = 8000000/(2*1024*(1+200))
    // F = 8000000/(2*1024*(1+194)) = 20.0320512
    OCR0A   = 194;

    TCCR0B |= ((1 << CS02) | (0 << CS01) | (1 << CS00)); // 101 clkIO/1024 (From prescaler)

    TimerTicks=0;

    // Enable Compare A Match Interrupt
    TIMSK0  |= (1 << OCIE0A );

    return 0;
}

ISR(TIMER0_COMPA_vect)
{
    TimerTicks++;
#if 0
    if ( led_on ){
        led_on = 0;
        PORTC &= ~(1 << PINC2);
    }
    else{
        led_on=1;
        PORTC |= (1 << PINC2);
    }
#endif
}

void UsSleep( unsigned long Delay )
{
    unsigned char TCNT_Prev;
    unsigned long Counts;

    /* 1 Lsb = 64/(F_CPU) Sec. */
    Counts = (F_CPU / (1000U * 64U));
    Counts *= Delay;
    Counts /= 1000;

    TCNT_Prev=0;
    while( Counts )
    {
        if ( TCNT_Prev != TCNT1L )
        {
            Counts--;
            TCNT_Prev = TCNT1L;
        }
    }
}


unsigned long timer0_GetTicks(void)
{
    unsigned long Ret;

    TIMSK0  &= ~(1 << OCIE0A );
    Ret = TimerTicks;
    TIMSK0  |= (1 << OCIE0A );

    return Ret;
}
