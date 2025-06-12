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

int timer1_init(void)
{
    TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode

    // Set CTC compare value to 0.1Hz AVR clock @2MHz, with a prescaler of 64 (16 bit value)
    // Internal oscillator + Clock div = 4
    //OCR1A   = 2000000/64/10 - 1;
    // Internal oscillator + Clock div = 2
    //OCR1A   = 4000000/64/10 - 1;

    // 8MHz Internal oscillator + Clock div = 1
    OCR1A   = 8000000/64/10 - 1;

    // 16MHz External oscillator + Clock div = 1
    //OCR1A   = 16000000/64/10 - 1;

    TCCR1B |= ((0 << CS12) | (1 << CS11) | (1 << CS10)); // Start timer at Fcpu/64

    TimerTicks=0;
    // Enable Compare A Match Interrupt
    TIMSK1  |= (1 << OCIE1A );

    return 0;
}

ISR(TIMER1_COMPA_vect)
{
    TimerTicks++;
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

unsigned long timer1_GetTicks(void)
{
    unsigned long Ret;

    TIMSK1  &= ~(1 << OCIE1A );
    Ret = TimerTicks;
    TIMSK1  |= (1 << OCIE1A );

    return Ret;
}
