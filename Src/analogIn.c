/*
    analogIn.c

    Temperature capture
    combustion detection


 */

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile unsigned short VCtrlVal;


void AnalogInInit( void )
{
    ADMUX  = 0x3;  // 0011 use ADC3

    //ADMUX |= (1 << REFS1) | (1 << REFS0);  // use 1.1V as the reference
    ADMUX |= (0 << REFS1) | (1 << REFS0);    // AVCC with external capacitor at AREF pin
    //ADMUX |= (1 << ADLAR);                 // Right adjust for 8 bit resolution
    ADMUX &= ~(1 << ADLAR);                  // clear for 10 bit resolution

    ADCSRA  = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 prescale for 16Mhz
    //ADCSRA |= (1 << ADATE);   // Set ADC Auto Trigger Enable

    ADCSRB = 0;               // 0 for free running mode

    ADCSRA |= (1 << ADEN);    // Enable the ADC
    ADCSRA |= (1 << ADIE);    // Enable Interrupts

    ADCSRA |= (1 << ADSC);    // Start the ADC conversion
}



ISR(ADC_vect)
{

    VCtrlVal  =  (unsigned short)ADCL;
    VCtrlVal |= (ADCH << 8);     // ADCH is read so ADC can be updated again

    ADCSRA |= (1 << ADSC);    // Re-start the ADC conversion
}

/*
 * Returns VCtrl
 *
 *****************************************/
unsigned short valVCtrl( void )
{
    return (255 - VCtrlVal / 4) & 0xFF;
}


#if 0 //Dual input
ISR(ADC_vect)
{

    if ( MuxValue == 0x06 ){
        FlameVal  =  (unsigned short)ADCL;
        FlameVal |= (ADCH << 8);     // ADCH is read so ADC can be updated again
        MuxValue = 0x07;
        //MuxValue = 0x08;   // Internal temp
    }
    else {
        TemperatureVal  =  (unsigned short)ADCL;
        TemperatureVal |= (ADCH << 8);     // ADCH is read so ADC can be updated again
        MuxValue = 0x06;
    }

    ADMUX &= ~0x0F;           // Clear MUX[3..0]
    ADMUX |=  MuxValue;       // Change value
    ADCSRA |= (1 << ADSC);    // Re-start the ADC conversion
}
#endif
