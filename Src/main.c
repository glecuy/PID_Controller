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
#include "debug.h"

// On board LED (Pro Mini board)
#define LED1        PINB5

// On main board LED
#define LED2        PINC2


// PWM  -  Output Compare Match B Output
#define PWM_OUTPUT  PINB2



#define SoftReset() wdt_enable(WDTO_30MS); while(1) {}


uint8_t dbg_var;

/* Global variables */
static char TempString[16];  // Dangerous !

static volatile uint16_t zc_count;
static volatile uint16_t zc_width;

static volatile uint8_t  cnt2H;

static volatile uint32_t T2_ticks_count = 0UL;

static volatile uint16_t _meas_count;

#define LED1_OFF()      (PORTB &= ~(1 << PINB5))
#define LED1_ON()       (PORTB |= (1 << PINB5))
#define LED1_TOGGLE()   { if ( PORTB & (1 << PINB5) ) LED1_OFF(); else LED1_ON(); }


#define LED2_OFF()      (PORTC &= ~(1 << PINC2))
#define LED2_ON()       (PORTC |= (1 << PINC2))
#define LED2_TOGGLE()   { if ( PORTC & (1 << PINC2) ) LED2_OFF(); else LED2_ON(); }



#define Tacho_Interrupt_disable() EIMSK &= ~(1 << INT0)
#define Tacho_Interrupt_enable()  EIMSK |= (1 << INT0)

/* Initialise PID controller
 * Most values come from PID_Test.c */
//static PIDController PID_Controller = {
//                .Kp=2.0f, .Ki=0.5f, .Kd=0.25f,
//                .tau=0.02f,
//                .limMin=-1.0f, .limMax=+1.0f,
//                .limMinInt=-5.0f, .limMaxInt=+5.0f,
//                .T=(TIMER_TICK_MS/1000.0f)
//};

static PIDController PID_Controller = {
        .Kp=0.4f,.Ki=1.0f,.Kd=0.02f,
        .tau=0.005f,
        .limMin=0.0f, .limMax=+1.0f,
        .limMinInt=0.0f, .limMaxInt=+5.0f,
        .T=0.100f
};


/* Tacho metter is connected to INT0 pin
 * (Was INT1 before HW failure !)
 * 4 interrupts expected for one revolution */
ISR(INT0_vect)
{
    uint16_t count;
    // Read HW timer
    count  = TCNT2;
    // Read SW counter
    count |= (cnt2H<<8);

    // Reset timer
    TCNT2 = 0;
    cnt2H = 0;

    _meas_count++;
    T2_ticks_count += (uint32_t)count;

    LED2_TOGGLE();
}

/* Timer 2overflow interrupt
 * Allows ticks count beyond 255 */
ISR(TIMER2_OVF_vect)
{
    // Count overflows (15..8 bits)
    if ( cnt2H < 255 )
        cnt2H++;
}


void io_ports_init( void ){
    DDRB = 0;
    DDRB |= (1 << LED1);    // Set LED as output

    DDRC = 0;
    DDRC |= (1 << LED2);     // Set LED as output
    // Keep PINC1    as Input (Connected to reset)

    DDRC |= (1 << DBG_PIN);     // Debug


    DDRB |= (1 << PWM_OUTPUT);  // Set PWM signal output for MOSFET gate drive

    // Gate signal OFF
    PORTB &= ~(1<<PWM_OUTPUT);

}



/* INT0 */
void Tacho_Interrupt_init(void){

    EICRA &= ~((1<<ISC01) | (1<<ISC00));
    EICRA |= (1<<ISC01) | (1<<ISC00);   // The rising edge of INT0 generates an interrupt request.

    //EICRA |= (0<<ISC01) | (1<<ISC00);   // Any logical change on INT0 generates an interrupt request.

    EIMSK |= (1 << INT0);  // PIN D2 (INT0)

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




/* Timer 1 is used to generate PWM output signal
 *
 *
 ***************************************************************/
void timer1Setup(void){

    // Set up Timer1
    // Fast PWM - non-inverting - mode PWM - 8 bit
    TCCR1A = (1 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (1 << WGM10);

    // Fast PWM - 8 bit - mode 7
    TCCR1B = (0 << WGM13) | (1 << WGM12);

    // CS[2:0] Prescaler setting
    // 001 clk/1
    // 010 clk/8
    // 011 clk/64
    // 100 clk/256
    // 101 clk/1024
    TCCR1B |= ((0 << CS12) | (0 << CS11) | (1 << CS10));
}

void set_PWM_Value( uint16_t pwm ){
    OCR1B = pwm;

}

/* Timer 2 is used for Tachometer
 * Frequency is measured by determining period length
 * of the tachometer signal
 *
 ***************************************************************/
void timer2Setup(void){
    // Set up Timer2
    TCNT2 = 0;      //set timer to zero, may not be necessary
    TCCR2A = 0;     //normal counting up - output compare pins not used, initially zero so not necessary

    // Start timer at Fcpu/1024
    // F = 8000000/1024 = 7812.5
    // t = 1024/8000000 = 0.000128 (Sec)

    // Tin =  CNT * t
    // Fin = 1 / (CNT * t)
    // Fin = 1 / ((CNT * 1024) / 8000000)
    // Fin = 8000000 / (CNT * 1024)
    // 256 -> RPM Min = 919 , 1024 > RPM Min = 229
    //TCCR2B |= ((1 << CS22) | (1 << CS21) | (0 << CS20));

    // 64
    TCCR2B |= ((1 << CS22) | (0 << CS21) | (0 << CS20));

    // 32
    //TCCR2B |= ((0 << CS22) | (1 << CS21) | (1 << CS20));

    // 8
    //TCCR2B |= ((0 << CS22) | (1 << CS21) | (0 << CS20));

    // Enable overflow interrupt
    TIMSK2 = 1<<TOIE2;

}


uint8_t db_on;


void print_uint_as_fixed_point( int digits, int pos, uint32_t n ){
    char text[32];
    int i;
    uint32_t num = n;
    int d;
    char *ptr;

    for ( i=digits-1 ; i>=0 ; i-- ){
        d = num %10;
        num /= 10;
        text[i] = '0' + d;
        if ( i==pos ){
            i--;
            text[i] = '.';
        }
    }
    text[digits]=' ';
    text[digits+1]='\0';
    ptr = text;
    while ( *ptr ){
        putchar( *ptr );
        ptr++;
    }

}

/*
 * Replace printf %f for AVR microcontroleur
 *  output format used for python plot tool
 */

void DoAvrPrint( uint32_t t, float setpoint, float measurement, float out ){

    print_uint_as_fixed_point( 6, 3, t);
    print_uint_as_fixed_point( 6, 2, (uint32_t)(setpoint*10001) );
    print_uint_as_fixed_point( 6, 2, (uint32_t)(measurement*10001) );
    print_uint_as_fixed_point( 8, 2, (uint32_t)(out*1000001) );

    putchar( '\n');
}


uint32_t GetRpmOrderInput(void){
    uint32_t rpm;
    uint32_t Vin=0;

    Vin = (uint32_t)valVCtrl();

    rpm = (( 1000 * Vin ) / 255 );

    // Return rpm in parts per thousand.
    return rpm;
}


#define MAX_RPM     6000.
#define MIN_RPM     800
#define RPM_RAMP_UP 200
#define SPIN_UP     10

int main(void)
{
    unsigned long Tick100ms=0;

    uint32_t SetRPM=0;
    uint32_t RampUpRPM=0;
    int SpinUpCnt=0;


    uint16_t pwmValue;

    uint32_t CNT;
    uint16_t MeasCount;

    uint16_t loopCtrl=0;

    float RPM;

    // Turn all pullup restistors ON
    PORTB = 0xFF;
    PORTC = 0xFF;
    PORTD = 0xFF;

    io_ports_init();
    LED1_ON();

    ClockSetup();
    timer0_init();

    AnalogInInit();

    //ssd1306_init();

    Tacho_Interrupt_init();

    timer1Setup();

    timer2Setup();

    PIDController_Init(&PID_Controller);

    //  Enable global interrupts
    sei();

    // Initialize UART print
    uart_printf_init();

    printf("Hello AVR\n\n");

    LED1_OFF();

    RampUpRPM = MIN_RPM;
    while( 1 ){
        if ( timer0_Get100msTicks() != Tick100ms )
        {
            Tick100ms = timer0_Get100msTicks();
            //LED1_TOGGLE();
            DBGPIN_TOGGLE();


            SetRPM = GetRpmOrderInput();
            SetRPM = SetRPM * (MAX_RPM-MIN_RPM)/1000 + MIN_RPM;

            if ( SetRPM > RampUpRPM ){
                SetRPM = RampUpRPM;
                RampUpRPM += RPM_RAMP_UP;
            }
            //printf("SetRPM=%lu\n", SetRPM );

            //uint32_t CNT = (uint32_t)_tacho_count;
            // Critical section
            Tacho_Interrupt_disable();
                CNT = T2_ticks_count;
                T2_ticks_count=0;
                MeasCount = _meas_count;
                _meas_count=0;
            Tacho_Interrupt_enable();

            /* RPM = (60 * f) / 4 ;  4 pulses per revolution */
            /* div32 prescaler) */
            if ( MeasCount > 0 ){
                RPM = (float)((8000000.0*60.0 / (CNT * 64))/4.0) * (float)MeasCount;
            }
            else{
                // Let some cycles to machine to start
                if ( SpinUpCnt < SPIN_UP ){
                    SpinUpCnt++;
                }else{
                    SpinUpCnt = 0;
                }
                RPM = 0;
            }

            if ( ( RPM == 0 ) && ( SpinUpCnt >= SPIN_UP ) ){
                RampUpRPM = MIN_RPM;
                PIDController_Init(&PID_Controller);
            }


            /* Compute new control signal */
            PIDController_Update(&PID_Controller, ((float)SetRPM)/MAX_RPM, RPM/MAX_RPM  );
            DoAvrPrint( (Tick100ms & 0xFF)*100, ((float)SetRPM)/MAX_RPM, RPM/MAX_RPM, PID_Controller.out );

            pwmValue = (uint16_t)(uint32_t)(PID_Controller.out*100.0);
            if ( pwmValue > 64){
                pwmValue = 64;
            }
            if ( pwmValue < 2){
                pwmValue = 2;
            }

            set_PWM_Value( pwmValue );

            if ( ++loopCtrl % 10 == 0 ){
                //uint32_t uSetRPM = SetRPM;

                //printf("pid.out=%lu\n", (uint32_t)(PID_Controller.out*100.0) );
                //printf("SetRPM=%lu RPM=%lu OUT=%u\n", uSetRPM, (uint32_t)RPM, pwmValue);
                //printf("SetRPM=%u VCTRL=%u\n", SetRPM, valVCtrl() );
                //printf("CNT = %lu  RPM=%lu (%u)\n", CNT, (uint32_t)(RPM+0.5), MeasCount );
                //printf("CNT = %lu  cnt=%lu (%u)\n", CNT, CNT/MeasCount, MeasCount );
            }

        }
    }


  return 0;
}


/* EOF  */

