/***********************************************************************
 *
 * Motor.c
 *
 * Very simplified Universal motor simulation model
 *
 * Parameters:
 *  - Load torque value
 *  - Max rmp value (No load)
 *  - Electrical time constant (L/R)
 *  - Mecanical  time constant (total rotor Rotor Inertia)
 *
 *
 ***********************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>



// Motor
// R=7 Ohms, L=125mH

//#define SIM_SAMPLE_TIME_S   0.025f  // Sec
#define SIM_SAMPLE_TIME_S   0.1f  // Sec
#define ELEC_TIME_CONSTANT  0.018f    // Sec
#define MECA_TIME_CONSTANT  0.3f    // Sec
#define EQU_SERIAL_RESISTOR 10      // Ohms
#define MAX_RPM             3000    // rpm

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 20.0f

//extern double input_signal[];
//extern int size_of_signal(void);

#define PI 3.1415927

// Max Torque = 50 N*m
double TorqueValue = 0.0;  // N*m

double Elec_History;
double Meca_History;

double Motor_rpm;

/* Electrical time constant filtering (LPF)
 * ALPHA = dt / ( dt + RC )   where dt is 1/samplerate.
 *
 *******************************/
double Elec_LPF(double x_input) {
    //double Alpha = 0.24; // Facteur d'amortissement (0 < A < 1)
    double y_out;

    double Alpha = SIM_SAMPLE_TIME_S / (SIM_SAMPLE_TIME_S + ELEC_TIME_CONSTANT);


    y_out = Elec_History + Alpha * (x_input - Elec_History);
    Elec_History = y_out;

    // Retourne la sortie filtrée
    return y_out;
}


/* Mecanical time constant filtering (LPF)
 * ALPHA = dt / ( dt + RC )   where dt is 1/samplerate.
 *
 *******************************/
double Meca_LPF(double x_input) {
    //double Alpha = 0.24; // Facteur d'amortissement (0 < A < 1)
    double y_out;

    double Alpha = SIM_SAMPLE_TIME_S / (SIM_SAMPLE_TIME_S + MECA_TIME_CONSTANT);


    y_out = Meca_History + Alpha * (x_input - Meca_History);
    Meca_History = y_out;

    // Retourne la sortie filtrée
    return y_out;
}


/*
 *
 * V Inpout voltage
 *   0 to 1 where 1 is nominal voltage
 *******************************************/
double computeRPM( double V ){
    double torque = TorqueValue + 1.0;
    double rpm;

    rpm = (MAX_RPM * (V)) - (100*torque);

    if ( rpm <= 0.0 ){
        rpm = 0.0;
    }

    return rpm;
}


double computeEFM( double rpm ){

    return 1*rpm/MAX_RPM;
}





double MotorSystemUpdate( double signal ){
    static double efm;
    double v_int;
    double rpm;


    v_int = signal - efm;
    rpm = computeRPM(v_int);
    rpm = Elec_LPF(rpm);
    rpm = Meca_LPF(rpm);
    efm = computeEFM(rpm);

    if ( TorqueValue > 100 )
        Motor_rpm = 0;
    else
        Motor_rpm = rpm;

    return rpm/600;
}



#include "PID.h"



float TestSystem_Update(float inp) {

    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (SIM_SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SIM_SAMPLE_TIME_S);

    return output;
}


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

    print_uint_as_fixed_point( 6, 3, t );
    print_uint_as_fixed_point( 6, 2, (uint32_t)(setpoint*10001) );
    print_uint_as_fixed_point( 6, 2, (uint32_t)(measurement*10001) );
    print_uint_as_fixed_point( 8, 2, (uint32_t)(out*1000001) );

    putchar( '\n');
}


int main()
{
    /* Initialise PID controller */
    PIDController pid = {
        //.Kp=2.0f, .Ki=0.5f, .Kd=0.25f,
        .Kp=0.2f,.Ki=0.5f,.Kd=0.05f,
          .tau=0.005f,
          .limMin=-0.0f, .limMax=+1.0f,
          .limMinInt=-0.0f, .limMaxInt=+1.0f,
          .T=SIM_SAMPLE_TIME_S
      };

    PIDController_Init(&pid);

    /* Simulate response using test system */
    float setpoint = 0.0f;

    //printf("Time (s)\tSystem Output\tControllerOutput\r\n");
    for (double t = 0.0f; t <= SIMULATION_TIME_MAX; t += SIM_SAMPLE_TIME_S) {

        double step = 1.0f / ((SIMULATION_TIME_MAX/4)/SIM_SAMPLE_TIME_S);
        if ( t < SIMULATION_TIME_MAX/4 ){
            setpoint += step;
        }

        if ( t > SIMULATION_TIME_MAX/2 ){
            TorqueValue = 16.0;  // N*m
        }
        if ( t > 3*SIMULATION_TIME_MAX/4 ){
            TorqueValue = 2.0;  // N*m
        }

        /* Get measurement from system */
        //float measurement = TestSystem_Update(pid.out);
        float measurement = MotorSystemUpdate(pid.out);

        /* Compute new control signal */
        PIDController_Update(&pid, setpoint, measurement);

        //printf("%f\t%f\t%f\r\n", t, measurement, pid.out);
        //printf("%.3f %.6f %.6f %.6f\n", t, setpoint, measurement, pid.out);
        DoAvrPrint( t*1000, setpoint, measurement, pid.out );

    }

    return 0;
}



