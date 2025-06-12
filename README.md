## Universal motor PID speed regulator for bench drill.

ATMEGA 328 based DC motor electronic regulator.

It uses pms67/PID library (https://github.com/pms67/PID)



# Hardware design
  - A potentiometer is used at input to select desired speed.
  - Actual speed is measured and sent to controller
  - ATmega microcontroller compute resulting PWM signal
  - A MOSFET modulates MOTOR DC voltage.
  - Switching frequency is 32KHz.

# Software design
  - Potentiometer position is read via ADC3 (setpoint)
  - Speed sensor connected to INT0 and measured by counting timer2 ticks (measurement)
  - Sampling rate is set to 0.1 Sec and call to PIDController_Update()
  - Output is used to control motor voltage (PWM)
  - Calculations are done in floating point and normalized to one.
