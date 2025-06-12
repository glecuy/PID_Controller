## Universal motor PID speed regulator for bench drill.

ATMEGA 328 based DC motor electronic regulator.

It uses pms67/PID library (https://github.com/pms67/PID)



# Hardware architecture
  - A potentiometer is used at input to select desired speed.
  - Actual speed is measured and sent to controller
  - ATmega microcontroller compute resulting PWM signal
  - A MOSFET modulates MOTOR DC voltage.
  - Switching frequency is 32KHz.

# Software architecture
  - Potentiometer position is read via ADC3
  - Speed (RPM) connected to INT0 and measured by counting timer2 ticks
  - Sampling rate is set to 0.1 Sec.




