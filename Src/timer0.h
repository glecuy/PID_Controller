
#ifndef _TIMER_0_H_
#define _TIMER_0_H_

#define TIMER_TICK_MS  25

#define TIMER_TICK_100MS   3
#define TIMER_TICK_1000MS  39



int timer0_init(void);
unsigned long timer0_GetTicks(void);
unsigned long timer0_Get100msTicks(void);

void UsSleep( unsigned long Delay );
#define MsSleep(d) UsSleep((unsigned long)d*1000U)

#endif // _TIMER_0_H_
