
#ifndef _TIMER_0_H_
#define _TIMER_0_H_

#define TIMER_TICK_MS  25

int timer0_init(void);
unsigned long timer0_GetTicks(void);

void UsSleep( unsigned long Delay );
#define MsSleep(d) UsSleep((unsigned long)d*1000U)

#endif // _TIMER_0_H_
