
#ifndef _TIMER_1_H_
#define _TIMER_1_H_

int timer1_init(void);
unsigned long timer1_GetTicks(void);

void UsSleep( unsigned long Delay );
#define MsSleep(d) UsSleep((unsigned long)d*1000U)

#endif // _TIMER_1_H_
