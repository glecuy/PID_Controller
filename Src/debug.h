
#ifndef _DEBUG_H_
#define _DEBUG_H_

#define DBG_PIN     PINC0

#define DBGPIN_OFF()      (PORTC &= ~(1 << DBG_PIN))
#define DBGPIN_ON()       (PORTC |= (1 << DBG_PIN))
#define DBGPIN_TOGGLE()   { if ( PORTC & (1 << DBG_PIN) ) DBGPIN_OFF(); else DBGPIN_ON(); }

#endif // _DEBUG_H_
