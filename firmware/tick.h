/* tick.h - 1 ms system tick (Timer2), defined in main.c */

#ifndef TICK_H
#define TICK_H

#include <stdint.h>

uint32_t millis_now(void);           /* milliseconds since power-up */

#endif
