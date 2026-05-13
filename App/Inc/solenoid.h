#ifndef SOLENOID_H
#define SOLENOID_H

#include <stdbool.h>

/* Vacuum / pick-and-place solenoid on PB0 (see WIRING.txt).
 * Drives a transistor; active-high. Powers up OFF. */

void solenoid_init(void);
void solenoid_set(bool on);
bool solenoid_toggle(void);   /* returns new state */
bool solenoid_get(void);

#endif
