#ifndef AXES_H
#define AXES_H

#include "motor.h"

void   axes_init(void);
Motor *axes_get(char letter);   /* 'X','Y','Z' (case-insensitive), NULL if unknown */

#endif
