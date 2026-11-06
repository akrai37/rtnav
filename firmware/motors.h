#ifndef RTNAV_MOTORS_H
#define RTNAV_MOTORS_H

#include "rtnav/nav.h"

// Timer0 fast PWM (~977 Hz) on OC0A/OC0B plus two direction pins per
// motor. Zero duty coasts: both direction pins low, PWM output off.
void motors_init(void);
void motors_set(nav_motors m);

#endif
