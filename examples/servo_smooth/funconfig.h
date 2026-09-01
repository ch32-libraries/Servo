#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

// Compile in ramped motion. write() then returns immediately and the ramp is
// advanced by the TIM2 update interrupt -- the only interrupt the hardware
// backend ever enables.
#define SERVO_SMOOTH 1

#endif
