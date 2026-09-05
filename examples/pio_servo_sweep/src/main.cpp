// Servo sweep on the TIM2 backend (the default).
//
// SERVO_TIM2_CH1_PIN is whichever pin TIM2 CH1 lands on for this chip family
// and remap group -- PD4 on the CH32V003 this project targets. The timer
// generates the pulses on its own: with SERVO_SMOOTH off this build enables no
// interrupts at all.

#include "ch32fun.h"
#include "Servo.h"

int main(void)
{
	SystemInit();

	Servo servo;
	servo.attach(SERVO_TIM2_CH1_PIN);

	while (1)
	{
		for (uint16_t us = SERVO_MIN_US; us < SERVO_MAX_US; us += 10)
		{
			servo.writeMicroseconds(us);
			Delay_Ms(20);
		}
		for (uint16_t us = SERVO_MAX_US; us > SERVO_MIN_US; us -= 10)
		{
			servo.writeMicroseconds(us);
			Delay_Ms(20);
		}
	}
}
