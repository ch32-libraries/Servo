// Smooth motion on the TIM2 backend.
//
// Note what the main loop does NOT contain: any call to service the servo.
// The ramp is advanced from the TIM2 update interrupt, so write() is
// fire-and-forget and the servo keeps moving while main() sleeps.

#include "ch32fun.h"
#include "Servo.h"

int main(void)
{
	SystemInit();

	Servo servo;
	servo.attach(SERVO_TIM2_CH1_PIN);
	servo.setSpeed(1000);   // us of pulse width per second: a full sweep in 1 s

	while (1)
	{
		servo.writeMicroseconds(SERVO_MAX_US);   // returns immediately
		while (servo.isMoving()) Delay_Ms(5);    // optional: wait it out

		servo.writeMicroseconds(SERVO_MIN_US);
		Delay_Ms(2000);                          // ramp completes during this
	}
}
