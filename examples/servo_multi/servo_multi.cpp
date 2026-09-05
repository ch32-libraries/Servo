// Four servos on the four TIM2 channels.
//
// SERVO_TIM2_CHn_PIN resolves to whichever pin that channel lands on for this
// chip family and remap group. On a CH32V003 in group 0 that is:
//
//   CH1 = PD4   CH2 = PD3   CH3 = PC0   CH4 = PD7
//
// PD7 is also NRST there. To use CH4 you must first disable the reset function
// via the option bytes (`minichlink -d`); until you do, that channel does
// nothing. The other three work as-is. Other families put these elsewhere and
// have their own caveats -- see the tables in Servo.h.
//
// All four share one 20 ms frame -- they are channels of a single timer -- but
// each holds its own pulse width.

#include "ch32fun.h"
#include "Servo.h"

int main(void)
{
	SystemInit();

	Servo a, b, c, d;
	a.attach(SERVO_TIM2_CH1_PIN);
	b.attach(SERVO_TIM2_CH2_PIN);
	c.attach(SERVO_TIM2_CH3_PIN);
	d.attach(SERVO_TIM2_CH4_PIN);

	a.writeMicroseconds(1000);
	b.writeMicroseconds(1300);
	c.writeMicroseconds(1700);
	d.writeMicroseconds(2000);

	// Nothing to do: the timer holds all four positions with no CPU involvement.
	while (1) Delay_Ms(1000);
}
