// Four servos on the four TIM2 channels, remap group 0.
//
//   CH1 = PD4   CH2 = PD3   CH3 = PC0   CH4 = PD7
//
// PD7 is also NRST. To use CH4 you must first disable the reset function via
// the option bytes (`minichlink -d`); until you do, that channel does nothing.
// The other three work as-is.
//
// All four share one 20 ms frame -- they are channels of a single timer -- but
// each holds its own pulse width.

#include "ch32fun.h"
#include "Servo.h"

int main(void)
{
	SystemInit();

	Servo a, b, c, d;
	a.attach(PD4);
	b.attach(PD3);
	c.attach(PC0);
	d.attach(PD7);

	a.writeMicroseconds(1000);
	b.writeMicroseconds(1300);
	c.writeMicroseconds(1700);
	d.writeMicroseconds(2000);

	// Nothing to do: the timer holds all four positions with no CPU involvement.
	while (1) Delay_Ms(1000);
}
