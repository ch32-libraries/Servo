// Three servos on the SysTick backend, on pins with no TIM2 channel function.
//
// TIM2 is never touched by this build: no clock enable, no register write. It
// is entirely yours -- run a TIM2-based library such as Ticker alongside this
// if you like.
//
// SysTick keeps free-running with its clock source untouched, so Delay_Ms()
// below still measures what it says.

#include "ch32fun.h"
#include "Servo.h"

int main(void)
{
	SystemInit();

	Servo a, b, c;
	a.attach(PC3);   // none of these are TIM2 channel pins
	a.writeMicroseconds(1000);
	b.attach(PC4);
	b.writeMicroseconds(1500);
	c.attach(PD2);
	c.writeMicroseconds(2000);

	while (1)
	{
		// A blocking main loop is fine for holding position; the handler keeps
		// pulsing regardless. It does add jitter, which is this backend's
		// trade-off against the timer it saves.
		Delay_Ms(1000);
	}
}
