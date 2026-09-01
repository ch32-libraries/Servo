#pragma once
// Servo: RC servo pulse generation for CH32 microcontrollers, on ch32fun.
//
// Two interchangeable backends, chosen at build time. The public API is
// identical under both, so switching is a -D plus a pin change:
//
//   (default)           TIM2 hardware PWM. Up to 4 servos. The timer generates
//                       the pulses; with SERVO_SMOOTH off this backend uses no
//                       interrupts at all and the CPU is uninvolved after
//                       attach(). Pulse width is immune to main-loop workload.
//
//   SERVO_USE_SYSTICK   SysTick compare-match IRQ + GPIO writes. Consumes no
//                       general-purpose timer, so TIM2 stays free for the rest
//                       of your firmware (including a TIM2-based library like
//                       Ticker). Any GPIO pin. Pulse width is subject to
//                       interrupt latency.
//
// Configuration is entirely compile-time. There are no runtime pin tables and
// no runtime conflict checks: a wrong #define gives you wrong hardware, not a
// diagnostic. Where a mistake is statically detectable it fails the build.

#include "ch32fun.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#ifndef SERVO_USE_SYSTICK
#define SERVO_USE_SYSTICK 0   // 1 = SysTick backend, 0 = TIM2 backend
#endif

#ifndef SERVO_SMOOTH
#define SERVO_SMOOTH 0        // 1 = compile in ramped motion + setSpeed/isMoving
#endif

#ifndef SERVO_ENABLE_ANGLE
#define SERVO_ENABLE_ANGLE 1  // 1 = compile in write(degrees); costs one __mulsi3
#endif

#ifndef SERVO_FRAME_US
#define SERVO_FRAME_US 20000  // frame period in us (20000 = 50 Hz)
#endif

// The nominal 1000-2000 us range is a starting point, not a fact about your
// servo. Real units vary; many accept 500-2500. Calibrate against the servo you
// actually have -- this pair is the calibration knob.
#ifndef SERVO_MIN_US
#define SERVO_MIN_US 1000
#endif
#ifndef SERVO_MAX_US
#define SERVO_MAX_US 2000
#endif

#ifndef SERVO_ANGLE_MAX
#define SERVO_ANGLE_MAX 180   // write(0..SERVO_ANGLE_MAX) spans MIN_US..MAX_US
#endif

#ifndef SERVO_MAX_SERVOS
#define SERVO_MAX_SERVOS 4
#endif

// TIM2 output remap group. This is a property of the timer, not of one servo:
// it moves all four channels at once. See the table in servo_tim2_channel().
#ifndef SERVO_TIM2_REMAP
#define SERVO_TIM2_REMAP 0
#endif

// State shared with an interrupt handler must be volatile: the compiler cannot
// see that a handler ever runs, so it will otherwise delete stores it has proved
// dead from visible code alone. Not theoretical -- it silently broke
// single-servo builds under LTO, while multi-servo builds happened to survive
// because the same analysis did not fold.
//
// Which handler exists depends on the configuration, and the plain TIM2 backend
// has none at all -- so this collapses to nothing there and costs zero.
#if SERVO_USE_SYSTICK || SERVO_SMOOTH
#define SERVO_ISR_SHARED volatile
#else
#define SERVO_ISR_SHARED
#endif

// The interrupt attribute is overridable purely so the pulse and ramp logic can
// be compiled and exercised on a host in test/. Firmware builds always get the
// real attribute -- nothing else may override this.
#ifndef SERVO_ISR_ATTR
#define SERVO_ISR_ATTR __attribute__((interrupt))
#endif

// ---------------------------------------------------------------------------
// Portability. This block is the ONLY MCU-dependent code in the library.
// Porting to another CH32 family means adding a branch here.
// ---------------------------------------------------------------------------

#if defined(CH32V003)
	#define SERVO_AF_PP       GPIO_CFGLR_OUT_10Mhz_AF_PP  // timer-driven output
	#define SERVO_OUT_PP      GPIO_CFGLR_OUT_10Mhz_PP     // plain output (detach)
	#define SERVO_TIM2_REMAP_SHIFT 8                      // AFIO->PCFR1[9:8]
	#define SERVO_TIM2_MAX_CH 4
	// Port clock bit in RCC->APB2PCENR for a ch32fun packed pin: GPIOA is bit 2,
	// and the packed pin's high nibble is the port index (A=0, B=1, C=2, D=3).
	#define SERVO_PORT_CLK(pin) (1u << (2 + ((pin) >> 4)))
#else
	#error "Servo: unsupported target MCU. Only CH32V003 has been ported and \
verified on hardware. Adding a family means adding a branch to the portability \
block in Servo.h -- see the README's Portability section. Building an unported \
target would silently misconfigure registers, so this is a hard error."
#endif

// ---------------------------------------------------------------------------
// Derived constants and static configuration checks
// ---------------------------------------------------------------------------

#define SERVO_FRAME_HZ    (1000000 / SERVO_FRAME_US)
#define SERVO_TIM2_PSC    ((FUNCONF_SYSTEM_CORE_CLOCK / 1000000) - 1)  // 1 MHz tick
#define SERVO_TICKS_PER_US DELAY_US_TIME               // SysTick ticks per us
#define SERVO_FRAME_TICKS ((uint32_t)SERVO_FRAME_US * SERVO_TICKS_PER_US)

// The registry exists only when something has to iterate servos: the SysTick
// backend's pulse chain, or the ramp advance. A plain TIM2 build has neither.
#define SERVO_NEED_REGISTRY (SERVO_USE_SYSTICK || SERVO_SMOOTH)

static_assert(SERVO_MIN_US < SERVO_MAX_US,
	"Servo: SERVO_MIN_US must be below SERVO_MAX_US");
static_assert(SERVO_MAX_US < SERVO_FRAME_US,
	"Servo: SERVO_MAX_US must fit inside SERVO_FRAME_US");
static_assert(SERVO_FRAME_US >= 2000 && SERVO_FRAME_US <= 40000,
	"Servo: SERVO_FRAME_US outside the range servos tolerate (2000..40000)");
static_assert(SERVO_MAX_SERVOS > 0 && SERVO_MAX_SERVOS < 255,
	"Servo: SERVO_MAX_SERVOS must be in (0, 255)");
static_assert(SERVO_ANGLE_MAX > 0 && SERVO_ANGLE_MAX <= 255,
	"Servo: SERVO_ANGLE_MAX must be in (0, 255]");
static_assert(SERVO_TIM2_REMAP >= 0 && SERVO_TIM2_REMAP <= 3,
	"Servo: SERVO_TIM2_REMAP must be 0..3");

#if SERVO_USE_SYSTICK
// The SysTick backend pulses servos one at a time, so every pulse plus the
// idle gap must fit in one frame. If the widths could fill the frame exactly
// the gap would be zero, the next compare value would already be in the past,
// and the missed interrupt would stall every servo until the 32-bit counter
// wrapped. Keeping this strict is what makes that unrepresentable.
static_assert((uint32_t)SERVO_MAX_SERVOS * SERVO_MAX_US < SERVO_FRAME_US,
	"Servo: SERVO_MAX_SERVOS * SERVO_MAX_US must be strictly less than "
	"SERVO_FRAME_US -- the sequential pulse chain needs an idle gap to close "
	"each frame. Reduce SERVO_MAX_SERVOS or SERVO_MAX_US.");
#else
static_assert(SERVO_MAX_SERVOS <= SERVO_TIM2_MAX_CH,
	"Servo: the TIM2 backend has one channel per servo, so SERVO_MAX_SERVOS "
	"cannot exceed 4. Use SERVO_USE_SYSTICK for more.");
#endif

// ---------------------------------------------------------------------------
// Compile-time pin resolution (TIM2 backend)
// ---------------------------------------------------------------------------

#if !SERVO_USE_SYSTICK
// Declared, never defined. A valid compile-time-constant pin folds the call
// away and costs nothing. Anything else -- an invalid pin, or a pin that is not
// a constant -- leaves the call and fails at link with the reason in the name.
extern "C" void servo_error__pin_is_not_a_tim2_channel_for_configured_remap(void);

// TIM2 channel pins by AFIO->PCFR1 remap group on CH32V003:
//
//   remap  CH1        CH2   CH3   CH4
//   0      PD4        PD3   PC0   PD7 (*)
//   1      PC5        PC2   PD2   PC1
//   2      PC1        PD3   PC0   PD7 (*)
//   3      PC1        PC7   PD6   PD5
//
//   (*) PD7 is also NRST. Using CH4 in groups 0/2 requires disabling the reset
//       function via the option bytes (minichlink -d).
//
// NB: the comments on AFIO_PCFR1_TIM2_REMAP_* in ch32v003hw.h are wrong for two
// entries -- they say CH4/PD4 for group 0 and CH3/PD4 for group 1. The table
// above follows the datasheet and ch32fun's own examples/tim2_pwm_remap, which
// agree with each other.
__attribute__((always_inline))
static inline uint8_t servo_tim2_channel(uint8_t pin)
{
#if SERVO_TIM2_REMAP == 0
	if (pin == PD4) return 1;
	if (pin == PD3) return 2;
	if (pin == PC0) return 3;
	if (pin == PD7) return 4;
#elif SERVO_TIM2_REMAP == 1
	if (pin == PC5) return 1;
	if (pin == PC2) return 2;
	if (pin == PD2) return 3;
	if (pin == PC1) return 4;
#elif SERVO_TIM2_REMAP == 2
	if (pin == PC1) return 1;
	if (pin == PD3) return 2;
	if (pin == PC0) return 3;
	if (pin == PD7) return 4;
#else
	if (pin == PC1) return 1;
	if (pin == PC7) return 2;
	if (pin == PD6) return 3;
	if (pin == PD5) return 4;
#endif
	servo_error__pin_is_not_a_tim2_channel_for_configured_remap();
	return 0;
}
#endif // !SERVO_USE_SYSTICK

// ---------------------------------------------------------------------------

// Handlers that walk servo state. Declared here so the friend declarations
// below refer to these C-linkage functions rather than introducing new ones.
#if SERVO_USE_SYSTICK
extern "C" void SysTick_Handler(void);
#elif SERVO_SMOOTH
extern "C" void TIM2_IRQHandler(void);
#endif

class Servo
{
public:
	Servo();

	// Detaches, so a servo going out of scope while still attached (e.g. a
	// stack-local) cannot leave a dangling pointer in the registry an
	// interrupt handler walks.
	~Servo() { detach(); }

	// A copy would leave two objects disagreeing about which one the registry
	// points at, so copying is disabled rather than left broken.
	Servo(const Servo &) = delete;
	Servo &operator=(const Servo &) = delete;

	// `pin` is a ch32fun packed pin constant (PD4, PC0, ...). Under the TIM2
	// backend it must be a compile-time constant naming a channel pin of the
	// configured remap group, or the build fails at link. Returns false only
	// when the registry is full.
#if SERVO_USE_SYSTICK
	bool attach(uint8_t pin);
#else
	// Inline and always_inline so servo_tim2_channel() folds in the caller's
	// translation unit. Were this left to LTO, a build without it would fail
	// to link on a perfectly valid pin.
	__attribute__((always_inline))
	inline bool attach(uint8_t pin) { return attach_ch(pin, servo_tim2_channel(pin)); }
#endif

	// Stops pulses and leaves the line driven LOW, so a detached servo is
	// passive rather than holding an indeterminate pulse.
	void detach();

	// Clamped to SERVO_MIN_US..SERVO_MAX_US. Safe before attach(): the value is
	// retained and applied when the servo is attached.
	void writeMicroseconds(uint16_t us);

	// The commanded position, after clamping. Under SERVO_SMOOTH this is the
	// target, which is what was commanded -- not the in-flight position.
	uint16_t readMicroseconds() const { return target; }

#if SERVO_ENABLE_ANGLE
	// 0..SERVO_ANGLE_MAX mapped linearly onto SERVO_MIN_US..SERVO_MAX_US.
	void write(uint8_t angle);
#endif

#if SERVO_SMOOTH
	// Ramp rate in microseconds of pulse width per second. 0 (the default)
	// means no ramp: a new position takes effect on the next frame.
	//
	// The step is quantised to whole microseconds per frame, so achievable
	// rates come in SERVO_FRAME_HZ increments and the slowest non-zero rate is
	// one step per frame -- 50 us/s at the default 50 Hz, a full 1000 us sweep
	// in 20 seconds. Slower than any real servo move, hence no accumulator.
	void setSpeed(uint16_t us_per_second);

	bool isMoving() const { return current != target; }
#endif

private:
#if SERVO_USE_SYSTICK
	friend void SysTick_Handler(void);
#elif SERVO_SMOOTH
	friend void TIM2_IRQHandler(void);
#endif
#if SERVO_NEED_REGISTRY
	friend void servo_frame_tick(void);
#endif

	// Emitted position. Equal to `target` unless a ramp is in flight.
	SERVO_ISR_SHARED uint16_t current;
	// Commanded position.
	SERVO_ISR_SHARED uint16_t target;
#if SERVO_SMOOTH
	// Microseconds of travel per frame; 0 = jump straight to target.
	SERVO_ISR_SHARED uint16_t step;
#endif
#if SERVO_USE_SYSTICK
	// SysTick_Handler drives this pin directly, so the handler reads it.
	volatile uint8_t pin;
#else
	// Under TIM2 the pin is only touched by attach()/detach(); the handler, if
	// there is one, goes through `ch` instead.
	uint8_t pin;
#endif
	uint8_t attached;               // 0/1; never read from a handler
#if !SERVO_USE_SYSTICK
	SERVO_ISR_SHARED uint8_t ch;    // TIM2 channel 1..4; read by apply()
#endif

#if !SERVO_USE_SYSTICK
	bool attach_ch(uint8_t pin, uint8_t channel);
#endif
	void apply(uint16_t us);
};
