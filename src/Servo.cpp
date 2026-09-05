#include "Servo.h"

// A Servo declared at file scope needs these: because the class has a
// destructor, GCC emits a __cxa_atexit call to register it, and ch32fun
// provides no C++ runtime (FUNCONF_SUPPORT_CONSTRUCTORS covers construction
// only). On a microcontroller whose main() never returns, a global's
// destructor never runs, so discarding the registration is not a shortcut --
// it is the correct answer. Weak, so any real implementation overrides these
// rather than colliding with them, and dropped by --gc-sections when no
// file-scope object references them.
extern "C" {
__attribute__((weak)) void *__dso_handle = 0;
__attribute__((weak)) int __cxa_atexit(void (*)(void *), void *, void *)
{
	return 0;
}

// And these for a function-local `static Servo`: GCC guards the one-time
// construction against concurrent initialisation. There is no concurrency here
// -- one core, and a constructor that cannot itself be re-entered -- so the
// guard collapses to a plain "has it run yet" flag. The ABI puts that flag in
// the first byte of the guard object.
__attribute__((weak)) int __cxa_guard_acquire(uint8_t *g) { return !*g; }
__attribute__((weak)) void __cxa_guard_release(uint8_t *g) { *g = 1; }
__attribute__((weak)) void __cxa_guard_abort(uint8_t *) { }
}

#define SERVO_CENTER_US ((SERVO_MIN_US + SERVO_MAX_US) / 2)

// Reciprocal of the frame rate in Q16, folded at build time. See write().
#define SERVO_FRAME_Q16 ((65536u + (SERVO_FRAME_HZ / 2)) / SERVO_FRAME_HZ)

static inline uint16_t servo_clamp(uint16_t us)
{
	if (us < SERVO_MIN_US) return SERVO_MIN_US;
	if (us > SERVO_MAX_US) return SERVO_MAX_US;
	return us;
}

// The registry exists only when something iterates servos: the SysTick pulse
// chain, or the ramp advance. A plain TIM2 build has neither and pays nothing.
#if SERVO_NEED_REGISTRY
// `volatile` on the pointers, not the pointee: the handler walks this array, and
// without it the compiler is free to conclude the stores in attach()/detach()
// are dead because no visible code reads them back.
static Servo *volatile s_servos[SERVO_MAX_SERVOS];
#endif

// ---------------------------------------------------------------------------
// Ramp advance. Called once per frame from whichever interrupt the active
// backend already runs, so the application never has to poll.
// ---------------------------------------------------------------------------

#if SERVO_NEED_REGISTRY
void servo_frame_tick(void)
{
#if SERVO_SMOOTH
	for (uint8_t i = 0; i < SERVO_MAX_SERVOS; i++)
	{
		Servo *s = s_servos[i];
		if (!s) continue;

		uint16_t c = s->current;
		const uint16_t t = s->target;
		if (c == t) continue;

		const uint16_t st = s->step;
		if (st == 0)
		{
			c = t;                                  // no rate set: jump
		}
		else if (c < t)
		{
			// Landing exactly on the target when the remainder is short is
			// what keeps the ramp from overshooting or resting near-but-not-on
			// the commanded position.
			c = ((uint16_t)(t - c) <= st) ? t : (uint16_t)(c + st);
		}
		else
		{
			c = ((uint16_t)(c - t) <= st) ? t : (uint16_t)(c - st);
		}
		s->current = c;
		s->apply(c);
	}
#endif
}
#endif

// ---------------------------------------------------------------------------
// SysTick backend
// ---------------------------------------------------------------------------

#if SERVO_USE_SYSTICK

// CH32V10x's SysTick is a different peripheral shape from every other target
// this backend supports: no SR status register to clear at all, and CMP is a
// 64-bit register that ch32fun's own reference example
// (ch32fun/examples_v10x/systick_irq) writes one byte at a time rather than
// as a single store -- "In CH32V103 we can write systick regs only in 8bit
// manner." That example never touches SR either, so clearing nothing is the
// verified behavior, not an oversight. These two macros isolate that one
// family's quirk so the rest of this backend reads identically for every
// other target. Macros rather than inline functions: on every other target
// this expands back to the exact same `SysTick->CMP = <expr>;` token
// sequence that was here before, which matters because the two write sites
// in SysTick_Handler below are identical enough for the compiler to fold
// into one shared store -- a wrapper function, even always_inline, disturbed
// that folding and cost 4 bytes on CH32V003.
#if defined(CH32V10x)
#define SERVO_SYSTICK_CLEAR_PENDING() ((void)0) // no SR on this target
#define SERVO_SYSTICK_WRITE_CMP(v) do { \
		uint64_t v_ = (v); \
		SysTick->CMP0 = (uint8_t)v_; \
		SysTick->CMP1 = (uint8_t)(v_ >> 8); \
		SysTick->CMP2 = (uint8_t)(v_ >> 16); \
		SysTick->CMP3 = (uint8_t)(v_ >> 24); \
		SysTick->CMP4 = (uint8_t)(v_ >> 32); \
		SysTick->CMP5 = (uint8_t)(v_ >> 40); \
		SysTick->CMP6 = (uint8_t)(v_ >> 48); \
		SysTick->CMP7 = (uint8_t)(v_ >> 56); \
	} while (0)
#else
#define SERVO_SYSTICK_CLEAR_PENDING() (SysTick->SR = 0)
#define SERVO_SYSTICK_WRITE_CMP(v) (SysTick->CMP = (v))
#endif

// Index of the servo whose pulse is currently HIGH. SERVO_MAX_SERVOS means we
// are in the idle gap that closes each frame.
static volatile uint8_t s_slot = SERVO_MAX_SERVOS;
static uint32_t s_frame;    // absolute tick at which the current frame began
static uint8_t s_running;

extern "C" void SysTick_Handler(void) SERVO_ISR_ATTR;
void SysTick_Handler(void)
{
	SERVO_SYSTICK_CLEAR_PENDING();

	uint8_t i;
	if (s_slot >= SERVO_MAX_SERVOS)
	{
		// The idle gap expired, so this is the start of a new frame. Taking
		// the frame origin from CMP rather than CNT keeps frames exactly
		// SERVO_FRAME_TICKS apart no matter how late this handler ran.
		s_frame = SysTick->CMP;
		servo_frame_tick();
		i = 0;
	}
	else
	{
		// May be null if this servo detached mid-pulse; detach() drove the
		// line low itself, so there is nothing left to do for it here.
		Servo *s = s_servos[s_slot];
		if (s) funDigitalWrite(s->pin, FUN_LOW);
		i = (uint8_t)(s_slot + 1);
	}

	while (i < SERVO_MAX_SERVOS && !s_servos[i]) i++;

	if (i < SERVO_MAX_SERVOS)
	{
		Servo *s = s_servos[i];
		s_slot = i;
		funDigitalWrite(s->pin, FUN_HIGH);
		// Advancing CMP by an exact delta, never recomputing it from "now",
		// is what keeps pulse widths free of accumulated interrupt latency.
		SERVO_SYSTICK_WRITE_CMP(SysTick->CMP + (uint32_t)s->current * SERVO_TICKS_PER_US);
	}
	else
	{
		s_slot = SERVO_MAX_SERVOS;
		SERVO_SYSTICK_WRITE_CMP(s_frame + SERVO_FRAME_TICKS);
	}
}

static void servo_engine_start(void)
{
	if (s_running) return;
	s_running = 1;
	s_slot = SERVO_MAX_SERVOS;          // begin in the gap; first IRQ opens a frame

	// Borrow SysTick, do not commandeer it. The counter keeps free-running and
	// its clock source is untouched -- we only add the compare interrupt. That
	// is what lets Delay_Us(), funSysTick32() and a TIM2-based library such as
	// Ticker keep working underneath us. Note in particular: no STRE (that
	// would turn on auto-reload) and no write to CNT.
	//
	// Reading the counter through funSysTick32() rather than SysTick->CNT
	// directly reuses ch32fun's own portable accessor -- it already resolves
	// to the right field name per family (CNT vs CNTL on CH32V10x/X03x), so
	// this file doesn't need to know that on top of what SERVO_SYSTICK_WRITE_CMP
	// above already tracks for writes.
	SERVO_SYSTICK_WRITE_CMP(funSysTick32() + 100 * SERVO_TICKS_PER_US);
	SERVO_SYSTICK_CLEAR_PENDING();
	SysTick->CTLR |= SYSTICK_CTLR_STIE;
	NVIC_EnableIRQ(SysTick_IRQn);
}

static void servo_engine_stop_if_idle(void)
{
	for (uint8_t i = 0; i < SERVO_MAX_SERVOS; i++)
		if (s_servos[i]) return;

	SysTick->CTLR &= ~SYSTICK_CTLR_STIE;
	NVIC_DisableIRQ(SysTick_IRQn);
	s_running = 0;
}

#else // ---------------------------------------------------------------------
// TIM2 backend
// ---------------------------------------------------------------------------

static uint8_t s_tim2_up;

static void servo_tim2_init(void)
{
	if (s_tim2_up) return;
	s_tim2_up = 1;

	// AFIO's clock is already on: this is only ever called from attach_ch(),
	// which enables it (alongside the pin's own GPIO port) before reaching
	// here. TIM2's clock is this backend's own to enable.
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

	// Masked on the way in as well as out: the remap field is not the top of
	// PCFR1 on every family (on CH32X03x TIM3's field sits directly above it),
	// so an out-of-range SERVO_TIM2_REMAP must not be allowed to spill. Both
	// operands are compile-time constants, so this folds to one immediate.
	AFIO->PCFR1 = (AFIO->PCFR1 & ~SERVO_TIM2_REMAP_MASK)
	            | (((uint32_t)SERVO_TIM2_REMAP << SERVO_TIM2_REMAP_SHIFT)
	               & SERVO_TIM2_REMAP_MASK);

	// A 1 MHz tick makes CH*CVR the pulse width in microseconds outright, so
	// writeMicroseconds() is a single store with no scaling arithmetic.
	TIM2->PSC    = SERVO_TIM2_PSC;
	TIM2->ATRLR  = SERVO_FRAME_US - 1;
	TIM2->CTLR1  = TIM_ARPE;
	TIM2->SWEVGR = TIM_UG;              // latch PSC and ATRLR

#if SERVO_SMOOTH
	// The only interrupt this backend ever enables, and only to advance ramps.
	TIM2->INTFR = (uint16_t)~TIM_UIF;
	TIM2->DMAINTENR |= TIM_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
#endif

#ifdef SERVO_TIM2_NEEDS_MOE
	// CH32X03x only: its TIM2 is an advanced-control timer, so the channel
	// outputs stay disconnected until the main output enable is set. Every
	// other family's TIM2 is general-purpose and has no BDTR at all.
	TIM2->BDTR |= TIM_MOE;
#endif

	TIM2->CTLR1 |= TIM_CEN;
}

static void servo_tim2_enable_ch(uint8_t ch)
{
	const uint8_t idx = (uint8_t)(ch - 1);
	const uint8_t sh  = (uint8_t)((idx & 1) * 8);
	// OCxM = 110 (PWM mode 1) with OCxPE (preload) set: a CH*CVR write lands at
	// the next update event. That is not an optimisation -- without it a write
	// landing mid-pulse emits one malformed pulse, which the servo obeys as a
	// real command.
	//
	// PWM mode 1 with CCxP clear (its reset state, which we never change) drives
	// the pin HIGH while CNT < CH*CVR, so CH*CVR is the pulse width and the line
	// idles LOW between pulses.
	//
	// Note for anyone checking this against the manual: CH32V003RM contradicts
	// itself here. The OCxM field descriptions in both the TIM1 and TIM2 chapters
	// say PWM mode 1 is "active when the core counter value falls below the
	// compare capture register value" -- what we rely on -- while the edge-
	// alignment prose in both chapters says OCxREF "rises to high when the core
	// counter value is greater than" it, describing mode 2. The field
	// descriptions are normative, agree with each other, and agree with the
	// manual's own statement that CCR sets the duty cycle. If a scope ever shows
	// an inverted pulse (~18.5 ms high for a 1500 us command), the fix is one
	// bit: OR TIM_CC1P << (4 * idx) into CCER below.
	const uint16_t cfg = (uint16_t)(0x68u << sh);
	const uint16_t msk = (uint16_t)(0xffu << sh);

	if (idx < 2) TIM2->CHCTLR1 = (uint16_t)((TIM2->CHCTLR1 & ~msk) | cfg);
	else         TIM2->CHCTLR2 = (uint16_t)((TIM2->CHCTLR2 & ~msk) | cfg);

	TIM2->CCER |= (uint16_t)(TIM_CC1E << (4 * idx));
}

#if SERVO_SMOOTH
extern "C" void TIM2_IRQHandler(void) SERVO_ISR_ATTR;
void TIM2_IRQHandler(void)
{
	TIM2->INTFR = (uint16_t)~TIM_UIF;
	servo_frame_tick();
}
#endif

#endif // SERVO_USE_SYSTICK

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

Servo::Servo()
	: current(SERVO_CENTER_US), target(SERVO_CENTER_US),
#if SERVO_SMOOTH
	  step(0),
#endif
	  pin(0), attached(0)
#if !SERVO_USE_SYSTICK
	  , ch(0)
#endif
{
}

void Servo::apply(uint16_t us)
{
#if SERVO_USE_SYSTICK
	(void)us;                       // the handler reads `current` directly
#else
	// CH1CVR..CH4CVR sit at TIM2 + 0x34 with a 4-byte stride. That is the
	// architectural layout, identical in CH32V003RM, CH32V00XRM, CH32X035RM,
	// CH32xRM, CH32FV2x_V3xRM and CH32L103RM -- but it is NOT what indexing
	// ch32fun's TIM_TypeDef gives you, on two of the seven families:
	//
	//   - CH32V10x/V20x/V30x declare the four members `uint16_t`, so
	//     `(&TIM2->CH1CVR)[n]` strides 2 bytes and channels 2-4 land on
	//     padding or on the wrong register.
	//   - CH32L103's header interleaves TIM4 aliases ahead of them, putting its
	//     CH1CVR at struct offset 0x38 -- one register too far, so every
	//     channel is off by one and channel 4 would write the pulse width
	//     into BDTR.
	//
	// So address the register directly rather than trusting the struct. Still
	// one indexed store, and unchanged on CH32V003.
	if (ch) *(volatile uint32_t *)((uintptr_t)TIM2 + 0x34u + 4u * (ch - 1u)) = us;
#endif
}

#if SERVO_USE_SYSTICK
bool Servo::attach(uint8_t p)
#else
// The channel is resolved by the inline attach() in the header, so the fold
// happens in the caller's translation unit and does not depend on LTO.
bool Servo::attach_ch(uint8_t p, uint8_t channel)
#endif
{
	if (attached) return true;

#if SERVO_NEED_REGISTRY
	uint8_t slot = 0;
	while (slot < SERVO_MAX_SERVOS && s_servos[slot]) slot++;
	if (slot >= SERVO_MAX_SERVOS) return false;
#endif

	pin = p;
	// Honour a position commanded before attach. Clamping also rescues a
	// file-scope Servo whose constructor never ran (a build without
	// FUNCONF_SUPPORT_CONSTRUCTORS leaves it zeroed in BSS): it starts at the
	// minimum rather than emitting a zero-width pulse.
	const uint16_t start = servo_clamp(target);
	target = start;
	current = start;
	RCC->APB2PCENR |= RCC_APB2Periph_AFIO | SERVO_PORT_CLK(p);

#if SERVO_USE_SYSTICK
	funDigitalWrite(p, FUN_LOW);
	funPinMode(p, SERVO_OUT_PP);
	attached = 1;
	s_servos[slot] = this;          // single store: the ISR sees null or valid
	servo_engine_start();
#else
	ch = channel;
	funPinMode(p, SERVO_AF_PP);
	servo_tim2_init();
	servo_tim2_enable_ch(ch);
	attached = 1;
	apply(current);
	TIM2->SWEVGR = TIM_UG;          // load the preloaded compare value now
#if SERVO_NEED_REGISTRY
	s_servos[slot] = this;
#endif
#endif
	return true;
}

void Servo::detach()
{
	if (!attached) return;
	attached = 0;

#if SERVO_NEED_REGISTRY
	for (uint8_t i = 0; i < SERVO_MAX_SERVOS; i++)
		if (s_servos[i] == this) { s_servos[i] = 0; break; }
#endif

#if !SERVO_USE_SYSTICK
	TIM2->CCER &= (uint16_t)~(TIM_CC1E << (4 * (ch - 1)));
	ch = 0;
#endif

	// Leave the line passive rather than holding an indeterminate pulse.
	funDigitalWrite(pin, FUN_LOW);
	funPinMode(pin, SERVO_OUT_PP);

#if SERVO_USE_SYSTICK
	servo_engine_stop_if_idle();
#endif
}

void Servo::writeMicroseconds(uint16_t us)
{
	us = servo_clamp(us);
	target = us;
#if SERVO_SMOOTH
	// While attached the ramp carries `current` to the target from the ISR.
	// Detached there is no ISR running, so track it directly -- otherwise
	// attach() would start from the wrong place.
	if (!attached) current = us;
#else
	current = us;
	apply(us);
#endif
}

#if SERVO_ENABLE_ANGLE
// Degrees -> microseconds without a runtime division.
//
// rv32ec has neither multiply nor divide. That matters more than it looks: GCC
// cannot use its usual "divide by a constant becomes a magic multiply" trick,
// because the magic multiply is itself unavailable -- so a plain
// `angle * span / SERVO_ANGLE_MAX` emits a real division and drags in
// __divsi3, __udivsi3 and __umodsi3, about 184 bytes for one conversion.
//
// Multiplying by a compile-time constant, by contrast, expands to a shift/add
// chain with no library call at all. So we fold the reciprocal in at build
// time and multiply-then-shift. The +ANGLE_MAX/2 term rounds the scale rather
// than truncating it.
#define SERVO_ANGLE_Q16 (((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * 65536u \
	+ (SERVO_ANGLE_MAX / 2)) / SERVO_ANGLE_MAX)

void Servo::write(uint8_t angle)
{
	// The top endpoint is handled exactly rather than trusting the rounded
	// scale, so write(SERVO_ANGLE_MAX) always lands precisely on SERVO_MAX_US.
	if (angle >= SERVO_ANGLE_MAX)
	{
		writeMicroseconds(SERVO_MAX_US);
		return;
	}
	writeMicroseconds((uint16_t)(SERVO_MIN_US
		+ (((uint32_t)angle * SERVO_ANGLE_Q16) >> 16)));
}
#endif

#if SERVO_SMOOTH
void Servo::setSpeed(uint16_t us_per_second)
{
	// Same reciprocal-multiply reason as write() above: a runtime divide here
	// would pull the libgcc division routines into every smoothing build.
	const uint16_t s = (uint16_t)(((uint32_t)us_per_second * SERVO_FRAME_Q16) >> 16);
	// ponytail: step quantised to whole microseconds per frame, so rates come
	// in SERVO_FRAME_HZ increments and the slowest is one step per frame.
	// Upgrade path if finer rates are ever wanted: keep the position in 8.8
	// fixed point and accumulate the remainder.
	// A non-zero rate must still move, or the servo sits still at a speed the
	// caller explicitly asked for.
	step = (us_per_second && !s) ? 1 : s;
}
#endif
