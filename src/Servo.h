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
//
// Both backends build for every CH32 family ch32fun defines a TIM2 register
// layout for -- CH32V003, CH32V00x, CH32V10x, CH32V20x, CH32V30x, CH32L103 and
// CH32X03x. The TIM2 backend additionally needs a per-family output-remap pin
// table, so which pins are available depends on the family and the configured
// remap group; a pin that is not a channel of that combination fails at link.
// Building either backend for a family with no register layout at all fails at
// compile time rather than misconfiguring registers.
//
// Only CH32V003 has been run on hardware. Every other family's tables were read
// off WCH reference manuals. See the portability block below, and the README's
// "Board support" section for what each family's support rests on.

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
// it moves all four channels at once. Legal values and the pins they select are
// per family -- see the channel-to-pin tables in the portability block below.
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
//
// The two backends need different things from a target, so they are gated
// separately instead of behind one combined check:
//
//   - The SysTick backend below needs only primitives confirmed identical
//     across every ch32fun family that has a TIM2 peripheral -- the push-pull
//     output config and the port clock-enable bit pattern. It never touches
//     TIM2 itself, so it doesn't need anything family-specific beyond that.
//   - The TIM2 hardware-PWM backend additionally needs to know where that
//     family's remap field sits, and which pin each channel lands on in each
//     remap group. That is the bulk of this block: a short descriptor chain,
//     then one table of four defines per family and group.
// ---------------------------------------------------------------------------

// Common to every ch32fun family with a TIM2 -- this list mirrors ch32fun.h's
// own family-dispatch chain (which register header it #includes) minus
// CH32H41x, which Ticker (a sibling TIM2-based library) excludes for having
// an incompatible clock/interrupt architecture. Confirmed identical across
// ch32v003hw.h, ch32x00xhw.h, ch32x03xhw.h, ch32v10xhw.h, ch32l103hw.h,
// ch32v20xhw.h and ch32v30xhw.h.
#if defined(CH32V003) || defined(CH32V002) || defined(CH32V00x) \
 || defined(CH32X03x) || defined(CH32V10x) || defined(CH32L103) \
 || defined(CH32V20x) || defined(CH32V30x)
	#define SERVO_OUT_PP      GPIO_CFGLR_OUT_10Mhz_PP     // plain output (detach)
	// Port clock bit in RCC->APB2PCENR for a ch32fun packed pin: GPIOA is bit 2,
	// and the packed pin's high nibble is the port index (A=0, B=1, C=2, D=3).
	// Same bit pattern on every family listed above.
	#define SERVO_PORT_CLK(pin) (1u << (2 + ((pin) >> 4)))
#else
	#error "Servo: unsupported target MCU -- ch32fun does not define a TIM2 \
register family for it, or Servo has not added it to the portability block \
below. See the README's Portability section. Building an unported target \
would silently misconfigure registers, so this is a hard error."
#endif

// TIM2 remap descriptors and pin tables, needed only when the TIM2 backend is
// actually selected -- the SysTick backend above never touches TIM2, so a
// family without a table is not an error there.
#if !SERVO_USE_SYSTICK

// TIM2 has four capture/compare channels, and GPIO_CFGLR_OUT_10Mhz_AF_PP has the
// same value, on every family here -- neither is per-family data.
#define SERVO_TIM2_MAX_CH 4
#define SERVO_AF_PP       GPIO_CFGLR_OUT_10Mhz_AF_PP  // timer-driven output

// Where each family's TIM2 output-remap field sits. Three different shapes, and
// two different names for the mask -- WCH spells it TIM2_REMAP on some families
// and TIM2_RM on others for the same register bits. Referencing each family's
// own constant rather than computing one means a wrong name fails to compile
// instead of writing a wrong mask.
#if defined(CH32V003)
	#define SERVO_TIM2_REMAP_SHIFT 8                      // AFIO->PCFR1[9:8]
	#define SERVO_TIM2_REMAP_MASK  AFIO_PCFR1_TIM2_REMAP


#elif defined(CH32V10x) || defined(CH32V20x) || defined(CH32V30x)
	#define SERVO_TIM2_REMAP_SHIFT 8                      // AFIO->PCFR1[9:8]
	#define SERVO_TIM2_REMAP_MASK  AFIO_PCFR1_TIM2_REMAP

#elif defined(CH32L103)
	// Same field position as above, but CH32L103RM calls the field
	// {TIM2_RM_H, TIM2_RM}: AFIO->PCFR2[21] is a third, high bit above these
	// two. We leave PCFR2 at its reset value, which pins the high bit to 0 and
	// makes groups 4-7 unreachable -- see the pin table below.
	#define SERVO_TIM2_REMAP_SHIFT 8                      // AFIO->PCFR1[9:8]
	#define SERVO_TIM2_REMAP_MASK  AFIO_PCFR1_TIM2_RM

#elif defined(CH32X03x)
	#define SERVO_TIM2_REMAP_SHIFT 18                     // AFIO->PCFR1[20:18]
	#define SERVO_TIM2_REMAP_MASK  AFIO_PCFR1_TIM2_REMAP
	// CH32X035RM puts TIM2 in the advanced-control timer module (chapter 12,
	// "TIM1/TIM2"), so unlike every other family here its outputs are gated by
	// BDTR.MOE (12.4.18, bit 15, reset 0). Without it the channels configure
	// correctly and the pin stays dead.
	#define SERVO_TIM2_NEEDS_MOE 1

#elif defined(CH32V002) || defined(CH32V00x)
	#define SERVO_TIM2_REMAP_SHIFT 14                     // AFIO->PCFR1[16:14]
	#define SERVO_TIM2_REMAP_MASK  AFIO_PCFR1_TIM2_RM

	// ch32x00xhw.h spells TIM2's control bits per-peripheral rather than with
	// the generic TIM_ prefix the rest of the families use. Same bit values,
	// different names, so alias them here and leave Servo.cpp family-agnostic.
	// Guarded, so they yield if ch32fun ever defines the generic names itself.
	#ifndef TIM_ARPE
		#define TIM_ARPE  TIM2_CTLR1_ARPE
	#endif
	#ifndef TIM_UG
		#define TIM_UG    TIM2_SWEVGR_UG
	#endif
	#ifndef TIM_CEN
		#define TIM_CEN   TIM2_CTLR1_CEN
	#endif
	#ifndef TIM_UIF
		#define TIM_UIF   TIM2_INTFR_UIF
	#endif
	#ifndef TIM_UIE
		#define TIM_UIE   TIM2_DMAINTENR_UIE
	#endif
	#ifndef TIM_CC1E
		#define TIM_CC1E  TIM2_CCER_CC1E
	#endif

#else
	#error "Servo: the TIM2 hardware-PWM backend has not been ported to this \
target -- its per-family output remap table doesn't exist. Build with \
SERVO_USE_SYSTICK=1 instead, or add this family's remap table to the \
portability block in Servo.h -- see the README's Portability section. \
Building an unported target would silently misconfigure registers, so this \
is a hard error."
#endif

// ---------------------------------------------------------------------------
// Channel-to-pin tables, by family and configured remap group.
//
// This is data, not logic: four defines per group, checked against the manual
// named above each block. servo_tim2_channel() below compares against whichever
// four are defined and needs no family branch of its own. A channel a family
// does not route in the configured group defines no macro; a group with no
// table defines none at all. Either way the pin fails to resolve and the build
// fails at link rather than driving something unintended.
//
// Only CH32V003 has been run on hardware. Every other table was read off a WCH
// reference manual, cross-checked where a second source exists -- see README.
// ---------------------------------------------------------------------------

#if defined(CH32V003)
	// CH32V003RM, agreeing with ch32fun's own examples/tim2_pwm_remap.
	// Hardware-verified.
	//
	//   remap  CH1   CH2   CH3   CH4
	//   0      PD4   PD3   PC0   PD7 (*)
	//   1      PC5   PC2   PD2   PC1
	//   2      PC1   PD3   PC0   PD7 (*)
	//   3      PC1   PC7   PD6   PD5
	//
	//   (*) PD7 is also NRST. Using CH4 in groups 0/2 requires disabling the
	//       reset function via the option bytes (minichlink -d).
	//
	// NB: the comments on AFIO_PCFR1_TIM2_REMAP_* in ch32v003hw.h are wrong for
	// two entries -- they say CH4/PD4 for group 0 and CH3/PD4 for group 1. The
	// table above follows the datasheet and the example, which agree.
	#if SERVO_TIM2_REMAP == 0
		#define SERVO_TIM2_CH1_PIN PD4
		#define SERVO_TIM2_CH2_PIN PD3
		#define SERVO_TIM2_CH3_PIN PC0
		#define SERVO_TIM2_CH4_PIN PD7
	#elif SERVO_TIM2_REMAP == 1
		#define SERVO_TIM2_CH1_PIN PC5
		#define SERVO_TIM2_CH2_PIN PC2
		#define SERVO_TIM2_CH3_PIN PD2
		#define SERVO_TIM2_CH4_PIN PC1
	#elif SERVO_TIM2_REMAP == 2
		#define SERVO_TIM2_CH1_PIN PC1
		#define SERVO_TIM2_CH2_PIN PD3
		#define SERVO_TIM2_CH3_PIN PC0
		#define SERVO_TIM2_CH4_PIN PD7
	#elif SERVO_TIM2_REMAP == 3
		#define SERVO_TIM2_CH1_PIN PC1
		#define SERVO_TIM2_CH2_PIN PC7
		#define SERVO_TIM2_CH3_PIN PD6
		#define SERVO_TIM2_CH4_PIN PD5
	#endif

#elif defined(CH32V10x) || defined(CH32V20x) || defined(CH32V30x) \
   || defined(CH32L103)
	// One table for four families: CH32xRM table 10-11 (V10x), CH32FV2x_V3xRM
	// table 10-16 (V20x/V30x) and CH32L103RM section 10 all describe the same
	// four groups with the same pins, and agree with the
	// AFIO_PCFR1_TIM2_REMAP_* pin comments in ch32v10xhw.h, ch32v20xhw.h and
	// ch32v30xhw.h. CH32L103DS0's alternate-function table corroborates L103
	// independently (its suffixes are remap group numbers: PA15 carries
	// TIM2_CH1_ETR_1 and _3, PA0 the unsuffixed default plus _2).
	//
	//   remap  CH1    CH2   CH3    CH4
	//   0      PA0    PA1   PA2    PA3
	//   1      PA15   PB3   PA2    PA3
	//   2      PA0    PA1   PB10   PB11
	//   3      PA15   PB3   PB10   PB11
	//
	// No pin here carries a secondary function, but check this yourself before
	// believing it, because ch32fun's headers say otherwise. They define
	// AFIO_PCFR1_SWJ_CFG with ST's semantics ("Full SWJ (JTAG-DP + SW-DP):
	// Reset State"), which on an ST part would make PA15 JTDI and PB3 JTDO and
	// put both under the debug port at reset. That macro is an ST leftover:
	// CH32FV2x_V3xRM gives PCFR1[26:24] as SW_CFG[2:0], "0xx: SWD enabled
	// (SDI)", and CH32xRM as SWCFG[2:0], "after reset, it is always used as the
	// SWD port". Neither manual mentions JTAG at all. Debug is two-wire on
	// PA13/PA14 -- CH32L103DS0's pin table lists those as SWDIO/SWCLK after
	// reset and PA15 as plain PA15 -- so PA15 and PB3 are ordinary GPIO here.
	//
	// CH32L103 stops at group 3 because its remap field is {TIM2_RM_H,
	// TIM2_RM} -- a third bit in AFIO->PCFR2[21] that we leave at reset. Groups
	// 4-7 (100/101/111; 110 is not defined) would need a second, L103-only
	// register write to reach, on a chip nobody here can test.
	#if SERVO_TIM2_REMAP == 0
		#define SERVO_TIM2_CH1_PIN PA0
		#define SERVO_TIM2_CH2_PIN PA1
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#elif SERVO_TIM2_REMAP == 1
		#define SERVO_TIM2_CH1_PIN PA15
		#define SERVO_TIM2_CH2_PIN PB3
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#elif SERVO_TIM2_REMAP == 2
		#define SERVO_TIM2_CH1_PIN PA0
		#define SERVO_TIM2_CH2_PIN PA1
		#define SERVO_TIM2_CH3_PIN PB10
		#define SERVO_TIM2_CH4_PIN PB11
	#elif SERVO_TIM2_REMAP == 3
		#define SERVO_TIM2_CH1_PIN PA15
		#define SERVO_TIM2_CH2_PIN PB3
		#define SERVO_TIM2_CH3_PIN PB10
		#define SERVO_TIM2_CH4_PIN PB11
	#endif

#elif defined(CH32X03x)
	// CH32X035RM section 8, corroborated by CH32X035DS0's pin table and, for
	// group 0, by ch32fun's own examples_x035/tim2_pwm and tim2_single_pulse.
	//
	//   remap  CH1     CH2    CH3    CH4
	//   0      PA0     PA1    PA2    PA3
	//   1      PB21*   PB15   PA2    PA3
	//   2      PA0     PA1    PB3    PB4
	//   3      PB21*   PB15   PB3    PB4
	//   4      PB16*   PB17*  PB18*  PB19*
	//   5      PC19*   PA12   PA13   PC0
	//   6,7    PC19*   PC14   PC15   PC0
	//
	//   (*) Unreachable, and so omitted below. X035's GPIO ports are 24 bits
	//       wide -- CH32X035RM table 8-12 gives each a CFGXR at offset 0x1C for
	//       pins 16-23 and a BSXR at 0x20 to drive them -- but ch32fun's pin
	//       constants stop at 15 per port and its GpioOf() is
	//       GPIOA_BASE + 0x400 * (pin >> 4), where pin 16 is already the next
	//       port. Group 4 loses every channel and defines nothing at all.
	//
	// No pin here carries a secondary function. RST is on PA21, PC3 or PB7
	// depending on package, PC17 is the BOOT detection pin, and SWD is on
	// PC18/PC19 -- PC19 being a CH1 we cannot name anyway.
	#if SERVO_TIM2_REMAP == 0
		#define SERVO_TIM2_CH1_PIN PA0
		#define SERVO_TIM2_CH2_PIN PA1
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#elif SERVO_TIM2_REMAP == 1
		#define SERVO_TIM2_CH2_PIN PB15
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#elif SERVO_TIM2_REMAP == 2
		#define SERVO_TIM2_CH1_PIN PA0
		#define SERVO_TIM2_CH2_PIN PA1
		#define SERVO_TIM2_CH3_PIN PB3
		#define SERVO_TIM2_CH4_PIN PB4
	#elif SERVO_TIM2_REMAP == 3
		#define SERVO_TIM2_CH2_PIN PB15
		#define SERVO_TIM2_CH3_PIN PB3
		#define SERVO_TIM2_CH4_PIN PB4
	#elif SERVO_TIM2_REMAP == 5
		#define SERVO_TIM2_CH2_PIN PA12
		#define SERVO_TIM2_CH3_PIN PA13
		#define SERVO_TIM2_CH4_PIN PC0
	#elif SERVO_TIM2_REMAP == 6 || SERVO_TIM2_REMAP == 7
		#define SERVO_TIM2_CH2_PIN PC14
		#define SERVO_TIM2_CH3_PIN PC15
		#define SERVO_TIM2_CH4_PIN PC0
	#endif

#elif defined(CH32V002) || defined(CH32V00x)
	// CH32V00XRM section 9. No second source: only the CH32V006 datasheet is on
	// hand, so the sibling parts' pinouts are unconfirmed. The manual does
	// document the one difference between them, at group 2.
	//
	//   remap  CH1   CH2      CH3   CH4
	//   0      PD4   PD3      PC0   PD7 (*)
	//   1      PC1   PD3      PC0   PD7 (*)
	//   2      PC5   PC2 (+)  PD2   PC1
	//   3      PC1   PC7      PD6   PD5
	//   4      PC0   PC1      PC3   PB6
	//   5      PA0   PA1 (#)  PA2 (#) PA3
	//   6      PB1   PA1 (#)  PA2 (#) PA3
	//   7      PD3   PD4      PA2 (#) PA3
	//
	//   (*) PD7 is also RST, as on CH32V003 -- CH4 in groups 0 and 1 needs the
	//       reset function disabled in the option bytes.
	//   (+) CH32V007 and CH32M007 put CH2 on PB3 here instead, and PB3 is
	//       SWCLK for the 2-wire debug interface. The manual calls this split
	//       out explicitly; it is the only per-part difference in this table.
	//   (#) PA1 and PA2 are the XI/XO crystal pins, gated by
	//       AFIO_PCFR1.PA1PA2_RM. Usable as GPIO only when no external crystal
	//       is fitted.
	#if SERVO_TIM2_REMAP == 0
		#define SERVO_TIM2_CH1_PIN PD4
		#define SERVO_TIM2_CH2_PIN PD3
		#define SERVO_TIM2_CH3_PIN PC0
		#define SERVO_TIM2_CH4_PIN PD7
	#elif SERVO_TIM2_REMAP == 1
		#define SERVO_TIM2_CH1_PIN PC1
		#define SERVO_TIM2_CH2_PIN PD3
		#define SERVO_TIM2_CH3_PIN PC0
		#define SERVO_TIM2_CH4_PIN PD7
	#elif SERVO_TIM2_REMAP == 2
		#define SERVO_TIM2_CH1_PIN PC5
		#if defined(CH32V007) || defined(CH32M007)
			#define SERVO_TIM2_CH2_PIN PB3
		#else
			#define SERVO_TIM2_CH2_PIN PC2
		#endif
		#define SERVO_TIM2_CH3_PIN PD2
		#define SERVO_TIM2_CH4_PIN PC1
	#elif SERVO_TIM2_REMAP == 3
		#define SERVO_TIM2_CH1_PIN PC1
		#define SERVO_TIM2_CH2_PIN PC7
		#define SERVO_TIM2_CH3_PIN PD6
		#define SERVO_TIM2_CH4_PIN PD5
	#elif SERVO_TIM2_REMAP == 4
		#define SERVO_TIM2_CH1_PIN PC0
		#define SERVO_TIM2_CH2_PIN PC1
		#define SERVO_TIM2_CH3_PIN PC3
		#define SERVO_TIM2_CH4_PIN PB6
	#elif SERVO_TIM2_REMAP == 5
		#define SERVO_TIM2_CH1_PIN PA0
		#define SERVO_TIM2_CH2_PIN PA1
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#elif SERVO_TIM2_REMAP == 6
		#define SERVO_TIM2_CH1_PIN PB1
		#define SERVO_TIM2_CH2_PIN PA1
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#elif SERVO_TIM2_REMAP == 7
		#define SERVO_TIM2_CH1_PIN PD3
		#define SERVO_TIM2_CH2_PIN PD4
		#define SERVO_TIM2_CH3_PIN PA2
		#define SERVO_TIM2_CH4_PIN PA3
	#endif
#endif

// One check, not one per family: if the target got a remap descriptor above but
// the configured group produced no pin at all, that group has no table on this
// family -- it is reserved, or every channel lands on a pin ch32fun cannot name.
// Without this the build still fails, but on a bare "SERVO_TIM2_CH1_PIN was not
// declared", which says nothing about why.
#if defined(SERVO_TIM2_REMAP_SHIFT) && !defined(SERVO_TIM2_CH1_PIN) \
 && !defined(SERVO_TIM2_CH2_PIN) && !defined(SERVO_TIM2_CH3_PIN) \
 && !defined(SERVO_TIM2_CH4_PIN)
	#error "Servo: SERVO_TIM2_REMAP selects a remap group that provides no \
usable TIM2 channel on this target -- either the group is reserved on this \
family, or every one of its channels lands on a pin ch32fun cannot address. \
See the channel-to-pin tables in the portability block above, and the README's \
TIM2 pin table, for the groups this family does provide."
#endif

#endif // !SERVO_USE_SYSTICK

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

// SERVO_TICKS_PER_US (= ch32fun's DELAY_US_TIME) is FUNCONF_SYSTEM_CORE_CLOCK
// divided by 8,000,000 by default, or by 1,000,000 under
// FUNCONF_SYSTICK_USE_HCLK=1 (forced to the /8,000,000 form on CH32V10x
// regardless of that flag). Plain integer division: below the resulting
// floor it truncates to 0, and every pulse width and frame length in this
// backend would collapse to a zero-tick compare instead of just losing
// precision. Every HSI/HSE value and PLL multiplier ch32fun documents a
// default for lands well above this floor -- it only bites with a
// deliberately low core clock (PLL disabled plus a low-frequency source).
static_assert(SERVO_TICKS_PER_US >= 1,
	"Servo: the effective SysTick rate is below 1 MHz, so SERVO_TICKS_PER_US "
	"truncates to zero and this backend cannot generate pulses at all. Needs "
	"a core clock of at least 8 MHz with the default SysTick/HCLK-8 divider "
	"(FUNCONF_SYSTICK_USE_HCLK unset or 0), or at least 1 MHz with "
	"FUNCONF_SYSTICK_USE_HCLK=1. Raise FUNCONF_PLL_MULTIPLIER or switch to a "
	"faster HSI/HSE source.");
#else
static_assert(SERVO_MAX_SERVOS <= SERVO_TIM2_MAX_CH,
	"Servo: the TIM2 backend has one channel per servo, so SERVO_MAX_SERVOS "
	"cannot exceed 4. Use SERVO_USE_SYSTICK for more.");

// SERVO_TIM2_PSC divides FUNCONF_SYSTEM_CORE_CLOCK down to a 1 MHz tick so
// that a CH*CVR write is the pulse width in microseconds with no scaling. A
// core clock that isn't an exact multiple of 1,000,000 Hz would silently
// scale every commanded pulse width by the same fixed factor instead of
// failing loudly. Every HSI/HSE value and PLL multiplier ch32fun documents a
// default for is a whole number of megahertz -- this only breaks with a
// custom, non-whole-megahertz crystal value.
static_assert(FUNCONF_SYSTEM_CORE_CLOCK % 1000000 == 0,
	"Servo: FUNCONF_SYSTEM_CORE_CLOCK must be an exact multiple of "
	"1,000,000 Hz for the TIM2 backend's 1 MHz tick. A non-exact core clock "
	"(typically from a custom, non-whole-megahertz HSE crystal value) would "
	"scale every commanded pulse width by a fixed, silent factor.");
#endif

// ---------------------------------------------------------------------------
// Compile-time pin resolution (TIM2 backend)
// ---------------------------------------------------------------------------

#if !SERVO_USE_SYSTICK
// Declared, never defined. A valid compile-time-constant pin folds the call
// away and costs nothing. Anything else -- an invalid pin, or a pin that is not
// a constant -- leaves the call and fails at link with the reason in the name.
extern "C" void servo_error__pin_is_not_a_tim2_channel_for_configured_remap(void);

// The channel-to-pin mapping itself lives in the portability block above, as
// four SERVO_TIM2_CHn_PIN defines for the configured family and remap group.
// This body is family-agnostic: it only compares. A channel a family does not
// route in the configured group simply defines no macro, so its comparison is
// not compiled and any pin naming it falls through to the link error -- which
// is also what catches a remap group that has no table at all.
__attribute__((always_inline))
static inline uint8_t servo_tim2_channel(uint8_t pin)
{
#ifdef SERVO_TIM2_CH1_PIN
	if (pin == SERVO_TIM2_CH1_PIN) return 1;
#endif
#ifdef SERVO_TIM2_CH2_PIN
	if (pin == SERVO_TIM2_CH2_PIN) return 2;
#endif
#ifdef SERVO_TIM2_CH3_PIN
	if (pin == SERVO_TIM2_CH3_PIN) return 3;
#endif
#ifdef SERVO_TIM2_CH4_PIN
	if (pin == SERVO_TIM2_CH4_PIN) return 4;
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
