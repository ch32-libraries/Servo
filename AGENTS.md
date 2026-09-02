# AGENTS.md

## Chip-family support roadmap

Servo currently builds only for CH32V003. Widening it to the rest of the CH32
family that has a usable TIM2 (same chip list Ticker already documents) is
tracked as a 4-step roadmap. Full detail and rationale:
`openspec/changes/expand-chip-family-support/proposal.md`.

Only CH32V003 hardware is on hand — everything past Step 0 is ported from WCH
datasheets and reference manuals, not verified on real silicon, and must be
disclosed as such wherever it's documented (mirrors Ticker's own README
caveat for the chips it can't test either).

1. **Step 0** (in progress): stop hard-erroring the SysTick backend on
   non-CH32V003 targets. It only needs primitives already identical across
   every ch32fun family with TIM2, so it can build broadly with no new pin
   tables. The TIM2 backend keeps failing to build on anything but CH32V003.
2. **Step 1**: TIM2 backend for CH32X03x — the only other family with an
   in-repo `tim2_pwm_remap` example to check the pin table against.
3. **Step 2**: TIM2 backend for CH32V10x, CH32V20x, CH32V30x, CH32L103 as one
   shared pin table (same 2-bit remap field, PA/PB pins). L103 has no
   datasheet comments in its ch32fun header, so it's the least confident of
   the four.
4. **Step 3**: TIM2 backend for CH32V00x (CH32V002, CH32V004-007). Different
   remap field shape (3-bit) plus renamed register bit macros
   (`TIM2_CTLR1_CEN` etc.), and no TIM2 PWM example anywhere in ch32fun to
   check against. Highest effort, lowest confidence.

Do each step as its own OpenSpec change rather than one large port.
