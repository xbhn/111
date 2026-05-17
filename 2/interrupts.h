/**
 * interrupts.h
 * PandOSsh Phase 2 – Interrupt handler interface.
 */

#ifndef INTERRUPTS_H
#define INTERRUPTS_H

/** Top-level interrupt dispatcher – called from exceptionHandler(). */
void interruptHandler(void);

/** PLT interrupt handler (Section 7.2). */
void handlePLTInterrupt(void);

/** Interval Timer / Pseudo-clock interrupt handler (Section 7.3). */
void handleTimerInterrupt(void);

#endif /* INTERRUPTS_H */
