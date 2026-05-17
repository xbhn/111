/**
 * exceptions.h
 * PandOSsh Phase 2 – Exception handler interface.
 */

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include "pandos_types.h"

/**
 * todStart – TOD clock value sampled at the moment the Nucleus last gained
 * control (exception entry).  Used by NSYS6 and blocking syscalls to
 * compute the time charged to the Current Process for the current quantum.
 *
 * Defined in exceptions.c; also used by interrupts.c.
 */
extern cpu_t todStart;

/** Placeholder TLB-Refill handler (Section 4). */
void uTLB_RefillHandler(void);

/** Top-level exception dispatcher (Section 5). */
void exceptionHandler(void);

#endif /* EXCEPTIONS_H */
