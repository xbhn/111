/**
 * initial.h
 * PandOSsh Phase 2 – Nucleus global variable declarations.
 *
 * Every module that needs to read or write the Nucleus-wide state
 * (processCount, softBlockCount, readyQueue, currentProcess,
 * deviceSemaphores) must include this header.
 */

#ifndef INITIAL_H
#define INITIAL_H

#include "pandos_types.h"
#include "pcb.h"

/* -----------------------------------------------------------------------
 * Nucleus-wide global variables (defined in initial.c)
 * ----------------------------------------------------------------------- */

/** Total number of started-but-not-yet-terminated processes. */
extern int processCount;

/**
 * Number of started-but-not-terminated processes currently in the
 * "blocked" state waiting for an I/O completion or timer event.
 */
extern int softBlockCount;

/** Head sentinel of the Ready Queue (PCBs in the "ready" state). */
extern struct list_head readyQueue;

/**
 * Pointer to the PCB currently executing ("running" state).
 * NULL when no process is running (e.g. during the Wait State).
 */
extern pcb_t *currentProcess;

/**
 * Array of device semaphores.
 * Indices 0 .. MAXDEVICECNT-1  → one semaphore per (sub)device.
 * Index  MAXDEVICECNT           → Pseudo-clock semaphore.
 *
 * Layout (interrupt lines 3-7, 8 devices each, terminals split):
 *   [0  .. 7 ] : disk devices       (line 3, devices 0-7)
 *   [8  .. 15] : flash devices      (line 4, devices 0-7)
 *   [16 .. 23] : network devices    (line 5, devices 0-7)
 *   [24 .. 31] : printer devices    (line 6, devices 0-7)
 *   [32 .. 39] : terminal TX        (line 7, devices 0-7, transmit)
 *   [40 .. 47] : terminal RX        (line 7, devices 0-7, receive)
 *   [48]       : Pseudo-clock semaphore
 */
extern int deviceSemaphores[MAXDEVICECNT + 1];

#endif /* INITIAL_H */
