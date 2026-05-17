/**
 * scheduler.c
 * PandOSsh Phase 2 – Nucleus Scheduler (Level 3, Section 3)
 *
 * Implements a simple preemptive round-robin scheduler with a 5 ms
 * time slice (TIMESLICE).  The Processor Local Timer (PLT) is used to
 * enforce the time quantum; the system-wide Interval Timer is left
 * exclusively for Pseudo-clock ticks.
 *
 * Behaviour:
 *  1. Ready Queue non-empty  → dequeue head PCB, load PLT, LDST.
 *  2. Ready Queue empty, processCount == 0 → HALT (normal shutdown).
 *  3. Ready Queue empty, softBlockCount >  0 → Wait State (WAIT).
 *  4. Ready Queue empty, softBlockCount == 0 → PANIC (deadlock).
 *
 * Authors: Luca Bassi, Luca Orlandello
 */

#include "scheduler.h"
#include "initial.h"
#include "pandos_const.h"
#include "pandos_types.h"
#include "pcb.h"
#include <uriscv/liburiscv.h>
#include <uriscv/arch.h>

/**
 * schedule() – main scheduler entry point.
 *
 * Called from:
 *   - main()               (initial dispatch after boot)
 *   - PLT interrupt handler (quantum expiry)
 *   - SYSCALL handlers      (blocking syscalls, process termination)
 *   - Interval Timer handler (after waking Pseudo-clock waiters, no
 *                             Current Process)
 *   - Non-timer interrupt handler (no Current Process path)
 */
void schedule(void)
{
    /* ------------------------------------------------------------------
     * Case 1: There is at least one process ready to run.
     * ------------------------------------------------------------------ */
    if (!emptyProcQ(&readyQueue))
    {
        /* Remove the PCB at the head of the Ready Queue. */
        currentProcess = removeProcQ(&readyQueue);

        /* Load the PLT with the time slice value (5 ms).
         * When it reaches zero it will fire a PLT interrupt, pre-empting
         * the process and causing re-entry into the Nucleus. */
        setTIMER(TIMESLICE);

        /* Atomically restore the saved processor state of this process.
         * This transfers control directly to the process; schedule()
         * does not return. */
        LDST(&(currentProcess->p_s));
    }

    /* ------------------------------------------------------------------
     * Case 2: No more processes exist at all → clean shutdown.
     * ------------------------------------------------------------------ */
    if (processCount == 0)
        HALT();

    /* ------------------------------------------------------------------
     * Case 3: Processes exist but all are blocked waiting for I/O or a
     * timer event.  Enter a Wait State until the next interrupt.
     *
     * Before executing WAIT we must:
     *   a) Ensure all external-device interrupts are enabled.
     *   b) Disable the PLT interrupt (no process is running, so there is
     *      no quantum to enforce – and we must not receive a spurious PLT
     *      interrupt while waiting).
     *   c) Enable the global machine interrupt enable bit (MIE in mstatus).
     * ------------------------------------------------------------------ */
    if (softBlockCount > 0)
    {
        /* currentProcess is NULL because the Ready Queue is empty; no
         * process is "running" during a Wait State. */
        currentProcess = NULL;

        /* Enable all interrupts except the PLT (MIE_MTIE_MASK). */
        setMIE(MIE_ALL & ~MIE_MTIE_MASK);

        /* Enable the global interrupt-enable bit in mstatus. */
        unsigned int status = getSTATUS();
        status |= MSTATUS_MIE_MASK;
        setSTATUS(status);

        /* Idle the processor; the first non-PLT interrupt will wake it. */
        WAIT();

        /* Control returns here after an interrupt has been handled by
         * the interrupt exception handler.  The scheduler will be called
         * again from there, so we simply loop back (or the process that
         * was unblocked will be on the Ready Queue). */
        return;
    }

    /* ------------------------------------------------------------------
     * Case 4: processCount > 0 but softBlockCount == 0 → deadlock.
     * ------------------------------------------------------------------ */
    PANIC();
}
