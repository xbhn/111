/**
 * initial.c
 * PandOSsh Phase 2 - Nucleus Initialization (Level 3)
 *
 * This module implements main(), which performs one-time Nucleus setup:
 *   - Declares and initializes all Level 3 global variables
 *   - Populates the Processor 0 Pass Up Vector (TLB-Refill handler +
 *     general exception handler)
 *   - Initializes the Level 2 data structures (PCBs and ASL)
 *   - Loads the system-wide Interval Timer with 100 ms (Pseudo-clock)
 *   - Instantiates the first process (test) and places it on the Ready Queue
 *   - Calls the Scheduler (never returns)
 *
 * Authors: Luca Bassi, Luca Orlandello
 */

#include "pandos_const.h"
#include "pandos_types.h"
#include "pcb.h"
#include "asl.h"
#include "scheduler.h"
#include "exceptions.h"
#include "interrupts.h"
#include <uriscv/liburiscv.h>
#include <uriscv/arch.h>

/* -----------------------------------------------------------------------
 * Nucleus global variables (exported to other modules via initial.h)
 * ----------------------------------------------------------------------- */

/* Number of processes that have been started but not yet terminated. */
int processCount;

/*
 * Number of started-but-not-terminated processes currently blocked
 * (waiting) due to an I/O or timer request.
 */
int softBlockCount;

/* Queue of PCBs in the "ready" state. */
struct list_head readyQueue;

/* Pointer to the PCB currently in the "running" state (NULL if none). */
pcb_t *currentProcess;

/*
 * Device semaphores: one per (sub)device.
 * Devices on interrupt lines 3-7, up to 8 devices each = 40 semaphores.
 * Terminal devices are split into two sub-devices (transmit + receive),
 * adding 8 extra semaphores for a total of MAXDEVICECNT (48 – or as
 * defined in pandos_const.h).
 * An additional semaphore at index MAXDEVICECNT is the Pseudo-clock semaphore.
 */
int deviceSemaphores[MAXDEVICECNT + 1]; /* +1 for the Pseudo-clock */

/* -----------------------------------------------------------------------
 * Forward declaration of the test process provided by the test harness.
 * ----------------------------------------------------------------------- */
extern void test();

/* -----------------------------------------------------------------------
 * main() - Nucleus entry point
 * ----------------------------------------------------------------------- */
int main(void)
{
    /* ------------------------------------------------------------------
     * Step 1: Populate the Processor 0 Pass Up Vector.
     *
     * The Pass Up Vector lives in the BIOS Data Page at 0x0FFF.F900 for
     * Processor 0.  We fill in:
     *   [0] TLB-Refill handler address
     *   [1] SP for the TLB-Refill handler  (top of Nucleus stack page)
     *   [2] General exception handler address
     *   [3] SP for the general exception handler (same stack page)
     * ------------------------------------------------------------------ */
    passupvector_t *passUpVec = (passupvector_t *) PASSUPVECTOR;

    passUpVec->tlb_refll_handler = (memaddr) uTLB_RefillHandler;
    passUpVec->tlb_refll_stackPtr = KERNELSTACK;
    passUpVec->exception_handler  = (memaddr) exceptionHandler;
    passUpVec->exception_stackPtr = KERNELSTACK;

    /* ------------------------------------------------------------------
     * Step 2: Initialize the Level 2 (Phase 1) data structures.
     * ------------------------------------------------------------------ */
    initPcbs();
    initASL();

    /* ------------------------------------------------------------------
     * Step 3: Initialize all Level 3 global variables.
     * ------------------------------------------------------------------ */
    processCount   = 0;
    softBlockCount = 0;
    mkEmptyProcQ(&readyQueue);
    currentProcess = NULL;

    /* All device semaphores (including the Pseudo-clock) start at 0,
     * making them pure synchronization semaphores (not mutual-exclusion). */
    for (int i = 0; i <= MAXDEVICECNT; i++)
        deviceSemaphores[i] = 0;

    /* ------------------------------------------------------------------
     * Step 4: Load the system-wide Interval Timer with 100 milliseconds.
     * This drives the Pseudo-clock tick [Section 7.3].
     * ------------------------------------------------------------------ */
    LDIT(PSECOND);

    /* ------------------------------------------------------------------
     * Step 5: Instantiate the first process.
     *
     * Allocate a PCB, configure its processor state, and enqueue it on
     * the Ready Queue.
     * ------------------------------------------------------------------ */
    pcb_t *firstProc = allocPcb();

    if (firstProc == NULL)
        PANIC(); /* Should never happen at startup. */

    /* Processor state initialisation:
     *   - Interrupts enabled  (mie = MIE_ALL)
     *   - Kernel-mode + previous-interrupt-enable  (MSTATUS_MPIE_MASK | MSTATUS_MPP_M)
     *   - SP = RAMTOP (last RAM frame used as stack)
     *   - PC = address of test                                          */
    firstProc->p_s.mie    = MIE_ALL;
    firstProc->p_s.status = MSTATUS_MPIE_MASK | MSTATUS_MPP_M;
    firstProc->p_s.reg_sp = RAMTOP;
    firstProc->p_s.pc_epc = (memaddr) test;

    /* Process tree fields: no parent, no children yet. */
    firstProc->p_parent  = NULL;
    firstProc->p_child.next  = NULL;
    firstProc->p_child.prev  = NULL;
    firstProc->p_sib.next    = NULL;
    firstProc->p_sib.prev    = NULL;

    /* No CPU time accumulated yet. */
    firstProc->p_time = 0;

    /* Not blocked on any semaphore. */
    firstProc->p_semAdd = NULL;

    /* No Support Structure at this level. */
    firstProc->p_supportStruct = NULL;

    /* Assign a unique PID.  Process IDs start at 1; we use a simple
     * static counter shared with the SYSCALL handler (see exceptions.c).
     * Here we directly assign 1 to the very first process. */
    firstProc->p_pid = 1;

    /* Enqueue on the Ready Queue and update the process count. */
    insertProcQ(&readyQueue, firstProc);
    processCount++;

    /* ------------------------------------------------------------------
     * Step 6: Hand control over to the Scheduler.
     * This call never returns.
     * ------------------------------------------------------------------ */
    schedule();

    /* Unreachable – the Scheduler either dispatches a process or HALTs. */
    return 0;
}
