/**
 * exceptions.c
 * PandOSsh Phase 2 – Exception Handling (Level 3, Sections 4-6 & 8)
 *
 * Modules:
 *   uTLB_RefillHandler  – placeholder TLB-Refill handler (Section 4).
 *   exceptionHandler    – top-level exception dispatcher (Section 5).
 *   syscallHandler      – handles SYSCALL exceptions (Section 6).
 *   programTrapHandler  – handles Program Trap exceptions (Section 8.2).
 *   tlbExceptionHandler – handles TLB exceptions (Section 8.3).
 *
 * SYSCALL services implemented (kernel-mode, negative a0):
 *   NSYS1  CreateProcess      (-1)
 *   NSYS2  TerminateProcess   (-2)
 *   NSYS3  Passeren (P)       (-3)
 *   NSYS4  Verhogen (V)       (-4)
 *   NSYS5  DoIO               (-5)
 *   NSYS6  GetCPUTime         (-6)
 *   NSYS7  WaitForClock       (-7)
 *   NSYS8  GetSupportData     (-8)
 *   NSYS9  GetProcessID       (-9)
 *   NSYS10 Yield              (-10)
 *
 * Authors: Luca Bassi, Luca Orlandello
 */

#include "exceptions.h"
#include "initial.h"
#include "scheduler.h"
#include "interrupts.h"
#include "pandos_const.h"
#include "pandos_types.h"
#include "pcb.h"
#include "asl.h"
#include <uriscv/liburiscv.h>
#include <uriscv/arch.h>

/* -----------------------------------------------------------------------
 * Static helper prototypes
 * ----------------------------------------------------------------------- */
static void         syscallHandler(void);
static void         programTrapHandler(void);
static void         tlbExceptionHandler(void);
static void         passUpOrDie(int exceptionType);
static void         terminateProcess(pcb_t *proc);
static int          semaphoreIndex(int *semAddr);
static cpu_t        elapsedTime(void);

/* PID counter – incremented each time a new process is created. */
static unsigned int nextPid = 2; /* 1 is assigned to the first process in initial.c */

/* TOD clock snapshot taken when the current process was dispatched (or
 * when the Nucleus last gained control on its behalf).                  */
cpu_t todStart;

/* -----------------------------------------------------------------------
 * Section 4 – TLB-Refill event handler placeholder
 * ----------------------------------------------------------------------- */

/**
 * uTLB_RefillHandler()
 *
 * Placeholder for Phase 2.  Maps every virtual address to physical
 * address 0 with a fixed TLB entry so that testing at Level 3 can
 * proceed without a full virtual-memory implementation.
 * The Support Level (Phase 3) will overwrite this function.
 */
void uTLB_RefillHandler(void)
{
    setENTRYHI(0x80000000);
    setENTRYLO(0x00000000);
    TLBWR();
    LDST((state_t *) BIOSDATAPAGE);
}

/* -----------------------------------------------------------------------
 * Section 5 – Top-level exception dispatcher
 * ----------------------------------------------------------------------- */

/**
 * exceptionHandler()
 *
 * Called by the BIOS after every exception (other than TLB-Refill).
 * The saved processor state is at BIOSDATAPAGE (Processor 0).
 *
 * Reads the Cause register from the saved state and dispatches to the
 * appropriate handler.
 */
void exceptionHandler(void)
{
    /* Snapshot the TOD clock the moment we enter the Nucleus so that
     * we can correctly charge CPU time to the running process later. */
    STCK(todStart);

    state_t *savedState = (state_t *) BIOSDATAPAGE;
    unsigned int cause  = savedState->cause;

    /* An interrupt is signalled by the MSB of the Cause register. */
    if (CAUSE_IS_INT(cause))
    {
        interruptHandler(); /* Section 7 */
        return;
    }

    /* Extract the exception code (lower bits of Cause). */
    unsigned int excCode = cause & CAUSE_EXCCODE_MASK;

    if (excCode == SYSEXCPCODE || excCode == 11)
    {
        /* SYSCALL exception (codes 8 and 11). */
        syscallHandler();
    }
    else if (excCode >= TLBEXCPCODE_LOW && excCode <= TLBEXCPCODE_HIGH)
    {
        /* TLB exceptions (codes 24-28). */
        tlbExceptionHandler();
    }
    else
    {
        /* Program Trap: codes 0-7, 9, 10, 12-23. */
        programTrapHandler();
    }
}

/* -----------------------------------------------------------------------
 * Section 6 – SYSCALL exception handler
 * ----------------------------------------------------------------------- */

/**
 * syscallHandler()
 *
 * Dispatches SYSCALL requests.  Negative a0 values are privileged Nucleus
 * services (kernel-mode only).  Positive or out-of-range values, and any
 * privileged call made from user-mode, are forwarded to passUpOrDie().
 */
static void syscallHandler(void)
{
    state_t *savedState = (state_t *) BIOSDATAPAGE;

    int sysNum  = (int) savedState->reg_a0;   /* Which syscall?      */
    int arg1    = (int) savedState->reg_a1;   /* First argument      */
    int arg2    = (int) savedState->reg_a2;   /* Second argument     */
    int arg3    = (int) savedState->reg_a3;   /* Third argument      */

    /* ---------------------------------------------------------------
     * User-mode check for privileged (negative) syscalls.
     * If the process was executing in user-mode (MPP field == 0) and
     * it attempted a privileged service, simulate a Program Trap.
     * --------------------------------------------------------------- */
    if (sysNum < 0)
    {
        int inKernelMode = (savedState->status & MSTATUS_MPP_MASK) != 0;
        if (!inKernelMode)
        {
            /* Simulate a Privileged Instruction trap. */
            savedState->cause = (savedState->cause & ~CAUSE_EXCCODE_MASK)
                                | PRIVINSTR;
            programTrapHandler();
            return;
        }
    }

    /* ---------------------------------------------------------------
     * Advance the PC past the SYSCALL instruction so that when control
     * eventually returns to the process it does not loop back into the
     * same SYSCALL.
     * --------------------------------------------------------------- */
    savedState->pc_epc += WORDLEN; /* WORDLEN = 4 bytes on µRISC-V */

    /* ---------------------------------------------------------------
     * Dispatch on the syscall number.
     * --------------------------------------------------------------- */
    switch (sysNum)
    {
        /* ---- NSYS1: CreateProcess ---------------------------------- */
        case CREATEPROCESS:
        {
            state_t    *childState   = (state_t *)    arg1;
            support_t  *childSupport = (support_t *)  arg3;

            pcb_t *child = allocPcb();
            if (child == NULL)
            {
                /* No free PCBs – return error code -1. */
                savedState->reg_a0 = (unsigned int) -1;
                break;
            }

            /* Initialise child PCB fields. */
            child->p_s            = *childState;
            child->p_supportStruct = childSupport;
            child->p_pid          = nextPid++;
            child->p_time         = 0;
            child->p_semAdd       = NULL;

            /* Attach child to the process tree under the current process. */
            insertChild(currentProcess, child);

            /* Place child on the Ready Queue. */
            insertProcQ(&readyQueue, child);
            processCount++;

            /* Return the new process's PID to the caller. */
            savedState->reg_a0 = child->p_pid;
            break;
        }

        /* ---- NSYS2: TerminateProcess ------------------------------- */
        case TERMINATEPROCESS:
        {
            int targetPid = arg1;
            if (targetPid == 0)
            {
                /* Terminate the calling process (and all its progeny). */
                terminateProcess(currentProcess);
            }
            else
            {
                /*
                 * Find the process with the given PID and terminate it.
                 * We walk the Ready Queue and the ASL for a match.
                 * (A full search across all possible locations.)
                 */
                /* Search ready queue */
                pcb_t *target = NULL;
                struct list_head *iter;
                list_for_each(iter, &readyQueue)
                {
                    pcb_t *p = container_of(iter, pcb_t, p_list);
                    if ((int) p->p_pid == targetPid)
                    {
                        target = p;
                        break;
                    }
                }
                /* If not in ready queue, check if it's current process */
                if (target == NULL && currentProcess != NULL
                    && (int) currentProcess->p_pid == targetPid)
                    target = currentProcess;

                if (target != NULL)
                    terminateProcess(target);
            }
            currentProcess = NULL;
            schedule();
            return; /* schedule() never returns */
        }

        /* ---- NSYS3: Passeren (P) ----------------------------------- */
        case PASSEREN:
        {
            int *semAddr = (int *) arg1;
            (*semAddr)--;
            if (*semAddr < 0)
            {
                /* Block the current process on this semaphore. */
                /* Save current processor state into the PCB. */
                currentProcess->p_s = *savedState;

                /* Charge CPU time accumulated so far this quantum. */
                currentProcess->p_time += elapsedTime();

                if (insertBlocked(semAddr, currentProcess) != 0)
                    PANIC(); /* ASL full – should not happen. */

                softBlockCount++;
                currentProcess = NULL;
                schedule();
                return;
            }
            /* Semaphore was positive: just continue. */
            break;
        }

        /* ---- NSYS4: Verhogen (V) ----------------------------------- */
        case VERHOGEN:
        {
            int *semAddr = (int *) arg1;
            (*semAddr)++;
            if (*semAddr <= 0)
            {
                /* Unblock the first PCB waiting on this semaphore. */
                pcb_t *unblocked = removeBlocked(semAddr);
                if (unblocked != NULL)
                {
                    softBlockCount--;
                    insertProcQ(&readyQueue, unblocked);
                }
            }
            break;
        }

        /* ---- NSYS5: DoIO ------------------------------------------ */
        case DOIO:
        {
            int  *cmdAddr  = (int *) arg1;
            int   cmdValue = arg2;

            /* Find the semaphore for this device and perform P. */
            int semIdx = semaphoreIndex(cmdAddr);
            deviceSemaphores[semIdx]--;
            /* DoIO always blocks (synchronisation semaphore starts at 0). */

            /* Save state and charge time. */
            currentProcess->p_s = *savedState;
            currentProcess->p_time += elapsedTime();

            if (insertBlocked(&deviceSemaphores[semIdx], currentProcess) != 0)
                PANIC();

            softBlockCount++;

            /* Write the command to the device register. */
            *cmdAddr = cmdValue;

            currentProcess = NULL;
            schedule();
            return;
        }

        /* ---- NSYS6: GetCPUTime ------------------------------------ */
        case GETCPUTIME:
        {
            /* Return accumulated time + time used in the current quantum. */
            savedState->reg_a0 = currentProcess->p_time + elapsedTime();
            break;
        }

        /* ---- NSYS7: WaitForClock ---------------------------------- */
        case WAITCLOCK:
        {
            /* P on the Pseudo-clock semaphore – always blocks. */
            deviceSemaphores[MAXDEVICECNT]--;

            currentProcess->p_s = *savedState;
            currentProcess->p_time += elapsedTime();

            if (insertBlocked(&deviceSemaphores[MAXDEVICECNT], currentProcess) != 0)
                PANIC();

            softBlockCount++;
            currentProcess = NULL;
            schedule();
            return;
        }

        /* ---- NSYS8: GetSupportData -------------------------------- */
        case GETSUPPORTPTR:
        {
            savedState->reg_a0 = (unsigned int) currentProcess->p_supportStruct;
            break;
        }

        /* ---- NSYS9: GetProcessID ---------------------------------- */
        case GETPID:
        {
            if (arg1 == 0)
                savedState->reg_a0 = currentProcess->p_pid;
            else
            {
                /* Return parent PID; 0 if the process is the root. */
                if (currentProcess->p_parent != NULL)
                    savedState->reg_a0 = currentProcess->p_parent->p_pid;
                else
                    savedState->reg_a0 = 0;
            }
            break;
        }

        /* ---- NSYS10: Yield ---------------------------------------- */
        case YIELD:
        {
            /* Save current state into the PCB. */
            currentProcess->p_s = *savedState;
            currentProcess->p_time += elapsedTime();

            /* Return the process to the tail of the Ready Queue. */
            insertProcQ(&readyQueue, currentProcess);
            currentProcess = NULL;
            schedule();
            return;
        }

        /* ---- Unknown / out-of-range / positive syscall numbers ----- */
        default:
        {
            if (sysNum >= 1)
            {
                /* Pass up to Support Level or terminate. */
                passUpOrDie(GENERALEXCEPT);
            }
            else
            {
                /* Undefined negative syscall: Program Trap. */
                programTrapHandler();
            }
            return;
        }
    }

    /* ------------------------------------------------------------------
     * For non-blocking syscalls, return control to the process by
     * loading the (already PC-incremented) saved exception state.
     * ------------------------------------------------------------------ */
    LDST(savedState);
}

/* -----------------------------------------------------------------------
 * Section 8.2 – Program Trap exception handler
 * ----------------------------------------------------------------------- */

/**
 * programTrapHandler()
 *
 * Handles exception codes 0-7, 9, 10, 12-23 (Program Traps) as well as
 * the simulated Privileged Instruction trap for user-mode SYSCALL abuse.
 * Delegates to passUpOrDie() with GENERALEXCEPT.
 */
static void programTrapHandler(void)
{
    passUpOrDie(GENERALEXCEPT);
}

/* -----------------------------------------------------------------------
 * Section 8.3 – TLB exception handler
 * ----------------------------------------------------------------------- */

/**
 * tlbExceptionHandler()
 *
 * Handles TLB exception codes 24-28.
 * Delegates to passUpOrDie() with PGFAULTEXCEPT.
 */
static void tlbExceptionHandler(void)
{
    passUpOrDie(PGFAULTEXCEPT);
}

/* -----------------------------------------------------------------------
 * Section 8 – Pass Up or Die
 * ----------------------------------------------------------------------- */

/**
 * passUpOrDie()
 *
 * @param exceptionType  PGFAULTEXCEPT (0) or GENERALEXCEPT (1)
 *
 * If the Current Process has a non-NULL Support Structure, the exception
 * is "passed up" to the Support Level:
 *   1. Copy the saved exception state into sup_exceptState[exceptionType].
 *   2. LDCXT with the fields from sup_exceptContext[exceptionType].
 *
 * Otherwise ("die"): terminate the Current Process and all its progeny,
 * then call the Scheduler.
 */
static void passUpOrDie(int exceptionType)
{
    if (currentProcess->p_supportStruct == NULL)
    {
        /* Die: terminate the process subtree. */
        terminateProcess(currentProcess);
        currentProcess = NULL;
        schedule();
        return;
    }

    /* Pass up. */
    support_t *sup = currentProcess->p_supportStruct;

    /* Copy the saved exception state to the appropriate slot. */
    sup->sup_exceptState[exceptionType] = *((state_t *) BIOSDATAPAGE);

    /* Jump to the Support Level handler. */
    context_t *ctx = &sup->sup_exceptContext[exceptionType];
    LDCXT(ctx->c_stackPtr, ctx->c_status, ctx->c_pc);
}

/* -----------------------------------------------------------------------
 * Section 10 – Process termination (recursive subtree)
 * ----------------------------------------------------------------------- */

/**
 * terminateProcess()
 *
 * Recursively terminates @proc and all of its descendants (progeny).
 *
 * For each terminated PCB:
 *   - Remove from wherever it lives (Ready Queue, ASL, or current).
 *   - Adjust processCount and softBlockCount as needed.
 *   - Return the PCB to the free list via freePcb().
 */
static void terminateProcess(pcb_t *proc)
{
    if (proc == NULL)
        return;

    /* First, recursively terminate all children. */
    while (!emptyChild(proc))
    {
        pcb_t *child = removeChild(proc);
        terminateProcess(child);
    }

    /* Now handle the process itself. */

    /* Orphan it from its parent (if any). */
    if (proc->p_parent != NULL)
        outChild(proc);

    if (proc == currentProcess)
    {
        /* The running process – just free it; the caller sets
         * currentProcess = NULL and calls the Scheduler. */
        processCount--;
        freePcb(proc);
        return;
    }

    /* Is it on the Ready Queue? */
    pcb_t *removed = outProcQ(&readyQueue, proc);
    if (removed != NULL)
    {
        processCount--;
        freePcb(proc);
        return;
    }

    /* It must be blocked on a semaphore (ASL). */
    if (proc->p_semAdd != NULL)
    {
        pcb_t *unblocked = outBlocked(proc);
        if (unblocked != NULL)
        {
            /* Determine if this was a device/timer semaphore or a
             * user-level mutex.  Device semaphores lie in the
             * deviceSemaphores array; adjust the semaphore value and
             * the soft-block count accordingly. */
            int *sem = proc->p_semAdd;
            /* If the semaphore address is within the device array,
             * it is a device/timer semaphore. */
            int *devSemStart = &deviceSemaphores[0];
            int *devSemEnd   = &deviceSemaphores[MAXDEVICECNT];
            if (sem >= devSemStart && sem <= devSemEnd)
            {
                (*sem)++;           /* Undo the P that blocked this PCB. */
                softBlockCount--;
            }
            else
            {
                /* Non-device semaphore: undo the decrement. */
                (*sem)++;
            }
        }
    }

    processCount--;
    freePcb(proc);
}

/* -----------------------------------------------------------------------
 * Helper: compute the semaphore array index for a device command address
 * ----------------------------------------------------------------------- */

/**
 * semaphoreIndex()
 *
 * Given a pointer to a device's command register, return the index into
 * deviceSemaphores[] that corresponds to that (sub)device.
 *
 * Layout mirrors the interrupt line/device numbering described in
 * Section 7 / Table 1.
 *
 *   Lines 3-6 (disk, flash, net, printer): 8 devices each → indices 0-31.
 *   Line 7 (terminal TX): indices 32-39.
 *   Line 7 (terminal RX): indices 40-47.
 */
static int semaphoreIndex(int *cmdAddr)
{
    memaddr addr = (memaddr) cmdAddr;

    /* Base address of the device register area. */
    memaddr devBase = DEV_REG_START; /* 0x10000054 */

    /* Each device register block is 0x10 bytes; interrupt lines 3-7
     * occupy 5 banks of 8 devices each.                               */
    int offset      = (int)(addr - devBase);
    int lineOffset  = offset / (DEVPERINT * DEV_REG_SIZE); /* 0-4 */
    int devNo       = (offset % (DEVPERINT * DEV_REG_SIZE)) / DEV_REG_SIZE;

    int line = lineOffset + 3; /* Convert to interrupt line number (3-7) */

    if (line == IL_TERMINAL)
    {
        /* Terminal: distinguish TX (TRANSM_COMMAND) from RX (RECV_COMMAND).
         * TRANSM_COMMAND is at offset 0xC from the device base;
         * RECV_COMMAND   is at offset 0x4 from the device base.       */
        int subDevOffset = offset % DEV_REG_SIZE;
        if (subDevOffset == 0xC)
            return (DEVPERINT * (IL_TERMINAL - 3)) + devNo;          /* TX: 32-39 */
        else
            return (DEVPERINT * (IL_TERMINAL - 3)) + DEVPERINT + devNo; /* RX: 40-47 */
    }

    /* Non-terminal device. */
    return (lineOffset * DEVPERINT) + devNo;
}

/* -----------------------------------------------------------------------
 * Helper: elapsed CPU time since todStart
 * ----------------------------------------------------------------------- */

/**
 * elapsedTime()
 *
 * Returns the number of microseconds elapsed since @todStart was
 * last sampled (i.e. since the Nucleus last gained control for this
 * process's benefit).
 */
static cpu_t elapsedTime(void)
{
    cpu_t now;
    STCK(now);
    return now - todStart;
}
