/**
 * interrupts.c
 * PandOSsh Phase 2 – Interrupt Exception Handler (Level 3, Section 7)
 *
 * Handles three categories of interrupts:
 *
 *   1. PLT (Processor Local Timer) interrupt (line 1, Section 7.2)
 *      – The current process's time quantum has expired.
 *      – Acknowledge PLT, save state to PCB, re-enqueue, call Scheduler.
 *
 *   2. Interval Timer interrupt (line 2, Section 7.3)
 *      – Pseudo-clock tick: unblock all PCBs waiting on the Pseudo-clock.
 *      – Reload Interval Timer with 100 ms, V all waiters, return to
 *        Current Process (or call Scheduler if there is none).
 *
 *   3. Non-timer device interrupts (lines 3-7, Section 7.1)
 *      – Acknowledge the highest-priority pending interrupt.
 *      – Save the device status, V the device semaphore, unblock the
 *        waiting PCB, place its status in its a0, enqueue it, return
 *        to the Current Process.
 *
 * Authors: Luca Bassi, Luca Orlandello
 */

#include "interrupts.h"
#include "initial.h"
#include "scheduler.h"
#include "exceptions.h"
#include "pandos_const.h"
#include "pandos_types.h"
#include "pcb.h"
#include "asl.h"
#include <uriscv/liburiscv.h>
#include <uriscv/arch.h>

/* -----------------------------------------------------------------------
 * Static helper prototypes
 * ----------------------------------------------------------------------- */
static void handleDeviceInterrupt(int intLineNo);
static int  highestPendingDevice(int intLineNo);
static int  terminalDevSemIndex(int devNo, int isTransmit);
static int  devSemIndex(int intLineNo, int devNo);

/* -----------------------------------------------------------------------
 * interruptHandler() – top-level interrupt dispatcher
 * ----------------------------------------------------------------------- */

/**
 * interruptHandler()
 *
 * Called from exceptionHandler() when CAUSE_IS_INT is true.
 * Determines the highest-priority pending interrupt and dispatches
 * to the appropriate sub-handler.
 *
 * Priority order (lowest number = highest priority):
 *   Line 1: PLT
 *   Line 2: Interval Timer (Pseudo-clock)
 *   Lines 3-7: peripheral devices (disk, flash, net, printer, terminal)
 */
void interruptHandler(void)
{
    unsigned int cause = ((state_t *) BIOSDATAPAGE)->cause;

    /* ---------------------------------------------------------------
     * Check interrupt lines from highest to lowest priority.
     * The Cause register bits corresponding to each interrupt line:
     *   bit 7  → line 1 (PLT,           IL_CPUTIMER, code 7)
     *   bit 3  → line 2 (Interval Timer, IL_TIMER,   code 3)
     *   bits 17-21 → lines 3-7 (devices, codes 17-21)
     *
     * We use bitwise AND with the Interrupt Pending mask bits that
     * are encoded in the Cause register.
     * --------------------------------------------------------------- */

    /* PLT (line 1) – highest priority */
    if (cause & CAUSE_IP(IL_CPUTIMER))
    {
        handlePLTInterrupt();
        return;
    }

    /* Interval Timer (line 2) */
    if (cause & CAUSE_IP(IL_TIMER))
    {
        handleTimerInterrupt();
        return;
    }

    /* Peripheral device lines 3-7 */
    for (int line = IL_DISK; line <= IL_TERMINAL; line++)
    {
        if (cause & CAUSE_IP(line))
        {
            handleDeviceInterrupt(line);
            return;
        }
    }

    /* Spurious interrupt – return to Current Process if it exists. */
    if (currentProcess != NULL)
        LDST((state_t *) BIOSDATAPAGE);
    else
        schedule();
}

/* -----------------------------------------------------------------------
 * Section 7.2 – PLT (Processor Local Timer) interrupt
 * ----------------------------------------------------------------------- */

/**
 * handlePLTInterrupt()
 *
 * The current process has used its 5 ms quantum.
 *   1. Acknowledge the PLT by loading a non-zero value (re-arm later
 *      by the Scheduler when a process is dispatched).
 *   2. Copy the saved exception state into the Current Process's PCB.
 *   3. Charge the CPU time for this quantum to the process.
 *   4. Return the process to the Ready Queue.
 *   5. Call the Scheduler to pick the next process.
 */
void handlePLTInterrupt(void)
{
    /* Acknowledge: writing any value resets the PLT interrupt. */
    setTIMER(TIMESLICE);

    /* Save the processor state at the time of the interrupt into the PCB. */
    currentProcess->p_s = *((state_t *) BIOSDATAPAGE);

    /* Charge the elapsed time to the process. */
    currentProcess->p_time += elapsedTime();

    /* Transition: "running" → "ready". */
    insertProcQ(&readyQueue, currentProcess);
    currentProcess = NULL;

    schedule();
}

/* -----------------------------------------------------------------------
 * Section 7.3 – Interval Timer / Pseudo-clock interrupt
 * ----------------------------------------------------------------------- */

/**
 * handleTimerInterrupt()
 *
 * The Interval Timer has fired: it is time for a Pseudo-clock tick.
 *   1. Reload the Interval Timer with 100 ms.
 *   2. V the Pseudo-clock semaphore once for every PCB blocked on it,
 *      moving all of them back to the Ready Queue.
 *   3. Return control to the Current Process (LDST) if one exists;
 *      otherwise call the Scheduler.
 */
void handleTimerInterrupt(void)
{
    /* Acknowledge and reload the Interval Timer. */
    LDIT(PSECOND);

    /* Unblock ALL processes waiting for the Pseudo-clock tick. */
    pcb_t *unblocked;
    while ((unblocked = removeBlocked(&deviceSemaphores[MAXDEVICECNT])) != NULL)
    {
        softBlockCount--;
        insertProcQ(&readyQueue, unblocked);
    }
    /* Reset the Pseudo-clock semaphore to 0 (all waiters released). */
    deviceSemaphores[MAXDEVICECNT] = 0;

    /* Return to the Current Process or call the Scheduler. */
    if (currentProcess != NULL)
        LDST((state_t *) BIOSDATAPAGE);
    else
        schedule();
}

/* -----------------------------------------------------------------------
 * Section 7.1 – Non-timer device interrupt
 * ----------------------------------------------------------------------- */

/**
 * handleDeviceInterrupt()
 *
 * Processes the single highest-priority pending interrupt on @intLineNo.
 *
 *   1. Determine the device number from the Interrupting Devices Bit Map.
 *   2. Compute the device register base address.
 *   3. Read and save the device status.
 *   4. Acknowledge the interrupt (write ACK to command register).
 *   5. V the device semaphore; if a PCB is unblocked, place the status
 *      in its a0, move it to the Ready Queue.
 *   6. Return to the Current Process (or call Scheduler if none).
 */
static void handleDeviceInterrupt(int intLineNo)
{
    /* Step 1: find the highest-priority (lowest-numbered) device with a
     * pending interrupt on this line. */
    int devNo = highestPendingDevice(intLineNo);
    if (devNo < 0)
    {
        /* No pending device on this line – spurious. */
        if (currentProcess != NULL)
            LDST((state_t *) BIOSDATAPAGE);
        else
            schedule();
        return;
    }

    /* Step 2: compute the device register base address.
     *   devAddrBase = 0x10000054 + ((IntlineNo - 3) * 0x80) + (DevNo * 0x10)  */
    devreg_t *devReg = (devreg_t *)(DEV_REG_START
                       + ((intLineNo - 3) * (DEVPERINT * DEV_REG_SIZE))
                       + (devNo * DEV_REG_SIZE));

    unsigned int statusCode;
    int          semIdx;
    int          isTerminal = (intLineNo == IL_TERMINAL);

    if (isTerminal)
    {
        termreg_t *termReg = (termreg_t *) devReg;

        /* Transmission sub-device has higher priority than receive. */
        if ((termReg->transm_status & DEV_TERM_STATUS_MASK) == DEV_TTRS_S_CHARTRSM)
        {
            /* Transmit interrupt. */
            statusCode = termReg->transm_status;
            termReg->transm_command = ACK;
            semIdx = terminalDevSemIndex(devNo, 1); /* TX */
        }
        else
        {
            /* Receive interrupt. */
            statusCode = termReg->recv_status;
            termReg->recv_command = ACK;
            semIdx = terminalDevSemIndex(devNo, 0); /* RX */
        }
    }
    else
    {
        /* Standard device. */
        statusCode        = devReg->dtp.status;
        devReg->dtp.command = ACK;
        semIdx = devSemIndex(intLineNo, devNo);
    }

    /* Step 4: V the device semaphore. */
    deviceSemaphores[semIdx]++;

    /* Step 5: if a PCB was waiting, unblock it. */
    pcb_t *unblocked = NULL;
    if (deviceSemaphores[semIdx] <= 0)
    {
        unblocked = removeBlocked(&deviceSemaphores[semIdx]);
    }

    if (unblocked != NULL)
    {
        softBlockCount--;
        /* Pass the device status back to the unblocked process via a0. */
        unblocked->p_s.reg_a0 = statusCode;
        insertProcQ(&readyQueue, unblocked);
    }

    /* Step 6: return to the Current Process or schedule. */
    if (currentProcess != NULL)
        LDST((state_t *) BIOSDATAPAGE);
    else
        schedule();
}

/* -----------------------------------------------------------------------
 * Helper: find the highest-priority (lowest-numbered) pending device
 * ----------------------------------------------------------------------- */

/**
 * highestPendingDevice()
 *
 * Scans the Interrupting Devices Bit Map word for @intLineNo and returns
 * the number (0-7) of the lowest-set bit (= highest priority device).
 * Returns -1 if no device has a pending interrupt on this line.
 */
static int highestPendingDevice(int intLineNo)
{
    /*
     * The Interrupting Devices Bit Map starts at 0x1000.0040.
     * Word 0 (offset 0x00) → line 3 (disk)
     * Word 1 (offset 0x04) → line 4 (flash)
     * ...
     * Word 4 (offset 0x10) → line 7 (terminal)
     */
    unsigned int *bitmap = (unsigned int *)(DEV_BITMAP_START
                           + ((intLineNo - 3) * WORDLEN));
    unsigned int word = *bitmap;

    if (word == 0)
        return -1;

    /* Find the lowest set bit. */
    for (int i = 0; i < DEVPERINT; i++)
    {
        if (word & (1u << i))
            return i;
    }
    return -1;
}

/* -----------------------------------------------------------------------
 * Helper: semaphore index for terminal sub-devices
 * ----------------------------------------------------------------------- */

/**
 * terminalDevSemIndex()
 *
 * Returns the index into deviceSemaphores[] for terminal @devNo.
 *   TX (transmit): indices 32-39
 *   RX (receive):  indices 40-47
 */
static int terminalDevSemIndex(int devNo, int isTransmit)
{
    /* Terminal TX starts at index (IL_TERMINAL - 3) * DEVPERINT = 4*8 = 32 */
    int txBase = (IL_TERMINAL - IL_DISK) * DEVPERINT; /* = 32 */
    if (isTransmit)
        return txBase + devNo;
    else
        return txBase + DEVPERINT + devNo; /* RX: 40-47 */
}

/* -----------------------------------------------------------------------
 * Helper: semaphore index for non-terminal devices
 * ----------------------------------------------------------------------- */

/**
 * devSemIndex()
 *
 * Returns the index into deviceSemaphores[] for a non-terminal device.
 *   Line 3 (disk):    0-7
 *   Line 4 (flash):   8-15
 *   Line 5 (net):    16-23
 *   Line 6 (printer): 24-31
 */
static int devSemIndex(int intLineNo, int devNo)
{
    return ((intLineNo - IL_DISK) * DEVPERINT) + devNo;
}
