/**
 * asl.c
 * PandOSsh Phase 1 – Active Semaphore List (ASL) Manager (Level 2, Section 3)
 *
 * A semaphore descriptor (semd_t) associates an integer semaphore key
 * (a pointer to the integer in memory) with the queue of PCBs blocked on
 * that semaphore.
 *
 * A semaphore is "active" if at least one PCB is blocked on it.  Active
 * descriptors live on the semd_h list (the ASL).  Unused descriptors live
 * on the semdFree_h list.  Both lists are doubly-circular linked lists
 * threaded through the s_link field of semd_t.
 *
 * The ASL is kept ordered by ascending key address value so that
 * lookups terminate as soon as a key greater than the target is found.
 *
 * Authors: Luca Bassi, Luca Orlandello
 */

#include "./headers/asl.h"

/* -----------------------------------------------------------------------
 * Module-private globals
 * ----------------------------------------------------------------------- */

/* Static pool of semaphore descriptors. */
static semd_t semd_table[MAXPROC];

/* Sentinel for the free-descriptor list. */
static struct list_head semdFree_h;

/* Sentinel for the Active Semaphore List. */
static struct list_head semd_h;

/* -----------------------------------------------------------------------
 * Private helpers
 * ----------------------------------------------------------------------- */

/**
 * findSemd()
 *
 * Search the ASL for a descriptor whose s_key equals @semAdd.
 * Returns a pointer to the descriptor if found, NULL otherwise.
 *
 * Because the ASL is ordered by ascending key value, the search can
 * stop early as soon as the current key exceeds @semAdd.
 */
static semd_t *findSemd(int *semAdd)
{
    semd_t *s;
    list_for_each_entry(s, &semd_h, s_link)
    {
        if (s->s_key == semAdd)
            return s;
        if (s->s_key > semAdd)
            break; /* not in the list (ordered) */
    }
    return NULL;
}

/**
 * allocSemd()
 *
 * Allocate a free semaphore descriptor from semdFree_h.
 * Returns a pointer to the descriptor, or NULL if the free list is empty.
 */
static semd_t *allocSemd(void)
{
    if (list_empty(&semdFree_h))
        return NULL;

    struct list_head *node = semdFree_h.next;
    list_del(node);
    return container_of(node, semd_t, s_link);
}

/**
 * freeSemd()
 *
 * Return a semaphore descriptor to the free list.
 */
static void freeSemd(semd_t *s)
{
    list_add_tail(&s->s_link, &semdFree_h);
}

/**
 * insertSemdInOrder()
 *
 * Insert semaphore descriptor @s into the ASL (semd_h) in ascending order
 * of s_key address value.
 */
static void insertSemdInOrder(semd_t *s)
{
    struct list_head *pos;

    list_for_each(pos, &semd_h)
    {
        semd_t *curr = container_of(pos, semd_t, s_link);
        if (curr->s_key > s->s_key)
        {
            /* Insert s just before curr (lower address = higher priority). */
            __list_add(&s->s_link, pos->prev, pos);
            return;
        }
    }

    /* No entry with a larger key was found → insert at the tail. */
    list_add_tail(&s->s_link, &semd_h);
}

/* =========================================================================
 * Public interface (Section 3.1)
 * ========================================================================= */

/**
 * initASL()
 *
 * Initialise both the free-descriptor list (semdFree_h) and the ASL (semd_h),
 * placing all MAXPROC entries of semd_table[] onto semdFree_h.
 * Called exactly once at system startup.
 */
void initASL(void)
{
    INIT_LIST_HEAD(&semdFree_h);
    INIT_LIST_HEAD(&semd_h);

    for (int i = 0; i < MAXPROC; i++)
        list_add_tail(&semd_table[i].s_link, &semdFree_h);
}

/**
 * insertBlocked()
 *
 * Insert PCB @p at the TAIL of the process queue associated with the
 * semaphore whose key is @semAdd, and set @p->p_semAdd = @semAdd.
 *
 * If the semaphore is not yet active (no descriptor exists for @semAdd):
 *   - Allocate a fresh descriptor from semdFree_h.
 *   - If semdFree_h is empty, return TRUE (error – no resources).
 *   - Otherwise initialise the descriptor (s_key = @semAdd, empty s_procq)
 *     and insert it into the ASL in key order.
 *
 * Returns FALSE (0) on success, TRUE (non-zero) if a descriptor was needed
 * but the free list was exhausted.
 */
int insertBlocked(int *semAdd, pcb_t *p)
{
    semd_t *s = findSemd(semAdd);

    if (s == NULL)
    {
        /* Semaphore not yet active: allocate a new descriptor. */
        s = allocSemd();
        if (s == NULL)
            return 1; /* semdFree_h is empty – error */

        /* Initialise the new descriptor. */
        s->s_key = semAdd;
        INIT_LIST_HEAD(&s->s_procq);

        /* Insert into the ASL in ascending key order. */
        insertSemdInOrder(s);
    }

    /* Link the PCB into the semaphore's queue and record the semaphore. */
    p->p_semAdd = semAdd;
    list_add_tail(&p->p_list, &s->s_procq);

    return 0; /* success */
}

/**
 * removeBlocked()
 *
 * Remove and return the first (head) PCB from the process queue of the
 * semaphore identified by @semAdd.
 *
 * If the semaphore's queue becomes empty after the removal, the descriptor
 * is taken off the ASL and returned to semdFree_h.
 *
 * Returns NULL if no descriptor for @semAdd exists in the ASL.
 */
pcb_t *removeBlocked(int *semAdd)
{
    semd_t *s = findSemd(semAdd);
    if (s == NULL)
        return NULL;

    /* Remove the head PCB from the semaphore's queue. */
    struct list_head *node = s->s_procq.next;
    list_del(node);

    pcb_t *p = container_of(node, pcb_t, p_list);
    p->p_semAdd = NULL;

    /* If the process queue is now empty, deactivate the semaphore. */
    if (list_empty(&s->s_procq))
    {
        list_del(&s->s_link);
        freeSemd(s);
    }

    return p;
}

/**
 * outBlocked()
 *
 * Remove PCB @p from the process queue of the semaphore recorded in
 * @p->p_semAdd.
 *
 * Returns NULL if @p is not found in that queue (error condition).
 * Returns @p on success.
 *
 * If the semaphore's queue becomes empty after the removal, the descriptor
 * is removed from the ASL and returned to semdFree_h.
 */
pcb_t *outBlocked(pcb_t *p)
{
    if (p->p_semAdd == NULL)
        return NULL;

    semd_t *s = findSemd(p->p_semAdd);
    if (s == NULL)
        return NULL;

    /* Verify that @p actually lives in this semaphore's queue. */
    struct list_head *pos;
    int found = 0;
    list_for_each(pos, &s->s_procq)
    {
        if (pos == &p->p_list)
        {
            found = 1;
            break;
        }
    }

    if (!found)
        return NULL; /* error: p not in this queue */

    list_del(&p->p_list);
    p->p_semAdd = NULL;

    /* Deactivate the semaphore if its queue is now empty. */
    if (list_empty(&s->s_procq))
    {
        list_del(&s->s_link);
        freeSemd(s);
    }

    return p;
}

/**
 * headBlocked()
 *
 * Return a pointer to the PCB at the head of the process queue for the
 * semaphore identified by @semAdd, without removing it.
 *
 * Returns NULL if @semAdd is not on the ASL or its queue is empty.
 */
pcb_t *headBlocked(int *semAdd)
{
    semd_t *s = findSemd(semAdd);
    if (s == NULL || list_empty(&s->s_procq))
        return NULL;

    return container_of(s->s_procq.next, pcb_t, p_list);
}
