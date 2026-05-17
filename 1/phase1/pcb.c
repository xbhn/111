/**
 * pcb.c
 * PandOSsh Phase 1 – PCB Manager (Level 2)
 *
 * This module implements three related sets of functions:
 *   1. PCB allocation/deallocation  (Section 2.1)
 *   2. Process queue maintenance    (Section 2.2)
 *   3. Process tree maintenance     (Section 2.3)
 *
 * All PCBs come from the static array pcbFree_table[MAXPROC].
 * Free PCBs are kept on the doubly-circular linked list pcbFree_h,
 * threaded through the p_list field of each PCB.
 *
 * Authors: Luca Bassi, Luca Orlandello
 */

#include "./headers/pcb.h"

/* -----------------------------------------------------------------------
 * Module-private globals
 * ----------------------------------------------------------------------- */

/* Sentinel node for the free-PCB list. */
static struct list_head pcbFree_h;

/* Static storage for all MAXPROC PCBs. */
static pcb_t pcbFree_table[MAXPROC];

/* Monotonically increasing PID counter.
 * Starts at 1; incremented on every successful allocPcb(). */
static int next_pid = 1;

/* =========================================================================
 * Section 2.1 – PCB Allocation and Deallocation
 * ========================================================================= */

/**
 * initPcbs()
 *
 * Initialize the free-PCB list (pcbFree_h) to contain all MAXPROC entries
 * of the static array pcbFree_table[].
 * Must be called exactly once, during system initialisation.
 */
void initPcbs(void)
{
    /* Make pcbFree_h an empty list (sentinel points to itself). */
    INIT_LIST_HEAD(&pcbFree_h);

    /* Insert every entry of the table onto the free list. */
    for (int i = 0; i < MAXPROC; i++)
        list_add_tail(&pcbFree_table[i].p_list, &pcbFree_h);
}

/**
 * freePcb()
 *
 * Return the PCB pointed to by @p to the free list so it can be reused.
 * The caller is responsible for having removed the PCB from any queue/tree
 * it was part of before calling this function.
 */
void freePcb(pcb_t *p)
{
    list_add_tail(&p->p_list, &pcbFree_h);
}

/**
 * allocPcb()
 *
 * Remove one PCB from the free list, zero-initialise ALL of its fields
 * (except p_pid, which is set to the next available PID), and return a
 * pointer to it.
 * Returns NULL if the free list is empty.
 */
pcb_t *allocPcb(void)
{
    if (list_empty(&pcbFree_h))
        return NULL;

    /* Take the first entry off the free list. */
    struct list_head *node = pcbFree_h.next;
    list_del(node); /* detach from free list */

    pcb_t *p = container_of(node, pcb_t, p_list);

    /* ----------------------------------------------------------------
     * Zero-initialise every field so no previous value persists.
     * ---------------------------------------------------------------- */

    /* Process queue linkage – make it a self-contained empty list. */
    INIT_LIST_HEAD(&p->p_list);

    /* Process tree fields. */
    p->p_parent = NULL;
    INIT_LIST_HEAD(&p->p_child);
    INIT_LIST_HEAD(&p->p_sib);

    /* Processor state – zeroed (the caller sets meaningful values). */
    /* state_t p_s is a struct; assign element-by-element would be
     * fragile across compiler versions – use a cast to zero the block. */
    int *raw = (int *) &p->p_s;
    for (int i = 0; i < (int)(sizeof(state_t) / sizeof(int)); i++)
        raw[i] = 0;

    /* Accumulated CPU time. */
    p->p_time = 0;

    /* Semaphore address – not blocked on anything. */
    p->p_semAdd = NULL;

    /* Support structure – none at this level. */
    p->p_supportStruct = NULL;

    /* Priority – default low. */
    p->p_prio = 0;

    /* Assign a unique PID. */
    p->p_pid = next_pid++;

    return p;
}

/* =========================================================================
 * Section 2.2 – Process Queue Maintenance
 * =========================================================================
 *
 * Process queues are doubly-circular linked lists, threaded through the
 * p_list field of each PCB. The "head" node is a bare list_head sentinel
 * that does not belong to any PCB; it is provided by the caller.
 *
 * Ordering rule: the queue is kept in descending priority order.
 * Among PCBs with the same priority, FIFO order is maintained (a new PCB
 * is inserted AFTER all existing PCBs with the same priority value).
 * ========================================================================= */

/**
 * mkEmptyProcQ()
 *
 * Initialise the list_head pointed to by @head as an empty process queue
 * (sentinel node pointing to itself).
 */
void mkEmptyProcQ(struct list_head *head)
{
    INIT_LIST_HEAD(head);
}

/**
 * emptyProcQ()
 *
 * Return TRUE (non-zero) if the process queue whose sentinel is @head
 * contains no PCBs, FALSE (0) otherwise.
 */
int emptyProcQ(struct list_head *head)
{
    return list_empty(head);
}

/**
 * insertProcQ()
 *
 * Insert PCB @p into the process queue headed by @head.
 *
 * The queue is maintained in descending priority order; among equal-priority
 * entries the new PCB goes after all existing ones (FIFO within a priority
 * band).
 *
 * Example: inserting N(priority=10) into [A(20), B(10), C(0)]
 * yields [A(20), B(10), N(10), C(0)].
 */
void insertProcQ(struct list_head *head, pcb_t *p)
{
    struct list_head *pos;

    /*
     * Walk the list from the head (highest priority) forward.
     * We want to stop just before the first entry whose priority is
     * strictly less than p->p_prio.  If no such entry exists, p goes
     * at the tail (after all entries with priority >= p->p_prio).
     */
    list_for_each(pos, head)
    {
        pcb_t *current = container_of(pos, pcb_t, p_list);
        if (current->p_prio < p->p_prio)
        {
            /* Insert p just before this lower-priority entry. */
            __list_add(&p->p_list, pos->prev, pos);
            return;
        }
    }

    /* All existing entries have priority >= p->p_prio; append at tail. */
    list_add_tail(&p->p_list, head);
}

/**
 * headProcQ()
 *
 * Return a pointer to the first (highest-priority) PCB in the process queue
 * headed by @head, without removing it.
 * Returns NULL if the queue is empty.
 */
pcb_t *headProcQ(struct list_head *head)
{
    if (list_empty(head))
        return NULL;

    return container_of(head->next, pcb_t, p_list);
}

/**
 * removeProcQ()
 *
 * Remove and return the first (highest-priority) PCB from the process queue
 * headed by @head.
 * Returns NULL if the queue is empty.
 */
pcb_t *removeProcQ(struct list_head *head)
{
    if (list_empty(head))
        return NULL;

    struct list_head *node = head->next;
    list_del(node);

    return container_of(node, pcb_t, p_list);
}

/**
 * outProcQ()
 *
 * Remove PCB @p from the process queue headed by @head.
 * Returns @p on success, or NULL if @p is not found in the queue.
 */
pcb_t *outProcQ(struct list_head *head, pcb_t *p)
{
    struct list_head *pos;

    list_for_each(pos, head)
    {
        if (pos == &p->p_list)
        {
            list_del(pos);
            return p;
        }
    }

    return NULL; /* p was not in this queue */
}

/* =========================================================================
 * Section 2.3 – Process Tree Maintenance
 * =========================================================================
 *
 * Each PCB can be the parent of a list of children. The parent holds a
 * sentinel (p_child) for that list; each child uses its p_sib field as
 * the link node within that sibling list.
 *
 * Layout for a parent P with children C1, C2, C3 (in insertion order):
 *
 *   P.p_child <-> C1.p_sib <-> C2.p_sib <-> C3.p_sib <-> P.p_child
 *
 * Each child's p_parent pointer refers back to P.
 * ========================================================================= */

/**
 * emptyChild()
 *
 * Return TRUE (non-zero) if PCB @p has no children, FALSE (0) otherwise.
 */
int emptyChild(pcb_t *p)
{
    return list_empty(&p->p_child);
}

/**
 * insertChild()
 *
 * Make PCB @p a child of PCB @prnt.
 * @p is added at the TAIL of @prnt's children list (preserving arrival order).
 */
void insertChild(pcb_t *prnt, pcb_t *p)
{
    p->p_parent = prnt;
    list_add_tail(&p->p_sib, &prnt->p_child);
}

/**
 * removeChild()
 *
 * Remove and return the first child of PCB @p.
 * Returns NULL if @p has no children.
 */
pcb_t *removeChild(pcb_t *p)
{
    if (list_empty(&p->p_child))
        return NULL;

    /* The first child's p_sib node is the first entry after the sentinel. */
    struct list_head *node = p->p_child.next;
    pcb_t *child = container_of(node, pcb_t, p_sib);

    list_del(node);
    child->p_parent = NULL;

    return child;
}

/**
 * outChild()
 *
 * Remove PCB @p from the children list of its parent.
 * Returns NULL if @p has no parent (i.e. it is a root or unattached).
 * Returns @p on success.
 */
pcb_t *outChild(pcb_t *p)
{
    if (p->p_parent == NULL)
        return NULL;

    list_del(&p->p_sib);
    p->p_parent = NULL;

    return p;
}
