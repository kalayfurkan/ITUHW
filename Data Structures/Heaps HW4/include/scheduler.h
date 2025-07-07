#ifndef SCHEDULER_H
#define SCHEDULER_H

/*
 * Process Scheduler Implementation Header
 * 
 * This header defines a process scheduler that implements the Completely Fair Scheduler (CFS)
 * algorithm, similar to the one used in the Linux kernel. The scheduler uses a min-heap
 * data structure to efficiently manage process execution order based on virtual runtime.
 * 
 * The scheduler ensures fair CPU time distribution among all processes by tracking their
 * execution time and prioritizing processes that have received less CPU time.
 */

#include "min_heap.h"
#include "process.h"

/* 
 * Scheduler Structure
 * Manages the execution and scheduling of processes using a min-heap queue.
 * The scheduler maintains the current running process and enforces time slicing
 * to ensure fair CPU distribution among all processes.
 */
typedef struct {
    MinHeap* process_queue;     /* Min-heap queue storing processes ordered by vruntime */
    Process* current_process;   /* Pointer to the currently running process */
    int time_slice;             /* Duration (in ms) each process runs before rescheduling */
} Scheduler;

/*
 * Creates and initializes a new scheduler instance.
 * The capacity parameter determines the maximum number of processes that can be managed.
 * Returns a pointer to the new scheduler, or NULL if allocation fails.
 */
Scheduler* create_scheduler(int capacity);

/*
 * Deallocates all resources associated with the scheduler.
 * This includes freeing the process queue, current process, and scheduler structure itself.
 * Should be called when the scheduler is no longer needed to prevent memory leaks.
 */
void destroy_scheduler(Scheduler* scheduler);

/*
 * Adds a new process to the scheduler's queue.
 * The process is inserted into the min-heap based on its virtual runtime,
 * maintaining the scheduling order where processes with lower virtual runtime
 * have higher priority for execution.
 */
void schedule_process(Scheduler* scheduler, Process process);

/*
 * Retrieves the next process to be executed based on virtual runtime.
 * Returns the process with the lowest virtual runtime from the queue.
 * If a process is currently running, it is placed back in the queue.
 * Returns NULL if no processes are available for execution.
 */
Process* get_next_process(Scheduler* scheduler);

/*
 * Updates the scheduler state for one time slice unit.
 * Increments the virtual runtime of the currently executing process
 */
void tick(Scheduler* scheduler);

#endif 