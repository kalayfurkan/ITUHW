#ifndef PROCESS_H
#define PROCESS_H

/*
 * Process Header
 * 
 * This header defines the Process structure and related functions for a simple
 * process scheduler implementation. It implements a Completely Fair Scheduler (CFS)
 * approach where each process has a virtual runtime that increases based on its
 * execution time and priority (nice value).
 */

#include <stdbool.h>

/* 
 * Process structure represents a single process in the system.
 * It contains essential information needed for process scheduling.
 */
typedef struct {
    int pid;               // Process ID
    int vruntime;          // Virtual runtime - tracks process's fair share of CPU
    int nice;              // Nice value (-20 to +19) - determines process priority
    bool is_running;       // Process state - indicates if process is currently running
} Process;

/*
 * Creates a new process with the specified process ID and nice value.
 * The nice value determines the process's priority in scheduling.
 * Nice values range from -20 (highest priority) to +19 (lowest priority).
 * Returns a new Process structure initialized with the given values.
 */
Process create_process(int pid, int nice);

/*
 * Updates the virtual runtime of a process based on its execution time.
 * The virtual runtime increases more quickly for processes with higher nice values,
 * ensuring fair CPU distribution among processes.
 * The execution_time parameter represents the actual time the process has run.
 */
void update_vruntime(Process* process, int execution_time);

#endif 