#include "../include/process.h"

Process create_process(int pid, int nice) {
    Process p = {
        .pid = pid,
        .vruntime = 0,
        .nice = nice,
        .is_running = false
    };
    
    // Ensure nice value is within bounds
    if (p.nice < -20) p.nice = -20;
    if (p.nice > 19) p.nice = 19;
    
    return p;
}

void update_vruntime(Process* process, int execution_time) {
    // Update vruntime based on execution time and nice value
    // Higher nice value means vruntime increases faster
    // We add 20 to nice to make it positive (range 0-39)
    // Add 1 to avoid division by zero
    int nice_factor = process->nice + 20 + 1;
    process->vruntime += execution_time * nice_factor;
} 