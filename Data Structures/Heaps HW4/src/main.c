#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "../include/scheduler.h"

// Function to print process information
void print_process_info(Process* p) {
    if (!p) return;
    printf("Process ID: %d, Nice: %d, VRuntime: %d, Running: %s\n",
           p->pid,
           p->nice,
           p->vruntime,
           p->is_running ? "Yes" : "No");
}

int main() {
    // Create a scheduler with capacity for 10 processes
    Scheduler* scheduler = create_scheduler(10);
    if (!scheduler) {
        fprintf(stderr, "Failed to create scheduler\n");
        return 1;
    }

    // Create processes with different nice values
    Process processes[] = {
        create_process(1, 1),    // Normal priority
        create_process(2, 1),  // High priority
        create_process(3, 1),   // Low priority
        create_process(4, 1),   // Above normal priority
        create_process(5, 1)     // Below normal priority
    };

    // Schedule all processes
    printf("Initial scheduling of processes:\n");
    for (int i = 0; i < 5; i++) {
        schedule_process(scheduler, processes[i]);
        printf("Scheduled process %d with nice value %d\n", 
               processes[i].pid, processes[i].nice);
    }
    printf("\n");

    // Simulate scheduler running for 50 ticks
    printf("Scheduler simulation for 50 ticks:\n");
    printf("-----------------------------------\n");
    
    for (int tick_count = 0; tick_count < 50; tick_count++) {
        // Get next process to run
        Process* current = get_next_process(scheduler);
        
        if (current) {
            printf("Tick %d: ", tick_count);
            print_process_info(current);
        } else {
            printf("Tick %d: No process to run\n", tick_count);
            continue;
        }

        // Simulate process execution
        tick(scheduler);

        // Add small delay to make output readable
        usleep(100000);  // 100ms delay
    }

    printf("\nSimulation completed\n");

    // Cleanup
    destroy_scheduler(scheduler);
    return 0;
} 