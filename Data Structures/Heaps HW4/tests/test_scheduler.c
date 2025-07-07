#include <munit.h>
#include <stdio.h>
#include "../include/scheduler.h"
#include "test_scheduler.h"

// Test scheduler creation and destruction
static MunitResult test_scheduler_create(const MunitParameter params[], void* data) {
    Scheduler* scheduler = create_scheduler(10);
    munit_assert_not_null(scheduler);
    munit_assert_null(scheduler->current_process);
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Test process scheduling
static MunitResult test_schedule_process(const MunitParameter params[], void* data) {
    Scheduler* scheduler = create_scheduler(5);
    
    // Create and schedule some processes
    Process processes[] = {
        {.pid = 1, .vruntime = 5, .nice = 0, .is_running = false},
        {.pid = 2, .vruntime = 3, .nice = 0, .is_running = false},
        {.pid = 3, .vruntime = 7, .nice = 0, .is_running = false}
    };
    
    for (int i = 0; i < 3; i++) {
        schedule_process(scheduler, processes[i]);
    }
    
    // Get next process and verify it's the one with lowest vruntime
    Process* next = get_next_process(scheduler);
    munit_assert_not_null(next);
    munit_assert_int(next->pid, ==, 2);  // Process with vruntime 3
    munit_assert_true(next->is_running);
    
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Test process scheduling with nice values
static MunitResult test_nice_values(const MunitParameter params[], void* data) {
    Scheduler* scheduler = create_scheduler(5);
    
    // Create processes with different nice values
    Process processes[] = {
        {.pid = 1, .vruntime = 10, .nice = -10, .is_running = false},  // Higher priority
        {.pid = 2, .vruntime = 5, .nice = 0, .is_running = false},     // Normal priority
        {.pid = 3, .vruntime = 1, .nice = 10, .is_running = false}     // Lower priority
    };
    
    for (int i = 0; i < 3; i++) {
        schedule_process(scheduler, processes[i]);
    }
    
    // Run for a few ticks and verify vruntime updates
    Process* next = get_next_process(scheduler);
    munit_assert_not_null(next);
    
    // Simulate some ticks
    for (int i = 0; i < 3; i++) {
        tick(scheduler);
    }
    
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Test scheduler tick functionality
static MunitResult test_scheduler_tick(const MunitParameter params[], void* data) {
    Scheduler* scheduler = create_scheduler(5);
    
    Process process = {.pid = 1, .vruntime = 0, .nice = 0, .is_running = false};
    schedule_process(scheduler, process);
    
    Process* running = get_next_process(scheduler);
    munit_assert_not_null(running);
    
    // Initial vruntime should be 0
    munit_assert_int(running->vruntime, ==, 0);
    
    // After one tick, vruntime should increase
    tick(scheduler);
    munit_assert_int(running->vruntime, >, 0);
    
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Test multiple scheduling cycles
static MunitResult test_multiple_scheduling_cycles(const MunitParameter params[], void* data) {
    Scheduler* scheduler = create_scheduler(5);
    
    // Create initial set of processes with different priorities
    Process processes[] = {
        {.pid = 1, .vruntime = 5, .nice = -5, .is_running = false},   // Higher priority
        {.pid = 2, .vruntime = 3, .nice = 0, .is_running = false},    // Normal priority
        {.pid = 3, .vruntime = 7, .nice = 5, .is_running = false}     // Lower priority
    };
    
    // Schedule all processes
    for (int i = 0; i < 3; i++) {
        schedule_process(scheduler, processes[i]);
    }
    
    // Run multiple scheduling cycles and verify behavior
    int expected_order[] = {2, 1, 3};  // Expected initial order based on vruntime
    for (int cycle = 0; cycle < 3; cycle++) {
        Process* next = get_next_process(scheduler);
        munit_assert_not_null(next);
        munit_assert_int(next->pid, ==, expected_order[cycle]);
        
        // Run process for some time
        for (int t = 0; t < 3; t++) {
            tick(scheduler);
        }
    }
    
    // Verify that processes with lower nice values (higher priority) 
    // accumulate vruntime more slowly
    Process* next = get_next_process(scheduler);
    munit_assert_not_null(next);
    munit_assert_int(next->pid, ==, 1);  // Higher priority process should run again
    
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Test dynamic process addition during execution
static MunitResult test_dynamic_process_addition(const MunitParameter params[], void* data) {
    Scheduler* scheduler = create_scheduler(10);
    
    // Start with two processes
    Process initial_processes[] = {
        {.pid = 1, .vruntime = 10, .nice = 0, .is_running = false},
        {.pid = 2, .vruntime = 15, .nice = 0, .is_running = false}
    };
    
    for (int i = 0; i < 2; i++) {
        schedule_process(scheduler, initial_processes[i]);
    }
    
    // Run first process
    Process* first = get_next_process(scheduler);
    munit_assert_not_null(first);
    munit_assert_int(first->pid, ==, 1);
    
    // Add new process with lower vruntime during execution
    Process new_process = {.pid = 3, .vruntime = 5, .nice = 0, .is_running = false};
    schedule_process(scheduler, new_process);
    
    // Run some ticks
    for (int i = 0; i < 2; i++) {
        tick(scheduler);
    }
    
    // Next process should be the newly added one
    Process* next = get_next_process(scheduler);
    munit_assert_not_null(next);
    munit_assert_int(next->pid, ==, 3);
    
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Test edge case: Dynamic capacity growth
static MunitResult test_scheduler_capacity(const MunitParameter params[], void* data) {
    const int INITIAL_CAPACITY = 3;
    Scheduler* scheduler = create_scheduler(INITIAL_CAPACITY);
    
    // Create more processes than initial capacity
    Process processes[] = {
        {.pid = 1, .vruntime = 5, .nice = 0, .is_running = false},
        {.pid = 2, .vruntime = 3, .nice = 0, .is_running = false},
        {.pid = 3, .vruntime = 7, .nice = 0, .is_running = false},
        {.pid = 4, .vruntime = 1, .nice = 0, .is_running = false},  // Beyond initial capacity
        {.pid = 5, .vruntime = 9, .nice = 0, .is_running = false}   // Beyond initial capacity
    };
    
    // Schedule all processes - should succeed due to dynamic growth
    for (int i = 0; i < 5; i++) {
        schedule_process(scheduler, processes[i]);
    }
    
    // Verify we can get all processes, including those beyond initial capacity
    Process* next;
    int expected_order[] = {4, 2, 1, 3, 5};  // Expected order based on vruntime
    
    for (int i = 0; i < 5; i++) {
        next = get_next_process(scheduler);
        munit_assert_not_null(next);
        munit_assert_int(next->pid, ==, expected_order[i]);
        tick(scheduler);
    }
    
    destroy_scheduler(scheduler);
    return MUNIT_OK;
}

// Define test suite
static MunitTest scheduler_tests[] = {
    {"/create", test_scheduler_create, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/schedule_process", test_schedule_process, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/nice_values", test_nice_values, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/tick", test_scheduler_tick, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/multiple_cycles", test_multiple_scheduling_cycles, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/dynamic_addition", test_dynamic_process_addition, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/capacity", test_scheduler_capacity, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {NULL, NULL, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL}
};

// Export the test suite
const MunitSuite scheduler_suite = {
    "/scheduler",
    scheduler_tests,
    NULL,
    1,
    MUNIT_SUITE_OPTION_NONE
}; 