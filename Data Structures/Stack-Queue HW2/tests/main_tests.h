#include <munit.h>
#include "process_manager.h"
#include "process_queue.h"
#include "type_process.h"
#include "failure_stack.h"
#include "insertion_queue.h"
#include "execution_functions.h"
#include <stdio.h>

// Test initialization of the process manager
static MunitResult test_initialize_process_manager(const MunitParameter params[], void* data) {
    PROCESS_MANAGER pm;
    initialize_process_manager(&pm);
    munit_assert_true(isEmpty(&pm));
    return MUNIT_OK;
}

// Test initialization of the execution queue
static MunitResult test_initialize_execution_queue(const MunitParameter params[], void* data) {
    INSERTION_QUEUE iq;
    initialize_execution_queue(&iq);
    munit_assert_true(isEmpty(&iq));
    return MUNIT_OK;
}

// Test reading process file into process manager
static MunitResult test_read_file_and_process(const MunitParameter params[], void* data) {
    PROCESS_MANAGER pm;
    initialize_process_manager(&pm);

    read_process_file("initial_processes.txt", &pm);
    munit_assert_false(isEmpty(&pm));

    PROCESS_QUEUE pq = delete_front(&pm);
    PROCESS p = dequeue(&pq);
    munit_assert_int(p.pid, ==, 151);
    munit_assert_int(p.priority, ==, 1);

    return MUNIT_OK;
}

// Test reading execution file into execution queue
static MunitResult test_read_execution_file_and_process(const MunitParameter params[], void* data) {
    INSERTION_QUEUE iq;
    initialize_execution_queue(&iq);

    read_insertion_file("arriving_processes.txt", &iq);
    munit_assert_false(isEmpty(&iq));

    PROCESS_QUEUE pq = dequeue(&iq);
    munit_assert_int(pq.iteration, ==, 1);
    PROCESS p = dequeue(&pq);
    munit_assert_int(p.pid, ==, 200);

    return MUNIT_OK;
}

// Test execution loop
static MunitResult test_execution_loop(const MunitParameter params[], void* data) {
    PROCESS_MANAGER pm;
    INSERTION_QUEUE iq;
    FAILURE_STACK fs;

    initialize_process_manager(&pm);
    initialize_execution_queue(&iq);
    initialize_failed_stack(&fs);

    // Load test data from files or set mock data for process manager and execution queue
    read_process_file("initial_processes.txt", &pm);
    read_insertion_file("arrving_processes.txt", &iq);

    execution_loop(&pm, &iq, &fs);

    munit_assert_true(isEmpty(&pm)); // Check that the process manager is empty after execution
    munit_assert_false(isEmpty(&fs)); // Check that the failed stack is populated if any process failed

    PROCESS_QUEUE failedQueue = pop(&fs);
    munit_assert_false(isEmpty(&failedQueue)); // Check that failed processes exist

    PROCESS p = dequeue(&failedQueue);
    munit_assert_int(p.pid % 8, ==, 0); // Check that failed process matches expected condition

    return MUNIT_OK;
}

// Define the test cases
static MunitTest test_suite_tests[] = {
    { "/test_initialize_process_manager", test_initialize_process_manager, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { "/test_initialize_execution_queue", test_initialize_execution_queue, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { "/test_read_file_and_process", test_read_file_and_process, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { "/test_read_execution_file_and_process", test_read_execution_file_and_process, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { "/test_execution_loop", test_execution_loop, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { NULL, NULL, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL } // End of tests
};

// Define the test suite
static const MunitSuite test_suite = {
    "/process_manager_tests", // Suite name
    test_suite_tests,         // Tests
    NULL,                     // No sub-suites
    1,                         // Number of iterations to run each test
    MUNIT_SUITE_OPTION_NONE    // Suite options
};
