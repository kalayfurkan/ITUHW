#include <munit.h>
#include <stdio.h>
#include "test_min_heap.h"
#include "test_scheduler.h"

// Define the main test suite
static MunitSuite test_suite;
static MunitSuite suites[3];  // One for min_heap, one for scheduler, one for NULL terminator

int main(int argc, char* argv[MUNIT_ARRAY_PARAM(argc + 1)]) {
    // Initialize suites at runtime
    suites[0] = min_heap_suite;
    suites[1] = scheduler_suite;
    suites[2] = (MunitSuite){NULL, NULL, NULL, 0, MUNIT_SUITE_OPTION_NONE};

    // Initialize main test suite
    test_suite = (MunitSuite){
        "/tests",
        NULL,
        suites,
        1,
        MUNIT_SUITE_OPTION_NONE
    };

    return munit_suite_main(&test_suite, NULL, argc, argv);
}