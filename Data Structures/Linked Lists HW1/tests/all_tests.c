#ifndef ALL_TESTS_H
#define ALL_TESTS_H

#include "munit.h"
#include "schedule_tests.h"

// Main function to run all test suites
int main(int argc, char* argv[MUNIT_ARRAY_PARAM(argc + 1)]) {
    // Run the schedule test suite
    return munit_suite_main(&schedule_test_suite, NULL, argc, argv);
}

#endif // ALL_TESTS_H
