#include "main_tests.h"

// Main function for running the test suite
int main(int argc, char* argv[]) {
    return munit_suite_main(&test_suite, NULL, argc, argv);
}