#include <munit.h>
#include <stdio.h>
#include <string.h>
#include "../include/min_heap.h"

// Helper function to compare integers
static int compare_ints(const void* a, const void* b) {
    return (*(int*)a) - (*(int*)b);
}

// Helper function to compare strings
static int compare_strings(const void* a, const void* b) {
    return strcmp(*(const char**)a, *(const char**)b);
}

// Custom struct for testing
typedef struct {
    int priority;
    char* data;
} CustomItem;

// Helper function to compare custom structs
static int compare_custom_items(const void* a, const void* b) {
    return ((CustomItem*)a)->priority - ((CustomItem*)b)->priority;
}

// Test heap creation and destruction
static MunitResult test_heap_create(const MunitParameter params[], void* data) {
    MinHeap* heap = heap_create(10, sizeof(int), compare_ints);
    munit_assert_not_null(heap);
    munit_assert_size(heap_size(heap), ==, 0);
    heap_destroy(heap);
    return MUNIT_OK;
}

// Test heap insertion
static MunitResult test_heap_insert(const MunitParameter params[], void* data) {
    MinHeap* heap = heap_create(5, sizeof(int), compare_ints);
    
    // Insert values beyond initial capacity
    int values[] = {5, 3, 7, 1, 4, 6, 8};  // 7 values for 5 capacity
    for (int i = 0; i < 7; i++) {
        munit_assert_int(heap_insert(heap, &values[i]), ==, 1);  // Should succeed due to dynamic growth
    }
    
    // Verify size
    munit_assert_size(heap_size(heap), ==, 7);
    
    // Verify order by extracting
    int expected[] = {1, 3, 4, 5, 6, 7, 8};
    for (int i = 0; i < 7; i++) {
        int result;
        munit_assert_int(heap_extract_min(heap, &result), ==, 1);
        munit_assert_int(result, ==, expected[i]);
    }
    
    heap_destroy(heap);
    return MUNIT_OK;
}

// Test heap extract min
static MunitResult test_heap_extract_min(const MunitParameter params[], void* data) {
    MinHeap* heap = heap_create(5, sizeof(int), compare_ints);
    
    // Insert values in random order
    int values[] = {5, 3, 7, 1, 4};
    for (int i = 0; i < 5; i++) {
        heap_insert(heap, &values[i]);
    }
    
    // Extract and verify order
    int expected[] = {1, 3, 4, 5, 7};
    for (int i = 0; i < 5; i++) {
        int result;
        munit_assert_int(heap_extract_min(heap, &result), ==, 1);
        munit_assert_int(result, ==, expected[i]);
    }
    
    // Try extracting from empty heap
    int result;
    munit_assert_int(heap_extract_min(heap, &result), ==, 0);
    
    heap_destroy(heap);
    return MUNIT_OK;
}

// Test heap peek
static MunitResult test_heap_peek(const MunitParameter params[], void* data) {
    MinHeap* heap = heap_create(3, sizeof(int), compare_ints);
    
    // Peek empty heap
    int result;
    munit_assert_int(heap_peek(heap, &result), ==, 0);
    
    // Insert and peek
    int values[] = {3, 1, 2};
    for (int i = 0; i < 3; i++) {
        heap_insert(heap, &values[i]);
    }
    
    munit_assert_int(heap_peek(heap, &result), ==, 1);
    munit_assert_int(result, ==, 1);  // Should be the minimum value
    
    heap_destroy(heap);
    return MUNIT_OK;
}

// Test heap merge
static MunitResult test_heap_merge(const MunitParameter params[], void* data) {
    MinHeap* heap1 = heap_create(6, sizeof(int), compare_ints);
    MinHeap* heap2 = heap_create(3, sizeof(int), compare_ints);
    
    // Insert values into first heap
    int values1[] = {5, 3, 7};
    for (int i = 0; i < 3; i++) {
        heap_insert(heap1, &values1[i]);
    }
    
    // Insert values into second heap
    int values2[] = {4, 1, 6};
    for (int i = 0; i < 3; i++) {
        heap_insert(heap2, &values2[i]);
    }
    
    // Merge heaps
    munit_assert_int(heap_merge(heap1, heap2), ==, 1);
    
    // Verify size after merge
    munit_assert_size(heap_size(heap1), ==, 6);
    
    // Extract and verify order
    int expected[] = {1, 3, 4, 5, 6, 7};
    for (int i = 0; i < 6; i++) {
        int result;
        heap_extract_min(heap1, &result);
        munit_assert_int(result, ==, expected[i]);
    }
    
    heap_destroy(heap1);
    heap_destroy(heap2);
    return MUNIT_OK;
}

// Test heap with strings
static MunitResult test_heap_strings(const MunitParameter params[], void* data) {
    MinHeap* heap = heap_create(4, sizeof(char*), compare_strings);
    munit_assert_not_null(heap);
    
    // Create string array
    char* strings[] = {"dog", "cat", "fish", "bird"};
    char* expected[] = {"bird", "cat", "dog", "fish"};  // Alphabetical order
    
    // Insert strings
    for (int i = 0; i < 4; i++) {
        munit_assert_int(heap_insert(heap, &strings[i]), ==, 1);
    }
    
    // Extract and verify order
    for (int i = 0; i < 4; i++) {
        char* result;
        munit_assert_int(heap_extract_min(heap, &result), ==, 1);
        munit_assert_string_equal(result, expected[i]);
    }
    
    heap_destroy(heap);
    return MUNIT_OK;
}

// Test heap with custom struct
static MunitResult test_heap_custom_struct(const MunitParameter params[], void* data) {
    MinHeap* heap = heap_create(3, sizeof(CustomItem), compare_custom_items);
    munit_assert_not_null(heap);
    
    // Create items with priorities
    CustomItem items[] = {
        {3, "low"},
        {1, "high"},
        {2, "medium"}
    };
    
    // Insert items
    for (int i = 0; i < 3; i++) {
        munit_assert_int(heap_insert(heap, &items[i]), ==, 1);
    }
    
    // Extract and verify order based on priority
    CustomItem result;
    
    munit_assert_int(heap_extract_min(heap, &result), ==, 1);
    munit_assert_int(result.priority, ==, 1);
    munit_assert_string_equal(result.data, "high");
    
    munit_assert_int(heap_extract_min(heap, &result), ==, 1);
    munit_assert_int(result.priority, ==, 2);
    munit_assert_string_equal(result.data, "medium");
    
    munit_assert_int(heap_extract_min(heap, &result), ==, 1);
    munit_assert_int(result.priority, ==, 3);
    munit_assert_string_equal(result.data, "low");
    
    heap_destroy(heap);
    return MUNIT_OK;
}

// Define test suite
static MunitTest min_heap_tests[] = {
    {"/create", test_heap_create, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/insert", test_heap_insert, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/extract_min", test_heap_extract_min, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/peek", test_heap_peek, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/merge", test_heap_merge, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/strings", test_heap_strings, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {"/custom_struct", test_heap_custom_struct, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL},
    {NULL, NULL, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL}
};

// Export the test suite
const MunitSuite min_heap_suite = {
    "/min_heap",
    min_heap_tests,
    NULL,
    1,
    MUNIT_SUITE_OPTION_NONE
}; 