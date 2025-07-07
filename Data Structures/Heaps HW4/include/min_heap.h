/*******************************************************************************
 * Min Heap Implementation
 * 
 * This is a generic min heap data structure that stores elements in an array.
 * The heap maintains the property that each parent node is smaller than or equal
 * to its children. The root element (index 0) is always the minimum.
 * 
 * The heap automatically grows its capacity when full by doubling the size.
 * 
 * Array representation of heap:
 * For an element at index i:
 * - Left child is stored at: 2*i + 1
 * - Right child is stored at: 2*i + 2
 * - Parent is stored at: (i-1) / 2
 * 
 * Time Complexity:
 * - Find minimum: O(1)
 * - Insert element: O(log n)
 * - Remove minimum: O(log n)
 * - Growth: O(n) when capacity is reached
 ******************************************************************************/

#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include <stdlib.h>
#include <string.h>

// MinHeap structure holds the heap data and metadata
// Uses void pointer to store any data type
// Requires a comparison function to determine element ordering
typedef struct {
    void* data;           // Array storing heap elements
    size_t element_size;  // Size of each element in bytes
    size_t capacity;      // Current maximum capacity (grows dynamically)
    size_t size;          // Current number of elements
    int (*compare)(const void*, const void*);  // Returns negative if first arg is smaller
} MinHeap;

// Creates a new heap with initial capacity
// Element_size specifies the size of stored elements in bytes
// Compare function must return negative if first argument is smaller
MinHeap* heap_create(size_t capacity, size_t element_size, 
                    int (*compare)(const void*, const void*));

// Deallocates all memory used by the heap
void heap_destroy(MinHeap* heap);

// Adds new element to heap
// Element is added at the end and bubbled up to maintain heap property
// If heap is full, capacity is doubled automatically
// Returns 1 if successful, 0 if memory allocation fails
int heap_insert(MinHeap* heap, const void* element);

// Removes and returns the minimum element (root)
// Last element is moved to root and bubbled down
// Returns 1 if successful, 0 if heap is empty
int heap_extract_min(MinHeap* heap, void* result);

// Returns the minimum element without removing it
// Returns 1 if successful, 0 if heap is empty
int heap_peek(const MinHeap* heap, void* result);

// Returns current number of elements in heap
size_t heap_size(const MinHeap* heap);

// Merges heap2 into heap1
// Grows capacity of heap1 if needed
// Returns 1 if successful, 0 if memory allocation fails or heaps are incompatible
int heap_merge(MinHeap* heap1, const MinHeap* heap2);

#endif 