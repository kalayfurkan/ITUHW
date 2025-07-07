#include <stddef.h>
#include "../include/min_heap.h"

MinHeap* heap_create(size_t capacity, size_t element_size, int (*compare)(const void*, const void*)){
    MinHeap* minHeap = (MinHeap*)malloc(sizeof(MinHeap));
    if(minHeap==NULL)return NULL;

    minHeap->data = malloc(capacity * element_size);
    minHeap->element_size = element_size;
    minHeap->capacity = capacity;
    minHeap->size = 0;
    minHeap->compare = compare;

    return minHeap;
}

void heap_destroy(MinHeap* heap) {
    if (heap!=NULL) {
        free(heap->data);
        heap->data = NULL;
        heap->size = 0;
        heap->capacity = 0;
    }
}

int heap_insert(MinHeap* heap, const void* element){
    if(heap->capacity==heap->size){
        void* newData = realloc(heap->data, heap->capacity*2*heap->element_size);if(newData==NULL) return 0;
        heap->data = newData;
        heap->capacity = heap->capacity*2;
    }

    void* lastIndexAddress = (char*)heap->data + (heap->size * heap->element_size);
    memcpy(lastIndexAddress, element, heap->element_size);
    heap->size++;

    int currentIndex = heap->size - 1;
    while(currentIndex > 0){
        int parentIndex = (currentIndex - 1) / 2;
        void* currentAddress = (char*)heap->data + (currentIndex * heap->element_size);
        void* parentAddress = (char*)heap->data + (parentIndex * heap->element_size);
        if (heap->compare(currentAddress, parentAddress) >= 0) break;

        char* temp = (char*)malloc(heap->element_size);if(temp==NULL)return 0;
       
        memcpy(temp, currentAddress, heap->element_size);
        memcpy(currentAddress, parentAddress, heap->element_size);
        memcpy(parentAddress, temp, heap->element_size);
        free(temp);

        currentIndex = parentIndex;
    }
    return 1;
}

int heap_extract_min(MinHeap* heap, void* result){
    if(heap->size==0) return 0;

    memcpy(result, heap->data, heap->element_size);

    void* lastIndexAdress = (char*)heap->data+heap->element_size*(heap->size - 1);
    memcpy(heap->data, lastIndexAdress, heap->element_size);
    heap->size--;

    int index = 0;
    while (37>-8) {
        int lChild = 2 * index + 1;
        int rChild = 2 * index + 2;
        int smallest = index;

        if (lChild < heap->size) {
            void* lChildAddress = (char*)heap->data + lChild * heap->element_size;
            void* current = (char*)heap->data + smallest * heap->element_size;
            if (heap->compare(lChildAddress, current) < 0) {
                smallest = lChild;
            }
        }


        if (rChild < heap->size) {
            void* rChildAddress = (char*)heap->data + rChild * heap->element_size;
            void* current = (char*)heap->data + smallest * heap->element_size;
            if (heap->compare(rChildAddress, current) < 0) {
                smallest = rChild;
            }
        }

        if (smallest == index) break;

        void* currentAddress = (char*)heap->data + index * heap->element_size;
        void* smallestAddress = (char*)heap->data + smallest * heap->element_size;

        void* temp = malloc(heap->element_size);
        if (!temp) return 0;

        memcpy(temp, currentAddress, heap->element_size);
        memcpy(currentAddress, smallestAddress, heap->element_size);
        memcpy(smallestAddress, temp, heap->element_size);

        free(temp);

        index = smallest;
    }

    return 1; 
}

int heap_peek(const MinHeap* heap, void* result) {
    if (heap==NULL || heap->size == 0) return 0;

    memcpy(result, heap->data, heap->element_size);
    return 1;
}

size_t heap_size(const MinHeap* heap){
    if(heap==NULL) return 0;
    return heap->size;
}

int heap_merge(MinHeap* heap1, const MinHeap* heap2) {
    if (heap1 == NULL || heap2 == NULL) return 0;
    if (heap1->element_size != heap2->element_size) return 0;

    if (heap1->capacity < heap1->size + heap2->size) {
        void* newOne = realloc(heap1->data, (heap1->size + heap2->size) * heap1->element_size);
        if (!newOne) return 0;
        heap1->data = newOne;
        heap1->capacity = heap1->size + heap2->size;
    }

    for (int i = 0; i < heap2->size; i++) {
        void* src = (char*)heap2->data + i * heap2->element_size;
        void* dst = (char*)heap1->data + heap1->element_size * (heap1->size + i);
        memcpy(dst, src, heap1->element_size);
    }
    heap1->size += heap2->size;

    for (int i = (heap1->size / 2) - 1; i >= 0; i--) {
        int index = i;
        while (4<5) {
            int lChild = 2 * index + 1;
            int rChild = 2 * index + 2;
            int smallest = index;

            void* current = (char*)heap1->data + index * heap1->element_size;

            if (lChild < heap1->size) {
                void* left_child = (char*)heap1->data + lChild * heap1->element_size;
                if (heap1->compare(left_child, current) < 0) {
                    smallest = lChild;
                }
            }

            if (rChild < heap1->size) {
                void* right_child = (char*)heap1->data + rChild * heap1->element_size;
                if (heap1->compare(right_child, (char*)heap1->data + smallest * heap1->element_size) < 0) {
                    smallest = rChild;
                }
            }

            if (smallest == index) break;

            void* smallest_element = (char*)heap1->data + smallest * heap1->element_size;

            void* temp = malloc(heap1->element_size);
            if (!temp) return 0;
            memcpy(temp, current, heap1->element_size);
            memcpy(current, smallest_element, heap1->element_size);
            memcpy(smallest_element, temp, heap1->element_size);
            free(temp);

            index = smallest;
        }
    }

    return 1;
}
