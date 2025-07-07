#ifndef PROCESS_MANAGER_H
#define PROCESS_MANAGER_H
#define MAX_PROCESS 50

#include "process_queue.h"

typedef struct {
    PROCESS_QUEUE deque[MAX_PROCESS];
    int front;
    int rear;
    int size;    
} PROCESS_MANAGER;

void initialize_process_manager(PROCESS_MANAGER *pm);

bool isFull(PROCESS_MANAGER *pm);

bool isEmpty(PROCESS_MANAGER *pm);

void insert_front(PROCESS_MANAGER *pm, PROCESS_QUEUE pq);

void insert_rear(PROCESS_MANAGER *pm, PROCESS_QUEUE pq);

PROCESS_QUEUE delete_front(PROCESS_MANAGER *pm);

PROCESS_QUEUE delete_rear(PROCESS_MANAGER *pm);

#endif