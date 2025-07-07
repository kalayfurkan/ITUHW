#ifndef INSERTION_QUEUE_H
#define INSERTION_QUEUE_H
#define MAX_OPERATION 10

#include "process_queue.h"

typedef struct {
    PROCESS_QUEUE queue[MAX_OPERATION];
    int front;
    int rear;
    int size;
} INSERTION_QUEUE;

void initialize_execution_queue(INSERTION_QUEUE *iq);

bool isFull(INSERTION_QUEUE *iq);

bool isEmpty(INSERTION_QUEUE *iq);

PROCESS_QUEUE peek(INSERTION_QUEUE *iq);

void enqueue(INSERTION_QUEUE *iq, PROCESS_QUEUE data);

PROCESS_QUEUE dequeue(INSERTION_QUEUE *iq);

#endif