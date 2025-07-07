#ifndef PROCESS_QUEUE_H
#define PROCESS_QUEUE_H
#define QUEUE_SIZE 5

#include "type_process.h"

typedef struct {
    PROCESS queue[QUEUE_SIZE];
    int front;
    int rear;
    int size;
    int priority;
    int iteration; // Necessary for process additions during execution
} PROCESS_QUEUE;

void initialize_process_queue(PROCESS_QUEUE *pq);

bool isFull(PROCESS_QUEUE *pq);

bool isEmpty(PROCESS_QUEUE *pq);

PROCESS peek(PROCESS_QUEUE *pq);

void enqueue(PROCESS_QUEUE *pq, PROCESS data);

PROCESS dequeue(PROCESS_QUEUE *pq);

#endif