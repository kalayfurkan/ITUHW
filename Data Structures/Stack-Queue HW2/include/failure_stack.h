#ifndef FAILURE_STACK_H
#define FAILURE_STACK_H
#define MAX_FAILED 10

#include "process_queue.h"

typedef struct {
    PROCESS_QUEUE stack[MAX_FAILED];
    int top;
} FAILURE_STACK;

void initialize_failed_stack(FAILURE_STACK *fs);

bool isFull(FAILURE_STACK *fs);

bool isEmpty(FAILURE_STACK *fs);

void push(FAILURE_STACK *fs, PROCESS_QUEUE data);

PROCESS_QUEUE pop(FAILURE_STACK *fs);

#endif