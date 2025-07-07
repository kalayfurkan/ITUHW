#ifndef EXECUTION_FUNCTIONS_H
#define EXECUTION_FUNCTIONS_H

#include "process_manager.h"
#include "insertion_queue.h"
#include "failure_stack.h"

void read_process_file(const char *filename, PROCESS_MANAGER *pm);

void read_insertion_file(const char *filename, INSERTION_QUEUE *eq);

void execution_loop(PROCESS_MANAGER *pm, INSERTION_QUEUE *eq, FAILURE_STACK *fs);

#endif