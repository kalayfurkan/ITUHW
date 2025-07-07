#include "process_queue.h"
#include "execution_functions.h"
#include "process_manager.h"
#include "failure_stack.h"
#include "insertion_queue.h"
#include <iostream>

void read_process_file(const char *filename, PROCESS_MANAGER *pm){
	FILE* fptr=fopen(filename,"r");
	if(fptr==NULL) return;

	initialize_process_manager(pm);
	char holder[20];

	PROCESS_QUEUE initial_queue;
	initialize_process_queue(&initial_queue);

	bool skipFirst=true;
	while(fgets(holder,sizeof(holder),fptr)){
		if(skipFirst){
			skipFirst=false;
			continue;
		}
		
		int pid,priority,isHead;
		if(sscanf(holder,"%d, %d, %d", &pid,&priority,&isHead) == 3){
			PROCESS newProcess;
			initialize_process(&newProcess,pid,priority);

			if(isHead==0){
				enqueue(&initial_queue,newProcess);
			}
			else if(isHead==1){
				enqueue(&initial_queue,newProcess);
				if(priority==1){
					insert_front(pm,initial_queue);
				}else{
					insert_rear(pm,initial_queue);
				}
				initialize_process_queue(&initial_queue);
			}
		}
	}

	fclose(fptr);
};

void read_insertion_file(const char *filename, INSERTION_QUEUE *eq){
	FILE* fptr=fopen(filename,"r");
	if(fptr==NULL) return;

	char holder[20];
	initialize_execution_queue(eq);

	PROCESS_QUEUE initial_queue;
	initialize_process_queue(&initial_queue);

	bool skipFirst=true;
	while(fgets(holder,sizeof(holder),fptr)){
		if(skipFirst){
			skipFirst=false;
			continue;
		}

		int iteration,pid,priority,isHead;
		if(sscanf(holder,"%d, %d, %d, %d",&iteration, &pid,&priority,&isHead) == 4){
			PROCESS newProcess;
			initialize_process(&newProcess,pid,priority);
			if(isHead==0){
				enqueue(&initial_queue,newProcess);
			}else if(isHead==1){
				enqueue(&initial_queue,newProcess);
				initial_queue.iteration=iteration;
				enqueue(eq,initial_queue);
				initialize_process_queue(&initial_queue);
			}
		}
	}
	fclose(fptr);
};

void execution_loop(PROCESS_MANAGER *pm, INSERTION_QUEUE *eq, FAILURE_STACK *fs) {
	FILE* fptr=fopen("execution_run.txt","w");
	if(fptr==NULL) return;

    int iter = 0;
    initialize_failed_stack(fs);

    while (!isEmpty(pm) || !isEmpty(eq)) {
        while (!isEmpty(&pm->deque[pm->front])) {
       	 	PROCESS_QUEUE *currentProcessQueue = &pm->deque[pm->front];
       	 	PROCESS currentProcess = currentProcessQueue->queue[currentProcessQueue->front];
            if (currentProcess.pid % 8 == 0) {
                push(fs, *currentProcessQueue);
				fprintf(fptr,"%d, %s\n",currentProcess.pid,"f");
				delete_front(pm);
                iter++;
                break;
            } else {
				fprintf(fptr,"%d, %s\n",currentProcess.pid,"s");
                dequeue(currentProcessQueue);
                iter++;
            }
        }
        if (isEmpty(&pm->deque[pm->front])) {
            delete_front(pm);
        }
        if (!isEmpty(eq)) {
            while(peek(eq).iteration <= iter) {
				PROCESS_QUEUE willBeInserted = dequeue(eq);
                if(willBeInserted.queue[willBeInserted.front].priority==1){
					insert_front(pm,willBeInserted);
				}else{
					insert_rear(pm,willBeInserted);
				}
            }
        }
    }

    fclose(fptr);
}
