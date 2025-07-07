#include <stdlib.h>
#include "../include/scheduler.h"


int compare(const void* process1, const void* process2) {
    const Process* p1 = (const Process*)process1;
    const Process* p2 = (const Process*)process2;
    if (p1->vruntime < p2->vruntime) return -1;
    else if (p1->vruntime > p2->vruntime) return 1;
    else return 0;
    
}

Scheduler* create_scheduler(int capacity) {
    Scheduler* scheduler = (Scheduler*)malloc(sizeof(Scheduler));
    if(scheduler==NULL) return NULL;

    scheduler->process_queue = heap_create(capacity, sizeof(Process), compare);
    scheduler->current_process = NULL;
    scheduler->time_slice = 1;

    return scheduler;
}



void destroy_scheduler(Scheduler* scheduler) {
    if (scheduler==NULL) return;

    heap_destroy(scheduler->process_queue);
    free(scheduler->current_process);
    scheduler->current_process=NULL;
    free(scheduler->process_queue);
    free(scheduler);
}

void schedule_process(Scheduler* scheduler, Process process) {
    heap_insert(scheduler->process_queue,&process);
}


Process* get_next_process(Scheduler* scheduler) {
    if (heap_size(scheduler->process_queue) == 0) return NULL;

    if(scheduler->current_process!=NULL){
        scheduler->current_process->is_running=false;
        heap_insert(scheduler->process_queue,scheduler->current_process);
        free(scheduler->current_process);
        scheduler->current_process = NULL;
    }
    
    Process *p=(Process*)malloc(sizeof(Process));
    heap_extract_min(scheduler->process_queue,p);
    scheduler->current_process=p;
    p->is_running=true;
    return p;
}


void tick(Scheduler* scheduler) {
    if (scheduler->current_process==NULL)return;
    update_vruntime(scheduler->current_process, scheduler->time_slice);
}