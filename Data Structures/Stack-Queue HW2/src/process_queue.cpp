#include <iostream>
#include <stdlib.h>
#include "process_queue.h"

void initialize_process_queue(PROCESS_QUEUE *pq){
	pq->front=0;
	pq->rear=-1;
	pq->size=0;
	pq->priority=0;
	pq->iteration=0;
};

bool isFull(PROCESS_QUEUE *pq){
	if(pq->size==QUEUE_SIZE) return true;
	else return false;
};

bool isEmpty(PROCESS_QUEUE *pq){
	if(pq->size==0) return true;
	else return false;
};

PROCESS peek(PROCESS_QUEUE *pq){
	PROCESS empty;
	if(!isEmpty(pq)){
		return pq->queue[pq->front];
	}
	else return empty;
}

void enqueue(PROCESS_QUEUE *pq, PROCESS data){
	if(!isFull(pq)){
		pq->rear = (pq->rear + 1) % QUEUE_SIZE;
        pq->queue[pq->rear] = data;
        pq->size++;
	}
};

PROCESS dequeue(PROCESS_QUEUE *pq){
	if(!isEmpty(pq)){
		PROCESS dequeued = pq->queue[pq->front];
        pq->front = (pq->front + 1) % QUEUE_SIZE;
        pq->size--;
        return dequeued;
	}
	PROCESS empty;
	return empty;
};
