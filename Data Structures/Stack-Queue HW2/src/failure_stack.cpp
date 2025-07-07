#include "failure_stack.h"

void initialize_failed_stack(FAILURE_STACK *fs){
	fs->top=-1;
};

bool isFull(FAILURE_STACK *fs){
	if(fs->top==MAX_FAILED-1) return true;
	else return false;
};

bool isEmpty(FAILURE_STACK *fs){
	if(fs->top==-1) return true;
	else return false;
};

void push(FAILURE_STACK *fs, PROCESS_QUEUE data){
	if(!isFull(fs)){
		fs->top++;
		fs->stack[fs->top]=data;
	}
};

PROCESS_QUEUE pop(FAILURE_STACK *fs){
	PROCESS_QUEUE willDeleted;
	if(!isEmpty(fs)){
		willDeleted=fs->stack[fs->top];
		fs->top--;
		return willDeleted;
	}
	return willDeleted;
};