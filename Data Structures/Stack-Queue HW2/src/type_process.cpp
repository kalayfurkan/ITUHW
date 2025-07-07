#include <iostream>
#include "type_process.h"

void initialize_process(PROCESS *p, int pid, int priority){
	p->pid=pid;
	p->priority=priority;
};