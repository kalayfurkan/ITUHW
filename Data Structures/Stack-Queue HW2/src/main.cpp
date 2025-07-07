#include <iostream>
#include "execution_functions.h"
using namespace std;
int main()
{
    PROCESS_MANAGER pm;
    initialize_process_manager(&pm);
    read_process_file("initial_processes.txt",&pm);
    INSERTION_QUEUE eq;
    initialize_execution_queue(&eq);
    read_insertion_file("arriving_processes.txt",&eq);
    FAILURE_STACK fs;
    initialize_failed_stack(&fs);
    execution_loop(&pm,&eq,&fs); 
    return 0;
}