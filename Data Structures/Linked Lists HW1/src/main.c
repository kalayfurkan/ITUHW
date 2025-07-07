#include "schedule.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
    // Create an initial schedule for testing
    struct Schedule* schedule = CreateSchedule();

    // Reading and writing schedule from/to a file
    const char* inputFile = "exam_schedule_input.txt";
    const char* outputFile = "exam_schedule_output.txt";

    // Uncomment to read schedule from file (assuming the input file exists and is correctly formatted)
    ReadScheduleFromFile(schedule, inputFile);

    AddExamToSchedule(schedule,"Monday",16,18,"BLG102E");
    ClearDay(schedule,"Monday");
    UpdateExam(schedule,"Monday",15,"Tuesday",17,19);
        UpdateExam(schedule,"Wednesday",8,"Friday",10,12);
        RemoveExamFromSchedule(schedule,"Sunday",16);
    RemoveExamFromSchedule(schedule,"Saturday",9);

    // Write schedule to file
    WriteScheduleToFile(schedule, outputFile);

    // Example: Delete the entire schedule
    DeleteSchedule(schedule);

    return 0;
}
