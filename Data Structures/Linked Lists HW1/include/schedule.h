#ifndef SCHEDULE_H
#define SCHEDULE_H

#include "exam.h"

// Define fixed-length arrays for day names and course codes
#define MAX_DAY_NAME_LEN 10
#define MAX_COURSE_CODE_LEN 50

// Day structure definition
struct Day {
    char dayName[MAX_DAY_NAME_LEN]; // C-style string for the day name
    struct Day* nextDay; // Next day in the circular list
    struct Exam* examList; // Head of the exam list for this day
};

// Schedule structure definition
struct Schedule {
    struct Day* head; // Head of the circular linked list of days
};

// Function prototypes for managing the schedule
struct Schedule* CreateSchedule();
int AddExamToSchedule(struct Schedule* schedule, const char* day, int startTime, int endTime, const char* courseCode);
int RemoveExamFromSchedule(struct Schedule* schedule, const char* day, int startTime);
int UpdateExam(struct Schedule* schedule, const char* oldDay, int oldStartTime, const char* newDay, int newStartTime, int newEndTime);
int ClearDay(struct Schedule* schedule, const char* day);
void DeleteSchedule(struct Schedule* schedule);

// File I/O functions
int ReadScheduleFromFile(struct Schedule* schedule, const char* filename);
int WriteScheduleToFile(struct Schedule* schedule, const char* filename);

#endif // SCHEDULE_H
