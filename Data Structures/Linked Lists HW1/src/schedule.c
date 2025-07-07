#include "schedule.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Create a new schedule with 7 days
struct Schedule* CreateSchedule() {
    struct Schedule* newSchedule=(struct Schedule*)malloc(sizeof(struct Schedule));

    char *days[]={"Sunday","Saturday","Friday","Thursday","Wednesday","Tuesday","Monday"};

    struct Day* lastDay=(struct Day*)malloc(sizeof(struct Day));
    strcpy(lastDay->dayName, days[0]);
    lastDay->examList=NULL;
    lastDay->nextDay=NULL;
    struct Day* holder=lastDay;

    for(int i=1;i<7;i++){
        struct Day* newDay=(struct Day*)malloc(sizeof(struct Day));
        strcpy(newDay->dayName, days[i]);
        newDay->examList=NULL;
        newDay->nextDay=holder;
        holder=newDay;
    }
    lastDay->nextDay=holder;

    newSchedule->head=holder;
    printf("Schedule creation complete.\n");
    return newSchedule;
}


int isThereConflict(struct Day* controlDay,int controlStartTime, int controlEndTime){
    if(controlDay->examList==NULL) return 0;
    struct Exam* currentExam = controlDay->examList;

    while(currentExam!=NULL){
        if (controlEndTime > currentExam->startTime && controlStartTime < currentExam->endTime) {
            return currentExam->endTime;
        }
        currentExam=currentExam->next;
    }
    return 0;
}

// Add an exam to a day in the schedule
int AddExamToSchedule(struct Schedule* schedule, const char* day, int startTime, int endTime, const char* courseCode) {
    int wantedDuration=endTime-startTime;
    if(wantedDuration>3 || (startTime>17 || startTime<8) || (endTime>20 || endTime<9)){
        printf("Invalid exam.\n");
        return 3;
    };

    
    struct Day* dayToAdd=schedule->head;
    while(!(strcmp(dayToAdd->dayName,day) == 0)){
        dayToAdd=dayToAdd->nextDay;
    }

    int conflictTime =isThereConflict(dayToAdd,startTime,endTime);
    
    if(conflictTime==0){
        struct Exam *theExam=CreateExam(startTime,endTime,courseCode);

        if(dayToAdd->examList==NULL || dayToAdd->examList->startTime > startTime){
            theExam->next = dayToAdd->examList;
            dayToAdd->examList=theExam;
            printf("%s exam added to %s at time %d to %d without conflict.\n",courseCode,day,startTime,endTime);
            return 0;
        }
        struct Exam *prevExam=dayToAdd->examList;
        while(prevExam->next !=NULL && prevExam->next->startTime < startTime){
            prevExam = prevExam->next;
        }   
        theExam->next = prevExam->next;
        prevExam->next = theExam;
        printf("%s exam added to %s at time %d to %d without conflict.\n", courseCode, day, startTime, endTime);
        return 0;
    } 
    else{
        int oldControl=conflictTime;
        for(int i=0;i<8;i++){
            while(oldControl<=17 && oldControl+wantedDuration<=20){
                conflictTime = isThereConflict(dayToAdd, oldControl, oldControl+wantedDuration);
                if(conflictTime==0){
                    struct Exam* theExam = CreateExam(oldControl, oldControl+wantedDuration, courseCode);

                    if (dayToAdd->examList == NULL || dayToAdd->examList->startTime > oldControl) {
                        theExam->next = dayToAdd->examList;
                        dayToAdd->examList = theExam;
                        printf("%s exam added to %s at time %d to %d due to conflict.\n", courseCode, dayToAdd->dayName,oldControl,oldControl+wantedDuration);
                        return 1;
                    }

                    struct Exam* prevExam = dayToAdd->examList;
                    while (prevExam->next != NULL && prevExam->next->startTime < oldControl) {
                        prevExam = prevExam->next;
                    }
                    theExam->next = prevExam->next;
                    prevExam->next = theExam;
                    printf("%s exam added to %s at time %d to %d due to conflict.\n", courseCode, dayToAdd->dayName,oldControl, oldControl+wantedDuration);
                    return 1;
                }
                oldControl = conflictTime;
            }
            dayToAdd = dayToAdd->nextDay;
            oldControl = 8;
        }
    }
    
    printf("Schedule full. Exam cannot be added.\n");
    return 2;
}
// Remove an exam from a specific day in the schedule
int RemoveExamFromSchedule(struct Schedule* schedule, const char* day, int startTime) {
    struct Day *removeDay=schedule->head;
    while(!(strcmp(removeDay->dayName,day) == 0)){
        removeDay=removeDay->nextDay;
    }
    if(removeDay->examList==NULL){
        printf("Exam could not be found.\n");
        return 1;
    }

    struct Exam* removeExam=removeDay->examList;
    struct Exam* prevExam=NULL;

    while(removeExam!=NULL && removeExam->startTime!=startTime){
        prevExam=removeExam;
        removeExam=removeExam->next;
    }

    if(removeExam==NULL){
        printf("Exam could not be found.\n");
        return 1;
    }

    if(removeExam==removeDay->examList){
        removeDay->examList=removeDay->examList->next;
        free(removeExam);
        printf("Exam removed successfully.\n");
        return 0;
    }else{
        prevExam->next=removeExam->next;
        free(removeExam);
        printf("Exam removed successfully.\n");
        return 0;
    }
}

// Update an exam in the schedule
int UpdateExam(struct Schedule* schedule, const char* oldDay, int oldStartTime, const char* newDay, int newStartTime, int newEndTime) {
    int wantedDuration=newEndTime-newStartTime;
    if(wantedDuration>3 || (newStartTime>17 || newStartTime<8) || (newEndTime>20 || newEndTime<9)){
        printf("Invalid exam.\n");
        return 3;
    };
    struct Day* willBeUpdatedDay=schedule->head;
    while(!(strcmp(willBeUpdatedDay->dayName,oldDay) == 0)){
        willBeUpdatedDay=willBeUpdatedDay->nextDay;
    }

    struct Exam* willBeUpdatedExam=willBeUpdatedDay->examList;
    while(willBeUpdatedExam!=NULL){
        if(willBeUpdatedExam->startTime==oldStartTime){
            break;
        }
        willBeUpdatedExam=willBeUpdatedExam->next;
    }
    if(willBeUpdatedExam==NULL){
        printf("Exam could not be found.\n");
        return 2;
    }

    struct Day* updateDay=schedule->head;
    while(!(strcmp(updateDay->dayName,newDay) == 0)){
        updateDay=updateDay->nextDay;
    }

    if(isThereConflict(updateDay,newStartTime,newEndTime)!=0){
        printf("Update unsuccessful.\n");
        return 1;
    }else{
        AddExamToSchedule(schedule,newDay,newStartTime,newEndTime,willBeUpdatedExam->courseCode);
        RemoveExamFromSchedule(schedule,oldDay,oldStartTime);
    }

   
    printf("Update successful.\n");
    return 0;
}

// Clear all exams from a specific day and relocate them to other days
int ClearDay(struct Schedule* schedule, const char* day) {
    struct Day* dayToClear=schedule->head;

    while(!(strcmp(dayToClear->dayName,day) == 0)){
        dayToClear=dayToClear->nextDay;
    }
    if(dayToClear->examList==NULL){
        printf("%s is already clear.",dayToClear->dayName);
        return 1;
    }

    struct Exam* controlExams=dayToClear->examList;
    char *dayName=dayToClear->dayName;
    int examCount=0;

    while(controlExams!=NULL){
        examCount++;
        controlExams=controlExams->next;
    }

    controlExams=dayToClear->examList;
    int goodTime=8;
    struct Day* nextDays=dayToClear->nextDay;

    while(controlExams!=NULL){
        int duration=controlExams->endTime-controlExams->startTime;

        if(isThereConflict(nextDays,goodTime,goodTime+duration)==0){
            examCount--;
            controlExams=controlExams->next;
            goodTime+=duration;
        }else{
            goodTime++;
        }
        if(goodTime>17 || goodTime+duration>20){
            nextDays=nextDays->nextDay;
            goodTime=8;
        }
        if(strcmp(dayToClear->dayName,nextDays->dayName) == 0 || examCount==0){
            break;
        }
    }

    if(examCount!=0){
        printf("Schedule full. Exams from %s could not be relocated.\n",dayToClear->dayName);
        return 2;
    }

    controlExams=dayToClear->examList;
    nextDays = dayToClear->nextDay;
    goodTime = 8;

    
    while (controlExams != NULL) {
        int duration = controlExams->endTime - controlExams->startTime;

        AddExamToSchedule(schedule, nextDays->dayName, goodTime, goodTime + duration, controlExams->courseCode);
        controlExams = controlExams->next;
        goodTime=goodTime+duration;
        if (goodTime > 17 || goodTime + duration > 20) {
            nextDays = nextDays->nextDay;
            goodTime = 8;
        }
    }

    while (dayToClear->examList != NULL) {
        RemoveExamFromSchedule(schedule, dayToClear->dayName, dayToClear->examList->startTime);
    }
        
    printf("%s is cleared, exams relocated.\n",dayName);
    return 0;
}

// Clear all exams and days from the schedule and deallocate memory
void DeleteSchedule(struct Schedule* schedule) {
    struct Day* removeDay = schedule->head;
    for (int i = 0; i < 7; i++) {
        struct Exam* currentExam = removeDay->examList;
        while (currentExam != NULL) {
            struct Exam* nextExam = currentExam->next;  
            RemoveExamFromSchedule(schedule, removeDay->dayName, currentExam->startTime);
            currentExam = nextExam;
        }
        removeDay = removeDay->nextDay;
    }
    removeDay = schedule->head;
    struct Day* tmp;
    for(int i=0;i<7;i++){
        tmp=removeDay->nextDay;
        free(removeDay);
        removeDay=tmp;
    }
    free(schedule);
    schedule->head=NULL;
}

// Read schedule from file
int ReadScheduleFromFile(struct Schedule* schedule, const char* filename) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        printf("Error.\n");
        return -1;
    }


    char line[20];
    struct Day* theDay = schedule->head;
    while(fgets(line,sizeof(line),file)){
        if(strstr(line,"day")!=NULL){
            if(strstr(line,"Monday")!=NULL){
                continue;
            }else{
                theDay=theDay->nextDay;
                continue;
            }
        }else{
            int startTime,endTime;
            char courseCode[50];
            if (sscanf(line, "%d %d %s", &startTime, &endTime, courseCode) == 3) {
                AddExamToSchedule(schedule,theDay->dayName,startTime,endTime,courseCode);
            }
        }
    }
    fclose(file);
    return 0;
}

// Write schedule to file
int WriteScheduleToFile(struct Schedule* schedule, const char* filename) {
     FILE* file = fopen(filename, "w");

    if (file == NULL) {
        printf("Error.\n");
        return -1;
    }
    
    struct Day* theDay = schedule->head;
    struct Day* startDay = schedule->head;
    while (theDay != NULL) {
        fprintf(file, "%s\n", theDay->dayName);
        struct Exam* exam = theDay->examList;
        if(exam==NULL){
            fprintf(file, "(No exams scheduled)\n");
        }
        while (exam != NULL) {
            fprintf(file, "%d %d %s\n", exam->startTime, exam->endTime, exam->courseCode);
            exam = exam->next;
        }
        theDay = theDay->nextDay;
        if(startDay==theDay) break;
        fprintf(file, "\n");
    }

    fclose(file);
    printf("Schedule successfully written to file.\n");
    return 0;
}