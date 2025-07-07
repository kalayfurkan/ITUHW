#ifndef SCHEDULE_TESTS_H
#define SCHEDULE_TESTS_H

#include "munit.h"
#include "schedule.h"
#include <stdlib.h>

// Test for creating and validating the schedule days
static MunitResult test_create_schedule(const MunitParameter params[], void* data) {
    // Create the schedule
    Schedule* schedule = CreateSchedule();
    
    // Assert the schedule and head are not null
    munit_assert_not_null(schedule);  // Check that schedule is created
    munit_assert_not_null(schedule->head);  // Check that head of the schedule is initialized

    // Expected days of the week
    const char* expectedDays[] = {"Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
    
    // Traverse the circular schedule and check that each day matches
    Day* currentDay = schedule->head;
    for (int i = 0; i < 7; i++) {
        munit_assert_not_null(currentDay);  // Ensure the current day is not null
        munit_assert_string_equal(currentDay->dayName, expectedDays[i]);  // Compare the day name to the expected day
        
        currentDay = currentDay->nextDay;  // Move to the next day
    }
    
    // After traversing 7 days, we should be back to the head of the schedule (Monday)
    munit_assert_ptr_equal(currentDay, schedule->head);  // Ensure circular structure
    
    // Clean up the schedule
    DeleteSchedule(schedule);
    
    return MUNIT_OK;
}


// Test for adding an exam to the schedule
static MunitResult test_add_exam(const MunitParameter params[], void* data) {
    // Create the schedule
    Schedule* schedule = CreateSchedule();
    
    // Add an exam to Monday
    int result = AddExamToSchedule(schedule, "Monday", 8, 10, "BLG113E");
    
    // Check if the exam is added successfully
    munit_assert_int(result, ==, 0);  // Ensure the function returned success
    munit_assert_not_null(schedule->head->examList);  // Ensure the exam list is not null

    // Check that the exam details are correct
    Exam* exam = schedule->head->examList;  // Get the first exam in Monday's exam list
    munit_assert_int(exam->startTime, ==, 8);  // Check the exam's start time
    munit_assert_int(exam->endTime, ==, 10);  // Check the exam's end time
    munit_assert_string_equal(exam->courseCode, "BLG113E");  // Check the course code

    // Clean up the schedule
    DeleteSchedule(schedule);
    
    return MUNIT_OK;
}


// Test for removing an exam from the schedule
static MunitResult test_remove_exam(const MunitParameter params[], void* data) {
    Schedule* schedule = CreateSchedule();
    AddExamToSchedule(schedule, "Monday", 9, 11, "BLG113E");

    int result = RemoveExamFromSchedule(schedule, "Monday", 9);
    munit_assert_int(result, ==, 0);  // Check if the exam is removed successfully
    munit_assert_null(schedule->head->examList);  // Ensure exam is removed

    DeleteSchedule(schedule);
    return MUNIT_OK;
}

// Test for updating an exam in the schedule
static MunitResult test_update_exam(const MunitParameter params[], void* data) {
    // Create the schedule
    Schedule* schedule = CreateSchedule();
    
    // Add an exam to Monday
    AddExamToSchedule(schedule, "Monday", 9, 11, "BLG113E");

    // Update the exam from Monday to Tuesday
    int result = UpdateExam(schedule, "Monday", 9, "Tuesday", 12, 14);
    
    // Check if the exam is updated successfully
    munit_assert_int(result, ==, 0);  // Ensure the update operation returned success

    // Ensure the exam is removed from Monday
    munit_assert_null(schedule->head->examList);

    // Ensure the exam is added to Tuesday
    Day* tuesday = schedule->head->nextDay;  // Get the next day (Tuesday)
    munit_assert_not_null(tuesday->examList);  // Ensure Tuesday has an exam

    // Verify that the exam on Tuesday has the correct values
    Exam* exam = tuesday->examList;
    munit_assert_int(exam->startTime, ==, 12);  // Check the exam's start time
    munit_assert_int(exam->endTime, ==, 14);  // Check the exam's end time
    munit_assert_string_equal(exam->courseCode, "BLG113E");  // Check the course code

    // Clean up the schedule
    DeleteSchedule(schedule);
    
    return MUNIT_OK;
}


// Test for clearing a day in the schedule and moving exams to the next day
static MunitResult test_clear_day(const MunitParameter params[], void* data) {
    // Create the schedule
    Schedule* schedule = CreateSchedule();

    // Add two back-to-back exams on Monday
    AddExamToSchedule(schedule, "Monday", 9, 11, "BLG113E");
    AddExamToSchedule(schedule, "Monday", 11, 12, "BLG212E");
    AddExamToSchedule(schedule, "Monday", 12, 14, "BLG102E");

    // Add one exam in the middle of Tuesday
    AddExamToSchedule(schedule, "Tuesday", 11, 14, "BLG223E");

    // Clear Monday (exams should be relocated to Tuesday)
    ClearDay(schedule, "Monday");

    // Ensure Monday is cleared
    munit_assert_null(schedule->head->examList);  // Ensure Monday's exam list is cleared

    // Now check Tuesday's exam list
    Exam* currentExam = schedule->head->nextDay->examList;  // Move to Tuesday's exam list

    // Check that the first exam is the one relocated from Monday
    munit_assert_not_null(currentExam);
    munit_assert_string_equal(currentExam->courseCode, "BLG113E");
    munit_assert_int(currentExam->startTime, ==, 8);
    munit_assert_int(currentExam->endTime, ==, 10);

    // Move to the next exam, should be the second one from Monday
    currentExam = currentExam->next;
    munit_assert_not_null(currentExam);
    munit_assert_string_equal(currentExam->courseCode, "BLG212E");
    munit_assert_int(currentExam->startTime, ==, 10);
    munit_assert_int(currentExam->endTime, ==, 11);

    // Move to the third exam, which should be the original Tuesday exam
    currentExam = currentExam->next;
    munit_assert_not_null(currentExam);
    munit_assert_string_equal(currentExam->courseCode, "BLG223E");
    munit_assert_int(currentExam->startTime, ==, 11);
    munit_assert_int(currentExam->endTime, ==, 14);

    // Move to the third exam from Monday, which should come after Tuesday's exam
    currentExam = currentExam->next;
    munit_assert_not_null(currentExam);
    munit_assert_string_equal(currentExam->courseCode, "BLG102E");
    munit_assert_int(currentExam->startTime, ==, 14);
    munit_assert_int(currentExam->endTime, ==, 16);

    // Ensure there are no more exams after this
    munit_assert_null(currentExam->next);

    // Clean up the schedule
    DeleteSchedule(schedule);

    return MUNIT_OK;
}


// Test for clearing the entire schedule
static MunitResult test_clear_schedule(const MunitParameter params[], void* data) {
    // Create the schedule and add exams
    Schedule* schedule = CreateSchedule();
    AddExamToSchedule(schedule, "Monday", 9, 11, "BLG113E");
    AddExamToSchedule(schedule, "Tuesday", 10, 12, "BLG223E");

    // Call the DeleteSchedule function to remove all exams and deallocate memory
    DeleteSchedule(schedule);

    // Verify that the head of the schedule is null, which means all days are cleared
    munit_assert_null(schedule->head);  // Ensure the entire schedule is null (no days remain)

    return MUNIT_OK;
}


// Define the array of tests
static MunitTest schedule_tests[] = {
    { (char*) "/create_schedule", test_create_schedule, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { (char*) "/add_exam", test_add_exam, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { (char*) "/remove_exam", test_remove_exam, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { (char*) "/update_exam", test_update_exam, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { (char*) "/clear_day", test_clear_day, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { (char*) "/clear_schedule", test_clear_schedule, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL },
    { NULL, NULL, NULL, NULL, MUNIT_TEST_OPTION_NONE, NULL }
};

// Define the test suite
const MunitSuite schedule_test_suite = {
    (char*) "/schedule_tests",  // Name of the suite
    schedule_tests,  // Pointer to test array
    NULL,  // Sub-suites, set to NULL if there are none
    1,  // Number of iterations to run the tests
    MUNIT_SUITE_OPTION_NONE  // Flags for the suite (usually set to NONE)
};

#endif // SCHEDULE_TESTS_H
