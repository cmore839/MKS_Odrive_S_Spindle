#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h> // Added for bool and unsigned long types

extern float target;
extern float loop_count;
extern unsigned long start;
extern unsigned long finish;
extern unsigned long looptime;
extern int loopcounter;
extern int followerrorcount;
extern int loopiter;

// --- Safety Pack Globals ---
extern bool drive_disabled;

// Current Limits
extern float continuous_current_limit;
extern float peak_current_limit;
extern unsigned long peak_current_timeout_ms;
extern unsigned long peak_current_start_time_ms; // Internal timer

// Following Error Limits
extern float max_following_error;
extern unsigned long max_following_error_timeout_ms;
extern unsigned long following_error_start_time_ms; // Internal timer

// Temperature Limits
extern float max_fet_temp;


#endif // GLOBALS_H