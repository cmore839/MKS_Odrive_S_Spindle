#include "globals.h"

float target = 0;
float loop_count = 0;
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int followerrorcount = 0;
int loopiter = 100;

// --- Safety Pack Globals ---
bool drive_disabled = false; // Fault flag

// Current Limits
float continuous_current_limit = 2.0f; // [Amps] Your original M1.current_limit
float peak_current_limit = 7.0f;       // [Amps] Max allowed peak current
unsigned long peak_current_timeout_ms = 2000; // [ms] Max time allowed *above* continuous limit
unsigned long peak_current_start_time_ms = 0; // Internal timer

// Following Error Limits
float max_following_error = 50.0f; // [rad/s or rad] Max velocity or position error
unsigned long max_following_error_timeout_ms = 2000; // [ms] Max time allowed for this error
unsigned long following_error_start_time_ms = 0; // Internal timer

// Temperature Limits
float max_fet_temp = 85.0f; // [Celsius] Max FET/Motor temperature