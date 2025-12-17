#include "globals.h"

// --- DRV8301 Driver ---
DRV8301 drv; 

// Important Globals
// --- Motor Parameters ---
float current_bandwidth = 200; //Hz
float phase_resistance = 0.2816; // each phase resistance is 3.24 Ohm, total is 6.48 Ohm
float d_phase_inductance = 0.00037;
float q_phase_inductance = 0.00037;
float kv_rating = _NC; // RPM/V
float continuous_current_limit = 10.0f; // [Amps]
float peak_current_limit = 15.0f;       // [Amps] Max allowed peak current

// --- Safety Pack Parameters ---
float max_following_error = 30.0f; // [rad/s or rad]
int max_temp = 60; // [Celsius]
// --- Soft Limit Parameters ---
// Set 'true' to run the full end-to-end calibration on next boot.
// After, set 'false' and fill in the 'known_stroke_distance' with the value from the serial monitor.
bool run_calibration_on_startup = false; 
float calibration_velocity = 4.0f;      // [rad/s] Low velocity for homing/calibration
float calibration_current_limit = 0.5f; // [Amps] Max current during calibration/homing
float calibration_following_error = 0.1f; // [rad] Max following error during calibration/homing
float homing_following_error = 100.0f;      // [rad] Set HIGH to allow final "go to zero" move
float calibration_stroke_buffer = 0.5f;     // [rad] Buffer to set soft limits inside the hard stops
float known_stroke_distance = 10.0f;     // [rad] SET THIS after calibration, e.g., 20.45

// --- Temperature Limits ---
float low_temp_limit = 80.0f;      // [Celsius] Start scaling down current
float high_temp_limit = 90.0f;     // [Celsius] Scale current to 0
float motor_cutoff_temp = 95.0f;   // [Celsius] Disable motor

// Less Important Globals
// --- Loop Timing & Control ---
unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopiter = 10; // Run control loop every 10 main loops

// --- Safety Pack Globals ---
bool drive_disabled = false; // Fault flag
unsigned long peak_current_timeout_ms = 5000; // [ms]
unsigned long max_following_error_timeout_ms = 5000; // [ms]

// --- Brake Resistor Gains ---
float BRAKE_P_GAIN =  0.1f;
float BRAKE_I_GAIN =  0.05f;

// --- VBUS for MKS ODRIVE-S---
int VBUS_DIVIDER_RATIO = 19;

// --- Sensing Parameters for MKS ODRIVE-S ---
//float SHUNT_RESISTOR = 0.005f; // 5 milliohms CUSTOM
float SHUNT_RESISTOR = 0.0005f; // 0.5 milliohms DEFAULT
int CSA_GAIN = 80; // Current Sense Amplifier Gain THIS MUST MATCH THE HARDWARE SETTING


// --- Internal Limit State ---
float soft_limit_positive = 0.0f;
float soft_limit_negative = 0.0f;
bool soft_limits_enabled = false;

// --- Sensor Readings Mainly for MCU Viewer
float VBUS_S = 0;
float Temp_M1 = 0;
float target = 0;
float loop_count = 0;
int loopcounter = 0;
int followerrorcount = 0;
float duty_cycle = 0;
unsigned long peak_current_start_time_ms = 0; // Internal timer
unsigned long following_error_start_time_ms = 0; // Internal timer
PhaseCurrent_s current1;
