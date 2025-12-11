#include "globals.h"

// --- DRV8301 Driver ---
DRV8301 drv; 

// --- Loop Timing & Control ---
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
float continuous_current_limit = 2.0f; // [Amps]
float peak_current_limit = 3.0f;       // [Amps] Max allowed peak current
unsigned long peak_current_timeout_ms = 2000; // [ms]
unsigned long peak_current_start_time_ms = 0; // Internal timer
float max_following_error = 50.0f; // [rad/s or rad]
unsigned long max_following_error_timeout_ms = 2000; // [ms]
unsigned long following_error_start_time_ms = 0; // Internal timer
int max_fet_temp = 33; // [Celsius]

// --- Brake Resistor ---
float BRAKE_TARGET_VOLTAGE = 12.5f; // MUST BE GREATER THAN SUPPLY VOLTAGE
float BRAKE_P_GAIN =  0.1f;
float BRAKE_I_GAIN =  0.05f;

// --- VBUS ---
int VBUS_DIVIDER_RATIO = 19;

// --- Sensing Parameters ---
float SHUNT_RESISTOR = 0.0005f; // 0.5 milliohms
int CSA_GAIN = 80;
float VBUS_S = 0;
float Fet_Temp_M1 = 0;
PhaseCurrent_s current1;
float duty_cycle = 0;

// --- Motor Parameters ---
float current_bandwidth = 200; //Hz
float phase_resistance = 3.24*2; // each phase resistance is 3.24 Ohm, total is 6.48 Ohm
float d_phase_inductance = 0.0086;
float q_phase_inductance = 0.00108;
float kv_rating = _NC; // RPM/V