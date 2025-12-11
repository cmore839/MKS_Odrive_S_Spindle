#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "utilities/stm32pwm/STM32PWMInput.h"
#include "pins.h"
#include "DRV8301.h"
#include "current_sense/hardware_specific/stm32/stm32f4/stm32f4_hal.h"

// --- Loop Timing & Control ---
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
extern float continuous_current_limit;
extern float peak_current_limit;
extern unsigned long peak_current_timeout_ms;
extern unsigned long peak_current_start_time_ms; 
extern float max_following_error;
extern unsigned long max_following_error_timeout_ms;
extern unsigned long following_error_start_time_ms;
extern int max_temp;

// --- DRV8301 Driver ---
extern DRV8301 drv; 

// --- Brake Resistor ---
extern float BRAKE_P_GAIN;
extern float BRAKE_I_GAIN;

// --- VBUS ---
extern int VBUS_DIVIDER_RATIO;

// --- Sensing Parameters ---
extern float SHUNT_RESISTOR;
extern int CSA_GAIN;
extern float VBUS_S;
extern float Temp_M1;
extern PhaseCurrent_s current1;
extern float duty_cycle;

// --- Motor Parameters ---
extern float current_bandwidth;
extern float phase_resistance;
extern float d_phase_inductance;
extern float q_phase_inductance;
extern float kv_rating;

// --- Soft Limit Globals ---
extern bool run_calibration_on_startup;
extern float calibration_velocity;
extern float calibration_current_limit;
extern float calibration_following_error; 
extern float homing_following_error;      // <-- ADD THIS (This will be our HIGH limit for the final move)
extern float calibration_stroke_buffer; // <-- ADD THIS
extern float known_stroke_distance;
extern float soft_limit_positive;
extern float soft_limit_negative;
extern bool soft_limits_enabled;

// --- Temperature Limits ---
extern float low_temp_limit;      // <-- ADD THIS
extern float high_temp_limit;     // <-- ADD THIS
extern float motor_cutoff_temp;   // <-- ADD THIS


#endif // GLOBALS_H