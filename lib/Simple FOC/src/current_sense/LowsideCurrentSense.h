#ifndef LOWSIDE_CS_LIB_H
#define LOWSIDE_CS_LIB_H

#include "Arduino.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"
#include "common/defaults.h"
#include "common/base_classes/CurrentSense.h"
#include "hardware_api.h"
#include "stm32f4xx_hal.h"

class BLDCMotor; // Forward declaration

class LowsideCurrentSense: public CurrentSense{
  public:
    // Main constructor
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC, int pinVbus, float vbus_gain, int pinTemp = NOT_SET);

    // Legacy constructors
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);
    LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = _NC);

    // Core functions
    int init() override;
    PhaseCurrent_s getPhaseCurrents() override;
    float getVbusVoltage();
    float getTemperature();

    // Feature initialization
    void initBrakeResistorPWM(int pin, float target_voltage, float p_gain, float i_gain);

    // Update functions (to be called in loop)
    void updateBrakeResistor();
    void checkTemperature();

    // Public members for monitoring
    float brake_duty_cycle = 0.0f;

    // Temperature protection settings
    // These will be set by the main app
    float low_temp_limit = 99.0f;
    float high_temp_limit = 99.1f;
    float motor_cutoff_temp = 60.0f;

    // Public function to reset temperature protection
    void resetTemperatureProtection();
    void linkMotorAndLimits(BLDCMotor* _motor, float* _continuous_current_limit_ptr);
    void setTemperatureLimits(float _low_temp, float _high_temp, float _cutoff_temp); // <-- ADD THIS LINE

  private:
    void calibrateOffsets();

    // Member variables
    int pinA, pinB, pinC;
    float shunt_resistor, amp_gain, volts_to_amps_ratio;

    int pinVbus = NOT_SET, vbus_rank = -1;
    float vbus_gain = 1.0f;

    int pinTemp = NOT_SET, temp_rank = -1;

    int pin_brake_resistor = NOT_SET;
    TIM_HandleTypeDef brake_timer_handle;
    float brake_target_voltage = 0, brake_p_gain = 0, brake_i_gain = 0, brake_integrator = 0;

    // Temperature protection variables
    float original_current_limit = 0.0f; // This will store the PEAK limit
    float original_continuous_current_limit = 0.0f; // <-- ADD THIS
    bool motor_cutoff = false;
    BLDCMotor* motor = nullptr;
    float* continuous_current_limit_ptr = nullptr; // <-- ADD THIS
};

#endif