#include "LowsideCurrentSense.h"
#include "communication/SimpleFOCDebug.h"
#include "hardware_specific/stm32/stm32_mcu.h"
#include "hardware_specific/stm32/stm32_adc_utils.h"
#include "../BLDCMotor.h" 

// Main constructor
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC, int _pinVbus, float _vbus_gain, int _pinTemp)
  : pinA(_pinA), pinB(_pinB), pinC(_pinC), shunt_resistor(_shunt_resistor), amp_gain(_gain), pinVbus(_pinVbus), vbus_gain(_vbus_gain), pinTemp(_pinTemp)
{
    volts_to_amps_ratio = 1.0f / _shunt_resistor / _gain;
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}

// Legacy constructors
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC)
    : LowsideCurrentSense(_shunt_resistor, _gain, _pinA, _pinB, _pinC, NOT_SET, 1.0f)
{
}
LowsideCurrentSense::LowsideCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC)
    : pinA(_pinA), pinB(_pinB), pinC(_pinC)
{
    volts_to_amps_ratio = 1000.0f / _mVpA;
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}

int LowsideCurrentSense::init(){
    if (driver==nullptr) return 0;
    
    int rank_counter = 0;
    if (_isset(pinA)) rank_counter++;
    if (_isset(pinB)) rank_counter++;
    if (_isset(pinC)) rank_counter++;
    if (_isset(pinVbus)) vbus_rank = rank_counter++;
    if (_isset(pinTemp)) temp_rank = rank_counter;
    
    params = _configureADCLowSide(driver->params, pinA, pinB, pinC, pinVbus, pinTemp);
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;

    void* r = _driverSyncLowSide(driver->params, params);
    if(r == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;
    
    if (motor) {
        original_current_limit = motor->current_limit;
    }
    
    calibrateOffsets();
    initialized = (params!=SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);
    return 1;
}

void LowsideCurrentSense::calibrateOffsets(){
    const int calibration_rounds = 1000;
    offset_ia=0; offset_ib=0; offset_ic=0;
    for (int i = 0; i < calibration_rounds; i++) {
        _startADC3PinConversionLowSide();
        if(_isset(pinA)) offset_ia += (_readADCVoltageLowSide(pinA, params));
        if(_isset(pinB)) offset_ib += (_readADCVoltageLowSide(pinB, params));
        if(_isset(pinC)) offset_ic += (_readADCVoltageLowSide(pinC, params));
        _delay(1);
    }
    if(_isset(pinA)) offset_ia /= calibration_rounds;
    if(_isset(pinB)) offset_ib /= calibration_rounds;
    if(_isset(pinC)) offset_ic /= calibration_rounds;
}

PhaseCurrent_s LowsideCurrentSense::getPhaseCurrents(){
    PhaseCurrent_s current;
    _startADC3PinConversionLowSide();
    current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageLowSide(pinA, params) - offset_ia)*gain_a;
    current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageLowSide(pinB, params) - offset_ib)*gain_b;
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageLowSide(pinC, params) - offset_ic)*gain_c;
    return current;
}

float LowsideCurrentSense::getVbusVoltage() {
  if (vbus_rank == -1) return 0.0f;
  uint32_t raw_adc = HAL_ADCEx_InjectedGetValue(((Stm32CurrentSenseParams*)params)->adc_handle, _getADCInjectedRank(vbus_rank));
  return (raw_adc * ((Stm32CurrentSenseParams*)params)->adc_voltage_conv) * vbus_gain;
}

float LowsideCurrentSense::getTemperature() {
    if (temp_rank == -1) return 0.0f;

    // Read the raw ADC value for the temperature sensor
    uint32_t raw_adc = HAL_ADCEx_InjectedGetValue(((Stm32CurrentSenseParams*)params)->adc_handle, _getADCInjectedRank(temp_rank));

    // Convert the ADC value to voltage
    float voltage = (raw_adc * ((Stm32CurrentSenseParams*)params)->adc_voltage_conv);

    // This formula assumes the NTC is connected to 3.3V and the fixed resistor is connected to ground.
    // R_ntc = R_fixed * (V_in - V_out) / V_out
    float R_ntc = 47000.0f * (3.3f - voltage) / voltage; // <-- UPDATED to 47k fixed resistor

    // Convert resistance to temperature using the Steinhart-Hart equation
    float R0 = 100000.0f; // Resistance at reference temperature (25°C) // <-- UPDATED to 100k NTC
    float T0 = 298.15f;    // Reference temperature in Kelvin (25°C)
    float B = 3950.0f;     // Beta coefficient of the thermistor (B3950)
    
    // Check for division by zero or invalid log input
    if (R_ntc <= 0) {
        // Handle error, e.g., return a sentinel value or the last known good temp
        // Returning a fixed value like -273.15 (absolute zero) might be an option
        return -273.15f; 
    }

    float steinhart = log(R_ntc / R0) / B;
    steinhart += 1.0f / T0;
    
    // Check for division by zero
    if (steinhart == 0) {
         return -273.15f; // Or another error value
    }
    
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f; // Convert from Kelvin to Celsius

    // Debugging output
    // SIMPLEFOC_DEBUG("Temp ADC Raw: ", (int)raw_adc);
    // SIMPLEFOC_DEBUG("Temp Voltage: ", voltage);
    // SIMPLEFOC_DEBUG("Temp R_ntc: ", R_ntc);
    // SIMPLEFOC_DEBUG("Temperature: ", steinhart);

    return steinhart;
}

void LowsideCurrentSense::initBrakeResistorPWM(int pin, float target_voltage, float p_gain, float i_gain) {
    pin_brake_resistor = pin;
    brake_target_voltage = target_voltage;
    brake_p_gain = p_gain;
    brake_i_gain = i_gain;
    if (!_isset(pin_brake_resistor)) return;
    __HAL_RCC_TIM2_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    brake_timer_handle.Instance = TIM2;
    brake_timer_handle.Init.Prescaler = 0;
    brake_timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    brake_timer_handle.Init.Period = 1023;
    brake_timer_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&brake_timer_handle);
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&brake_timer_handle, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&brake_timer_handle, TIM_CHANNEL_4);
}

void LowsideCurrentSense::updateBrakeResistor() {
    if (!_isset(pin_brake_resistor) || brake_target_voltage == 0) return;
    float vbus = getVbusVoltage();
    float duty_cycle = 0;
    if (vbus > brake_target_voltage) {
        float error = vbus - brake_target_voltage;
        duty_cycle = error * brake_p_gain;
        brake_integrator += error * brake_i_gain;
        brake_integrator = _constrain(brake_integrator, 0.0f, 1.0f);
        duty_cycle += brake_integrator;
    } else {
        brake_integrator = 0;
        duty_cycle = 0;
    }
    duty_cycle = _constrain(duty_cycle, 0.0f, 0.95f);
    brake_duty_cycle = duty_cycle;
    __HAL_TIM_SET_COMPARE(&brake_timer_handle, TIM_CHANNEL_4, (uint32_t)(brake_duty_cycle * 1023));
}

void LowsideCurrentSense::checkTemperature() {
    if (!motor) {
        //SIMPLEFOC_DEBUG("Motor not linked, skipping temperature check.");
        return;
    }
    if (motor_cutoff) return;

    float temp = getTemperature();

    if (temp > motor_cutoff_temp) {
        motor->disable();
        motor_cutoff = true;
        //SIMPLEFOC_DEBUG("CUTOFF! Temp exceeded motor_cutoff_temp.");
    } else if (temp > high_temp_limit) {
        motor->current_limit = 0.0f;
        //SIMPLEFOC_DEBUG("WARNING! Temp exceeded high_temp_limit.");
    } else if (temp > low_temp_limit) {
        float scale = 1.0f - (temp - low_temp_limit) / (high_temp_limit - low_temp_limit);
        motor->current_limit = original_current_limit * scale;
        //SIMPLEFOC_DEBUG("INFO: Temp in ramp zone. Current limit scaled.");
    } else {
        motor->current_limit = original_current_limit;
        //SIMPLEFOC_DEBUG("INFO: Temp is normal.");
    }
}

void LowsideCurrentSense::resetTemperatureProtection() {
    if (motor) {
        motor->enable();
    }
    motor_cutoff = false;
    SIMPLEFOC_DEBUG("Temperature protection reset.");
}

void LowsideCurrentSense::linkMotor(BLDCMotor* _motor) {
    motor = _motor;
}