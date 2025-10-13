#ifndef HARDWARE_UTILS_CURRENT_H
#define HARDWARE_UTILS_CURRENT_H

#include "../common/foc_utils.h"
#include "../common/time_utils.h"

#define SIMPLEFOC_CURRENT_SENSE_INIT_FAILED ((void*)-1)

typedef struct GenericCurrentSenseParams {
  int pins[3];
  float adc_voltage_conv;
} GenericCurrentSenseParams;

// Function declarations
void* _configureADCLowSide(const void *driver_params, const int pinA,const int pinB,const int pinC = NOT_SET);
void* _configureADCLowSide(const void* driver_params, const int pinA, const int pinB, const int pinC, const int vbus_pin, const int temp_pin = NOT_SET);

float _readVbusADCVoltage(const void* cs_params);

void _startADC3PinConversionLowSide();
float _readADCVoltageLowSide(const int pinA, const void* cs_params);
void* _driverSyncLowSide(void* driver_params, void* cs_params);

#endif