#ifndef STM32_CURRENTSENSE_MCU_DEF
#define STM32_CURRENTSENSE_MCU_DEF

#include "../../hardware_api.h"
#include "../../../common/foc_utils.h"
#include "../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "../../../drivers/hardware_specific/stm32/stm32_timerutils.h"

#if defined(_STM32_DEF_)

// This struct now correctly includes the vbus_pin member.
typedef struct Stm32CurrentSenseParams {
  int pins[3] = {(int)NOT_SET};
  int vbus_pin = (int)NOT_SET; // <-- This was the missing line
  float adc_voltage_conv;
  ADC_HandleTypeDef* adc_handle = NP;
  TIM_HandleTypeDef* timer_handle = NP;
} Stm32CurrentSenseParams;


#endif
#endif