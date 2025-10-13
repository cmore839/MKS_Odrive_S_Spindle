#include "stm32f4_hal.h"

#if defined(STM32F4xx)

#include "../../../../communication/SimpleFOCDebug.h"

ADC_HandleTypeDef hadc;

int _adc_init(Stm32CurrentSenseParams* cs_params, const STM32DriverParams* driver_params)
{
  ADC_InjectionConfTypeDef sConfigInjected;
  int all_pins[5] = {cs_params->pins[0], cs_params->pins[1], cs_params->pins[2], cs_params->vbus_pin, cs_params->temp_pin};
  hadc.Instance = _findBestADCForPins(5, all_pins);

  if(hadc.Instance == ADC1) __HAL_RCC_ADC1_CLK_ENABLE();
  #ifdef ADC2
  else if(hadc.Instance == ADC2) __HAL_RCC_ADC2_CLK_ENABLE();
  #endif
  #ifdef ADC3
  else if(hadc.Instance == ADC3) __HAL_RCC_ADC3_CLK_ENABLE();
  #endif
  else { return -1; }

  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = ENABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if ( HAL_ADC_Init(&hadc) != HAL_OK){ return -1; }

  sConfigInjected.InjectedNbrOfConversion = 0;
  if (_isset(cs_params->pins[0])) sConfigInjected.InjectedNbrOfConversion++;
  if (_isset(cs_params->pins[1])) sConfigInjected.InjectedNbrOfConversion++;
  if (_isset(cs_params->pins[2])) sConfigInjected.InjectedNbrOfConversion++;
  if (_isset(cs_params->vbus_pin)) sConfigInjected.InjectedNbrOfConversion++;
  if (_isset(cs_params->temp_pin)) sConfigInjected.InjectedNbrOfConversion++;

  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  
  for(int i=0; i < 6; i++){
    TIM_HandleTypeDef* timer_handle = driver_params->timers_handle[i];
    if (timer_handle == NP) continue;
    uint32_t trigger_flag = _timerToInjectedTRGO(timer_handle);
    if(trigger_flag != _TRGO_NOT_AVAILABLE) {
        if(!((timer_handle->Instance->CR2 & LL_TIM_TRGO_ENABLE) || (timer_handle->Instance->CR2 & LL_TIM_TRGO_UPDATE))) {
            cs_params->timer_handle = timer_handle;
            sConfigInjected.ExternalTrigInjecConv = trigger_flag;
            break;
        }
    }
  }
  if( cs_params->timer_handle == NP ){ return -1; }

  uint8_t channel_no = 0;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  for(int i=0; i<3; i++){
    if (!_isset(cs_params->pins[i])) continue;
    sConfigInjected.InjectedRank = _getADCInjectedRank(channel_no++);
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->pins[i]), hadc.Instance);
    if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK) return -1;
  }
  
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (_isset(cs_params->vbus_pin)) {
    sConfigInjected.InjectedRank = _getADCInjectedRank(channel_no++);
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->vbus_pin), hadc.Instance);
    if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK) return -1;
  }
  
  if (_isset(cs_params->temp_pin)) {
    sConfigInjected.InjectedRank = _getADCInjectedRank(channel_no++);
    sConfigInjected.InjectedChannel = _getADCChannel(analogInputToPinName(cs_params->temp_pin), hadc.Instance);
    if (HAL_ADCEx_InjectedConfigChannel(&hadc, &sConfigInjected) != HAL_OK) return -1;
  }

  cs_params->adc_handle = &hadc;
  return 0;
}

int _adc_gpio_init(Stm32CurrentSenseParams* cs_params, const int pinA, const int pinB, const int pinC)
{
    int pins[3] = {pinA, pinB, pinC};
    for(int i=0; i<3; i++){
        if(_isset(pins[i])){
            pinmap_pinout(analogInputToPinName(pins[i]), PinMap_ADC);
            cs_params->pins[i] = pins[i];
        }
    }
    if (_isset(cs_params->vbus_pin)) {
        pinmap_pinout(analogInputToPinName(cs_params->vbus_pin), PinMap_ADC);
    }
    if (_isset(cs_params->temp_pin)) {
        pinmap_pinout(analogInputToPinName(cs_params->temp_pin), PinMap_ADC);
    }
    return 0;
}


extern "C" {
  void ADC_IRQHandler(void)
  {
      HAL_ADC_IRQHandler(&hadc);
  }
}

#endif