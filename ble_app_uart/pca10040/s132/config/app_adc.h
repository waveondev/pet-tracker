

#ifndef __APP_ADC_H__
#define __APP_ADC_H__

#include "stdint.h"
void saadc_deinit(void);
int16_t adc_get_data(void);
void saadc_init(void);
uint32_t battery_voltage_get(void);
uint32_t battery_percent_get(void);
uint16_t saadc_timer_handler(void * p_context);
#endif
