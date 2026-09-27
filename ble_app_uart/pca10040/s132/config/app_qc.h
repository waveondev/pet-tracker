

#ifndef __APP_QC_H__
#define __APP_QC_H__

#include "stdint.h"
#include "stdbool.h"
void app_qc_timer_create(void);
void led_timer_start(bool r, bool g, bool b, uint32_t next_time);
void app_qc_timer_create(void);
void app_qc_mode(void);
void error_set_flag(uint32_t flag);

#define FLASH_ERROR   0x00000001UL
#define ADC_ERROR     0x00000002UL
#define MOTION_ERROR  0x00000004UL
#define BLE_ERROR     0x00000008UL

#endif
