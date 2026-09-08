

#ifndef __APP_RTC_H__
#define __APP_RTC_H__

#include <stdint.h>
#include <time.h>

// RTC 동기화 모듈 초기화
void rtc_sync_init(void);

// 현재 동기화된 시각 가져오기 (GetTime)
uint32_t rtc_sync_get_time(void);

// struct tm 형태의 시간 정보 가져오기
void rtc_sync_get_time_tm(struct tm * time_info);


#endif
