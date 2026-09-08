

#ifndef __APP_FLASH_H__
#define __APP_FLASH_H__

#include "stdint.h"
#include "stdbool.h"
#include "ble_gap.h"

// nRF5 SDK 필수 헤더 파일들
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"  // SoftDevice용 fstorage 드라이버 헤더
#include "sdk_errors.h"       // ret_code_t 정의 헤더

#define DEVICE_NAME_SIZE 32


// 플래시 링 버퍼 메모리 주소 및 크기 정의
#define FLASH_START_ADDR    0x40000
#define FLASH_PAGE_SIZE     0x1000   // 4 KB (nRF52 페이지 크기)
#define FLASH_END_ADDR      0xf7000 




#define FLASH_SETTING_ADDR    (FLASH_END_ADDR - 0x1000)


typedef struct {
    uint8_t  device_name[DEVICE_NAME_SIZE];
    uint8_t  peripheral_1[DEVICE_NAME_SIZE];
    uint8_t  peripheral_2[DEVICE_NAME_SIZE];
    uint8_t  peripheral_3[DEVICE_NAME_SIZE];
    uint8_t  peripheral_4[DEVICE_NAME_SIZE];
    uint8_t  beacon_data[DEVICE_NAME_SIZE];

    uint16_t data_collect_sec;

    int16_t  beacon_tx_power;
    uint16_t beacon_adv_interval;

    uint8_t  reserved[40];
    uint16_t crc16;                                // 2B (맨 마지막에 추가)
} tracker_setting_t;

typedef struct{
  ble_gap_addr_t addr;
  uint16_t       reserved;// 2바이트 (데이터 정렬을 맞추기 위한 빈 공간)
  uint16_t crc16;
}white_mac_t;

void tracker_factory(void);
void factory_enable(void);
tracker_setting_t* Tracker_Get_Setting(void);
ble_gap_addr_t* Tracker_Get_WhiteList(void);
ret_code_t flash_erase(uint32_t addr);
ret_code_t flash_write(uint32_t addr, uint8_t* data, uint16_t len);
ret_code_t flash_read(uint32_t addr, uint8_t* data, uint16_t len);
void flash_init(void);

#endif

