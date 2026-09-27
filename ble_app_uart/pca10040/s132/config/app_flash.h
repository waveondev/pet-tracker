

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

#include "nrf_sdm.h"

// ==============================================================================
// 1. nRF52840 (S140 SoftDevice 사용 시)
// ==============================================================================
#if defined(NRF52840_XXAA) || defined(SOFTDEVICE_S140)
    #define FLASH_PAGE_SIZE       0x1000   // Page Size: 4 KB

    
    // 링 버퍼 매핑 (nRF52840의 넉넉한 공간 활용)
    #define FLASH_START_ADDR      0x40000  // 링 버퍼 시작 주소 (256 KB 지점)
    #define FLASH_END_ADDR        0xF7000  // 링 버퍼 끝 주소 (FDS/Bootloader 이전까지)

#elif defined(NRF52832_XXAA) || defined(SOFTDEVICE_S132)


    #define FLASH_PAGE_SIZE       0x1000   // Page Size: 4 KB

    
    // 링 버퍼 매핑 (512KB 칩 용량에 맞춰 축소 조정)
    // 0x26000부터 App 코드가 들어가므로 0x40000~0x78000 영역을 사용
    #define FLASH_START_ADDR      0x40000  // 링 버퍼 시작 주소
    #define FLASH_END_ADDR        0x77000  // 링 버퍼 끝 주소 (Bootloader/FDS 시작 전 0x78000 지점)

#else
    #error "지원하지 않는 Target Board/SoftDevice 설정입니다."
#endif




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

void flash_setting_save(void);
void tracker_factory(void);
void factory_enable(void);
tracker_setting_t* Tracker_Get_Setting(void);
ble_gap_addr_t* Tracker_Get_WhiteList(void);
ret_code_t flash_erase(uint32_t addr);
ret_code_t flash_write(uint32_t addr, uint8_t* data, uint16_t len);
ret_code_t flash_read(uint32_t addr, uint8_t* data, uint16_t len);
uint32_t flash_init(void);

#endif

