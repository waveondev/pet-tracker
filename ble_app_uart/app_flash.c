#include "app_flash.h"
#include "nordic_common.h"
#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_sensor_flash.h"
#include <stdlib.h>
#include "app_qc.h"

#include "nrf_fstorage_sd.h"
uint32_t BLE_Send_byte(uint8_t* data, uint16_t len);
/**
 * @brief fstorage 비동기 이벤트 핸들러 인터럽트
 */
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Flash operation failed. Evt ID: %d, Result: %d", p_evt->id, p_evt->result);
        error_set_flag(FLASH_ERROR);
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
             NRF_LOG_INFO("Flash write success.");
          break;
        case NRF_FSTORAGE_EVT_ERASE_RESULT:
             NRF_LOG_INFO("Flash erase success.");
          break;
        default:
          break;
    }
}

/**
 * @brief fstorage 인스턴스 공식 선언 (이 매크로가 내부 구조체를 정의함)
 */
NRF_FSTORAGE_DEF(nrf_fstorage_t my_fstorage) = {
    .evt_handler = fstorage_evt_handler, // 이벤트 콜백 등록
    .start_addr  = FLASH_START_ADDR,     // fstorage 가 다룰 구역 시작 주소
    .end_addr    = FLASH_END_ADDR,       // fstorage 가 다룰 구역 끝 주소
};


static tracker_setting_t tracker_setting = 
{
  .device_name = "T100_Tracker",
  .peripheral_1 = "F100",
  .peripheral_2 = "W100",
  .peripheral_3 = "C100",
  .peripheral_4 = "P100",
  .beacon_data = "T100",
  .data_collect_sec = 60,

  .beacon_tx_power = -20,
  .beacon_adv_interval = 500
};

white_mac_t disconnet_mac;


/**
 * @brief CRC-16-CCITT 기본 계산 함수
 */
uint16_t crc16_ccitt_compute(uint8_t const * p_data, uint32_t size)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < size; i++)
    {
        crc ^= (uint16_t)p_data[i] << 8;
        for (uint32_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

tracker_setting_t* Tracker_Get_Setting(void)
{
   return &tracker_setting;
} 

#include "nrf_soc.h"
static void flash_wait_idle(void)
{
    while (nrf_fstorage_is_busy(&my_fstorage))
    {
        sd_app_evt_wait();
    }
}

ret_code_t flash_erase(uint32_t addr)
{
    ret_code_t rc = NRF_SUCCESS;
// 0x1100 & 0xFFFFF000 = 0x1000이 됩니다.
    uint32_t aligned_addr = addr & ~(FLASH_PAGE_SIZE - 1); 

    // NRF_LOG를 쓰신다면 정렬된 주소를 확인해볼 수 있게 로그를 찍어두면 좋습니다.
    if (aligned_addr != addr)
    {
        NRF_LOG_DEBUG("Address 0x%08X is aligned to 0x%08X for erasure", addr, aligned_addr);
    }

    // 데이터를 쓰기 전 4KB 페이지를 미리 지웁니다. (정렬된 주소 사용)
    rc = nrf_fstorage_erase(&my_fstorage, aligned_addr, 1, NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Flash erase failed: %d", rc);     
    }
    flash_wait_idle();
    return rc;
}

ret_code_t flash_write(uint32_t addr, uint8_t* data, uint16_t len)
{
    ret_code_t rc = NRF_SUCCESS;

    if ((addr % FLASH_PAGE_SIZE) == 0)
    {
        NRF_LOG_INFO("Page boundary detected. Erasing 4KB at: 0x%08x", addr);
        NRF_LOG_INFO("FS start = 0x%08X", my_fstorage.start_addr);
        NRF_LOG_INFO("FS end   = 0x%08X", my_fstorage.end_addr);
        NRF_LOG_INFO("erase    = 0x%08X", addr);
        // 데이터를 쓰기 전 4KB 페이지를 미리 지웁니다.'
        rc = nrf_fstorage_erase(&my_fstorage, addr, 1, NULL);
        if (rc != NRF_SUCCESS)
        {
            error_set_flag(FLASH_ERROR);
            NRF_LOG_ERROR("Flash erase failed: %d", rc);
            return rc;
        }
    }
    flash_wait_idle();
 // 2. 8바이트 데이터 쓰기 요청
    rc = nrf_fstorage_write(&my_fstorage, addr, data, len, NULL);
    if (rc == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Stored at 0x%08x", addr);

    }
    else
    {
        error_set_flag(FLASH_ERROR);
        NRF_LOG_ERROR("Flash write failed: %d", rc);
    }
    flash_wait_idle();
    return rc;
}

ret_code_t flash_read(uint32_t addr, uint8_t* data, uint16_t len)
{
#if 0
// 1. 주소 범위 예외 처리 예시 (설정한 범위를 벗어나면 에러 리턴)
    if (addr < FLASH_START_ADDR || (addr + len) > FLASH_END_ADDR)
    {
        NRF_LOG_ERROR("Read address out of bounds: 0x%08x (len: %d)", addr, len);
        return NRF_ERROR_INVALID_ADDR;
    }

    // 2. nrf_fstorage_read 호출 (호출 즉시 p_dest 버퍼에 데이터가 채워짐)
    ret_code_t rc = nrf_fstorage_read(&my_fstorage, addr, data, len);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("fstorage read failed: %d", rc);
    }
    NRF_LOG_INFO("read at 0x%08x", addr);
    return rc;
#else
    if (data == NULL || len == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // Flash 영역 범위 확인
    if (addr < FLASH_START_ADDR ||
        ((uint64_t)addr + len) > FLASH_END_ADDR)
    {
        NRF_LOG_ERROR("Read address out of bounds: 0x%08x (len: %d)",
                      addr, len);
        return NRF_ERROR_INVALID_ADDR;
    }

    // 1바이트씩 직접 Flash에서 읽기
    for (uint16_t i = 0; i < len; i++)
    {
        data[i] = *(volatile uint8_t *)(addr + i);
    }

    NRF_LOG_INFO("Flash read: addr=0x%08x len=%d", addr, len);

    return NRF_SUCCESS;
#endif

}
static bool factory_flag = false;
static bool setting_flag = false;
void factory_enable(void)
{
  factory_flag = true;
}
void setting_save(void)
{
  setting_flag = true;
}

void tracker_factory(void)
{
  if(setting_flag)
  {
    setting_flag = false;
    flash_setting_save();
  }
  if(factory_flag)
  {
    flash_erase(FLASH_SETTING_ADDR);
  }
}

void tracker_setting_dump(const tracker_setting_t *s)
{
    NRF_LOG_INFO("========== Tracker Setting ==========");

    NRF_LOG_INFO("device_name              : %s", s->device_name);
    NRF_LOG_INFO("peripheral_1             : %s", s->peripheral_1);
    NRF_LOG_INFO("peripheral_2             : %s", s->peripheral_2);
    NRF_LOG_INFO("peripheral_3             : %s", s->peripheral_3);

    NRF_LOG_INFO("peripheral_4             : %s", s->peripheral_4);
    NRF_LOG_INFO("beacon_data              : %s", s->beacon_data);

    NRF_LOG_INFO("data_collect_sec         : %u", s->data_collect_sec);

    NRF_LOG_INFO("beacon_tx_power          : %d", s->beacon_tx_power);
    NRF_LOG_INFO("beacon_adv_interval      : %u", s->beacon_adv_interval);

    NRF_LOG_INFO("crc16                    : 0x%04X", s->crc16);

    NRF_LOG_INFO("=====================================");
}
void setting_change(Motion_Packet_t *Motion_Packet)
{
    bool flag = false;
    
    // flash_packet.cmd_type으로 깔끔하게 분기 처리 가능
    switch(Motion_Packet->flash_packet.cmd_type)
    {
        case 0:
          flag = true;
          strncpy(tracker_setting.device_name, (char*)Motion_Packet->flash_packet.flash_req_name.name, sizeof(tracker_setting.device_name));
          break;
        case 1:
          flag = true;
          strncpy(tracker_setting.peripheral_1, (char*)Motion_Packet->flash_packet.flash_req_name.name, sizeof(tracker_setting.peripheral_1));
          break; 
        case 2:
          flag = true;
          strncpy(tracker_setting.peripheral_2, (char*)Motion_Packet->flash_packet.flash_req_name.name, sizeof(tracker_setting.peripheral_2));
          break;
        case 3:
          flag = true;
          strncpy(tracker_setting.peripheral_3, (char*)Motion_Packet->flash_packet.flash_req_name.name, sizeof(tracker_setting.peripheral_3));
          break;
        case 4:
          flag = true;
          strncpy(tracker_setting.peripheral_4, (char*)Motion_Packet->flash_packet.flash_req_name.name, sizeof(tracker_setting.peripheral_4));
          break;

        case 6:
          flag = true;
          tracker_setting.data_collect_sec = Motion_Packet->flash_packet.flash_req_duration.duration;
          if(tracker_setting.data_collect_sec < 5)
            tracker_setting.data_collect_sec = 5;
        break;

        case 7:
          flag = true;
          tracker_setting.beacon_tx_power = Motion_Packet->flash_packet.flash_req_txpower.txpower;
          break;

        case 8:
          flag = true;
          tracker_setting.beacon_adv_interval = Motion_Packet->flash_packet.flash_req_beacon_interval.interval;
          break;
    }

    if(flag)
    {
        setting_save();
        Motion_Packet->event_code = FLASH_RESPONSE;
        BLE_Send_byte((uint8_t*)Motion_Packet,sizeof(Motion_Packet_t));
    }
}


void flash_setting_save(void)
{
    tracker_setting.crc16 = crc16_ccitt_compute((uint8_t*)&tracker_setting,sizeof(tracker_setting)-2);
    flash_write(FLASH_SETTING_ADDR,(uint8_t*)&tracker_setting, sizeof(tracker_setting));

    tracker_setting_t read_setting;
    flash_read(FLASH_SETTING_ADDR, (uint8_t*)&read_setting ,sizeof(read_setting));
    if(memcmp(&tracker_setting,&read_setting,sizeof(tracker_setting_t)) == 0)
    {
      NRF_LOG_INFO("FLASH_WRITE_OK");
    }
    else 
    {
      NRF_LOG_INFO("FLASH_WRITE_FAIL");
    }
}


static void fds_dump_all(void)
{
    ret_code_t rc;
    tracker_setting_t read_setting;

    flash_read(FLASH_SETTING_ADDR, (uint8_t*)&read_setting ,sizeof(read_setting));
    if(crc16_ccitt_compute((uint8_t*)&read_setting,sizeof(read_setting)-2) != read_setting.crc16)
    {
      flash_setting_save();
    }
    else
    {
      memcpy(&tracker_setting,&read_setting,sizeof(tracker_setting_t));
    }

    tracker_setting_dump(&tracker_setting);
}
#include "app_error.h"

uint32_t flash_init(void)
{
    ret_code_t rc = nrf_fstorage_init(&my_fstorage, &nrf_fstorage_sd, NULL);
    return rc;
    fds_dump_all();
}

