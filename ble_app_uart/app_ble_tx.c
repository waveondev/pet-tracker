#include "app_ble_tx.h"

#include <nrfx.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_flash.h"
#include "string.h"
#include "app_timer.h"
#include "app_adc.h"
#include "ble_dfu.h"
#include "nrf_log.h"
APP_TIMER_DEF(m_pack_send_timer);
APP_TIMER_DEF(m_ota_timer);
APP_TIMER_DEF(m_uptime_timer);

uint32_t BLE_Send_byte(uint8_t* data, uint16_t len);

static uint16_t send_seq = 0;

static bool m_is_pack_timer_running = false;
uint8_t sensor_get_data(Motion_Packet_t* data);
uint32_t calculate_total_send_count(void);
void sensing_send_init(void);
void sensing_ack_input(void);
#define TRACKER_MAJOR 1
#define TRACKER_MINOR 1
#define TRACKER_PATCH 1
static volatile uint32_t g_system_uptime_sec = 0;
static void uptime_timer_handler(void * p_context)
{
    g_system_uptime_sec++;
}

void health_data_send(Motion_Packet_t* rx_packet, int8_t rssi)
{
    Motion_Packet_t Motion_Packet;

    memset(&Motion_Packet, 0, sizeof(Motion_Packet));

    Motion_Packet.event_code = HEALTH_DATA_RESPONSE;
    uint8_t Bat_level = battery_percent_get();
    Motion_Packet.health_data_res.uptime_sec = g_system_uptime_sec;
    Motion_Packet.health_data_res.Bat_Level = Bat_level;
    uint8_t Bat_voltage = (battery_voltage_get()/100);
    Motion_Packet.health_data_res.Bat_Voltage = Bat_voltage;
    Motion_Packet.health_data_res.major = TRACKER_MAJOR;
    Motion_Packet.health_data_res.minor = TRACKER_MINOR;
    Motion_Packet.health_data_res.patch = TRACKER_PATCH;
    Motion_Packet.health_data_res.target_rssi = rssi;
    fault_code fault_flag;
    if(Bat_voltage < 35)
      Motion_Packet.health_data_res.fault_flag.bit.Bat_Status = 2;
    else if(Bat_level < 40)
      Motion_Packet.health_data_res.fault_flag.bit.Bat_Status = 1;
    else
      Motion_Packet.health_data_res.fault_flag.bit.Bat_Status = 0;
    BLE_Send_byte((uint8_t*)&Motion_Packet,sizeof(Motion_Packet));
  
}

void time_data_send(Motion_Packet_t* rx_packet, int8_t rssi)
{
    Motion_Packet_t Motion_Packet;

    memset(&Motion_Packet, 0, sizeof(Motion_Packet));

    Motion_Packet.event_code = TIME_REQUEST;
    Motion_Packet.time_req.cmd_type = 1;

    BLE_Send_byte((uint8_t*)&Motion_Packet,sizeof(Motion_Packet));
}




static void pack_send_timer_handler(void * p_context)
{
    Motion_Packet_t Motion_Packet;
    memset(&Motion_Packet, 0, sizeof(Motion_Packet));
    
    Motion_Packet.event_code = MOTION_DATA;
    Motion_Packet.motion_data.seq = (uint8_t)send_seq; 
    uint8_t has_more_data = sensor_get_data(&Motion_Packet);
    uint32_t err_code = BLE_Send_byte((uint8_t*)&Motion_Packet, sizeof(Motion_Packet));
    if (err_code != NRF_SUCCESS)
    {
        m_is_pack_timer_running = false;
        return;
    }
    if (has_more_data == 0 || send_seq == 0xff)
    {
          m_is_pack_timer_running = false;
          NRF_LOG_INFO("--- All Flash Data Send Complete! ---\r\n");
    }
    else
    {
          send_seq++;
          app_timer_start(m_pack_send_timer, APP_TIMER_TICKS(500), NULL);
    }
}


void sensor_data_send(Motion_Packet_t* rx_packet, int8_t rssi)
{
    retained_data_t* retained_data = Sensor_Get_Seq();
    tracker_setting_t* setting = Tracker_Get_Setting();
    Motion_Packet_t Motion_Packet;
    memset(&Motion_Packet,0,sizeof(Motion_Packet));

    sensing_send_init();
    Motion_Packet.event_code = MOTION_START_RESPONSE;
    Motion_Packet.motion_req.interval = setting->data_collect_sec;
// 1. 계산된 포인트 수 가져오기
    uint32_t total = calculate_total_send_count();

    // 2. 0xFF 기준 제한 (255개 패킷 * 9 = 2295 포인트)
    if (total > (9 * 0x100)) {
        total = 9 * 0x100;
    }
    Motion_Packet.motion_req.total_points = total;

    if(m_is_pack_timer_running == false)
    {
        uint32_t err_code = BLE_Send_byte((uint8_t*)&Motion_Packet,sizeof(Motion_Packet));
        if(err_code == NRF_SUCCESS && Motion_Packet.motion_req.total_points != 0)
        {
            send_seq = 0;
            m_is_pack_timer_running = true;
            app_timer_start(m_pack_send_timer,\
                        APP_TIMER_TICKS(500),\
                        NULL);
        }
    }
}

void sensor_ack_input(Motion_Packet_t* rx_packet)
{
    retained_data_t* retained_data = Sensor_Get_Seq();
    if(rx_packet->motion_ack.ack_seq_no == send_seq)
    {
      sensing_ack_input();
      NRF_LOG_INFO("ack_input = %d \r\n",rx_packet->motion_ack.ack_seq_no);
    }
}



static void ota_timer_handler(void * p_context)
{
 #if DFUMODE
   ble_dfu_buttonless_bootloader_start_prepare();
  #endif
}

void sensor_ota_send(Motion_Packet_t* rx_packet)
{
    Motion_Packet_t Motion_Packet;
    memset(&Motion_Packet,0,sizeof(Motion_Packet));
    Motion_Packet.event_code = OTA_MODE_RESPONSE;
    Motion_Packet.ota_res.cmd_type = 1;
     uint32_t bat = battery_percent_get();
    #if DFUMODE
    if(bat >= 40 && rx_packet->ota_req.cmd_type == 1)
    {
      Motion_Packet.ota_res.status = 0;
    }
    else
    #endif
    {
      Motion_Packet.ota_res.status = 1;
    }

      uint32_t err_code = BLE_Send_byte((uint8_t*)&Motion_Packet,sizeof(Motion_Packet));
      if(err_code == NRF_SUCCESS)
      {
          if(Motion_Packet.ota_res.status == 0)
          {
              app_timer_start(m_ota_timer,\
                          APP_TIMER_TICKS(200),\
                          NULL);
          }
      }

}


void sensor_tx_timer_init(void)
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_pack_send_timer,\
                            APP_TIMER_MODE_SINGLE_SHOT,\
                            pack_send_timer_handler);
    err_code = app_timer_create(&m_ota_timer,\
                            APP_TIMER_MODE_SINGLE_SHOT,\
                            ota_timer_handler);
    err_code = app_timer_create(&m_uptime_timer,\
                            APP_TIMER_MODE_REPEATED,\
                            uptime_timer_handler);
    app_timer_start(m_uptime_timer,\
                APP_TIMER_TICKS(1000),\
                NULL);



    APP_ERROR_CHECK(err_code); 
}
