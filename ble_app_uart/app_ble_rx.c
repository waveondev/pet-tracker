#include "app_ble_rx.h"
#include "app_sensor_flash.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_ble_tx.h"
#include "app_gpio.h"
#include "app_flash.h"
#include "app_rtc.h"

void lsb6_ble_data(Motion_Packet_t* rx_packet);
void setting_change(Motion_Packet_t *Motion_Packet);
void rtc_sync_set_time(Motion_Packet_t* Motion_Packet);
void timer_connection_stop_start(uint8_t state);
void ble_data_input(uint8_t* in_data, uint16_t len, int8_t rssi)
{
    Motion_Packet_t* Motion_Packet = (Motion_Packet_t*)in_data;
    timer_connection_stop_start(true);
    if(len < 20)
      return;
    switch(Motion_Packet->event_code)
    {
      case MOTION_START_REQUEST:
        sensor_data_send(Motion_Packet,rssi);
      break;
      case HEALTH_DATA_REQUEST:
        health_data_send(Motion_Packet,rssi);
      break;
      case MOTION_DATA_ACK:
        sensor_ack_input(Motion_Packet);
        led_ble_ack_input();
      break;
      case OTA_MODE_REQUEST:
        sensor_ota_send(Motion_Packet);
      break;
      case FACTORY_REQUEST:
        factory_enable();
      break;
      case LSM6_DATA_REQUEST:
        lsb6_ble_data(Motion_Packet);
      break;
      case TIME_RESPONSE:
        rtc_sync_set_time(Motion_Packet);
      break;
      case FLASH_REQUEST:
        setting_change(Motion_Packet);
      break;
      
      default:
      break;
    }
}

