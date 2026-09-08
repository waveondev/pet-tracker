#ifndef __MAIN_H__
#define __MAIN_H__


#include <stdint.h>
#include <string.h>


typedef struct 
{
  void (*app_peri_ble_event_handler)(void* p_ble_evt, uint16_t m_conn_handle);
  uint32_t (*app_peri_data_send_handler)(char* ch, uint16_t len, uint16_t m_conn_handle);
  void (*app_cent_ble_event_handler)(void* p_ble_evt, uint16_t m_conn_handle);
  uint32_t (*app_cent_data_send_handler)(char* ch, uint16_t len, uint16_t m_conn_handle);
}system_config_t;

#endif