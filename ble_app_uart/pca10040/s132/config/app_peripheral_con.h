

#ifndef __APP_PERIPHERAL_CON_H__
#define __APP_PERIPHERAL_CON_H__


#include "main.h"
#include "ble_gap.h"
uint32_t app_peri_data_send(char* Packet, uint16_t len,uint16_t m_conn_handle);
void App_advertising_stop(void);
void App_advertising_start(uint8_t* Name, uint8_t data,ble_gap_addr_t* whitelist_addr);
void App_Peripheral_init(system_config_t* system_config); 
void peri_disconnect_peer(void);
#endif
