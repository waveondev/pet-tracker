

#ifndef __APP_BLE_TX_H__
#define __APP_BLE_TX_H__

#include "stdint.h"
#include "app_sensor_flash.h"

void sensor_tx_timer_init(void);
void time_data_send(Motion_Packet_t* rx_packet, int8_t rssi);
void health_data_send(Motion_Packet_t* rx_packet, int8_t rssi);
void sensor_data_send(Motion_Packet_t* rx_packet,int8_t rssi);
void sensor_ack_input(Motion_Packet_t* rx_packet);
void sensor_ota_send(Motion_Packet_t* rx_packet);
#endif
