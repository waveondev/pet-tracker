

#ifndef __APP_CENTRAL_CON_H__
#define __APP_CENTRAL_CON_H__


#include "main.h"

void App_scan_stop(void);
void App_scan_start(void);
void App_Central_init(system_config_t* system_config); 
void cent_disconnect_peer(void);
uint32_t cent_connect_peer(void);

#endif
