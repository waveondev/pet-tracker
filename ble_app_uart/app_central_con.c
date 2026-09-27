#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "ble_nus_c.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "app_central_con.h"

#include "ble_dfu.h"
#include "app_ble_rx.h"

#define APP_BLE_CONN_CFG_TAG            1      

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE Nordic UART Service (NUS) client instance. */                                         
BLE_DB_DISCOVERY_DEF(m_db_disc);  
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);   

NRF_BLE_SCAN_DEF(m_scan);
static uint16_t   conn_handle          = BLE_CONN_HANDLE_INVALID;    

#define SCAN_DEVICE_SIZE 4
typedef struct{
  uint8_t used;
  int8_t rssi;
  ble_gap_addr_t peer_addr;
}Central_Device_t;

static Central_Device_t device_detect[SCAN_DEVICE_SIZE];




static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}
/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


static void nus_error_handler(uint32_t nrf_error)
{
    if (nrf_error == NRF_ERROR_RESOURCES)
    {
        // 버퍼가 찼으므로 리셋시키지 않고, 다음 틱이나 TX_COMPLETE 이벤트 후 재시도하도록 처리
        NRF_LOG_WARNING("GATT Queue Full, retry later.");
        return; // 또는 재시도 플래그 처리
    }
    APP_ERROR_HANDLER(nrf_error);
}

static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    int ret = memcmp(p_data, "RESET", 5);

    if (ret == 0) {
       
        cent_disconnect_peer();
    }
    else
    {
        NRF_LOG_INFO("ret = %d",ret);
    }
}
extern int8_t get_rssi(void);
/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    int8_t rssi_dbm = 0;
    ret_code_t err_code;
            NRF_LOG_INFO("ble_nus_c_evt_handler %d",p_ble_nus_evt->evt_type);
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Discovery complete.");
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);
            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Connected to device with Nordic UART Service.");
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT: ///Central 수신부
            rssi_dbm = get_rssi();

            NRF_LOG_DEBUG("ble_nus_c_evt_handler Receiving data.");
            NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);

            NRF_LOG_INFO("c-nus = %d \r\n",p_ble_nus_evt->data_len);
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                NRF_LOG_INFO("%02x",p_ble_nus_evt->p_data[i]);
            }
            NRF_LOG_INFO("\r\n");
            ble_data_input(p_ble_nus_evt->p_data,p_ble_nus_evt->data_len,rssi_dbm);
            //ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);

            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            //scan_start();
            break;
    }
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = nus_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Scanning Module events.
 */

#include "app_flash.h"
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    const ble_gap_evt_adv_report_t * p_adv =
        p_scan_evt->params.p_not_found;
    tracker_setting_t* setting = Tracker_Get_Setting();
    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;
         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
             NRF_LOG_INFO("NRF_BLE_SCAN_EVT_CONNECTED");
              ble_gap_evt_connected_t const * p_connected =
                               p_scan_evt->params.connected.p_connected;
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]
                      );
           
            
         } break;
       case NRF_BLE_SCAN_EVT_NOT_FOUND:
        {
           const ble_gap_evt_adv_report_t * p_adv = p_scan_evt->params.p_not_found;
          if (p_adv == NULL || p_adv->data.p_data == NULL || p_adv->data.len == 0) break;

          uint16_t parsed_name_len = 0;
          uint16_t data_offset = 0;

          // 1. Complete Local Name 검색
          data_offset = 0;
          parsed_name_len = ble_advdata_search(p_adv->data.p_data,
                                                        p_adv->data.len,
                                                        &data_offset,
                                                        BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);

          // 2. 실패 시 Short Local Name 검색
          if (parsed_name_len == 0) {
              data_offset = 0;
              parsed_name_len = ble_advdata_search(p_adv->data.p_data,
                                                            p_adv->data.len,
                                                            &data_offset,
                                                            BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME);
          }

          // 이름이 없거나 길이가 정상이 아닌 경우 제외
          if (parsed_name_len == 0 || parsed_name_len > 31) break;

        // 3. 인덱스 오버플로우 안전성 체크 (포인터가 패킷 범위를 벗어나는지 확인)
            if ((data_offset + parsed_name_len) > p_adv->data.len) break;

            uint8_t * p_device_name = &p_adv->data.p_data[data_offset];

            // 4. 안전한 출력 처리
            uint8_t raw_name[32] = {0};
            memcpy(raw_name, p_device_name, parsed_name_len);
            raw_name[parsed_name_len] = '\0'; // 널 문자 추가

            NRF_LOG_INFO("Parsed Name (len: %d) = %s", parsed_name_len, raw_name);
            char * target_prefixes[4];
            target_prefixes[0] = setting->peripheral_1;
            target_prefixes[1] = setting->peripheral_2;
            target_prefixes[2] = setting->peripheral_3;
            target_prefixes[3] = setting->peripheral_4;
    
            size_t prefix_count = sizeof(target_prefixes) / sizeof(target_prefixes[0]);
            bool is_matched = false;

            for (size_t k = 0; k < prefix_count; k++)
            {
                if (target_prefixes[k] == NULL) continue;

                size_t prefix_len = strlen(target_prefixes[k]);
                if (prefix_len == 0) continue; 

                if (parsed_name_len >= prefix_len)
                {
                    if (memcmp(p_device_name, target_prefixes[k], prefix_len) == 0)
                    {
                        is_matched = true;
                        break;
                    }
                }
            }

            if (is_matched)
            {
                ble_gap_addr_t const * addr = &p_adv->peer_addr;
                bool found = false;

                for (int i = 0; i < SCAN_DEVICE_SIZE; i++)
                {
                    if (device_detect[i].used)
                    {
                        if (memcmp(device_detect[i].peer_addr.addr, addr->addr, 6) == 0)
                        {
                            device_detect[i].rssi = p_adv->rssi;
                            found = true;
                            break;
                        }
                    }
                }

                if (!found)
                {
                    for (int i = 0; i < SCAN_DEVICE_SIZE; i++)
                    {
                        if (!device_detect[i].used)
                        {
                            device_detect[i].used = 1;
                            memcpy(&device_detect[i].peer_addr, addr, sizeof(ble_gap_addr_t));
                            uint8_t name[32] = {0};
                            memcpy(name,p_device_name,parsed_name_len);
                            NRF_LOG_INFO("filter = %s" , name);
                            device_detect[i].rssi = p_adv->rssi;
                            break;
                        }
                    }
                }
            }
        } 
        break;
         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.\r\n");
         } break;

         default:
             break;
    }
}
static void app_cent_ble_event_handler(void* p_ble_evt, uint16_t m_conn_handle)
{
    ret_code_t err_code;
    ble_evt_t const * ble_evt = (ble_evt_t const *)p_ble_evt;
    conn_handle = m_conn_handle;
    err_code = ble_nus_c_handles_assign(&m_ble_nus_c, m_conn_handle, NULL);
    APP_ERROR_CHECK(err_code);
    // start discovery of services. The NUS Client waits for a discovery result
    err_code = ble_db_discovery_start(&m_db_disc, ble_evt->evt.gap_evt.conn_handle);
    APP_ERROR_CHECK(err_code);

    //nrf_ble_gatt_att_mtu_central_set

}
static uint32_t app_cent_data_send_handler(char* ch, uint16_t len,uint16_t m_conn_handle) ///Central 송신부 
{
      uint32_t                err_code;

      err_code = ble_nus_c_string_send(&m_ble_nus_c,  (uint8_t*)ch, len);

      return err_code;
}

void cent_disconnect_peer(void)
{
    ret_code_t err_code;

    if (conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        NRF_LOG_INFO("Not connected");
        return;
    }

    err_code = sd_ble_gap_disconnect(
                    conn_handle,
                    BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Disconnect failed: 0x%x", err_code);
    }
}

uint32_t cent_connect_peer(void)
{
    int best_index = -1;
    int8_t best_rssi = -127;

    for (int i = 0; i < SCAN_DEVICE_SIZE; i++)
    {
        if (device_detect[i].used)
        {
            if (device_detect[i].rssi > best_rssi)
            {
                best_rssi = device_detect[i].rssi;
                best_index = i;
            }
        }
    }

    if (best_index < 0)
    {
       // NRF_LOG_INFO("No device to connect");
        return NRF_ERROR_NOT_FOUND;
    }

    NRF_LOG_INFO("Best RSSI device idx=%d rssi=%d",
                 best_index,
                 best_rssi);
    ble_gap_addr_t peer_addr;
    memset(&peer_addr, 0, sizeof(peer_addr));

    memcpy(&peer_addr,
           &device_detect[best_index].peer_addr,
           sizeof(ble_gap_addr_t));
    ret_code_t err_code;
       // 기존: &m_scan.scan_params 대신 사용
    ble_gap_scan_params_t conn_scan_params = m_scan.scan_params;

    // 연결 대기 타임아웃을 5초(500 * 10ms)로 별도 지정! (0이면 무제한 대기)
    conn_scan_params.timeout = 1000;
    err_code = sd_ble_gap_connect(
                    &peer_addr,
                    &conn_scan_params,
                    &m_scan.conn_params,
                    APP_BLE_CONN_CFG_TAG
                    );

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("Connect failed: %d", err_code);
    }
    return err_code;
}

void App_scan_stop(void)
{
      uint32_t err_code; 
      NRF_LOG_INFO("\r\nSTOP SCAN\r\n");
      err_code = sd_ble_gap_scan_stop();

      if ((err_code != NRF_SUCCESS) &&
          (err_code != NRF_ERROR_INVALID_STATE))
      {
          //APP_ERROR_CHECK(err_code);
          NRF_LOG_ERROR("App_scan_stop failed: 0x%x", err_code);
      }
}

void scan_name_filter_set(char* scan_name1, char* scan_name2, char* scan_name3, char* scan_name4)
{
#if 0
    uint32_t err_code; 
    nrf_ble_scan_all_filter_remove(&m_scan);
    if(scan_name1 != NULL)
    {
      err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, scan_name1);
      APP_ERROR_CHECK(err_code);
    }
    if(scan_name2 != NULL)
    {
      err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, scan_name2);
      APP_ERROR_CHECK(err_code);
    }
    if(scan_name3 != NULL)
    {
      err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, scan_name3);
      APP_ERROR_CHECK(err_code);
    }
    if(scan_name4 != NULL)
    {
      err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, scan_name4);
      APP_ERROR_CHECK(err_code);
    }
    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
    #endif
}

void App_scan_start(void)
{
      uint32_t err_code; 

    NRF_LOG_INFO("\r\nSTART SCAN \r\n");
    memset(device_detect,0,sizeof(device_detect));

    err_code = nrf_ble_scan_start(&m_scan);

    APP_ERROR_CHECK(err_code);

}

static void scan_init(void)
{
    ret_code_t err_code;
    nrf_ble_scan_init_t scan_init = {0};
 
    scan_init.conn_cfg_tag    = 0;
    err_code = nrf_ble_scan_init(&m_scan,
                                 &scan_init,
                                 scan_evt_handler);

    APP_ERROR_CHECK(err_code);
}


void App_Central_init(system_config_t* system_config)
{

    system_config->app_cent_ble_event_handler = app_cent_ble_event_handler;
    system_config->app_cent_data_send_handler = app_cent_data_send_handler;
    
    db_discovery_init();
    nus_c_init();
    scan_init();
    tracker_setting_t* setting = Tracker_Get_Setting();
    //scan_name_filter_set(setting->peripheral_1,setting->peripheral_2,setting->peripheral_3,setting->peripheral_4);
}


