


#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"




#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_peripheral_con.h"
#include "ble_dfu.h"
#include "nrf_power.h"
#include "app_ble_rx.h"

#include "app_flash.h"
#include "app_qc.h"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(100, UNIT_0_625_MS)       
#define APP_ADV_DURATION                0   
#define APP_BLE_CONN_CFG_TAG            1    

static uint16_t   conn_handle          = BLE_CONN_HANDLE_INVALID;    
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising); 
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
extern int8_t get_rssi(void);
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    int8_t rssi_dbm = 0;
    uint8_t ch_index = 0;
    uint32_t err_code;
    switch(p_evt->type)
    {
        case BLE_NUS_EVT_RX_DATA:///peripheral 수신부 
        rssi_dbm = get_rssi();

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        NRF_LOG_INFO("nus = %d \r\n",p_evt->params.rx_data.length);
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            NRF_LOG_INFO("%02x",p_evt->params.rx_data.p_data[i]);
        }
        NRF_LOG_INFO("\r\n");
        ble_data_input((uint8_t*)p_evt->params.rx_data.p_data,p_evt->params.rx_data.length,rssi_dbm);
        break;
        case BLE_NUS_EVT_COMM_STOPPED:
        NRF_LOG_INFO("BLE_NUS_EVT_COMM_STOPPED");
        break;
    }

}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code ;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:

        break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
        break;
        default:
        break;
    }
}
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_on_disconnect_disabled = true;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void app_peri_ble_event_handler(void* p_ble_evt, uint16_t m_conn_handle)
{
      uint32_t                err_code;
      conn_handle = m_conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);
}
static uint32_t app_peri_data_send_handler(char* ch, uint16_t len,uint16_t m_conn_handle)///peripheral 송신부 
{
    uint32_t                err_code;
    err_code = ble_nus_data_send(&m_nus, (uint8_t*)ch, &len, m_conn_handle);
    return err_code;
}

void peri_disconnect_peer(void)
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
void App_advertising_stop(void)
{
      uint32_t err_code; 
      NRF_LOG_INFO("\r\nSTOP ADV\r\n");
      err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
      NRF_LOG_INFO("adv_stop handle=%d err=0x%08lx\r\n",
       m_advertising.adv_handle,
       err_code);
      if ((err_code != NRF_SUCCESS) &&
          (err_code != NRF_ERROR_INVALID_STATE))
      {
          APP_ERROR_CHECK(err_code);
      }
}
static void advertising_beacon_name(char* beacon_name)
{
    ret_code_t err;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) beacon_name,
                                          strlen(beacon_name));
}
uint8_t is_ParingMode(void);
static uint32_t advertising_data_start(void)
{

    tracker_setting_t* setting = Tracker_Get_Setting();
    ret_code_t err;
    ble_advdata_t next_advdata;
    int8_t target_power = 0;
    static ble_advdata_service_data_t service;
    memset(&next_advdata, 0, sizeof(next_advdata));

    next_advdata.name_type = BLE_ADVDATA_FULL_NAME;
    next_advdata.flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    next_advdata.include_appearance = false;
    //next_advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    if(is_ParingMode())
    {
      service.service_uuid = 0x4321;
      target_power = 4;
    }
    else
    {
      service.service_uuid = 0x1234;
      target_power = setting->beacon_tx_power; 
    }
    uint8_t beacon_data[30];
// SoftDevice로부터 칩의 MAC 주소 가져오기

    sprintf(beacon_data, "%s",setting->beacon_data);

    service.data.p_data = beacon_data;
    service.data.size = strlen(beacon_data);
    next_advdata.p_service_data_array = &service;
    next_advdata.service_data_count = 1;
  
    
    err = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 
                                  m_advertising.adv_handle, 
                                  target_power);
    if (err == NRF_SUCCESS)
    {
        NRF_LOG_INFO("TX Power set ok 0x%08X\r\n", err);
    }
    else
    {
        return err;
    }

    err = ble_advertising_advdata_update(&m_advertising,
                                                     &next_advdata,
                                                     NULL);

    return err;
}

static uint32_t advertising_beacon_start(void)
{
    ret_code_t err;
    ble_advdata_t advdata;     
    ble_uuid_t adv_uuid;

    // 🌟 static 선언 필수 (지역변수로 선언 시 메모리 오염 방지)
    static ble_advdata_service_data_t service; 
    tracker_setting_t* setting = Tracker_Get_Setting();

    adv_uuid.type = m_nus.uuid_type;
    NRF_LOG_INFO("uuid_type=%d", m_nus.uuid_type);
    adv_uuid.uuid = BLE_UUID_NUS_SERVICE; // 0x0001

    memset(&advdata, 0, sizeof(advdata));

// 1. [Adv Data (본 패킷)] : Flags + 이름 + 비콘 데이터
    advdata.flags     = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; 
    advdata.name_type = BLE_ADVDATA_FULL_NAME; 
    if(is_ParingMode())
      service.service_uuid = 0x4321;
    else
      service.service_uuid = 0x1234;
    service.data.p_data  = (uint8_t*)setting->beacon_data;
    service.data.size    = strlen(setting->beacon_data); // 줄이지 않고 원래 길이 그대로 사용 가능!
    
    advdata.p_service_data_array = &service;
    advdata.service_data_count   = 1;

    err = ble_advertising_advdata_update(&m_advertising, &advdata,NULL);
    if (err != NRF_SUCCESS)
    {
        return err;
    }

    // TX Power 설정
    int8_t target_power = 4; 
    err = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 
                                  m_advertising.adv_handle, 
                                  target_power);
    return err;
}





void App_advertising_start(uint8_t* Name, uint8_t data,ble_gap_addr_t* whitelist_addr)
{
    #define BLE_UNIT_MS_NUM 8
    #define BLE_UNIT_MS_DEN 5

    NRF_LOG_INFO("\r\nSTART ADV %d\r\n",data);
    tracker_setting_t* setting = Tracker_Get_Setting();
    uint32_t err_code = 0; // 에러 코드 변수 추가
    if(Name == NULL)
      Name = setting->device_name;
    if(data)
    {
        // 1. 순수 비콘 모드 (CONNECT 버튼 없애기)
        advertising_beacon_name(Name);
        if(advertising_data_start() != NRF_SUCCESS)
          err_code++;
        
        //m_advertising.adv_modes_config.ble_adv_fast_interval = setting->beacon_adv_interval * BLE_UNIT_MS_NUM / BLE_UNIT_MS_DEN;
        m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
        m_advertising.adv_params.interval = setting->beacon_adv_interval * BLE_UNIT_MS_NUM / BLE_UNIT_MS_DEN;
        // ⭐ 핵심: 비콘 모드일 때는 얄미운 라이브러리 함수 대신 로우레벨 API 직접 호출!
        if(sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, &m_advertising.adv_data, &m_advertising.adv_params) != NRF_SUCCESS)
          err_code++; 

        if(sd_ble_gap_adv_start(m_advertising.adv_handle, APP_BLE_CONN_CFG_TAG) != NRF_SUCCESS)
          err_code++; 

    }
   else
    {
        // 2. 폰 연결 모드 (CONNECT 버튼 살리기)
        advertising_beacon_name(Name);
        if(advertising_beacon_start() != NRF_SUCCESS)
          err_code++;

        m_advertising.adv_modes_config.ble_adv_fast_interval = 100 * BLE_UNIT_MS_NUM / BLE_UNIT_MS_DEN;
        m_advertising.adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        
        // 연결 모드일 때는 기존 라이브러리 함수 그대로 사용
        if(ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST) != NRF_SUCCESS)
          err_code++;
        
    }
    if(err_code)
    {
      error_set_flag(BLE_ERROR);
    }
}

void App_Peripheral_init(system_config_t* system_config)
{
      system_config->app_peri_ble_event_handler = app_peri_ble_event_handler;
      system_config->app_peri_data_send_handler = app_peri_data_send_handler;

      services_init();
      advertising_init();
}


