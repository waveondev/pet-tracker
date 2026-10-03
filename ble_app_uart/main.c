/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */



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
#include "app_flash.h"
#include "app_rtc.h"
#include "app_qc.h"
#include "app_wdg.h"

/* DFUMODE 1 */
//FLASH1 RX 0x0 0x100000 RAM1 RWX 0x20000000 0x40000 uicr_bootloader_start_address RX 0x10001014 0x4

/* DFUMODE 0 */
//FLASH1 RX 0x0 0x100000 RAM1 RWX 0x20000000 0x40000 

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "main.h"
#include "app_peripheral_con.h"
#include "app_central_con.h"
#include "app_gpio.h"
#include "app_adc.h"
#include "app_twi.h"

#include "app_sensor_flash.h"
#include "app_ble_tx.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#if 1
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define SLAVE_LATENCY        49
#else
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY        3
#endif

#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(5000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */

static system_config_t system_config; 
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static void Paring_Timer_Set(uint8_t next_state, uint8_t enable, uint32_t next_time);
void pairing_set(void);
static uint8_t sleep_count = 0;
APP_TIMER_DEF(m_role_timer);
#define TIMER_MINUTE_INTERVAL   (60 * 1000)
#define TIMER_15_MIN_INTERVAL   (15 * TIMER_MINUTE_INTERVAL)
#define TIMER_REGI_MIN_INTERVAL   (2 *TIMER_MINUTE_INTERVAL)


// 2. 타이머 ID 변수 선언
APP_TIMER_DEF(m_15min_timer_id);
APP_TIMER_DEF(m_paring_timer);
APP_TIMER_DEF(m_regi_timer);
APP_TIMER_DEF(m_connection_timer);
APP_TIMER_DEF(m_not_conn_timer);


void Sleep_Set(void);
void timer_regi_stop_start(uint8_t state);
static uint8_t Next_state;
static void app_timers_stop(void);
static void app_timers_start(void);
void timer_15min_stop_start(uint32_t next_time, uint8_t state);
int8_t get_rssi(void);
void timer_connection_stop_start(uint8_t state);
void timer_not_conn_stop_start(uint8_t state);
static uint32_t not_conn_time = 0;

typedef enum
{
    ROLE_BOOT_UP,
    ROLE_PAIRING_CENTRAL,
    ROLE_PAIRING_PERIPHERAL,
    ROLE_PAIRING_ING,
    ROLE_PAIRING_END,
    ROLE_PAIRING_CONNECT,
    ROLE_SLEEP,
} role_state_t;
typedef enum
{
    PARING_CENTRAL_SCAN,
    PARING_CENTRAL_SCAN_END,
    PARING_CENTRAL_CONNECT,
    PARING_PERIPHERAL,
    PARING_SLEEP,
} Paring_state_t;

float enmo_get(void);

static uint32_t Sleep_Enable = 0;

#include "app_sensor.h"
#include "nrf_fstorage.h"

static role_state_t m_role_state = ROLE_BOOT_UP;
static Paring_state_t m_paring_state = PARING_CENTRAL_SCAN;
static uint8_t connect_target = 0xFF;
void BLE_ALL_Disconnect(void);

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;



    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    uint8_t state = 0;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_TIMEOUT:
        {
            NRF_LOG_ERROR(
                "BLE_GAP_EVT_TIMEOUT src=%d",
                p_ble_evt->evt.gap_evt.params.timeout.src
            );

            if (p_ble_evt->evt.gap_evt.params.timeout.src ==
                BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_ERROR(">>> CONNECTION TIMEOUT <<<");
        	Paring_Timer_Set(PARING_PERIPHERAL, 1,100);
            }
        }
        break;
        case BLE_GAP_EVT_CONNECTED:
            not_conn_time = 0;
            timer_not_conn_stop_start(false);
            timer_regi_stop_start(0);
            timer_connection_stop_start(true);
            comunication_connect_event();
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;   
           
            switch (m_paring_state)
            {
                case PARING_CENTRAL_CONNECT:
                    system_config.app_cent_ble_event_handler((void*)p_ble_evt,m_conn_handle);
                    connect_target = CENTRAL_CONNECT;
                break;
            
                case PARING_PERIPHERAL:            
                    system_config.app_peri_ble_event_handler((void*)p_ble_evt,m_conn_handle);
                    connect_target = PERIPHERAL_CONNECT;
  
                    // 2. FDS가 실제로 플래시에 쓰기 전까지 사라지지 않도록 static 공간에 복사
                    ble_gap_addr_t const * peer_info = &p_ble_evt->evt.gap_evt.params.connected.peer_addr;
  
                    // 2. FDS가 실제 플래시에 쓰기 전까지 소멸하지 않도록 static 공간에 통째로 복사

                break;
            }
            //led_pairing_stop();
            //timer_15min_stop_start(1000, 1);
            timer_15min_stop_start(TIMER_15_MIN_INTERVAL, 1);
            Paring_Timer_Set(PARING_CENTRAL_SCAN,0,0);
            m_role_state = ROLE_PAIRING_END;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected %d" ,p_ble_evt->evt.gap_evt.params.disconnected.reason);
            uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            timer_connection_stop_start(false);
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                sd_ble_gap_rssi_stop(m_conn_handle);
            }
            app_timers_start();
            pairing_set();
            timer_15min_stop_start(0, 0);
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // ★ 현재 링커가 할당한 ram_start 주소를 RTT 로그로 출력
    NRF_LOG_INFO("Required RAM Start Address: 0x%08X", ram_start);

    // 여기서 ram_start 값이 링커(.ld / SES 설정) 값과 다르면 에러 반환
    err_code = nrf_sdh_ble_enable(&ram_start);
    
    // 만약 주소가 틀리면 여기서 APP_ERROR_CHECK에 걸려 멈춤
    if (err_code == NRF_ERROR_NO_MEM) {
        NRF_LOG_ERROR("RAM Start address is too low! Change it to 0x%08X", ram_start);
    }
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    // 2. Peripheral(nRF52)의 최대 수신 MTU 상한선을 NRF_SDH_BLE_GATT_MAX_MTU_SIZE(247)로 강제 지정
    // 스마트폰이 512를 날려도 SoftDevice가 247로만 응답하게 만듭니다.
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    // 3. Data Length 상한선 설정 (251)
    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
    APP_ERROR_CHECK(err_code);
}
// 1. 타임스탬프 함수 정의 (uint32_t를 리턴해야 함)
uint32_t my_timestamp_func(void) {
    return app_timer_cnt_get(); // 또는 RTC 틱 값 리턴
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(my_timestamp_func);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling Scanning Module events.
 */
void go_system_off(void)
{
    uint32_t err_code = sd_power_ram_power_set(0, 0x03);
    APP_ERROR_CHECK(err_code); 
    NRF_LOG_INFO("go_system_off enter");
    NRF_LOG_FLUSH();

    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);

    NRF_LOG_INFO("go_system_off return");
    NRF_LOG_FLUSH();
}

void nrf_system_off_mode(void)
{
  led_all_off();
  nrf_gpio_cfg_sense_input(LSM_INT_PIN,NRF_GPIO_PIN_NOPULL,NRF_GPIO_PIN_SENSE_HIGH);
  nrf_gpio_cfg_sense_input(PKEY_STAT_SW_PIN,NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_SENSE_LOW );
  nrf_gpio_cfg_sense_input(VBUS_IN_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH );
  go_system_off();  
}

uint8_t is_ParingMode(void)
{
    return (m_role_state == ROLE_PAIRING_ING )|| (m_role_state == ROLE_PAIRING_END);
}


static void paring_timer_handler(void * p_context)
{
    tracker_setting_t* setting = Tracker_Get_Setting();
    m_paring_state = Next_state;
    uint32_t peripheral_time = 0;
    if(not_conn_time > 1800)
      peripheral_time = 300000;
    else if(not_conn_time > 600)
      peripheral_time = 60000;
    else if(not_conn_time > 180)
      peripheral_time = 20000;    
    else
      peripheral_time = 10000;
    switch (m_paring_state)
    {
      case PARING_CENTRAL_SCAN:
      {
            led_pairing_start(CENTRAL_CONNECT);
                        App_advertising_start(NULL,1,0);   
            App_scan_start();
            Paring_Timer_Set(PARING_CENTRAL_SCAN_END,1,10000);
      }break;
      case PARING_CENTRAL_SCAN_END:
      {
            BLE_ALL_Disconnect();
            Paring_Timer_Set(PARING_CENTRAL_CONNECT,1,100);
      }break;
      case PARING_CENTRAL_CONNECT:
      {
            if(cent_connect_peer() != NRF_SUCCESS)
            {
               Paring_Timer_Set(PARING_PERIPHERAL, 1,100);
            }

      }break;
      case PARING_PERIPHERAL:
      {
            BLE_ALL_Disconnect();
            led_pairing_start(PERIPHERAL_CONNECT);
            App_advertising_start(NULL, 0,0);
            Paring_Timer_Set(PARING_SLEEP, 1,peripheral_time);
      }break;
      case PARING_SLEEP:
      {
            App_advertising_stop();
            led_pairing_stop();
            connect_fail_event();
            {
                Paring_Timer_Set(PARING_CENTRAL_SCAN, 1,100);
            }
      }break;
        
    }
}
static void role_timer_handler(void * p_context)
{
    ret_code_t err_code;

    int8_t rssi_dbm ;
    switch (m_role_state)
    {
        case ROLE_BOOT_UP:
        break;              
        case ROLE_PAIRING_CENTRAL:   

            Paring_Timer_Set(PARING_CENTRAL_SCAN,1,100);
            m_role_state = ROLE_PAIRING_ING;
        break;
        case ROLE_PAIRING_PERIPHERAL:      
            Paring_Timer_Set(PARING_PERIPHERAL,1,100);
            m_role_state = ROLE_PAIRING_ING;
        break;
        case ROLE_PAIRING_END:
            App_advertising_stop();
             sd_ble_gap_rssi_start(m_conn_handle, 0, 0);
            m_role_state = ROLE_PAIRING_CONNECT;
        break;
        case ROLE_PAIRING_CONNECT:
            App_advertising_start(NULL, 1,0);
            app_timers_stop();
            m_role_state = ROLE_SLEEP;
        case ROLE_SLEEP:

        break;
        default:
        break;
    }

}

// 3. 15분마다 실행될 콜백 함수
static void timer_15min_timeout_handler(void * p_context)
{ 
    int8_t rssi_dbm;
    NRF_LOG_INFO("15 Minutes Timer Expired! Doing task...");
    if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
       rssi_dbm = get_rssi();
    health_data_send(NULL, rssi_dbm);
    timer_15min_stop_start(TIMER_15_MIN_INTERVAL,1);
}

// 3. 15분마다 실행될 콜백 함수
static void timer_regi_timeout_handler(void * p_context)
{ 
  NVIC_SystemReset();
}
// 3. 15분마다 실행될 콜백 함수
static void timer_conn_timeout_handler(void * p_context)
{ 
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        sd_ble_gap_disconnect(
            m_conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION
        );
    }
}

// 3. 15분마다 실행될 콜백 함수
static void timer_not_conn_timeout_handler(void * p_context)
{ 
    not_conn_time++;
}


uint32_t BLE_Send_byte(uint8_t* data, uint16_t len)
{ 
    uint32_t err_code = NRF_ERROR_NOT_SUPPORTED;
    switch (connect_target)
    {
      case PERIPHERAL_CONNECT:
       err_code = system_config.app_peri_data_send_handler((char*)data,len,m_conn_handle);
      break;
      case CENTRAL_CONNECT:
       err_code = system_config.app_cent_data_send_handler((char*)data,len,m_conn_handle);
      break;
    }

    NRF_LOG_INFO("BLE_Send_byte = %d ",err_code);
    return err_code;
}


/**@brief Function for initializing the timer module.
 */
void timer_15min_stop_start(uint32_t next_time, uint8_t state)
{
    uint32_t err_code;
    uint32_t timer_tick = APP_TIMER_TICKS(next_time);
    app_timer_stop(m_15min_timer_id);
    if(state)
    {
        err_code = app_timer_start(m_15min_timer_id, timer_tick, NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("15-Minute Timer Started.");
    }
}


/**@brief Function for initializing the timer module.
 */
void timer_regi_stop_start(uint8_t state)
{
    uint32_t err_code;
    uint32_t timer_tick = APP_TIMER_TICKS(TIMER_REGI_MIN_INTERVAL);
    app_timer_stop(m_regi_timer);
    if(state)
    {
        err_code = app_timer_start(m_regi_timer, timer_tick, NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("regi Timer Started.");
    }
}

void timer_connection_stop_start(uint8_t state)
{
    uint32_t err_code;
    uint32_t timer_tick = APP_TIMER_TICKS(TIMER_15_MIN_INTERVAL);
    app_timer_stop(m_connection_timer);
    if(state)
    {
        err_code = app_timer_start(m_connection_timer, timer_tick, NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("connection Timer Started.");
    }
}


/**@brief Function for initializing the timer module.
 */
void timer_not_conn_stop_start(uint8_t state)
{
    uint32_t err_code;
    uint32_t timer_tick = APP_TIMER_TICKS(1000);
    app_timer_stop(m_not_conn_timer);
    if(state)
    {
        err_code = app_timer_start(m_not_conn_timer, timer_tick, NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("not conn Timer Started.");
    }
}



static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_role_timer,\
                            APP_TIMER_MODE_REPEATED,\
                            role_timer_handler);

    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_paring_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                paring_timer_handler);
    APP_ERROR_CHECK(err_code);
    // 1. 15분 주기 타이머 생성 (반복 모드: APP_TIMER_MODE_REPEATED)
    err_code = app_timer_create(&m_15min_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_15min_timeout_handler);

    APP_ERROR_CHECK(err_code);    

    err_code = app_timer_create(&m_regi_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_regi_timeout_handler);

    APP_ERROR_CHECK(err_code);    

    err_code = app_timer_create(&m_connection_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_conn_timeout_handler);

    APP_ERROR_CHECK(err_code);    
    err_code = app_timer_create(&m_not_conn_timer,
                                APP_TIMER_MODE_REPEATED,
                                timer_not_conn_timeout_handler);

    APP_ERROR_CHECK(err_code);    
    
    sensor_tx_timer_init();
}
static void Paring_Timer_Set(uint8_t next_state, uint8_t enable, uint32_t next_time)
{
  uint32_t timer_tick = APP_TIMER_TICKS(next_time);

  Next_state = next_state;
  if(enable)
  {
      app_timer_stop(m_paring_timer);
      app_timer_start(m_paring_timer,\
                    timer_tick,\
                    NULL);
  }
  else
  {
      led_pairing_stop();
      sleep_count = 0;
      app_timer_stop(m_paring_timer);
  }
}
void BLE_ALL_Disconnect(void)
{
    App_advertising_stop();
    App_scan_stop();
    peri_disconnect_peer();
    cent_disconnect_peer();
}
void Regimode_set(void)
{
    BLE_ALL_Disconnect();
    
    Paring_Timer_Set(PARING_CENTRAL_SCAN,0,0);
    m_role_state = ROLE_BOOT_UP;
    timer_regi_stop_start(true);
    led_regi_start();
    App_advertising_start("T100-R-Tracker",0,0);
}
  
void pairing_set(void)
{
    BLE_ALL_Disconnect();
    m_role_state = ROLE_PAIRING_CENTRAL;
    timer_not_conn_stop_start(true);
}

/**@brief Function for starting advertising.
 */

static void app_timers_start(void)
{
    uint32_t err_code;
    
    err_code = app_timer_start(m_role_timer,
                               APP_TIMER_TICKS(100), // 20ms polling
                               NULL);
    APP_ERROR_CHECK(err_code);
}
static void app_timers_stop(void)
{
    uint32_t err_code;

    app_timer_stop(m_role_timer);
}

void Sleep_Set(void)
{
    app_timers_stop();
    power_en_down();
    NRF_LOG_INFO("sleep start\r\n");
    m_role_state = ROLE_SLEEP;
}

#include "ble_dfu.h"

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
             NRF_LOG_INFO("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
             NRF_LOG_INFO("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
             NRF_LOG_INFO("Unknown event from ble_dfu_buttonless.");
            break;
    }
}
/**@brief Application main function.
 */


#include "app_flash.h"
#include "app_sensor_flash.h"

int8_t get_rssi(void)
{
    int8_t rssi_dbm = 0;
    uint8_t ch_index = 0;
    uint32_t err = sd_ble_gap_rssi_get(m_conn_handle, &rssi_dbm, &ch_index);

    if (err == NRF_SUCCESS) {

    } else if (err == NRF_ERROR_NOT_FOUND) {
        // ⚠️ 아직 RSSI 샘플이 수집되지 않았거나 start를 안 한 상태!
        
        NRF_LOG_WARNING("RSSI not ready yet or rssi_start not called!");
    } else {
        if (err == BLE_ERROR_INVALID_CONN_HANDLE)
        {

        }
        else if (err == NRF_ERROR_INVALID_STATE)
        {
   
        }
        else
        {
     
        }
        NRF_LOG_ERROR("sd_ble_gap_rssi_get failed: 0x%x", err);
    }
    return rssi_dbm;
}

void BLE_Init(void)
{

    power_management_init();

    gap_params_init();
    gatt_init();
    App_Peripheral_init(&system_config);
    conn_params_init();

    NRF_LOG_INFO("UART started.");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
    App_Central_init(&system_config);
    
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

}
void clock_debug(void);

int main(void)
{
    bool erase_bonds;
        uint32_t                  err_code;
#if 1
    APP_ERROR_CHECK(ble_dfu_buttonless_async_svci_init());


    ble_dfu_buttonless_init_t dfus_init = {0};

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);

#endif
#if NRF_LOG_ENABLED
    log_init();
#endif
  
    timers_init();
    ble_stack_init();
    //rtc_sync_init();
    gpio_init();
#if 1
#if !DEBUG
    wdt_init();
#endif
   // saadc_init();
    app_qc_timer_create();
    saadc_timer_handler(NULL);
    if(flash_init() != 0)
      error_set_flag(FLASH_ERROR);


    //twi_init();
    if(Sensor_init(false) == false)
      error_set_flag(MOTION_ERROR);

    Sensor_update();

    #if 1
    BLE_Init();            
    app_timers_start();
    
    #endif
#endif
    #if DEBUG
    pairing_set();
    #endif
    for (;;)
    {
        tracker_factory();
        Sensor_update();
        idle_state_handle();
    }
}


/**
 * @}
 */
