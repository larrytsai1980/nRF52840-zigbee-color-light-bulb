/**
 * Copyright (c) 2018, Nordic Semiconductor ASA
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
 * @defgroup zigbee_examples_multiprotocol_nus_switch main.c
 * @{
 * @ingroup  zigbee_examples
 * @brief    UART over BLE application with Zigbee HA color light bulb profile.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service
 * and a color light bulb operating a Zigbee network.
 * This application uses the @ref srvlib_conn_params module.
 */
#include "zboss_api.h"
//#include "zb_mem_config_min.h"
#include "zb_error_handler.h"

#include "zigbee_color_light.h"

#include "nrf_drv_pwm.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "ble_nus.h"

#include "app_timer.h"
#include "boards.h"
#include "bsp_btn_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                         "Zigbee_UART"                           /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE               BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO               1                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                    320                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 200 ms). */
#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (200 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(300, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (300 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */
#define NRF_BLE_GATT_ATT_MTU_DEFAULT        NRF_SDH_BLE_GATT_MAX_MTU_SIZE           /**< Requested ATT_MTU size. This value most not be greater than NRF_SDH_BLE_GATT_MAX_MTU_SIZE. */

#define CAFE_CAFE                           0xCAFECAFE                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                    256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                    256                                     /**< UART RX buffer size. */

#define IEEE_CHANNEL_MASK                   ((1l << 11)|(1l << 12)|(1l << 13)|(1l << 14)|(1l << 15)|(1l << 16)|(1l << 17)|(1l << 18)|(1l << 19)|(1l << 20)|(1l << 21)|(1l << 22)|(1l << 23)|(1l << 24)|(1l << 25)|(1l << 26))                  /**< Scan all channels to find the coordinator. */
#define HA_COLOR_LIGHT_ENDPOINT             1                                       /**< Source endpoint used to control light bulb. */
#define MAX_CHILDREN                        10                                      /**< The maximum amount of connected devices. Setting this value to 0 disables association to this device.  */
#define ERASE_CONFIG_BUTTON                 BSP_BOARD_BUTTON_0                      /**< Do full device erase if the button is pressed. */

static void zigbee_command_handler(const uint8_t * p_command_str, uint16_t length);

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_pwm_values_individual_t m_demo1_seq_values;
static uint16_t const              m_demo1_top  = 255;
static nrf_pwm_sequence_t const    m_demo1_seq =
{
    .values.p_individual = &m_demo1_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_demo1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/* Declare context variable and cluster attribute list for first endpoint */
static zb_bulb_dev_ctx_t zb_dev_ctx;
ZB_DECLARE_COLOR_LIGHT_BULB_CLUSTER_ATTR_LIST(zb_dev_ctx, color_light_bulb_clusters);

/* Declare endpoint for Color Light Bulb device. */
ZB_ZCL_DECLARE_COLOR_LIGHT_EP(color_light_bulb_ep,
                              HA_COLOR_LIGHT_ENDPOINT,
                              color_light_bulb_clusters);

/* Declare application's device context (list of registered endpoints) for Color Light Bulb device. */
ZBOSS_DECLARE_DEVICE_CTX_1_EP(color_light_ctx, color_light_bulb_ep);

/* Structure for storing data to be written to Thingy's LED characteristic */
typedef PACKED_STRUCT led_params_s
{
    __PACKED union
    {
        PACKED_STRUCT
        {
            uint8_t  r_value;               /**< Red color value. */
            uint8_t  g_value;               /**< Green color value. */
            uint8_t  b_value;               /**< Blue color value. */
        };
    };
} led_params_t;

static led_params_t                led_params;                         /**< Table to store RGB color values to control thigny LED. */

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
    app_error_handler(CAFE_CAFE, line_num, p_file_name);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and
 *          pass it to the zigbee command handler.
 *
 * @param[in] p_evt  Nordic UART Service event.
 */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        zigbee_command_handler(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
    }

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


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t                              err_code;

    UNUSED_PARAMETER(p_context);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(BSP_BOARD_LED_0);
            bsp_board_led_off(BSP_BOARD_LED_1);
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


/**@brief Function for handling events from the GATT library.
 *
 * @param[in]  p_gatt  Reference to the GATT instance structure that contains status information for the GATT module.
 * @param[in]  p_evt   Reference to the GATT event structure.
 */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
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


/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
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

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
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


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
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
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
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

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the application timer.
 */
static void timer_init(void)
{
    uint32_t error_code = NRF_SUCCESS;
    error_code          = app_timer_init();
    APP_ERROR_CHECK(error_code);
}

static void rgb_color_set(uint8_t r_value, uint8_t g_value, uint8_t b_value)
{
    NRF_LOG_DEBUG("Color set = %d %d %d\n", r_value, g_value, b_value);

    m_demo1_seq_values.channel_0 = r_value;
    m_demo1_seq_values.channel_1 = g_value;
    m_demo1_seq_values.channel_2 = b_value;

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1,
                                      NRF_DRV_PWM_FLAG_LOOP);
}

static void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            LED2_R | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            LED2_G | NRF_DRV_PWM_PIN_INVERTED, // channel 1
            LED2_B | NRF_DRV_PWM_PIN_INVERTED, // channel 2
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_demo1_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    m_demo1_seq_values.channel_0 = 0;
    m_demo1_seq_values.channel_1 = 0;
    m_demo1_seq_values.channel_2 = 0;

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1,
                                      NRF_DRV_PWM_FLAG_LOOP);
}

/**@brief Function to convert hue_stauration to RGB color space.
 *
 * @param[IN]  hue          Hue value of color.
 * @param[IN]  saturation   Saturation value of color.
 * @param[IN]  brightness   Brightness value of color.
 * @param[OUT] p_led_params Pointer to structure containing parameters to write to LED characteristic
 */
static void convert_hsb_to_rgb(uint8_t hue, uint8_t saturation, uint8_t brightness, led_params_t * p_led_params)
{
    /* Check if p_leds_params is not NULL pointer */
    if (p_led_params == NULL)
    {
        NRF_LOG_INFO("Incorrect pointer to led params");
        return;
    }
    /* C, X, m are auxiliary variables */
    float C     = 0.0;
    float X     = 0.0;
    float m     = 0.0;
    /* Convertion HSB --> RGB */
    C = (brightness / 255.0f) * (saturation / 254.0f);
    X = (hue / 254.0f) * 6.0f;
    /* Casting in var X is necessary due to implementation of floating-point modulo_2 */
    /*lint -e653 */
    X = (X - (2 * (((uint8_t) X) / 2)));
    /*lint -restore */
    X -= 1.0f;
    X = C * (1.0f - ((X > 0.0f) ? (X) : (-1.0f * X)));
    m = (brightness / 255.0f) - C;

    /* Hue value is stored in range (0 - 255) instead of (0 - 360) degree */
    if (hue <= 42) /* hue < 60 degree */
    {
        p_led_params->r_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((0.0f + m) * 255.0f);
    }
    else if (hue <= 84)  /* hue < 120 degree */
    {
        p_led_params->r_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((0.0f + m) * 255.0f);
    }
    else if (hue <= 127) /* hue < 180 degree */
    {
        p_led_params->r_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((X + m) * 255.0f);
    }
    else if (hue < 170)  /* hue < 240 degree */
    {
        p_led_params->r_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((C + m) * 255.0f);
    }
    else if (hue <= 212) /* hue < 300 degree */
    {
        p_led_params->r_value = (uint8_t)((X + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((C + m) * 255.0f);
    }
    else                /* hue < 360 degree */
    {
        p_led_params->r_value = (uint8_t)((C + m) * 255.0f);
        p_led_params->g_value = (uint8_t)((0.0f + m) * 255.0f);
        p_led_params->b_value = (uint8_t)((X + m) * 255.0f);
    }
}

/**@brief Function for setting HSB color value to color LED.
 *
 * @param[IN] hue new value for hue.
 * @param[IN] saturation new value for saturation.
 * @param[IN] brightness new value for brightness.
 */
static void zb_set_hsb_color_values(zb_uint8_t hue, zb_uint8_t saturation, zb_uint8_t brightness)
{
    // convert HSB color model to RGB color model
    convert_hsb_to_rgb(hue, saturation, brightness, &led_params);

    // set RGB value to color LEDs
    rgb_color_set(led_params.r_value, led_params.g_value, led_params.b_value);
}

/**@brief Function for changing the hue of the light bulb.
 *
 * @param[IN] new_hue       New value for hue.
 */
static void color_control_set_value_hue(zb_uint8_t new_hue)
{
    NRF_LOG_INFO("Set color hue value: %i", new_hue);
    zb_set_hsb_color_values(new_hue, zb_dev_ctx.color_control_attr.set_color_info.current_saturation, zb_dev_ctx.level_control_attr.current_level);
}

/**@brief Function for changing the saturation of the light bulb.
 *
 * @param[IN] new_saturation new value for saturation.
 */
static void color_control_set_value_saturation(zb_uint8_t new_saturation)
{
    NRF_LOG_INFO("Set color saturation value: %i", new_saturation);
    zb_set_hsb_color_values(zb_dev_ctx.color_control_attr.set_color_info.current_hue, new_saturation, zb_dev_ctx.level_control_attr.current_level);
}

/**@brief Function for setting the light bulb brightness.
 *
 * @param[IN] new_level    Light bulb brightness value.
 */
static void level_control_set_value(zb_uint16_t new_level)
{
    NRF_LOG_INFO("Set level value: %i", new_level);
    zb_set_hsb_color_values(zb_dev_ctx.color_control_attr.set_color_info.current_hue, zb_dev_ctx.color_control_attr.set_color_info.current_saturation, new_level);

    /* According to the table 7.3 of Home Automation Profile Specification v 1.2 rev 29, chapter 7.1.3. */
   zb_dev_ctx.on_off_attr.on_off = (new_level ? ZB_TRUE : ZB_FALSE);
}

/**@brief Function for turning ON/OFF the light bulb.
 *
 * @param[IN] p_ep_dev_ctx Pointer to endpoint device ctx.
 * @param[IN] on           Boolean light bulb state.
 */
static void on_off_set_value(zb_bool_t on)
{
    zb_dev_ctx.on_off_attr.on_off = on;

    NRF_LOG_INFO("Set ON/OFF value: %i", on);
    if (on)
    {
        level_control_set_value(zb_dev_ctx.level_control_attr.current_level);
    }
    else
    {
        led_params.r_value = 0;
        led_params.g_value = 0;
        led_params.b_value = 0;
        rgb_color_set(led_params.r_value, led_params.g_value, led_params.b_value);
    }
}

/***************************************************************************************************
 * @section Zigbee stack related functions.
 **************************************************************************************************/

/**@brief Indicate the commissioning in progress.
 *
 * @param[in]   param   Reference to ZigBee stack buffer that will be used to construct leave request.
 */
static void zb_commissioning_indication(zb_uint8_t led_state)
{
    zb_ret_t zb_err_code;

    if(ZB_JOINED() == ZB_FALSE)
    {
        if (led_state)
        {
            led_params.r_value = 0;
            led_params.g_value = 100;
            led_params.b_value = 0;
            rgb_color_set(led_params.r_value, led_params.g_value, led_params.b_value);
            zb_err_code = ZB_SCHEDULE_ALARM(zb_commissioning_indication, 0, ZB_MILLISECONDS_TO_BEACON_INTERVAL(200));
            ZB_ERROR_CHECK(zb_err_code);
        }
        else
        {
            led_params.r_value = 0;
            led_params.g_value = 0;
            led_params.b_value = 0;
            rgb_color_set(led_params.r_value, led_params.g_value, led_params.b_value);
            zb_err_code = ZB_SCHEDULE_ALARM(zb_commissioning_indication, 1, ZB_MILLISECONDS_TO_BEACON_INTERVAL(2000));
            ZB_ERROR_CHECK(zb_err_code);
        }
    }
}

/**@brief Perform local operation - leave network.
 *
 * @param[in]   param   Reference to ZigBee stack buffer that will be used to construct leave request.
 */
static void device_leave_nwk(zb_uint8_t param)
{
    zb_ret_t zb_err_code;

    /* We are going to leave */
    if (param)
    {
        zb_buf_t                  * p_buf = ZB_BUF_FROM_REF(param);
        zb_zdo_mgmt_leave_param_t * p_req_param;

        p_req_param = ZB_GET_BUF_PARAM(p_buf, zb_zdo_mgmt_leave_param_t);
        UNUSED_RETURN_VALUE(ZB_BZERO(p_req_param, sizeof(zb_zdo_mgmt_leave_param_t)));

        /* Set dst_addr == local address for local leave */
        p_req_param->dst_addr = ZB_PIBCACHE_NETWORK_ADDRESS();
        p_req_param->rejoin   = ZB_FALSE;
        UNUSED_RETURN_VALUE(zdo_mgmt_leave_req(param, NULL));
    }
    else
    {
        zb_err_code = ZB_GET_OUT_BUF_DELAYED(device_leave_nwk);
        ZB_ERROR_CHECK(zb_err_code);
    }
}


/**@brief Function for starting join/rejoin procedure.
 *
 * param[in]   leave_type   Type of leave request (with or without rejoin).
 */
static zb_void_t device_retry_join(zb_uint8_t leave_type)
{
    zb_bool_t comm_status;

    if (leave_type == ZB_NWK_LEAVE_TYPE_RESET)
    {
        comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
        ZB_COMM_STATUS_CHECK(comm_status);
    }
}


/**@brief Function for leaving current network and starting join procedure afterwards.
 *
 * @param[in]   param   Optional reference to ZigBee stack buffer to be reused by leave and join procedure.
 */
static zb_void_t device_leave_and_join(zb_uint8_t param)
{
    if (ZB_JOINED())
    {
        /* Leave network. Joining procedure will be initiated inisde ZigBee stack signal handler. */
        device_leave_nwk(param);
    }
    else
    {
        /* Already left network. Start joining procedure. */
        device_retry_join(ZB_NWK_LEAVE_TYPE_RESET);

        if (param)
        {
            ZB_FREE_BUF_BY_REF(param);
        }
    }
}

/**@brief Callback for button events.
 *
 * @param[in]   evt      Incoming event from the BSP subsystem.
 */
static void buttons_handler(bsp_event_t evt)
{
    switch(evt)
    {
        default:
            NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
            return;
    }
}


/**@brief Function for initializing LEDs and buttons.
 */
static void leds_buttons_init(void)
{
    ret_code_t error_code;

    /* Initialize LEDs and buttons - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, buttons_handler);
    APP_ERROR_CHECK(error_code);
    bsp_board_leds_off();
}

/**@brief Function for handling commands received from the Nordic UART Service.
 *
 * @details This function will check received data agains predefined commands and
 *          generate a corresponding Zigbee request upon successfull match.
 *
 * @param[in] p_command_str  Command string received over UART.
 * @param[in] length         Length of the data.
 */
static void zigbee_command_handler(const uint8_t * p_command_str, uint16_t length)
{
    NRF_LOG_INFO("Unrecognized UART command received:");
    NRF_LOG_HEXDUMP_INFO(p_command_str, length);
}

/**@brief Function for initializing clusters attributes.
 * 
 * @param[IN]   p_device_ctx   Pointer to structure with device_ctx.
 * @param[IN]   ep_id          Endpoint ID.
 */
static void bulb_clusters_attr_init(zb_bulb_dev_ctx_t * p_device_ctx, zb_uint8_t ep_id)
{
    /* Basic cluster attributes data */
    p_device_ctx->basic_attr.zcl_version   = ZB_ZCL_VERSION;
    p_device_ctx->basic_attr.app_version   = BULB_INIT_BASIC_APP_VERSION;
    p_device_ctx->basic_attr.stack_version = BULB_INIT_BASIC_STACK_VERSION;
    p_device_ctx->basic_attr.hw_version    = BULB_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.mf_name,
                          BULB_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.model_id,
                          BULB_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.date_code,
                          BULB_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_DATE_CODE));

    p_device_ctx->basic_attr.power_source = BULB_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(p_device_ctx->basic_attr.location_id,
                          BULB_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(BULB_INIT_BASIC_LOCATION_DESC));


    p_device_ctx->basic_attr.ph_env = BULB_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    p_device_ctx->identify_attr.identify_time       = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
    p_device_ctx->identify_attr.commission_state    = ZB_ZCL_ATTR_IDENTIFY_COMMISSION_STATE_HA_ID_DEF_VALUE;

    /* On/Off cluster attributes data */
    p_device_ctx->on_off_attr.on_off                = (zb_bool_t)ZB_ZCL_ON_OFF_IS_ON;
    p_device_ctx->on_off_attr.global_scene_ctrl     = ZB_TRUE;
    p_device_ctx->on_off_attr.on_time               = 0;
    p_device_ctx->on_off_attr.off_wait_time         = 0;

    /* Level control cluster attributes data */
    p_device_ctx->level_control_attr.current_level  = ZB_ZCL_LEVEL_CONTROL_LEVEL_MAX_VALUE; // Set current level value to maximum
    p_device_ctx->level_control_attr.remaining_time = ZB_ZCL_LEVEL_CONTROL_REMAINING_TIME_DEFAULT_VALUE;
    ZB_ZCL_LEVEL_CONTROL_SET_ON_OFF_VALUE(ep_id, p_device_ctx->on_off_attr.on_off);
    ZB_ZCL_LEVEL_CONTROL_SET_LEVEL_VALUE(ep_id, p_device_ctx->level_control_attr.current_level);
    
    /* Color control cluster attributes data */
    p_device_ctx->color_control_attr.set_color_info.current_hue         = ZB_ZCL_COLOR_CONTROL_HUE_RED;
    p_device_ctx->color_control_attr.set_color_info.current_saturation  = ZB_ZCL_COLOR_CONTROL_CURRENT_SATURATION_MAX_VALUE;

    /* Set to use hue & saturation */
    p_device_ctx->color_control_attr.set_color_info.color_mode          = ZB_ZCL_COLOR_CONTROL_COLOR_MODE_HUE_SATURATION;
    p_device_ctx->color_control_attr.set_color_info.color_temperature   = ZB_ZCL_COLOR_CONTROL_COLOR_TEMPERATURE_DEF_VALUE;
    p_device_ctx->color_control_attr.set_color_info.remaining_time      = ZB_ZCL_COLOR_CONTROL_REMAINING_TIME_MIN_VALUE;
    p_device_ctx->color_control_attr.set_color_info.color_capabilities  = ZB_ZCL_COLOR_CONTROL_CAPABILITIES_HUE_SATURATION;

    /* According to ZCL spec 5.2.2.2.1.12 0x00 shall be set when CurrentHue and CurrentSaturation are used. */
    p_device_ctx->color_control_attr.set_color_info.enhanced_color_mode = 0x00;

    /* According to 5.2.2.2.1.10 execute commands when device is off. */
    p_device_ctx->color_control_attr.set_color_info.color_capabilities  = ZB_ZCL_COLOR_CONTROL_OPTIONS_EXECUTE_IF_OFF;

    /* According to ZCL spec 5.2.2.2.2 0xFF shall be set when specific value is unknown. */
    p_device_ctx->color_control_attr.set_defined_primaries_info.number_primaries = 0xff;
}

/**@brief Callback function for handling ZCL commands.
 *
 * @param[IN]   param   Reference to ZigBee stack buffer used to pass received data.
 */
static zb_void_t zcl_device_cb(zb_uint8_t param)
{
    zb_uint16_t                       cluster_id;
    zb_uint8_t                        attr_id;
    zb_buf_t                        * p_buffer          = ZB_BUF_FROM_REF(param);
    zb_zcl_device_callback_param_t  * p_device_cb_param = ZB_GET_BUF_PARAM(p_buffer, zb_zcl_device_callback_param_t);

    NRF_LOG_INFO("Received new ZCL callback %hd on endpoint %hu", p_device_cb_param->device_cb_id, p_device_cb_param->endpoint);

    /* Set default response value. */
    p_device_cb_param->status = RET_OK;

    switch (p_device_cb_param->device_cb_id)
    {
        case ZB_ZCL_LEVEL_CONTROL_SET_VALUE_CB_ID:
            level_control_set_value(p_device_cb_param->cb_param.level_control_set_value_param.new_value);
            break;

        case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
            cluster_id = p_device_cb_param->cb_param.set_attr_value_param.cluster_id;
            attr_id    = p_device_cb_param->cb_param.set_attr_value_param.attr_id;

            if (cluster_id == ZB_ZCL_CLUSTER_ID_ON_OFF)
            {
                uint8_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data8;

                if (attr_id == ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID)
                {
                    on_off_set_value((zb_bool_t)value);
                }
            }
            else if (cluster_id == ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL)
            {
                uint16_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data16;
                if (attr_id == ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID)
                {
                    level_control_set_value(value);
                }
            }
            else if (cluster_id == ZB_ZCL_CLUSTER_ID_COLOR_CONTROL)
            {
                uint16_t value = p_device_cb_param->cb_param.set_attr_value_param.values.data16;

                switch (attr_id)
                {
                    case ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID:
                        color_control_set_value_hue(value);
                        break;

                    case ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID:
                        color_control_set_value_saturation(value);
                        break;

                    default:
                        NRF_LOG_INFO("Unused attribute");
                        break;
                }
            }
            else
            {
                /* Other clusters can be processed here */
                NRF_LOG_INFO("Unhandled cluster attribute id: %d", cluster_id);
            }
            break;

        default:
            p_device_cb_param->status = RET_ERROR;
            NRF_LOG_INFO("Default case, returned error");
            break;
    }

    NRF_LOG_INFO("zcl_device_cb status: %hd", p_device_cb_param->status);
}

/**@brief ZigBee stack event handler.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass arguments (signal).
 */
void zboss_signal_handler(zb_uint8_t param)
{
    zb_zdo_app_signal_hdr_t      * p_sg_p         = NULL;
    zb_zdo_signal_leave_params_t * p_leave_params = NULL;
    zb_zdo_app_signal_type_t       sig            = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t                       status         = ZB_GET_APP_SIGNAL_STATUS(param);
    zb_ret_t                       zb_err_code;

    switch(sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == RET_OK)
            {
                /* Connected. Set level of bulb */
                NRF_LOG_INFO("Joined network successfully");
                level_control_set_value(zb_dev_ctx.level_control_attr.current_level);
            }
            else
            {
                NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                zb_err_code = ZB_SCHEDULE_ALARM(device_leave_and_join, 0, ZB_TIME_ONE_SECOND);
                ZB_ERROR_CHECK(zb_err_code);
            }
            break;

        case ZB_ZDO_SIGNAL_LEAVE:
            if (status == RET_OK)
            {
                p_leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(p_sg_p, zb_zdo_signal_leave_params_t);
                NRF_LOG_INFO("Network left. Leave type: %d", p_leave_params->leave_type);
                device_retry_join(p_leave_params->leave_type);
                /* Start ZB Commissioning Indication LED */
                ZB_SCHEDULE_CALLBACK(zb_commissioning_indication, 1);
            }
            else
            {
                NRF_LOG_ERROR("Unable to leave network. Status: %d", status);
            }
            break;

        case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            if (status != RET_OK)
            {
                NRF_LOG_WARNING("Production config is not present or invalid");
            }
            break;

        default:
            /* Unhandled signal. For more information see: zb_zdo_app_signal_type_e and zb_ret_e */
            NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}

/**@brief Function for initializing the Zigbee Stack
 */
static void zigbee_init(void)
{
    zb_ieee_addr_t ieee_addr;

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("color_light_bulb");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set up Zigbee protocol main parameters. */
    zb_set_network_router_role(IEEE_CHANNEL_MASK);
    zb_set_max_children(MAX_CHILDREN);
    zb_set_nvram_erase_at_start(bsp_button_is_pressed(ERASE_CONFIG_BUTTON) ? ZB_TRUE : ZB_FALSE);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(3000));

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&zb_dev_ctx, 0, sizeof(zb_dev_ctx)));

    /* Register color light bulb device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&color_light_ctx);

     /* Register callback for handling ZCL commands. */
    ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

    /* Init attributes for endpoints */
    bulb_clusters_attr_init(&zb_dev_ctx, HA_COLOR_LIGHT_ENDPOINT);
}


/***************************************************************************************************
* @section Main
**************************************************************************************************/


/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    zb_ret_t   zb_err_code;

    /* Initialize loging system and GPIOs. */
    log_init();
    timer_init();
    leds_buttons_init();
    pwm_init();

    /* Bluetooth initialization. */
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    /* Initialize Zigbee stack. */
    zigbee_init();

    /* Start execution. */
    NRF_LOG_INFO("BLE Zigbee dynamic color light bulb example started.");

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start();
    ZB_ERROR_CHECK(zb_err_code);

    /* Start ZB Commissioning Indication LED */
    ZB_SCHEDULE_CALLBACK(zb_commissioning_indication, 1);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}


/**
 * @}
 */
