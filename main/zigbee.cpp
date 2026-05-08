#include "zigbee.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"

#include "ezbee/af.h"
#include "ezbee/app_signals.h"
#include "ezbee/bdb.h"
#include "ezbee/nwk.h"
#include "ezbee/zcl/zcl_common.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zcl/cluster/basic_desc.h"
#include "ezbee/zcl/cluster/illuminance_measurement_desc.h"
#include "ezbee/zcl/cluster/rel_humidity_measurement_desc.h"
#include "ezbee/zcl/cluster/temperature_measurement_desc.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>


static const char *TAG = "zigbee";

static constexpr uint8_t SENSOR_ENDPOINT = 10;
static constexpr uint16_t HA_TEMPERATURE_SENSOR_DEVICE_ID = 0x0302;
static constexpr uint32_t ZIGBEE_ALL_CHANNELS_MASK = 0x07FFF800UL; // channels 11..26

constexpr int32_t ZB_TEMP_MULTIPLIER = 100; // ZCL temperature: int16, 0.01 degC
constexpr int32_t ZB_HUM_MULTIPLIER = 100;  // ZCL humidity: uint16, 0.01 %RH

static bool s_joined = false;
static bool s_steering_retry_task_running = false;

static char s_manufacturer_name_pstr[33] = {};
static char s_model_name_pstr[33] = {};


static void make_zcl_char_string(char *dst, size_t dst_size, const char *src)
{
    if (dst == nullptr || dst_size < 2) {
        return;
    }

    const size_t max_len = dst_size - 1;
    const size_t len = src ? std::min(std::strlen(src), max_len) : 0;

    dst[0] = static_cast<char>(len);
    if (len > 0) {
        std::memcpy(&dst[1], src, len);
    }
    if ((len + 1) < dst_size) {
        std::memset(&dst[len + 1], 0, dst_size - len - 1);
    }
}


static esp_err_t check_ezb(ezb_err_t err)
{
    return esp_zigbee_err_to_esp(err);
}


static void start_network_steering()
{
    const ezb_err_t err = ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    if (err != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Failed to start network steering: 0x%04X", err);
    }
}


static void network_steering_retry_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    if (!ezb_bdb_dev_joined()) {
        ESP_LOGI(TAG, "Retry network steering");
        if (esp_zigbee_lock_acquire(portMAX_DELAY)) {
            start_network_steering();
            esp_zigbee_lock_release();
        }
    }

    s_steering_retry_task_running = false;
    vTaskDelete(nullptr);
}


static void schedule_network_steering_retry()
{
    if (s_steering_retry_task_running) {
        return;
    }

    s_steering_retry_task_running = true;
    const BaseType_t ok = xTaskCreate(network_steering_retry_task,
                                      "zb_steer_retry",
                                      3072,
                                      nullptr,
                                      4,
                                      nullptr);
    if (ok != pdPASS) {
        s_steering_retry_task_running = false;
        ESP_LOGW(TAG, "Failed to create steering retry task");
    }
}


static bool app_signal_handler(const ezb_app_signal_t *app_signal)
{
    const ezb_app_signal_type_t sig_type = ezb_app_signal_get_type(app_signal);
    const void *params = ezb_app_signal_get_params(app_signal);

    switch (sig_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        ESP_ERROR_CHECK(check_ezb(ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION)));
        return true;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        const auto *simple = static_cast<const ezb_bdb_signal_simple_params_t *>(params);
        const uint8_t status = simple ? simple->status : EZB_BDB_STATUS_SUCCESS;

        if (status == EZB_BDB_STATUS_SUCCESS) {
            const bool factory_new = ezb_bdb_is_factory_new();
            ESP_LOGI(TAG,
                     "Device started up in %s factory-reset mode",
                     factory_new ? "" : "non");

            if (factory_new) {
                ESP_LOGI(TAG, "Start network steering");
                start_network_steering();
            } else {
                s_joined = ezb_bdb_dev_joined();
                ESP_LOGI(TAG,
                         "Device rebooted, joined=%d, rx_on_when_idle=%d",
                         s_joined,
                         ezb_nwk_get_rx_on_when_idle());
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack, BDB status: 0x%02X", status);
        }
        return true;
    }

    case EZB_BDB_SIGNAL_STEERING: {
        const auto *simple = static_cast<const ezb_bdb_signal_simple_params_t *>(params);
        const uint8_t status = simple ? simple->status : EZB_BDB_STATUS_NO_NETWORK;

        if (status == EZB_BDB_STATUS_SUCCESS) {
            ezb_extpanid_t extended_pan_id = {};
            ezb_nwk_get_extended_panid(&extended_pan_id);

            s_joined = true;
            s_steering_retry_task_running = false;

            ESP_LOGI(TAG,
                     "Joined network successfully (Extended PAN ID: "
                     "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, "
                     "PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx, "
                     "rx_on_when_idle=%d)",
                     extended_pan_id.u8[7], extended_pan_id.u8[6], extended_pan_id.u8[5], extended_pan_id.u8[4],
                     extended_pan_id.u8[3], extended_pan_id.u8[2], extended_pan_id.u8[1], extended_pan_id.u8[0],
                     ezb_nwk_get_panid(),
                     ezb_nwk_get_current_channel(),
                     ezb_nwk_get_short_address(),
                     ezb_nwk_get_rx_on_when_idle());
        } else {
            s_joined = false;
            ESP_LOGI(TAG, "Network steering was not successful, BDB status: 0x%02X", status);
            schedule_network_steering_retry();
        }
        return true;
    }

    default:
        ESP_LOGI(TAG,
                 "Zigbee signal: %s (0x%04X)",
                 ezb_app_signal_to_string(sig_type),
                 sig_type);
        return false;
    }
}


static void zcl_core_set_attr_value_handler(ezb_zcl_set_attr_value_message_t *message)
{
    if (message == nullptr) {
        ESP_LOGW(TAG, "SetAttributeValue message is null");
        return;
    }

    ESP_LOGI(TAG,
             "SetAttributeValue: endpoint=%u cluster=0x%04X role=%s status=0x%02X attr=0x%04X size=%u",
             message->info.dst_ep,
             message->info.cluster_id,
             message->info.cluster_role == EZB_ZCL_CLUSTER_SERVER ? "server" : "client",
             message->info.status,
             message->in.attribute.id,
             message->in.attribute.data.size);
}


static void zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        zcl_core_set_attr_value_handler(static_cast<ezb_zcl_set_attr_value_message_t *>(message));
        break;

    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        auto *default_rsp = static_cast<ezb_zcl_cmd_default_rsp_message_t *>(message);
        if (default_rsp != nullptr) {
            ESP_LOGI(TAG,
                     "Default response: endpoint=%u cluster=0x%04X status=0x%02X",
                     default_rsp->info.dst_ep,
                     default_rsp->info.cluster_id,
                     default_rsp->in.status_code);
        }
        break;
    }

    default:
        ESP_LOGW(TAG, "Unhandled ZCL core action callback: 0x%04lX", callback_id);
        break;
    }
}


static ezb_zcl_cluster_desc_t create_basic_cluster(uint8_t power_source,
                                                   const char *manufacturer_name,
                                                   const char *model_name)
{
    ezb_zcl_basic_cluster_server_config_t basic_cfg = {};
    basic_cfg.zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    basic_cfg.power_source = power_source;

    ezb_zcl_cluster_desc_t cluster = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(cluster != nullptr, nullptr, TAG, "Failed to create Basic cluster");

    if (manufacturer_name != nullptr) {
        make_zcl_char_string(s_manufacturer_name_pstr, sizeof(s_manufacturer_name_pstr), manufacturer_name);
        ESP_ERROR_CHECK(check_ezb(ezb_zcl_basic_cluster_desc_add_attr(
            cluster,
            EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
            s_manufacturer_name_pstr)));
    }

    if (model_name != nullptr) {
        make_zcl_char_string(s_model_name_pstr, sizeof(s_model_name_pstr), model_name);
        ESP_ERROR_CHECK(check_ezb(ezb_zcl_basic_cluster_desc_add_attr(
            cluster,
            EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
            s_model_name_pstr)));
    }

    return cluster;
}


static esp_err_t create_sensor_endpoint(uint8_t ep_id)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    ESP_RETURN_ON_FALSE(dev_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create Zigbee device descriptor");

    ezb_af_ep_config_t ep_config = {};
    ep_config.ep_id = ep_id;
    ep_config.app_profile_id = EZB_AF_HA_PROFILE_ID;
    ep_config.app_device_id = HA_TEMPERATURE_SENSOR_DEVICE_ID;
    ep_config.app_device_version = 0;

    ezb_af_ep_desc_t ep_desc = ezb_af_create_endpoint_desc(&ep_config);
    ESP_RETURN_ON_FALSE(ep_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create Zigbee endpoint descriptor");

    // Basic cluster
    ezb_zcl_cluster_desc_t basic_desc = create_basic_cluster(
        EZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
        "embedblog",
        "ESP32H2-THISensor");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, basic_desc)));

    // Temperature Measurement cluster
    ezb_zcl_temperature_measurement_cluster_server_config_t temp_cfg = {};
    temp_cfg.measured_value = EZB_ZCL_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_DEFAULT_VALUE;
    temp_cfg.min_measured_value = static_cast<int16_t>(-10 * ZB_TEMP_MULTIPLIER);
    temp_cfg.max_measured_value = static_cast<int16_t>(80 * ZB_TEMP_MULTIPLIER);

    ezb_zcl_cluster_desc_t temp_desc = ezb_zcl_temperature_measurement_create_cluster_desc(
        &temp_cfg,
        EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(temp_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create temperature cluster");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, temp_desc)));

    // Relative Humidity Measurement cluster
    ezb_zcl_rel_humidity_measurement_cluster_server_config_t hum_cfg = {};
    hum_cfg.measured_value = EZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT_VALUE;
    hum_cfg.min_measured_value = static_cast<uint16_t>(0 * ZB_HUM_MULTIPLIER);
    hum_cfg.max_measured_value = static_cast<uint16_t>(100 * ZB_HUM_MULTIPLIER);

    ezb_zcl_cluster_desc_t hum_desc = ezb_zcl_rel_humidity_measurement_create_cluster_desc(
        &hum_cfg,
        EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(hum_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create humidity cluster");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, hum_desc)));

    // Illuminance Measurement cluster
    ezb_zcl_illuminance_measurement_cluster_server_config_t ill_cfg = {};
    ill_cfg.measured_value = EZB_ZCL_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_DEFAULT_VALUE;
    ill_cfg.min_measured_value = 0;
    ill_cfg.max_measured_value = 65000;

    ezb_zcl_cluster_desc_t ill_desc = ezb_zcl_illuminance_measurement_create_cluster_desc(
        &ill_cfg,
        EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(ill_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create illuminance cluster");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, ill_desc)));

    ESP_ERROR_CHECK(check_ezb(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc)));
    ESP_ERROR_CHECK(check_ezb(ezb_af_device_desc_register(dev_desc)));

    return ESP_OK;
}


static ezb_err_t report_attr(uint16_t cluster_id, uint16_t attr_id)
{
    ezb_zcl_report_attr_cmd_t report_attr_cmd = {};
    report_attr_cmd.cmd_ctrl.dst_addr = EZB_ADDRESS_NONE(); // use binding table / reporting destination
    report_attr_cmd.cmd_ctrl.src_ep = SENSOR_ENDPOINT;
    report_attr_cmd.cmd_ctrl.cluster_id = cluster_id;
    report_attr_cmd.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.payload.attr_id = attr_id;

    return ezb_zcl_report_attr_cmd_req(&report_attr_cmd);
}


static void esp_zigbee_task(void *)
{
    esp_zigbee_config_t config = {};

    config.device_config.device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE;
    config.device_config.install_code_policy = false;
    config.device_config.zed_config.ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN;
    config.device_config.zed_config.keep_alive = 3000;

    config.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;
    config.platform_config.storage_partition_name = "zb_storage";

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    // v2.x sleep is controlled by ESP-IDF PM/tickless-idle. Make the NWK device sleepy by RX-off-when-idle.
    ezb_nwk_set_rx_on_when_idle(false);

    ESP_ERROR_CHECK(check_ezb(ezb_bdb_set_primary_channel_set(ZIGBEE_ALL_CHANNELS_MASK)));
    ESP_ERROR_CHECK(create_sensor_endpoint(SENSOR_ENDPOINT));

    ESP_ERROR_CHECK(check_ezb(ezb_app_signal_add_handler(app_signal_handler)));
    ezb_zcl_core_action_handler_register(zcl_core_action_handler);

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    ESP_ERROR_CHECK(esp_zigbee_launch_mainloop());

    ESP_ERROR_CHECK(esp_zigbee_deinit());
    vTaskDelete(nullptr);
}

/**
 * @brief Start Zigbee task.
 *
 * @pre nvs_flash_init() must be called before this.
 */
void zigbee::startTask()
{
    xTaskCreate(esp_zigbee_task, "Zigbee_main", 6144, nullptr, 5, nullptr);
}


/**
 * @brief Update temperature and humidity attributes.
 *
 * @param temp Temperature in 0.01 degC, e.g. 2534 means 25.34 degC.
 * @param hum  Humidity in 0.01 %, e.g. 4534 means 45.34 %RH.
 */
void zigbee::updateTempHum(int16_t temp, uint16_t hum)
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGD(TAG, "Skip temp/hum update: Zigbee not joined");
        return;
    }

    esp_zigbee_lock_acquire(portMAX_DELAY);

    const ezb_zcl_status_t temp_status = ezb_zcl_set_attr_value(
        SENSOR_ENDPOINT,
        EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
        EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
        EZB_ZCL_STD_MANUF_CODE,
        &temp,
        false);

    const ezb_zcl_status_t hum_status = ezb_zcl_set_attr_value(
        SENSOR_ENDPOINT,
        EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
        EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID,
        EZB_ZCL_STD_MANUF_CODE,
        &hum,
        false);

    if (temp_status == EZB_ZCL_STATUS_SUCCESS) {
        const ezb_err_t report_err = report_attr(
            EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID);
        if (report_err != EZB_ERR_NONE) {
            ESP_LOGW(TAG, "Failed to report temperature: 0x%04X", report_err);
        }
    }

    if (hum_status == EZB_ZCL_STATUS_SUCCESS) {
        const ezb_err_t report_err = report_attr(
            EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
            EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID);
        if (report_err != EZB_ERR_NONE) {
            ESP_LOGW(TAG, "Failed to report humidity: 0x%04X", report_err);
        }
    }

    esp_zigbee_lock_release();

    if (temp_status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update temperature attribute: 0x%02X", temp_status);
    }
    if (hum_status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update humidity attribute: 0x%02X", hum_status);
    }
}


void zigbee::updateIlluminance(uint32_t ill_mLx)
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGD(TAG, "Skip illuminance update: Zigbee not joined");
        return;
    }

    uint16_t illum = 0;

    if (ill_mLx == 0) {
        illum = 0; // too low to measure
    } else {
        const float lux = ill_mLx / 1000.0f;
        const float raw = 10000.0f * std::log10(lux) + 1.0f;

        if (raw <= 0.0f) {
            illum = 0;
        } else if (raw >= 65534.0f) {
            illum = 0xFFFE;
        } else {
            illum = static_cast<uint16_t>(std::round(raw));
        }
    }

    esp_zigbee_lock_acquire(portMAX_DELAY);

    const ezb_zcl_status_t status = ezb_zcl_set_attr_value(
        SENSOR_ENDPOINT,
        EZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
        EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        EZB_ZCL_STD_MANUF_CODE,
        &illum,
        false);

    if (status == EZB_ZCL_STATUS_SUCCESS) {
        const ezb_err_t report_err = report_attr(
            EZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
            EZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID);
        if (report_err != EZB_ERR_NONE) {
            ESP_LOGW(TAG, "Failed to report illuminance: 0x%04X", report_err);
        }
    }

    esp_zigbee_lock_release();

    if (status != EZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to update illuminance attribute: 0x%02X", status);
    }
}
