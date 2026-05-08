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
#include "ezbee/zcl/zcl_desc.h"
#include "ezbee/zcl/zcl_general_cmd.h"
#include "ezbee/zcl/zcl_reporting.h"
#include "ezbee/zcl/cluster/basic_desc.h"
#include "ezbee/zcl/cluster/illuminance_measurement_desc.h"
#include "ezbee/zcl/cluster/power_config_desc.h"
#include "ezbee/zcl/cluster/rel_humidity_measurement_desc.h"
#include "ezbee/zcl/cluster/temperature_measurement_desc.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>


static const char *TAG = "zigbee";

static constexpr auto SENSOR_ENDPOINT = uint8_t{10};
static constexpr auto HA_TEMPERATURE_SENSOR_DEVICE_ID = uint16_t{0x0302};
static constexpr auto ZIGBEE_ALL_CHANNELS_MASK = uint32_t{0x07FFF800UL}; // channels 11..26

static constexpr auto ZB_TEMP_MULTIPLIER = int32_t{100}; // ZCL temperature: int16, 0.01 degC
static constexpr auto ZB_HUM_MULTIPLIER = int32_t{100};  // ZCL humidity: uint16, 0.01 %RH
static constexpr auto BATTERY_WARNING_MV = uint16_t{3500};
static constexpr auto BATTERY_CRITICAL_MV = uint16_t{3200};
static constexpr auto BATTERY_ALARM_STATE_OK = uint32_t{0};
static constexpr auto BATTERY_ALARM_STATE_LOW = uint32_t{0x00000001};
static constexpr auto BATTERY_ALARM_STATE_CRITICAL = uint32_t{0x00000002};

static bool s_joined = false;
static bool s_steering_retry_task_running = false;

static char s_manufacturer_name_pstr[33] = {};
static char s_model_name_pstr[33] = {};
static uint8_t s_battery_voltage = EZB_ZCL_VALUE_UINT8_NONE;
static uint32_t s_battery_alarm_state = BATTERY_ALARM_STATE_OK;

struct ReportableAttribute {
    const char *name;
    uint16_t cluster_id;
    uint16_t attr_id;
};

static constexpr ReportableAttribute TEMP_ATTR = {"temperature", EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT, EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID};
static constexpr ReportableAttribute HUM_ATTR = {"humidity", EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID};
static constexpr ReportableAttribute ILL_ATTR = {"illuminance", EZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, EZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID};
static constexpr ReportableAttribute BATTERY_VOLTAGE_ATTR = {"battery voltage", EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID};
static constexpr ReportableAttribute BATTERY_ALARM_STATE_ATTR = {"battery alarm state", EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID};
static constexpr ReportableAttribute REPORTABLE_ATTRS[] = {TEMP_ATTR, HUM_ATTR, ILL_ATTR, BATTERY_VOLTAGE_ATTR, BATTERY_ALARM_STATE_ATTR};


static void make_zcl_char_string(char *dst, size_t dst_size, const char *src)
{
    if (dst == nullptr || dst_size < 2)
        return;

    const auto max_len = dst_size - 1;
    const auto len = src ? std::min(std::strlen(src), max_len) : 0;

    dst[0] = static_cast<char>(len);
    if (len > 0)
        std::memcpy(&dst[1], src, len);
    if ((len + 1) < dst_size)
        std::memset(&dst[len + 1], 0, dst_size - len - 1);
}


static esp_err_t check_ezb(ezb_err_t err)
{
    return esp_zigbee_err_to_esp(err);
}


static ezb_zcl_reporting_info_t find_reporting_info(const ReportableAttribute &attr)
{
    return ezb_zcl_reporting_info_find(SENSOR_ENDPOINT, attr.cluster_id, EZB_ZCL_CLUSTER_SERVER, attr.attr_id, EZB_ZCL_STD_MANUF_CODE);
}


static void start_attr_reporting(const ReportableAttribute &attr)
{
    const auto info = find_reporting_info(attr);
    if (info == nullptr) {
        ESP_LOGW(TAG, "No reporting config for %s (cluster=0x%04X attr=0x%04X)", attr.name, attr.cluster_id, attr.attr_id);
        return;
    }

    const auto err = ezb_zcl_reporting_start_attr_report(info);
    if (err == EZB_ERR_NONE)
        ESP_LOGI(TAG, "Started reporting for %s (cluster=0x%04X attr=0x%04X)", attr.name, attr.cluster_id, attr.attr_id);
    else
        ESP_LOGW(TAG, "Failed to start reporting for %s (cluster=0x%04X attr=0x%04X): 0x%04X", attr.name, attr.cluster_id, attr.attr_id, err);
}


static void start_configured_reporting()
{
    for (const auto &attr : REPORTABLE_ATTRS)
        start_attr_reporting(attr);
}


static void report_attr_confirm(ezb_af_user_cnf_t *cnf, void *user_ctx)
{
    const auto *name = static_cast<const char *>(user_ctx);
    if (cnf == nullptr) {
        ESP_LOGW(TAG, "Report confirm for %s is null", name ? name : "attribute");
        return;
    }

    ESP_LOGI(TAG, "Report confirm for %s: status=0x%02X tsn=%u cluster=0x%04X src_ep=%u dst_ep=%u", name ? name : "attribute", cnf->status, cnf->tsn, cnf->cluster_id, cnf->src_ep, cnf->dst_ep);
}


static ezb_err_t report_attr(const ReportableAttribute &attr)
{
    if (find_reporting_info(attr) == nullptr)
        ESP_LOGW(TAG, "Reporting %s without local reporting config (cluster=0x%04X attr=0x%04X)", attr.name, attr.cluster_id, attr.attr_id);
    else
        ESP_LOGD(TAG, "Reporting %s with existing reporting config", attr.name);

    auto report_attr_cmd = ezb_zcl_report_attr_cmd_t{};
    report_attr_cmd.cmd_ctrl.dst_addr = EZB_ADDRESS_NONE(); // use binding table / reporting destination
    report_attr_cmd.cmd_ctrl.src_ep = SENSOR_ENDPOINT;
    report_attr_cmd.cmd_ctrl.cluster_id = attr.cluster_id;
    report_attr_cmd.cmd_ctrl.fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.cmd_ctrl.cnf_ctx.cb = report_attr_confirm;
    report_attr_cmd.cmd_ctrl.cnf_ctx.user_ctx = const_cast<char *>(attr.name);
    report_attr_cmd.payload.attr_id = attr.attr_id;

    return ezb_zcl_report_attr_cmd_req(&report_attr_cmd);
}


static ezb_zcl_status_t set_attr_value(const ReportableAttribute &attr, void *value)
{
    return ezb_zcl_set_attr_value(SENSOR_ENDPOINT, attr.cluster_id, EZB_ZCL_CLUSTER_SERVER, attr.attr_id, EZB_ZCL_STD_MANUF_CODE, value, false);
}


static void log_set_status(const ReportableAttribute &attr, ezb_zcl_status_t status)
{
    if (status == EZB_ZCL_STATUS_SUCCESS)
        ESP_LOGD(TAG, "Updated %s attribute", attr.name);
    else
        ESP_LOGW(TAG, "Failed to update %s attribute (cluster=0x%04X attr=0x%04X): 0x%02X", attr.name, attr.cluster_id, attr.attr_id, status);
}


static void report_after_successful_set(const ReportableAttribute &attr, ezb_zcl_status_t status)
{
    if (status != EZB_ZCL_STATUS_SUCCESS)
        return;

    const auto report_err = report_attr(attr);
    if (report_err != EZB_ERR_NONE)
        ESP_LOGW(TAG, "Failed to request %s report: 0x%04X", attr.name, report_err);
}


static uint8_t battery_voltage_to_zcl(uint16_t voltage_mV)
{
    return static_cast<uint8_t>(std::min<uint16_t>((voltage_mV + 50) / 100, UINT8_MAX));
}


static uint32_t battery_alarm_state_from_voltage(uint16_t voltage_mV)
{
    if (voltage_mV <= BATTERY_CRITICAL_MV)
        return BATTERY_ALARM_STATE_CRITICAL;
    if (voltage_mV <= BATTERY_WARNING_MV)
        return BATTERY_ALARM_STATE_LOW;
    return BATTERY_ALARM_STATE_OK;
}


static void start_network_steering()
{
    const auto err = ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    if (err != EZB_ERR_NONE)
        ESP_LOGW(TAG, "Failed to start network steering: 0x%04X", err);
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
    if (s_steering_retry_task_running)
        return;

    s_steering_retry_task_running = true;
    const auto ok = xTaskCreate(network_steering_retry_task, "zb_steer_retry", 3072, nullptr, 4, nullptr);
    if (ok == pdPASS)
        return;

    s_steering_retry_task_running = false;
    ESP_LOGW(TAG, "Failed to create steering retry task");
}


static bool app_signal_handler(const ezb_app_signal_t *app_signal)
{
    const auto sig_type = ezb_app_signal_get_type(app_signal);
    const auto *params = ezb_app_signal_get_params(app_signal);

    switch (sig_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        ESP_ERROR_CHECK(check_ezb(ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION)));
        return true;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        const auto *simple = static_cast<const ezb_bdb_signal_simple_params_t *>(params);
        const auto status = simple ? simple->status : EZB_BDB_STATUS_SUCCESS;

        if (status != EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack, BDB status: 0x%02X", status);
            return true;
        }

        const auto factory_new = ezb_bdb_is_factory_new();
        ESP_LOGI(TAG, "Device started up in %s factory-reset mode", factory_new ? "" : "non");

        if (factory_new) {
            ESP_LOGI(TAG, "Start network steering");
            start_network_steering();
        } else {
            s_joined = ezb_bdb_dev_joined();
            ESP_LOGI(TAG, "Device rebooted, joined=%d, rx_on_when_idle=%d", s_joined, ezb_nwk_get_rx_on_when_idle());
            if (s_joined)
                start_configured_reporting();
        }
        return true;
    }

    case EZB_BDB_SIGNAL_STEERING: {
        const auto *simple = static_cast<const ezb_bdb_signal_simple_params_t *>(params);
        const auto status = simple ? simple->status : EZB_BDB_STATUS_NO_NETWORK;

        if (status == EZB_BDB_STATUS_SUCCESS) {
            auto extended_pan_id = ezb_extpanid_t{};
            ezb_nwk_get_extended_panid(&extended_pan_id);

            s_joined = true;
            s_steering_retry_task_running = false;

            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel: %d, Short Address: 0x%04hx, rx_on_when_idle=%d)", extended_pan_id.u8[7], extended_pan_id.u8[6], extended_pan_id.u8[5], extended_pan_id.u8[4], extended_pan_id.u8[3], extended_pan_id.u8[2], extended_pan_id.u8[1], extended_pan_id.u8[0], ezb_nwk_get_panid(), ezb_nwk_get_current_channel(), ezb_nwk_get_short_address(), ezb_nwk_get_rx_on_when_idle());
            start_configured_reporting();
        } else {
            s_joined = false;
            ESP_LOGI(TAG, "Network steering was not successful, BDB status: 0x%02X", status);
            schedule_network_steering_retry();
        }
        return true;
    }

    default:
        ESP_LOGI(TAG, "Zigbee signal: %s (0x%04X)", ezb_app_signal_to_string(sig_type), sig_type);
        return false;
    }
}


static void zcl_core_set_attr_value_handler(ezb_zcl_set_attr_value_message_t *message)
{
    if (message == nullptr) {
        ESP_LOGW(TAG, "SetAttributeValue message is null");
        return;
    }

    ESP_LOGI(TAG, "SetAttributeValue: endpoint=%u cluster=0x%04X role=%s status=0x%02X attr=0x%04X size=%u", message->info.dst_ep, message->info.cluster_id, message->info.cluster_role == EZB_ZCL_CLUSTER_SERVER ? "server" : "client", message->info.status, message->in.attribute.id, message->in.attribute.data.size);
}


static void zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID:
        zcl_core_set_attr_value_handler(static_cast<ezb_zcl_set_attr_value_message_t *>(message));
        break;

    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        auto *default_rsp = static_cast<ezb_zcl_cmd_default_rsp_message_t *>(message);
        if (default_rsp != nullptr)
            ESP_LOGI(TAG, "Default response: endpoint=%u cluster=0x%04X status=0x%02X", default_rsp->info.dst_ep, default_rsp->info.cluster_id, default_rsp->in.status_code);
        break;
    }

    default:
        ESP_LOGW(TAG, "Unhandled ZCL core action callback: 0x%04lX", static_cast<unsigned long>(callback_id));
        break;
    }
}


static ezb_zcl_cluster_desc_t create_basic_cluster(uint8_t power_source, const char *manufacturer_name, const char *model_name)
{
    auto basic_cfg = ezb_zcl_basic_cluster_server_config_t{};
    basic_cfg.zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    basic_cfg.power_source = power_source;

    auto cluster = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(cluster != nullptr, nullptr, TAG, "Failed to create Basic cluster");

    if (manufacturer_name != nullptr) {
        make_zcl_char_string(s_manufacturer_name_pstr, sizeof(s_manufacturer_name_pstr), manufacturer_name);
        ESP_ERROR_CHECK(check_ezb(ezb_zcl_basic_cluster_desc_add_attr(cluster, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, s_manufacturer_name_pstr)));
    }

    if (model_name != nullptr) {
        make_zcl_char_string(s_model_name_pstr, sizeof(s_model_name_pstr), model_name);
        ESP_ERROR_CHECK(check_ezb(ezb_zcl_basic_cluster_desc_add_attr(cluster, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, s_model_name_pstr)));
    }

    return cluster;
}


static esp_err_t add_reportable_attr(ezb_zcl_cluster_desc_t cluster, uint16_t attr_id, uint8_t attr_type, const void *value)
{
    auto attr_desc = ezb_zcl_create_attr_desc(attr_id, attr_type, EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_REPORTING, EZB_ZCL_STD_MANUF_CODE, value);
    ESP_RETURN_ON_FALSE(attr_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create reportable attribute 0x%04X", attr_id);

    const auto err = ezb_zcl_cluster_add_attr_desc(cluster, attr_desc);
    if (err != EZB_ERR_NONE) {
        ezb_zcl_free_attr_desc(attr_desc);
        ESP_LOGE(TAG, "Failed to add reportable attribute 0x%04X: 0x%04X", attr_id, err);
        return check_ezb(err);
    }

    return ESP_OK;
}


static ezb_zcl_cluster_desc_t create_power_config_cluster()
{
    auto power_cfg = ezb_zcl_power_config_cluster_server_config_t{};
    power_cfg.mains_voltage = EZB_ZCL_POWER_CONFIG_MAINS_VOLTAGE_DEFAULT_VALUE;
    power_cfg.mains_voltage_min_threshold = EZB_ZCL_POWER_CONFIG_MAINS_VOLTAGE_MIN_THRESHOLD_DEFAULT_VALUE;
    power_cfg.mains_voltage_max_threshold = EZB_ZCL_POWER_CONFIG_MAINS_VOLTAGE_MAX_THRESHOLD_DEFAULT_VALUE;

    auto cluster = ezb_zcl_power_config_create_cluster_desc(&power_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(cluster != nullptr, nullptr, TAG, "Failed to create Power Configuration cluster");
    ESP_ERROR_CHECK(add_reportable_attr(cluster, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, EZB_ZCL_ATTR_TYPE_UINT8, &s_battery_voltage));
    ESP_ERROR_CHECK(add_reportable_attr(cluster, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_ALARM_STATE_ID, EZB_ZCL_ATTR_TYPE_MAP32, &s_battery_alarm_state));

    return cluster;
}


static esp_err_t create_sensor_endpoint(uint8_t ep_id)
{
    auto dev_desc = ezb_af_create_device_desc();
    ESP_RETURN_ON_FALSE(dev_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create Zigbee device descriptor");

    auto ep_config = ezb_af_ep_config_t{};
    ep_config.ep_id = ep_id;
    ep_config.app_profile_id = EZB_AF_HA_PROFILE_ID;
    ep_config.app_device_id = HA_TEMPERATURE_SENSOR_DEVICE_ID;
    ep_config.app_device_version = 0;

    auto ep_desc = ezb_af_create_endpoint_desc(&ep_config);
    ESP_RETURN_ON_FALSE(ep_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create Zigbee endpoint descriptor");

    auto basic_desc = create_basic_cluster(EZB_ZCL_BASIC_POWER_SOURCE_BATTERY, "embedblog", "ESP32H2-THISensor");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, basic_desc)));

    auto power_desc = create_power_config_cluster();
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, power_desc)));

    auto temp_cfg = ezb_zcl_temperature_measurement_cluster_server_config_t{};
    temp_cfg.measured_value = EZB_ZCL_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_DEFAULT_VALUE;
    temp_cfg.min_measured_value = static_cast<int16_t>(-10 * ZB_TEMP_MULTIPLIER);
    temp_cfg.max_measured_value = static_cast<int16_t>(80 * ZB_TEMP_MULTIPLIER);

    auto temp_desc = ezb_zcl_temperature_measurement_create_cluster_desc(&temp_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(temp_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create temperature cluster");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, temp_desc)));

    auto hum_cfg = ezb_zcl_rel_humidity_measurement_cluster_server_config_t{};
    hum_cfg.measured_value = EZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_DEFAULT_VALUE;
    hum_cfg.min_measured_value = static_cast<uint16_t>(0 * ZB_HUM_MULTIPLIER);
    hum_cfg.max_measured_value = static_cast<uint16_t>(100 * ZB_HUM_MULTIPLIER);

    auto hum_desc = ezb_zcl_rel_humidity_measurement_create_cluster_desc(&hum_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(hum_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create humidity cluster");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, hum_desc)));

    auto ill_cfg = ezb_zcl_illuminance_measurement_cluster_server_config_t{};
    ill_cfg.measured_value = EZB_ZCL_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_DEFAULT_VALUE;
    ill_cfg.min_measured_value = 0;
    ill_cfg.max_measured_value = 65000;

    auto ill_desc = ezb_zcl_illuminance_measurement_create_cluster_desc(&ill_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_RETURN_ON_FALSE(ill_desc != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create illuminance cluster");
    ESP_ERROR_CHECK(check_ezb(ezb_af_endpoint_add_cluster_desc(ep_desc, ill_desc)));

    ESP_ERROR_CHECK(check_ezb(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc)));
    ESP_ERROR_CHECK(check_ezb(ezb_af_device_desc_register(dev_desc)));

    return ESP_OK;
}


static void esp_zigbee_task(void *)
{
    auto config = esp_zigbee_config_t{};

    config.device_config.device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE;
    config.device_config.install_code_policy = false;
    config.device_config.zed_config.ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN;
    config.device_config.zed_config.keep_alive = 3000;

    config.platform_config.radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE;
    config.platform_config.storage_partition_name = "zb_storage";

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

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


void zigbee::startTask()
{
    xTaskCreate(esp_zigbee_task, "Zigbee_main", 6144, nullptr, 5, nullptr);
}


void zigbee::updateTempHum(int16_t temp, uint16_t hum)
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGD(TAG, "Skip temp/hum update: Zigbee not joined");
        return;
    }

    esp_zigbee_lock_acquire(portMAX_DELAY);

    const auto temp_status = set_attr_value(TEMP_ATTR, &temp);
    const auto hum_status = set_attr_value(HUM_ATTR, &hum);

    report_after_successful_set(TEMP_ATTR, temp_status);
    report_after_successful_set(HUM_ATTR, hum_status);

    esp_zigbee_lock_release();

    log_set_status(TEMP_ATTR, temp_status);
    log_set_status(HUM_ATTR, hum_status);
}


void zigbee::updateIlluminance(uint32_t ill_mLx)
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGD(TAG, "Skip illuminance update: Zigbee not joined");
        return;
    }

    auto illum = uint16_t{0};

    if (ill_mLx == 0)
        illum = 0;
    else {
        const auto lux = ill_mLx / 1000.0f;
        const auto raw = 10000.0f * std::log10(lux) + 1.0f;

        if (raw <= 0.0f)
            illum = 0;
        else if (raw >= 65534.0f)
            illum = 0xFFFE;
        else
            illum = static_cast<uint16_t>(std::round(raw));
    }

    esp_zigbee_lock_acquire(portMAX_DELAY);

    const auto status = set_attr_value(ILL_ATTR, &illum);
    report_after_successful_set(ILL_ATTR, status);

    esp_zigbee_lock_release();

    log_set_status(ILL_ATTR, status);
}


void zigbee::updateBattery(uint16_t voltage_mV)
{
    if (!ezb_bdb_dev_joined()) {
        ESP_LOGD(TAG, "Skip battery update: Zigbee not joined");
        return;
    }

    auto voltage = battery_voltage_to_zcl(voltage_mV);
    auto alarm_state = battery_alarm_state_from_voltage(voltage_mV);

    esp_zigbee_lock_acquire(portMAX_DELAY);

    const auto voltage_status = set_attr_value(BATTERY_VOLTAGE_ATTR, &voltage);
    const auto alarm_status = set_attr_value(BATTERY_ALARM_STATE_ATTR, &alarm_state);

    report_after_successful_set(BATTERY_VOLTAGE_ATTR, voltage_status);
    report_after_successful_set(BATTERY_ALARM_STATE_ATTR, alarm_status);

    esp_zigbee_lock_release();

    ESP_LOGI(TAG, "Battery voltage %u mV -> ZCL %u, alarm_state=0x%08lX", static_cast<unsigned>(voltage_mV), static_cast<unsigned>(voltage), static_cast<unsigned long>(alarm_state));
    log_set_status(BATTERY_VOLTAGE_ATTR, voltage_status);
    log_set_status(BATTERY_ALARM_STATE_ATTR, alarm_status);
}
