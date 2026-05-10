#include "zigbee.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_zigbee_core.h"
#include "esp_check.h"
#include "esp_log.h"

#include <cstring>

//this is missing for some reason
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID 0x0020
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_INVALID 0xFF
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID 0x0021
#define ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_INVALID 0xFF


static const char *TAG = "zigbee";
constexpr int32_t ZB_TEMP_MULTIPLIER = 100;		//the ZCL spec defines the temperature attribute as a signed 16 bit integer with a resolution of 0.01 °C
constexpr int32_t ZB_HUM_MULTIPLIER = 100;		//the ZCL spec defines the humidity attribute as a unsigned 16 bit integer with a resolution of 0.01 %


static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}


void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct)
{
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)(*(signal_struct->p_app_signal));

    switch (sig_type) 
    {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Zigbee stack initialized");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) 
            {
                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) 
                {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } 
                else 
                    ESP_LOGI(TAG, "Device rebooted");
            } 
            else
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));       // commissioning failed
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) 
            {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                        extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                        extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                        esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            } 
            else 
            {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
		case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
			esp_zb_sleep_now();
			break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
            break;
    }
}


static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) 
    {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
		{
			esp_zb_zcl_set_attr_value_message_t* setAtrMsg = (esp_zb_zcl_set_attr_value_message_t*)message;
			ESP_LOGI(TAG, "Received message: endpoint %d, cluster 0x%x, attribute 0x%x, data size %d", setAtrMsg->info.dst_endpoint, setAtrMsg->info.cluster,
             	setAtrMsg->attribute.id, setAtrMsg->attribute.data.size);
			break;
		}
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
		{
            esp_zb_zcl_cmd_default_resp_message_t* msg = (esp_zb_zcl_cmd_default_resp_message_t*)message;
            ESP_LOGI(TAG, "Default response callback: dst 0x%x, status: %u", msg->info.dst_address, msg->status_code);
            break;
		}
        default:
            ESP_LOGW(TAG, "Unhandled Zigbee action 0x%x callback", callback_id);
            break;
    }
    return ESP_OK;
}


//for a good list of the possible attributes, see https://www.nxp.com/docs/en/user-guide/JN-UG-3115.pdf
esp_zb_attribute_list_t* createBasicCluster(uint8_t powerSource, const char* manufacturerName, const char* modelName)
{
	esp_zb_basic_cluster_cfg_s basicConfig = 
	{
		.zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
		.power_source = powerSource
	};
	auto cluster = esp_zb_basic_cluster_create(&basicConfig);		//this creates all three mandatory attributes

	char strBuf[34];
	if (manufacturerName)
	{
		strncpy(&strBuf[1], manufacturerName, 32);
		strBuf[0] = strlen(manufacturerName);
		esp_zb_basic_cluster_add_attr(cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, strBuf);
	}
	if (modelName)
	{
		strncpy(&strBuf[1], modelName, 32);
		strBuf[0] = strlen(modelName);
		esp_zb_basic_cluster_add_attr(cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, strBuf);
	}

	return cluster;
}


static void esp_zb_task(void *pvParameters)
{
    esp_zb_platform_config_t config = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    esp_zb_sleep_enable(true);
    
    esp_zb_cfg_t zb_nwk_cfg = {};
    zb_nwk_cfg.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
    zb_nwk_cfg.install_code_policy = false;
    zb_nwk_cfg.nwk_cfg.zed_cfg = {
        .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
        .keep_alive = 10000,
    };
    
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_set_rx_on_when_idle(false);
    esp_zb_sleep_set_threshold(20);

    //create & populate cluster list
    esp_zb_cluster_list_t* cluster_list = esp_zb_zcl_cluster_list_create();		
	esp_zb_cluster_list_add_basic_cluster(cluster_list, createBasicCluster(ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY, "embedblog", "ESP32H2-THISensor"), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //add temperature
    esp_zb_temperature_meas_cluster_cfg_s tempConfig;
	tempConfig.min_value = -10 * ZB_TEMP_MULTIPLIER;
	tempConfig.max_value = 80 * ZB_TEMP_MULTIPLIER;
	tempConfig.measured_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_UNKNOWN;
	esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&tempConfig), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //add humidity
    esp_zb_humidity_meas_cluster_cfg_s humConfig;
    humConfig.min_value = 0 * ZB_HUM_MULTIPLIER;
    humConfig.max_value = 100 * ZB_HUM_MULTIPLIER;
    humConfig.measured_value = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_UNKNOWN;
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(&humConfig), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //add illuminance
    esp_zb_illuminance_meas_cluster_cfg_s illConfig;
    illConfig.min_value = 0;
    illConfig.max_value = 65'000;
    illConfig.measured_value = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_INVALID;
    esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster_create(&illConfig), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
	
    //add battery measurement
    auto pwrAttributes = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    static uint8_t s_battVolt = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_INVALID;
    esp_zb_cluster_add_attr(pwrAttributes, 
        ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &s_battVolt);
    static uint8_t s_battPct = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_INVALID;
    esp_zb_cluster_add_attr(pwrAttributes, 
        ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
        ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        &s_battPct);
    esp_zb_cluster_list_add_power_config_cluster(cluster_list, pwrAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	//create endpoint list and populate it
	esp_zb_ep_list_t* ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpointConfig;
	endpointConfig.endpoint = 10;
	endpointConfig.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID;
	endpointConfig.app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID;
	esp_zb_ep_list_add_ep(ep_list, cluster_list, endpointConfig);
    esp_zb_device_register(ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}


/**
 * @brief 
 * 
 * @pre         nvs_flash_init() must be called before this.
 */
void zigbee::startTask()
{
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}


/**
 * @brief 
 * 
 * @param temp      Temperature in 0.01 °C, e.g. 2534 means 25.34 °C
 * @param hum       Humidity in 0.01 %, e.g. 4534 means 45.34 %
 */
void zigbee::updateTempHum(int16_t temp, uint16_t hum)
{    
    esp_zb_lock_acquire(portMAX_DELAY);
    const auto stat1 = esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temp, false);
    const auto stat2 = esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &hum, false);
    esp_zb_lock_release();

    if (stat1 != ESP_ZB_ZCL_STATUS_SUCCESS)
        ESP_LOGW(TAG, "Failed to update temperature attribute value: 0x%04X", stat1);
    if (stat2 != ESP_ZB_ZCL_STATUS_SUCCESS)
        ESP_LOGW(TAG, "Failed to update humidity attribute value: 0x%04X", stat2);
}


void zigbee::updateIlluminance(uint32_t ill_mLx)
{
    //this could be handled without using floats, but using floats increased code size by 1.5 kB, so it does not matter
    
    float lux = ill_mLx / 1000.0f;		//convert to lx
    float raw = 10000.0f * std::log10(lux) + 1.0f;
    uint16_t illum = std::round(raw);
    
    esp_zb_lock_acquire(portMAX_DELAY);
    const auto stat = esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &illum, false);
    esp_zb_lock_release();

    if (stat != ESP_ZB_ZCL_STATUS_SUCCESS)
        ESP_LOGW(TAG, "Failed to update illuminance attribute value: 0x%04X", stat);
}


void zigbee::updateVbat(uint16_t vbat_mV, uint8_t vbat_pct)
{
    uint8_t vbat = (vbat_mV + 50) / 100;
    vbat_pct *= 2;
    
    esp_zb_lock_acquire(portMAX_DELAY);
    const auto stat = esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &vbat, false);
    const auto stat_pct = esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &vbat_pct, false);
    esp_zb_lock_release();

    if (stat != ESP_ZB_ZCL_STATUS_SUCCESS)
        ESP_LOGW(TAG, "Failed to update battery voltage attribute value: 0x%04X", stat);
    if (stat_pct != ESP_ZB_ZCL_STATUS_SUCCESS)
        ESP_LOGW(TAG, "Failed to update battery percentage attribute value: 0x%04X", stat_pct);
}