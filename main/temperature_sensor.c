/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h" // Для продуктового таймера переподключения

#include "esp_zigbee.h"
#include "ezbee/zha.h"

#include "driver/gpio.h"
#include "onewire_bus.h"
#include "ds18b20.h"

#include "temperature_sensor.h"

static const char *TAG = "TEMPERATURE_SENSOR";

#define ONEWIRE_GPIO GPIO_NUM_0
static ds18b20_device_handle_t ds18b20 = NULL;

#define ESP_TEMP_SENSOR_MIN_VALUE       (-10)
#define ESP_TEMP_SENSOR_MAX_VALUE       (80)

static int16_t zb_temperature_to_s16(float temp) { return (int16_t)(temp * 100); }
static esp_timer_handle_t rejoin_timer;
static bool is_rejoining = false; // Флаг, чтобы не запускать таймер повторно
static int64_t last_parent_link_failure_log = 0;

static void rejoin_timer_callback(void* arg)
{
    ESP_LOGI(TAG, "Таймер сработал: попытка восстановления (INITIALIZATION)...");
    is_rejoining = false; // Сбрасываем флаг
    
    esp_zigbee_lock_acquire(portMAX_DELAY);
    // Вместо STEERING вызываем INITIALIZATION
    (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
    esp_zigbee_lock_release();
}

static void init_rejoin_timer(void)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &rejoin_timer_callback,
        .name = "zigbee_rejoin"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &rejoin_timer));
}

static ezb_err_t config_temp_reporting(uint8_t ep_id)
{
    ezb_err_t ret = EZB_ERR_NONE;
    
    ezb_zcl_reporting_info_t report_info = ezb_zcl_reporting_info_find(
        ep_id,
        EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
        EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
        EZB_ZCL_STD_MANUF_CODE
    );

    if (report_info == EZB_ZCL_INVALID_REPORTING_INFO) {
        ESP_LOGE(TAG, "Слот для отчета НЕ найден!");
        return ESP_FAIL;
    }

    ezb_zcl_attr_variable_t delta;
    delta.s16 = 10; // 0.1 °C

    esp_zigbee_lock_acquire(portMAX_DELAY);
    ret = ezb_zcl_reporting_info_update(report_info, 60, 600, &delta);
    esp_zigbee_lock_release();

    if (ret == EZB_ERR_NONE) {
        ESP_LOGI(TAG, "Reporting обновлен: 0.1°C, 1-10 минут");
        
        esp_zigbee_lock_acquire(portMAX_DELAY);
        ret = ezb_zcl_reporting_start_attr_report(report_info);
        esp_zigbee_lock_release();
        
        if (ret == EZB_ERR_NONE) {
            ESP_LOGI(TAG, "Механизм отчетов запущен.");
        } else {
            ESP_LOGE(TAG, "Ошибка запуска отчетов: 0x%04x", ret);
        }
    } else {
        ESP_LOGE(TAG, "Ошибка обновления Reporting: 0x%04x", ret);
    }

    return ret;
}

static void esp_app_temp_sensor_handler(float temperature)
{
    int16_t measured_value = zb_temperature_to_s16(temperature);
    
    esp_zigbee_lock_acquire(portMAX_DELAY);
    // false = просто обновить в памяти. Стек сам решит, когда отправить.
    ezb_zcl_set_attr_value(ESP_ZIGBEE_HA_TEMPERATURE_SENSOR_EP_ID, 
                           EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
                           EZB_ZCL_CLUSTER_SERVER, 
                           EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
                           EZB_ZCL_STD_MANUF_CODE, 
                           (uint8_t *)&measured_value, 
                           false); 
    esp_zigbee_lock_release();
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    ESP_RETURN_ON_FALSE(!is_inited, ESP_OK, TAG, "Deferred driver already initialized");

    ESP_LOGI(TAG, "Инициализация 1-Wire шины на GPIO %d", ONEWIRE_GPIO);
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = { .bus_gpio_num = ONEWIRE_GPIO };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) {
            ds18b20_config_t ds_cfg = {};
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20) == ESP_OK) {
                ESP_LOGI(TAG, "Найден DS18B20, адрес: %016llX", next_onewire_device.address);
                break;
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));

    if (ds18b20 == NULL) {
        ESP_LOGE(TAG, "Датчик DS18B20 не найден!");
        return ESP_FAIL;
    }

    is_inited = true;
    return ESP_OK;
}

static void ds18b20_read_task(void *pvParameters)
{
    float temperature;
    while (1) {
        if (ds18b20) {
            if (ds18b20_trigger_temperature_conversion(ds18b20) == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(800));
                if (ds18b20_get_temperature(ds18b20, &temperature) == ESP_OK) {
                    esp_app_temp_sensor_handler(temperature); 
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Читаем каждые 10 секунд
    }
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zigbee_lock_acquire(portMAX_DELAY);
        (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        esp_zigbee_lock_release();
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            is_rejoining = false; // Мы в сети
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", ezb_bdb_is_factory_new() ? "" : " non");
            
            if (ezb_bdb_is_factory_new()) {
                // Новое устройство, ищем сеть
                esp_zigbee_lock_acquire(portMAX_DELAY);
                (void)ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
                esp_zigbee_lock_release();
            } else {
                ESP_LOGI(TAG, "Device reboot");
                config_temp_reporting(ESP_ZIGBEE_HA_TEMPERATURE_SENSOR_EP_ID);
            }
        } else {
            ESP_LOGW(TAG, "%s failed with status(0x%02x). Ждем 10с перед повторной попыткой...", ezb_app_signal_to_string(signal_type), status);
            // Запускаем таймер, который вызовет INITIALIZATION
            if (!is_rejoining) {
                is_rejoining = true;
                esp_timer_stop(rejoin_timer);
                esp_timer_start_once(rejoin_timer, 10000000); // 10 секунд
            }
        }
    } break;

    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            is_rejoining = false;
            // ... логирование ...
            config_temp_reporting(ESP_ZIGBEE_HA_TEMPERATURE_SENSOR_EP_ID);
        } else {
            ESP_LOGW(TAG, "Failed to join network with status(0x%02x), retry in 10s", status);
            if (!is_rejoining) {
                is_rejoining = true;
                esp_timer_stop(rejoin_timer);
                esp_timer_start_once(rejoin_timer, 10000000);
            }
        }
    } break;

    case EZB_NWK_SIGNAL_NETWORK_STATUS: {
        const ezb_nwk_signal_network_status_params_t *params = 
            (const ezb_nwk_signal_network_status_params_t *)ezb_app_signal_get_params(app_signal);
        
        if (params->status == EZB_NWK_NETWORK_STATUS_PARENT_LINK_FAILURE) {
            // Получаем текущее время в микросекундах
            int64_t now = esp_timer_get_time();
            
            // Логируем не чаще, чем раз в 10 секунд (10 000 000 мкс)
            if (now - last_parent_link_failure_log > 10000000) {
                ESP_LOGW(TAG, "Потеря связи с родителем. Ждем восстановления SLS...");
                last_parent_link_failure_log = now;
            }
            // НИЧЕГО НЕ ДЕЛАЕМ. Просто ждем, пока SLS вернется.
        }
    } break;

    case EZB_ZDO_SIGNAL_LEAVE: {
        const ezb_zdo_signal_leave_params_t *leave_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "Вышли из сети (type: 0x%02x). Ждем...", leave_params->leave_type);
        // НИЧЕГО НЕ ДЕЛАЕМ.
    } break;
    
    default:
        break;
    }
    return true;
}

static void esp_zigbee_zcl_core_action_handler(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    switch (callback_id) {
    case EZB_ZCL_CORE_DEFAULT_RSP_CB_ID: {
        ezb_zcl_cmd_default_rsp_message_t *default_rsp = (ezb_zcl_cmd_default_rsp_message_t *)message;
        ESP_LOGI(TAG, "Received ZCL Default Response: status(0x%02x)", default_rsp->in.status_code);
    } break;
    default:
        break;
    }
}

esp_err_t esp_zigbee_create_zha_temperature_sensor_device(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    ezb_zha_temperature_sensor_config_t temperature_sensor_cfg = EZB_ZHA_TEMPERATURE_SENSOR_CONFIG();
    temperature_sensor_cfg.temp_meas_cfg.min_measured_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MIN_VALUE);
    temperature_sensor_cfg.temp_meas_cfg.max_measured_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE);

    ezb_af_ep_desc_t ep_desc = ezb_zha_create_temperature_sensor(ESP_ZIGBEE_HA_TEMPERATURE_SENSOR_EP_ID, &temperature_sensor_cfg);
    ezb_zcl_cluster_desc_t basic_desc = {0};
    uint8_t power_source = 0x01; // MAINS_SINGLE_PHASE

    basic_desc = ezb_af_endpoint_get_cluster_desc(ep_desc, EZB_ZCL_CLUSTER_ID_BASIC, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ESP_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ESP_MODEL_IDENTIFIER);

    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep_desc));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));

    ezb_zcl_core_action_handler_register(esp_zigbee_zcl_core_action_handler);
    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_nwk_set_min_join_lqi(20);
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler));
    return ESP_OK;
}

static void esp_zigbee_stack_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = ESP_ZIGBEE_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());
    ESP_ERROR_CHECK(esp_zigbee_create_zha_temperature_sensor_device());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    init_rejoin_timer();
    ESP_LOGI(TAG, "Start ESP Zigbee Stack");
    
    xTaskCreate(ds18b20_read_task, "ds18b20_read", 4096, NULL, 4, NULL);
    xTaskCreate(esp_zigbee_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
