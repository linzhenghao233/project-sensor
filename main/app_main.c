#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "mqtt_client.h"

// 包含你自己的传感器和舵机驱动头文件
#include "dht11.h"
#include "sg90.h"

static const char *TAG = "SMART_PLANT_SYSTEM";

// ====================================================================
//                       系统参数配置 (请修改)
// ====================================================================
#define WIFI_SSID           "Xiaomi_D550"
#define WIFI_PASSWORD       "lin3399919.."

// --- ThingSpeak MQTT 接口配置 ---
// 这里是你的 ThingSpeak MQTT 设备凭证
#define THINGSPEAK_CLIENT_ID    "DDE2Hh8eNCA0FzsCBS8WMDQ"
#define THINGSPEAK_USERNAME     "DDE2Hh8eNCA0FzsCBS8WMDQ"
#define THINGSPEAK_PASSWORD     "FeGUF42nOA1nTvsY9t89Bd6K"
// 你的 ThingSpeak 频道 ID (请到 Channel Settings 中查看填到这里)
#define THINGSPEAK_CHANNEL_ID   "3069197"


// --- 传感器校准值 (仅光敏电阻) ---
#define LIGHT_DARK_VALUE    4095
#define LIGHT_BRIGHT_VALUE  450

// --- 自动浇灌逻辑阈值 ---
#define WATERING_THRESHOLD_PERCENT  40.0f // 土壤湿度低于40%时浇水

// --- 硬件引脚配置 ---
#define LIGHT_SENSOR_ADC_CHANNEL    ADC_CHANNEL_1 // 光敏电阻 -> GPIO1
#define MODBUS_UART_PORT      UART_NUM_1
#define MODBUS_TX_PIN         GPIO_NUM_7
#define MODBUS_RX_PIN         GPIO_NUM_6
#define SENSOR_SLAVE_ADDR     0x01
#define BAUD_RATE             4800
// ====================================================================

// 定义传感器数据结构体 
typedef struct {
    int air_temperature;
    int air_humidity;
    float light_intensity;
    // --- 新增字段 ---
    float soil_moisture;
    float soil_temp;
    float soil_ec;
    float soil_ph;
    int soil_n;
    int soil_p;
    int soil_k;
} SensorData_t;

// 全局队列句柄
static QueueHandle_t sensor_data_queue;

// Modbus RTU CRC16 校验码计算函数
static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

// --- Wi-Fi 初始化相关函数 ---
void wifi_init_sta(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);


/**
 * @brief 任务1：传感器读取任务 
 */
void sensor_reader_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor Reader Task started.");

    // --- ADC 初始化 (仅用于光敏电阻) ---
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc_init_config = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc1_handle));
    adc_oneshot_chan_cfg_t adc_chan_config = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LIGHT_SENSOR_ADC_CHANNEL, &adc_chan_config));

    // --- Modbus UART 初始化 ---
    uart_config_t uart_config = {.baud_rate = BAUD_RATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
    uart_driver_install(MODBUS_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(MODBUS_UART_PORT, &uart_config);
    uart_set_pin(MODBUS_UART_PORT, MODBUS_TX_PIN, MODBUS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    SensorData_t current_data = {0}; // 初始化数据包

    while (1)
    {
        // 1. 读取 DHT11 (空气温湿度)
        DHT11();
        if (wendu < 85 && shidu <= 100) {
            current_data.air_temperature = wendu;
            current_data.air_humidity = shidu;
        } else {
            ESP_LOGW(TAG, "Invalid DHT11 reading discarded");
        }

        // 2. 读取光敏电阻 (ADC)
        int light_adc_raw;
        adc_oneshot_read(adc1_handle, LIGHT_SENSOR_ADC_CHANNEL, &light_adc_raw);
        current_data.light_intensity = 100.0f * (LIGHT_DARK_VALUE - light_adc_raw) / (LIGHT_DARK_VALUE - LIGHT_BRIGHT_VALUE);
        if (current_data.light_intensity > 100.0f) current_data.light_intensity = 100.0f;
        if (current_data.light_intensity < 0.0f) current_data.light_intensity = 0.0f;
        
        // 3. 读取 485 传感器 (Modbus)
        uint8_t request_frame[8];
        request_frame[0] = SENSOR_SLAVE_ADDR;
        request_frame[1] = 0x03;
        request_frame[2] = 0x00;
        request_frame[3] = 0x00;
        request_frame[4] = 0x00;
        request_frame[5] = 0x07; // 读取7个寄存器
        uint16_t crc = crc16(request_frame, 6);
        request_frame[6] = crc & 0xFF;
        request_frame[7] = (crc >> 8) & 0xFF;

        uart_write_bytes(MODBUS_UART_PORT, (const char*)request_frame, sizeof(request_frame));
        uint8_t response_frame[256];
        int length = uart_read_bytes(MODBUS_UART_PORT, response_frame, 256, pdMS_TO_TICKS(1000));

        if (length >= 19) {
            uint16_t moisture_raw = (response_frame[3] << 8) | response_frame[4];
            int16_t  temp_raw     = (response_frame[5] << 8) | response_frame[6];
            uint16_t ec_raw       = (response_frame[7] << 8) | response_frame[8];
            uint16_t ph_raw       = (response_frame[9] << 8) | response_frame[10];
            
            current_data.soil_moisture = moisture_raw / 10.0f;
            current_data.soil_temp     = temp_raw / 10.0f;
            current_data.soil_ec       = ec_raw;
            current_data.soil_ph       = ph_raw / 10.0f;
            current_data.soil_n        = (response_frame[11] << 8) | response_frame[12];
            current_data.soil_p        = (response_frame[13] << 8) | response_frame[14];
            current_data.soil_k        = (response_frame[15] << 8) | response_frame[16];
        } else {
            ESP_LOGE(TAG, "Modbus sensor read timeout or invalid frame length.");
        }

        // 4. 发送整合后的数据到队列
        if (xQueueSend(sensor_data_queue, &current_data, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to post sensor data to queue.");
        }

        vTaskDelay(pdMS_TO_TICKS(20000));
    }
}


// --- MQTT 客户端句柄定义 ---
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// MQTT 事件处理回调函数
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED to ThingSpeak!");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            break;
    }
}

/**
 * @brief 任务2：逻辑控制与云端上传任务 (ThingSpeak MQTT版)
 */
void logic_and_upload_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Logic & Upload Task started.");
    SensorData_t data;
    
    ESP_LOGI(TAG, "Waiting 10s for Wi-Fi connection...");
    vTaskDelay(pdMS_TO_TICKS(10000)); 
    
    // 初始化 MQTT 客户端
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtts://mqtt3.thingspeak.com:8883",
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.client_id = THINGSPEAK_CLIENT_ID,
        .credentials.username  = THINGSPEAK_USERNAME,
        .credentials.authentication.password = THINGSPEAK_PASSWORD,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    char publish_topic[100];
    snprintf(publish_topic, sizeof(publish_topic), "channels/%s/publish", THINGSPEAK_CHANNEL_ID);
    
    char payload_buffer[256];

    while (1)
    {
        if (xQueueReceive(sensor_data_queue, &data, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "-------------------- New Data Packet --------------------");
            ESP_LOGI(TAG, "Air Temp: %dC, Air Hum: %d%%, Light: %.1f%%", data.air_temperature, data.air_humidity, data.light_intensity);
            ESP_LOGI(TAG, "Soil Temp: %.1fC, Soil Hum: %.1f%%, EC: %.0f, PH: %.1f", data.soil_temp, data.soil_moisture, data.soil_ec, data.soil_ph);
            ESP_LOGI(TAG, "NPK: N=%d, P=%d, K=%d", data.soil_n, data.soil_p, data.soil_k);

            // --- 核心浇水逻辑 ---
            if (data.soil_moisture >= 0 && data.soil_moisture < WATERING_THRESHOLD_PERCENT)
            {
                ESP_LOGW(TAG, "Soil moisture LOW! Starting watering procedure.");
                sg90_SetAngle(90);
                vTaskDelay(pdMS_TO_TICKS(3000));
                sg90_SetAngle(0);
                ESP_LOGW(TAG, "Watering complete.");
            }
            
            // --- ThingSpeak MQTT 数据上传逻辑 ---
            if (mqtt_connected) {
                // ThingSpeak 接受 URL query 格式的 Payload，最多 8 个字段 (field1 ~ field8)
                // 这里我们选择上传: 1.空气温度 2.空气湿度 3.光照 4.土壤水分 5.土壤温度 6.EC 7.PH 8.氮(作为代表)
                snprintf(payload_buffer, sizeof(payload_buffer), 
                         "field1=%d&field2=%d&field3=%.1f&field4=%.1f&field5=%.1f&field6=%.0f&field7=%.1f&field8=%d",
                         data.air_temperature,
                         data.air_humidity,
                         data.light_intensity,
                         data.soil_moisture,
                         data.soil_temp,
                         data.soil_ec,
                         data.soil_ph,
                         data.soil_n);
                
                int msg_id = esp_mqtt_client_publish(mqtt_client, publish_topic, payload_buffer, 0, 0, 0);
                if (msg_id != -1) {
                     ESP_LOGI(TAG, "[ThingSpeak] Enqueued data to MQTT, msg_id=%d", msg_id);
                     ESP_LOGI(TAG, "Payload: %s", payload_buffer);
                } else {
                     ESP_LOGE(TAG, "[ThingSpeak] Failed to enqueue MQTT message.");
                }
            } else {
                ESP_LOGW(TAG, "[ThingSpeak] MQTT not connected, skipping upload.");
            }
        }
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Initializing System...");

    // 1. 初始化 NVS Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化并连接 Wi-Fi
    wifi_init_sta();

    // 3. 初始化舵机
    sg90_init();
    sg90_SetAngle(0);

    // 4. 创建数据队列
    sensor_data_queue = xQueueCreate(5, sizeof(SensorData_t));

    // 5. 创建任务
    xTaskCreate(sensor_reader_task, "SensorReaderTask", 4096, NULL, 5, NULL);
    xTaskCreate(logic_and_upload_task, "LogicAndUploadTask", 6144, NULL, 5, NULL); // 轻微增加了栈大小，以防使用JSON/HTTP时栈溢出

    ESP_LOGI(TAG, "All tasks created. System is now running.");
}


// --- 之前能正常工作的 wifi_init_sta 和 wifi_event_handler 函数 ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from Wi-Fi, trying to reconnect...");
        esp_wifi_connect();
    }
}

void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi initialization finished. Waiting for connection...");
}
