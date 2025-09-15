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

// 包含你自己的传感器和舵机驱动头文件
#include "dht11.h"
#include "sg90.h"

static const char *TAG = "PLANT_MONITOR_IOT_FINAL";
static SemaphoreHandle_t wifi_mutex;

// ====================================================================
//                       系统参数配置 (请修改)
// ====================================================================
// --- Wi-Fi 和 ThingSpeak 配置 ---
#define WIFI_SSID           "Xiaomi_D550"
#define WIFI_PASSWORD       "lin3399919.."
#define THINGSPEAK_API_KEY  "I0HL9KQ0E0ODA0X7"

// --- 传感器校准值 ---
#define SOIL_DRY_VALUE      4095
#define SOIL_WET_VALUE      2440
#define LIGHT_DARK_VALUE    4095
#define LIGHT_BRIGHT_VALUE  450

// --- 自动浇灌逻辑阈值 ---
#define WATERING_THRESHOLD_PERCENT  30.0f

// --- 传感器连接引脚 (ADC通道) ---
#define SOIL_MOISTURE_ADC_CHANNEL   ADC_CHANNEL_0
#define LIGHT_SENSOR_ADC_CHANNEL    ADC_CHANNEL_1
// ====================================================================

// 定义传感器数据结构体
typedef struct {
    int air_temperature;
    int air_humidity;
    float soil_moisture;
    float light_intensity;
} SensorData_t;

// 全局队列句柄
static QueueHandle_t sensor_data_queue;


// --- Wi-Fi 初始化相关函数 ---
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

/**
 * @brief 任务1：传感器读取任务 (修正版)
 */
void sensor_reader_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor Reader Task started.");

    // 在任务内部初始化ADC，避免句柄冲突
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc_init_config = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_init_config, &adc1_handle));
    adc_oneshot_chan_cfg_t adc_chan_config = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOIL_MOISTURE_ADC_CHANNEL, &adc_chan_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LIGHT_SENSOR_ADC_CHANNEL, &adc_chan_config));
    
    SensorData_t current_data = {0}; // 初始化数据包

    while (1)
    {
        DHT11();

        // 2. 检查函数执行后的"结果"是否在合理范围内
        if (wendu < 85 && shidu <= 100) {
            current_data.air_temperature = wendu;
            current_data.air_humidity = shidu;
        } else {
            ESP_LOGW(TAG, "Invalid or failed DHT11 reading discarded (value: %dC, %d%%)", wendu, shidu);
            // 当读数无效时，可以选择不更新数据，保留上一次的有效读数
        }

        // --- ADC读取保护，避免Wi-Fi干扰 ---
        // 使用互斥锁保护Wi-Fi操作
        if (xSemaphoreTake(wifi_mutex, portMAX_DELAY) == pdTRUE) {
            ESP_ERROR_CHECK(esp_wifi_stop());
            vTaskDelay(pdMS_TO_TICKS(100)); // 延时确保Wi-Fi射频完全停止

            int soil_adc_raw, light_adc_raw;
            adc_oneshot_read(adc1_handle, SOIL_MOISTURE_ADC_CHANNEL, &soil_adc_raw);
            adc_oneshot_read(adc1_handle, LIGHT_SENSOR_ADC_CHANNEL, &light_adc_raw);
            
            ESP_ERROR_CHECK(esp_wifi_start());
            xSemaphoreGive(wifi_mutex); // 释放锁
            
            // 计算土壤湿度（确保分母不为零）
            float moisture = 0.0f;
            if (SOIL_DRY_VALUE != SOIL_WET_VALUE) {
                moisture = 100.0f * (SOIL_DRY_VALUE - soil_adc_raw) / (SOIL_DRY_VALUE - SOIL_WET_VALUE);
            }
            if (moisture > 100.0f) moisture = 100.0f;
            if (moisture < 0.0f) moisture = 0.0f;
            current_data.soil_moisture = moisture;

            // 计算光照强度（确保分母不为零）
            float light = 0.0f;
            if (LIGHT_DARK_VALUE != LIGHT_BRIGHT_VALUE) {
                light = 100.0f * (LIGHT_DARK_VALUE - light_adc_raw) / (LIGHT_DARK_VALUE - LIGHT_BRIGHT_VALUE);
            }
            if (light > 100.0f) light = 100.0f;
            if (light < 0.0f) light = 0.0f;
            current_data.light_intensity = light;
        } else {
            ESP_LOGE(TAG, "Failed to acquire mutex for ADC reading!");
            // 如果无法获取锁，跳过本次ADC读取
            continue;
        }
        // ------------------------------------

        // 发送数据到队列
        if (xQueueSend(sensor_data_queue, &current_data, pdMS_TO_TICKS(100)) != pdPASS) {
            ESP_LOGE(TAG, "Failed to post sensor data to queue.");
        }

        // 每10秒读取一次
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


/**
 * @brief 任务2：逻辑控制与云端上传任务 (修正版，合并了两个任务)
 */
void logic_and_upload_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Logic & Upload Task started.");
    SensorData_t received_data;
    char url_buffer[256];

    while (1)
    {
        // 从队列接收数据，如果队列为空，将在此无限期等待
        if (xQueueReceive(sensor_data_queue, &received_data, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "-------------------- New Data Packet --------------------");
            ESP_LOGI(TAG, "Air: %dC, %d%% | Soil: %.1f%% | Light: %.1f%%",
                     received_data.air_temperature,
                     received_data.air_humidity,
                     received_data.soil_moisture,
                     received_data.light_intensity);

            // --- 核心逻辑：判断是否需要浇水 ---
            if (received_data.soil_moisture >= 0 && received_data.soil_moisture < WATERING_THRESHOLD_PERCENT)
            {
                ESP_LOGW(TAG, "Soil moisture LOW! Starting watering procedure.");
                sg90_SetAngle(90); // 打开阀门
                vTaskDelay(pdMS_TO_TICKS(3000)); // 浇水3秒
                sg90_SetAngle(0);  // 关闭阀门
                ESP_LOGW(TAG, "Watering complete.");
            }
            
            // --- 云端上传逻辑 ---
            snprintf(url_buffer, sizeof(url_buffer),
                     "http://api.thingspeak.com/update?api_key=%s&field1=%d&field2=%d&field3=%.1f&field4=%.1f",
                     THINGSPEAK_API_KEY,
                     received_data.air_temperature,
                     received_data.air_humidity,
                     received_data.soil_moisture,
                     received_data.light_intensity);
            
            // 使用互斥锁保护HTTP上传操作
            if (xSemaphoreTake(wifi_mutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                esp_http_client_config_t config = { .url = url_buffer, .method = HTTP_METHOD_GET };
                esp_http_client_handle_t client = esp_http_client_init(&config);
                
                // 添加上传重试机制
                int retry_count = 0;
                const int max_retries = 3;
                esp_err_t err = ESP_FAIL;
                
                do {
                    err = esp_http_client_perform(client);
                    if (err == ESP_OK) {
                        ESP_LOGI(TAG, "[Cloud] Data uploaded successfully.");
                        break; // 成功则跳出重试循环
                    }
                    
                    ESP_LOGW(TAG, "[Cloud] Upload attempt %d failed: %s", retry_count+1, esp_err_to_name(err));
                    retry_count++;
                    
                    if (retry_count < max_retries) {
                        vTaskDelay(pdMS_TO_TICKS(2000)); // 等待2秒后重试
                    }
                } while (retry_count < max_retries);
                
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "[Cloud] Failed to upload data after %d attempts: %s", max_retries, esp_err_to_name(err));
                }
                
                esp_http_client_cleanup(client);
                xSemaphoreGive(wifi_mutex); // 释放锁
            } else {
                ESP_LOGE(TAG, "Failed to acquire mutex for HTTP upload!");
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing System...");

    // 1. 初始化 NVS Flash (Wi-Fi 必须)
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

    // 5. 创建互斥锁
    wifi_mutex = xSemaphoreCreateMutex();
    if (wifi_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex!");
        return;
    }

    // 6. 创建任务
    xTaskCreate(sensor_reader_task, "SensorReaderTask", 4096, NULL, 5, NULL);
    xTaskCreate(logic_and_upload_task, "LogicAndUploadTask", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "All tasks created. System is now running.");
}
















#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "MODBUS_TEST_AUTO";

// ==========================================================
//          ↓↓↓  根据您的开发板引脚图进行修正  ↓↓↓
// ==========================================================
#define MODBUS_UART_PORT      UART_NUM_1 // UART1, 因为 UART0被用于日志打印
#define MODBUS_TX_PIN         GPIO_NUM_7 // 新的 TX 引脚
#define MODBUS_RX_PIN         GPIO_NUM_6 // 新的 RX 引脚
#define MODBUS_RTS_PIN        UART_PIN_NO_CHANGE // 您的模块是自动流控，不需要此引脚
// ==========================================================

#define SENSOR_SLAVE_ADDR     0x01
#define BAUD_RATE             4800

// CRC16 计算函数 (保持不变)
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

void app_main(void)
{
    // 1. 初始化UART
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // 注意这里也改为了 MODBUS_UART_PORT
    uart_driver_install(MODBUS_UART_PORT, 256, 256, 0, NULL, 0);
    uart_param_config(MODBUS_UART_PORT, &uart_config);
    uart_set_pin(MODBUS_UART_PORT, MODBUS_TX_PIN, MODBUS_RX_PIN, MODBUS_RTS_PIN, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "Modbus UART Initialized on TX=%d, RX=%d", MODBUS_TX_PIN, MODBUS_RX_PIN);

    // 2. 构造Modbus请求帧 (保持不变)
    uint8_t request_frame[8];
    request_frame[0] = SENSOR_SLAVE_ADDR;
    request_frame[1] = 0x03;
    request_frame[2] = 0x00;
    request_frame[3] = 0x00;
    request_frame[4] = 0x00;
    request_frame[5] = 0x07; // 一次性读取7个寄存器
    uint16_t crc = crc16(request_frame, 6);
    request_frame[6] = crc & 0xFF;
    request_frame[7] = (crc >> 8) & 0xFF;

    while(1) {
        // 3. 发送请求
        uart_write_bytes(MODBUS_UART_PORT, (const char*)request_frame, sizeof(request_frame));
        
        // 4. 读取响应
        uint8_t response_frame[256];
        int length = uart_read_bytes(MODBUS_UART_PORT, response_frame, 256, pdMS_TO_TICKS(1000));

        if (length > 0) {
            ESP_LOGI(TAG, "Received %d bytes", length);
            // 成功读取7个寄存器，返回的字节数应为 1+1+1+(7*2)+2 = 19字节
            if (length >= 19) {
                // 5. 解析数据 (和之前一样)
                uint16_t moisture_raw = (response_frame[3] << 8) | response_frame[4];
                int16_t  temp_raw     = (response_frame[5] << 8) | response_frame[6];
                uint16_t ec_raw       = (response_frame[7] << 8) | response_frame[8];
                uint16_t ph_raw       = (response_frame[9] << 8) | response_frame[10];
                uint16_t n_raw        = (response_frame[11] << 8) | response_frame[12];
                uint16_t p_raw        = (response_frame[13] << 8) | response_frame[14];
                uint16_t k_raw        = (response_frame[15] << 8) | response_frame[16];

                float moisture    = moisture_raw / 10.0f;
                float temperature = temp_raw / 10.0f;
                float ec          = ec_raw;
                float ph          = ph_raw / 10.0f;
                int   nitrogen    = n_raw;
                int   phosphorus  = p_raw;
                int   potassium   = k_raw;

                ESP_LOGI(TAG, "====== SENSOR DATA ======");
                ESP_LOGI(TAG, "Soil Moisture:    %.1f %%", moisture);
                ESP_LOGI(TAG, "Soil Temperature: %.1f C", temperature);
                ESP_LOGI(TAG, "Soil EC:          %.0f us/cm", ec);
                ESP_LOGI(TAG, "Soil PH:          %.1f", ph);
                ESP_LOGI(TAG, "Nitrogen (N):     %d mg/kg", nitrogen);
                ESP_LOGI(TAG, "Phosphorus (P):   %d mg/kg", phosphorus);
                ESP_LOGI(TAG, "Potassium (K):    %d mg/kg", potassium);
                ESP_LOGI(TAG, "=========================");
            }
        } else {
            ESP_LOGE(TAG, "Modbus response timeout.");
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


idf_component_register(SRCS "main_lastversion.c" "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        esp_system
                        spi_flash
                        log
                        driver
                        DHT11
                        esp_driver_mcpwm
                        esp_adc
                        SG90
                        # --- 新增的网络组件 ---
                        nvs_flash        # 用于存储Wi-Fi配置
                        esp_netif        # TCP/IP网络接口
                        esp_event        # 事件循环库
                        esp_wifi         # Wi-Fi驱动
                        esp_http_client  # HTTP客户端库
                        )

