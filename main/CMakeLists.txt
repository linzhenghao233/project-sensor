# 声明 main 组件需要依赖的所有其他系统组件

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

target_add_binary_data(__idf_main "thingspeak_ca.crt" TEXT)