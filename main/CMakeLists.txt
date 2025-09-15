idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                            # 核心库
                            log
                            nvs_flash
                            esp_event
                            esp_netif
                            
                            # 硬件驱动
                            driver           # 用于 UART 和 GPIO
                            esp_adc          # 用于光敏电阻
                            
                            # 网络库
                            esp_wifi
                            esp_http_client
                            
                            # 您的自定义组件
                            DHT11
                            SG90
)