/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi.h"
#include "uart1.h"
#include "i2c_chipset.h"
#include "adc.h"
#include "bluetooth.h"

#define TAG "MAIN"


void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
      
    // UART1 초기화
    uart1_init();
    //i2c_chipset_init();
    //adc_init();
    bt_init();

    // 통신 전용 Task
   // xTaskCreatePinnedToCore(uart1_task, "uart1_task", 8192, NULL, 9, NULL, tskNO_AFFINITY);

    // Wi-Fi 태스크
    //xTaskCreatePinnedToCore(wifi_task, "wifi_task", 8192, NULL, 8, NULL, tskNO_AFFINITY);    

    // SHT20(Temp, Humi), PCF85063AT(RTC)
   // xTaskCreatePinnedToCore(i2c_chipset_task, "i2c_task", 4096, NULL, 10, NULL, tskNO_AFFINITY);

   // xTaskCreatePinnedToCore(adc_task, "adc_task", 4096, NULL, 8, NULL, tskNO_AFFINITY);

    xTaskCreatePinnedToCore(bt_task, "bt_task", 4096, NULL, 8, NULL, tskNO_AFFINITY);
   

}
