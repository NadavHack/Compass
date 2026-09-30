#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lora.h"

static const char *TAG = "RECIEVER";

void app_main(void)
{
    int tick = 0;
    while (1) {
        ESP_LOGI(TAG, "Hello from Reciever, tick %d", tick++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
