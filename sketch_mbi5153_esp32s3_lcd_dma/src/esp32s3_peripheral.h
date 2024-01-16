#pragma once

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

// Peripheral
#include "driver/ledc.h"


void setup_gpio_dir();
void setup_gpio_output();


#ifdef __cplusplus
}
#endif
