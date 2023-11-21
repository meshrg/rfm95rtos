#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

// pins to use, please refer to
// variants/esp32/pins_arduino.h

// HSPI
// #define GPIO_MISO 12
// #define GPIO_MOSI 13
// #define GPIO_SCLK 14
// #define GPIO_CS   15

// VSPI
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   05

#define CLOCK_SPEED_HZ 10

static const char TAG[] = "VSPI";

void app_main(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num   = GPIO_MOSI,
        .miso_io_num   = GPIO_MISO,
        .sclk_io_num   = GPIO_SCLK
    };

    spi_device_interface_config_t devcfg = {
        .mode           = 0,
        .spics_io_num   = GPIO_CS,
        .clock_speed_hz = CLOCK_SPEED_HZ * 1000 * 1000,
        .queue_size     = 1
    };

    // spi initialization
    spi_bus_initialize(VSPI_HOST, &buscfg, 1);

    // spi handle
    spi_device_handle_t spi_handle;
    spi_bus_add_device(VSPI_HOST, &devcfg, &spi_handle);

    // spi transaction
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.length = 20;

    const char test_str[] = "A"; // 01000001
    transaction.tx_buffer = test_str;

    while (1)
    {
        ESP_LOGI(TAG, "Sending char A");
        spi_device_transmit(spi_handle, &transaction);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
