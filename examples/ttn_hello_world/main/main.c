/**
 * Sample C program for showing how to send Hello World to the Things Network (TTN).
 *
 * This uses the SPI3_HOST (VSPI) on the ESP32 with the following pins:
 * - GPIO_NUM_MOSI (23)
 * - GPIO_NUM_MISO (19)
 * - GPIO_NUM_SCLK (18)
 * - GPIO_NUM_CS   ( 5)
 * - GPIO_NUM_RST  (13)
 * - GPIO_NUM_G0   (27)
 *
 * Change at your own setup.
 *
 * This example uses the ttn-esp library by Manuel Bleichenbacher
 * If you need more information about this library, please visit:
 * https://github.com/manuelbl/ttn-esp32/
 *
 * Maestría en Ingeniería para la Innovación Tecnológica (MIIT)
 * Masters in Engineering for Technological Innovation
 * Specialization: IoT and Embedded Systems.
 * Project: LoRaWAN Node with ESP32, RFM9x and FreeRTOS
 *
 * UAZ. Universidad Autonoma de Zacatecas, Mexico.
 *
 * Director: Dr. Viktor Iván Rodríguez Abdalá (abdala@uaz.mx)
 * Student: Fernando Valderrábano Reyes (fernando.valderrabano@uaz.mx)
 *
**/

#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "ttn.h"

// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'idf.py menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.

// Copy the below hex strings from the TTN console (Applications > Your application > End devices
// > Your device > Activation information)

// AppEUI (sometimes called JoinEUI)
const char *appEui = "0000000000000000";
// DevEUI
const char *devEui = "70B3D57ED00680A6";
// AppKey
const char *appKey = "FA696B924D79DC0E2A79C83ABEE37067";

// Pins and other resources
#define TTN_SPI_HOST      SPI3_HOST
#define TTN_SPI_DMA_CHAN  SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK  18
#define TTN_PIN_SPI_MOSI  23
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       5
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       13
#define TTN_PIN_DIO0      27
#define TTN_PIN_DIO1      26

#define TX_INTERVAL       30
static uint8_t msgData[] = "Hello, world";


void sendMessages(void* pvParameter)
{
    while (1) {
        printf("Sending message...\n");
        ttn_response_code_t res = ttn_transmit_message(msgData, sizeof(msgData) - 1, 1, false);
        printf(res == TTN_SUCCESSFUL_TRANSMISSION ? "Message sent.\n" : "Transmission failed.\n");

        vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
    }
}

void messageReceived(const uint8_t* message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

void app_main(void)
{
    esp_err_t err;
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config = {
        .miso_io_num = TTN_PIN_SPI_MISO,
        .mosi_io_num = TTN_PIN_SPI_MOSI,
        .sclk_io_num = TTN_PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Initialize TTN
    ttn_init();

    // Configure the SX127x pins
    ttn_configure_pins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn_provision(devEui, appEui, appKey);

    // Register callback for received messages
    ttn_on_message(messageReceived);

    // ttn_set_adr_enabled(false);
    // ttn_set_data_rate(TTN_DR_US915_SF7);
    // ttn_set_max_tx_pow(14);

    printf("Joining...\n");
    if (ttn_join())
    {
        printf("Joined.\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, NULL);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
}
