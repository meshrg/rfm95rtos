#include "esp_log.h"
#include <string.h>
#include <driver/gpio.h>
#include "driver/spi_master.h"

// SPI3_HOST
#define GPIO_NUM_MOSI 			23
#define GPIO_NUM_MISO 			19
#define GPIO_NUM_SCLK 			18
#define GPIO_NUM_CS    			 5

#define GPIO_NUM_RST  			13
#define GPIO_NUM_G0   			27
#define TAG 		  			"RFM9x"
#define TIMEOUT_RESET			100

// This is the bit in the SPI address that marks it as a write
#define RH_RF95_REG_VERSION      	0x42
#define RH_FIFO_TX_BASE_ADDR        0x0e
#define RH_FIFO_RX_BASE_ADDR        0x0f

// function prototypes
esp_err_t spi_init(void);
void reset_radio(void);
int register_read(int reg);

static spi_device_handle_t spi_handle;

void app_main(void)
{
	uint8_t version_register;
	uint8_t result;

	spi_init();
	reset_radio();

	version_register = RH_RF95_REG_VERSION;
	printf("Reading version register: %x\n", version_register);
	result = register_read(version_register);
	printf("fetched result is: 0x%x = %i\n", result, result);
	if (result == 0x12) {
		printf("LoRa radio init success\n");
	} else{
		printf("LoRa radio init failed. Check wiring.\n");
	}

}

esp_err_t spi_init(void)
{
	esp_err_t ret;

	// reset pin configuration
	gpio_reset_pin(GPIO_NUM_RST);
   	gpio_set_direction(GPIO_NUM_RST, GPIO_MODE_OUTPUT);

	// chip select pin configuration
   	gpio_reset_pin(GPIO_NUM_CS);
   	gpio_set_direction(GPIO_NUM_CS, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_CS, 1);

	spi_bus_config_t spi_bus_config = {
        .miso_io_num = GPIO_NUM_MISO,
        .mosi_io_num = GPIO_NUM_MOSI,
        .sclk_io_num = GPIO_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
		.max_transfer_sz = 0
    };

	ret = spi_bus_initialize(SPI3_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	assert(ret == ESP_OK);

	spi_device_interface_config_t spi_device_interface_config = {
		.clock_speed_hz = 5 * 1000 * 1000,
		.mode = 0,  // CPOL=0, CPHA=0
		.queue_size = 1,
		.spics_io_num = -1,
		// .flags = SPI_DEVICE_NO_DUMMY,
		.pre_cb = NULL,
		.post_cb = NULL
	};

	ret = spi_bus_add_device(SPI3_HOST, &spi_device_interface_config, &spi_handle);
	ESP_LOGI(TAG, "spi_bus_add_device=%d", ret);
	assert(ret == ESP_OK);

	return ESP_OK;
}

void reset_radio(void)
{
	ESP_LOGI(TAG, "reseting lora radio. Starting...");
	gpio_set_level(GPIO_NUM_RST, 0);
	vTaskDelay(pdMS_TO_TICKS(1));
	gpio_set_level(GPIO_NUM_RST, 1);
	vTaskDelay(pdMS_TO_TICKS(1));
	ESP_LOGI(TAG, "reseting lora radio. Done.");
}


int register_read(int reg)
{
   uint8_t buffer_out[2] = { reg, 0xff };
   uint8_t buffer_in[2];

   spi_transaction_t t = {
      .flags = 0,	// transaction line mode
      .length = 8 * sizeof(buffer_out),
      .tx_buffer = buffer_out,
      .rx_buffer = buffer_in
   };

   gpio_set_level(GPIO_NUM_CS, 0);
   spi_device_transmit(spi_handle, &t);
   gpio_set_level(GPIO_NUM_CS, 1);

   return buffer_in[1];
}