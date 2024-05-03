/**
 * SPI driver to communicate with the RFM9x LoRa radio module on ESP-IDF Framework.
 *
 * This uses the SPI3_HOST (VSPI) on the ESP32 with the following pins:
 * - GPIO_NUM_MOSI (23)
 * - GPIO_NUM_MISO (19)
 * - GPIO_NUM_SCLK (18)
 * - GPIO_NUM_CS   ( 5)
 * - GPIO_NUM_RST  (13)
 * - GPIO_NUM_G0   (27)
 *
 * Based on the RFM9x datasheet, the SX1276/77/78/79 datasheet and the RadioHead library.
 * Uses the RadioHead packet format compatible with the RadioHead Arduino library.
 *
 * Postgraduate Innovation Technology // Posgrado en Ingenieria para la Innovacion Tecnologica
 * UAZ. Universidad Autonoma de Zacatecas, Mexico
 *
 *
 * Fernando Valderrabano (fernando.valderrabano@uaz.mx)
*/
#include "esp_log.h"
#include <string.h>
#include <driver/gpio.h>
#include "driver/spi_master.h"

// define SPI3_HOST / VSPI pins
#define GPIO_NUM_MOSI 			23
#define GPIO_NUM_MISO 			19
#define GPIO_NUM_SCLK 			18
#define GPIO_NUM_CS    			 5
#define GPIO_NUM_RST  			13
#define GPIO_NUM_G0   			27

#define TAG 		  			"RFM9x"
#define WNR_BIT_MASK 			0x80
#define CLOCK_SPEED_HZ 			5

// Table 41. Registers Summary
#define RFM9X_00_REG_FIFO       				0x00
#define RFM9X_01_REG_OP_MODE     				0x01
#define RFM9X_02_REG_BITRATE_MSB 				0x02
#define RFM9X_03_REG_BITRATE_LSB 				0x03
#define RFM9X_04_REG_FDEV_MSB   				0x04
#define RFM9X_05_REG_FDEV_LSB   				0x05
#define RFM9X_06_REG_FRF_MSB    				0x06
#define RFM9X_07_REG_FRF_MID    				0x07
#define RFM9X_08_REG_FRF_LSB    				0x08
#define RFM9X_09_REG_PA_CONFIG   				0x09
#define RFM9X_0A_REG_PA_RAMP     				0x0a
#define RFM9X_0B_REG_OCP       					0x0b
#define RFM9X_0C_REG_LNA       					0x0c
#define RFM9X_0D_REG_FIFO_ADDR_PTR 				0x0d
#define RFM9X_0E_REG_FIFO_TX_BASE_ADDR 			0x0e
#define RFM9X_0F_REG_FIFO_RX_BASE_ADDR 			0x0f
#define RFM9X_10_REG_FIFO_RX_CURRENT_ADDR 		0x10
#define RFM9X_11_REG_IRQ_FLAGS_MASK 			0x11
#define RFM9X_12_REG_IRQ_FLAGS     				0x12
#define RFM9X_13_REG_RX_NB_BYTES   				0x13
#define RFM9X_14_REG_RX_HEADER_CNT_VALUE_MSB 	0x14
#define RFM9X_15_REG_RX_HEADER_CNT_VALUE_LSB 	0x15
#define RFM9X_16_REG_RX_PACKET_CNT_VALUE_MSB 	0x16
#define RFM9X_17_REG_RX_PACKET_CNT_VALUE_LSB 	0x17
#define RFM9X_18_REG_MODEM_STAT    				0x18
#define RFM9X_19_REG_PKT_SNR_VALUE 				0x19
#define RFM9X_1A_REG_PKT_RSSI_VALUE 			0x1a
#define RFM9X_1B_REG_RSSI_VALUE    				0x1b
#define RFM9X_1C_REG_HOP_CHANNEL   				0x1c
#define RFM9X_1D_REG_MODEM_CONFIG1 				0x1d
#define RFM9X_1E_REG_MODEM_CONFIG2 				0x1e
#define RFM9X_1F_REG_SYMB_TIMEOUT_LSB 			0x1f
#define RFM9X_20_REG_PREAMBLE_MSB  				0x20
#define RFM9X_21_REG_PREAMBLE_LSB  				0x21
#define RFM9X_22_REG_PAYLOAD_LENGTH 			0x22
#define RFM9X_23_REG_MAX_PAYLOAD_LENGTH 		0x23
#define RFM9X_24_REG_HOP_PERIOD    				0x24
#define RFM9X_25_REG_FIFO_RX_BYTE_ADDR 			0x25
#define RFM9X_26_REG_MODEM_CONFIG3 				0x26
#define RFM9X_28_REG_FEI_MSB       				0x28
#define RFM9X_29_REG_FEI_MID       				0x29
#define RFM9X_2A_REG_FEI_LSB       				0x2A
#define RFM9X_2C_REG_RSSI_WIDEBAND 				0x2C
#define RFM9X_31_REG_DETECTION_OPTIMIZE 		0x31
#define RFM9X_33_REG_INVERT_IQ     				0x33
#define RFM9X_37_REG_DETECTION_THRESHOLD 		0x37
#define RFM9X_39_REG_SYNC_WORD     				0x39
#define RFM9X_40_REG_DIO_MAPPING1  				0x40
#define RFM9X_41_REG_DIO_MAPPING2  				0x41
#define RFM9X_42_REG_VERSION       				0x42
#define RFM9X_4B_REG_TCXO         				0x4B
#define RFM9X_4D_REG_PA_DAC        				0x4D
#define RFM9X_5B_REG_FORMER_TEMP  				0x5B
#define RFM9X_61_REG_AGC_REF      				0x61
#define RFM9X_62_REG_AGC_THRESH1  				0x62
#define RFM9X_63_REG_AGC_THRESH2  				0x63
#define RFM9X_64_REG_AGC_THRESH3  				0x64
#define RFM9X_70_REG_PLL  						0x70

// Table 16. LoRa Operating Mode Functionality
#define RFM9X_LONG_RANGE_MODE					0x80
#define RFM9X_ACCESS_SHARED_REG					0x40
#define RFM9X_LOW_FREQUENCY_MODE                0x08
#define RFM9X_MODE                              0x07
#define RFM9X_MODE_SLEEP                        0x00
#define RFM9X_MODE_STDBY                        0x01
#define RFM9X_MODE_FSTX                         0x02
#define RFM9X_MODE_TX                           0x03
#define RFM9X_MODE_FSRX                         0x04
#define RFM9X_MODE_RXCONTINUOUS                 0x05
#define RFM9X_MODE_RXSINGLE                     0x06
#define RFM9X_MODE_CAD                          0x07

// default value for register version
#define RFM9X_42_REG_VERSION_VALUE 				0x12

// FXOSC = 32MHz
#define RFM9X_FXOSC 							32000000.0

// function prototypes
esp_err_t spi_init(void);
void reset_radio(void);
int register_read(int reg);
esp_err_t register_write(int reg, int value);
uint8_t getRegOpMode();
uint8_t getRegOpLoraMode();
uint8_t setRegOpMode(uint8_t mode);
void getCurrentOpMode();

static spi_device_handle_t spi_handle;

void app_main(void)
{
	uint8_t version_register;
	uint8_t result;
	uint8_t regOpMode;

	spi_init();
	reset_radio();

	version_register = RFM9X_42_REG_VERSION;
	printf("Reading version register: %x\n", version_register);
	result = register_read(version_register);
	printf("fetched result is: 0x%x = %i\n", result, result);
	if (result == RFM9X_42_REG_VERSION_VALUE) {
		printf("LoRa radio init success\n");
	} else {
		printf("LoRa radio init failed. Check wiring.\n");
	}

	// testing setRegOpMode
	printf("setting sleep mode\n");
	setRegOpMode(RFM9X_MODE_SLEEP);
	vTaskDelay(pdMS_TO_TICKS(10));

	// testing getRegOpMode
	regOpMode = getRegOpMode();
	printf("RegOpMode: 0x%x\n", regOpMode);
	vTaskDelay(pdMS_TO_TICKS(10));

	if (regOpMode != (RFM9X_MODE_SLEEP | RFM9X_LONG_RANGE_MODE)) {
		printf("Failed to set sleep mode\n");
	} else {
		printf("Sleep mode set successfully\n");
	}

	// testing getCurrentOpMode
	getCurrentOpMode();

	// testing getRegOpLoraMode
	regOpMode = getRegOpLoraMode();
	if (regOpMode == RFM9X_MODE_SLEEP) {
		printf("LoRa radio is in sleep mode\n");
	} else {
		printf("LoRa radio is not in sleep mode\n");
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
        .miso_io_num   = GPIO_NUM_MISO,
        .mosi_io_num   = GPIO_NUM_MOSI,
        .sclk_io_num   = GPIO_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
		.max_transfer_sz = 0
    };

	ret = spi_bus_initialize(SPI3_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
	ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	assert(ret == ESP_OK);

	// set SPI device config and mode to CPOL=0, CPHA=0
	spi_device_interface_config_t spi_device_interface_config = {
		.clock_speed_hz = CLOCK_SPEED_HZ * 1000 * 1000,
		.mode = 0,
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

esp_err_t register_write(int reg, int value)
{
	uint8_t buffer_out[2] = { reg | WNR_BIT_MASK, value };

	spi_transaction_t t = {
		.flags = 0,    // transaction line mode
		.length = 8 * sizeof(buffer_out),
		.tx_buffer = buffer_out,
		.rx_buffer = NULL
	};

	gpio_set_level(GPIO_NUM_CS, 0);
	spi_device_transmit(spi_handle, &t);
	gpio_set_level(GPIO_NUM_CS, 1);

	return ESP_OK;
}

uint8_t getRegOpMode()
{
	uint8_t regOpMode = register_read(RFM9X_01_REG_OP_MODE);

	return regOpMode;
}

uint8_t setRegOpMode(uint8_t mode)
{
	switch (mode) {
		case RFM9X_MODE_SLEEP:
		case RFM9X_MODE_STDBY:
		case RFM9X_MODE_FSTX:
		case RFM9X_MODE_TX:
		case RFM9X_MODE_FSRX:
		case RFM9X_MODE_RXCONTINUOUS:
		case RFM9X_MODE_RXSINGLE:
			register_write(RFM9X_01_REG_OP_MODE, mode | RFM9X_LONG_RANGE_MODE);
			break;

		default:
			printf("Unknown mode\n");
	}

	return getRegOpMode();
}

uint8_t getRegOpLoraMode()
{
	uint8_t regOpMode = getRegOpMode();
	uint8_t mode = regOpMode & 0x07;

	return mode;
}

/**
 * Get the current operation mode of the RFM9x module
*/
void getCurrentOpMode()
{
	uint8_t regOpMode = getRegOpMode();
    uint8_t mode = regOpMode & 0x80;

	if (mode == RFM9X_LONG_RANGE_MODE) {
		printf("LoRa mode: ");
		mode = regOpMode & 0x07;

		switch (mode) {
			case RFM9X_MODE_SLEEP:
				printf("Sleep mode\n");
				break;
			case RFM9X_MODE_STDBY:
				printf("Stdby mode\n");
				break;
			case RFM9X_MODE_FSTX:
				printf("FS mode TX (FSTx)\n");
				break;
			case RFM9X_MODE_TX:
				printf("Transmitter mode (Tx)\n");
				break;
			case RFM9X_MODE_FSRX:
				printf("FS mode RX (FSRx)\n");
				break;
			case RFM9X_MODE_RXCONTINUOUS:
				printf("Continuous receiver mode (Rx)\n");
				break;
			case RFM9X_MODE_RXSINGLE:
				printf("Single receiver mode (Rx)\n");
				break;
			default:
				printf("Unknown mode\n");
		}

	} else {
		printf("FSK/OOK mode\n");
	}
}
