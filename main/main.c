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
 * Posgrado en Ingenieria para la Innovacion Tecnologica (PIIT)
 * Postgraduate in Engineering for Technological Innovation
 * Project: LoRa Node with ESP32, RFM9x and FreeRTOS
 *
 * UAZ. Universidad Autonoma de Zacatecas, Mexico.
 *
 * TODO:
 * 	- move register addresses to a header file
 *  - move function prototypes to a header file
 *  - move functions to separete driver file
 *  - add more comments and function documentation
 *
 *
 * Fernando Valderrabano (fernando.valderrabano@uaz.mx)
*/
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>
#include <driver/gpio.h>
#include "driver/spi_master.h"

// define SPI3_HOST / VSPI pins
#define GPIO_NUM_MOSI 							23
#define GPIO_NUM_MISO 							19
#define GPIO_NUM_SCLK 							18
#define GPIO_NUM_CS    			 				5
#define GPIO_NUM_RST  							13
#define GPIO_NUM_G0   							27

#define TAG 		  							"RFM9x"
#define SPI_WRITE_BIT_MASK 						0x80
// clock speed in MHz
#define CLOCK_SPEED_MHZ							5
// +20dBm power
#define PA_BOOST_PIN 							0x80

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

// RadioHead packet format
#define RFM9X_HEADER_LEN 						4
#define RFM9X_BROADCAST_ADDR 					0xff
#define IRQ_TX_DONE_MASK 						0x08

// initialize SPI and radio functions
esp_err_t spi_init(void);
void reset_radio(void);
void radio_init(void);
void check_radio_version(void);

// register read/write functions
int register_read(int reg);
esp_err_t register_write(int reg, int value);

// LoRa radio functions
void setPreambleLength(uint16_t length);
uint8_t getPreambleLength();
void setFrequency(uint32_t frequency);
uint32_t getFrecuency();
void setTxPower(int8_t power);
uint8_t getTxPower();
void setBandwidth(double bandwidth);
uint8_t getSignalBandwidth();
void setCodingRate(uint8_t denominator);
uint8_t getCodingRate();
void setImplicitHeaderMode();
void setExplicitHeaderMode();
void setSpreadingFactor(uint8_t sf);
uint8_t getSpreadingFactor();
void enableCRC();
void disableCRC();
void send(char *data);
void waitPacketSent();

// operation mode functions
uint8_t getRegOpMode();
uint8_t getRegOpLoraMode();
uint8_t setRegOpMode(uint8_t mode);
void getCurrentOpMode();

static spi_device_handle_t spi_handle;
// static uint8_t _sequenceNumber = 0;

void app_main(void)
{
	// uint8_t regOpMode;

	spi_init();

	radio_init();

	printf("LoRa radio init done\n");

	printf("Set frequency to 915 MHz\n");

	// set tx power = 23 dBm
	// printf("Set tx power to 23 dBm\n");
	// setTxPower(23);

	send("Hello World");
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
		.clock_speed_hz = CLOCK_SPEED_MHZ * 1000 * 1000,
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
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level(GPIO_NUM_RST, 1);
	vTaskDelay(pdMS_TO_TICKS(10));
	ESP_LOGI(TAG, "reseting lora radio. Done.");
}

/**
 * Initialize default radio settings
 */
void radio_init(void)
{
	// reset radio
	reset_radio();

	// check device present
	check_radio_version();

    // set sleep mode, so we can also set LoRa mode:
	setRegOpMode(RFM9X_MODE_SLEEP);
	vTaskDelay(pdMS_TO_TICKS(10));

    // set up FIFO
	register_write(RFM9X_0E_REG_FIFO_TX_BASE_ADDR, 0);
	register_write(RFM9X_0F_REG_FIFO_RX_BASE_ADDR, 0);

	// set stby mode
	setRegOpMode(RFM9X_MODE_STDBY);
	vTaskDelay(pdMS_TO_TICKS(10));

	// settings compatible with RadioHead
	// mode Bw125Cr45Sf128 with AGC enabled
	// BW = 125kHz, CR 4/5, SF = 128chips/symbol (SF7)
	setBandwidth(125e3);
	setCodingRate(5);
	setSpreadingFactor(7);
	enableCRC();
	setImplicitHeaderMode();

	// the default is 13 dBm
	setTxPower(13);

	// default 8 bytes to match radiohead packet
	setPreambleLength(8);

	// set frequency = 915 MHz
	setFrequency(915000000);
	// printf("Frequency: %" PRIu32 "\n", getFrecuency());
}

void check_radio_version(void)
{
	uint8_t version_register;
	uint8_t result;

	version_register = RFM9X_42_REG_VERSION;
	printf("Reading register: %x\n", version_register);
	result = register_read(version_register);
	printf("fetched result is: 0x%x\n", result);
	if (result == RFM9X_42_REG_VERSION_VALUE) {
		printf("LoRa radio init success\n");
	} else {
		printf("LoRa radio init failed. Check wiring.\n");
	}
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
	uint8_t buffer_out[2] = { reg | SPI_WRITE_BIT_MASK, value };

	spi_transaction_t t = {
		.flags = 0,
		.length = 8 * sizeof(buffer_out),
		.tx_buffer = buffer_out,
		.rx_buffer = NULL
	};

	gpio_set_level(GPIO_NUM_CS, 0);
	spi_device_transmit(spi_handle, &t);
	gpio_set_level(GPIO_NUM_CS, 1);

	return ESP_OK;
}

/*
 * Set the preamble length
 * param length, can be from 6 to 65535 symbols
*/
void setPreambleLength(uint16_t length)
{
	// write the MSB in case length > 255
	register_write(RFM9X_20_REG_PREAMBLE_MSB, (length >> 8) & 0xff);
	// write the LSB
	register_write(RFM9X_21_REG_PREAMBLE_LSB, length & 0xff);
}

/*
 * Get the preamble length
*/
uint8_t getPreambleLength()
{
	uint8_t msb = register_read(RFM9X_20_REG_PREAMBLE_MSB);
	uint8_t lsb = register_read(RFM9X_21_REG_PREAMBLE_LSB);

	uint16_t length = (msb << 8) | lsb;

	return length;
}

void setFrequency(uint32_t frequency)
{
	// calculate frf register value
	uint64_t frf = ((uint64_t)frequency << 19) / RFM9X_FXOSC;

	register_write(RFM9X_06_REG_FRF_MSB, (frf >> 16) & 0xff);
	register_write(RFM9X_07_REG_FRF_MID, (frf >> 8) & 0xff);
	register_write(RFM9X_08_REG_FRF_LSB, frf & 0xff);
}

uint32_t getFrecuency()
{
	uint32_t frequency = 0;

	uint8_t msb = register_read(RFM9X_06_REG_FRF_MSB);
	uint8_t mid = register_read(RFM9X_07_REG_FRF_MID);
	uint8_t lsb = register_read(RFM9X_08_REG_FRF_LSB);

	frequency = ((msb << 16) | (mid << 8) | lsb) * RFM9X_FXOSC / (1 << 19);

	return frequency;
}

/* set output power in dBm */
void setTxPower(int8_t power)
{
	// set limits
	if (power > 17) {
		power = 17;
	} else if (power < 2) {
		power = 2;
	}

	register_write(RFM9X_09_REG_PA_CONFIG, PA_BOOST_PIN | (power - 2));
}

uint8_t getTxPower()
{
	uint8_t regPaConfig = register_read(RFM9X_09_REG_PA_CONFIG);

	uint8_t power = regPaConfig & 0x0f;

	return power + 2;
}

void setBandwidth(double bandwidth)
{
	uint8_t signal_bw;
	uint8_t regModemConfig1 = register_read(RFM9X_1D_REG_MODEM_CONFIG1);

	if (bandwidth <= 7.8e3) {
		signal_bw = 0b00000000;
	} else if (bandwidth <= 10.4e3) {
		signal_bw = 1;
	} else if (bandwidth <= 15.6e3) {
		signal_bw = 2;
	} else if (bandwidth <= 20.8e3) {
		signal_bw = 3;
	} else if (bandwidth <= 31.25e3) {
		signal_bw = 4;
	} else if (bandwidth <= 41.7e3) {
		signal_bw = 5;
	} else if (bandwidth <= 62.5e3) {
		signal_bw = 6;
	} else if (bandwidth <= 125e3) {
		signal_bw = 7;
	} else if (bandwidth <= 250e3) {
		signal_bw = 8;
	} else if (bandwidth <= 500e3) {
		signal_bw = 9;
	} else {
		printf("Invalid bandwidth\n");
		return;
	}

	// Bw are in bits 7-4
	regModemConfig1 = (regModemConfig1 & 0x0f) | (signal_bw << 4);

	register_write(RFM9X_1D_REG_MODEM_CONFIG1, regModemConfig1);
}

/*
 * set implicit header mode
 */
void setImplicitHeaderMode()
{
	uint8_t regModemConfig1 = register_read(RFM9X_1D_REG_MODEM_CONFIG1);

	// set bit 0 = 1
	regModemConfig1 |= 0x01;

	register_write(RFM9X_1D_REG_MODEM_CONFIG1, regModemConfig1);
}

/*
 * set explicit header mode
 */
void setExplicitHeaderMode()
{
	uint8_t regModemConfig1 = register_read(RFM9X_1D_REG_MODEM_CONFIG1);

	// clear bit 0
	regModemConfig1 &= 0xfe;

	register_write(RFM9X_1D_REG_MODEM_CONFIG1, regModemConfig1);
}

/*
 * Set the spreading factor
 * param sf, spreading factor from 6 to 12
 * sf 6 is a special case and uses implicit header mode
*/
void setSpreadingFactor(uint8_t sf)
{
	uint8_t regModemConfig2 = register_read(RFM9X_1E_REG_MODEM_CONFIG2);

	if (sf < 6 || sf > 12) {
		printf("Invalid spreading factor\n");
		return;
	}

	// SF are in bits 7-4 (SpreadingFactor of register RegModemConfig2)
	regModemConfig2 = (regModemConfig2 & 0x0f) | ((sf << 4) & 0xf0);

	register_write(RFM9X_1E_REG_MODEM_CONFIG2, regModemConfig2);
}

/*
 * Get the spreading factor
*/
uint8_t getSpreadingFactor()
{
	uint8_t regModemConfig2 = register_read(RFM9X_1E_REG_MODEM_CONFIG2);

	uint8_t sf = (regModemConfig2 >> 4) & 0x0f;

	return sf;
}

/*
* Get the signal bandwidth number from 0 to 9
*/
uint8_t getSignalBandwidth()
{
	uint8_t regModemConfig1 = register_read(RFM9X_1D_REG_MODEM_CONFIG1);

	uint8_t bw = (regModemConfig1 >> 4) & 0x0f;

	return bw;
}

/*
 * Set the coding rate denominator (4/x)
*/
void setCodingRate(uint8_t denominator)
{
	uint8_t regModemConfig1 = register_read(RFM9X_1D_REG_MODEM_CONFIG1);

	if (denominator < 5 || denominator > 8) {
		printf("Invalid denominator\n");
		return;
	}

	// CR are in bits 3-1 with an offset of 4
	// 0xf1 is used to clear the bits 3-1
	regModemConfig1 = (regModemConfig1 & 0xf1) | ((denominator - 4) << 1);

	register_write(RFM9X_1D_REG_MODEM_CONFIG1, regModemConfig1);
}

/*
 * Get the coding rate denominator (4/x)
*/
uint8_t getCodingRate()
{
	uint8_t regModemConfig1 = register_read(RFM9X_1D_REG_MODEM_CONFIG1);

	uint8_t denominator = (regModemConfig1 >> 1) & 0x07;

	return denominator + 4;
}

/*
 * Enable CRC
*/
void enableCRC()
{
	uint8_t regModemConfig2 = register_read(RFM9X_1E_REG_MODEM_CONFIG2);

	// set bit 2 = 1
	regModemConfig2 |= 0x04;

	register_write(RFM9X_1E_REG_MODEM_CONFIG2, regModemConfig2);
}

/*
 * Disable CRC
*/
void disableCRC()
{
	uint8_t regModemConfig2 = register_read(RFM9X_1E_REG_MODEM_CONFIG2);

	// clear bit 2
	regModemConfig2 &= 0xfb;

	register_write(RFM9X_1E_REG_MODEM_CONFIG2, regModemConfig2);
}

/*
 * Get the current operation mode
 */
uint8_t getRegOpMode()
{
	uint8_t regOpMode = register_read(RFM9X_01_REG_OP_MODE);

	return regOpMode;
}

/*
 * Set the operation mode
 */
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

/*
 * Get the LoRa operation mode
  */
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

/**
 * Send data using the LoRa radio module
 * payload is TO + FROM + ID + FLAGS + message data
*/
void send(char *data)
{
	// waitPacketSent();

	// // clear FIFO
	// setRegOpMode(RFM9X_MODE_STDBY);
	// vTaskDelay(pdMS_TO_TICKS(10));

	// // set FIFO address pointer
	// register_write(RFM9X_0D_REG_FIFO_ADDR_PTR, 0);

	// // RadioHead packet format
	// printf("writing header to FIFO\n");
	// register_write(RFM9X_00_REG_FIFO, RFM9X_BROADCAST_ADDR);
	// register_write(RFM9X_00_REG_FIFO, RFM9X_BROADCAST_ADDR);
	// register_write(RFM9X_00_REG_FIFO, RFM9X_BROADCAST_ADDR);

	// // send data
	// printf("writing data to FIFO\n");
	// for (int i = 0; i < strlen(data); i++) {
    //     register_write(RFM9X_00_REG_FIFO, data[i]);
    // }

	// // set tx mode
	// if (getRegOpMode() != RFM9X_MODE_TX) {
	// 	printf("setting tx mode\n");
	// 	setRegOpMode(RFM9X_MODE_TX);
	// 	vTaskDelay(pdMS_TO_TICKS(10));
	// } else {
	// 	printf("already in tx mode\n");
	// }

	// // interrupt on TxDone
	// printf("setting DIO0 to TxDone\n");
	// register_write(RFM9X_40_REG_DIO_MAPPING1, 0x40);

	// waitPacketSent();
}

void waitPacketSent()
{
	if (getRegOpLoraMode() == RFM9X_MODE_TX) {
		while (1) {
			printf("waiting for packet sent\n");
			uint8_t irqFlags = register_read(RFM9X_12_REG_IRQ_FLAGS);
			if (irqFlags & 0x08) {
				break;
			}
		}
		register_write(RFM9X_12_REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
		printf("packet sent\n");
	}
}
