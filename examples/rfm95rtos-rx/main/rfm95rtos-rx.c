#include <stdio.h>
#include <inttypes.h>
#include "rfm95rtos.h"

/*
 * Example of receiving a message using the rfm95rtos library.
**/
void app_main(void)
{
    spi_init();

	radio_init();

	printf("LoRa radio init done\n");

	printf("Set tx power to 23 dBm\n");
	setTxPower(23);

	// receiver example
	int i = 0;
	uint8_t messageBuffer[MAX_PAYLOAD_LENGTH];

	while (1) {
		setRxMode();
		vTaskDelay(pdMS_TO_TICKS(1000));

		uint8_t irq_flags = register_read(RFM9X_12_REG_IRQ_FLAGS);

		if (irq_flags & IRQ_RX_DONE_MASK) {
			printf("Packet received\n");

			// check the CrcOnPayload flag
			uint8_t regHopChannel = register_read(RFM9X_1C_REG_HOP_CHANNEL);

			// CRC Information extracted from the received packet header
			if (regHopChannel & 0x40) {
				printf("Header indicates CRC on\n");
				// check PayloadCrcError on the 6th bit of irq_flags
				if (irq_flags & 0x20) {
					printf("CRC error\n");
				}
			} else {
				printf("Header indicates CRC off\n");
			}

			// check the validHeader flag
			if (irq_flags & 0x10) {
				printf("Valid header\n");
			} else {
				printf("Invalid header\n");
				continue;
			}

			// RegRxNbBytes indicates the number of bytes that have been received
			uint8_t length = register_read(RFM9X_13_REG_RX_NB_BYTES);
			printf("Length: %d\n", length);

			// reset the fifo pointer to the beginning of the packet
			register_write(RFM9X_0D_REG_FIFO_ADDR_PTR, register_read(RFM9X_25_REG_FIFO_RX_BYTE_ADDR));

			// read the packet
			for (i = 0; i < length; i++) {
				messageBuffer[i] = register_read(RFM9X_00_REG_FIFO);
			}

			// print the message
			printf("Message: %s\n", messageBuffer);

			// send("And hello back to you");

			// clear the RX_DONE and IRQ flags
			register_write(RFM9X_12_REG_IRQ_FLAGS, 0xff);
		}
	}
}
