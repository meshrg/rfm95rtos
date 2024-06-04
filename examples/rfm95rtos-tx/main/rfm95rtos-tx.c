/**
 * Sample program to transmit a message using the rfm95rtos library.
 *
 * This uses the SPI3_HOST (VSPI) on the ESP32 with the following pins:
 * - GPIO_NUM_MOSI (23)
 * - GPIO_NUM_MISO (19)
 * - GPIO_NUM_SCLK (18)
 * - GPIO_NUM_CS   ( 5)
 * - GPIO_NUM_RST  (13)
 * - GPIO_NUM_G0   (27)
 * - GPIO_NUM_G1   (26)
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

#include <stdio.h>
#include <inttypes.h>
#include "rfm95rtos.h"

/*
 * Example of sending a message using the rfm95rtos library.
**/
void app_main(void)
{
    spi_init();

	radio_init();

	printf("LoRa radio init done\n");

	printf("Set tx power to 23 dBm\n");
	setTxPower(23);

	send("Hello World!");

}
