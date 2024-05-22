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
