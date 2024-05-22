# API Reference

## Header files

- [rfm95rtos.h](#file-rfm95rtos)

## File rfm95rtos.h

_RFM95/96/97/98(W) LoRa modem driver for ESP-IDF._

To use this driver:

* Initialize SPI with [**spi\_init()**](#function-spi_init)
* Initialize radio with default settings with [**radio\_init()**](#function-radio_init)
* Call [**send()**](#function-send) to send a message with RadioHead packet format.
* Call [**setRxMode()**](#function-setRxMode) to set continuous rx mode.

Take a look at rfm95rtos-rx example for transmitting and rfm95rtos-tx for receiving.

