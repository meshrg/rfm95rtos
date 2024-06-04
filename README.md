# RFM95/96/97/98(W) LoRa modem driver for ESP-IDF

This repository contains an ESP-IDF driver for a RFM95/96/97/98(W) LoRa modem connected over SPI.

It is used for a Masters degree project at Universidad de Zacatecas (UAZ) about an ESP32 LoRaWAN node with FreeRTOS.

## Using the component

Build as usual:
```bash
cd examples\rfm95rtos-rx
idf.py build
```
And flash it to the board:
```bash
idf.py -p PORT flash monitor
```

The examples uses the following GPIOs:

Tested on SPI3_HOST (VSPI) with the following pins:
 - GPIO_NUM_MOSI (23)
 - GPIO_NUM_MISO (19)
 - GPIO_NUM_SCLK (18)
 - GPIO_NUM_CS   ( 5)
 - GPIO_NUM_RST  (13)
 - GPIO_NUM_G0   (27)
 - GPIO_NUM_G1   (26)

Because this is based on MCCI/LMIC library, it needs the DIO0 and DIO1 pins to get instant status information from the radio:

DIO0: TxDone and RxDone
DIO1: RxTimeout

When using only LoRa mode the additional G1 (DIO1) is not used.

## Examples

This component have the following examples:

* rfm95rtos-tx

Sample program for transmiting a message using LoRa with RadioHead's packet format

* rfm95rtos-rx

Sample program for receiving a message using LoRa with RadioHead's packet format

* ttn_hello_world

Sample program that uses the ttn-esp32 library to create a TTN node using FreeRTOS on ESP-IDF. This uses the Frequency of 915 MHz, if you need to change it run  > idf.py menuconfig.
The settings are in Component config ---> The Things Network ---> TTN LoRa frequency / region

## License

This component is created for a Masters Degree project.

* Masters in Engineering for Technological Innovation
* Specialization: IoT and Embedded Systems.
* Project: LoRaWAN Node with ESP32, RFM9x and FreeRTOS
* UAZ. Universidad Autonoma de Zacatecas, Mexico.
* Director: Dr. Viktor Iván Rodriguez Abdalá (abdala@uaz.mx)
* Student: Fernando Valderrábano Reyes (fernando.valderrabano@uaz.mx)

This component is provided under Apache 2.0 license, see [LICENSE](LICENSE.md) file for details.

## Contributing

Please check [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.
