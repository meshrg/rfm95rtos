# RFM95/96/97/98(W) LoRa modem driver for ESP-IDF

This repository contains an ESP-IDF driver for a RFM95/96/97/98(W) LoRa modem connected over SPI.

It is used for a Masters degree project at Universidad de Zacatecas (UAZ) about a LoRa node with FreeRTOS.

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

The example uses the following GPIOs:

 This uses the SPI3_HOST (VSPI) on the ESP32 with the following pins:
 - GPIO_NUM_MOSI (23)
 - GPIO_NUM_MISO (19)
 - GPIO_NUM_SCLK (18)
 - GPIO_NUM_CS   ( 5)
 - GPIO_NUM_RST  (13)
 - GPIO_NUM_G0   (27)

## License

This component is provided under Apache 2.0 license, see [LICENSE](LICENSE.md) file for details.

## Contributing

Please check [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.
