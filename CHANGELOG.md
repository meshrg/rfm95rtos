# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

- Removed debugging code

## [1.0.0] - 2024-05-21

- Added receiver example.
- Added transmiter example.
- Moved all the code to the ESP-IDF driver component.
- Fixed a bug in the coding rate function that was not working.
- Setted explicit header mode to be compatible with RadioHead's packet.
- Added comments on each function to document the code.
- Added the send function to transmit data.
- Added a function to enable and disable CRC.
- Added a function to send a packet using the RadioHead format.

## [0.0.3] - 2023-05-06

- Added a function to set and retrieve Tx power.
- Added a function to set the bandwidth (BW).
- Added a function to set the coding rate (CR).
- Added a function to set header mode (implicit or explicit).
- Added a function to set the spreading factor (SF).


## [0.0.2] - 2023-03-25

- Added functions to init radio through SPI.
- Added functions to write and fetch radio registers.
