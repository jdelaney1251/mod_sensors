# Zephyr OOT Sensors Module

This repository is a collection of sensor drivers and utilities for use with 
the [Zephyr Framework](https://docs.zephyrproject.org/latest/introduction/index.html) that 
have been created during and for my own Zephyr RTOS-based projects. 
Some are still work in progress and others are completed but may not have been tested 
on your particular platform. For reference, they have all been tested on a board 
equivalent to a NUCLEO-L476RG (i.e. STM32L476RG).

The current list of driver implementations by peripheral/interface type are:
It should be noted that these drivers plug directly into the standard Zephyr peripheral interfaces.
Some modules still need more detailed comments but to those familiar with the relevant API 
interfacing with them will be self-explanatory. 
### ADC
- MCP3424 4-channel 18-bit I2C ADC

### Sensor
- BH1750 Ambient Light sensor
- SPW2430 MEMS microphone (SPL sensor only)

### Video
- Arducam SPI (WIP)