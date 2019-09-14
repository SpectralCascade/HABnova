# Wisp-1
The source code for my first High-Altitude Balloon payload tracker and data logger system, designed for the PIC16F18446 (PDIP20) microcontroller. Developed in C using Microchip MPLAB X IDE with the XC-8 compiler and Microchip Code Configurator.

## Features:
- UBlox Max-M8C GPS configured in flight-mode over EUSART (UART)
- BME280 environment sensor measuring temperature, pressure and humidity
- RTTY transmission complying with the [UKHAS protocol](https://ukhas.org.uk/communication:protocol)

## Coming soon (should time allow):
- Data logging to micro-SD card with FatFs (or possibly Petit FatFs)
- Minature TTL serial JPEG camera image capture at 640x480 resolution
