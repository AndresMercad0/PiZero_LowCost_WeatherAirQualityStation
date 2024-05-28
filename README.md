# README

## A Raspberry Pi Zero W 2 based weather and air quality station with low-cost sensors

## Overview

This project demonstrates the implementation of a weather and air quality monitoring system using the Raspberry Pi Zero W 2 and a set of low-cost sensors. The sensors included in this setup are the SHT45 for temperature and humidity, the SGP40 for gas measurements, and the Sparkfun Weather Meter Kit for measuring wind speed and direction.

The following documentation provides detailed instructions on setting up the sensors, wiring them to the Raspberry Pi, and running the provided Python code to gather and display environmental data.

## Author Contact

Feel free to contact the author by email at andres@mevel.com.mx.

## Date and Location

- **Date:** 2024-May-28 2:24 PM London Time
- **Author:** Andres A. Mercado-Velazquez
- **Location:** IoT Lab at Queen Mary University of London

## Hardware Requirements

- **Raspberry Pi Zero W 2**
  - [Raspberry Pi Zero W 2 Documentation](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/)
  - [Raspberry Pi Pinout](https://pinout.xyz)

- **Sensors**
  - **SHT45 (I2C)**
    - [Pinout](https://learn.adafruit.com/assets/99235)
    - [Tutorial](https://learn.adafruit.com/adafruit-sht40-temperature-humidity-sensor/python-circuitpython#python-computer-wiring-3082732)
  - **SGP40 (I2C)**
    - [Pinout](https://learn.adafruit.com/assets/98203)
    - [Tutorial](https://learn.adafruit.com/adafruit-sgp40/python-circuitpython)
  - **Sparkfun Weather Meter Kit**
    - [Documentation](https://learn.sparkfun.com/tutorials/weather-meter-hookup-guide)
    - [Pinout for ADS1115](https://learn.adafruit.com/assets/112709)
    - [Tutorial](https://projects.raspberrypi.org/en/projects/build-your-own-weather-station/5)

## Software Requirements

- Python 3
- Required Python libraries:
  - `adafruit-circuitpython-sht4x`
  - `adafruit-circuitpython-sgp40`
  - `adafruit-circuitpython-ads1x15`
  - `gpiozero`

## Setup and Wiring

### SHT45 Sensor
```
  *   SHT45             Raspberry Pi Zero W 2
  *   1 VIN ---------------- 3.3V - Pin 1
  *   2 GND ---------------- GND - Pin 6
  *   3 SCL ---------------- SCL - Pin 5
  *   4 SDA ---------------- SDA - Pin 3
```

### SGP40 Sensor
```
  *   SGP40             Raspberry Pi Zero W 2
  *   1 VIN ---------------- 3.3V - Pin 1
  *   2 GND ---------------- GND - Pin 6
  *   3 SCL ---------------- SCL - Pin 5
  *   4 SDA ---------------- SDA - Pin 3
```

### Wind Speed and Direction (Sparkfun Weather Meter Ki) & ADS1115
```
  *   Raspberry Pi Zero W 2                           ADS1115            Weather Meter Kit
  *       3.3V - Pin 1   -------------------------------- VIN ------------------ RED
  *       GND - Pin 6    -------------------------------- GND ------------------ BLACK
  *       SCL - Pin 5    -------------------------------- SCL 
  *       SDA - Pin 3    -------------------------------- SDA
  *       3.3V - Pin 1   --------- 10k resistor ---------  A0 ------------------ GREEN
  *       GPIO 4 - Pin 7 ------------------------------------------------------- YELLOW
```

## Execution
To run the code, ensure you have Python 3 installed along with the necessary libraries. You can install the required libraries using the following commands:

```bash
sudo apt-get install python3-pip
sudo pip3 install adafruit-circuitpython-sht4x
sudo pip3 install adafruit-circuitpython-sgp40
sudo pip3 install adafruit-circuitpython-ads1x15
sudo pip3 install gpiozero
```
```bash
Execute the script with:
python3 codeAllSensors.py
```

## License

This code is licensed under the MIT License. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

Although it is not a requirement of the license, the author, Andres A. Mercado-Velazquez, would appreciate it if you give credit when using or distributing this code.