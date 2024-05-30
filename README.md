# A Raspberry Pi Zero W 2 based weather and air quality monitoring station with low-cost sensors

## Overview

This project demonstrates the implementation of a weather and air quality monitoring system using the Raspberry Pi Zero W 2 and a set of low-cost sensors. The sensors included in this setup are the SHT45 for temperature and humidity, the SGP40 for gas measurements, and the Sparkfun Weather Meter Kit for measuring wind speed and direction.

The following documentation provides detailed instructions on setting up the sensors, wiring them to the Raspberry Pi, and running the provided Python code to gather and display environmental data.

## Author Contact

Feel free to contact the author by email at andres@mevel.com.mx.

## Date and Location

- **Date:** 2024-May-30 5:52 PM London Time
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
    - [Tutorial](https://learn.adafruit.com/adafruit-sgp40/python-circuitpython#python-computer-wiring-3080640)
  - **Sparkfun Weather Meter Kit**
    - [Documentation](https://learn.sparkfun.com/tutorials/weather-meter-hookup-guide)
    - [Pinout for ADS1115](https://learn.adafruit.com/assets/112709)
    - [Tutorial](https://projects.raspberrypi.org/en/projects/build-your-own-weather-station/5)
  - **SPS30 (I2C)**
    - [Pinout](https://cdn.sparkfun.com/assets/2/d/2/a/6/Sensirion_SPS30_Particulate_Matter_Sensor_v0.9_D1__1_.pdf)
    - [Tutorial](https://github.com/dvsu/sps30/tree/main?tab=readme-ov-file#sensirion-sps30)


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
    SHT45              Raspberry Pi Zero W 2
    1 VIN ---------------- 3.3V - Pin 1
    2 GND ---------------- GND - Pin 6
    3 SCL ---------------- SCL - Pin 5
    4 SDA ---------------- SDA - Pin 3
```

### SGP40 Sensor
```
    SGP40              Raspberry Pi Zero W 2
    1 VIN ---------------- 3.3V - Pin 1
    2 GND ---------------- GND - Pin 6
    3 SCL ---------------- SCL - Pin 5
    4 SDA ---------------- SDA - Pin 3
```

### Wind Speed and Direction (Sparkfun Weather Meter Ki) & ADS1115
```
    Raspberry Pi Zero W 2                             ADS1115             Weather Meter Kit
        GND - Pin 6    -------------------------------- GND ------------------ BLACK
        3.3V - Pin 1   -----.-------------------------- VIN ------------------ RED
                            '----- 10k resistor -------  A0 ------------------ GREEN
        GPIO 4 - Pin 7 ------------------------------------------------------- YELLOW
        SCL - Pin 5    -------------------------------- SCL
        SDA - Pin 3    -------------------------------- SDA
```

### SPS30 Sensor
```
    SPS30                        Raspberry Pi Zero W 2
    Pin 1 - VDD ---------------- 5V - Pin 2/4
    Pin 2 - SDA ---------------- SDA - Pin 3
    Pin 3 - SCL ---------------- SCL - Pin 5
    Pin 4 - SEL ----.----------- GND - Pin 6/9
    Pin 5 - GND ----'

        .-------------------------------------------------.
        |  SPS30 PINOUT By Dave (https://github.com/dvsu) |
        '-------------------------------------------------'
                                           Pin 1   Pin 5
                                           |       |
                                           V       V
        .------------------------------------------------.
        |                                .-----------.   |
        |                                | x x x x x |   |
        |                                '-----------'   |
        |     []          []          []          []     |
        '------------------------------------------------'
```

## Execution
To run the code, ensure you have Python 3 installed along with the necessary libraries. You can install the required libraries using the following commands:

```bash
sudo apt-get install python3-pip
sudo pip3 install gpiozero
sudo pip3 install adafruit-circuitpython-sht4x
sudo pip3 install adafruit-circuitpython-sgp40
sudo pip3 install adafruit-circuitpython-ads1x15
```
Execute the script with:
```bash
python3 codeAllSensors.py
```

## To-do
- [x] SHT45 (I2C)
- [x] SGP40 (I2C)
- [x] Wind speed and direction (Sparkfun Weather Meter Kit) & ADS1115 (I2C)
- [x] SPS30 (I2C)
- [ ] Grove Multichannel Gas Sensor V2

## License
[MIT](https://choosealicense.com/licenses/mit/)

> [!NOTE]
> Although it is not a requirement of the license, the author, Andres A. Mercado-Velazquez, would appreciate it if you give credit when using or distributing this code.