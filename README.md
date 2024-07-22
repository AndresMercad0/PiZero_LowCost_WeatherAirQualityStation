# A Raspberry Pi Zero W 2 based weather and air quality monitoring station with low-cost sensors

## Overview

This project demonstrates the implementation of a weather and air quality monitoring system using the Raspberry Pi Zero W 2 and a set of low-cost sensors. The sensors included in this setup are the SHT45 for temperature and humidity, the SGP40 for gas measurements, the SPS30 for particulate matter (PM) PM10 and PM2.5, the Grove - Gas Sensor V2 (Multichannel) for CO and NO2, and the Sparkfun Weather Meter Kit for measuring wind speed and direction.

The following documentation provides detailed instructions on setting up the sensors, wiring them to the Raspberry Pi, and running the provided Python code to gather and display environmental data.

## Author Contact

Feel free to contact the author by email at andres@mevel.com.mx.

## Date and Location

- **Date:** 2024-July-22 2:10 PM London Time
- **Author:** Andres A. Mercado-Velazquez
- **Location:** IoT Lab at Queen Mary University of London

## Hardware Requirements

- **Raspberry Pi Zero W 2**
  - [Raspberry Pi Zero W 2 Documentation](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/)
  - [Raspberry Pi Pinout](https://pinout.xyz)

- **Sensors**
  - **I2C**
    - **SHT45 (I2C)**
      - [Documentation](https://learn.adafruit.com/adafruit-sht40-temperature-humidity-sensor/python-circuitpython)
    - **SGP40 (I2C)**
      - [Documentation](https://learn.adafruit.com/adafruit-sgp40/python-circuitpython)
    - **SPS30 (I2C)**
      - [Documentation](https://cdn.sparkfun.com/assets/2/d/2/a/6/Sensirion_SPS30_Particulate_Matter_Sensor_v0.9_D1__1_.pdf)
    - **Grove - Multichannel Gas Sensor v2 (I2C)**
      - [Documentation](https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/)
  - **Analogue or digital communication**
    - **Sparkfun Weather Meter Kit**
      - [Documentation](https://learn.sparkfun.com/tutorials/weather-meter-hookup-guide)
      - **ADS1115**
        - [Documentation](https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/python-circuitpython)
      - **MQ131 Ozone Gas Sensor**
        - [Documentation](https://cdn.sparkfun.com/assets/9/9/6/e/4/mq131-datasheet-low.pdf)



## Software Requirements
- Python 3
- pip3
- Required Python modules (sudo pip3 install name_of_module):
  - `gpiozero`
  - `adafruit-circuitpython-board`
  - `adafruit-circuitpython-sht4x`
  - `adafruit-circuitpython-sgp40`
  - `adafruit-circuitpython-ads1x15`

## Setup and Wiring
### SHT45 Sensor
```
    SHT45             Raspberry Pi Zero W 2
    ---------------------------------------
    1 VIN ---------------- 3.3V - Pin 1
    2 GND ---------------- GND - Pin 6
    3 SCL ---------------- SCL - Pin 5
    4 SDA ---------------- SDA - Pin 3
```

### SGP40 Sensor
```
    SGP40             Raspberry Pi Zero W 2
    ---------------------------------------
    1 VIN ---------------- 3.3V - Pin 1
    2 GND ---------------- GND - Pin 6
    3 SCL ---------------- SCL - Pin 5
    4 SDA ---------------- SDA - Pin 3
```

### SPS30 Sensor
```
     SPS30                        Raspberry Pi Zero W 2
    --------------------------------------------------
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

### Grove Multichannel Gas v2
```
    Grove Multichannel gas V2         Raspberry Pi Zero W 2
    -------------------------------------------------------
        1 GND -------------------------- GND - Pin 6/9
        2 VCC -------------------------- 5V - Pin 2/4
        3 SDA -------------------------- SDA - Pin 3
        4 SCL -------------------------- SCL - Pin 5
```

### Wind Speed and Direction (Sparkfun Weather Meter Ki) & ADS1115
```
    Raspberry Pi Zero W 2                             ADS1115                               Weather Meter Kit
    ----------------------------------------------------------------------------------------------------------
        SCL - Pin 5    -------------------------------- SCL                      
        SDA - Pin 3    -------------------------------- SDA                      
        5V - Pin 2/4   --.--.-------------------------- VIN                      
                         |  '--------------------------  A3 (Input voltage reference)
                         '-------- 10k resistor -------  A0 ------------------------------------- GREEN
        GND - Pin 6/9  -------------------------------- GND ------------------------------------- BLACK
        3.3V - Pin 1   -------------------------------------------------------------------------- RED
        GPIO 4 - Pin 7 -------------------------------------------------------------------------- YELLOW
```

### MQ131 (with FC-22 Board) [Ozone Gas Sensor] & ADS1115
```
    Raspberry Pi Zero W 2              ADS1115          MQ131 (with FC-22 Board)
    ----------------------------------------------------------------------------
        SCL - Pin 5    ----------------- SCL
        SDA - Pin 3    ----------------- SDA
        5V - Pin 2/4   ----------------- VIN ----------------- VCC
        GND - Pin 6/9  ----------------- GND ----------------- GND
                                          A1 ----------------- A0
```

## Execution
To run the code, ensure you have Python 3 installed along with the necessary libraries. You can install the required libraries using the following commands:

```bash
sudo apt-get install python3-pip
sudo pip3 install gpiozero
sudo pip3 install adafruit-circuitpython-board
sudo pip3 install adafruit-circuitpython-sht4x
sudo pip3 install adafruit-circuitpython-sgp40
sudo pip3 install adafruit-circuitpython-ads1x15
```

### Run the code
1. Clone this repo to your directory using the command:
```bash
git clone https://github.com/AndresMercad0/RPi-MultichannelGasV2-PythonLib.git
```
2. Run the code with the command:
```bash
python3 codeAllSensors.py
```



> [!WARNING]
> ## CALIBRATE SENSOR Grove Multichannel Gas v2
> by Veselin Hadzhiyski 2021 (vcoder@abv.bg)
> ### How to calibrate?
> Visit [this Repo](https://github.com/AndresMercad0/RPi-MultichannelGasV2-PythonLib), clone it, and run the script. Wait for the sensor to heat up. If this is the sensor's first run, you must preheat it for more than 72 hours. If it's already preheated, run it for a few hours to achieve stable parameters. After that, note the R_gas value.
> 
> Next, record the R_gas value as the R0 value in this script. Specifically, write the R_gas value as the R0 value in "codeAllSensors.py". Please ensure you follow the calibration instructions on [this Repo](https://github.com/AndresMercad0/RPi-MultichannelGasV2-PythonLib).
> 
> And that's it! The sensor is ready for operation.

## To-do
- [x] SHT45 (I2C)
- [x] SGP40 (I2C)
- [x] Wind speed and direction (Sparkfun Weather Meter Kit) & ADS1115 (I2C)
- [x] SPS30 (I2C)
- [x] Grove Multichannel Gas Sensor V2 (I2C)
- [x] MQ131 & ADS1115 (I2C)

## License
[MIT](https://choosealicense.com/licenses/mit/)

> [!NOTE]
> Although it is not a requirement of the license, the author, Andres A. Mercado-Velazquez, would appreciate it if you give credit when using or distributing this code.