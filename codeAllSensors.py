###############################################################################
#                                                                             #
#                      ──── LICENSE INFORMATION ────                          #
#                                                                             #
# This code is licensed under the MIT License.                                #
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  #
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    #
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.                       #
#                                                                             #
# In addition, while it is not a requirement of the license, I would greatly  #
# appreciate it if you give credit to the author,                             #
# Andres A. Mercado-Velazquez, when using or distributing this code.          #
#                                                                             #
###############################################################################

# Note: Use Python 3 to run the code and enjoy!
# Note 2: Feel free to contact the author of this code by email at andres@mevel.com.mx

'''
Code for Raspberry Pi Zero W 2 that runs the following sensors together:

1. SHT45 (I2C)
2. SGP40 (I2C)
3. Wind Speed and Direction - Sparkfun Weather Meter Kit (Speed: Digital, Direction: Analogue)
4. SPS30 (I2C)
5. Grove - Gas Sensor V2 (Multichannel) - I2C

'''


'''
  DATE:             2024-May-31 8:56 PM London Time
  AUTHOR:           Andres A. Mercado-Velazquez
  LOCATION:         IoT Lab at Queen Mary University of London
  BOARD:            Raspberry Pi Zero W 2
  BOARD DOC:        https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/
                    https://pinout.xyz
  REPO/CODE:        https://github.com/AndresMercad0/PiZero_LowCost_WeatherAirQualityStation


  #################################################################################
  #                    CONNECTING DEVICES/SENSORS TO THE BOARD                    #
  #################################################################################

  ------------------------------------------------------------------------------------------------
                                            I2C
  ------------------------------------------------------------------------------------------------

  ------------- SHT45 -----------------------------------------------------------------------------------------------------------------
  * Pinout     =>      https://learn.adafruit.com/assets/99235
  * Tutorial   =>      https://learn.adafruit.com/adafruit-sht40-temperature-humidity-sensor/python-circuitpython#python-computer-wiring-3082732
  * Library    =>      python3-pip (sudo apt-get install python3-pip)
                          |->  adafruit-circuitpython-sht4x (sudo pip3 install adafruit-circuitpython-sht4x)

    SHT45             Raspberry Pi Zero W 2
    1 VIN ---------------- 3.3V - Pin 1
    2 GND ---------------- GND - Pin 6
    3 SCL ---------------- SCL - Pin 5
    4 SDA ---------------- SDA - Pin 3

  ------------- SGP40 -----------------------------------------------------------------------------------------------------------------
  * Pinout     =>      https://learn.adafruit.com/assets/98203
  * Tutorial   =>      https://learn.adafruit.com/adafruit-sgp40/python-circuitpython#python-computer-wiring-3080640
  * Library    =>      python3-pip (sudo apt-get install python3-pip)
                          |->  adafruit-circuitpython-sgp40 (sudo pip3 install adafruit-circuitpython-sgp40)

    SGP40             Raspberry Pi Zero W 2
    1 VIN ---------------- 3.3V - Pin 1
    2 GND ---------------- GND - Pin 6
    3 SCL ---------------- SCL - Pin 5
    4 SDA ---------------- SDA - Pin 3

  ------------- SPS30 -----------------------------------------------------------------------------------------------------------------
  * Pinout     =>      https://cdn.sparkfun.com/assets/2/d/2/a/6/Sensirion_SPS30_Particulate_Matter_Sensor_v0.9_D1__1_.pdf
  * Tutorial   =>      https://github.com/dvsu/sps30/tree/main?tab=readme-ov-file#sensirion-sps30
  * Library    =>      https://github.com/dvsu/sps30/blob/main/sps30.py
                          |->  The file "sps30.py" from the public repository https://github.com/dvsu/sps30.git is responsible for managing the i2c communication with the SPS30 sensor. This file is invoked by "codeAllSensors.py" and is included in this repository.

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
  
  ------------- Grove - Gas Sensor V2 (Multichannel) -----------------------------------------------------------------------------------------------------------------
  * Pinout     =>      https://wiki.seeedstudio.com/Grove-Multichannel-Gas-Sensor-V2/
  * Source     =>      https://github.com/Seeed-Studio/Seeed_Arduino_MultiGas
  * Library    =>      python3-pip (sudo apt-get install python3-pip)
                                       |->  adafruit-circuitpython-board (sudo pip3 adafruit-circuitpython-board)

    Grove Multichannel gas V2         Raspberry Pi Zero W 2
        1 GND -------------------------- GND - Pin 6
        2 VCC -------------------------- 5V - Pin 4
        3 SDA -------------------------- SDA - Pin 3
        4 SCL -------------------------- SCL - Pin 5

        .-------------------------------------------------------------------.
        |                Calibrate Grove Multichannel Gas v2                |
        '-------------------------------------------------------------------'
        by Veselin Hadzhiyski 2021 (vcoder@abv.bg)

        How to calibrate?
        |
        '->  Visit [this Repo](https://github.com/AndresMercad0/RPi-MultichannelGasV2-PythonLib), clone it, and run the script.
             Wait for the sensor to heat up. If this is the sensor's first run, you must preheat it for more than 72 hours.
             If it's already preheated, run it for a few hours to achieve stable parameters. After that, note the R_gas value.
             Next, record the R_gas value as the R0 value in this script. Specifically, write the R_gas value as the R0 value in "codeAllSensors.py".
             Please ensure you follow the calibration instructions on [this Repo](https://github.com/AndresMercad0/RPi-MultichannelGasV2-PythonLib).
             And that's it! After edit the "R0" value in this code the sensor is ready for operation.


  ------------------------------------------------------------------------------------------------
                                  Analogue or digital communication
  ------------------------------------------------------------------------------------------------
  ------------- Wind speed and direction & ADS1115 -------------
  * DOC Weather Meter Kit    =>      https://learn.sparkfun.com/tutorials/weather-meter-hookup-guide
  * Pinout ADS1115           =>      https://learn.adafruit.com/assets/112709
  * Tutorial                 =>      https://projects.raspberrypi.org/en/projects/build-your-own-weather-station/5
  * Library                  =>      python3-pip (sudo apt-get install python3-pip)
                                       |->  adafruit-circuitpython-ads1x15 (sudo pip3 install adafruit-circuitpython-ads1x15)

    Raspberry Pi Zero W 2                             ADS1115             Weather Meter Kit
        GND - Pin 6    -------------------------------- GND ------------------ BLACK
        3.3V - Pin 1   -----.-------------------------- VIN ------------------ RED
                            '----- 10k resistor -------  A0 ------------------ GREEN
        GPIO 4 - Pin 7 ------------------------------------------------------- YELLOW
        SCL - Pin 5    -------------------------------- SCL
        SDA - Pin 3    -------------------------------- SDA

'''


''' 
 * LIBRARIES *
'''
# General Purpose
import time
import math
import json
import board # I2C
# SHT45
import adafruit_sht4x
# SGP40
import adafruit_sgp40
# Wind Speed - Weather Meter Kit
from gpiozero import Button
# Wind Direction - ADS1115 for Weather Meter Kit
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
# SPS30
from sps30library.sps30 import SPS30
# Grove - Gas Sensor V2 (Multichannel)
from MGSv2Lib.multichannel_gas_gmxxx import MultichannelGasGMXXX



'''
 * GLOBAL CONSTANTS *
'''
# Wind Speed - Weather Meter Kit
WIND_SPEED_SENSOR_PIN = 4
MEASUREMENT_TIME = 5  # Measurement time in seconds
SENSOR_DIAMETER_CM = 18  # Wind speed sensor diameter in cm

# Wind direction values
SENSOR_MAX = [1800, 2250, 2500, 3325, 4825, 6325, 7425, 10425, 11825, 15325, 16100, 17900, 20000, 21050, 22550, 24000]
SENSOR_MIN = [1550, 2000, 2251, 3075, 4575, 6075, 7175, 10175, 11575, 15075, 15850, 17650, 19750, 20800, 22300, 23750]
DIR_DEG = [22.5, 337.5, 0, 67.5, 45, 112.5, 90, 292.5, 315, 157.5, 135, 247.5, 270, 202.5, 225, 180]

# Grove - Gas Sensor V2 (Multichannel)
R0_CO = 31 # This value is obtained from the ambient air and must be determined through sensor calibration, as referenced in the code's initial comments.
R0_NO2 = 40 # This value is obtained from the ambient air and must be determined through sensor calibration, as referenced in the code's initial comments.


'''
 * GLOBAL VARIABLES *
'''
# Wind Speed - Weather Meter Kit
pulses = 0


'''
 *  INIT COMMUNICATIONS  *
'''
# Initialize I2C
i2c = board.I2C()  # Uses board.SCL and board.SDA
# SHT45
sht = adafruit_sht4x.SHT4x(i2c)
sht.mode = adafruit_sht4x.Mode.NOHEAT_HIGHPRECISION
# SGP40
sgp = adafruit_sgp40.SGP40(i2c)
# Wind Speed - Weather Meter Kit
wind_speed_sensor = Button(WIND_SPEED_SENSOR_PIN, pull_up=False) # Setup wind sensor as a button
# Wind Direction - ADS1115 for Weather Meter Kit
ads = ADS.ADS1115(i2c)
wind_dir = AnalogIn(ads, ADS.P0)
# SPS30
pm_sensor = SPS30()
print(f"Firmware version: {pm_sensor.firmware_version()}")
print(f"Product type: {pm_sensor.product_type()}")
print(f"Serial number: {pm_sensor.serial_number()}")
print(f"Status register: {pm_sensor.read_status_register()}")
print(f"Auto cleaning interval: {pm_sensor.read_auto_cleaning_interval()}s")
pm_sensor.start_measurement()
# Init Grove - Gas Sensor V2 (Multichannel)
gas_sensor = MultichannelGasGMXXX(i2c)
time.sleep(5)



'''
 *  FUNCTIONS  *
'''
####################################
#  Wind Speed - Weather Meter Kit  #
####################################
def count_pulses():
    global pulses
    pulses += 1

wind_speed_sensor.when_pressed = count_pulses

def calculate_wind_speed():
    global pulses
    # Reset pulses before starting measurement
    pulses = 0
    
    start_time = time.time()
    print(f"Start time: {start_time}")
    print(f"Counting pulses for {MEASUREMENT_TIME} seconds...")
    
    time.sleep(MEASUREMENT_TIME)
    
    elapsed_time = time.time() - start_time
    rotations = pulses / 2
    distance_cm = rotations * math.pi * SENSOR_DIAMETER_CM
    wind_speed_cm_s = distance_cm / elapsed_time
    wind_speed_m_s = wind_speed_cm_s * 0.01
    
    print(f"Total pulses: {pulses}")
    print(f"Sensor rotations: {rotations:.2f}")
    print(f"Elapsed time: {elapsed_time:.2f} seconds")
    print(f"Distance traveled: {distance_cm:.2f} cm")
    print(f"Wind speed: {wind_speed_cm_s:.2f} cm/s")
    print(f"Wind speed: {wind_speed_m_s:.2f} m/s")

####################################################
#  Wind Direction - ADS1115 for Weather Meter Kit  #
####################################################
def read_wind_direction():
    incoming = wind_dir.value
    for i in range(16):
        if SENSOR_MIN[i] <= incoming <= SENSOR_MAX[i]:
            return DIR_DEG[i]
    return None



def main():
    while True:

        print("-----------------------------------")

        ################
	    #  READ SHT45  #
        ################
        # Read temperature and humidity
        temperature, humidity = sht.measurements
        print(f"Temperature: {temperature:.1f} C")
        print(f"Humidity: {humidity:.1f} %")
        
        ################
	    #  READ SGP40  #
        ################
        # Read and compensate raw gas measurements
        compensated_raw_gas = sgp.measure_raw(temperature=temperature, relative_humidity=humidity)
        print(f"Raw Gas: {sgp.raw}")
        print(f"Compensated Raw Gas: {compensated_raw_gas}")
        
        
        ###################################
	    #  READ Wind speed and direction  #
        ###################################
        # Read wind direction
        angle = read_wind_direction()
        if angle is not None:
            print(f"Wind direction: {angle} degrees")
        else:
            print("Wind direction not found")
        # Read wind speed
        calculate_wind_speed()

        ################
        #  READ SPS30  #
        ################
        # Read particulate matter data
        pm_data = pm_sensor.get_measurement()
        if pm_data:
            print(json.dumps(pm_data, indent=2))
        else:
            print("Failed to read from SPS30 sensor")


        ###########################################
        #  READ GROVE MULTICHANNEL GAS SENSOR V2  #
        ###########################################
        val = gas_sensor.measure_no2()
        print(f"NO2: {val}  =  {gas_sensor.calc_vol(val)}V")
        val = gas_sensor.measure_c2h5oh()
        print(f"C2H5OH: {val}  =  {gas_sensor.calc_vol(val)}V")
        val = gas_sensor.measure_voc()
        print(f"VOC: {val}  =  {gas_sensor.calc_vol(val)}V")
        val = gas_sensor.measure_co()
        print(f"CO: {val}  =  {gas_sensor.calc_vol(val)}V")
        '''
        -------------------------  PPM CO  -------------------------
        CO range: 0 - 1000 PPM
        Calibrated according calibration curve by Winsen: 0 - 150 PPM
        Calibration by Veselin Hadzhiyski 2021 (vcoder@abv.bg)

        RS/R0       PPM
        1           0
        0.77        1
        0.6         3
        0.53        5
        0.4         10
        0.29        20
        0.21        50
        0.17        100
        0.15        150
        '''
        print("----- PPM CO -----")
        sensorValue = gas_sensor.measure_co()
        sensor_volt = gas_sensor.calc_vol(sensorValue)
        print(f"CO: {sensorValue}  eq  {sensor_volt}V")

        RS_gas = (3.3-sensor_volt)/sensor_volt
        print(f"RS_gas: {RS_gas}")

        global R0_CO
        R0_CO = 31 # This value is obtained from the ambient air and must be determined through sensor calibration, as referenced in the code's initial comments.
        print(f"R0_CO: {R0_CO}")

        ratio =  RS_gas/R0_CO
        print(f"ratio: {ratio}")
        
        lgPPM = (math.log10(ratio) * -3.82) - 0.66  # - 3.82) - 0.66; - default      - 2.82) - 0.12; - best for range up to 150 ppm
        PPM = pow(10,lgPPM)
        print(f"PPM: {PPM}")
        '''
        -------------------------  PPM NO2  -------------------------
        NO2 range: 0 - 10 PPM
        Calibrated according calibration curve by Winsen: 0 - 10 PPM
        Calibration by Veselin Hadzhiyski 2021 (vcoder@abv.bg)

        RS/R0       PPM
        1           0
        1.4         1
        1.8         2
        2.25        3
        2.7         4
        3.1         5
        3.4         6
        3.8         7
        4.2         8
        4.4         9
        4.7         10
        '''
        print("----- PPM NO2 -----")
        sensorValue = gas_sensor.measure_no2()
        sensor_volt = gas_sensor.calc_vol(sensorValue)
        print(f"NO2: {sensorValue}  eq  {sensor_volt}V")

        RS_gas = (3.3-sensor_volt)/sensor_volt
        print(f"RS_gas: {RS_gas}")

        global R0_NO2
        R0_NO2 = 40 # This value is obtained from the ambient air and must be determined through sensor calibration, as referenced in the code's initial comments.
        print(f"R0_NO2: {R0_NO2}")

        ratio =  RS_gas/R0_NO2
        print(f"ratio: {ratio}")

        lgPPM = (math.log10(ratio) * + 1.9) - 0.2  # + 2   -0.3
        PPM = pow(10,lgPPM)
        print(f"PPM: {PPM}")
        
        
        # Wait before the next measurement
        time.sleep(1)

if __name__ == "__main__":
    main()
