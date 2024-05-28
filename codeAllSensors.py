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

'''


'''
  DATE:             2024-May-28 2:24 PM London Time
  AUTHOR:           Andres A. Mercado-Velazquez
  LOCATION:         IoT Lab at Queen Mary University of London
  BOARD:            Raspberry Pi Zero W 2
  BOARD DOC:        https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/
                    https://pinout.xyz
  REPO/CODE:

  ------------------------------------------
  | Connecting Devices/Sensors to the Board |
  ------------------------------------------

  ------------------------------------------------------------------------------------------------
                                            I2C
  ------------------------------------------------------------------------------------------------

  ------------- SHT45 ---------
  * Pinout     =>      https://learn.adafruit.com/assets/99235
  * Tutorial   =>      https://learn.adafruit.com/adafruit-sht40-temperature-humidity-sensor/python-circuitpython#python-computer-wiring-3082732
  * Library    =>      python3-pip (sudo apt-get install python3-pip)
                          |->  adafruit-circuitpython-sht4x (sudo pip3 install adafruit-circuitpython-sht4x)
  *   SHT45             Raspberry Pi Zero W 2
  *   1 VIN ---------------- 3.3V - Pin 1
  *   2 GND ---------------- GND - Pin 6
  *   3 SCL ---------------- SCL - Pin 5
  *   4 SDA ---------------- SDA - Pin 3

  ------------- SGP40 ---------
  * Pinout     =>      https://learn.adafruit.com/assets/98203
  * Tutorial   =>      https://learn.adafruit.com/adafruit-sgp40/python-circuitpython
  * Library    =>      python3-pip (sudo apt-get install python3-pip)
                          |->  adafruit-circuitpython-sgp40 (sudo pip3 install adafruit-circuitpython-sgp40)
  *   SGP40             Raspberry Pi Zero W 2
  *   1 VIN ---------------- 3.3V - Pin 1
  *   2 GND ---------------- GND - Pin 6
  *   3 SCL ---------------- SCL - Pin 5
  *   4 SDA ---------------- SDA - Pin 3
  
  ------------------------------------------------------------------------------------------------
                                  Analogue or digital communication
  ------------------------------------------------------------------------------------------------
  ------------- Wind speed and direction & ADS1115 -------------
  * DOC Weather Meter Kit    =>      https://learn.sparkfun.com/tutorials/weather-meter-hookup-guide
  * Pinout ADS1115           =>      https://learn.adafruit.com/assets/112709
  * Tutorial                 =>      https://projects.raspberrypi.org/en/projects/build-your-own-weather-station/5
  * Library                  =>      python3-pip (sudo apt-get install python3-pip)
                                       |->  adafruit-circuitpython-ads1x15 (sudo pip3 install adafruit-circuitpython-ads1x15)
  *   Raspberry Pi Zero W 2                           ADS1115            Weather Meter Kit
  *       3.3V - Pin 1   -------------------------------- VIN ------------------ RED
  *       GND - Pin 6    -------------------------------- GND ------------------ BLACK
  *       SCL - Pin 5    -------------------------------- SCL 
  *       SDA - Pin 3    -------------------------------- SDA
  *       3.3V - Pin 1   --------- 10k resistor ---------  A0 ------------------ GREEN
  *       GPIO 4 - Pin 7 ------------------------------------------------------- YELLOW

'''


''' 
 * LIBRARIES *
'''
# General Purpose
import time
import math
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
        
        
        
        
        # Wait before the next measurement
        print("")
        time.sleep(1)

if __name__ == "__main__":
    main()
