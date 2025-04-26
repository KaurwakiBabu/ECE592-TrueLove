"""
Initializes TFLuna Lidar and reads values

"""

import serial,time
import numpy as np

ser = serial.Serial("/dev/ttyS0", 115200,timeout=0) # mini UART serial device

# read_tfluna_data() function reads bytes received in ttys0 (rPI3 UART) and stores them to their corresponding variable
def read_tfluna_data():
    while True:
        counter = ser.in_waiting # count the number of bytes of the serial port
        if counter > 8:
            bytes_serial = ser.read(9) # read 9 bytes
            ser.reset_input_buffer() # reset buffer

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # check first two bytes
                distance = bytes_serial[2] + bytes_serial[3]*256 # distance in next two bytes
                strength = bytes_serial[4] + bytes_serial[5]*256 # signal strength in next two bytes
                temperature = bytes_serial[6] + bytes_serial[7]*256 # temp in next two bytes
                temperature = (temperature/8.0) - 256.0 # temp scaling and offset
                return distance/100.0,strength,temperature

# Only the distance variable was needed in the main code so lidar_test() only returns distance, however, can easily be modified to return other variables if needed
def run_lidar():
    if ser.isOpen() == False:
    ser.open() # open serial port if not open
    while True:
        distance,strength,temperature = read_tfluna_data() # read value
        format(distance,strength,temperature)) # print sample data
        return distance
    ser.close()
