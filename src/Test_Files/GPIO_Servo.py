"""
Initializes GPIO pin and servo motor. 
This is not a class with functions to change servo/rod positioning, but rather a file we ran to test servo/rod movement and determine baselines for drop-off, pick-up, and neutral. 

"""

import RPi.GPIO as GPIO
import time

servoPIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 18 for PWM with 50Hz
p.start(2.5) # Initialization
try:
  while True:
    p.ChangeDutyCycle(7)
    print("this is 7/neutral")
    time.sleep(5)

    p.ChangeDutyCycle(2)
    print("this is hold")
    time.sleep(5) 
    p.ChangeDutyCycle(12.5)
    print("this is drop")
    time.sleep(5)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()
