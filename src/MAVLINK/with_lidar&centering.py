from __future__ import print_function

import math
import time
import sys
import os
gpio_available = False  # Default to False
GPIO = None  # Default to None
import test_lidar 

try:
    # Check if we are on a Raspberry Pi
    if os.uname().machine in ('armv6l', 'armv7l', 'aarch64'):
        import RPi.GPIO as GPIO
        gpio_available = True
    else:
        print("Not running on a Raspberry Pi. GPIO commands will be skipped.")
except ImportError:
    print("RPi.GPIO not found.  Running in SITL or on a non-RPi system. GPIO commands will be skipped.")
    # gpio_available remains False
    # GPIO remains None
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 2
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track o    GPIO.setup(servoPIN, GPIO.OUT)f if joystick to arm has returned to center
rcin_4_center = False

servoPIN = 18
print("it works")

if gpio_available:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPIN, GPIO.OUT)
    p = GPIO.PWM(servoPIN, 50) # GPIO 25 for PWM with 50Hz
    p.start(2.5) # Initialization
else:
    p = None

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distanceToWaypoint(coordinates):
    """
    Returns distance between vehicle and specified coordinates
    """
    distance = get_distance_metres(vehicle.location.global_frame, coordinates)
    return distance

def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, altitude)

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # delay to wait until yaw of copter is at desired yaw angle
    time.sleep(3)
    
    
def send_forward_velocity(vehicle, velocity_x, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, 0, 0,
        0, 0, 0,
        0, 0)
    print(f"Sending forward velocity: {velocity_x} m/s for {duration}s")
    for i in range(int(duration)):
        print(f"Tick {i+1}")
        vehicle.send_mavlink(msg)
        time.sleep(1)

def set_altitude(altitude):
    """Sets the target altitude and waits until it's reached."""
    target_location = LocationGlobalRelative(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, altitude)
    vehicle.simple_goto(target_location)
    print(f"Target altitude set to: {altitude} meters (relative)")
    while abs(vehicle.location.global_relative_frame.alt - altitude) > 0.3:
        print(f"Current altitude: {vehicle.location.global_relative_frame.alt:.2f} meters")
        time.sleep(0.1)
    print(f"Reached altitude: {vehicle.location.global_relative_frame.alt:.2f} meters")
    time.sleep(1)

def stop_motion_msg():
    return vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0)


# Set up option parsing to get connection string and mission plan file
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=False)

print('Succesfully connected to vehicle')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("GUIDED")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)

    # STEP 1: Get initial GPS coordinates
    initial_location_global = vehicle.location.global_frame
    print(f"Initial GPS coordinates (Global): Latitude={initial_location_global.lat}, Longitude={initial_location_global.lon}, Altitude={initial_location_global.alt}")

    if gpio_available:
        p.ChangeDutyCycle(7)
        print("Skewer is set in neutral position.")
        time.sleep(5)
    else:
        print("Simulating rod in neutral position.")
        time.sleep(1)

    # STEP 2: Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)

    # STEP 3: Yaw west
    condition_yaw(270)
    print("Turning west.")
    time.sleep(5)  # Give time to complete yaw

    # STEP 4: Head forward 3 m
    send_forward_velocity(vehicle, 0.3, 10) # To move approximately 3 meters at 0.3 m/s
    print("Current heading:", vehicle.heading)

    # STEP 5: Move down to 1 m
    print("Lowering down to 1 m.")
    set_altitude(1)

    # after lowering down, the drone centers on the target horizontally and vertically 

    controller = VisualAligner(vehicle)
    controller.center_on_target(horizontal=True, stop=True, show=True)
    controller.center_on_target(horizontal=False, stop=True, show=True)

    # STEP 6: Head forward 3 m
    print("Moving foward 3 m")

    # check if gpio is available, if not then the code is run in a virtual environment and cannot be tested 
    if (gpio_available): 
        distance = test_lidar.run_lidar()    # call the run_lidar function and receive the distance of the drone from the ground
        while distance > 0.5: 
            send_forward_velocity(vehicle, 0.3, 1)     # move the drone forward 0.3m at a time and wait until the lidar detects the table (the distance will be 0.5m)
            distance = test_lidar.run_lidar()
            
        if distance <= 0.5:     # if the distance is less than 0.5m, then the drone is above the table on which the letter is located. 
            try: 
                send_forward_velocity(vehicle, 0.3, 6)    # move the drone forward 2m
            finally: 
                p.ChangeDutyCycle(2)    # move the servo/rod to the 'up' position, assuming the drone had picked up the letter
                time.sleep(5)
    else: 
        print("RPi.GPIO not found.  Running in SITL or on a non-RPi system. GPIO commands will be skipped.")

    # STEP 7: Head to Dropoff Location
    print("Package secured! Heading to dropoff...")
    dropoff_location_alt2 = get_location_metres(initial_location_global, 5, -2, 2.0) # 5m North, 2m West, 2m altitude
    vehicle.groundspeed = 1  # Set a slower groundspeed for the approach
    vehicle.simple_goto(dropoff_location_alt2)
    print(f"Heading to dropoff location (2m alt): North {5}m, West {2}m.")
    while distanceToWaypoint(dropoff_location_alt2) > 0.5:
        time.sleep(0.2)
        print(f"Distance to dropoff (2m alt): {distanceToWaypoint(dropoff_location_alt2):.2f} meters")
    print("Arrived at dropoff location (2m alt).")
    time.sleep(2)

    # STEP 8: Lower down to 1 m at dropoff location
    print("Lowering down to 1 m at dropoff.")
    dropoff_location_alt1 = get_location_metres(initial_location_global, 5, -2, 1.0) # Same horizontal, 1m altitude
    vehicle.simple_goto(dropoff_location_alt1)
    while abs(vehicle.location.global_relative_frame.alt - 1.0) > 0.3:
        time.sleep(0.2)
        print(f"Current altitude at dropoff: {vehicle.location.global_relative_frame.alt:.2f} meters")
    print("Reached 1m altitude at dropoff.")
    time.sleep(2)

    # STEP 9: Make dropoff
    print("Releasing package!")
    if gpio_available:
        p.ChangeDutyCycle(12.5) # Revert servo to drop position
        print("Servo released! Package dropped.")
        time.sleep(15)
    else:
        print("Simulating package drop.")
        time.sleep(2)

    # STEP 10: Landing
    print("Returning to Launch (RTL)...")
    vehicle.mode = VehicleMode("RTL")

    if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
        # disarm Rover
        vehicle.armed = False

    # Stay connected to vehicle until landed and disarmed
    while vehicle.armed:
        time.sleep(1)

    print("Done!")

    # Close vehicle object before exiting script
    if gpio_available:
        p.stop()
        GPIO.cleanup()

    vehicle.close()
