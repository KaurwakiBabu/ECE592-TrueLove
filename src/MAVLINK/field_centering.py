# field_centering.py

"""
Demonstrates drone’s ability to center horizontally on a red object after takeoff.
Uses VisualAligner to continuously adjust yaw based on blob detection.
"""

from image_to_movement import VisualAligner
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import argparse

# Configuration constants
TARGET_ALTITUDE = 1               # Target altitude in meters
ALTITUDE_REACH_THRESHOLD = 0.95  # Percentage of target altitude to consider as 'reached'

# Parse vehicle connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vision alignment.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

connection_string = args.connect
if not connection_string:
    raise SystemExit("Please specify connection string using --connect")

# Connect to vehicle
print(f"Connecting to vehicle on: {connection_string}")
vehicle = connect(connection_string, wait_ready=True)
print("Connected successfully.")

# Wait for manual arming
print("Waiting for safety pilot to arm...")
while not vehicle.armed:
    time.sleep(1)

print("Drone armed. Switching to GUIDED mode.")
vehicle.mode = VehicleMode("GUIDED")

# Execute takeoff
print("Initiating takeoff to target altitude...")
vehicle.simple_takeoff(TARGET_ALTITUDE)
while True:
    if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
        break
    time.sleep(0.5)

# Run horizontal centering loop using vision
controller = VisualAligner(vehicle)
try:
    controller.center_on_target(horizontal=True, stop=True, show=True)
finally:
    vehicle.close()
    print("Field centering complete. Disconnected.")
