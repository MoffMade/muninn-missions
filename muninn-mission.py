__author__ = 'moff'

"""
Main loop for Muninn drone mission control. Reads bluetooth.out for commands from the mobile app and configures the
drone's missions accordingly. Helper functions contained in muninn_common.py
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import muninn_common as muninn

message_parameters = {'flight_mode': None, 'camera_mode': None, 'launch_land': None, 'hover_distance': None,
                      'follow_distance': None, 'loop_radius': None, 'settings': None, 'GPS': None}
status_file = 'status.out'

def parse_message(message):
    """
    Parses message from the bluetooth connection to the muninn app, if parameter is not present in the message it is
    set to None
    :param message: New message string from bluetooth.out
    Updates the message_parameters values
    """
    new_parameters = dict()
    message.split(';')
    for param in message:
        param.split(':')
        new_parameters[param[0]] = param[1]
    for key in message_parameters.keys():
        if new_parameters[key]:
            message_parameters[key] = new_parameters[key]
        else:
            message_parameters[key] = None

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(
    description='Example which runs basic mission operations. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                   help="vehicle connection target. Default '127.0.0.1:14550'")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True)

while True:
    """
    Main mission loop:
    - Listen for new message from bluetooth script
        - On new message:  Update mission plan as needed
        - On no message:   Continue
    - Monitor mission
        - For all:  Ensure drone is in good health (battery level, connection, etc)
        - For loop: Monitor for end of waypoint chain, reset to 0 if needed
    - Update status log every 5 seconds
    """
    # Update status if 5 seconds has elapsed

    # Check for new message

    # Monitor mission





print 'Create a new mission (for current location)'
muninn.build_loop_mission(vehicle.location.global_frame, 50, 50)

muninn.arm_and_takeoff(10)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# Monitor mission.
# Demonstrates getting and setting the command number
# Uses distance_to_current_waypoint(), a convenience function for finding the
#   distance to the next waypoint.

while True:
    nextwaypoint=vehicle.commands.next
    print 'Distance to waypoint (%s): %s' % (nextwaypoint, muninn.distance_to_current_waypoint(vehicle))

    if nextwaypoint==12:  # Dummy waypoint - as soon as we reach last loop, reset to beginning
        vehicle.commands.next=0
    time.sleep(1)

print 'Return to launch'
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
