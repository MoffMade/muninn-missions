__author__ = 'moff'

"""
Main loop for Muninn drone mission control. Reads bluetooth.out for commands from the mobile app and configures the
drone's missions accordingly. Helper functions contained in muninn_common.py
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import argparse
import os
import math
from pymavlink import mavutil
import muninn_common as muninn

message_parameters = {'flight_mode': None, 'camera_mode': None, 'launch_land': None, 'hover_distance': None,
                      'follow_distance': None, 'loop_radius': None, 'settings': None, 'GPS': {'lat': None, 'lon': None}}
message_file_path = os.path.join(os.getcwd(), 'message.out')
status_file_path = os.path.join(os.getcwd(), 'status.out')


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
        if param[0] == 'GPS':
            param[1].split(',')
            new_parameters['GPS'] = {'lat': param[1][0], 'lon': param[1][1]}
        else:
            new_parameters[param[0]] = param[1]
    for key in message_parameters.keys():
        if new_parameters[key]:
            message_parameters[key] = new_parameters[key]
        else:
            message_parameters[key] = None


# Set up option parsing to get connection string
parser = argparse.ArgumentParser(
    description='Example which runs basic mission operations. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                    help="vehicle connection target. Default '127.0.0.1:14550'")
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True)
last_message_time = os.stat(message_file_path).st_mtime
last_status_update = 0
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
    if abs(time.time() - last_status_update) >= 8:
        with open(status_file_path, 'r+w') as status_file:
            # ensure that status file is opened
            status_file.write("STATUS:")
        last_status_update = time.time()

    # Check for new message
    if os.stat(message_file_path).st_mtime > last_message_time:
        # New message has arrived since last check
        last_message_time = os.stat(message_file_path).st_mtime
        with open(message_file_path, 'r') as message_file:
            parse_message(message_file.read())

    # Monitor mission


print 'Create a new mission (for current location)'
muninn.build_loop_mission(vehicle, vehicle.location.global_frame, 50, 50)

muninn.arm_and_takeoff(vehicle, 10)

print "Starting mission"
# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# Monitor mission.
# Demonstrates getting and setting the command number
# Uses distance_to_current_waypoint(), a convenience function for finding the
#   distance to the next waypoint.

while True:
    nextwaypoint = vehicle.commands.next
    print 'Distance to waypoint (%s): %s' % (nextwaypoint, muninn.distance_to_current_waypoint(vehicle))

    if nextwaypoint == 12:  # Dummy waypoint - as soon as we reach last loop, reset to beginning
        vehicle.commands.next = 0
    time.sleep(1)

print 'Return to launch'
vehicle.mode = VehicleMode("RTL")
# Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
