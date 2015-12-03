__author__ = 'moff'

"""
Main loop for Muninn drone mission control. Reads bluetooth.out for commands from the mobile app and configures the
drone's missions accordingly. Helper functions contained in muninn_common.py
"""
from dronekit import connect, VehicleMode,LocationGlobalRelative
import muninn_common as muninn
import argparse
import time
import os
import sys

message_parameters = {'flight_mode': None, 'camera_mode': None, 'launch_land': None, 'hover_distance': None,
                      'follow_distance': None, 'loop_radius': None, 'settings': None, 'GPS': {'lat': None, 'lon': None}}
message_file_path = os.path.join(os.getcwd(), 'message.out')
status_file_path = os.path.join(os.getcwd(), 'status.out')

# Ensure that message file exists where expected, abort otherwise
if not os.path.exists(message_file_path):
    print "No message file found at %s, cannot proceed" % message_file_path
    sys.exit("Fatal error - No message file found")

# Initialize file times
last_message_time = 0
last_status_update = 0


def parse_message(msg):
    """
    Parses message from the bluetooth connection to the muninn app, if parameter is not present in the message it is
    set to None
    :param msg: New message string from bluetooth.out
    Updates the message_parameters values
    """
    print msg
    new_parameters = dict()
    parsed = msg.strip(';').split(';')  # remove trailing delimiter and split on others
    print parsed
    for param in parsed:
        param = param.split(':')
        print param
        if param[0] == 'GPS':
            param[1] = param[1].split(',')
            new_parameters['GPS'] = {'lat': param[1][0], 'lon': param[1][1]}
        else:
            new_parameters[param[0]] = param[1]
    for key in message_parameters.keys():
        try:
            message_parameters[key] = new_parameters[key]
        except KeyError:
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

mission_updated = False
muninn_launched = False

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
    # Update status if 8 seconds has elapsed
    if abs(time.time() - last_status_update) >= 8:
        with open(status_file_path, 'w') as status_file:
            # write status data to file
            status_file.write("STATUS:")
        last_status_update = time.time()  # Update last status time

    # Check for new message
    if os.stat(message_file_path).st_mtime > last_message_time:
        # New message has arrived since last check
        last_message_time = os.stat(message_file_path).st_mtime
        with open(message_file_path, 'r') as message_file:
            message = message_file.read()
            parse_message(message)  # Open file and parse into message_parameters
            mission_updated = True
        print message_parameters

    # Monitor mission
    if mission_updated:
        if message_parameters['launch_land'] == 'launch':
            muninn.arm_and_takeoff(vehicle, message_parameters['hover_distance'])
            muninn_launched = True

        elif message_parameters['launch_land'] == 'land':
            muninn.land_and_disarm(vehicle)
            muninn_launched = False

        elif message_parameters['flight_mode'] == 'hover':
            with muninn_launched:
                muninn.build_hover_mission(vehicle,
                                           LocationGlobalRelative(message_parameters['GPS']['lat'],
                                                                  message_parameters['GPS']['lon'],
                                                                  message_parameters['hover_distance']),
                                           message_parameters['hover_distance'])

        elif message_parameters['flight_mode'] == 'loop':
            with muninn_launched:
                muninn.build_loop_mission(vehicle,
                                          LocationGlobalRelative(message_parameters['GPS']['lat'],
                                                                 message_parameters['GPS']['lon'],
                                                                 message_parameters['hover_distance']),
                                          message_parameters['loop_radius'],
                                          message_parameters['hover_distance'])

        elif message_parameters['flight_mode'] == 'follow':
            with muninn_launched:
                muninn.add_to_follow_mission(vehicle,
                                             LocationGlobalRelative(message_parameters['GPS']['lat'],
                                                                    message_parameters['GPS']['lon'],
                                                                    message_parameters['hover_distance']),
                                             message_parameters['hover_distance'])

    if message_parameters['flight_mode'] == 'loop':
        nextwaypoint = vehicle.commands.next
        print 'Distance to waypoint (%s): %s' % (nextwaypoint, muninn.distance_to_current_waypoint(vehicle))
        if nextwaypoint == 12:  # Dummy waypoint - as soon as we reach last loop, reset to beginning
            vehicle.commands.next = 0


"""
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

# Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
"""
