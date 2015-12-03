__author__ = 'moff'

"""
Main loop for Muninn drone mission control. Reads bluetooth.out for commands from the mobile app and configures the
drone's missions accordingly. Helper functions contained in muninn_common.py
"""
from dronekit import connect, Command, LocationGlobalRelative, LocationGlobal, VehicleMode
import argparse
import time
import os
import sys
import math
from pymavlink import mavutil

FOLLOW_THRESHOLD = 1.5  # Used to decide if new location should be added to mission for Follow-Me mode

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(
    description='Example which runs basic mission operations. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='127.0.0.1:14550',
                    help="vehicle connection target. Default '127.0.0.1:14550'")
parser.add_argument('--path', default=os.getcwd(),
                    help='path where the message.out and status.out files are expected, default is cwd')
args = parser.parse_args()

# Set up control members and connect to vehicle
message_parameters = {'flight_mode': None, 'camera_mode': None, 'launch_land': None, 'hover_distance': None,
                      'follow_distance': None, 'loop_radius': None, 'settings': None, 'GPS': {'lat': None, 'lon': None}}
message_file_path = os.path.join(args.path, 'message.out')
status_file_path = os.path.join(args.path, 'status.out')

# Ensure that message file exists where expected, abort otherwise
if not os.path.exists(message_file_path):
    print "No message file found at %s, cannot proceed" % message_file_path
    sys.exit("Fatal error - No message file found")

# Initialize file times
last_message_time = 0
last_status_update = 0

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True)

mission_updated = False
muninn_launched = False


# Mission and Helper method definitions
def arm_and_takeoff(target_altitude):
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(float(target_altitude))  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt >= float(target_altitude)*0.95:  # Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)


def land_and_disarm():
    """

    :return:
    """
    print 'Returning to launch'
    vehicle.mode = VehicleMode("RTL")


def get_location_metres(original_location, d_north, d_east):
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Returns a LocationGlobal object containing the latitude/longitude `d_north` and `d_east` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    d_lat = float(d_north)/earth_radius
    d_lon = float(d_east)/(earth_radius*math.cos(math.pi*float(original_location.lat)/180))

    # New position in decimal degrees
    new_lat = float(original_location.lat) + (d_lat * 180/math.pi)
    new_lon = float(original_location.lon) + (d_lon * 180/math.pi)
    return LocationGlobal(new_lat, new_lon, float(original_location.alt))


def get_distance_metres(loc1, loc2):
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    d_lat = loc2.lat - loc1.lat
    d_long = loc2.lon - loc1.lon
    return math.sqrt((d_lat*d_lat) + (d_long*d_long)) * 1.113195e5


def distance_to_current_waypoint():
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    next_waypoint = vehicle.commands.next
    if next_waypoint == 0:
        return None
    mission_item = vehicle.commands[next_waypoint-1]  # commands are zero indexed
    lat = mission_item.x
    lon = mission_item.y
    alt = mission_item.z
    target_waypoint_location = LocationGlobalRelative(lat, lon, alt)
    distance_to_point = get_distance_metres(vehicle.location.global_frame, target_waypoint_location)
    return distance_to_point


def download_mission():
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.


def build_loop_mission(loop_center, loop_radius, altitude):
    """
    Adds a takeoff command and 12 waypoint commands to the current mission.
    The waypoints are positioned to form a dodecagon with vertices at loop_radius around the specified
    LocationGlobal (loop_center).

    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    Modified from Dronekit-python
    """

    cmds = vehicle.commands

    print " Clearing any existing commands"
    cmds.clear()

    print " Building loop waypoints."
    # Add new commands. The meaning/order of the parameters is documented in the Command class.
    # Define the twelve MAV_CMD_NAV_WAYPOINT locations and add the commands and last dummy waypoint
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0, 0, 0, float(altitude)))
    for n in range(0, 12, 1):
        print "Adding waypoint %d to mission" % n
        d_north = math.sin(math.radians(n*30))*float(loop_radius)
        d_east = math.cos(math.radians(n*30))*float(loop_radius)
        point = get_location_metres(loop_center, float(d_north), float(d_east))
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, point.lat, point.lon, float(altitude)))
        if n == 11:
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                             point.lat, point.lon, float(altitude)))

    print " Upload new commands to vehicle"
    cmds.upload()


def build_hover_mission(hover_location, altitude):
    """
    Adds a takeoff command and waypoint to the current mission. Used for Hover mode or to begin a Follow-Me mode flight

    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    Modified from Dronekit-python
    """
    cmds = vehicle.commands

    print " Clearing any existing commands"
    cmds.clear()

    print " Building hover waypoint"
    # Add new commands. The meaning/order of the parameters is documented in the Command class.

    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0, 0, 0, float(altitude)))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0, float(hover_location.lat), float(hover_location.lon), float(altitude)))

    print " Upload new commands to vehicle"
    cmds.upload()


def add_to_follow_mission(next_location, altitude):
    """
    Adds the next_location to the waypoint list for a follow-me mission. Preserves the last waypoint as well to help
    avoid jittery flight. Should be called every couple of seconds at most

    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    """
    cmds = vehicle.commands

    # Get the previous waypoint
    current_waypoint = cmds.vehicle.commands.next
    mission_item = vehicle.commands[current_waypoint-1]  # commands are zero indexed
    # Convert waypoint into location object
    lat = mission_item.x
    lon = mission_item.y
    alt = mission_item.z
    current_waypoint_location = LocationGlobalRelative(lat, lon, alt)

    print " Clearing any existing commands"
    cmds.clear()

    print " Building follow me waypoints"
    # Add the previous waypoint.No Takeoff command because it should aready be in the air
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0, current_waypoint_location.lat, current_waypoint_location.lon, float(altitude)))
    # If the distance between the new location and the previous waypoint exceeds the FOLLOW_THRESHOLD, add to mission
    if get_distance_metres(current_waypoint_location, next_location) >= FOLLOW_THRESHOLD:
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, float(next_location.lat), float(next_location.lon), float(altitude)))

    # Upload the commands to the drone
    print " Upload new commands to vehicle"
    cmds.upload()


def parse_message(msg):
    """
    Parses message from the bluetooth connection to the muninn app, if parameter is not present in the message it is
    set to None
    :param msg: New message string from bluetooth.out
    Updates the message_parameters values
    """
    # print msg
    try:
        new_parameters = dict()
        parsed = msg.strip(';').split(';')  # remove trailing delimiter and split on others
        for param in parsed:
            param = param.split(':')
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

    except IndexError:
        print "Ill formed message, skipping read"

# arm_and_takeoff(10)  # For testing of Simulated drone

# Starting the mission loop
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
        time.sleep(.01)
        # New message has arrived since last check
        print 'New message recv'
        last_message_time = os.stat(message_file_path).st_mtime
        with open(message_file_path, 'r') as message_file:
            message = message_file.read()
            parse_message(message)  # Open file and parse into message_parameters
            mission_updated = True
        print message_parameters

    # Monitor mission
    if mission_updated:
        download_mission()  # Allows for updating the mission on drone

        # Land commands
        if message_parameters['launch_land'] == 'land' and muninn_launched:
            # Check if currently returning to landing location
            print "Land command recv"
            if vehicle.mode.name != "RTL":
                land_and_disarm()
            # If vehicle has landed, toggle flag
            if vehicle.location == vehicle.home_location:
                print "Landed, setting flag"
                muninn_launched = False
        else:
            # Launch commands
            if message_parameters['launch_land'] == 'launch' and not muninn_launched:
                print "Launch command recv"
                arm_and_takeoff(message_parameters['hover_distance'])
                muninn_launched = True

            # Takes care of non launch/land commands and ensures that drone is in AUTO mode
            if message_parameters['flight_mode'] == 'hover':
                print 'Hover mode set'
                if muninn_launched:
                    build_hover_mission(LocationGlobalRelative(message_parameters['GPS']['lat'],
                                                               message_parameters['GPS']['lon'],
                                                               message_parameters['hover_distance']),
                                        message_parameters['hover_distance'])
                else:
                    print 'Must launch before setting flight mode'

            elif message_parameters['flight_mode'] == 'loop':
                print 'Loop mode set'
                if muninn_launched:
                    build_loop_mission(LocationGlobalRelative(message_parameters['GPS']['lat'],
                                                              message_parameters['GPS']['lon'],
                                                              message_parameters['hover_distance']),
                                       message_parameters['loop_radius'],
                                       message_parameters['hover_distance'])
                else:
                    print 'Must launch before setting flight mode'

            elif message_parameters['flight_mode'] == 'follow':
                print 'Loop mode set'
                if muninn_launched:
                    add_to_follow_mission(LocationGlobalRelative(message_parameters['GPS']['lat'],
                                                                 message_parameters['GPS']['lon'],
                                                                 message_parameters['hover_distance']),
                                          message_parameters['hover_distance'])
                else:
                    print 'Must launch before setting flight mode'

            print "Starting new mission"
            # Reset mission set to first (0) waypoint
            vehicle.commands.next = 0
            # Set mode to AUTO to start mission
            vehicle.mode = VehicleMode("AUTO")
            mission_updated = False

    if message_parameters['flight_mode'] == 'loop':
        nextwaypoint = vehicle.commands.next
        if nextwaypoint == 13:  # Dummy waypoint - as soon as we reach last loop, reset to beginning
            vehicle.commands.next = 0


# Close vehicle object before exiting script on shutdown
print "Close vehicle object"
vehicle.close()
