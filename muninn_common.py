__author__ = 'Cory Maughmer'

from dronekit import Command, LocationGlobalRelative, LocationGlobal, VehicleMode
import time
import math
from pymavlink import mavutil

FOLLOW_THRESHOLD = 1.5  # Used to decide if new location should be added to mission for Follow-Me mode


def arm_and_takeoff(vehicle, target_altitude):
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
    vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt >= target_altitude*0.95:  # Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)


def land_and_disarm(vehicle):
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
    d_lat = d_north/earth_radius
    d_lon = d_east/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    new_lat = original_location.lat + (d_lat * 180/math.pi)
    new_lon = original_location.lon + (d_lon * 180/math.pi)
    return LocationGlobal(new_lat, new_lon, original_location.alt)


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


def distance_to_current_waypoint(vehicle):
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


def download_mission(vehicle):
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.


def build_loop_mission(vehicle, loop_center, loop_radius, altitude):
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

    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                     0, 0, 0, 0, 0, 0, 0, 0, 10))

    # Define the twelve MAV_CMD_NAV_WAYPOINT locations and add the commands
    for n in range(0, 11, 1):
        d_north = math.sin(math.radians(n*30))*loop_radius
        d_east = math.cos(math.radians(n*30))*loop_radius
        point = get_location_metres(loop_center, d_north, d_east)
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, point.lat, point.lon, altitude))

    print " Upload new commands to vehicle"
    cmds.upload()


def build_hover_mission(vehicle, hover_location, altitude):
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
                     0, 0, 0, 0, 0, 0, 0, 0, 10))
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                     0, 0, 0, 0, 0, 0, hover_location.lat, hover_location.lon, altitude))

    print " Upload new commands to vehicle"
    cmds.upload()


def add_to_follow_mission(vehicle, next_location, altitude):
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
                     0, 0, 0, 0, 0, 0, current_waypoint_location.lat, current_waypoint_location.lon, altitude))
    # If the distance between the new location and the previous waypoint exceeds the FOLLOW_THRESHOLD, add to mission
    if get_distance_metres(current_waypoint_location, next_location) >= FOLLOW_THRESHOLD:
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, 0, 0, 0, next_location.lat, next_location.lon, altitude))

    # Upload the commands to the drone
    print " Upload new commands to vehicle"
    cmds.upload()
