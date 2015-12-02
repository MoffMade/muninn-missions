__author__ = 'Cory Maughmer'
"""
mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.

Full documentation is provided at http://python.dronekit.io/examples/mission_basic.html
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


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


def get_location_metres(original_location, dNorth, dEast):
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

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
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint=vehicle.commands.next
    if nextwaypoint ==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat=missionitem.x
    lon=missionitem.y
    alt=missionitem.z
    targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    -------------CODE TAKEN DIRECTLY FROM DRONEKIT-PYTHON EXAMPLE CODE-------------
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



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

    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    points = []
    for n in range(0,11,1):
        points.append(get_location_metres((loop_center, math.sin(math.radians(n*30))*loop_radius,
                                           math.cos(math.radians(n*30))*loop_radius)))

    for point in points:
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point.lat, point.lon, altitude))


    print " Upload new commands to vehicle"
    cmds.upload()


def arm_and_takeoff(aTargetAltitude):
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
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)


print 'Create a new mission (for current location)'
build_loop_mission(vehicle.location.global_frame, 50, 50)

arm_and_takeoff(10)

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
    print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())

    if nextwaypoint==12:  # Dummy waypoint - as soon as we reach last loop, reset to beginning
        vehicle.commands.next=0
    time.sleep(1)

print 'Return to launch'
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

