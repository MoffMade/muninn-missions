__author__ = 'Cory Maughmer'
"""
Mission code for Muninn drone - Hover Mode
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Hover Mode setting for Muninn Drone')
parser.add_argument('--connect', default='/dev/ttyACM0',
                   help="vehicle connection target. Default '/dev/ttyACM0'")
parser.add_argument('--baud', default=57600,
                   help="Baud rate for connection. Default 57600")
parser.add_argument('--altitude', default='3',
                    help='Target altitude for hover in meters. Default is 3 meters')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
print 'Baud Rate: %d' % args.baud
vehicle = connect(args.connect, args.baud, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)

def move_to_location(targetLocation):
    """

    :param targetLocation:
    :return:
    """
    print "Moving to location"


arm_and_takeoff(args.altitude)