__author__ = 'cory'

from dronekit import Command, LocationGlobalRelative, LocationGlobal, VehicleMode
import time
import math
from pymavlink import mavutil


class Attitude:
    def __init__(self, p, r, y):
        self.pitch = p
        self.roll = r
        self.yaw = y


class MuninnGimbal:
    def __init__(self, a, lock = True):
        """
        Initialize MuninnGimbal object
        :param a: starting attitude (ypr angles) for the gimbal
        :param lock: True to lock yaw axis or False to allow movement
        :return:
        """
        self.position = a
        self.target = None
        self.location = None
        self.lock_yaw = lock

    def pan(self, degrees):
        """

        :param degrees:
        :return:
        """

        return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                       0, 0, self.position.pitch, self.position.roll, self.position.yaw, 0, 0, 0, 0)

    def point_to_target(self):
        """
        Calculate yaw/pitch angles to keep gimbal pointed at the target location. Roll should stay stabilized
        Calculation taken from http://math.stackexchange.com/questions/470112/calculate-camera-pitch-yaw-to-face-point
        :return: A dronekit Command to set the gimbal position
        """
        dx = self.location.lat - self.target.lat
        dy = self.location.lon - self.target.lon
        dz = self.location.alt - self.target.alt
        self.position.pitch = -math.atan2(dy, math.sqrt(dz*dz + dx*dx))

        if not self.lock_yaw:
            self.position.yaw = math.atan2(dz, dx) - math.radians(90)

        print "Moving gimbal to yaw ", self.position.yaw, " pitch ", self.position.pitch, ' roll ', self.position.roll
        return Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                       0, 0, self.position.pitch, self.position.roll, self.position.yaw, 0, 0, 0, 0)
