#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
import logging
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# PID CONTROL PARAMS
kp = 1.2
kd = 0.1
ki = 0.0
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9  # meters
DESIRED_DISTANCE_LEFT = 0.9
VELOCITY = 2.00  # meters per second
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """

    def __init__(self):
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(
            lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=1)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.

        i = int((math.radians(angle) - data.angle_min) / data.angle_increment)
        range = 10 if math.isnan(data.ranges[i]) else data.ranges[i]

        return range

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd

        prev_error = error

        if error < 0.01:
            integral = 0
        else:
            integral += error

        angle = (kp * error) + (ki * integral) + \
            (kd * (error - prev_error) / 0.1)
        rospy.loginfo("[Wall Follow] Steering angle: %.2f degrees", math.degrees(angle))

        deg_angle = abs(math.degrees(angle))
        if 1 < deg_angle <= 10:
            velocity = 1.5
        elif 10 < deg_angle <= 20:
            velocity = 1.0
        elif deg_angle > 20:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        # Follow left wall as per the algorithm
        deg_a = 35
        deg_b = 90
        dist_a = self.getRange(data, deg_a)
        dist_b = self.getRange(data, deg_b)

        theta = abs(math.radians(deg_a - deg_b))
        alpha = math.atan2(dist_a * math.cos(theta) -
                           dist_b, dist_a * math.sin(theta))

        dist_wall = dist_b * math.cos(alpha)
        dist_wall_projected = dist_wall + 1.5 * CAR_LENGTH * math.sin(alpha)
        error = dist_wall_projected - leftDist

        return error

    def lidar_callback(self, data):
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        # send error to pid_control
        self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
