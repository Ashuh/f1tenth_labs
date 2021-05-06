#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class reactive_follow_gap:
    def __init__(self):
        # Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(
            lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(
            drive_topic, AckermannDriveStamped, queue_size=1)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = ranges

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None

    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        """
        max_range = 0
        max_range_id = 0

        for i in range(start_i, end_i + 1):
            if ranges[i] > max_range:
                max_range = ranges[i]
                max_range_id = i

        return max_range_id

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        proc_ranges = list(proc_ranges)

        # Find closest point to LiDAR

        min_id = int((math.radians(-90) - data.angle_min) /
                     data.angle_increment)
        max_id = int((math.radians(90) - data.angle_min) /
                     data.angle_increment)

        min_range = float("inf")
        min_range_id = 0
        for i in range(min_id, max_id + 1):
            if proc_ranges[i] < min_range:
                min_range = proc_ranges[i]
                min_range_id = i

        # Eliminate all points inside 'bubble' (set them to zero)

        for i in range(len(proc_ranges)):
            dist = min_range * abs(i - min_range_id) * data.angle_increment
            if dist < 0.5:
                proc_ranges[i] = 0

        # Find max length gap

        left_gap_start_id = 0
        right_gap_end_id = 0

        left_gap_size = 0
        right_gap_size = 0

        for i in range(min_id, max_id + 1):
            if proc_ranges[i] > 0:
                right_gap_size += 1
            else:
                right_gap_end_id = i
                break

        for i in range(max_id, min_id - 1, -1):
            if proc_ranges[i] > 0:
                left_gap_size += 1
            else:
                left_gap_start_id = i
                break

        best_gap_start_id = 0
        best_gap_end_id = 0
        if right_gap_size > left_gap_size:
            best_gap_start_id = min_id
            best_gap_end_id = right_gap_end_id
        else:
            best_gap_start_id = left_gap_start_id
            best_gap_end_id = max_id

        # Find the best point in the gap

        best_point_angle = data.angle_min + \
            self.find_best_point(
                best_gap_start_id, best_gap_end_id, proc_ranges) * data.angle_increment

        rospy.loginfo_throttle(
            0.1, "[Reactive Gap Follow] Steering Angle: %.2f", math.degrees(best_point_angle))

        # Publish Drive message

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = best_point_angle

        if abs(best_point_angle) > math.radians(20):
            drive_msg.drive.speed = 1.0
        elif abs(best_point_angle) > math.radians(10):
            drive_msg.drive.speed = 2.0
        else:
            drive_msg.drive.speed = 5.0

        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
