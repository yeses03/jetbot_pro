#!/usr/bin/env python

""" odom_ekf.py - Version 1.1 2013-12-20
    Republish the /robot_pose_ekf/odom_combined topic which is of type 
    geometry_msgs/PoseWithCovarianceStamped as an equivalent message of
    type nav_msgs/Odometry so we can view it in RViz.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import copy

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Raw chassis twist older than this (seconds) is treated as stale and zeroed.
RAW_TWIST_TIMEOUT = 0.5

# robot_pose_ekf adds the driver's 1e6 variance for z/roll/pitch (which the
# robot cannot measure) on every odom message, so these entries of
# /odom_combined grow without bound (observed 7.8e10 after 78k messages).
# Cap them at the driver's "unknown" value; x, y and yaw are untouched.
UNOBSERVED_COV_INDEXES = (14, 21, 28)
MAX_UNOBSERVED_COV = 1e6


def build_odometry(combined, raw, now, odom_cls=Odometry):
    """Build the /odom message.

    The pose comes from the robot_pose_ekf output (`combined`). robot_pose_ekf
    publishes no velocity, so the twist is taken from the latest raw chassis
    odometry (`raw`, may be None) and left at its default (zero) when stale.
    """
    odom = odom_cls()
    odom.header = copy.copy(combined.header)
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_footprint'
    odom.pose = copy.deepcopy(combined.pose)
    cov = list(odom.pose.covariance)
    for i in UNOBSERVED_COV_INDEXES:
        cov[i] = min(cov[i], MAX_UNOBSERVED_COV)
    odom.pose.covariance = cov
    if raw is not None and (now - raw.header.stamp).to_sec() < RAW_TWIST_TIMEOUT:
        odom.twist = raw.twist
    return odom


class OdomEKF():
    def __init__(self):
        # Give the node a name
        rospy.init_node('odom_ekf', anonymous=False)

        self.raw = None

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = rospy.Publisher('odom', Odometry, queue_size=5)

        # Wait for the /odom_combined topic to become available
        rospy.wait_for_message('odom_combined', PoseWithCovarianceStamped)

        # Raw chassis odometry supplies the velocity
        rospy.Subscriber('odom_raw', Odometry, self.raw_cb)

        # Subscribe to the /odom_combined topic
        rospy.Subscriber('odom_combined', PoseWithCovarianceStamped, self.pub_ekf_odom)

        rospy.loginfo("Publishing combined odometry on /odom")

    def raw_cb(self, msg):
        self.raw = msg

    def pub_ekf_odom(self, msg):
        self.ekf_pub.publish(build_odometry(msg, self.raw, rospy.Time.now()))


if __name__ == '__main__':
    try:
        OdomEKF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
