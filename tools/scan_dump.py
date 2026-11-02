#!/usr/bin/env python
"""Run ON THE JETSON (read-only): print a few merged /scan messages as JSON, in the ROBOT frame.

    ssh nvidia@<ip> 'source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash && python -' < tools/scan_dump.py > scan.json

Robot frame: x forward, y left. The RPLidar's 0 degrees faces the robot's REAR, so the scan angle is rotated by 180 degrees.
This only subscribes; it never publishes. Works with Python 2 (Melodic) and 3.
"""
import json
import math

import rospy
from sensor_msgs.msg import LaserScan

N_SCANS = 6
MAX_RANGE = 8.0

rospy.init_node("scan_dump", anonymous=True)
points = []
for _ in range(N_SCANS):
    s = rospy.wait_for_message("/scan", LaserScan, 5)
    for i, r in enumerate(s.ranges):
        if s.range_min <= r <= MAX_RANGE:
            a = s.angle_min + i * s.angle_increment + math.pi
            points.append([round(r * math.cos(a), 3), round(r * math.sin(a), 3)])
print(json.dumps(points))
