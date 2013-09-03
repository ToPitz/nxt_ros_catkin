#!/usr/bin/env python

import roslib; roslib.load_manifest('nxt_assisted_teleop')  
import nxt.locator
import rospy
import math
import thread
from sensor_msgs.msg import PointCloud
from nxt_msgs.msg import Range
from geometry_msgs.msg import Point32
from math import sin, cos

class Converter:
    def __init__(self):
        self.sub = rospy.Subscriber('range_topic', Range, self.sub_cb)
        self.pub = rospy.Publisher('cloud_topic', PointCloud)

    def sub_cb(self, msg):
        pnt = PointCloud()
        pnt.header = msg.header
        angle_step = 1.0/(msg.range*100.0)
        angle = -msg.spread_angle
        if msg.range < msg.range_max and msg.range > msg.range_min:
            while angle < msg.spread_angle:
                pnt.points.append(Point32(msg.range*cos(angle), msg.range*sin(angle), 0))
                angle += angle_step
        self.pub.publish(pnt)

def main():
    rospy.init_node('range_to_pointcloud')
    converter = Converter()
    rospy.spin()



if __name__ == '__main__':
    main()
