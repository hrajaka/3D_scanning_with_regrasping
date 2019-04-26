#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
    print('captured pointcloud')

def pointcloud_listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points',
                     PointCloud2,
                     callback)
    rospy.spin()

if __name__ == '__main__':
    pointcloud_listener()

