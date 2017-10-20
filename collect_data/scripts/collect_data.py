#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3

from math import radians
from sensor_msgs.msg import Imu
import time
import numpy

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('imu_listener', anonymous=True)

    rospy.Subscriber("/rexrov/imu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
