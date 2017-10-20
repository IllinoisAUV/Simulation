#!/usr/bin/env python
import rospy
from rospy_message_converter import json_message_converter
import json
import numpy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, MagneticField

count1 = 0
count2 = 0

imu_file = open('imu.txt', 'wb')
mag_file = open('magnetometer.txt', 'wb')


def Imucallback(data):
    global count1
    global imu_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    count1 = count1 + 1
    json.dump(json_str, imu_file)


def Magnetometercallback(data):
    global count2
    global mag_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    count2 = count2 + 1
    json.dump(json_str, mag_file)

def listener():
    rospy.init_node('simdata_listener', anonymous=True)

    rospy.Subscriber("/rexrov/imu", Imu, Imucallback)
    rospy.Subscriber("/rexrov/magnetometer", MagneticField, Magnetometercallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
