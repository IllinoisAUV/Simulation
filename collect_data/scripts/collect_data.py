#!/usr/bin/env python
import rospy
from rospy_message_converter import json_message_converter
import json
import numpy

from geometry_msgs.msg import Accel
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

imu_file            = open('imu.txt', 'wb')
mag_file            = open('magnetometer.txt', 'wb')
gps_file            = open('gps.txt', 'wb')
pose_gt_file        = open('pose_gt.txt', 'wb')
accel_b_file        = open('accel_b.txt', 'wb')
accel_w_file        = open('accel_w.txt', 'wb')
pressure_file       = open('pressure.txt', 'wb')
g_model_states_file_lin = open('g_model_states_twist.txt', 'wb')
g_model_states_file_pos = open('g_model_states_position.txt', 'wb')

def Imucallback(data):
    global imu_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, imu_file)

def Magnetometercallback(data):
    global mag_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, mag_file)

def GPScallback(data):
    global gps_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, gps_file)

def Pressurecallback(data):
    global pressure_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, pressure_file)

def PoseGTcallback(data):
    global pose_gt_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, pose_gt_file)

def GModelStatescallback(data):
    global g_model_states_file_lin
    global g_model_states_file_pos

    json_str1 = json_message_converter.convert_ros_message_to_json(data.twist[3])
    json.dump(json_str1, g_model_states_file_lin)

    json_str2 = json_message_converter.convert_ros_message_to_json(data.pose[3])
    json.dump(json_str2, g_model_states_file_pos)

def AccelBcallback(data):
    global accel_b_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, accel_b_file)

def AccelWcallback(data):
    global accel_w_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, accel_w_file)

def listener():
    rospy.init_node('simdata_listener', anonymous=True)

    # measured orientation and acceleration, Body frame
    rospy.Subscriber("/rexrov/imu", Imu, Imucallback)

    # measured Magnetic field
    rospy.Subscriber("/rexrov/magnetometer", MagneticField, Magnetometercallback)

    # measured position, Body frame
    rospy.Subscriber("/rexrov/gps", NavSatFix, GPScallback )

    # measured pressure
    rospy.Subscriber("/rexrov/pressure", FluidPressure, Pressurecallback )

    # True position, orientation , velocity  Body frame
    rospy.Subscriber("/rexrov/pose_gt", Odometry , PoseGTcallback )

    # True position, orientation , velocity World frame
    rospy.Subscriber("/gazebo/model_states", ModelStates, GModelStatescallback)

    # True Accelration Body frame
    rospy.Subscriber("/accel_b_gazebo", Accel, AccelBcallback)

    # True Accelration World frame
    rospy.Subscriber("/accel_w_gazebo", Accel, AccelWcallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
