#!/usr/bin/env python
import rospy
from rospy_message_converter import json_message_converter
import json
import numpy

from geometry_msgs.msg import Accel
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, FluidPressure
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock


imu_file                = open('imu.txt', 'wb')
mag_t_file              = open('magnetometer_true.txt', 'wb')
mag_m_file              = open('magnetometer_measured.txt', 'wb')
gps_file                = open('gps.txt', 'wb')
pose_gt_file            = open('pose_gt.txt', 'wb')
accel_b_file            = open('accel_b.txt', 'wb')
accel_w_file            = open('accel_w.txt', 'wb')
pressure_file           = open('pressure.txt', 'wb')
clock_file              = open('clock.txt', 'wb')
g_model_states_file_lin = open('g_model_states_twist.txt', 'wb')
g_model_states_file_pos = open('g_model_states_position.txt', 'wb')

def Imucallback(data):
    global imu_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, imu_file)

def MagnetometerTcallback(data):
    global mag_t_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, mag_t_file)

def MagnetometerMcallback(data):
    global mag_m_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, mag_m_file)

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
    global g_model_states_file_clo

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

def Clockcallback(data):
    global clock_file
    json_str = json_message_converter.convert_ros_message_to_json(data)
    json.dump(json_str, clock_file)

def listener():
    rospy.init_node('simdata_listener', anonymous=True)

    # measured orientation and acceleration, Body frame
    rospy.Subscriber("/rexrov/imu", Imu, Imucallback)

    # true Magnetic field
    rospy.Subscriber("/rexrov/magentomter_true", MagneticField, MagnetometerTcallback)

    # measured Magnetic field
    rospy.Subscriber("/rexrov/magentomter_measurement", MagneticField, MagnetometerMcallback)

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

    # Global clock
    rospy.Subscriber("/clock", Clock, Clockcallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print "Collecting Data"
    listener()
