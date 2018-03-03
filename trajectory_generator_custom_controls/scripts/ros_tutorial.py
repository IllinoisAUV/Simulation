import rospy
import roslib
import sys

import geometry_msgs.msg as geometry_msgs
from rospy.numpy_msg import numpy_msg



def main(args):
    rospy.init_node("barrel_role")
    des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)
    i = 0
    while not rospy.is_shutdown():
        msg = geometry_msgs.Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0
        msg.linear.z = 0

        # pitch
        msg.angular.y = 0
        # yaw
        msg.angular.z = 0


        if( i < 1000000):
            # roll
            msg.angular.x = 0.5
            i += 1
        else:
            # roll
            msg.angular.x = 0.0

        print(msg)
        print(i)

        des_vel_pub.publish(msg)

if __name__ == '__main__':
    main(sys.argv)
