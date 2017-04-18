#!/usr/bin/env python
#
#
import rospy
import tf
import math
from tf import transformations

from nav_msgs.msg import Odometry

class vel_calib:
    def  __init__(self):
        self.node = rospy.init_node("velodyne_calib")
        self.min_vel = rospy.get_param("~min_vel", 0.2)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.on_odom )
        self.velodyne_odom_sub = rospy.Subscriber("/velodom", Odometry, self.on_velodom )
        self.vx = 0
        self.vy = 0

    def on_odom(self, msg):
        self.v = msg.twist.twist.linear.x


    def on_velodom(self, msg):
        if (math.fabs(self.v) < self.min_vel):
            return
        sign = 1 if (self.v > 0) else -1
        self.vx += msg.twist.twist.linear.x * sign
        self.vy += msg.twist.twist.linear.y * sign

        rospy.loginfo("angle is %s"%math.atan2(self.vy, self.vx))

def main():

    rospy.loginfo("velodyne calib started")
    calib = vel_calib()
    rospy.spin()

if __name__ == "__main__":
    main()
