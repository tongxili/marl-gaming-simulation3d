#!/usr/bin/python
# -*- coding: UTF-8 -*-

#########
# This script is to set a random velocity for the red team
# Usage: python red_random_vel.py {vehicle_type} {blue_num} {uav_id}
#########

import rospy
from geometry_msgs.msg import Twist, PoseStamped
# from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import sys
import numpy


blue_num = int(sys.argv[2])
uav_id = int(sys.argv[3])
# red_num = int(sys.argv[3])
red_num = 6

class Agent:

    def __init__(self, uav_type, uav_id, uav_num):
        self.uav_type = uav_type
        self.uav_num = uav_num
        self.id = uav_id
        self.f = 0.5

        self.pose = PoseStamped()
        self.vel = Twist()
        
        self.pose_sub = rospy.Subscriber(self.uav_type+'_'+str(self.id + blue_num)+'/mavros/local_position/pose', PoseStamped, self.pose_callback, queue_size=1)
        self.vel_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id + blue_num)+'/cmd_vel_enu', Twist, queue_size=1)


    def pose_callback(self, msg):
        self.pose = msg

    def loop(self):
        rospy.init_node("red_agent" + str(self.id))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            self.vel.linear.x = 0
            self.vel.linear.y = 0
            
            if (self.pose.pose.position.z < 10):
                self.vel.linear.z = 1
            elif (self.pose.pose.position.z > 110):
                self.vel.linear.z = -1
            else:
                self.vel.linear.z = numpy.random.uniform(0.5, 1.5)

            print("red_vel: ", self.vel.linear)
            self.vel_pub.publish(self.vel)
            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    agent = Agent(sys.argv[1], uav_id, red_num)
    print("Red agent " + str(uav_id) + " is running.")
    agent.loop()
    # rospy.spin()