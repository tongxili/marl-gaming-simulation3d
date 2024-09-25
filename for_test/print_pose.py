#!/usr/bin/python
# -*- coding: UTF-8 -*-

######
# This script can subscribe the positions of a group and print them with assigned freq
# Usage: python print_pose.py {vehicle_type} {vehicle_num}
# Other params: 
#   /mavros/local_position/pose - PoseStampeed: ros topic subscribed -> the pose_callback need according changes
#       /mavros/global_position/local - Odometry
#   rate = rospy.Rate(1) frequency of printing message
# By tongxi
######

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
import sys

vehicle_type = sys.argv[1]
vehicle_num = int(sys.argv[2])

class PosePrinter:
    def __init__(self, uav_id):
        self.last_pose = []
        self.id = uav_id
        # for i in range(uav_num):
        # rospy.Subscriber(vehicle_type+'_'+str(uav_id)+'/mavros/global_position/local', Odometry, self.pose_callback)
        rospy.Subscriber(vehicle_type+'_'+str(uav_id)+'/mavros/vision_pose/pose', PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        """
        Callback function for subscriber
        Get new pose
        """
        # self.last_pose = msg.pose.pose.position
        self.last_pose = msg.pose.position
        # print(self.last_pose) # global_local, does not print???
        # print("Position: x={:.2f}, y={:.2f}, z={:.2f}".format(position.x, position.y, position.z))

    def run(self):
            if self.last_pose:
                print("Position for uav {:d}: x={:.2f}, y={:.2f}, z={:.2f}".format(self.id, self.last_pose.x, self.last_pose.y, self.last_pose.z))

if __name__ == '__main__':
    rospy.init_node(vehicle_type+'_pos_listener')
    rate = rospy.Rate(2)

    pose_printer = []
    for i in range(vehicle_num):
        print("Starting uav %d"%i)
        pose_printer.append(PosePrinter(i))
    
    while not rospy.is_shutdown():
        for i in range(vehicle_num):
            pose_printer[i].run()
            if (i==vehicle_num-1):
                print()
                print("Getting Position:")
        try:
            rate.sleep()
        except:
            continue
    