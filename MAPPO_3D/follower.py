#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String
import sys
import numpy

### OPTION: 增加follower死亡机制

class Follower:

    def __init__(self, uav_type, uav_id, uav_num, formation_i):
        self.hover = "HOVER"
        self.uav_type = uav_type
        self.uav_num = uav_num
        self.id = uav_id
        self.f = 30 

        if uav_num == 6:
            from formation_dict import formation_dict_6 as formation_dict
        elif uav_num == 9:
            from formation_dict import formation_dict_9 as formation_dict
        elif uav_num == 18:
            from formation_dict import formation_dict_18 as formation_dict
        else:
            print("Only 6, 9 and 18 UAVs are supported.")

        if uav_num == 18:
            formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
        elif uav_num == 9:
            formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
        elif uav_num == 6:
            formation_configs = ['waiting', 'T', 'diamond', 'triangle']
        elif uav_num == 1:
            formation_configs = ['stop controlling']
            print("Stop controlling!")

        self.formation_config = formation_configs[formation_i]
        self.formation_pattern = numpy.array(formation_dict[self.formation_config]).reshape(3, self.uav_num-1)

        self.pose = PoseStamped()
        self.cmd_vel_enu = Twist()
        self.leader_vel = Twist()
        self.avoid_vel = Vector3()
        self.Kp = 1.0 
        self.Kp_avoid = 2.0
        self.vel_max = 2.0
        self.leader_pose = PoseStamped()

        self.pose_sub = rospy.Subscriber(self.uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+self.uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
        # self.formation_pattern_sub = rospy.Subscriber("/xtdrone/formation_pattern", Float32MultiArray, self.formation_pattern_callback, queue_size=1)
        # self.leader_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel_flu", Twist, self.cmd_leader_vel_callback, queue_size=1)
        self.leader_vel_sub = rospy.Subscriber("/xtdrone/iris_0/cmd_vel_flu", Twist, self.cmd_leader_vel_callback, queue_size=1)

        self.vel_enu_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=1)
        self.info_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/info', String, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',String,queue_size=1)
        self.leader_pose_sub = rospy.Subscriber(self.uav_type+"_0/mavros/local_position/pose", PoseStamped, self.leader_pose_callback, queue_size=1)

    # def formation_pattern_callback(self, msg):
    #     print("Blue formation pattern: ", msg.data)
    #     self.formation_pattern = numpy.array(msg.data).reshape(3, self.uav_num-1) 

    def pose_callback(self, msg):
        self.pose = msg

    def leader_pose_callback(self, msg):
        self.leader_pose = msg    

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg
    
    def cmd_leader_vel_callback(self, msg):
        self.leader_vel = msg

    def vel_assign(self, vel):
        # print("Blue leader pos: ", self.leader_pose.pose.position)
        # print("Blue pos: ", self.pose.pose.position)
        self.cmd_vel_enu.linear.x = self.Kp * ((self.leader_pose.pose.position.x + self.formation_pattern[0, self.id - 1]) - self.pose.pose.position.x)
        self.cmd_vel_enu.linear.y = self.Kp * ((self.leader_pose.pose.position.y + self.formation_pattern[1, self.id - 1]) - self.pose.pose.position.y) 
        self.cmd_vel_enu.linear.z = self.Kp * ((self.leader_pose.pose.position.z + self.formation_pattern[2, self.id - 1]) - self.pose.pose.position.z) 
        self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.Kp_avoid * self.avoid_vel.x
        self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.Kp_avoid * self.avoid_vel.y
        self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.Kp_avoid * self.avoid_vel.z
        cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5 
        if cmd_vel_magnitude > 3**0.5 * self.vel_max:
            self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
            self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
            self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max
        
        self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.leader_vel.linear.x
        self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.leader_vel.linear.y
        self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.leader_vel.linear.z
        # print("Blue Leader vel: ", self.leader_vel.linear)
        # print("Blue %d vel: "%self.id, self.cmd_vel_enu.linear)
        self.vel_enu_pub.publish(self.cmd_vel_enu)

    def loop(self):
        rospy.init_node('follower'+str(self.id-1))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if (not self.formation_pattern is None):
                self.cmd_vel_enu.linear.x = self.Kp * ((self.leader_pose.pose.position.x + self.formation_pattern[0, self.id - 1]) - self.pose.pose.position.x)
                self.cmd_vel_enu.linear.y = self.Kp * ((self.leader_pose.pose.position.y + self.formation_pattern[1, self.id - 1]) - self.pose.pose.position.y) 
                self.cmd_vel_enu.linear.z = self.Kp * ((self.leader_pose.pose.position.z + self.formation_pattern[2, self.id - 1]) - self.pose.pose.position.z) 
                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.Kp_avoid * self.avoid_vel.x
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.Kp_avoid * self.avoid_vel.y
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.Kp_avoid * self.avoid_vel.z
                cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5 
                if cmd_vel_magnitude > 3**0.5 * self.vel_max:
                    self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max
                
                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.leader_vel.linear.x
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.leader_vel.linear.y
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.leader_vel.linear.z
                self.vel_enu_pub.publish(self.cmd_vel_enu)

            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    follower = Follower(sys.argv[1],int(sys.argv[2]), int(sys.argv[3]))
    follower.loop()