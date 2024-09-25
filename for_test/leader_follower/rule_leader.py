#!/usr/bin/python
# -*- coding: UTF-8 -*-

#####
# This code is about the movement control of leader agents when flying a fixed route, or approaching a goal 
#####

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import sys
import numpy

multirotor_num = int(sys.argv[2])

if sys.argv[2] == '6':
    from formation_dict import formation_dict_6 as formation_dict
elif sys.argv[2] == '9':
    from formation_dict import formation_dict_9 as formation_dict
elif sys.argv[2] == '18':
    from formation_dict import formation_dict_18 as formation_dict
else:
    print("Only 6, 9 and 18 UAVs are supported.")

if multirotor_num == 18:
    formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
elif multirotor_num == 9:
    formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
elif multirotor_num == 6:
    formation_configs = ['waiting', 'T', 'diamond', 'triangle']
elif multirotor_num == 1:
    formation_configs = ['stop controlling']
    print("Stop controlling!")

class Leader:

    def __init__(self, uav_type, leader_id, uav_num, adv_num, formation_i):
        self.id = leader_id
        self.pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.cmd_vel_enu = Twist()
        self.uav_num = uav_num
        self.adv_num = adv_num
        self.avoid_vel = Vector3(0,0,0)
        self.formation_config = formation_configs[formation_i]
        self.origin_formation = formation_dict["origin"]
        self.new_formation = formation_dict[self.formation_config]
        self.adj_matrix = None
        self.communication_topology = None
        self.changed_id = numpy.arange(0, self.uav_num - 1)
        self.target_height_recorded = False
        self.cmd = String()
        self.f = 200
        self.K_attr = 0.5 
        self.K_repul = 0.5 
        self.vel_scale_target = 5.0
        self.max_vel = 1.5
        self.pose_sub = rospy.Subscriber(uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped , self.pose_callback, queue_size=1)
        self.target_pos_sub = rospy.Subscriber("target_sephere_0/mavros/vision_pose/pose", PoseStamped, self.target_pose_callback, queue_size=1) # 获取目标位置
        self.cmd_vel_sub = rospy.Subscriber("/xtdrone/leader/cmd_vel_flu", Twist, self.cmd_vel_callback, queue_size=1)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
        # self.leader_cmd_sub = rospy.Subscriber("/xtdrone/leader/cmd",String, self.cmd_callback, queue_size=1)

        # red pos subscriber
        self.red_pos_sub = []
        self.red_pos = [PoseStamped() for i in range(self.adv_num)]
        for i in range(1, self.adv_num):
            self.red_pos_sub.append(rospy.Subscriber("red_"+str(i)+"/mavros/vision_pose/pose", PoseStamped, self.red_pos_callback, queue_size=1))

        self.pose_pub = rospy.Publisher("/xtdrone/leader/pose", PoseStamped , queue_size=1)
        self.formation_pattern_pub = rospy.Publisher('/xtdrone/formation_pattern', Float32MultiArray, queue_size=1)
        self.communication_topology_pub = rospy.Publisher('/xtdrone/communication_topology', Int32MultiArray, queue_size=1)
        self.vel_enu_pub =  rospy.Publisher('/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd_vel_flu', Twist, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+uav_type+'_'+str(self.id)+'/cmd', String, queue_size=1)

    def pose_callback(self, msg):
        self.pose = msg
    
    def red_pos_callback(self, msg):
        self.red_pos[int(msg.header.frame_id)] = msg

    def target_pose_callback(self, msg):
        self.target_pose = msg
    
    def cmd_vel_callback(self, msg):
        self.cmd_vel_enu = msg

    def cmd_callback(self, msg):
        if (msg.data in formation_dict.keys() and not msg.data == self.formation_config):
            self.formation_config = msg.data
            print("Formation pattern: ", self.formation_config)
            # Get a new formation directly from formation_dict
            self.new_formation = formation_dict[self.formation_config]
            print(self.new_formation)
            self.communication_topology = self.get_communication_topology(self.new_formation)
            print(self.communication_topology)
            self.origin_formation = self.new_formation
        else:
            self.cmd = msg.data

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def calc_dist(self):
        '''
        New function: calculate forces posed on blue UAVs
        including: attraction from target, repulsion from red UAVs
        '''
        leader_position = self.pose.pose.position
        
        # attraction
        target_position = self.target_pose.pose.position
        delta_pos = [target_position.x - leader_position.x, target_position.y - leader_position.y, target_position.z - leader_position.z]
        distance = ((target_position.x - leader_position.x) ** 2 +
                    (target_position.y - leader_position.y) ** 2 +
                    (target_position.z - leader_position.z) ** 2) ** 0.5
        if distance < 1.5:
            d_p = [0] * len(delta_pos)
        else:
            d_p = [self.K_attr * x / distance for x in delta_pos]
        # print("distance: ", d_p)
        # print("velocity: ", self.cmd_vel_enu.linear.x, self.cmd_vel_enu.linear.y, self.cmd_vel_enu.linear.z)
        
        # repulsion
        for i in range(1, self.adv_num):
            red_position = self.red_pos[i].pose.position
            delta_pos = [leader_position.x - red_position.x, leader_position.y - red_position.y, leader_position.z - red_position.z]
            dist_red = ((leader_position.x - red_position.x) ** 2 +
                        (leader_position.y - red_position.y) ** 2 +
                        (leader_position.z - red_position.z) ** 2) ** 0.5
            if dist_red < 0.5:
                pass # 死亡机制
            else: 
                d_p = [d_p[j] + self.K_repul * delta_pos[j] / distance for j in range(len(delta_pos))]
    
        return d_p

    def get_communication_topology(self, rel_posi):

        c_num = int((self.uav_num) / 2)
        min_num_index_list = [0] * c_num

        comm = [[] for i in range(self.uav_num)]
        communication = numpy.ones((self.uav_num, self.uav_num)) * 0
        nodes_next = []
        node_flag = [self.uav_num - 1]
        node_mid_flag = []

        rel_d = [0] * (self.uav_num - 1)

        for i in range(0, self.uav_num - 1):
            rel_d[i] = pow(rel_posi[0][i], 2) + pow(rel_posi[1][i], 2) + pow(rel_posi[2][i], 2)

        c = numpy.copy(rel_d)
        c.sort()
        count = 0

        for j in range(0, c_num):
            for i in range(0, self.uav_num - 1):
                if rel_d[i] == c[j]:
                    if not i in node_mid_flag:
                        min_num_index_list[count] = i
                        node_mid_flag.append(i)
                        count = count + 1
                        if count == c_num:
                            break
            if count == c_num:
                break

        for j in range(0, c_num):
            nodes_next.append(min_num_index_list[j])

            comm[self.uav_num - 1].append(min_num_index_list[j])

        size_ = len(node_flag)

        while (nodes_next != []) and (size_ < (self.uav_num - 1)):

            next_node = nodes_next[0]
            nodes_next = nodes_next[1:]
            min_num_index_list = [0] * c_num
            node_mid_flag = []
            rel_d = [0] * (self.uav_num - 1)
            for i in range(0, self.uav_num - 1):

                if i == next_node or i in node_flag:

                    rel_d[i] = 2000
                else:

                    rel_d[i] = pow((rel_posi[0][i] - rel_posi[0][next_node]), 2) + pow(
                        (rel_posi[1][i] - rel_posi[1][next_node]), 2) + pow((rel_posi[2][i] - rel_posi[2][next_node]),
                                                                            2)
            c = numpy.copy(rel_d)
            c.sort()
            count = 0

            for j in range(0, c_num):
                for i in range(0, self.uav_num - 1):
                    if rel_d[i] == c[j]:
                        if not i in node_mid_flag:
                            min_num_index_list[count] = i
                            node_mid_flag.append(i)
                            count = count + 1
                            if count == c_num:
                                break
                if count == c_num:
                    break
            node_flag.append(next_node)

            size_ = len(node_flag)

            for j in range(0, c_num):

                if min_num_index_list[j] in node_flag:

                    nodes_next = nodes_next

                else:
                    if min_num_index_list[j] in nodes_next:
                        nodes_next = nodes_next
                    else:
                        nodes_next.append(min_num_index_list[j])

                    comm[next_node].append(min_num_index_list[j])

        for i in range(0, self.uav_num):
            for j in range(0, self.uav_num - 1):
                if i == 0:
                    if j in comm[self.uav_num - 1]:
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
                else:
                    if j in comm[i - 1] and i < (j+1):
                        communication[j + 1][i] = 1
                    else:
                        communication[j + 1][i] = 0
            
        for i in range(1, self.uav_num):  # 防止某个无人机掉队
            if sum(communication[i]) == 0:
                communication[i][0] = 1
        return communication

    def vel_assign(self, vel_enu):
        self.vel_enu_pub.publish(vel_enu)
        self.pose_pub.publish(self.pose)
        self.cmd_pub.publish(self.cmd)

    def loop(self):
        rospy.init_node('leader')
        rate = rospy.Rate(self.f)
        while True:
            # self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.avoid_vel.x
            # self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.avoid_vel.y
            # self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.avoid_vel.z
            # 在这里设置leader的速度
            d_p = self.calc_dist()
            if d_p==[0,0,0]:
                self.cmd_vel_enu.linear.x = 0
                self.cmd_vel_enu.linear.y = 0
                self.cmd_vel_enu.linear.z = 0
            else:
                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.f * d_p[0]
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.f * d_p[1]
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.f * d_p[2]
                cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5
                if cmd_vel_magnitude > 3**0.5 * self.max_vel:
                    self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.max_vel
                    self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.max_vel
                    self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.max_vel
                
            formation_pattern = Float32MultiArray()
            formation_pattern.data = self.new_formation.flatten().tolist()
            self.formation_pattern_pub.publish(formation_pattern)

            if(not self.communication_topology is None):
                communication_topology = Int32MultiArray()
                communication_topology.data = self.communication_topology.flatten().tolist()
                self.communication_topology_pub.publish(communication_topology)
            
            self.vel_enu_pub.publish(self.cmd_vel_enu)
            self.pose_pub.publish(self.pose)
            self.cmd_pub.publish(self.cmd)

            try:
                rate.sleep()
            except:
                continue

if __name__ == '__main__':
    leader = Leader(sys.argv[1], 0, int(sys.argv[2]), int(sys.argv[3]))
    leader.loop()
