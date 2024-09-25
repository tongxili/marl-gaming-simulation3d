import numpy as np
import random
import math
import copy
from core import World, Agent, Landmark, Rule_based_Agent
# from multiagent.scenario import BaseScenario

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import sys

BLUE_NUM = 6
RED_NUM = 6
LMK_NUM = 0 # no obstacles
ELEMENT_NUM = BLUE_NUM + RED_NUM * 2 + 1 + LMK_NUM

class PosePrinter:
    def __init__(self, name, uav_id):
        self.last_pose = []
        self.name = name
        self.id = uav_id
        rospy.Subscriber(name+'_'+str(uav_id)+'/mavros/vision_pose/pose', PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        """
        Callback function for subscriber
        Get new pose
        """
        # msg.pose.position是geometry_msgs.msg.Point 类型的消息，需要转换为list
        self.last_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # print("Position: x={:.2f}, y={:.2f}, z={:.2f}".format(position.x, position.y, position.z))

    def run(self):
        if self.last_pose:
            print("Position for {:s} {:d}: x={:.2f}, y={:.2f}, z={:.2f}".format(self.name, self.id, self.last_pose[0], self.last_pose[1], self.last_pose[2]))
            return self.last_pose
        # else:
        #     return None
            
class VelPrinter:
    def __init__(self, uav_id):
        self.last_vel = []
        self.id = uav_id
        rospy.Subscriber('/xtdrone/'+'iris'+'_'+str(uav_id)+'/cmd_vel_enu', Twist, self.vel_callback)
    
    def vel_callback(self, msg):
        self.last_vel = [msg.linear.x, msg.linear.y, msg.linear.z]
    
    def run(self):
        if self.last_vel:
            # print("Velocity for uav {:d}: x={:.2f}, y={:.2f}, z={:.2f}".format(self.id, self.last_vel[0], self.last_vel[1], self.last_vel[2]))
            return self.last_vel

class Scenario():
    def __init__(self):
        self.ratio = 50 # XTDrone和env的比例
        # 用于接受info的节点
        rospy.init_node('scenario')
        self.pose_printer = []
        # vel_printer = []
        for i in range(0, RED_NUM):
            print("Starting red uav %d"%i)
            # idx = i+blue_num
            self.pose_printer.append(VelPrinter(i+BLUE_NUM))
        for i in range(0, RED_NUM):
            self.pose_printer.append(PosePrinter("iris", i+BLUE_NUM))
        
        for i in range(BLUE_NUM):
            print("Starting blue uav %d"%i)
            self.pose_printer.append(PosePrinter("iris", i))
        
        print("Starting landmark")
        self.pose_printer.append(PosePrinter('target_sephere', 0))
        for i in range(LMK_NUM):
            self.pose_printer.append(PosePrinter('lmk', i))
        
        self.obs = []
        self.rate = rospy.Rate(10)
    
    def get_obs(self):
        """
        从XTDrone [0, 100] -> mpe [-1, 1]
        """
        self.obs = []
        for i in range(ELEMENT_NUM):
            obs_i = self.pose_printer[i].run()
            if obs_i is None:
                obs_i = [0.0, 0.0, 0.0]
            else:
                for j in range(3):
                    obs_i[j] = obs_i[j]/self.ratio -1
            self.obs.append(obs_i)
        
        # print("Obs data type:", type(self.obs), "\nObs data size:", len(self.obs))
        # print("Obs data:", self.obs)
        
        self.rate.sleep()
        
    # def obs_callback(self, msg):
    #     print("obs_callback", msg)
    #     self.obs = msg.data
        
    def make_world(self):
        world = World()
        world.ratio = self.ratio
        #配置无人机个数，环境维度
        world.dim_c = 0
        world.dim_p = 3
        
        num_agents = 6 # red
        num_rule_agents = 6 # blue
        world.red_num = num_agents
        world.blue_num = num_rule_agents
        num_landmarks = 0 # TEMP: no obstacles
        num_food = 1

        self.num_rule_agents = num_rule_agents
        self.num_good_agents = num_agents
        self.num_food = num_food
        #对无人机进行状态设置等等
        world.agents = [Agent() for _ in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'red %d' % i
            agent.collide = True
            agent.silent = True
            agent.accel = 0.4
            agent.size = 0.02
            agent.max_speed = 0.1
            agent.death = False
            agent.pusai = np.pi
            agent.hit = False
            agent.movable = True
        
        # rule_based_agents
        world.rule_agents = [Rule_based_Agent() for _ in range(num_rule_agents)]
        for i, rule_agent in enumerate(world.rule_agents):
            rule_agent.name = 'blue %d' % i
            rule_agent.collide = True
            rule_agent.silent = True
            rule_agent.accel = 0.4
            rule_agent.size = 0.02
            rule_agent.max_speed = 0.1 # 实际发现太快了，改小一点
            rule_agent.death = False
            rule_agent.pusai = np.pi
            rule_agent.hit = False
            rule_agent.win = False # 蓝方到达目标点
            rule_agent.movable = True
            rule_agent.action.u = np.zeros(world.dim_p) # initialize action.u
            rule_agent.action.c = np.zeros(world.dim_c)

        #从上到下为food 0 1 2
        world.food = [Landmark() for i in range(num_food)]
        for i, landmark in enumerate(world.food):
            landmark.name = 'food %d' % i
            landmark.collide = True
            landmark.movable = False
            landmark.size = 0.03
            landmark.boundary = False
            landmark.death = False
        
        #障碍物
        world.landmarks = [Landmark() for _ in range(num_landmarks)]
        for i, landmark in enumerate(world.landmarks):
            landmark.name = 'landmark %d' % i
            landmark.collide = True
            landmark.movable = False
            landmark.size = 0.025
            landmark.boundary = False
            landmark.death = False
        
        world.landmarks += world.food
        # make initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):
        #一轮就一次
        print("Resetting world...")
        for agent in world.agents:
            agent.color = np.array([255, 0, 0])
        for i, landmark in enumerate(world.landmarks):
            landmark.color = np.array([0, 255, 255])
        for i, landmark in enumerate(world.food):
            landmark.color = np.array([1, 1, 0])
        
        # generate random positions
        screen_size = [-0.9, 0.9]
        # screen_size = [0.1, 0.9]

        ## 根据gazebo位置进行初始化
        if world.master_world:
            self.get_obs()
            blue_pos, red_pos, food_pos = subscribeRealPos(screen_size=screen_size, scale=100, num_blue=self.num_rule_agents, 
                                                           num_red=self.num_good_agents, num_food=self.num_food, obs=self.obs)

        else:
            # 随机初始化并给gazebo
            blue_pos, red_pos, food_pos = generate_initial_coordinates(screen_size=screen_size, num_rule=self.num_rule_agents, dist_rule=0.06,
                                                                            num_adv=self.num_good_agents, num_food=self.num_food, random_init=False)
            
            # TODO: 更新随机位置信息到gazebo中
            red_ros_sim_pos = [PoseStamped() for _ in range(self.num_good_agents)]
            blue_ros_sim_pos = [PoseStamped() for _ in range(self.num_rule_agents)]
            lmk_ros_sim_pos = [PoseStamped() for _ in range(self.num_food)]
        
        # print("Pos: ", blue_pos[0], food_pos[0])
        for rule_agent in world.rule_agents:
            rule_agent.color = np.array([0, 0, 255])
        
        for i, agent in enumerate(self.good_agents(world)):
            # interval = 2.0 / (len(self.good_agents(world)) + 1)
            # print(interval)，中心距离为0.5
            # agent.state.p_pos = np.array([-0.9, 1 - (i+1)*interval])
            agent.state.p_pos = red_pos[i]
            agent.state.p_vel = np.zeros(world.dim_p)
            agent.state.c = np.zeros(world.dim_c)
            
            agent.death = False
            agent.hit = False

            if not world.master_world:
                [x, y, z] = red_pos[i].tolist()
                red_ros_sim_pos[i].pose.position = Point(x*world.ratio, y*world.ratio, z*world.ratio)
                world.red_pos_pub[i].publish(red_ros_sim_pos[i])
        
        for i, rule_agent in enumerate(self.rule_agents(world)):
            # interval = 2.0 / (len(self.rule_agents(world)) + 1)
            # rule_agent.state.p_pos = np.array([0.8, 1 - (i+1)*interval])
            rule_agent.movable = True
            rule_agent.state.p_pos = blue_pos[i]
            rule_agent.state.p_vel = np.zeros(world.dim_p)
            rule_agent.state.c = np.zeros(world.dim_c)
            
            rule_agent.death = False
            rule_agent.hit = False
            rule_agent.win = False

            if not world.master_world:
                [x, y, z] = blue_pos[i].tolist()
                blue_ros_sim_pos[i].pose.position = Point(x*world.ratio, y*world.ratio, z*world.ratio)
                world.blue_pos_pub[i].publish(blue_ros_sim_pos[i])

        for i, landmark in enumerate(world.landmarks):
            if not landmark.boundary: # other landmarks
                # Random position and 0 velocity for landmarks
                landmark.state.p_pos = np.random.uniform(0.2, 0.8, world.dim_p)
                landmark.state.p_vel = np.zeros(world.dim_p) # 速度为0

        for i, landmark in enumerate(world.food):
            # landmark.state.p_pos = food_pos[i] # change for 4 quardants
            landmark.state.p_pos = food_pos[i] # 设置固定位置 
            landmark.state.p_vel = np.zeros(world.dim_p)
            
            if not world.master_world:
                [x, y, z] = food_pos[i].tolist()
                # lmk_ros_sim_pos[i].pose.position = Point(x*world.ratio, y*world.ratio, z*world.ratio)
                lmk_ros_sim_pos[i].pose.position = Point(60, 0, 60)
                # world.food_pos_pub[i].publish(lmk_ros_sim_pos[i])
    
        world.rate.sleep()
        # world.step()

    def benchmark_data(self, agent, world):
        # returns data for benchmarking purposes
        if agent.adversary:
            collisions = 0
            for a in self.good_agents(world):
                if self.is_collision(a, agent):
                    collisions += 1
            return collisions
        else:
            return 0

    def is_collision(self, agent1, agent2):
        delta_pos = agent1.state.p_pos - agent2.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        dist_min = agent1.size + agent2.size # TODO: change scale?
        return True if dist <= dist_min else False

    def good_agents(self, world):
        return [agent for agent in world.agents]
    
    def rule_agents(self, world):
        return [agent for agent in world.rule_agents]

    def reward(self, agent, world): # reward for red object
        main_reward = self.agent_reward(agent, world)
        return main_reward

    def agent_reward(self, agent, world):
        # ADD: agent reward for red agents, adding capture of blue
        if agent.death:
            return 0
        
        if (agent.state.p_pos[0] > 1 or agent.state.p_pos[0] < -1 or agent.state.p_pos[1] > 1 or agent.state.p_pos[1] < -1 or
            agent.state.p_pos[2] > 1 or agent.state.p_pos[2] < -1):
            rew = -5 # 越界扣分
        else:
            rew = 0            
        
        for obstacle in world.landmarks:
            # 和landmarks非常近时，奖励相应增加或减少
            # 暂时没用到障碍物landmark
            if np.sqrt(np.sum(np.square(obstacle.state.p_pos - agent.state.p_pos)))<0.1:
                if 'landmark' in obstacle.name:
                    rew -= 5 # 不能太近
                # elif 'food' in obstacle.name:
                #     rew -= 5

            if self.is_collision(obstacle, agent):
                if 'landmark' in obstacle.name:
                    agent.death = True
                    rew-=3
        
        for other_agent in world.agents:
            if agent == other_agent: continue
            if np.sqrt(np.sum(np.square(other_agent.state.p_pos - agent.state.p_pos)))<0.1:
                rew -= 1 # 红方相互之间不能太近
            if self.is_collision(agent, other_agent):
                rew -= 2 # 红方不能相撞
        
        num_rule_agents = len(world.rule_agents)
        for i, rule_agent in enumerate(world.rule_agents):
            if rule_agent.death: 
                rew += 1
                continue
            # 红方和蓝方，距离越近，reward越大
            dist_red = np.sqrt(np.sum(np.square(agent.state.p_pos - rule_agent.state.p_pos)))
            if dist_red<0.1:
                rew += 1
            rew += 2.5 * np.exp(-dist_red) / num_rule_agents # 平均一下

            if self.is_collision(agent, rule_agent):
                rew += 10
                if i==0:
                    rew += 10 # 碰撞leader，奖励更多
                agent.death = True
                # agent.movable = False
                rule_agent.death = True
                print("reward: {} collision!".format(agent.name + ' ' + rule_agent.name))
                rule_agent.color = np.array([0, 0, 0])
                rule_agent.movable = False
            
            # 蓝方和目标点
            for target in world.food: 
                dist_food = np.sqrt(np.sum(np.square(target.state.p_pos - rule_agent.state.p_pos)))
                # rew += 2 * dist_food # 蓝方距离目标点越远，reward越大
                rew -= 5 * np.exp(-dist_food) / num_rule_agents # 蓝方距离目标点越近，reward扣越多
                
                # 蓝方到达目标点，扣分
                if self.is_collision(rule_agent, target):
                    print("reward: blue get food!")
                    rew -= 10
                    rule_agent.win = True
                    rule_agent.death = True
                    rule_agent.movable = False
                    self.rule_agents(world)[0].win = True # leader也到达
                    self.rule_agents(world)[0].death = True
                    self.rule_agents(world)[0].movable = False
        
        #这一段训练后期再加
        #for i, landmark in enumerate(world.food):
        #    if(landmark.color == ([1, 0, 0])):
        #                finished = True
        #                print("赢了！")world.agents
        def bound(x):
            if x < 0.9:
                return 0
            if (x < 1.0 and x > 0.9 ):
                return (x - 0.9) * 10
            return min(np.exp(2 * x - 2), 10)

        for p in range(world.dim_p):
            x = abs(agent.state.p_pos[p])
            rew -= 3 * bound(x)

        return rew

    def observation(self, agent, world):
        # # ADD: 修改get_obs，从节点中读取
        idx = int(agent.name.split('red')[1])
        idx_pos = (idx + self.num_good_agents)

        obs_n = []
        # while len(self.obs) == 0:
        #     self.get_obs()
        if(idx==0):
            print("Take a step and get Observation...")
            self.get_obs()
            print("Obs:\n", self.obs)
            for i, red_agent in enumerate(world.agents):
                if red_agent.death:
                    continue
                else:
                    red_agent.state.p_pos = np.array(self.obs[i+world.red_num])
            
            for i, blue_agent in enumerate(world.rule_agents):
                if blue_agent.death:
                    continue
                else:
                    blue_agent.state.p_pos = np.array(self.obs[i+world.red_num*2])
        # print("adding obs", idx)
        # print("original obs shape", len(self.obs), len(self.obs[1])) # 19*3
        
        for vel in self.obs[idx]:
            obs_n.append(vel)
        for pos in self.obs[idx_pos]:
            obs_n.append(pos)
        # obs_n.append(self.obs[idx_pos])

        for i, item in enumerate(self.obs):
            if i==idx or i==idx_pos:
                continue
            else:
                for pos in item:
                    obs_n.append(pos)

        obs_n = np.array(obs_n)
        # print("obs_n: ", obs_n)
        # print("obs_n shape: ", len(obs_n))
        return obs_n
        # return np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + other_vel +  other_pos + rule_pos + entity_pos)
        
        
    def finish(self, agent, world):
    # 判断特定agent是否结束（死亡或对方leader死亡）
        rule_agents = self.rule_agents(world)
        if rule_agents[0].death:
            return True
        
        if agent.death:
            return True
        return False

    def get_info(self, agent, world): # win info
        rule_agents = self.rule_agents(world)
        red_agents = self.good_agents(world)
        if rule_agents[0].death and not rule_agents[0].win:
            return "Red wins!"
        elif(rule_agents[0].win or all(red_agents[i].death for i in range(len(red_agents)))): # all red agents are dead
            return "Blue wins!"
        else:
            return "Training..."

def generate_initial_coordinates(screen_size, num_rule, dist_rule, num_adv, num_food, random_init=False):
    '''
    生成初始位置
    输入：
        坐标范围screen_size 蓝方数量num_rule 蓝方follower和leader的距离dist_rule 红方数量num_adv 目标点数量num_food 是否随机random
    随机：红方和目标在同一个象限，蓝方在对角线
    固定：蓝方[-0.3, -0.3, -0.3] 目标[0.2, 0.2, 0.2] 红方[0-0.4]随机
    返回：
        坐标(np.array格式) blue_swarm_pos, red_swarm_pos, food_check_pos
    '''
    blue_swarm_pos = []
    red_swarm_pos = []
    food_check_pos = []

    x_mid = sum(screen_size) / 2
    y_mid = sum(screen_size) / 2
    z_mid = sum(screen_size) / 2

    if random_init:
        x = random.uniform(screen_size[0], screen_size[1])
        y = random.uniform(screen_size[0], screen_size[1])
        z = random.uniform(screen_size[0], screen_size[1])
        blue_swarm_pos.append([x, y, z])
        for i in range(num_rule-1):
            angle = 2 * np.pi * i / (num_rule-1)
            x_blue = x + dist_rule * np.cos(angle)
            y_blue = y + dist_rule * np.sin(angle)
            z_blue = z + dist_rule * np.cos(angle)
            blue_swarm_pos.append([x_blue, y_blue, z_blue])

        red_mark=[]
        red_mark.append(-1 if x < x_mid else 1)
        red_mark.append(-1 if y < y_mid else 1)
        red_mark.append(-1 if z < z_mid else 1)

        # 生成红方位置
        red_swarm_pos = [[0.5, 0.5, 0.5] for _ in range(num_adv)]
        food_check_pos = [[0.6, 0.6, 0.6] for _ in range(num_food)]

        if red_mark[0] == 1:
            for i in range(num_adv):
                red_swarm_pos[i][0] = random.uniform(0.1, x_mid)
            for i in range(num_food):
                food_check_pos[i][0] = random.uniform(0.3, 0.4)
        else:
            for i in range(num_adv):
                red_swarm_pos[i][0] = random.uniform(x_mid, 0.9)
            for i in range(num_food):
                food_check_pos[i][0] = random.uniform(0.6, 0.7)
        
        if red_mark[1] == 1:
            for i in range(num_adv):
                red_swarm_pos[i][1] = random.uniform(0.1, y_mid)
            for i in range(num_food):
                food_check_pos[i][1] = random.uniform(0.3, 0.4)
        else:
            for i in range(num_adv):
                red_swarm_pos[i][1] = random.uniform(y_mid, 0.9)
            for i in range(num_food):
                food_check_pos[i][1] = random.uniform(0.3, 0.4)
        
        if red_mark[2] == 1:
            for i in range(num_adv):
                red_swarm_pos[i][2] = random.uniform(0.1, z_mid)
            for i in range(num_food):
                food_check_pos[i][2] = random.uniform(0.3, 0.4)
        else:
            for i in range(num_adv):
                red_swarm_pos[i][2] = random.uniform(z_mid, 0.9)
            for i in range(num_food):
                food_check_pos[i][2] = random.uniform(0.3, 0.4)
        

    else:
        x, y, z = -0.3, -0.3, -0.3
        blue_swarm_pos.append([x, y, z])
        for i in range(num_rule-1):
            angle = 2 * np.pi * i / (num_rule-1)
            x_blue = x + dist_rule * np.cos(angle)
            y_blue = y + dist_rule * np.sin(angle)
            z_blue = z + dist_rule * np.cos(angle)
            blue_swarm_pos.append([x_blue, y_blue, z_blue])
        
        for i in range(num_food):
            food_check_pos.append([0.2, 0, 0.2])
        
        for _ in range(num_adv):
            x = random.uniform(0, 0.4)
            y = random.uniform(0, 0.4)
            z = random.uniform(0, 0.4)
            red_swarm_pos.append([x, y, z])

               
    return np.array(blue_swarm_pos), np.array(red_swarm_pos), np.array(food_check_pos)

def subscribeRealPos(screen_size, num_blue, num_red, num_food, obs, scale=100):
    """
        从节点获得真实信息，返回用于初始化
    """
    food_check_pos = [[0.2, 0, 0.2]]
    blue_swarm_pos = []
    red_swarm_pos = []
    
    is_init = False
    print(len(obs))
    if len(obs) == num_blue+num_red+num_food:
        is_init=True
    else:
        is_init=False

    dim = len(obs[0])
    for i in range(num_red):
        if is_init:
            # pos = [obs[i][j]/scale for j in range(dim)]
            pos = obs[i] # 现在的obs已经除了比例
        else:
            # pos = [obs[i*2+1][j]/scale for j in range(dim)]
            pos = obs[i+num_red]
        red_swarm_pos.append(pos)
        
    for i in range(num_blue):
        if is_init:
            # pos = [obs[i+num_red][j]/scale for j in range(dim)]
            pos = obs[i+num_red]
        else:
            # pos = [obs[i+num_red*2][j]/scale for j in range(dim)]
            pos = obs[i+num_red*2]
        blue_swarm_pos.append(pos)

    # print("Original pos: ", blue_swarm_pos, red_swarm_pos, food_check_pos)
    
    return np.array(blue_swarm_pos), np.array(red_swarm_pos), np.array(food_check_pos)