import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray
import sys

#####
# This script is to publish the observation information of the red team
# Usage: python red_observation.py {vehicle_type} {blue_num} {red_num}
#
# observation information includes: [red_pos, red_vel, blue_pos, blue_vel, target_pos, lmk_pos]
# NOTICE: observation 顺序要和训练模型的输入顺序一致！！！
# 
# By tongxi
#####

vehicle_type = sys.argv[1]
blue_num = int(sys.argv[2])
red_num = int(sys.argv[3])
lmk_num = 0 # stationary
episode_num = blue_num + red_num * 2 + 1 + lmk_num

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
        self.last_pose = msg.pose.position
        # print(self.last_pose) # global_local, does not print???
        # print("Position: x={:.2f}, y={:.2f}, z={:.2f}".format(position.x, position.y, position.z))

    def run(self):
        if self.last_pose:
            print(type(self.last_pose))
            print("Position for {:s} {:d}: x={:.2f}, y={:.2f}, z={:.2f}".format(self.name, self.id, self.last_pose.x, self.last_pose.y, self.last_pose.z))
            return self.last_pose
        # else:
        #     return None
            
class VelPrinter:
    def __init__(self, uav_id):
        self.last_vel = []
        self.id = uav_id
        rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+str(uav_id)+'/cmd_vel_enu', Twist, self.vel_callback)
    
    def vel_callback(self, msg):
        self.last_vel = msg.linear
    
    def run(self):
        if self.last_vel:
            print(type(self.last_vel))
            print("Velocity for uav {:d}: x={:.2f}, y={:.2f}, z={:.2f}".format(self.id, self.last_vel.x, self.last_vel.y, self.last_vel.z))
            return self.last_vel
        # else:
        #     return None

# TODO: 3）赋值给打印节点obs_pub

if __name__ == '__main__':
    rospy.init_node('observation_data')
    rate = rospy.Rate(1)

    obs_pub = rospy.Publisher('observation/data', Float32MultiArray)
    obs_msg = Float32MultiArray()
    obs = []

    ### 创建需要打印的对象
    pose_printer = []
    # vel_printer = []
    for i in range(0, red_num):
        print("Starting red uav %d"%i)
        # idx = i+blue_num
        pose_printer.append(VelPrinter(i+blue_num))
    for i in range(0, red_num):
        pose_printer.append(PosePrinter(vehicle_type, i+blue_num))
    
    for i in range(blue_num):
        print("Starting blue uav %d"%i)
        pose_printer.append(PosePrinter(vehicle_type, i))
    
    print("Starting landmark")
    pose_printer.append(PosePrinter('target_sephere', 0))
    for i in range(lmk_num):
        pose_printer.append(PosePrinter('lmk', i))
    
    ### 打印位置和速度，并赋予obs
    while not rospy.is_shutdown():
        obs = []
        for i in range(episode_num):
            # pose_printer[i].run()
            obs.append(pose_printer[i].run())
            
            # if (i==episode_num-1):
        obs_msg.data = obs # Problem: obs.data是list，而发布的应该是float
        # print("Observation: ", obs)
        print("Obs data type:", type(obs_msg.data), "\nObs data size:", len(obs_msg.data))
        obs_pub.publish(obs_msg)
        print("Obs data: ", obs_msg.data)
        print("\n\nGetting Position:")

        try:
            rate.sleep()
        except:
            continue
