import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from gazebo_msgs.msg import ModelStates
######
# This script is to publish the posititon of target 
# (optional: add obstacles)
######

target_pose_pub = None
target_speed_pub = None
target_local_pose = PoseStamped()
target_speed = Vector3Stamped()

def gazebo_model_state_callback(msg):
    id = msg.name.index('target_sephere')
    target_local_pose.header.stamp = rospy.Time().now()
    target_local_pose.header.frame_id = 'map'
    target_local_pose.pose = msg.pose[id]
    target_speed.header.stamp = rospy.Time().now()
    target_speed.header.frame_id = 'map'
    target_speed.vector = msg.twist[id]

if __name__ == '__main__':
    rospy.init_node('target_pos')
    rate = rospy.Rate(1)
    gazebo_model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, gazebo_model_state_callback,queue_size=1)
    
    target_pose_pub = rospy.Publisher('target_sephere_0/mavros/vision_pose/pose', PoseStamped, queue_size=1) # change to local_position
    target_speed_pub = rospy.Publisher('target_sephere_0/mavros/vision_speed/speed', Vector3Stamped, queue_size=1)
    print("Get target sephere groundtruth pose")

    while not rospy.is_shutdown():
        target_pose_pub.publish(target_local_pose)
        # print(target_local_pose)
        target_speed_pub.publish(target_speed)
        try:
            rate.sleep()
        except:
            continue
