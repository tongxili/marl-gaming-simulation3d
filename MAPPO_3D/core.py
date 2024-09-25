import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Int32MultiArray, Float32MultiArray
import sys
from rule_leader import Leader
from follower import Follower

# physical/external base state of all entites
class EntityState(object):
    def __init__(self):
        # physical position
        self.p_pos = None
        # physical velocity
        self.p_vel = None

# state of agents (including communication and internal/mental state)
class AgentState(EntityState):
    def __init__(self):
        super(AgentState, self).__init__()
        # communication utterance
        self.c = None

# action of the agent
class Action(object):
    def __init__(self):
        # physical action
        self.u = None
        # communication action
        self.c = None

# properties and state of physical world entity
class Entity(object):
    def __init__(self):
        # name 
        self.name = ''
        # properties:
        self.size = 0.050
        # entity can move / be pushed
        self.movable = False
        # entity collides with others
        self.collide = True
        # material density (affects mass)
        self.density = 25.0
        # color
        self.color = None
        # max speed and accel
        self.max_speed = None
        self.accel = None
        # state
        self.state = EntityState()
        # mass
        self.initial_mass = 1.0

    @property
    def mass(self):
        return self.initial_mass

# properties of landmark entities
class Landmark(Entity):
     def __init__(self):
        super(Landmark, self).__init__()

# properties of rule-based agent (blue) entities
class Rule_based_Agent(Entity):
    def __init__(self):
        super(Rule_based_Agent, self).__init__()
        self.movable = True
        self.silent = False
        self.blind = False
        self.collide = True
        self.accel = 0.5
        self.max_speed = 0.1
        self.state = EntityState()
        self.action = Action()
        self.rule_based = True

# properties of agent entities
class Agent(Entity):
    def __init__(self):
        super(Agent, self).__init__()
        # agents are movable by default
        self.movable = True
        # cannot send communication signals
        self.silent = True
        # cannot observe the world
        self.blind = False
        # physical motor noise amount
        self.u_noise = None
        # communication noise amount
        self.c_noise = None
        # control range
        self.u_range = 1.0
        # state
        self.state = AgentState()
        # action
        self.action = Action()
        # script behavior to execute
        self.action_callback = None
        self.rule_based = False

# multi-agent world
class World(object):
    def __init__(self):
        # list of agents and entities (can change at execution-time!)
        self.agents = []
        self.landmarks = []
        self.rule_agents = []
        # communication channel dimensionality
        self.dim_c = 0
        # position dimensionality
        self.dim_p = 3
        # color dimensionality
        self.dim_color = 3
        # simulation timestep # 可能要根据实际变化，或者干脆通过accel控制红方
        self.dt = 0.1
        self.rate = rospy.Rate(1/self.dt) # rospy rate, 根据实际调整！
        # real/simulation dist & speed ratio
        self.ratio = 50.0
        self.uav_type = "iris"
        # physical damping
        self.damping = 0.25
        # contact response parameters
        self.contact_force = 1e+2
        self.contact_margin = 1e-3
        self.discrete_action = True
        self.master_world = True # mpe环境中位置来自gazebo, true; 否则初始化时gazebo由mpe决定，false
        ### ROS 消息订阅节点，在red_vs_blue中进行初始化
        self.red_num = 6
        self.blue_num = 6 
        self.blue_formation = 2 # 1, 2, 3
        self.red_pos_pub = [rospy.Publisher(self.uav_type+'_'+str(i + self.blue_num)+'/mavros/local_position/pose', PoseStamped, queue_size=1) for i in range(self.red_num)]
        self.red_vel_pub = [rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(i + self.blue_num)+'/cmd_vel_enu', Twist, queue_size=1) for i in range(self.red_num)]
        ### 蓝方leader，暂时直接调用文件？
        self.blue_pos_pub = [rospy.Publisher(self.uav_type+'_'+str(i)+'/mavros/local_position/pose', PoseStamped, queue_size=1) for i in range(self.blue_num)]
        self.blue_leader = Leader(uav_type="iris", leader_id=0, uav_num=self.blue_num, adv_num=self.red_num, formation_i=self.blue_formation)
        self.blue_follower = [Follower(uav_type="iris", uav_id=i, uav_num=self.blue_num, formation_i=self.blue_formation) for i in range(1, self.blue_num)]

    # return all entities in the world
    @property
    def entities(self):
        # print(len(self.agents),'agents',len(self.landmarks),'landmarks',len(self.rule_agents),'rule_agents')
        return self.agents + self.landmarks + self.rule_agents

    # return all agents controllable by external policies
    @property
    def policy_agents(self):
        return [agent for agent in self.agents if agent.action_callback is None]

    # return all agents controlled by world scripts
    @property
    def scripted_agents(self):
        # return [agent for agent in self.agents if agent.action_callback is not None]
        return [agent for agent in self.rule_agents]

    # update state of the world
    def step(self):
        # set actions for scripted agents 
        
        ## 对蓝方动作更新: directly adding force to rule-based agents
        if self.scripted_agents[0].movable: # leader可动时才计算力
            p_force_rule = [None] * len(self.scripted_agents)
            p_force_rule = self.apply_rule_force(p_force_rule)
            self.integrate_state(self.scripted_agents, p_force_rule)
            # agent.action = self.apply_action_force(p_force)
            
        ## 红方动作更新
        # gather forces applied to entities
        p_force = [None] * len(self.policy_agents)
        # apply agent physical controls
        p_force = self.apply_action_force(p_force)
        # apply environment forces
        p_force = self.apply_environment_force(p_force)
        # integrate physical state
        self.integrate_state(self.policy_agents, p_force)
        # update agent state
        for agent in self.agents:
            self.update_agent_state(agent)
        
        self.rate.sleep()
    
    def apply_rule_force(self, p_force):
        # check if any agent reaches landmark
        print("\nPosition: ")
        for agent in self.scripted_agents:
            print("%s pos: "%agent.name, agent.state.p_pos)
            for lmk in self.landmarks:
                if 'landmark' in lmk.name:
                    continue
                if 'food' in lmk.name:
                    delta_pos_blue_food = lmk.state.p_pos - agent.state.p_pos
                    distance_blue_food = np.sqrt(np.sum(np.square(delta_pos_blue_food)))
                    if(distance_blue_food < agent.size + lmk.size):
                        p_force = [np.zeros(agent.action.u.shape)] * len(self.scripted_agents)
                        # print("Blue wins!")
                        # self.scripted_agents[0].movable = False
                        # lmk.color = np.array([0, 0, 1])
                        return p_force
        
        # set applied forces on rule-based agents, currently same as leader
        leader_agent = self.scripted_agents[0]
        p_force_leader = np.zeros(leader_agent.action.u.shape)
        # print("action shape: ", leader_agent.action.u.shape)
        scale_blue_red = 0.5
        scale_blue_food = 3
        border_force = 0.9
        
        # 判断是否靠近边界[0, 1]
        if(leader_agent.state.p_pos[0] < 0.1):
            p_force_leader[0] += border_force
        elif(leader_agent.state.p_pos[0] > 0.9):
            p_force_leader[0] -= border_force
        if(leader_agent.state.p_pos[1] < 0.1):
            p_force_leader[1] += border_force
        elif(leader_agent.state.p_pos[1] > 0.9):
            p_force_leader[1] -= border_force
        if(leader_agent.state.p_pos[2] < 0.1):
            p_force_leader[2] += border_force
        elif(leader_agent.state.p_pos[2] > 0.9):
            p_force_leader[2] -= border_force
        
        for agent in self.policy_agents:
            # repulsion from red agents
            print("%s pos: "%agent.name, agent.state.p_pos)
            delta_pos_blue_red = agent.state.p_pos - leader_agent.state.p_pos
            distance_blue_red = np.sqrt(np.sum(np.square(delta_pos_blue_red)))
            d_t_blue_red = delta_pos_blue_red / distance_blue_red # direction vector from leader to blue
            p_force_leader[0] -= d_t_blue_red[0] * np.exp(-distance_blue_red) * scale_blue_red
            p_force_leader[1] -= d_t_blue_red[1] * np.exp(-distance_blue_red) * scale_blue_red
            p_force_leader[2] -= d_t_blue_red[2] * np.exp(-distance_blue_red) * scale_blue_red
            # print(distance_blue_red, d_t_blue_red, p_force_leader)

        for lmk in self.landmarks:
            # repulsion(or attraction) from landmarks
            delta_pos_blue_landmark = lmk.state.p_pos - leader_agent.state.p_pos
            distance_blue_landmark = np.sqrt(np.sum(np.square(delta_pos_blue_landmark)))
            d_t_blue_lmk = delta_pos_blue_landmark / distance_blue_landmark
            
            if 'food' in lmk.name:
                print("%s pos: "%lmk.name, lmk.state.p_pos)
                p_force_leader[0] += d_t_blue_lmk[0] * np.exp(-distance_blue_landmark) * scale_blue_food
                p_force_leader[1] += d_t_blue_lmk[1] * np.exp(-distance_blue_landmark) * scale_blue_food
                p_force_leader[2] += d_t_blue_lmk[2] * np.exp(-distance_blue_landmark) * scale_blue_food
            elif 'landmark' in lmk.name:
                p_force_leader[0] -= d_t_blue_lmk[0] * np.exp(-distance_blue_landmark) * scale_blue_red
                p_force_leader[1] -= d_t_blue_lmk[1] * np.exp(-distance_blue_landmark) * scale_blue_red
                p_force_leader[2] -= d_t_blue_lmk[2] * np.exp(-distance_blue_landmark) * scale_blue_red

        # acceleration limit
        p_force_scale = np.sqrt(np.square(p_force_leader[0]) + np.square(p_force_leader[1]))
        if p_force_scale > leader_agent.accel:
            p_force_leader = p_force_leader / p_force_scale * leader_agent.accel
        
        p_force = [p_force_leader] * len(self.scripted_agents)
        return p_force

    # gather agent action forces
    def apply_action_force(self, p_force):
        # set applied forces
        for i,agent in enumerate(self.agents):
            if agent.movable:
                print("red agent.action.u: ", agent.action.u)
                noise = np.random.randn(*agent.action.u.shape) * agent.u_noise if agent.u_noise else 0.0
                p_force[i] = agent.action.u + noise                
        return p_force

    # gather physical forces acting on entities
    def apply_environment_force(self, p_force):
        # simple (but inefficient) collision response
        for a,entity_a in enumerate(self.entities):
            for b,entity_b in enumerate(self.entities):
                if(b <= a): continue
                [f_a, f_b] = self.get_collision_force(entity_a, entity_b)
                if(f_a is not None):
                    if(p_force[a] is None): p_force[a] = 0.0
                    p_force[a] = f_a + p_force[a] 
                if(f_b is not None):
                    if(p_force[b] is None): p_force[b] = 0.0
                    p_force[b] = f_b + p_force[b]        
        return p_force

    # integrate physical state
    def integrate_state(self, agents, p_force):
        for i, entity in enumerate(agents):
            if entity.death:
                print("Agent %s is dead!"%entity.name)
                vel_to_pub = Twist(linear=Vector3(x=0, y=0, z=0))
            else:
                # print("Agent %s p_force: "%entity.name, p_force[i])
                if (p_force[i] is not None):
                    entity.state.p_vel += (p_force[i]/entity.mass) * self.dt
                if entity.max_speed is not None:
                    speed = np.sqrt(np.sum(np.square(entity.state.p_vel)))
                    if speed > entity.max_speed:
                        entity.state.p_vel = entity.state.p_vel / speed * entity.max_speed
                entity.state.p_pos += entity.state.p_vel * self.dt
                ### publish speed
                vel_to_pub = Twist(linear=Vector3(x=self.ratio * entity.state.p_vel[0], y=self.ratio * entity.state.p_vel[1], z=self.ratio * entity.state.p_vel[2]))
                
            if 'blue' in entity.name:
                if(i==0):
                    print("Blue leader force and vel: \n", p_force, '\n', vel_to_pub.linear)
                    self.blue_leader.vel_assign(vel_to_pub)
                else:
                    self.blue_follower[i-1].vel_assign(vel_to_pub) # 实际上follower的速度由自己计算
                    
            else:
                # print("Red agent %d force and vel:\n"%i, p_force, '\n', vel_to_pub.linear)
                self.red_vel_pub[i].publish(vel_to_pub)


    def update_agent_state(self, agent):
        # set communication state (directly for now)
        if agent.silent:
            agent.state.c = np.zeros(self.dim_c)
        else:
            noise = np.random.randn(*agent.action.c.shape) * agent.c_noise if agent.c_noise else 0.0
            agent.state.c = agent.action.c + noise      

    # get collision forces for any contact between two entities
    def get_collision_force(self, entity_a, entity_b):
        if (not entity_a.collide) or (not entity_b.collide):
            return [None, None] # not a collider
        if (entity_a is entity_b):
            return [None, None] # don't collide against itself
        # compute actual distance between entities
        delta_pos = entity_a.state.p_pos - entity_b.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        # minimum allowable distance
        dist_min = entity_a.size + entity_b.size
        # softmax penetration
        k = self.contact_margin
        penetration = np.logaddexp(0, -(dist - dist_min)/k)*k
        # force = self.contact_force * delta_pos / dist * penetration
        # force_a = +force if entity_a.movable else None
        # force_b = -force if entity_b.movable else None
        force_a = None
        force_b = None
        return [force_a, force_b]