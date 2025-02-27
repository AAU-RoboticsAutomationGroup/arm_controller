import rclpy
#import rospy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Duration
import rclpy.time
import time
import os
import lh_interfaces
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TwistStamped
from tf2_ros import TransformException

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Empty
from interface_wsg.msg import MoveFingers
from interface_wsg.msg import GripperState

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from lh_interfaces.msg import ObjectPresence
from lh_interfaces.msg import PathStatus
from geometry_msgs.msg import Transform
#from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path

import sys
from importlib import import_module
from opcua import Client, ua
from lh_interfaces.msg import Statekey

from tf2_ros import TransformListener
from arm_planner import ur5_transforms
import numpy as np


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import queue

joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']  

############################


#Festo stuff not being used
#IPs for each festo module
#server_ip = '172.20.8.1' #IP of CM
#server_port = 4840 #opc-ua port of CM
#sensor_name = 'ns=2;s=|var|CECC-LK.Application.TransportBranch.xBG42'

#def waitForTrigger():
#    opc_client = Client("opc.tcp://{}:{}".format(server_ip, server_port))
#    opc_client.connect()
#    carrier_detected=False
#    while(not carrier_detected):
#        print("**********************")
#        print("reading...")
#        carrier_detected = opc_client.get_node(sensor_name).get_value()
#        time.sleep(.01)

#to do, change to use cfg class
def distance_between_cfgs(cfg1, cfg2):
    d=0
    #if(cfg1==None or cfg2==None):
    #    return 0
    if(len(cfg1)!=len(cfg2)):
        print("error: computing distances of cfgs with different lengths")
        return 0
    for j in range(0,len(cfg1)):
        d=d+pow((cfg1[j]-cfg2[j]),2)
    d=pow(d,.5)
    return d

"""
def getSetGripperMessage_clawGripper(command):
    openwidth=0.0
    closewidth=30.0
    speed=200.0
    message=""
    if(command=="connect"):
        msg = Empty()
        message=["/wsg_50/connect",msg]
    if(command=="disconnect"):
        msg = Empty()
        message=["/wsg_50/disconnect",msg]
    if(command=="preposition"):
        msg = MoveFingers()
        msg.width=openwidth
        msg.speed=speed
        message=["/wsg_50/preposition",msg]
        print("prepos",message)
    if(command=="grasp"):
        msg = MoveFingers()
        msg.width=closewidth
        msg.speed=speed
        message=["/wsg_50/grasp",msg]
    if(command=="release"):
        msg = MoveFingers()
        msg.width=openwidth
        msg.speed=speed
        message=["/wsg_50/release",msg]
    if(command=="gripper_state"):
        msg = GripperState()
        message=["/wsg_50/gripper_state",msg]
    print(message)
    return message
"""

"""
def publish_gripper_msg(command,node):
    msg=getSetGripperMessage_clawGripper(command)
    publisher=node.create_publisher(MoveFingers, msg[0], 10)
    # Timer to publish messages at regular intervals (2 seconds here)
    #node.timer = node.create_timer(2.0, node.publish_message)
    publisher.publish(msg[1])
    node.get_logger().info(f'Publishing: "{msg}"')
"""
    
    
def getSetGripperMessage_magneticGripper(active):
    if active:
        message = "ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO \"fun: 1\npin: 1\nstate: 1.0\""
    else:
        message = "ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO \"fun: 1\npin: 1\nstate: 0.0\""
    print(message)
    return message
        
def getTrajMessage(joint_names, n_points, cfg_list,batch_send,speed_multiplier):
    print("************ in msg list")
    #speed_multiplier=.25
    message_list=[]            
    #if self.latest_joint_states_message is not None:
    #    from_point = JointTrajectoryPoint()
    #    from_point.positions = self.latest_joint_states_message.position
    #    from_point.velocities = [0.0 for p in from_point.positions]           
    #    goal_message.trajectory.points.append(from_point)
    cfg_last=None
    time_from_start=0
    #duration_last=0
    goal_message = FollowJointTrajectory.Goal()
    for joint_name in joint_names:
        goal_message.trajectory.joint_names.append(joint_name)
    for i in range(0,n_points):
        print("top of loop")
        #print(cfg_list)
        start = i*13
        end = start + 12
        cfg = [float(cfg_list[j]) for j in range(start,start+6)]
        #cfg[4]=0.585545
        start=start+6
        #end = start+12
        vel = [float(cfg_list[j])*speed_multiplier for j in range(start,end)]
        duration = float(cfg_list[end])/speed_multiplier*1000000000  ####1000000000 is convertion to nanoseconds            
        res=100.25
        d=0
 
        if(i!=0):
            d=distance_between_cfgs(cfg,cfg_last)
        if(d>res):            
            cfg_step=[0,0,0,0,0,0]
            cfg_step=[cfg_last[j] for j in range(0,len(cfg_last))]            
            n_steps=int(d//res)
            duration_step=duration/n_steps
            vel_step = [float(cfg_list[j])*speed_multiplier/n_steps for j in range(start,end)]

            for q in range(0,n_steps):
                for j in range(0,len(cfg)):
                    cfg_step[j]=cfg_step[j]+(cfg[j]-cfg_last[j])/n_steps
                #duration_step=duration_step+(duration-duration_last)/n_steps
                
                to_point = JointTrajectoryPoint()
                to_point.positions=cfg_step
                #print("position in stepping")
                #print(to_point.positions)
                to_point.velocities = vel_step                   ######removed vels 
                #to_point.time_from_start = Duration(seconds=duration_step).to_msg()
                to_point.time_from_start = Duration(seconds=0,nanoseconds=time_from_start).to_msg() 
                time_from_start=time_from_start+duration_step
                print("time to start", to_point.time_from_start)
                if not batch_send:
                    goal_message = FollowJointTrajectory.Goal()                
                    for joint_name in joint_names:
                        goal_message.trajectory.joint_names.append(joint_name)
                goal_message.trajectory.points.append(to_point)
                print("appending 1",goal_message)
                print()
                print()
                if not batch_send:
                    #print('message =')
                    #print(format(goal_message))
                    message_list.append(goal_message)
                    goal_message.trajectory.points.clear()
                    goal_message.trajectory.points.append(to_point)
                cfg_last=cfg
                #duration_last=duration        
        else:
            #print("cfg=")
            #print(cfg)
            #print("vel=")
            #print(vel)
            #print("duration=")
            #print(duration)            
            to_point = JointTrajectoryPoint()
            to_point.positions=cfg
            #print("pos")
            #print(to_point.positions)
            to_point.velocities = vel ###########removed velocities
            to_point.time_from_start = Duration(seconds=0,nanoseconds=time_from_start).to_msg() 
            time_from_start=time_from_start+duration
            print("time to start", to_point.time_from_start)
            if not batch_send:
                goal_message = FollowJointTrajectory.Goal()            
                for joint_name in joint_names:
                    goal_message.trajectory.joint_names.append(joint_name)
            
            goal_message.trajectory.points.append(to_point)
            print("appending 1",to_point)
            if not batch_send:
                #print('message =')
                #print(format(goal_message))
                message_list.append(goal_message)
                goal_message.trajectory.points.clear()
                goal_message.trajectory.points.append(to_point)
        cfg_last=cfg
        #duration_last=duration
    #print("out of the loop")
    if batch_send:
        message_list.append(goal_message)
    print("goal message",goal_message)
    return message_list
    

#add load from topic
class messageList:
    messages=[]
    def loadFromArrOfStrings(self, joint_names, str_arr,speed_multiplier):
        batch_send=True
        print(str_arr)
        for Line in str_arr:
            print("Line",str(Line))
            line_arr=Line.split(' ')
            line_tmp=line_arr[0]
            if(str(line_tmp) == "-f" ):
                file1 = open(str(line_arr[1]).strip(), 'r')
                Lines = file1.readlines()
                self.loadFromArrOfStrings(joint_names, Lines,float(line_arr[2]),speed_multiplier)
                # loadFromFile(joint_names,str(line_arr[1]))
            else:
               if(str(line_tmp) == "set_grasper" or str(line_tmp) == "set_grasper\n"):
                  self.messages.append([getSetGripperMessage_clawGripper("grasp")])
               else:
                  if(str(line_tmp) == "unset_grasper" or str(line_tmp) == "unset_grasper\n"):
                    self.messages.append([getSetGripperMessage_clawGripper("release")])
                  else:
                      if(str(line_tmp) == "sleep" or str(line_tmp) == "sleep_until_time_from_keypoint"):
                          self.messages.append([Line])
                      else:
                          traj = list(map(float,Line.split(' ')))
                          n_points=len(traj)//(2*len(joint_names)+1)
                          print("here",len(traj),len(joint_names)+1,n_points)
                          message =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
                          #interpolate(cfg1,cfg2, stepsize)
                          self.messages.append(message)
        print("here",self.messages)
    
    def loadFromFile(self,joint_names,filename,speed_multiplier):
        print("filename =",filename)
        file1 = open(filename, 'r')
        Lines = file1.readlines()
        self.loadFromArrOfStrings(joint_names, Lines,speed_multiplier)

        
#note, this functions expects positional component of cfg
def interpolate(cfg1,cfg2, stepsize,time=.25):
    n_steps=math.floor(distance_between_cfgs(cfg1, cfg2)/stepsize)
    #n_steps=0
    print("interpolate n steps", n_steps)
    steps=[cfg2]
    v=[]
    for i in range(0,len(cfg1)):
        v.append((cfg1[i]-cfg2[i])/(n_steps+1))
    cfg_list=[]
    for i in range(0,n_steps):
        cfg_step=[]
        #cfg_list=cfg_list+[0,0,0,0,0,0,0,0,0,0,0,0]
        for j in range(0,len(cfg1)):
            x=(cfg2[j]*i+cfg1[j]*(n_steps+1-i))/(n_steps+1) #todo:  change first cfg2 to cfg1 to do correctly after testing
            if(i==n_steps):
                x=cfg2[j]
            cfg_step.append(x)
            cfg_list.append(x)
            #cfg_list.append(v[j])
        steps.append(cfg_step)
        cfg_list=cfg_list+v
        cfg_list.append(time/(n_steps+1))
    cfg_step=[]
    #for j in range(0,len(cfg1)):
    #    cfg_step.append(cfg2[j])
    #    cfg_list.append(cfg2[j])
    #steps.append(cfg_step)
    cfg_list=cfg_list+v
    print(steps)
    print("cfg list")
    print(cfg_list)
    return cfg_list


def checkForReverseAngles(start,goal):
    for i in range(0,len(start)):
        if start[i]-goal[i] > 3.141596:
            g_candidate=goal[i]+3.141596*2
            if ur5_transforms.isInBound(g_candidate,i):
                goal[i]=g_candidate
        if goal[i]-start[i] > 3.141596:
            g_candidate=goal[i]-3.141596*2
            if ur5_transforms.isInBound(g_candidate,i):
                goal[i]=g_candidate
        return goal


def find_closest_pose(poses,x):
    best=-1
    best_i=0
    for i in range(0, len(poses)):
        point = poses[i].pose.position
        point_v=[point.x,point.y,point.z]
        d=distance_between_cfgs(point, x)        
        if i == 0 or d<best:
            best_i=i
            best=d
    return best_i

def get_base_pose_at_time(poses,t):
    for i in range(0,len(poses)):
        t_pose=poses[i].header.stamp
        if t_pose>t:
            return poses[i].pose

def get_pregrasp(poses,target,grasp_motion_duration):
    closest_i=find_closest_point(poses,x)
    t_grasp=poses[closest_i].header.stamp
    t_pregrasp_grasp-grasp_duration
    pose_pregrasp_i=get_base_pose_at_time(poses,t_pregrasp)
    initial_guess=[pose.position.x,pose.position.y,1.25]
    target_ee_position=project_point_onto_cone(initial_guess, target, cone_angle)
    target_ee_position[0]-=pose.position.x
    target_ee_position[1]-=pose.position.y
    v = np.array([0, 0, 1])  # Any 3D vector
    target_rotation = vector_to_rotation_matrix(v)    
    cfg = inverse_kinematics(target_ee_position, target_rotation,initial_guess,dh_params_ur5)
    return cfg


class Arm_Controller_Node(Node):
    #my_waypoint: JointState = None
    latest_joint_states_message: JointState = None
    upcoming_keypoints=[]
    time_to_keypoints=[]
    object_presence=False
    latest_obj_transform: Transform = None
    #latest_obj_rotation: Rotation = None
    arm_plan_buffer = queue.Queue()
    base_path: Path = None
    vel_arm_base: TwistStamped = None 
    grasping_path_state: PathStatus = None
    grasp_point=[]
    grasp_threshold_distance=.01
    
    def __init__(self) -> None:
        super().__init__('send_traj')
        self.joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.save_joint_state, 1)#take out if causes error
        self.statekey_subscriber = self.create_subscription(Statekey, '/statekey', self.get_statekey, 1)#take out if causes error
        self.follow_joint_trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.object_detection_subscriber = self.create_subscription(ObjectPresence, '/object_presence', self.get_object_presence, 1)
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.get_tf, 10)#take out if causes error
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm_motion_plan_subscriber = self.create_subscription(String, '/arm_plan', self.save_arm_plan, 10)
        self.base_path_subscriber = self.create_subscription(Path, '/plan', self.save_base_plan, 10)
        self.arm_base_vel_subscriber = self.create_subscription(TwistStamped, '/vel_arm_base', self.save_vel_arm_base, 1)
        self.grasping_path_state_subscriber = self.create_subscription(PathStatus, '/grasping_path_state', self.save_grasping_path_state, 1)

        # Publishers
        self.gripper_connect_publisher = self.create_publisher(Empty, "/wsg_50/connect", 10)
        self.gripper_disconnect_publisher = self.create_publisher(Empty, "/wsg_50/disconnect", 10)
        self.gripper_preposition_publisher = self.create_publisher(MoveFingers, "/wsg_50/preposition", 10)
        self.gripper_grasp_publisher = self.create_publisher(MoveFingers, "/wsg_50/grasp", 10)
        self.gripper_release_publisher = self.create_publisher(MoveFingers, "/wsg_50/release", 10)

    def publish_gripper_msg(self, command):
        openwidth = 0.0
        closewidth = 30.0
        speed = 200.0
        msg = None
        if (command == "connect"):
            msg = Empty()
            self.gripper_connect_publisher.publish(msg)
        elif (command == "disconnect"):
            msg = Empty()
            self.gripper_disconnect_publisher.publish(msg)
        elif (command == "preposition"):
            msg = MoveFingers()
            msg.width = openwidth
            msg.speed = speed
            self.gripper_preposition_publisher.publish(msg)
            self.get_logger().info("prepos", msg)
        elif (command == "grasp"):
            msg = MoveFingers()
            msg.width = closewidth
            msg.speed = speed
            self.gripper_grasp_publisher.publish(msg)
        elif (command == "release"):
            msg = MoveFingers()
            msg.width = openwidth
            msg.speed = speed
            self.gripper_release_publisher.publish(msg)

        self.get_logger().info(f"Published: {msg}")

    def save_arm_plan(self, message: String):
        print("got arm plan message",message)
        self.arm_plan_buffer.put(message)
        arm_plan=get_arm_plan()    
        print("recieved path",arm_plan)
        mess=messageList()
        mess.loadFromArrOfStrings(joint_names,arm_plan.data.splitlines(),1)     
        sendMessageList(mess)
        #send_traj.setupTfListener()
        ur5_transforms.testEEPosTransform()

    def save_base_plan(self, message: Path):
        print("got base plan message",message)
        self.base_plan = message
        
    def save_vel_arm_base(self, message: TwistStamped):
        print("got vel_arm_base message",message)
        self.vel_arm_base = message

    def save_grasping_path_state(self, message: PathStatus):
        print("got path status message",message)
        self.grasping_path_state = message

        
    def get_arm_plan(self):
        return self.arm_plan_buffer.get()
        
    def save_joint_state(self, message: JointState):
        self.latest_joint_states_message = message
        print()
        print("comparing to grasp point",self.grasp_point)
        if len(self.grasp_point)!=0:
            current_cfg_borked = self.latest_joint_states_message.position
            current_cfg=[current_cfg_borked[5],current_cfg_borked[0],current_cfg_borked[1],current_cfg_borked[2],current_cfg_borked[3],current_cfg_borked[4]]
            print()
            print(current_cfg)
            print()
            print(distance_between_cfgs(current_cfg,self.grasp_point))
            if distance_between_cfgs(current_cfg, self.grasp_point) < self.grasp_threshold_distance:
                print("is in distance, triggering gripper")
                self.publish_gripper_msg("grasp")
                self.grasp_point=[]
                
            
    def get_object_presence(self, message: ObjectPresence):
        #print("object present")
        self.object_presence=message.object_present
        print("object detected, performing vs")
        if self.object_presence and self.grasping_path_state.trigger:
            self.grasp_point=graspObj_transform_tree(self,joint_names,"object","ur5_base_link")
            self.publish_gripper_msg("grasp")
        
    def get_tf(self, message: TFMessage):
        #print("got tf")
        if len(message.transforms)==0:
           return
        print(message.transforms[0].child_frame_id)
        if message.transforms[0].child_frame_id == "object":
            self.latest_obj_transform=message.transforms[0].transform
        #print(self.latest_obj_transform)
        
    def get_statekey(self, message: Statekey):
        self.upcoming_keypoints=message.upcoming_keypoints
        self.time_to_keypoints=message.time_to_keypoints
        #print("updating statekey")

        
    def goto_feedback_callback(self, feedback_message):
        self.get_logger().info('Feedback: {}'.format(feedback_message))

    def get_joint_state(self):
        while self.latest_joint_states_message==None:
            print("warning:  arm controller expects joint states to be sent prior to object detection trigger")
            rclpy.spin_once(self)
        return self.latest_joint_states_message.position
        
    def withinTimeOfKeypoint(self, threshold):
        if len(self.time_to_keypoints) == 0:
            return False
        else:
            return self.time_to_keypoints[0] <= threshold
        
    def wait_for_time_to_keypoint(self, threshold):
        while not self.withinTimeOfKeypoint(threshold):
            time.sleep(.01)
            rclpy.spin_once(self)


    def wait_for_object_presence(self):
        while not self.object_presence:
            time.sleep(.01)
            rclpy.spin_once(self)

    def wait_for_keypoint_or_position(self, threshold):
        while not self.withinTimeOfKeypoint(threshold):
            time.sleep(.01)
            rclpy.spin_once(self)
            if(self.latest_obs_camera_potition!=None):
                #call grasp object function which goes to position given by object detection                
                graspObj_transform_tree(self,joint_names)
                #replan(self,joint_names,[])

    def get_transform(self, target_frame: str, source_frame: str):
        try:
            # Look up the transform from source_frame to target_frame
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {source_frame} to {target_frame}: {transform}")
            return transform
        except TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return None


        
        
    def send_traj(self,joint_names,argv):
        n_points=int(sys.argv[1])
        print("number of points = ")
        print(n_points)
        argv.pop(0)
        argv.pop(0)
        batch_send=True
        speed_multiplier=.1
        goal_messages=getTrajMessage(joint_names, n_points, argv, batch_send,speed_multiplier)

             
        for goal_message in goal_messages:
            print('message =')
            print(format(goal_message))
            if (self.follow_joint_trajectory_action_client.wait_for_server(timeout_sec=10) is False):
                return
            print("sending")
            time.sleep(1)
            self.follow_joint_trajectory_action_client.send_goal_async(goal_message, feedback_callback=self.goto_feedback_callback)            
            print("sent")
            self.get_logger().info('Sending goal: {}'.format(goal_message))
      

    def sendMessageList(self, messageList):
        for message_plan in messageList.messages:
            for message in message_plan:
                if(type(message)==str):
                    str_arr=message.split(' ')                
                    if(str_arr[0]=="sleep"):
                        time.sleep(float(str_arr[1].strip()))                       
                        continue
                    else:
                        if(str_arr[0]=="sleep_until_time_from_keypoint"):
                            self.wait_for_time_to_keypoint(float(str_arr[1].strip()))
                            continue
                        else:
                            os.system(message)#magnetic gripper message
                else:
                    print("trying to send: ",message)
                    if (self.follow_joint_trajectory_action_client.wait_for_server(timeout_sec=10) is False):
                        return
                    print("sending 2")
                    
                    self.follow_joint_trajectory_action_client.send_goal_async(message, feedback_callback=self.goto_feedback_callback)
                    time.sleep(.5)
                    print("sent")
                    self.get_logger().info('Sending goal: {}'.format(message))

        
        
def getCfgFromEEPoint(ee_pos):
    offset_hand=[-.09265,.10915,.0823]  #add claw height to param 3
    #d_hand=offset_hand[2]
    z_shoulder=.08916
    d_la=.425
    d_ua=.39225
    c_wrist=ee_pos
    c_wrist[2]=c_wrist[2]+offset_hand
    c_shoulder=[0,0,z_shoulder]
    cfg=[]
    cfg.append(atan2(ee_pos[0],ee_pos[1]))  #may need to offset this
    c_shoulder_wrist=math.sqrt(pow(c_wrist[0],2)+pow(c_wrist[1],2)+pow(c_wrist[2]-c_shoulder[2],2))
    a1=math.acos(pow(d_ua,2)/(2*c_shoulder_wrist,d_la))
    a2=math.acos(pow(c_shoulder_wrist,2)/(2*d_ua,d_la))
    a3=math.acos(pow(d_la,2)/(2*c_shoulder_wrist,d_ua))
    cfg.append(-a1-math.asin((c_wrist[2]-c_shoulder[2])/c_shoulder_wrist))
    cfg.append(a2)
    cfg.append(-3.141596+cfg[1]-cfg[2])
    cfg.append(math.asin((c_wrist[2]-c_shoulder[2])/c_shoulder_wrist))
    cfg.append(3.141596/2)
    return cfg


def get_adjusted_target(node,target,t):

    if node.vel_arm_base == None:
        print("velocity not recieved, not offsetting target")
        return target
    
    dx_base=node.vel_arm_base.twist.linear[0]*t
    dy_base=node.vel_arm_base.twist.linear[1]*t

    #adjust for fact that arm is at 45 degree angle
    # dx_arm=np.sin(3.141596/4)*dx_base+np.cos(3.141596/4)*dy_base
    # dy_arm=np.cos(3.141596/4)*dx_base+np.sin(3.141596/4)*dy_base
    
    # new base vel is the items movement in the ur5_base_link frame 
    dx_arm = dx_base 
    dy_arm = dy_base 

    target[0]=target[0]+dx_arm
    target[1]=target[1]+dy_arm
    return target


def graspObj(node,joint_names,start,target_position,target_rotation):
    clearance=-.04
    speed_multiplier=.5
    path_time_over=2
    path_time_goal=.25
    t_tf_delay=.2
    target_position=get_adjusted_target(node,target_position,(path_time_over+path_time_goal+t_tf_delay)*speed_multiplier)
    target_position[2]=target_position[2]+clearance
    #first go to point above target then go target
    target_over = target_position
    d_above = .08
    target_over[2]=target_over[2]+d_above
    goal_over=ur5_transforms.inverse_kinematics(target_over,target_rotation,start,ur5_transforms.dh_params_ur5_w_tool) #added tool to dh
    checkForReverseAngles(start,goal_over)
    goal=ur5_transforms.inverse_kinematics(target_position,target_rotation,goal_over,ur5_transforms.dh_params_ur5_w_tool) #added tool to dh
    checkForReverseAngles(goal_over,goal)
    
    print("goal",goal)
    for g in goal:
        print(180*g/3.141596)

    
    #target_position=get_adjusted_target(node,target_position,(path_time_over+path_time_goal)*speed_multiplier)
    eePosGoal=ur5_transforms.forward_kinematics(goal,ur5_transforms.dh_params_ur5_w_tool)  #added tool to dh  

    #rotate by 90 so grasping short end
    #if goal[5]>start[5]:
    #    goal[5]=goal[5]-3.141596/2
    #    goal_over[5]=goal_over[5]-3.141596/2
    #else:
    #    goal[5]=goal[5]+3.141596/2
    #    goal_over[5]=goal_over[5]+3.141596/2
    
    #get path from start to goal
    n_steps_over=10
    step_size=distance_between_cfgs(start, goal_over)/n_steps_over
    traj_over=interpolate(start,goal, step_size,path_time_over)
    n_steps_goal=3
    path_time=1
    step_size=d_above/n_steps_goal
    traj_goal=interpolate(goal_over,goal, step_size,path_time_goal)
    traj=traj_over+traj_goal

    print("traj:")
    print(traj)
    print
    
    #send path to controller
    n_points=len(traj)//(2*len(joint_names)+1)
    batch_send=False
    messages =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
    ml=messageList()
    ml.messages=[messages]
    print()
    print("messages")
    print(messages)
    print()
    print()
    node.sendMessageList(ml)


    
    #get current cfg

    #time.sleep(5)
    #start_borked=node.get_joint_state()
    #cfg_borked=node.get_joint_state()
    #while len(cfg_borked) == 0:
    #    rclpy.spin_once(node)
    #    cfg_borked=node.get_joint_state()
    #cfg=[cfg_borked[5],cfg_borked[0],cfg_borked[1],cfg_borked[2],cfg_borked[3],cfg_borked[4]]

    #print("end cfg",cfg)
    #get end effector position and rotation
    #eePos,eeRotation=ur5_transforms.forward_kinematics(cfg,ur5_transforms.dh_params_ur5_w_tool)  #added tool to dh
    #print("eePos",eePos)
    #print("eeRotation",eeRotation)
    return goal



def graspObj_w_computation(node,joint_names):
    #wait for object presence
    node.wait_for_object_presence()
    print("detected object")
        
    #get current cfg
    start_borked=node.get_joint_state()
    while len(start_borked) == 0:
        rclpy.spin_once(node)
        start_borked=node.get_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]
    print("start*************",start,"__",start_borked)
    T_ee=ur5_transforms.forward_kinematics_helper(start,ur5_transforms.dh_params_ur5)
    print("T",T_ee)
    
    #get object transform from camera
    while node.latest_obj_transform == None:
        time.sleep(.01)
        rclpy.spin_once(node)

    camera_T=np.array([[0, -1, 0, 0.062],
                       [1, 0, 0, -0.008],
                       [0, 0, 1, 0.044],
                       [0, 0, 0, 1]])

    T=np.dot(T_ee,camera_T)
    print("t2",T)
    q=node.latest_obj_transform.rotation
    object_rotation = ur5_transforms.rot_m_from_qu(q)
    print("or",object_rotation)
    object_translation=node.latest_obj_transform.translation
    print(object_rotation[0][0])
    obj_T=np.array([[object_rotation[0][0],object_rotation[0][1],object_rotation[0][2],object_translation.x],
                    [object_rotation[1][0],object_rotation[1][1],object_rotation[1][2],object_translation.y],
                    [object_rotation[2][0],object_rotation[2][1],object_rotation[2][2],object_translation.z],
                    [0, 0, 0,1]])
    print("obj_T",obj_T)
    T=np.dot(T,obj_T)
    print("t3",T)
    print("diff",T-T_ee)
    target_position = T[:3, 3]
    print("target position************",target_position)
    target_rotation = T[:3, :3]
    return graspObj(node,joint_names,start,target_position,target_rotation)


    
def graspObj_transform_tree(node,joint_names,object_name="object",base_name="ur5_base_link"):
    #wait for object presence
    #node.wait_for_object_presence()
    #print("detected object")
        
    #get current cfg
    start_borked=node.get_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]

    #get transform of object
    transform_obj=node.get_transform(base_name,object_name)
    while transform_obj == None:
        print("warning: arm controller expects object transform tree to be sent prior to object detection trigger.")
        rclpy.spin_once(node, timeout_sec=0.1)
        transform_obj=node.get_transform(base_name,object_name)
    print()
    print("Current Time = ",node.get_clock().now())
    print()
    print("transform object= ",transform_obj)
    print()
    target_position_v=transform_obj.transform.translation
    object_height=.035
    target_position=[-target_position_v.x,-target_position_v.y,target_position_v.z-object_height]
    print("target position",target_position)
    target_rotation_q=transform_obj.transform.rotation
    target_rotation=ur5_transforms.rot_m_from_qu(target_rotation_q)

    print("target rotation",target_rotation)    

    return graspObj(node,joint_names,start,target_position,target_rotation)


    
def main_script(args=None):
    print("here")
    args_init=None   
    rclpy.init(args=args_init)
    args=sys.argv[1:]
    print(args)
    script_file_name=args[0]#"src/arm_planner/arm_planner/data/outfile_table_step_
    file = open(str( script_file_name).strip(), 'r')
    Lines = file.readlines()
    print("done init")        
    joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    node = Arm_Controller_Node()
    #publish_gripper_msg("preposition",send_traj)
    #time.sleep(4)
    #publish_gripper_msg("grasp",node)
    #time.sleep(3)
    #publish_gripper_msg("release",send_traj)
    #return

    
    for line in Lines:
        line=str(line).strip()
        if line=="MP":
            while node.arm_plan_buffer.empty():
                rclpy.spin_once(node)
            arm_plan=node.get_arm_plan()    
            print("recieved path",arm_plan)
            mess=messageList()
            mess.loadFromArrOfStrings(joint_names,arm_plan.data.splitlines(),1)        
            node.sendMessageList(mess)
            #send_traj.setupTfListener()
            ur5_transforms.testEEPosTransform()
        elif line == "VS_computation":
            graspObj_w_computation(node,joint_names)
        elif line == "VS":
            graspObj_transform_tree(node,joint_names)
        elif line == "GRASP":           
            node.publish_gripper_msg("grasp")
        else:
            print("script command not recognized:",line)
    return


def main(args=None):
    args_init=None   
    rclpy.init(args=args_init)
    args=sys.argv[1:]
    print(args)
    node = Arm_Controller_Node()
    #node.publish_gripper_msg("connect")
    node.publish_gripper_msg("release")
    rclpy.spin(node)
        
if __name__ == '__main__':
    main()
