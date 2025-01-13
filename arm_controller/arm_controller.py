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
from geometry_msgs.msg import Transform
#from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

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


def getSetGripperMessage_clawGripper(command):
    openwidth=0.0
    closewidth=50.0
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

def publish_gripper_msg(command,node):
    msg=getSetGripperMessage_clawGripper(command)
    publisher=node.create_publisher(MoveFingers, msg[0], 10)
    # Timer to publish messages at regular intervals (2 seconds here)
    #node.timer = node.create_timer(2.0, node.publish_message)
    publisher.publish(msg[1])
    node.get_logger().info(f'Publishing: "{msg}"')
        
    
def getSetGripperMessage_magneticGripper(active):
    if active:
        message = "ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO \"fun: 1\npin: 1\nstate: 1.0\""
    else:
        message = "ros2 service call /io_and_status_controller/set_io ur_msgs/srv/SetIO \"fun: 1\npin: 1\nstate: 0.0\""
    print(message)
    return message
        
def getTrajMessage(joint_names, n_points, cfg_list,batch_send,speed_multiplier):
    #speed_multiplier=.25
    message_list=[]            
    #if self.latest_joint_states_message is not None:
    #    from_point = JointTrajectoryPoint()
    #    from_point.positions = self.latest_joint_states_message.position
    #    from_point.velocities = [0.0 for p in from_point.positions]           
    #    goal_message.trajectory.points.append(from_point)
    cfg_last=None
    duration_last=0
    for i in range(0,n_points):
        #print("top of loop")
        #print(cfg_list)
        start = i*13
        end = start + 12
        cfg = [float(cfg_list[j]) for j in range(start,start+6)]
        #cfg[4]=0.585545
        start=start+6
        #end = start+12
        vel = [float(cfg_list[j])*speed_multiplier for j in range(start,end)]
        duration = float(cfg_list[end])/speed_multiplier            
        res=.25
        d=0
        if(i!=0):
            d=distance_between_cfgs(cfg,cfg_last)
        if(d>res):            
            cfg_step=[0,0,0,0,0,0]
            cfg_step=[cfg_last[j] for j in range(0,len(cfg_last))]
            duration_step=duration_last
            n_steps=int(d//res)
            for q in range(0,n_steps):
                for j in range(0,len(cfg)):
                    cfg_step[j]=cfg_step[j]+(cfg[j]-cfg_last[j])/n_steps
                duration_step=duration_step+(duration-duration_last)/n_steps
                to_point = JointTrajectoryPoint()
                to_point.positions=cfg_step
                #print("position in stepping")
                #print(to_point.positions)
                to_point.velocities = vel                    
                to_point.time_from_start = Duration(seconds=duration_step).to_msg()
                goal_message = FollowJointTrajectory.Goal()
                
                for joint_name in joint_names:
                    goal_message.trajectory.joint_names.append(joint_name)
                goal_message.trajectory.points.append(to_point)
                if not batch_send:
                    #print('message =')
                    #print(format(goal_message))
                    message_list.append(goal_message)
                    goal_message.trajectory.points.clear()
                    goal_message.trajectory.points.append(to_point)
                cfg_last=cfg
                duration_last=duration        
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
            to_point.velocities = vel
            to_point.time_from_start = Duration(seconds=duration).to_msg()

            goal_message = FollowJointTrajectory.Goal()
            for joint_name in joint_names:
                goal_message.trajectory.joint_names.append(joint_name)
            
            goal_message.trajectory.points.append(to_point)
            if not batch_send:
                #print('message =')
                #print(format(goal_message))
                message_list.append(goal_message)
                goal_message.trajectory.points.clear()
                goal_message.trajectory.points.append(to_point)
        cfg_last=cfg
        duration_last=duration
    #print("out of the loop")
    if batch_send:
        message_list.append(goal_message)
    return message_list
    

#add load from topic
class messageList:
    messages=[]
    def loadFromArrOfStrings(self, joint_names, str_arr,speed_multiplier):
        batch_send=False
        print(str_arr)
        for Line in str_arr:
            print("Line",str(Line))
            line_arr=Line.split(' ')
            line_tmp=line_arr[0]
            if(str(line_tmp) == "-f" ):
                file1 = open(str(line_arr[1]).strip(), 'r')
                Lines = file1.readlines()
                self.loadFromArrOfStrings(joint_names, Lines,float(line_arr[2]))
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
                          #print(len(traj),len(joint_names)+1,n_points)
                          message =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
                          #interpolate(cfg1,cfg2, stepsize)
                          self.messages.append(message)
        print("here",self.messages)
    
    def loadFromFile(self,joint_names,filename,speed_multiplier):
        print("filename =",filename)
        file1 = open(filename, 'r')
        Lines = file1.readlines()
        self.loadFromArrOfStrings(joint_names, Lines,speed_multiplier)

    #def loadFromMessage(self,node):
    #   print("code goes here")
        
#not, this functions expects positional component of cfg
#add time 
def interpolate(cfg1,cfg2, stepsize,time=.25):
    n_steps=math.floor(distance_between_cfgs(cfg1, cfg2)/stepsize)
    n_steps=0
    print(n_steps)
    steps=[cfg2]
    v=[]
    for i in range(0,len(cfg1)):
      v.append((cfg1[i]-cfg2[i])/n_steps)
    cfg_list=[]
    for i in range(0,n_steps+1):
        cfg_step=[]
        #cfg_list=cfg_list+[0,0,0,0,0,0,0,0,0,0,0,0]
        for j in range(0,len(cfg1)):
            x=(cfg1[j]*i+cfg2[j]*(n_steps-i))/n_steps #todo:  change first cfg2 to cfg1 to do correctly after testing
            if(i==n_steps):
                x=cfg2[j]
            cfg_step.append(x)
            cfg_list.append(x)
            #cfg_list.append(v[j])
        steps.append(cfg_step)
        cfg_list=cfg_list+v
        cfg_list.append(i*time/(n_steps+1))
    print(steps)
    print("cfg list")
    print(cfg_list)
    return cfg_list

#defunct, replace with code from grasp function                          
#def replan(node,joint_names,argv):
#    current=node.joint_states
#    target=getCfgFromGraspPoint(node.latest_obs_camera_potition)  #may need to adjust for offset
#    if(len.current==0):
#        #no current state so can't do replanning
#        return
#    #clear buffer
#    stepsize=.1    
#    node.upcoming_keypoints=interpolate(current,target, stepsize)
#    time_step=node.time_to_keypoints[-1]/len(node.upcoming_keypoints)  #aussmes last point of path is grasp time.  Is this a safe assumption?
#    node.time_to_keypoints=[]    
#    for i in range(0,len(node.upcoming_keypoints)):
#        node.time_to_keypoints.append(i*time_step)
#    node.send_traj(joint_names,argv)


class Arm_Controller_Node(Node):
    #my_waypoint: JointState = None
    latest_joint_states_message: JointState = None
    upcoming_keypoints=[]
    time_to_keypoints=[]
    object_presence=False
    latest_obj_transform: Transform = None
    #latest_obj_rotation: Rotation = None
    arm_plan: String = None
    
    def __init__(self) -> None:
        super().__init__('send_traj')
        self.joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.get_joint_states, 10)#take out if causes error
        self.statekey_subscriber = self.create_subscription(Statekey, '/statekey', self.get_statekey, 10)#take out if causes error
        self.follow_joint_trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.object_detection_subscriber = self.create_subscription(ObjectPresence, '/object_presence', self.get_object_presence, 2)
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.get_tf, 10)#take out if causes error
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm_motion_plan_subscriber = self.create_subscription(String, '/arm_plan', self.save_arm_plan, 10)

    def save_arm_plan(self, message: String):
        print("got arm plan message",message)
        self.arm_plan = message

        
    def get_joint_states(self, message: JointState):
        print("got mess",message)
        self.latest_joint_states_message = message
        
    def get_object_presence(self, message: ObjectPresence):
        print("object present")
        self.object_presence=message.object_present
        
    def get_tf(self, message: TFMessage):
        #print("got tf")
        print(message.transforms[0].child_frame_id)
        if message.transforms[0].child_frame_id == "object":
            self.latest_obj_transform=message.transforms[0].transform
        #print(self.latest_obj_transform)
        
    def get_statekey(self, message: Statekey):
        self.upcoming_keypoints=message.upcoming_keypoints
        self.time_to_keypoints=message.time_to_keypoints
        print("updating statekey")

    #def get_object_presence(self, message: ObjectPresence):
    #    self.object_position=message.upcoming_keypoints
        
    def goto_feedback_callback(self, feedback_message):
        self.get_logger().info('Feedback: {}'.format(feedback_message))

    def get_latest_joint_state(self):
        if(self.latest_joint_states_message==None):
            #print("no joint state recieved")
            return []
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

    #################Add inturrupt to this function to do replanning        
    def wait_for_keypoint_or_position(self, threshold):
        ##########write this
        while not self.withinTimeOfKeypoint(threshold):
            time.sleep(.01)
            rclpy.spin_once(self)
            if(self.latest_obs_camera_potition!=None):
                #call grasp object function which goes to position given by object detection
                joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']  #this is very very bad.  Need to change to pass this as a paremeter
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
        speed_multiplier=1
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
                    time.sleep(1)
                    self.follow_joint_trajectory_action_client.send_goal_async(message, feedback_callback=self.goto_feedback_callback)            
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

#defunct
#def graspObj_tt(send_traj,joint_names):
#    #wait for object presence
#    send_traj.wait_for_object_presence()
#    print("detected object")
#        
#    #get current cfg
#    start_borked=send_traj.get_latest_joint_state()
#    while len(start_borked) == 0:
#        rclpy.spin_once(send_traj)
#        start_borked=send_traj.get_latest_joint_state()
#    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]
#    print("start",start,"__",start_borked)
#
#    #get ee pos and rotation from transform tree
#    transform_obj = None
#    #while transform_obj == None:
#    #    transform_obj = send_traj.get_transform('object', 'base_link_inertia')
#    #print("tr obj",transform_obj)
#    transform_ee = None
#    while transform_ee== None:
#        transform_ee=send_traj.get_transform('wrist_3_link', 'base_link_inertia')
#        
#    #get end effector position and rotation
#    eePos,eeRotation=ur5_transforms.forward_kinematics(start,ur5_transforms.dh_params_ur5)  #make first param height of gripper
#    eeRotTranspose = np.transpose(eeRotation)#[[eeRotation[j][i] for j in range(len(eeRotation))] for i in range(len(eeRotation[0]))]
#    print("eePos",eePos,"==",transform_ee.transform.translation)
#    print("eeRotation",eeRotation,"==",transform_ee.transrrom.rotation)
#    target = transform_obj.translation
#    tool_height=.0175
#    target[2]=target[2]+tool_height

#get goal cfg
#    print("target")
#    print(target)
#    target_rotation=transform_obj.rotation#*send_traj.latest_obj_transform.rotation
#    goal=ur5_transforms.inverse_kinematics(target,target_rotation,start,ur5_transforms.dh_params_ur5) #to do, get right rotation based on object
#    print(goal)    
#    eePosGoal=ur5_transforms.forward_kinematics(goal,ur5_transforms.dh_params_ur5)  #make second parameter camera offset
#    print(eePosGoal)

#    #get path from start to goal
#    traj=interpolate(start,goal, .1)
#    #goal.append([0,0,0,0,0,0,1])
#    #traj=goal
#    #return
#    #send path to controller
#    n_points=1#len(traj)//(2*len(joint_names)+1)
#    batch_send=False
#    speed_multiplier=1
#    messages =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
#    ml=messageList()
#    ml.messages=[messages]
#    print()
#    #print("messages")
#    #print(messages)
#    print()
#    print()
#    send_traj.sendMessageList(ml)   
#    return


def graspObj(node,joint_names,start,target_position,target_rotation):
    goal=ur5_transforms.inverse_kinematics(target_position,target_rotation,start,ur5_transforms.dh_params_ur5_w_tool) #added tool to dh
    print("goal",goal)
    for g in goal:
        print(180*g/3.141596)
    eePosGoal=ur5_transforms.forward_kinematics(goal,ur5_transforms.dh_params_ur5_w_tool)  #added tool to dh  
    
    #get path from start to goal
    traj=interpolate(start,goal, .1,2)
    #traj=goal+[0,0,0,0,0,0,1]
    #traj=goal

    #return
    #send path to controller
    n_points=len(traj)//(2*len(joint_names)+1)
    batch_send=False#True#False
    speed_multiplier=.1
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

    time.sleep(5)
    start_borked=node.get_latest_joint_state()
    cfg_borked=node.get_latest_joint_state()
    while len(cfg_borked) == 0:
        rclpy.spin_once(node)
        cfg_borked=node.get_latest_joint_state()
    cfg=[cfg_borked[5],cfg_borked[0],cfg_borked[1],cfg_borked[2],cfg_borked[3],cfg_borked[4]]

    print("end cfg",cfg)
    #get end effector position and rotation
    eePos,eeRotation=ur5_transforms.forward_kinematics(cfg,ur5_transforms.dh_params_ur5_w_tool)  #added tool to dh
    print("eePos",eePos)
    print("eeRotation",eeRotation)


def graspObj_w_computation(node,joint_names):
    #wait for object presence
    node.wait_for_object_presence()
    print("detected object")
        
    #get current cfg
    start_borked=node.get_latest_joint_state()
    while len(start_borked) == 0:
        rclpy.spin_once(node)
        start_borked=node.get_latest_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]
    print("start",start,"__",start_borked)
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
    offset_tool=[0,0,.195]
    target_position = T[:3, 3]#+offset_tool
    print("tp",target_position)
    target_rotation = T[:3, :3]
    graspObj(node,joint_names,start,target_position,target_rotation)


    
def graspObj_transform_tree(node,joint_names):
    #wait for object presence
    node.wait_for_object_presence()
    print("detected object")
        
    #get current cfg
    start_borked=node.get_latest_joint_state()
    while len(start_borked) == 0:
        rclpy.spin_once(node)
        start_borked=node.get_latest_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]

    #get transform of object
    transform_obj = None
    while transform_ee== None:
        transform_obj=node.get_transform('wrist_3_link', 'base_link_inertia')

    target_position_v=transform_obj.translation
    target_position=[target_position_v.x,target_position_v.y,target_position_v.z]
    target_position_q=transform_obj.rotation
    target_position=ur5_transforms.rot_m_from_qu(target_position_q)    
    graspObj(node,joint_names,start,target_position,target_rotation)


    
def main(args=None):
    print("here")
    args_init=None   
    rclpy.init(args=args_init)
    print("done init")        
    joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    node = Arm_Controller_Node()
    #publish_gripper_msg("preposition",send_traj)
    #time.sleep(4)
    #publish_gripper_msg("grasp",node)
    #time.sleep(3)
    #publish_gripper_msg("release",send_traj)
    #return
    while node.arm_plan==None:
        rclpy.spin_once(node)
    print("recieved path",node.arm_plan)
    mess=messageList()
    mess.loadFromArrOfStrings(joint_names,node.arm_plan.data.splitlines(),.1)
    node.arm_plan=None
    node.sendMessageList(mess)
    #send_traj.setupTfListener()
    ur5_transforms.testEEPosTransform()
    graspObj_w_computation(node,joint_names)
    time.sleep(4)
    publish_gripper_msg("grasp",node)
    while node.arm_plan==None:
        rclpy.spin_once(node)
    print("recieved path",node.arm_plan)
    mess=messageList()
    mess.loadFromArrOfStrings(joint_names,node.arm_plan.data.splitlines(),.1)
    node.arm_plan=None

    return
    print("send traj")
    if(str(sys.argv[1])=="-f"):
        mess=messageList()
        mess.loadFromFile(joint_names,sys.argv[2],.1)
        #waitForTrigger()
        node.sendMessageList(mess)
    if(str(sys.argv[1])=="-topic"):
        print("read from topic")
    else:
        node.send_traj(joint_names,sys.argv)

        
if __name__ == '__main__':
    main()
