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

    
def getSetGripperMessage(active):
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
                  self.messages.append([getSetGripperMessage(True)])
               else:
                  if(str(line_tmp) == "unset_grasper" or str(line_tmp) == "unset_grasper\n"):
                    self.messages.append([getSetGripperMessage(False)])
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

    def loadFromMessage(self,node):
       print("code goes here")
        
#not, this functions expects positional component of cfg
def interpolate(cfg1,cfg2, stepsize):
    n_steps=int(distance_between_cfgs(cfg1, cfg2)/stepsize)
    print(n_steps)
    steps=[cfg2]
    v=[]
    for i in range(0,len(cfg1)):
      v.append((cfg1[i]-cfg2[i])/n_steps)
    cfg_list=[]
    for i in range(0,n_steps):
        cfg_step=[]
        #cfg_list=cfg_list+[0,0,0,0,0,0,0,0,0,0,0,0]
        for j in range(0,len(cfg1)):
            x=(cfg2[j]*i+cfg2[j]*(n_steps-i))/n_steps
            cfg_step.append(x)
            cfg_list.append(x)
            #cfg_list.append(v[j])
        steps.append(cfg_step)
        cfg_list=cfg_list+v
        cfg_list.append(i*.25)
    print(steps)
    print("cfg list")
    print(cfg_list)
    return cfg_list

                          
def replan(node,joint_names,argv):
    current=node.joint_states
    target=getCfgFromGraspPoint(send_traj.latest_obs_camera_potition)  #may need to adjust for offset
    if(len.current==0):
        #no current state so can't do replanning
        return
    #clear buffer
    stepsize=.1    
    node.upcoming_keypoints=interpolate(current,target, stepsize)
    time_step=node.time_to_keypoints[-1]/len(node.upcoming_keypoints)  #aussmes last point of path is grasp time.  Is this a safe assumption?
    node.time_to_keypoints=[]    
    for i in range(0,len(node.upcoming_keypoints)):
        node.time_to_keypoints.append(i*time_step)
    node.send(joint_names,argv)


class SendTraj(Node):
    #my_waypoint: JointState = None
    latest_joint_states_message: JointState = None
    upcoming_keypoints=[]
    time_to_keypoints=[]
    object_presence=False
    latest_obj_transform: Transform = None
    #latest_obj_rotation: Rotation = None
    
    def __init__(self) -> None:
        super().__init__('send_traj')
        self.joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.get_joint_states, 10)#take out if causes error
        self.statekey_subscriber = self.create_subscription(Statekey, '/statekey', self.get_statekey, 10)#take out if causes error
        self.follow_joint_trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        #add position subsriber
        self.objectdetection_subscriber = self.create_subscription(ObjectPresence, '/object_presence', self.get_object_presence, 10)
        self.tf_subscriber = self.create_subscription(TFMessage, '/tf', self.get_tf, 10)#take out if causes error
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
#    def setupTfListener(self):   
        
    def get_joint_states(self, message: JointState):
        print("got mess",message)
        self.latest_joint_states_message = message
        
    def get_object_presence(self, message: ObjectPresence):
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
                joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']  #this is very very bad.  Need to change to pass this as a paremeter
                replan(self,joint_names,[])

    def get_transform(self, target_frame: str, source_frame: str):
        try:
            # Look up the transform from source_frame to target_frame
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {source_frame} to {target_frame}: {transform}")
            return transform
        except TransformException as e:
            self.get_logger().warn(f"Could not get transform: {e}")
            return None


        
        
    def send(self,joint_names,argv):
        n_points=int(sys.argv[1])
        print("number of points = ")
        print(n_points)
        argv.pop(0)
        argv.pop(0)
        batch_send=False
        speed_multiplier=1
        goal_messages=getTrajMessage(joint_names, n_points, argv, batch_send,speed_multiplier)

             
        for goal_message in goal_messages:
            print('message =')
            print(format(goal_message))
            if (self.follow_joint_trajectory_action_client.wait_for_server(timeout_sec=10) is False):
                return
            print("sending")
            self.follow_joint_trajectory_action_client.send_goal_async(goal_message, feedback_callback=self.goto_feedback_callback)            
            print("send")
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
                            os.system(message)
                else:
                    print("trying to send: ",message)
                    if (self.follow_joint_trajectory_action_client.wait_for_server(timeout_sec=10) is False):
                        return
                    print("sending")
                    self.follow_joint_trajectory_action_client.send_goal_async(message, feedback_callback=self.goto_feedback_callback)            
                    print("send")
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
               
def graspObj_tt(send_traj,joint_names):
    #wait for object presence
    send_traj.wait_for_object_presence()
    print("detected object")
        
    #get current cfg
    start_borked=send_traj.get_latest_joint_state()
    while len(start_borked) == 0:
        rclpy.spin_once(send_traj)
        start_borked=send_traj.get_latest_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]
    print("start",start,"__",start_borked)

    #get ee pos and rotation from transform tree
    transform_obj = None
    #while transform_obj == None:
    #    transform_obj = send_traj.get_transform('object', 'base_link_inertia')
    #print("tr obj",transform_obj)
    transform_ee = None
    while transform_ee== None:
        transform_ee=send_traj.get_transform('wrist_3_link', 'base_link_inertia')
        
    #get end effector position and rotation
    eePos,eeRotation=ur5_transforms.forward_kinematics(start,ur5_transforms.dh_params_ur5)  #make first param height of gripper
    eeRotTranspose = np.transpose(eeRotation)#[[eeRotation[j][i] for j in range(len(eeRotation))] for i in range(len(eeRotation[0]))]
    print("eePos",eePos,"==",transform_ee.transform.translation)
    print("eeRotation",eeRotation,"==",transform_ee.transrrom.rotation)

    target = transform_obj.translation
    tool_height=.0175
    target[2]=target[2]+tool_height
    
    #get goal cfg
    print("target")
    print(target)
    target_rotation=transform_obj.rotation#*send_traj.latest_obj_transform.rotation
    goal=ur5_transforms.inverse_kinematics(target,target_rotation,start,ur5_transforms.dh_params_ur5_w_tool) #to do, get right rotation based on object
    print(goal)    
    eePosGoal=ur5_transforms.forward_kinematics(goal,ur5_transforms.dh_params_ur5_w_tool)  #make second parameter camera offset
    print(eePosGoal)

    #get path from start to goal
    traj=interpolate(start,goal, .1)
    #goal.append([0,0,0,0,0,0,1])
    #traj=goal
    #return
    #send path to controller
    n_points=1#len(traj)//(2*len(joint_names)+1)
    batch_send=False
    speed_multiplier=1
    messages =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
    ml=messageList()
    ml.messages=[messages]
    print()
    #print("messages")
    #print(messages)
    print()
    print()
    send_traj.sendMessageList(ml)

    
    return


def graspObj(send_traj,joint_names):
    #wait for object presence
    send_traj.wait_for_object_presence()
    print("detected object")
        
    #get current cfg
    start_borked=send_traj.get_latest_joint_state()
    while len(start_borked) == 0:
        rclpy.spin_once(send_traj)
        start_borked=send_traj.get_latest_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]
    #start=[-1.661064926777975, -1.4241474310504358, -1.754960838948385, -1.5322664419757288, 1.5725599527359009, 3.0504724979400635]
    print("start",start,"__",start_borked)
    
    #get end effector position and rotation
    eePos,eeRotation=ur5_transforms.forward_kinematics(start,ur5_transforms.dh_params_ur5)  #make first param height of gripper
    eeRotTranspose = np.transpose(eeRotation)#[[eeRotation[j][i] for j in range(len(eeRotation))] for i in range(len(eeRotation[0]))]
    print("eePos",eePos)
    print("eeRotation",eeRotation)


    #get object transform from camera
    while send_traj.latest_obj_transform == None:
        time.sleep(.01)
        rclpy.spin_once(send_traj)

    #set offsets
    camera_offset_real=np.array([0.062,-.008,.044])
    camera_offset=np.array([0.008,-.062,.044])
    q=send_traj.latest_obj_transform.rotation

    object_rotation = ur5_transforms.rot_m_from_qu(q)

    camera_q=np.array([[0, -1, 0, 0.062],
                       [1, 0, 0, -0.008],
                       [0, 0, 1, 0.044],
                       [0, 0, 0, 1]])
    camera_rotation = ur5_transforms.rot_m_from_qu(camera_q)
    
    obj_camera_translation_v = send_traj.latest_obj_transform.translation
    obj_camera_translation=np.array([obj_camera_translation_v.x,obj_camera_translation_v.y,obj_camera_translation_v.z])
    #obj_camera_translation=[0.03405322, 0.06416492, 0.31300002]
    offset_tool=[0,0,.175]

    t1=camera_offset+obj_camera_translation
    t1a=camera_offset_real+obj_camera_translation
    t15a=np.dot(np.transpose(camera_rotation),np.transpose(t1a))
    
    t2=np.dot(np.transpose(eeRotation),np.transpose(t1))
    t2a=np.dot(np.transpose(eeRotation),np.transpose(t15a))
    
    print("t2",t2,"="t2a)
    t3=eePos+t2
    target=t3#+offset_tool

    #this block is defunct and can be deleted
    #transform camera position to frame of base    
    print("obj_camera_translation",obj_camera_translation)
    obj_ee_transform=obj_camera_translation+camera_offset
    print("object,camera",obj_camera_translation)
    obj_ee_transform=obj_ee_transform[0:3]
    print("object,ee",obj_ee_transform)
    arr=np.array([obj_ee_transform])
    obj_ee_trans_rotated=np.dot(eeRotTranspose,obj_ee_transform)
    print("obj_ee_trans_rotated",obj_ee_trans_rotated)
    object_pos_base=obj_ee_trans_rotated+eePos
    #get target position
    offset_tool=[0,0,.175+.02]
    target_2 = object_pos_base+offset_tool

    #get goal cfg
    print("target")
    print(target,"=",target_2)
    print("ee pos",eePos)
    print(eePos+object_pos_base)
    target_rotation=eeRotation*camera_rotation*object_rotation#*send_traj.latest_obj_transform.rotation
    print("target_rotation",target_rotation)
    # Rotation matrix for 180 degrees around the x-axis
    #target_rotation = np.array([
    #    [1, 0, 0],
    #    [0, -1, 0],
    #    [0, 0, -1]
    #])


    goal=ur5_transforms.inverse_kinematics(target,target_rotation,start,ur5_transforms.dh_params_ur5_w_tool) #to do, get right rotation based on object
    #goal_borked=object_pos_base[goal[1],goal[2],goal[3],goal[4],goal[5],goal[0]]
    print("goal",goal)    
    eePosGoal=ur5_transforms.forward_kinematics(goal,ur5_transforms.dh_params_ur5_w_tool)  #make second parameter camera offset
    print(eePosGoal)

    #get path from start to goal
    traj=interpolate(start,goal, .1)
    #traj=goal+[0,0,0,0,0,0,1]
    #traj=goal

    #return
    #send path to controller
    n_points=len(traj)//(2*len(joint_names)+1)
    batch_send=True#False
    speed_multiplier=.1
    messages =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
    ml=messageList()
    ml.messages=[messages]
    print()
    print("messages")
    print(messages)
    print()
    print()
    send_traj.sendMessageList(ml)
    #get current cfg

    time.sleep(5)
    start_borked=send_traj.get_latest_joint_state()
    while len(start_borked) == 0:
        rclpy.spin_once(send_traj)
        start_borked=send_traj.get_latest_joint_state()
    start=[start_borked[5],start_borked[0],start_borked[1],start_borked[2],start_borked[3],start_borked[4]]

    print("end cfg",start)
    #get end effector position and rotation
    eePos,eeRotation=ur5_transforms.forward_kinematics(start,ur5_transforms.dh_params_ur5_w_tool)  #make first param height of gripper
    eeRotTranspose = np.transpose(eeRotation)#[[eeRotation[j][i] for j in range(len(eeRotation))] for i in range(len(eeRotation[0]))]
    print("eePos",eePos)
    print("eeRotation",eeRotation)


    
def main(args=None):
    print("here")
    args_init=None
    rclpy.init(args=args_init)
    print("done init")
        
    #joint_names=['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']
    joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    send_traj = SendTraj()
    #send_traj.setupTfListener()
    ur5_transforms.testEEPosTransform()
    graspObj(send_traj,joint_names)
    return
    print("send traj")
    if(str(sys.argv[1])=="-f"):
        mess=messageList()
        mess.loadFromFile(joint_names,sys.argv[2],1)
        #waitForTrigger()
        send_traj.sendMessageList(mess)
    if(str(sys.argv[1])=="-topic"):
        print("read from topic")
    else:
        send_traj.send(joint_names,sys.argv)

        
if __name__ == '__main__':
    main()
