import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Duration
import rclpy.time
import time
import os 

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
#from lh_interfaces import ObjectPresence
import sys
from importlib import import_module
from opcua import Client, ua
from lh_interfaces.msg import Statekey

#from tf2_ros import TransformListener

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
    if(cfg1==None or cfg2==None):
        return 0
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
    goal_message = FollowJointTrajectory.Goal()
    message_list=[]
    for joint_name in joint_names:
        goal_message.trajectory.joint_names.append(joint_name)        
    #if self.latest_joint_states_message is not None:
    #    from_point = JointTrajectoryPoint()
    #    from_point.positions = self.latest_joint_states_message.position
    #    from_point.velocities = [0.0 for p in from_point.positions]           
    #    goal_message.trajectory.points.append(from_point)
    cfg_last=None
    duration_last=0
    for i in range(0,n_points):
        print("top of loop")
        print(cfg_list)
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
                print("position in stepping")
                print(to_point.positions)
                to_point.velocities = vel                    
                to_point.time_from_start = Duration(seconds=duration_step).to_msg()
                goal_message.trajectory.points.append(to_point)
                if not batch_send:
                    print('message =')
                    print(format(goal_message))
                    message_list.append(goal_message)
                    goal_message.trajectory.points.clear()
                    goal_message.trajectory.points.append(to_point)
                cfg_last=cfg
                duration_last=duration        
        else:
            print("cfg=")
            print(cfg)
            print("vel=")
            print(vel)
            print("duration=")
            print(duration)            
            to_point = JointTrajectoryPoint()
            to_point.positions=cfg
            print("pos")
            print(to_point.positions)
            to_point.velocities = vel
            to_point.time_from_start = Duration(seconds=duration).to_msg()
            goal_message.trajectory.points.append(to_point)
            if not batch_send:
                print('message =')
                print(format(goal_message))
                message_list.append(goal_message)
                goal_message.trajectory.points.clear()
                goal_message.trajectory.points.append(to_point)
        cfg_last=cfg
        duration_last=duration
    print("out of the loop")
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
        

def interpolate(cfg1,cfg2, stepsize):
    n_steps=int(distance_between_cfgs(cfg1, cfg2)/stepsize)
    print(n_steps)
    steps=[cfg1]
    for i in range(1,n_steps):
        cfg_step=[]
        for j in range(0,len(cfg1)):
            x=(cfg2[j]*i+cfg1[j]*(n_steps-i))/n_steps
            cfg_step.append(x)        
        steps.append(cfg2)
    print(steps)
    return steps

                          
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
    object_position=[]
    object_presence=False
    latest_obs_camera_potition = None
    
    def __init__(self) -> None:
        super().__init__('send_traj')
        self.joint_states_subscriber = self.create_subscription(JointState, '/joint_states', self.get_joint_states, 10)#take out if causes error
        self.statekey_subscriber = self.create_subscription(Statekey, '/statekey', self.get_statekey, 10)#take out if causes error
        self.follow_joint_trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        #add position subsriber
        #self.objectdetection_subscriber = self.create_subscription(ObjectPresence, '/object_presence', self.get_object_presence, 10)#take out if causes error
        #tfBuffer = tf2_ros.Buffer()
        #listener = tf2_ros.TransformListener(tfBuffer)
        
    def get_joint_states(self, message: JointState):
        print("got mess")
        self.latest_joint_states_message = message

        
    #def get_object_presence(self, message: ObjectPresence):
        #self.object_presence=message.object_present
        #if(self.object_presence):
            #trans = tfBuffer.lookup_transform()
            #object_position = []
            
    #def get_object_position(self, message: PositionClient):
    #    print("write this")
        
    def get_statekey(self, message: Statekey):
        self.upcoming_keypoints=message.upcoming_keypoints
        self.time_to_keypoints=message.time_to_keypoints
        print("updating statekey")

    #def get_object_presence(self, message: ObjectPresence):
        #self.object_position=message.upcoming_keypoints
        
    def goto_feedback_callback(self, feedback_message):
        self.get_logger().info('Feedback: {}'.format(feedback_message))

    def get_latest_joint_statejoint_states(self):
        if(JointState==None):
            print("no joint state recieved")
            return []
        return self.JointState.position
        
    def withinTimeOfKeypoint(self, threshold):
        if len(self.time_to_keypoints) == 0:
            return False
        else:
            return self.time_to_keypoints[0] <= threshold
        
    def wait_for_time_to_keypoint(self, threshold):
        while not self.withinTimeOfKeypoint(threshold):
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

        
        


def interpolateTest(send_traj,joint_names):
    start=[-1.553973,-0.894160,1.232620,-1.485636,-0.671691,-0.146161,-0.718528, 0.277630,-0.590998,0.308936,0.312119,-0.313029,.05]
    goal=[-1.420226,-0.561880,0.837741,-1.528992,-0.919493,-0.068503,0.489961,0.286832,-0.508009,-0.303644,-0.225533,0.047273,.05]
    traj_2=interpolate(start,goal, .1)
    traj=[]
    for t in traj_2:
        traj=traj+t
    n_points=len(traj)//(2*len(joint_names)+1)
    batch_send=False
    speed_multiplier=.1
    messages =  getTrajMessage(joint_names, n_points, traj, batch_send,speed_multiplier)
    ml=messageList()
    ml.messages=[messages]
    print(messages)
    send_traj.sendMessageList(ml)

                    
    
def main(args=None):
    print("here")
    args_init=None
    rclpy.init(args=args_init)
    print("done init")
        
    #joint_names=['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']
    joint_names=[ 'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    send_traj = SendTraj()
    interpolateTest(send_traj,joint_names)
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
