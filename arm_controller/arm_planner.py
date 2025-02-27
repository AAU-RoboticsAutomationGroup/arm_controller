import rclpy
from rclpy.node import Node


#from rclpy.action import ActionClient
#from rclpy.time import Duration
#import rclpy.time
import time
import os 
from arm_planner import ur5_transforms
#from rclpy.node import Node

from std_msgs.msg import String
#from r2msgs.msg import StringArray
import sys

class cfg:
    cfg=[]

    def loadFromString(self,str):
        cfg=[]
        arr=str.split(' ')
        for s in arr:
            cfg.append(float(s))

    def toString(self):
        str=""
        for i in cfg:
            str = str + " " + string(i)
        return str

def getCfgFromGraspPoint(greap_point):
    print("write this")

#in message class add code to load from path
class Path:
    cfg_list=[]
    
    def loadFromArrOfString(self,arr):
        Path.cfg_list.clear()
        for s in arr:
            cfg=cfg()
            cfg.loadFromString(s)
            Path.cfg_list.append(s)
                          
    def loadFromFile(self,filename):
        file = open(filename, 'r')
        Lines = file1.readlines()
        self.loadFromArrOfStrings(Lines)

    def saveToFile(self,filename):
        f = open(filename, "a")
        f.write(self.toString())
        f.close()

    def toString(self):
        str=""
        for cfg in cfg_list:
            str=str + " " + cfg.toString()
    
    def sendOverTopic(self,publisher):
        msg=String()
        msg.data= self.toString()
        publisher.publisher_.publish(msg)
        #publisher.i+=1
        
    def sendOverTopic(self,topicName):
        print("add after create meg")

#when doing planning set start to be goal from last one
class mpProblem:
    start=[]
    goal=[]
    env_file_name=""
    exe_name="ros2 run motion_planning_server motion_planning_server_new"
    seed="111111"

    def doPlanning(self):
        cmd=mpProblem.exe_name+" "+mpProblem.env_file_name+" "+mpProblem.seed+" "+mpProblem.start+" "+mpProblem.goal
        print(cmd)
        #do planning, load into path and return


class mpProblemSet:
    start=[]
    goal=[]
    keypoints=[]
    env_file_name=""
    exe_name="ros2 run motion_planning_server motion_planning_server_new"
    seed="111111"
    mpProblems=[]


    def parseFileLine(self,line):
        arr=str.split(' ')
        sep=""
        if(arr[0]=="exe_name"):
            self.exe_name=""
            i=0
            for s in arr:
                if i!=0:
                    self.exe_name=self.exe_name+sep+arr[i]
                    sep=" "
                i=i+1
        if(arr[0]=="env_file"):
            self.env_file_name=arr[1]
        if(arr[0]=="seed"):
            self.seed=int(arr[1])
        if(arr[0]=="start" or arr[0]=="goal" or arr[0]=="grasp_point" or arr[0]=="pregrasp_point" or arr[0]=="postgrasp_point"):
            cfg=[]
            for i in range(0,len(arr)):
                if(i!=0):
                    cfg.append(float(arr[i]))            
            if(arr[0]=="start"):
                self.start=cfg
            elif(arr[0]=="end"):
                self.end=cfg
            else:
                self.keypoints.append(cfg)

    def setMPProblems(self):
        problems.clear()
        prev_g=self.start
        goals = keypoints
        goals.append(self.goal)
        for g in goals:
            problem = MPProblem()
            problem.start=prev_g
            problem.goal=g
            problem.env_file_name=self.env_file_name
            problem.exe_name=self.exe_name
            problem.seed=self.seed
            self.problems.append(problem)
            prev_g=g            
        print("code goes here")

    def loadProblemsFromFile(self,filename):
        file = open(filename, 'r')
        Lines = file1.readlines()
        for line in Lines:
            self.parseFileLine(line)
        self.setMPProblems()

                
def distance_between_cfgs(self,cfg1, cfg2):
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


    
class Publisher(Node):
    #data: String = ""
    
    def __init__(self):
        super().__init__('arm_plan_publisher')
        self.publisher_ = self.create_publisher(String, '/arm_plan', 10)
        timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def publish_path(self,path):
        msg = String()#Array()
        msg.data = path
        print("publishing",msg)
        self.publisher_.publish(msg)
        
    #def timer_callback(self):
    #    msg = String()#Array()
    #    msg.data = self.data 
    #    self.publisher_.publish(msg)
    #    self.get_logger().info('Publishing: "%s"' % msg.data)
    #    #self.i += 1



def main_planner(args=None):
    mps=mpProblemSet()
    filename="foo"
    mps.loadProblemsFromFile(filename)
    mps.setMPProblems()
    for problem in mps.mpProblems:
        problem.doPlanning()
    
#to do:  move this to seperate exe and make running planner main
def main(args=None):
    args_init=None
    rclpy.init(args=args_init)
    args=sys.argv[1:]
    print(args)
    path_file_name=args[0]#"src/arm_planner/arm_planner/data/outfile_table_step_1"
    with open(path_file_name, 'r') as file:
        path = file.read()
        print("read file")
    #pps=mpProblemSet()
    #mpps.loadProblemsFromFile(problem_file_name)
    #entire_plan=[]
    #strs_for_msg=[]
    #for problem in mpps.mpProblems:
    #    path=problem.doPlanning()
    #    strs_for_msg.append(path.toString())        
    #    entire_plan.append(path)
    publisher = Publisher()
    publisher.publish_path(path)
    #publisher.data=path
    #print("ready to send")

    #rclpy.spin(publisher)

    
if __name__ == '__main__':
    main()
