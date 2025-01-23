This package contains the controller for the little helper arm.  The arm controller can be executed as follows

ros2 run arm_controller arm_controller "script_file_name"

Where script_file_name is defines the sequence of controls that the controller will execute.  Right now the controller is able to execute the following

MP:  Execute a motion planning path sent by the arm planner
VS:  Execute a visual servoing path that goes to a target sent by the object detection module.  
GRASP:  Close the grasper.

I will shortly add the following options
SLEEP
UNGRASP


Examples of script files are found in

src/arm_controller/scripts/



The arm controller requires the following packages

ur_driver:  recieves and executes paths
arm_planner:  Sends motion plans for the arm
object_detection:  Sends target positions used by VS
lh_interfaces:  Defines messages used by object detection

