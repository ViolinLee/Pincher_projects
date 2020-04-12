The purpose of this chapter (project) is to familiarize ourselves with move_it ros package and use it to control the motion of pincher arm.

## Preparations
### ROS Packages:
1.[arbotix](http://wiki.ros.org/arbotix) -- ArbotiX Drivers   
2.[turtlebot_arm](http://wiki.ros.org/turtlebot_arm) -- The turtlebot arm meta package.   
Note that the turtlebot_arm package is use for the white-green TurtleBot-arm (as shown below) by default, so it is required to modify some code such that the package can be used to control the PhantomX-Pincher arm.   
<center>![avatar](figures/arm_small.png)</center>     
<center>Fig 1. White-green Turtlebot-arm </center>   

In this project, we use a branch repository: [corot/turtlebot_arm](https://github.com/corot/turtlebot_arm)

### Arbotix firmware for ROS
If you have install the arduino IDE, and configured the arbotix library, you should skip this part. If not, you can follow the [familiar_with_pincher](https://github.com/ViolinLee/Pincher_projects/blob/master/0_familiar_with_pincher/familiar_with_pincher.md) in part 1, or visit the [pincher-arm official website (trossenrobotics)](https://learn.trossenrobotics.com/interbotix/robot-arms/pincher-arm). In addition, the library has a github repository: [arbotix](https://github.com/Interbotix/arbotix).


## Usage
Assume that you have install the arduino IDE and configure the library correctly and the required driver has been installed.    
### Step 1. Burn ros firmware into Arbotix-M


    File -> Sketchbook -> Arbotix Sketches -> ros   
    Tools -> Serial Port -> "The corresponding USB port"
    Tools -> Board -> Arbotix    


### Step 2. Install the Arbotix Package

    $ sudo apt-get update   
    $ sudo apt-get install ros-kinetic-arbotix


### Step 3. Confugure ros workspace
1.arbotix_ros

    $ cd ~/turtlebot_arm_ws/src/   
    $ git clone -b turtlebot2i https://github.com/Interbotix/arbotix_ros.git   
    $ cd ..   
    $ catkin_make   

2.turtlebot_arm

    $ cd ~/turtlebot_arm_ws/src   
    $ git clone https://github.com/corot/turtlebot_arm   
    $ cd ..   
    $ catkin_make   

### Step 4. Test servos
Then, you can check that you arm is effectively connected to your robot (or computer machine). Connect the PhantomX Pincher arm through USB port to your robot laptop and plug power cable to socket. make sure that you have read, write and execute access on the USP port using this command

    $ sudo chmod 777 /dev/ttyUSB0

Assuming that your arm is connected on port ttyUSB0 to the robot laptop. Then, execute the following command:

    $ arbotix_terminal

You should see the following:

    ArbotiX Terminal --- Version 0.1
    Copyright 2011 Vanadium Labs LLC
    >>

You can check that your servos are all active using the `ls` command

    ArbotiX Terminal --- Version 0.1   
    Copyright 2011 Vanadium Labs LLC   
    >>  ls   
    1    2    3    4    5 .... .... .... ....   
    .... .... .... .... .... .... .... .... ....   
   

### Step 5. Test and Develop the Arm PhantomX Pincher robot
Now, you want to move the robot arm. This can be done through the arbotix_gui interface, but some prior work is needed beforehand. First, you need to describe the turtlebot arm in a yaml description file. Here is a working description of the PhatomX Pincher arm, assuming that the arm is connected through port `ttyUSB0`

    source: pincher_arm.yaml
    port: /dev/ttyUSB0
    read_rate: 15
    write_rate: 25
    joints: {
        arm_shoulder_pan_joint: {id: 1,  neutral: 512, max_angle: 140, min_angle: -140, max_speed: 90, type: dynamixel},
        arm_shoulder_lift_joint: {id: 2, max_angle: 126, min_angle: -119, max_speed: 90, type: dynamixel},
        arm_elbow_flex_joint: {id: 3, max_angle: 136, min_angle: -139, max_speed: 90, type: dynamixel},
        arm_wrist_flex_joint: {id: 4, max_angle: 96, min_angle: -98, max_speed: 90, type: dynamixel},
        gripper_joint: {id: 5, max_angle: 0, min_angle: -145, max_speed: 90, type: prismatic, radius: .0078, connector: .024, offset: .016}
    }
    controllers: {
        arm_controller: {type: follow_controller, joints: [arm_shoulder_pan_joint, arm_shoulder_lift_joint, arm_elbow_flex_joint, arm_wrist_flex_joint], action_name: arm_controller/follow_joint_trajectory, onboard: False }
    }

As it can be observed, five joints are defined, each joint related to one servo. For example, `servo 1` is defined as `arm_shoulder_pan_joint`. Then, controllers are defined. All joints are of follow_controller type. Consult [arbotix wiki page](http://wiki.ros.org/arbotix) for more details about controllers types and other information.

The above information is defined in the file called `pincher_arm.yaml` and is located in the `turtlebot_arm_bringup` package.

Now, we need to modify the launch file to run the arbotix_driver for the PhantomX Pincher arm and it is called `arm.launch` which is located in `turtlebot_arm_bringup/launch`:

    <launch>
      <!-- To use, first set Environment variable TURTLEBOT_ARM1 to either:
       turtlebot or pincher (for Trossen PhantomX Pincher)
       NOTE: passing arm_type as argument NOT yet fully supported! -->
    
      <arg name="simulation" default="false"/>
      <arg name="arm_type" default="$(optenv TURTLEBOT_ARM1 pincher)"/>
    
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find turtlebot_arm_description)/urdf/$(arg arm_type)_arm.urdf.xacro'"/>
      <node name="robot_state_pub" pkg="robot_state_publisher" type="robot_state_publisher"/>
      <node name="fake_joint_pub" pkg="turtlebot_arm_bringup" type="fake_joint_pub.py"/>
    
      <node name="arbotix" pkg="arbotix_python" type="arbotix_driver" output="screen">
    <rosparam file="$(find turtlebot_arm_bringup)/config/$(arg arm_type)_arm.yaml" command="load"/>
    <param name="sim" value="$(arg simulation)"/>
      </node>
    
      <node name="gripper_controller" pkg="arbotix_controllers" type="gripper_controller" output="screen">
    <rosparam file="$(find turtlebot_arm_bringup)/config/$(arg arm_type)_gripper.yaml" command="load" />
      </node>
    </launch>

In order to use PhantomX Pincher, we should first set Environment variable TURTLEBOT_ARM1 to `pincher`.   

This launch file will start the node `arbotix_driver` located in the arbotix_python and will take as argument the yaml description file of the PhantomX Pincher arm named pincher_arm.yaml. You can now start up the driver as follows:

    roslaunch turtlebot_arm_bringup arm.launch

You should see the following output: 
    
    robond@udacity:~/catkin_ws$ roslaunch turtlebot_arm_bringup arm.launch
    ... logging to /home/robond/.ros/log/a05cbcf4-3419-11e9-96a5-000c29f18e57/roslaunch-udacity-13764.log
    Checking log directory for disk usage. This may take awhile.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.
    
    started roslaunch server http://192.168.19.132:36213/
    
    SUMMARY
    ========
    
    PARAMETERS
     * /arbotix/controllers/arm_controller/action_name: arm_controller/fo...
     * /arbotix/controllers/arm_controller/joints: ['arm_shoulder_pa...
     * /arbotix/controllers/arm_controller/onboard: False
     * /arbotix/controllers/arm_controller/type: follow_controller
     * /arbotix/joints/arm_elbow_flex_joint/id: 3
     * /arbotix/joints/arm_elbow_flex_joint/max_angle: 136
     * /arbotix/joints/arm_elbow_flex_joint/max_speed: 90
     * /arbotix/joints/arm_elbow_flex_joint/min_angle: -139
     * /arbotix/joints/arm_elbow_flex_joint/type: dynamixel
     * /arbotix/joints/arm_shoulder_lift_joint/id: 2
     * /arbotix/joints/arm_shoulder_lift_joint/max_angle: 126
     * /arbotix/joints/arm_shoulder_lift_joint/max_speed: 90
     * /arbotix/joints/arm_shoulder_lift_joint/min_angle: -119
     * /arbotix/joints/arm_shoulder_lift_joint/type: dynamixel
     * /arbotix/joints/arm_shoulder_pan_joint/id: 1
     * /arbotix/joints/arm_shoulder_pan_joint/max_angle: 140
     * /arbotix/joints/arm_shoulder_pan_joint/max_speed: 90
     * /arbotix/joints/arm_shoulder_pan_joint/min_angle: -140
     * /arbotix/joints/arm_shoulder_pan_joint/neutral: 512
     * /arbotix/joints/arm_shoulder_pan_joint/type: dynamixel
     * /arbotix/joints/arm_wrist_flex_joint/id: 4
     * /arbotix/joints/arm_wrist_flex_joint/max_angle: 96
     * /arbotix/joints/arm_wrist_flex_joint/max_speed: 90
     * /arbotix/joints/arm_wrist_flex_joint/min_angle: -98
     * /arbotix/joints/arm_wrist_flex_joint/type: dynamixel
     * /arbotix/joints/gripper_joint/connector: 0.024
     * /arbotix/joints/gripper_joint/id: 5
     * /arbotix/joints/gripper_joint/max_angle: 0
     * /arbotix/joints/gripper_joint/max_speed: 90
     * /arbotix/joints/gripper_joint/min_angle: -145
     * /arbotix/joints/gripper_joint/offset: 0.016
     * /arbotix/joints/gripper_joint/radius: 0.0078
     * /arbotix/joints/gripper_joint/type: prismatic
     * /arbotix/port: /dev/ttyUSB0
     * /arbotix/read_rate: 15
     * /arbotix/sim: False
     * /arbotix/source: pincher_arm.yaml
     * /arbotix/write_rate: 25
     * /gripper_controller/center: 0
     * /gripper_controller/invert: False
     * /gripper_controller/joint: gripper_joint
     * /gripper_controller/max_opening: 0.031
     * /gripper_controller/min_opening: 0.002
     * /gripper_controller/model: parallel
     * /gripper_controller/neutral: 0.015
     * /gripper_controller/pad_width: 0.002
     * /gripper_controller/source: pincher_gripper.yaml
     * /gripper_controller/tighten: 0.001
     * /robot_description: <?xml version="1....
     * /rosdistro: kinetic
     * /rosversion: 1.12.14
    
    NODES
      /
    arbotix (arbotix_python/arbotix_driver)
    fake_joint_pub (turtlebot_arm_bringup/fake_joint_pub.py)
    gripper_controller (arbotix_controllers/gripper_controller)
    robot_state_pub (robot_state_publisher/robot_state_publisher)
    
    auto-starting new master
    process[master]: started with pid [13778]
    ROS_MASTER_URI=http://localhost:11311
    
    setting /run_id to a05cbcf4-3419-11e9-96a5-000c29f18e57
    process[rosout-1]: started with pid [13791]
    started core service [/rosout]
    process[robot_state_pub-2]: started with pid [13801]
    process[fake_joint_pub-3]: started with pid [13809]
    process[arbotix-4]: started with pid [13810]
    process[gripper_controller-5]: started with pid [13811]
    [INFO] [1550562030.129105]: Started parallel Gripper Controller.
    [INFO] [1550562031.824252]: Started ArbotiX connection on port /dev/ttyUSB0.
    [INFO] [1550562031.836726]: Started Servo 3  arm_elbow_flex_joint
    [INFO] [1550562031.849249]: Started Servo 4  arm_wrist_flex_joint
    [INFO] [1550562031.862975]: Started Servo 2  arm_shoulder_lift_joint
    [INFO] [1550562031.874851]: Started Servo 1  arm_shoulder_pan_joint
    [INFO] [1550562031.888276]: Started Servo 5  gripper_joint
    [INFO] [1550562031.895136]: gripper_joint prismatic joint
    [INFO] [1550562031.912074]: Started FollowController (arm_controller). Joints: ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint'] on C1
    [INFO] [1550562031.914414]: ArbotiX connected.

This means that your arm is successfully connected and ready to receive motion command. The easiest way to control the servos is using the arbotix_gui command:

    arbotix_gui

Then, you should see the following interface of the Arbotix Simulator: 
<center>![arbotix](figures/arbotix_gui.png)</center>    
You can now control the joints of the pincher arm by draging the sliders. Check this [vedio](https://www.youtube.com/watch?v=4jrBzh3I_xI&t=4s).   


### Step 6. Pincher Aram Motion Planning using MoveIt   
If you use the turtlebot\_arm package in this repository, you need not to modify anything. But if you use other turtlebot\_arm package, you may need to modify some code to make moveit work! The most obvious one is to change the arg 
First, Start up the arbotix driver:

    roslaunch turtlebot_arm_bringup arm.launch --screen
    
Then, launch moveit:   

    roslaunch turtlebot_arm_moveit_config turtlebot_arm_moveit.launch sim:=false --screen

Up to now, you can do motion planning for pincher arm. Check this vedio.   

Note: We can directly control the every joints of the pincher arm by sending relative message to specific topics. Run the command `rostopic list` to check out the topics: 



We can see many interesting topic, among which some can be use to control the joints. For example:    

    rostopic pub /gripper_joint/command std_msgs/Float64 1.0 -1
    rostopic pub /arm_shoulder_lift_joint/command std_msgs/Float64 1.0 -1
    rostopic pub /arm_shoulder_pan_joint/command std_msgs/Float64 1.0 -1
    rostopic pub /arm_elbow_flex_joint/command std_msgs/Float64 1.0 -1
    rostopic pub /arm_wrist_flex_joint/command std_msgs/Float64 1.0 -1



## Moveit in simulation mode (Inverse kinematics using moveit test)
### Step 1. Preparation
Copy moveit\_ikine.py to "turtlenot_arm_moveit_config/scripts" directory.

### Step 2. Run the following command

    chmod +x moveit_ikine.py

In "turtlenot_arm_moveit_config/scripts" directory.

    roslaunch turtlebot_arm_bringup arm.launch
    roslaunch turtlebot_arm_moveit_config demo.launch
    rosrun turtlebot_arm_moveit_config moveit_ikine.py












