#!/usr/bin/env python

"""
    moveit_ik.py - Version 0.1 2019-04-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
"""

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_ik')
                
        # Initialize the move group for the right arm
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
        print end_effector_link
                        
        # Set the reference frame for pose targets
        reference_frame = '/base_link'
        
        # Set the arm reference frame accordingly
        arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.002)
        arm.set_goal_orientation_tolerance(3.14)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        #arm.set_named_target('resting')
        #arm.go()
        #rospy.sleep(2)

        # Test forward kinematics
        joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(joint_positions)
        traj = arm.plan()
        arm.execute(traj)
        
        # LeeChan
        tpose = arm.get_current_pose()
        print type(tpose)
        print tpose

               
        # Set the target pose.  This particular pose has the gripper oriented horizontally
        # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
        # the center of the robot base.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.25515
        target_pose.pose.position.y = -0.255147
        target_pose.pose.position.z = 0.510048
        #target_pose.pose.orientation.x = 0.034
        #target_pose.pose.orientation.y = 0.673
        #target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1
        print type(target_pose)
        print target_pose
        
        # Set the start state to the current state
        arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        arm.set_pose_target(target_pose, end_effector_link)
        
        # Plan the trajectory to the goal
        traj = arm.plan()
        
        # Execute the planned trajectory
        arm.execute(traj)
    
        # Pause for a second
        rospy.sleep(1)
           
        ###############################################
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.25305
        target_pose.pose.position.y = -0.1963
        target_pose.pose.position.z = 0.6633
        #target_pose.pose.orientation.x = 0.034
        #target_pose.pose.orientation.y = 0.673
        #target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1
        print type(target_pose)
        print target_pose
        
        arm.set_start_state_to_current_state()
        
        arm.set_pose_target(target_pose, end_effector_link)
        
        traj = arm.plan()
        
        arm.execute(traj)
    
        rospy.sleep(1)

        #################################################

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItDemo()

    
    
