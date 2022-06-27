#!/usr/bin/env python3

from math import pi
import time 
import copy
import sys
import rospy
import sensor_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface

from spnav import *

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    # Instantiate the robot commander object,
    # which is used to control the whole robot
    robot = moveit_commander.RobotCommander()

    # Create an object for PlanningSceneInterface.
    scene = moveit_commander.PlanningSceneInterface()
    
    # Instantiate the MoveGroupCommander object.
    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    group_name_gripper = "hand"
    group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
    joint_goal = group.get_current_joint_values()
    state_valid_service = rospy.ServiceProxy('check_state_validity', moveit_msgs.srv.GetStateValidity)
    
    # go to start pose
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0
    group.go(joint_goal, wait=True)
    group.stop()
    
    prev_time = now = rospy.Time.from_sec(time.time())
    spnav_open() 
    while True:
        now = rospy.Time.from_sec(time.time())
        # limit loop to 30 Hz
        if (now - prev_time).to_sec() > 1 / 30.0:
            prev_time = now
            event = event = spnav_poll_event()   
            if event != None:                  
                if type(event) == SpnavMotionEvent: 
                    print('X')               
                    # translations:
                        # translation[0] x -left +right
                        # translation[2] y -backwards +forwards
                        # translation[1] z -down +up  
                        # left handed coordinate system!                      
                    # rotations
                        # rotation[0] x -tilt forwards +tilt backwards
                        # rotation[2] y -tilt left + tilt right
                        # rotation[1] z -tilt clockwise + tilt counterclockwise                        
                    # all values between -350 to +350  
                    scale =  8000 
                    temp_joint_goal = copy.deepcopy(joint_goal)
                    temp_joint_goal[0] += event.rotation[1] / scale         
                    temp_joint_goal[1] += -event.rotation[0] / scale        
                    temp_joint_goal[2] += -event.rotation[2] / scale        
                    temp_joint_goal[3] += event.translation[0] / scale      
                    temp_joint_goal[4] += event.translation[1] / scale      
                    temp_joint_goal[5] += event.translation[2] / scale      
                    
                    # checl if joint state is valid
                    temp_joint_state = sensor_msgs.msg.JointState()
                    temp_joint_state.position = temp_joint_goal
                    req = moveit_msgs.srv.GetStateValidityRequest()
                    req.group_name = group_name
                    req.robot_state = moveit_msgs.msg.RobotState()
                    req.robot_state.joint_state = temp_joint_state
                    res = state_valid_service(req)
                    if res.valid:                    
                        try:
                            group.go(temp_joint_goal, wait=False)
                            joint_goal = temp_joint_goal
                        except Exception as e:
                            pass
                    spnav_remove_events(SPNAV_EVENT_ANY) # clean pipeline