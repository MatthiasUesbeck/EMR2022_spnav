# EMR 2022
# Authors: Dominik Kuesters, Jens Ludwig, Matthias Uesbeck
# Desc:    Control a 6-DOF robot in moveit via a spacenavigator 3d mouse
# This script is based on a script by Ryohei Ueda and Dave Coleman.
# Read their initial comment below:


# ********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, JSK, The University of Tokyo.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the JSK, The University of Tokyo nor the names of its
#     nor the names of its contributors may be
#     used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ********************************************************************/

#   Author: Ryohei Ueda, Dave Coleman
#   Desc:   Interface between PS3/XBox controller and MoveIt Motion Planning Rviz Plugin


# python standard libraries
import threading
import time 
import sys

# common 3rd party
import numpy

# 3rd party modified
from spnav import *

# pyqt
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt

# ros
import rospy
import tf
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarkerInit

# moveit
import moveit_msgs.msg
import moveit_msgs.srv
from moveit_ros_planning_interface._moveit_robot_interface import RobotInterface

# setup node
rospy.init_node('spacenavigator', anonymous=True)

def signedSquare(val):
    if val > 0:
        sign = 1
    else:
        sign = -1
    return val * val * sign

class Status():
    def __init__(self, x_trans, y_trans, z_trans, x_rot, y_rot, z_rot, csys):
        self.x_trans = x_trans
        self.y_trans = y_trans
        self.z_trans = z_trans
        self.x_rot = x_rot
        self.y_rot = y_rot
        self.z_rot = z_rot
        self.csys = csys

class MyUI(QWidget):
    def __init__(self):
        super(MyUI, self).__init__()
        self.app_running = True
        self.initial_poses = {}
        self.planning_groups_tips = {}
        self.tf_listener = tf.TransformListener()
        self.marker_lock = threading.Lock()
        self.prev_time = rospy.Time.now()
        self.counter = 0
        #self.history = StatusHistory(max_length=10)
        self.pre_pose = PoseStamped()
        self.pre_pose.pose.orientation.w = 1
        self.current_planning_group_index = 0
        self.current_eef_index = 0
        self.initialize_poses = False
        self.initialized = False
        self.parseSRDF()
        self.plan_group_pub = rospy.Publisher("/rviz/moveit/select_planning_group", String, queue_size=5)
        self.act_planning_group = self.updatePlanningGroup(0)
        self.updatePoseTopic(0, False)
        self.joy_pose_pub = rospy.Publisher("/joy_pose", PoseStamped, queue_size=1)
        self.plan_pub = rospy.Publisher("/rviz/moveit/plan", Empty, queue_size=5)
        self.execute_pub = rospy.Publisher("/rviz/moveit/execute", Empty, queue_size=5)
        self.update_start_state_pub = rospy.Publisher("/rviz/moveit/update_start_state", Empty, queue_size=5)
        self.update_goal_state_pub = rospy.Publisher("/rviz/moveit/update_goal_state", Empty, queue_size=5)
        self.interactive_marker_sub = rospy.Subscriber(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full",
            InteractiveMarkerInit,
            self.markerCB,
            queue_size=1,)         
        
        self.modes = {"moveit": "MoveIt MotionPlanner", "direct": "Direct control"}
        self.csys = {"tcp": "TCP", "world": "World"}
        self.axes = {"x_trans": False, 
                     "y_trans": False,
                     "z_trans": False, 
                     "x_rot": False,                       
                     "y_rot": False,                      
                     "z_rot": False}
        
        self.layout = QVBoxLayout()
        
        # mode
        hbox = QHBoxLayout()
        hbox.addWidget(QLabel("Mode:"))
        self.cb_mode = QComboBox()
        self.cb_mode.addItems(self.modes.values())
        self.cb_mode.currentIndexChanged.connect(self.mode_changed)
        hbox.addWidget(self.cb_mode)        
        self.layout.addLayout(hbox)   
        
        # Csys
        hbox = QHBoxLayout()
        hbox.addWidget(QLabel("Coordinate system:"))
        self.cb_csys = QComboBox()
        self.cb_csys.addItems(self.csys.values())
        self.cb_csys.currentIndexChanged.connect(self.csys_changed)
        hbox.addWidget(self.cb_csys)        
        self.layout.addLayout(hbox)   
        
        # Sensitivity
        hbox = QHBoxLayout()
        hbox.addWidget(QLabel("Sensitivity:"))
        self.slider_sensitivity = QSlider(Qt.Horizontal)
        self.slider_sensitivity.setMinimum(1)
        self.slider_sensitivity.setMaximum(4)
        self.slider_sensitivity.setValue(2)
        hbox.addWidget(self.slider_sensitivity)        
        self.layout.addLayout(hbox) 
        
        # invert axis
        self.cb_swap_xy_rot = QCheckBox("swap x and y rotation")
        self.layout.addWidget(self.cb_swap_xy_rot)
        hbox = QHBoxLayout()
        vbox = QVBoxLayout()
        for axis in self.axes.keys():  
            cb = QCheckBox("invert " + axis)
            f = lambda state, _axis=axis: self.actualize_axis(state, _axis)
            cb.stateChanged.connect(f)
            vbox.addWidget(cb)  
        hbox.addLayout(vbox)
        
        # add space
        hbox.addWidget(QLabel("                  "))
        hbox.addWidget(QLabel("                  "))
        
        # show position, label column
        self.lbl_x_trans_description = QLabel("X    : ")
        self.lbl_y_trans_description = QLabel("Y    : ")
        self.lbl_z_trans_description = QLabel("Z    : ")
        self.lbl_x_rot_description   = QLabel("X_rot: ")
        self.lbl_y_rot_description   = QLabel("Y_rot: ")
        self.lbl_z_rot_description   = QLabel("Z_rot: ")
        vbox = QVBoxLayout()
        vbox.addWidget(self.lbl_x_trans_description)  
        vbox.addWidget(self.lbl_y_trans_description)  
        vbox.addWidget(self.lbl_z_trans_description)  
        vbox.addWidget(self.lbl_x_rot_description)  
        vbox.addWidget(self.lbl_y_rot_description)  
        vbox.addWidget(self.lbl_z_rot_description)  
        hbox.addLayout(vbox)
        
        # show position
        self.lbl_x_trans = QLabel("0.000")
        self.lbl_y_trans = QLabel("0.000")
        self.lbl_z_trans = QLabel("0.000")
        self.lbl_x_rot   = QLabel("0.000")
        self.lbl_y_rot   = QLabel("0.000")
        self.lbl_z_rot   = QLabel("0.000")
        vbox = QVBoxLayout()
        vbox.addWidget(self.lbl_x_trans)  
        vbox.addWidget(self.lbl_y_trans)  
        vbox.addWidget(self.lbl_z_trans)  
        vbox.addWidget(self.lbl_x_rot)  
        vbox.addWidget(self.lbl_y_rot)  
        vbox.addWidget(self.lbl_z_rot) 
        hbox.addLayout(vbox)
        
        self.layout.addLayout(hbox)   
        self.setLayout(self.layout)

        self.setGeometry(300, 300, 500, 150)
        self.setWindowTitle("EMR22 - SpaceNavigator Robot Control")
        self.show()
        self.initialze_planning_group()
        
        self.spnav_thread = threading.Thread(target=self.spacenav_thread) 
        self.spnav_thread.start()
        
    def parseSRDF(self):
        ri = RobotInterface("/robot_description")
        planning_groups = {}
        for g in ri.get_group_names():
            self.planning_groups_tips[g] = ri.get_group_joint_tips(g)
            if len(self.planning_groups_tips[g]) > 0:
                planning_groups[g] = [
                    "/rviz/moveit/move_marker/goal_" + l
                    for l in self.planning_groups_tips[g]
                ]
        for name in planning_groups.keys():
            print(name, planning_groups[name])
        self.planning_groups = planning_groups
        self.planning_groups_keys = list(
            planning_groups.keys()
        )  # we'd like to store the 'order'
        self.frame_id = ri.get_planning_frame()
        
    def updatePlanningGroup(self, next_index):
        if next_index >= len(self.planning_groups_keys):
            self.current_planning_group_index = 0
        elif next_index < 0:
            self.current_planning_group_index = len(self.planning_groups_keys) - 1
        else:
            self.current_planning_group_index = next_index
        next_planning_group = None
        try:
            next_planning_group = self.planning_groups_keys[
                self.current_planning_group_index
            ]
        except IndexError:
            msg = "Check if you started movegroups. Exiting."
            rospy.logfatal(msg)
            raise rospy.ROSInitException(msg)
        rospy.loginfo("Changed planning group to " + next_planning_group)
        self.plan_group_pub.publish(next_planning_group)
        return next_planning_group
        
    def updatePoseTopic(self, next_index, wait=True):
        planning_group = self.planning_groups_keys[self.current_planning_group_index]
        topics = self.planning_groups[planning_group]
        if next_index >= len(topics):
            self.current_eef_index = 0
        elif next_index < 0:
            self.current_eef_index = len(topics) - 1
        else:
            self.current_eef_index = next_index
        next_topic = topics[self.current_eef_index]

        rospy.loginfo(
            "Changed controlled end effector to "
            + self.planning_groups_tips[planning_group][self.current_eef_index]
        )
        self.pose_pub = rospy.Publisher(next_topic, PoseStamped, queue_size=5)
        if wait:
            self.waitForInitialPose(next_topic)
        self.current_pose_topic = next_topic
        
    def markerCB(self, msg):
        try:
            self.marker_lock.acquire()
            if not self.initialize_poses:
                return
            self.initial_poses = {}
            for marker in msg.markers:
                if marker.name.startswith("EE:goal_"):
                    # resolve tf
                    if marker.header.frame_id != self.frame_id:
                        ps = PoseStamped(header=marker.header, pose=marker.pose)
                        try:
                            transformed_pose = self.tf_listener.transformPose(
                                self.frame_id, ps
                            )
                            self.initial_poses[marker.name[3:]] = transformed_pose.pose
                        except (
                            tf.LookupException,
                            tf.ConnectivityException,
                            tf.ExtrapolationException,
                            e,
                        ):
                            rospy.logerr("tf error when resolving tf: %s" % e)
                    else:
                        self.initial_poses[
                            marker.name[3:]
                        ] = marker.pose  # tf should be resolved
        finally:
            self.marker_lock.release()
            
    def waitForInitialPose(self, next_topic, timeout=None):
        counter = 0
        while not rospy.is_shutdown():
            counter = counter + 1
            if timeout and counter >= timeout:
                return False
            try:
                self.marker_lock.acquire()
                self.initialize_poses = True
                topic_suffix = next_topic.split("/")[-1]
                if topic_suffix in self.initial_poses:
                    self.pre_pose = PoseStamped(pose=self.initial_poses[topic_suffix])
                    self.update_pose_display(self.pre_pose)
                    self.initialize_poses = False
                    return True
                else:
                    rospy.logdebug(self.initial_poses.keys())
                    rospy.loginfo("Waiting for pose topic of '%s' to be initialized", topic_suffix)
                    rospy.sleep(1)
            finally:
                self.marker_lock.release()
                
    def computePoseFromSpaceNavigatorJoy(self, pre_pose, status: Status) -> PoseStamped:
        new_pose = PoseStamped()
        new_pose.header.frame_id = self.frame_id
        new_pose.header.stamp = rospy.Time(0.0)

        # translation
        scale = 1000
        x_diff = signedSquare(status.x_trans) / scale
        y_diff = signedSquare(status.y_trans) / scale    
        z_diff = signedSquare(status.z_trans) / scale 
         
        # rotation
        scale_rot = 400
        roll  = signedSquare(status.x_rot) / scale_rot
        pitch = signedSquare(status.y_rot) / scale_rot    
        yaw   = signedSquare(status.z_rot) / scale_rot 
        
        q = numpy.array((pre_pose.pose.orientation.x,
                         pre_pose.pose.orientation.y,
                         pre_pose.pose.orientation.z,
                         pre_pose.pose.orientation.w,))
        if status.csys == 0:      # tcp
            # translation
            local_move = numpy.array((x_diff, y_diff, z_diff, 1.0 ))            
            xyz_move = numpy.dot(tf.transformations.quaternion_matrix(q), local_move)
            new_pose.pose.position.x = numpy.clip(pre_pose.pose.position.x + xyz_move[0], -1, 1)
            new_pose.pose.position.y = numpy.clip(pre_pose.pose.position.y + xyz_move[1], -1, 1)
            new_pose.pose.position.z = numpy.clip(pre_pose.pose.position.z + xyz_move[2], -1, 1)
            
            # rotation
            diff_q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            new_q = tf.transformations.quaternion_multiply(q, diff_q)
            new_pose.pose.orientation.x = new_q[0]
            new_pose.pose.orientation.y = new_q[1]
            new_pose.pose.orientation.z = new_q[2]
            new_pose.pose.orientation.w = new_q[3]        
        elif status.csys == 1:      # world  
           # translation
            new_pose.pose.position.x = numpy.clip(pre_pose.pose.position.x + x_diff, -1, 1)
            new_pose.pose.position.y = numpy.clip(pre_pose.pose.position.y + y_diff, -1, 1)
            new_pose.pose.position.z = numpy.clip(pre_pose.pose.position.z + z_diff, -1, 1)
        
            # rotation        
            old_roll, old_pitch, old_yaw = tf.transformations.euler_from_quaternion(q, axes='sxyz')
            new_q = tf.transformations.quaternion_from_euler(roll+ old_roll, pitch + old_pitch, yaw + old_yaw, axes='sxyz')
            new_pose.pose.orientation.x = new_q[0]
            new_pose.pose.orientation.y = new_q[1]
            new_pose.pose.orientation.z = new_q[2]
            new_pose.pose.orientation.w = new_q[3]
        return new_pose
        
    def mode_changed(self):
        print('mode changed')
        
    def csys_changed(self):
        print('csys changed')
        
    def actualize_axis(self, state, axis):
        self.axes[axis] = True if state == 2 else False
        
    def initialze_planning_group(self):
        # when not initialized, we will force to change planning_group
        while True:
            self.updatePlanningGroup(self.current_planning_group_index)
            planning_group = self.planning_groups_keys[
                self.current_planning_group_index
            ]
            topics = self.planning_groups[planning_group]
            next_topic = topics[self.current_eef_index]
            if not self.waitForInitialPose(next_topic, timeout=3):
                rospy.logwarn("Unable to initialize planning group "+ planning_group + ". Trying different group.")
                rospy.logwarn("Is 'Allow External Comm.' enabled in Rviz? Is the 'Query Goal State' robot enabled?")
            else:
                rospy.loginfo("Initialized planning group")
                self.initialized = True
                self.updatePoseTopic(self.current_eef_index)
                break
            # Try to initialize with different planning group
            self.current_planning_group_index += 1
            if self.current_planning_group_index >= len(self.planning_groups_keys):
                self.current_planning_group_index = 0  # reset loop  
        
    def spacenav_thread(self):
        try:
            spnav_open()           
            while self.app_running:   # condition to keep this thread running -> used to exit this thread                
                event = spnav_wait_event()                     
                if event != None:                    
                    if not self.initialized:
                        self.initialze_planning_group()  
                    self.marker_lock.acquire()      
                    if type(event) == SpnavMotionEvent: # one of the 6 DOF changed
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
                        x_trans = (-1 if self.axes["x_trans"] else 1) * float(event.translation[0])/350/2*self.slider_sensitivity.value()
                        y_trans = (-1 if self.axes["y_trans"] else 1) * float(event.translation[2])/350/2*self.slider_sensitivity.value() 
                        z_trans = (-1 if self.axes["z_trans"] else 1) * float(event.translation[1])/350/2*self.slider_sensitivity.value()                          
                        x_rot   = (-1 if self.axes["x_rot"] else 1) * float(event.rotation[0])/350/2*self.slider_sensitivity.value()
                        y_rot   = (-1 if self.axes["y_rot"] else 1) * float(event.rotation[2])/350/2*self.slider_sensitivity.value()
                        z_rot   = (-1 if self.axes["z_rot"] else 1) * float(event.rotation[1])/350/2*self.slider_sensitivity.value()                            
                        if self.cb_swap_xy_rot.isChecked:
                            buff = x_rot
                            x_rot = y_rot
                            y_rot = buff      
                        # set csys
                        if self.cb_csys.currentText() == self.csys["tcp"]:
                            csys = 0
                        elif self.cb_csys.currentText() == self.csys["world"]:
                            csys = 1
                        else:
                            csys = -1            
                        status = Status(x_trans, y_trans, z_trans, x_rot, y_rot, z_rot, csys)
                        
                        if self.cb_mode.currentText() == self.modes["moveit"]:  
                            new_pose = self.computePoseFromSpaceNavigatorJoy(self.pre_pose, status)
                            now = rospy.Time.from_sec(time.time())
                            # placement.time_from_start = now - self.prev_time
                            if (now - self.prev_time).to_sec() > 1 / 30.0:
                                # rospy.loginfo(new_pose)
                                self.pose_pub.publish(new_pose)
                                self.joy_pose_pub.publish(new_pose)
                                self.prev_time = now
                            # sync start state to the real robot state
                            self.counter = self.counter + 1
                            self.pre_pose = new_pose
                            self.update_pose_display(new_pose)                            
                            # update self.initial_poses                            
                            self.initial_poses[self.current_pose_topic.split("/")[-1]] = new_pose.pose 
                        elif self.cb_mode.currentText() == self.modes["direct"]:
                            # not implemented yet
                            pass  
                             
                    if type(event) == SpnavButtonEvent:
                        if self.cb_mode.currentText() == self.modes["moveit"]:  
                            if event.bnum == 0 and event.press == 1:# [0] button left, [1] button right  
                                rospy.loginfo("Plan")
                                self.plan_pub.publish(Empty())               
                            elif event.bnum == 1 and event.press == 1:
                                rospy.loginfo("Execute")
                                self.execute_pub.publish(Empty()) 
                        elif self.cb_mode.currentText() == self.modes["direct"]:
                            # not implemented yet
                            pass  
                    self.marker_lock.release()   
        except Exception as e:
            print(e)
        spnav_close()
        
    def update_pose_display(self, new_pose):
        self.lbl_x_trans.setText("{:.3f}".format(float(new_pose.pose.position.x)))
        self.lbl_y_trans.setText("{:.3f}".format(float(new_pose.pose.position.y)))
        self.lbl_z_trans.setText("{:.3f}".format(float(new_pose.pose.position.z)))
        self.lbl_x_rot.setText("{:.3f}".format(float(new_pose.pose.orientation.x)))
        self.lbl_y_rot.setText("{:.3f}".format(float(new_pose.pose.orientation.y)))
        self.lbl_z_rot.setText("{:.3f}".format(float(new_pose.pose.orientation.z)))
        
    def on_exit(self):
        self.app_running = False 
        self.spnav_thread.join()  

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        ui = MyUI()
        #pub_thread = threading.Thread(target=ui.spacenav_thread) 
        #pub_thread.start()
        app.aboutToQuit.connect(ui.on_exit)       
        app.exec_()
        #pub_thread.join()
    except rospy.ROSInterruptException:
        pass
    sys.exit()