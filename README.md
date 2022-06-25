# EMR2022_spnav
Control robot in ROS/ MoveIt via 3dConnexion SpaceNavigator 3d-mouse.

# Installation #
* Install all necessary libraries via the install.sh batch script:
  ```console
  foo@bar:~$ ./install.sh
  ```
# Control moveit motion planning via 3d mouse #
  * start robot 
    * UR 5:  
      ```console
      foo@bar:~$ ./start_ur5_gazebo.sh
      ```  
      ```console
      foo@bar:~$ ./start_ur5_moveit.sh
      ```  
      ```console
      foo@bar:~$ ./start_ur5_rviz.sh
      ```  
    * panda:
      ```console
      foo@bar:~$ ./start_panda.sh
      ```
      In Rviz add MotionPlanning plugin and enalbe external control.
  * start GUI:
    ```console
    foo@bar:~$ python3 main.py
    ```
  
# Control the joints of the panda robot via the 3d mouse #
  * Joint control is only implemented for the panda robot
  * start robot (see above)
  * start script:
    ```console
    foo@bar:~$ python3 main_joints.py
    ```
