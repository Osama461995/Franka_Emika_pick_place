

#  Pick & Place Sorting System using Franka Emika Panda Robot


This project uses a Franka Emika robot arm equipped with an RGB camera to detect, pick, and place objects based on their color ( Green, Blue, Red). The system uses ROS Noetic and MoveIt for motion planning and object manipulation.


----------------------------
Installation & Setup Steps
----------------------------

1. **Install ROS Noetic**
   Follow the official ROS Noetic installation guide for your OS (Ubuntu 20.04 is recommended):
   http://wiki.ros.org/noetic/Installation/Ubuntu

2. **Create your own ROS Workspace**
   
   Open a terminal and create a workspace (you can name it whatever you like, here we use `catkin_ws` as an example):

   
            mkdir -p ~/catkin_ws/src
            cd ~/catkin_ws
            catkin_make
   
           source devel/setup.bash

4. **Install Moveit for ROS Noetic**

You can install MoveIt for ROS Noetic by following the official instructions from the MoveIt website:

🔗 Official Link:
https://moveit.picknik.ai/install/


4. **Add the Project Folder**
After MoveIt has been successfully downloaded into the src folder, copy the folder named moveit_test (which contains our main programs) into the same src directory.
note that you have to take the code from zip files for completing tasks.

So your folder structure should look like this:

    
              ~/catkin_ws/src/
               ├── moveit
               ├── moveit_test/
                 │   ├── scripts/realsense_image7.py
                  │   └── src/pick_place16.cpp

5. **Build moveit_test package**
   Now build moveit_test in your workspace using catkin:
    ```bash 
       cd ~/catkin_ws
       catkin build moveit_test

7. **Install Python Dependencies**
Make sure you have all required Python dependencies installed. You can install them using the provided requirements.txt file.
the list of requirement:

cv_bridge==1.16.2

numpy==1.21.0

opencv_python==4.10.0.84

pyrealsense2==2.55.1.6486

rospy==1.17.0

scikit_learn==1.3.2

scipy==1.3.3

sensor_msgs==1.13.1

tf2_geometry_msgs==0.7.7

tf2_ros==0.7.7

ultralytics==8.3.59

the Steps for installation requirements:

1- Open a terminal and navigate to the folder where requirements.txt is located.

2- Run the following command:

    
            pip install -r requirements.txt
#  Execution Steps

Before executing the system, you must **modify the XML launch file** `franka_control.launch`, which is located in the `panda_moveit_config` package. This step is necessary to include the static transform for the camera link.

Add the following line **within the `<launch>` tag** of the file:

        <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf_publisher"
               args="0.062225397 -0.062225397 0.033 0 0.785398163 0 panda_link8 camera_link" />

1.**Terminal 1 – Launch the Robot Controller**
         ```bash
         
           cd path/to/your/workspace
          source devel/setup.bash
          roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2

1.**Terminal 2 – Run the Object Detection Script**
       ```bash
       
          cd path/to/moveit_test/script
          python3 realsense_image7.py
          
This script detects objects based on color using the RGB camera and publishes their poses.

1.**Terminal 3 – Run the Pick & Place Program**
        ```bash
        
        cd path/to/your/workspace
        source devel/setup.bash

        rosrun moveit_test moveit_test_pick_place16.cpp

           
