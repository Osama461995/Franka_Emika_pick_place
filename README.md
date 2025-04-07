# Franka_Emika_pick_place

## 📤 How to Push Local Changes Using CMD

Follow these steps to push your local changes to the remote Git repository:

- Open **Command Prompt** and navigate to your project directory:
  ```bash
  cd path\to\your\osama


Pick and Place Sorting System using Franka Emika and ROS


This project uses a Franka Emika robot arm equipped with an RGB camera to detect, pick, and place objects based on their color (red, green, blue, and yellow). The system uses ROS Noetic and MoveIt for motion planning and object manipulation.

----------------------------
Installation & Setup Steps
----------------------------

1. **Install ROS Noetic**
   Follow the official ROS Noetic installation guide for your OS (Ubuntu 20.04 is recommended):
   http://wiki.ros.org/noetic/Installation/Ubuntu

2. **Create your own ROS Workspace**
   Open a terminal and create a workspace (you can name it whatever you like, here we use `catkin_ws` as an example):

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash

