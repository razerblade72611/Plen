# Troubleshooting Guide for PLEN2 Gazebo Simulation

## **1. Gazebo Simulation Won’t Start**
### **Issue:** Nothing appears when running `ros2 launch plen2_gazebo gazebo.launch.py`
**Solution:**
- Ensure ROS2 environment is sourced:
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
- Check if Gazebo is installed:
  ```bash
  gazebo --version
  ```
  If Gazebo is missing, install it:
  ```bash
  sudo apt install gazebo11 ros-iron-gazebo-ros-pkgs
  ```

## **2. Teleoperation (Keyboard Control) Not Working**
### **Issue:** `WASD` keys do not move the robot.
**Solution:**
- Ensure `teleop_twist_keyboard` is installed:
  ```bash
  sudo apt install ros-iron-teleop-twist-keyboard
  ```
- Run teleop with the correct launch command:
  ```bash
  ros2 launch plen2_gazebo teleop.launch.py
  ```

## **3. Joint Movement Not Responding**
### **Issue:** The robot does not move when sending joint position commands.
**Solution:**
- Verify that the controller is running:
  ```bash
  ros2 control list_controllers
  ```
  Expected output:
  ```
  joint_state_broadcaster - active
  plen2_controller - active
  ```
- If the controller is not active, restart it:
  ```bash
  ros2 control load_controller plen2_controller
  ros2 control switch_controller --activate plen2_controller
  ```

## **4. URDF or Mesh Files Not Found**
### **Issue:** Gazebo cannot locate URDF or STL files.
**Solution:**
- Ensure the package is built correctly:
  ```bash
  cd ~/ros2_ws
  colcon build --symlink-install
  ```
- Source the workspace:
  ```bash
  source install/setup.bash
  ```
- Check file paths inside `plen2.urdf.xacro` and `plen2.gazebo.xacro`.

## **5. Gazebo Simulation Lagging or Crashing**
### **Issue:** Slow performance or Gazebo crashes.
**Solution:**
- Reduce physics update rate:
  Edit `plen2.gazebo.xacro` and lower `<update_rate>` value.
- Use a lighter Gazebo world:
  ```bash
  ros2 launch plen2_gazebo gazebo.launch.py world:=empty.world
  ```

## **6. ROS2 Nodes Not Found**
### **Issue:** Errors like `package not found` when running launch files.
**Solution:**
- Check if ROS2 Iron is installed and sourced:
  ```bash
  source /opt/ros/iron/setup.bash
  ```
- Confirm package exists:
  ```bash
  ros2 pkg list | grep plen2_gazebo
  ```

If issues persist, try restarting your system and repeating the steps. 🚀


