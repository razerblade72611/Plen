# PLEN2 Gazebo Simulation Setup Guide

## Prerequisites
Ensure that you have **ROS 2 Iron** installed on your system. If not, follow the official installation guide: [ROS 2 Iron Installation](https://docs.ros.org/en/iron/Installation.html).

### **1. Install Dependencies**
```bash
sudo apt update
sudo apt install -y gazebo11 ros-iron-gazebo-ros-pkgs ros-iron-teleop-twist-keyboard
```

### **2. Set Up the Workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### **3. Clone the PLEN2 Simulation Repository**
```bash
git clone https://github.com/YOUR_USERNAME/plen2_gazebo.git
```

### **4. Build the Workspace**
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### **5. Launch the Simulation**
To start the **Gazebo simulation** with PLEN2:
```bash
ros2 launch plen2_gazebo gazebo.launch.py
```

### **6. Enable Teleoperation**
To control PLEN2 manually using the keyboard:
```bash
ros2 launch plen2_gazebo teleop.launch.py
```
- `W` → Move Forward  
- `S` → Move Backward  
- `A` → Turn Left  
- `D` → Turn Right  
- `J` → Increase Left Shoulder Joint Angle  
- `K` → Decrease Left Shoulder Joint Angle  
- `L` → Increase Right Shoulder Joint Angle  
- `Q` → Quit Teleop Mode  

### **7. Testing Joint Control**
To send joint position commands manually:
```bash
ros2 topic pub /joint_states sensor_msgs/JointState '{name: ["left_shoulder_pitch"], position: [0.5]}'
```

### **8. Troubleshooting**
- If **Gazebo doesn’t launch**, check your ROS2 environment:
  ```bash
  source ~/ros2_ws/install/setup.bash
  ```
- If **teleop keys don’t work**, ensure `teleop_twist_keyboard` is installed:
  ```bash
  sudo apt install ros-iron-teleop-twist-keyboard
  ```

Now, you’re ready to **simulate PLEN2 in Gazebo!** 🚀


