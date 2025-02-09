mobility_ws/                     # Main ROS2 workspace
│── src/                          # Source directory for all ROS2 packages
│   │── my_robot_description/     # URDF, Gazebo, and RViz configurations
│   │   ├── urdf/                 # URDF & Xacro files for the rover
│   │   │   ├── rover.urdf.xacro  # Main rover model
│   │   │   ├── rover.gazebo.xacro # Gazebo-specific extensions
│   │   ├── launch/               # Launch files for simulation & visualization
│   │   │   ├── gazebo_launch.py  # Gazebo simulation launch
│   │   │   ├── rviz_launch.py    # RViz visualization launch
│   │   ├── rviz/                 # RViz configuration files
│   │   │   ├── rover.rviz        # RViz display settings
│   │   ├── CMakeLists.txt        # CMake build instructions
│   │   ├── package.xml           # ROS2 package metadata
│   │
│   │── my_robot_control/         # ROS2 nodes for motor & servo control
│   │   ├── scripts/              # Python control scripts
│   │   │   ├── rover.py          # Converts `/cmd_vel` into wheel & servo commands
│   │   │   ├── motor_controller.py # PWM control for motors & servos
│   │   │   ├── joint_publisher.py # Publishes joint states for simulation
│   │   ├── launch/               # Launch files for motor control
│   │   │   ├── motor_control_launch.py # Launches motor controller node
│   │   ├── CMakeLists.txt        
│   │   ├── package.xml           
│   │
│   │── my_robot_bringup/         # ROS2 package for running the full robot
│   │   ├── launch/               # Launch files for real-world deployment
│   │   │   ├── bringup_launch.py # Starts all necessary nodes on Raspberry Pi
│   │   ├── config/               # Config files for different environments
│   │   │   ├── robot.yaml        # Hardware-specific settings
│   │   ├── CMakeLists.txt        
│   │   ├── package.xml           
│   │
│   │── my_joystick_control/      # Joystick integration for manual driving
│   │   ├── scripts/              # Python joystick handling
│   │   │   ├── joystick_teleop.py # Reads joystick & publishes `/cmd_vel`
│   │   ├── launch/               
│   │   │   ├── joystick_launch.py # Launch file for joystick teleoperation
│   │   ├── CMakeLists.txt        
│   │   ├── package.xml           
│
│── install/                      # ROS2 install directory (after `colcon build`)
│── build/                        # Temporary build files
│── log/                          # Logs from ROS2 nodes
│── CMakeLists.txt                # ROS2 workspace-level CMake instructions
│── colcon.meta                   # Colcon metadata for building
