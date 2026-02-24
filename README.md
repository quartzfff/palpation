# Palpation and Kinematic Calibration Package

This repository contains ROS 2 packages for NDI Aurora tracking, NetFT force sensing, and automated palpation grid routines.

## 1. Hardware Setup

Before running the software, ensure:
* **NDI Aurora:** Connected via Serial/USB.
* **ATI Force Sensor:** Connected via Ethernet.
* **Network:** Set your wired connection to a static IP (usually in the `192.168.1.x` range).
* **Verify ATI:** Open a browser and go to `192.168.1.1`. The ATI sensor is used to collect force data; the sync node will not run without a live force data stream.

## 2. Robot Initialization

To enable communication with the robot, navigate to your core library and setup the CAN interface:

```bash
cd ~/aliss_core/src/teleop/ves-ros-interface
sudo ./setup-can.sh

Connect to the robot (skipping visualizers and disconnected SIDs):
Bash

ros2 run ves_ros_interface ves_ros_interface --no-visualizer --left-sid-disconnected --right-sid-disconnected --no-kuka

3. Testing Movement

In a new terminal, you can test Cartesian positions.

    ⚠️ WARNING: Always "zero" the position (return to a safe home) after testing. All units are in meters.

Bash

ros2 topic pub /ves/left/joint/servo_cp geometry_msgs/PoseStamped "{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 0.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

4. Launching Sensors

Launch the NDI and Force sensor synchronization node. Ensure the network wired connection is selected and both sensors are connected.
Bash

cd ~/code/palpation
ros2 launch sync_logger_node multi_sensor_logger_launch.py

5. Running the Palpation Algorithm

To run the automated grid palpation:
Bash

ros2 run ves_kinematic_calibration palpation_grid

Configuration Tips

The logic is located in palpation_grid.py. If you modify these parameters, you must rebuild the workspace.

    Grid Points: Edit self.grid_points = [] (around line 93) to insert individual grid points at the surface depth.

    Depth/Direction: Adjust lines 81 and 82 to change pushing direction and pushing depth. Be careful: everything is in meters!

    Data Logging: Change the CSV filename to avoid overwriting data:
    self.logfile = open('pbh_palpation_grid_002_anglepush.csv', 'w', newline='')

6. Building the Code

Every time you modify a Python script or C++ file, go to the root of your workspace and build:
Bash

cd ~/code/palpation
colcon build
source install/setup.bash
