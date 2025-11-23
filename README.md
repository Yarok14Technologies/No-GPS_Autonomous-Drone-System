# No-GPS_Autonomous-Drone-System
PX4 + ROS2 + Visual SLAM + VIO + Obstacle Avoidance + Autonomous Return Home A fully autonomous drone capable of navigating, mapping, avoiding obstacles, detecting targets, and returning to its launch position without GPS.  Designed for research-grade missions, ISRO-style No-GPS challenges, DARPA-style underground navigation, and autonomous flight.

Here is a **clean, professional, production-grade README.md** for your **No-GPS Autonomous Drone** project.
You can copy–paste this directly into your repo.

---

# 🛰️ **No-GPS Autonomous Drone System**

**PX4 + ROS2 + Visual SLAM + VIO + Obstacle Avoidance + Autonomous Return-Home**

This repository contains the full software stack for a **fully autonomous drone** capable of navigating, mapping, avoiding obstacles, detecting targets, and returning to its launch position **without GPS**.

Designed for research-grade missions, ISRO-style No-GPS challenges, DARPA-style underground navigation, and indoor autonomous flight.

---

## 🚀 **Key Capabilities**

✔ GPS-Denied Navigation
✔ Visual SLAM + VIO Fusion
✔ Autonomous Exploration
✔ Obstacle Detection & Avoidance
✔ Visual Landing Pad Detection
✔ No-GPS Return-to-Home (Keyframe Homing)
✔ Fully ROS2-based Modular Architecture
✔ PX4 Offboard Flight Control
✔ Simulation Ready (PX4 SITL + Gazebo/Ignition)

---

# 🧱 **System Architecture**

### 🔵 **1. Visual SLAM (ORB-SLAM3 / RTAB-Map)**

* Frontend: Feature extraction + tracking
* Backend: Bundle adjustment + loop closure
* Output: `/slam/pose`, `/slam/map`

### 🔵 **2. State Estimation (EKF Fusion)**

* IMU + SLAM + optical flow + barometer
* Provides `/odom`, `/tf`
* Smooth, drift-corrected pose

### 🔵 **3. Perception**

* Depth obstacle detection
* Optical flow for drift control
* AprilTag / landing pad tracking

### 🔵 **4. Mapping**

* Voxel map / Octomap generation
* Used by local planner for obstacle avoidance

### 🔵 **5. Planning**

* Global Planner → A* / D* Lite
* Local Planner → MPC / DWA
* Trajectory smoothing → Polynomial/MPC

### 🔵 **6. Mission Manager**

* Flight FSM
* Exploration logic
* Fail-safe handling
* Visual return-home
* Battery-aware landing

### 🔵 **7. PX4 Bridge**

* Offboard mode control
* Setpoint publishing
* Arm/takeoff/land API

---

# 📁 **Repository Structure**

```
no_gps_drone/
├── README.md
├── docker/
├── environment/
├── src/
│   ├── slam/
│   ├── perception/
│   ├── state_estimation/
│   ├── mapping/
│   ├── planning/
│   ├── mission_manager/
│   ├── px4_bridge/
│   ├── control/
│   └── simulation/
├── launch/
├── config/
├── data/
├── tests/
└── docs/

```
# 📁 **Detailed Repository Structure**
```
no_gps_drone/
│
├── README.md
├── LICENSE
├── .gitignore
├── docker/
│   ├── Dockerfile.dev
│   ├── Dockerfile.sim
│   ├── docker-compose.yml
│   └── entrypoint.sh
│
├── environment/
│   ├── ros2.repos            # vcs import repos (ORB-SLAM3, mavlink, perception libs)
│   ├── requirements.txt      # Python requirements
│   └── setup_instructions.md
│
├── src/
│   ├── slam/
│   │   ├── orb_slam3_ros/
│   │   ├── rtabmap_ros/
│   │   └── vio_fusion/       # VIO/IMU fusion wrapper (ekf2 alternative)
│   │
│   ├── perception/
│   │   ├── apriltag_detector/
│   │   ├── optical_flow/
│   │   ├── obstacle_depth/
│   │   └── landing_pad_detector/
│   │
│   ├── state_estimation/
│   │   ├── ekf_fusion/
│   │   ├── imu_preintegration/
│   │   └── tf_manager/
│   │
│   ├── mapping/
│   │   ├── octomap_server/
│   │   ├── voxel_map/
│   │   └── occupancy_grid_tools/
│   │
│   ├── planning/
│   │   ├── global_planner/        # A*/D* Lite / RRT*
│   │   ├── local_planner/         # MPC / DWA / APF
│   │   ├── trajectory_optimizer/   # polynomial, bezier, or MPC smoothening
│   │   └── path_follower/         # converts path->waypoints->commands
│   │
│   ├── mission_manager/
│   │   ├── autonomous_flight_node/
│   │   ├── return_home_manager/
│   │   ├── keyframe_homing/
│   │   ├── failsafe_manager/
│   │   ├── battery_monitor/
│   │   └── mission_api.srv
│   │
│   ├── px4_bridge/
│   │   ├── microRTPS_agent/
│   │   ├── px4_msgs/
│   │   ├── mavros_plugins/
│   │   └── setpoint_api/
│   │
│   ├── control/
│   │   ├── attitude_controller/
│   │   ├── velocity_controller/
│   │   └── landing_controller/
│   │
│   ├── utils/
│   │   ├── transforms/
│   │   ├── logging_tools/
│   │   ├── calibration/
│   │   └── math_lib/
│   │
│   └── simulation/
│       ├── gazebo_worlds/
│       ├── px4_sitl_launcher/
│       ├── sensor_emulators/
│       ├── fake_vio/
│       ├── fake_apriltag/
│       └── challenge_worlds/
│
├── launch/
│   ├── full_system.launch.py
│   ├── slam_only.launch.py
│   ├── perception.launch.py
│   ├── planning.launch.py
│   ├── mission.launch.py
│   ├── return_home_test.launch.py
│   └── sim_world.launch.py
│
├── config/
│   ├── cameras/
│   │   ├── calibration.yaml
│   │   ├── stereo_params.yaml
│   │   └── rectification.yaml
│   ├── ekf/
│   │   ├── ekf_params.yaml
│   │   └── noise_models.yaml
│   ├── planners/
│   │   ├── global_planner.yaml
│   │   ├── local_planner.yaml
│   │   └── mpc.yaml
│   ├── slam/
│   │   ├── orb_slam3.yaml
│   │   └── rtabmap.yaml
│   ├── mission/
│   │   └── mission_params.yaml
│   └── px4/
│       ├── fw_params.params
│       ├── ekf2_no_gps.params
│       └── vision_yaw_fusion.params
│
├── data/
│   ├── bags/
│   │   ├── flight1/
│   │   └── slam_debug/
│   ├── logs/
│   │   ├── test_runs/
│   │   └── errors/
│   ├── maps/
│   │   ├── octomap/
│   │   └── voxel/
│   └── keyframes/
│
├── tests/
│   ├── hardware_tests/
│   │   ├── imu_noise_test.md
│   │   ├── camera_latency_test.md
│   │   └── system_id/
│   │
│   ├── simulation_tests/
│   │   ├── slam_relocalization_test.md
│   │   ├── return_home_test.md
│   │   ├── obstacle_avoidance_test.md
│   │   └── landing_accuracy_test.md
│   │
│   └── unit_tests/
│       ├── test_slam_utils.cpp
│       ├── test_planner.py
│       └── test_mission_node.cpp
│
└── docs/
    ├── architecture.md
    ├── sensors_and_calibration.md
    ├── mission_fsm.md
    ├── return_home_algorithm.md
    ├── failsafe_modes.md
    ├── simulation_setup.md
    └── evaluation_metrics.md

```

Full explanation is inside `docs/architecture.md`.

---

# 🔧 **Installation**

## 1️⃣ Clone Repository

```
git clone https://github.com/your-name/no_gps_drone.git
cd no_gps_drone
```

## 2️⃣ Install ROS2 Dependencies

```
sudo apt install python3-colcon-common-extensions \
                 ros-humble-navigation2 \
                 ros-humble-slam-toolbox \
                 ros-humble-tf2-tools
```

## 3️⃣ Import External Packages

```
vcs import < environment/ros2.repos
```

## 4️⃣ Install Python Dependencies

```
pip install -r environment/requirements.txt
```

## 5️⃣ Build the Workspace

```
colcon build --symlink-install
source install/setup.bash
```

---

# 🛫 **Running the System**

### **Start PX4 SITL**

```
cd PX4-Autopilot
make px4_sitl gazebo
```

### **Start Full Autonomy Stack**

```
ros2 launch no_gps_drone full_system.launch.py
```

---

# 🔍 **Return-Home Without GPS (Core Algorithm)**

The drone uses **keyframe-based visual homing**:

1. Capture keyframes during outbound flight
2. Store positions + descriptors
3. For RTH, match live camera feed to stored keyframes
4. Use reprojection + homography to estimate direction home
5. Global planner generates RTH path
6. Local planner avoids obstacles
7. Autonomous landing at return position

Detailed in: `docs/return_home_algorithm.md`.

---

# 🧪 **Testing**

### **Simulation Tests**

* SLAM drift test
* Relocalization test
* Obstacle avoidance test
* No-GPS return-home test
* Landing accuracy test

Run:

```
ros2 launch no_gps_drone simulation/sim_world.launch.py
```

---

# 📦 **Hardware Requirements**

* PX4 flight controller (Pixhawk 6C / CUAV X7 / Holybro Durandal)
* Stereo camera (Intel Realsense D455 / ZED2 / MYNT-EYE)
* IMU (built-in or external)
* Companion computer (Jetson Orin Nano / Xavier NX / Raspberry Pi 5)
* Optical flow sensor (optional)
* LiDAR or depth camera (optional)

---

# 🧠 **Software Stack**

* ROS2 Humble / Iron
* PX4 / MAVROS / microRTPS
* ORB-SLAM3 or RTAB-Map
* Nav2 Stack
* FastDDS
* OpenCV / Eigen / g2o / Ceres

---

# 🤝 **Contributing**

PRs, issues, and feature requests are welcome.
Follow the coding standards in:
`docs/contribution_guidelines.md`

---

# 📜 **License**

MIT License (unless you choose otherwise)

---

# 📞 **Contact**

For queries, reach out at:
**[bibinnbiji924@gmail.com](mailto:bibinnbiji924@gmail.com)**

---

