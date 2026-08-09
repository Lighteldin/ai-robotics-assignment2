## Assignment 2: Gazebo-ROS 2 Bridging & Lane-Following Race

---

## Team

| Name | Student ID |
|------|------------|
| Noureldin Ashraf Ahmed | 23012107 |
| Momen Wagdy Hamed | 2206134 |

---

### Run Video
[Video on Google Drive](https://drive.google.com/file/d/1yCmptpfqyY31PHBevA7ObKCzWhHCDblD/view?usp=sharing)

### ROS 2 Bag
[`/cmd_vel` bag on Google Drive](https://drive.google.com/file/d/1FKmUBuA7GvVuDqjWCO6mMYi6BMGayuIE/view?usp=sharing)

> The bag corresponds to the same run shown in the video.

---

# Project Overview

>The goal of this assignment was to build an autonomous lane-following system using Gazebo, ROS 2, and the simulated Prius race car.

Our system is divided into two main parts:

- **Lane Perception** - processes the front camera image and estimates the
  car's position relative to the lane.
- **Lane Controller** - uses the lane error from the perception node to
  generate velocity commands for the car.

The overall pipeline is:

    Gazebo Camera
          |
          v
    Lane Perception
          |
          | lane error
          v
    Lane Controller
          |
          | /cmd_vel
          v
    Gazebo Prius

---

# Installation Guide

## 1. Clone the Repository

Clone the repository and enter the workspace:

    git clone https://github.com/Lighteldin/ai-robotics-assignment2.git
    cd ai-robotics-assignment2

## 2. Install Dependencies

Make sure ROS 2 Lyrical and Gazebo are installed and available on the system.

The project uses the following main ROS 2 packages:

- `ros_gz_bridge`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `cv_bridge`
- OpenCV

If the required ROS-Gazebo bridge package is not installed:

    sudo apt install ros-lyrical-ros-gz

## 3. Build the Workspace

Build the two packages used by the lane-following system:

    colcon build --packages-select lane_perception lane_controller

After building, source the workspace:

    source install/setup.bash

The `build/`, `install/`, and `log/` directories are intentionally excluded
from Git and are generated locally when the workspace is built.

---

# Running the System

The system is run using three terminals.

## Terminal 1 - Gazebo Simulator

Source ROS 2 Lyrical:

    source /opt/ros/lyrical/setup.bash

Start Gazebo:

    gz sim

Open the Prius raceway world in Gazebo.

---

## Terminal 2 - ROS-Gazebo Bridge

Source ROS 2 Lyrical:

    source /opt/ros/lyrical/setup.bash

Source the workspace:

    source install/setup.bash

Start the bridge:

    ros2 run ros_gz_bridge parameter_bridge \
    --ros-args -p config_file:=config/gz_sim_bridge_car.yaml

The bridge connects the required Gazebo topics to their ROS 2 equivalents,
including the Prius camera and control topics.

---

## Terminal 3 - Lane Following System

Build the lane perception and lane controller packages:

    colcon build --packages-select lane_perception lane_controller

Source the workspace:

    source install/setup.bash

Launch the complete lane-following system:

    ros2 launch lane_controller lane_following.launch.py

This launch file starts the lane perception and lane controller nodes.

---

## Running Order

The recommended order is:

    Terminal 1
        |
        v
    Start Gazebo
        |
        v
    Terminal 2
        |
        v
    Start ROS-Gazebo Bridge
        |
        v
    Terminal 3
        |
        v
    Launch Lane Following
---

# Package Logic

## Lane Perception

The `lane_perception` package receives the front camera image from the Prius and
estimates the vehicle's position relative to the lane.

The basic process is:

    Camera Image
         |
         v
    Region of Interest
         |
         v
    HLS Saturation
         |
         v
    Thresholding + Morphological Filtering
         |
         v
    Hough Line Detection
         |
         v
    Left / Right Lane Classification
         |
         v
    Lane Center
         |
         v
    Lane Error


Only the relevant road area is processed. The saturation channel is thresholded
to isolate the colored lane markings, after which Hough lines are detected and
classified as left or right lane boundaries.

>[!IMPORTANT]
>If only one lane is visible, the missing lane is estimated using the expected
>lane width. If both lanes are temporarily lost, the detector enters a recovery
>mode and continues steering in the required direction until lane markings are
>detected again.

The resulting lane error is published on:

    /lane/error

---

## Lane Controller

The `lane_controller` package receives the lane error from the perception
package and converts it into a steering command using a PD controller.

The controller calculates:

    Steering = -(Kp * Error + Kd * Derivative)

where:

    Kp = 0.02
    Kd = 0.0005


The controller maintains a constant forward velocity of `5.0` and limits the
steering command to `±0.7`.

The resulting velocity command is published as a `Twist` message on:

    /cmd_vel

This creates the main control loop:

    Camera
      |
      v
    Lane Perception
      |
      | /lane/error
      v
    PD Controller
      |
      | /cmd_vel
      v
    Prius

---
#  Final Result

The final system almost completed one full lap, but failed when lanes interconnected with each other. We were unable to debug any further due to hardware problems and immense lag.

The submitted video and ROS 2 bag correspond to the same run.
