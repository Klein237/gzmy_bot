# Differential Drive Robot with ROS 2 Jazzy & Gazebo Sim Harmonic

This repository demonstrates two approaches for simulating and teleoperating a differential-drive mobile robot in ROS 2 Jazzy with Gazebo Sim Harmonic:

1. **Gazebo DiffDrive Plugin** – using the built-in Gazebo `gz-sim-diff-drive-system` plugin
2. **ros2\_control Framework** – leveraging the ROS 2 control stack (`DiffDriveController`) for a hardware-abstracted interface

---

## Features

* **Dual Simulation Modes**

  * **Plugin Mode**: quick setup via the `gz-sim-diff-drive-system` Gazebo plugin
  * **ros2\_control Mode**: full ROS 2 hardware interface with `DiffDriveController`
* **Teleoperation Ready**

  * standard `/cmd_vel` teleop with `teleop_twist_keyboard`
  * remapped and stamped command support in ros2\_control mode
* **Modular Launch System**

  * single launch file (`gzsim.launch.py`) toggles modes via `use_ros2_control`
  * conditional parameters for stamped vs. unstamped Twist messages

---

## Prerequisites

* Ubuntu 24.04 LTS
* ROS 2 Jazzy (including `ros-jazzy-gz-ros2-control` & demos)
* Gazebo Sim Harmonic
* `teleop_twist_keyboard` package
* Ament build tools (`colcon`, `ros2 launch`)

---

## Installation

```bash
# 1. Clone this repository
git clone https://github.com/Klein237/gzmy_bot.git
cd gzmy_bot

# 2. Install dependencies
sudo apt update
sudo apt install \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-ros2-control-demos \
  ros-jazzy-teleop-twist-keyboard

# 3. Build and source
colcon build --packages-select gzmy_bot
source install/setup.bash
```

---

## Usage

### 1. Plugin Mode (Gazebo DiffDrive)

1. **Launch the simulation**

   ```bash
   ros2 launch gzmy_bot gzsim.launch.py
   ```
2. **Teleoperate**

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

### 2. ros2\_control Mode

1. **Launch with ROS 2 control enabled**

   ```bash
   ros2 launch gzmy_bot gzsim.launch.py use_ros2_control:=true
   ```
2. **Teleoperate with stamped commands**

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args -p stamped:=true \
     --remap cmd_vel:=/diff_cont/cmd_vel
   ```

---

> **Tip:** When launching with `use_ros2_control:=true`, consider setting `use_stamped_vel: true`
> or adding a remap in your launch file if you want to use `TwistStamped` messages.

---

## Acknowledgements

* Inspired by [Articulated Robotics’ Gazebo tutorial](https://youtu.be/fH4gkIFZ6W8?si=HUpXoAhIaewlbRPj)
* Thanks to the ROS and Gazebo communities for their ongoing support and innovation

---

