# Pose Estimation of an Uncooperative Target Satellite for On-Orbit Servicing (OOS) Using EKF

This project implements pose estimation of an uncooperative target satellite for On-Orbit Servicing (OOS) using an Extended Kalman Filter (EKF) on a chaser satellite. The chaser satellite is equipped with a camera and a robotic arm, and pose estimation is performed using measurements from the camera. The project leverages the Robot Operating System (ROS 2) and Gazebo Ignition (Fortress) for simulation, with the `robot_localization` package for EKF implementation. Two methods are provided for pose estimation:

1. **Direct Odometry with Noise**: Pose data is obtained directly from a Gazebo Ignition simulation of the target satellite with added Gaussian noise.
2. **Visual Odometry via Dummy Publisher**: A simulated visual odometry pipeline where pose data with noise is published as an array, converted to `/Odometry` messages, and processed by the EKF.

This README provides detailed instructions to set up, build, and run the project, including how to visualize the results.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Project Setup](#project-setup)
- [File Structure](#file-structure)
- [Dependencies](#dependencies)
- [Running the Project](#running-the-project)
  - [Method 1: Direct Odometry from Gazebo Ignition Simulation](#method-1-direct-odometry-from-gazebo-ignition-simulation)
  - [Method 2: Visual Odometry via Dummy Publisher](#method-2-visual-odometry-via-dummy-publisher)
- [Visualizing Results](#visualizing-results)
- [Troubleshooting](#troubleshooting)
- [Notes](#notes)

## Prerequisites
- **Operating System**: Ubuntu 22.04 (Jammy Jellyfish) or later, with ROS 2 Humble installed.
- **ROS 2**: Ensure ROS 2 Humble is installed. Follow the [official ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) if needed.
- **Gazebo Ignition (Fortress)**: Install Gazebo Ignition Fortress for simulation:
  ```bash
  sudo apt install ros-humble-ign-ros2-control ros-humble-ros-ign
  ```
- **Git**: For cloning the repository.
- **Python**: Python 3.8 or later (typically included with ROS 2).
- **Colcon**: ROS 2 build tool. Install with:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```

## Project Setup
1. **Clone the Repository**:
   Clone the project repository from Git to your local machine:
   ```bash
   git clone https://github.com/SKaveendya27/Extended_kalman_filter.git ~/ros2_ws/src/satellite_mover
   ```
   

2. **Navigate to Workspace**:
   ```bash
   cd ~/ros2_ws
   ```

3. **Build the Project**:
   Build the workspace using `colcon`:
   ```bash
   colcon build --symlink-install
   ```
   The `--symlink-install` option allows changes to Python scripts without rebuilding.

4. **Source the Workspace**:
   Source the workspace to make the package available:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
   Add this to your `~/.bashrc` for convenience:
   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

## File Structure
The project is organized under the `satellite_mover` package in your ROS 2 workspace (`~/ros2_ws/src/satellite_mover`). Key files and directories include:
- **launch/satellite_mover.launch.py**: Launch file for the Gazebo Ignition simulation and EKF nodes.
- **config/ekf_config_gz.yaml**: Configuration file for the EKF node in Method 1 (Gazebo Ignition odometry).
- **config/ekf_config_vis.yaml**: Configuration file for the EKF node in Method 2 (visual odometry).
- **sdf/**: Directory containing the Satellite Description Format (SDF) files for the target satellite with added Gaussian noise.
- **scripts/odom_plotter.py**: Python script to plot EKF output for Method 1.
- **scripts/sat_pose_ekf_plot.py**: Python script to plot EKF output for Method 2.
- **scripts/dummy_pose_publisher.py**: Node that publishes noisy pose data as an array (Method 2).
- **scripts/pose_to_odom_converter.py**: Node that converts array data to `/Odometry` messages (Method 2).
- **package.xml**: ROS 2 package manifest.
- **CMakeLists.txt**: Build configuration for the package.

## Dependencies
The project requires the following ROS 2 packages:
- `robot_localization`: For EKF implementation.
- `geometry_msgs`: For pose and odometry message types.
- `nav_msgs`: For `/Odometry` message types.
- `ros-ign`: For Gazebo Ignition simulation integration.
- `matplotlib`: For plotting (Python dependency).

Install the dependencies:
```bash
sudo apt update
sudo apt install ros-humble-robot-localization ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-ign-ros2-control ros-humble-ros-ign
pip install matplotlib
```

Ensure `pip` is for Python 3:
```bash
pip3 install matplotlib
```

## Running the Project
The project supports two methods for pose estimation using EKF. Follow the instructions for each method below.

### Method 1: Direct Odometry from Gazebo Ignition Simulation
In this method, pose data is obtained directly from the Gazebo Ignition simulation of the target satellite, with Gaussian noise added to the satellite's SDF model. The EKF processes this data using the `ekf_config_gz.yaml` configuration and publishes the filtered pose on the `/odometry/filtered` topic.

1. **Launch the Simulation**:
   Navigate to your workspace and launch the Gazebo Ignition simulation with the EKF node:
   ```bash
   cd ~/ros2_ws
   ros2 launch satellite_mover satellite_mover.launch.py
   ```
   This command:
   - Starts Gazebo Ignition with the target satellite model.
   - Loads the EKF node from `robot_localization` configured with `ekf_config_gz.yaml`.
   - Publishes noisy odometry data from the satellite simulation.

2. **Check EKF Output**:
   Monitor the filtered pose estimates published on the `/odometry/filtered` topic:
   ```bash
   ros2 topic echo /odometry/filtered
   ```
   This displays the `nav_msgs/Odometry` messages containing the estimated pose of the target satellite.

3. **Visualize Results**:
   To plot the EKF-estimated poses, run the plotting script after the simulation has been running for at least 30 seconds:
   ```bash
   ros2 run satellite_mover odom_plotter
   ```
   The script generates plots of the pose data (position and orientation) using `matplotlib` and displays them after 30 seconds of data collection.

### Method 2: Visual Odometry via Dummy Publisher
In this method, a dummy publisher simulates visual odometry by publishing noisy pose data as an array. A converter node subscribes to this data, transforms it into `/Odometry` messages, and publishes them on the `/camera/odom/updated` topic. The EKF, configured with `ekf_config_vis.yaml`, subscribes to this topic and outputs filtered poses on `/odometry/filtered`.

1. **Launch the Simulation**:
   Use the same launch file as Method 1, which includes the dummy publisher and converter nodes:
   ```bash
   cd ~/ros2_ws
   ros2 launch satellite_mover satellite_mover.launch.py
   ```
   This command:
   - Starts Gazebo Ignition (optional, depending on configuration).
   - Runs the `dummy_pose_publisher.py` node to publish noisy pose arrays.
   - Runs the `pose_to_odom_converter.py` node to convert arrays to `/Odometry` messages on `/camera/odom/updated`.
   - Runs the EKF node with `ekf_config_vis.yaml` to process `/camera/odom/updated` and publish filtered poses on `/odometry/filtered`.

2. **Check EKF Output**:
   Monitor the filtered pose estimates:
   ```bash
   ros2 topic echo /odometry/filtered
   ```
   This displays the `nav_msgs/Odometry` messages with the EKF-estimated poses.

3. **Visualize Results**:
   To plot the EKF-estimated poses, run the plotting script after the simulation has been running for at least 30 seconds:
   ```bash
   ros2 run satellite_mover sat_pose_ekf_plot
   ```
   The script uses `matplotlib` to generate plots of the pose data (position and orientation) after 30 seconds.

## Visualizing Results
Both methods use `matplotlib` to generate plots of the EKF-estimated poses:
- **Position Plots**: X, Y, Z coordinates over time.
- **Orientation Plots**: Quaternion components or Euler angles (depending on implementation) over time.
- **Error Plots** (if implemented): Difference between noisy input and EKF-filtered output.

The plotting scripts (`odom_plotter.py` and `sat_pose_ekf_plot.py`) collect data for 30 seconds before generating and displaying the plots. Ensure the launch file is running and the `/odometry/filtered` topic is publishing data before running the plotting scripts.

To customize the plots (e.g., change duration or plot types), modify the respective Python scripts in the `scripts/` directory.

## Troubleshooting
- **Gazebo Ignition Fails to Launch**: Ensure `ros-humble-ign-ros2-control` and `ros-humble-ros-ign` are installed and the SDF files in the `sdf/` directory are valid for Gazebo Ignition.
- **No Data on `/odometry/filtered`**: Verify that the EKF node is running and subscribed to the correct input topics (`/odometry` for Method 1, `/camera/odom/updated` for Method 2). Check the `ekf_config_gz.yaml` (Method 1) or `ekf_config_vis.yaml` (Method 2) file for correct topic names.
- **Plotting Script Fails**: Ensure `matplotlib` is installed and the simulation has run for at least 30 seconds.
- **ROS 2 Node Errors**: Source the workspace (`source ~/ros2_ws/install/setup.bash`) and rebuild (`colcon build`) after any changes.
- **Permission Issues**: Run `sudo apt update` and `sudo apt install` with appropriate permissions.

## Notes
- The Gaussian noise in the SDF file (Method 1) simulates real-world uncertainties in satellite pose measurements.
- The dummy publisher (Method 2) is a placeholder for actual visual odometry data from a camera. Replace it with real camera data when available.
- The `ekf_config_gz.yaml` and `ekf_config_vis.yaml` files are critical for EKF performance. Tune parameters like covariance matrices and sensor trust levels for optimal results.
- Ensure the ROS 2 workspace is sourced in every new terminal session or add it to `~/.bashrc`.
- For further details on `robot_localization`, refer to the [official documentation](https://docs.ros.org/en/humble/pkgs/robot_localization.html).
- For Gazebo Ignition, refer to the [Ignition Fortress documentation](https://ignitionrobotics.org/docs/fortress).