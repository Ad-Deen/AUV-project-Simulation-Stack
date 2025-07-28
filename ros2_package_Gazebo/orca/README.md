# AUV ORCA Simulation Package (ROS2 Humble)

This repository contains the simulation files for Project AUV ORCA, designed to work with **ROS2 Humble** and **Ignition Gazebo** (Fortress version). The package provides an environment for simulating the ORCA model in a custom-built pool world with accurate hydrodynamic underwater physics.

The simulation package requires **ros2-gz-bridge** to interface between ROS2 and Ignition Gazebo. Once set up, you will be able to control the ORCA model using keyboard commands and publish velocity messages to control its movement.

<img src="Git content/Screenshot from 2025-07-28 16-03-38.png" width="600" height="400" />

<img src="Git content/Screenshot from 2025-07-28 16-04-07.png" width="600" height="400" />

## Features

- **Custom Pool World**: A custom-built underwater pool environment for ORCA with realistic hydrodynamics.
- **ROS2-Gazebo Interface**: Communication between ROS2 nodes and Ignition Gazebo through `ros2-gz-bridge`.
- **Hydrodynamic Physics**: Accurate simulation of underwater dynamics using Ignition Gazebo Fortress version.
- **Control**: Topics for publishing velocity messages for ORCA's movement.
- **Teleop**: A ready-to-use teleoperation script for controlling the ORCA model via keyboard commands.

## Prerequisites

1. **ROS2 Humble**: Follow the [official ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html).
2. **Ignition Gazebo Fortress**: Ensure that you have Ignition Gazebo Fortress installed. You can find installation instructions [here](https://ignitionrobotics.org/docs/fortress).
3. **ros2-gz-bridge**: This package is needed to bridge messages between ROS2 and Ignition Gazebo. Follow the installation instructions [here](https://github.com/ignitionrobotics/ros_ign).

## Pre-Build Instructions

Before building the package, you need to download the **AUV.dae** file and place it in the appropriate directory.

1. Download the **AUV.dae** file from the following link:  
   [Download AUV.dae](https://drive.google.com/drive/folders/1k21B9THWymgC9J7LdG8_TtC-5e7bhADc?usp=sharing)

2. Once downloaded, place the **AUV.dae** file in the `worlds` directory of the ORCA simulation package. The path should be:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/auv-orca-simulation.git
   ```

3. After placing the file in the correct directory, you can proceed with building the package as outlined in the [installation section](#installation).


## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   /orca/worlds/AUV.dae
   ```
2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the package:
   ```bash
   colcon build --symlink-install
   ```
4. Source the setup file:

   ```bash
   source install/setup.bash
   ```
## Usage

### Starting the Simulation

To start the simulation, you need to launch the simulation world along with the ORCA model in Ignition Gazebo. The following command launches the simulation:

```bash
ros2 launch auv_orca_simulation start_simulation.launch.py
```
This will:

- Launch the **Ignition Gazebo Fortress** simulation with the custom pool world.
- Spawn the **ORCA model** in the environment with accurate underwater physics.

### Teleoperation (Keyboard Control)

The package includes a teleoperation script that allows you to control the ORCA model using keyboard commands. To start the teleoperation, run:

```bash
ros2 run auv_orca_simulation teleop_control
```
<img src="Git content/Screencast from 07-28-2025 04-07-59 PM (1).gif" width="800" height="600" />

Once started, you can control the ORCA model using the following keys:

- **W**: Forward thrust
  - Activates propellers 4 and 7 for forward movement.
  - Deactivates propellers 5 and 6.

- **S**: Reverse thrust
  - Activates propellers 5 and 6 for reverse movement.
  - Deactivates propellers 4 and 7.

- **D**: Strafing right
  - Activates propellers 4 and 6 for strafing to the right.
  - Deactivates propellers 5 and 7.

- **A**: Strafing left
  - Activates propellers 5 and 7 for strafing to the left.
  - Deactivates propellers 4 and 6.

- **E**: Steering right
  - Activates propellers 6 and 7 for steering right.
  - Deactivates propellers 4 and 5.

- **Q**: Steering left
  - Activates propellers 4 and 5 for steering left.
  - Deactivates propellers 6 and 7.

- **Any other key**: Propellers off
  - Deactivates all propellers.

Each key press adjusts the speed of the relevant propellers. The speed value is set to **-10.0** for forward/reverse thrust, and propellers are deactivated if no valid key is pressed.

Make sure to use the script for controlling the movement and navigation of the ORCA model in the simulation.
Each key press adjusts the speed of the relevant propellers. The speed value is set to -10.0 for forward/reverse thrust, and propellers are deactivated if no valid key is pressed.

Make sure to use the script for controlling the movement and navigation of the ORCA model in the simulation.


### Stopping the Simulation

To stop the simulation, press `Ctrl+C` in the terminal where the simulation is running.
