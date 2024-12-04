## NOVAMOB


This repository provides simulation environments for the **NovaMob** and **Agilex Scout** robots in **ROS2**, compatible with both **Gazebo Classic** (only basic features) and **Gazebo Fortress** (Ignition Gazebo). This packages include different robot models, sensors, controllers, and configuration files necessary for seamless simulation and testing.


## Installation

### Prerequisites

- **ROS2** Humble installed on your system.
- **Gazebo Classic** and/or **Gazebo Fortress** installed.

### Clone the Repository

1. Create or navigate to your ROS2 workspace:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. Clone this repository and its submodules into your workspace:

    ```bash
    git clone --recurse-submodules https://github.com/rafa-oak/novamob.git
    ```
3. Install necessary dependencies:

    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the workspace:

    ```bash
    colcon build
    ```
5. Source the workspace:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
    
## Robot Descriptions

### Novamob
Novamob is a small 4 wheel drive custom robot designed for indoor environments...

It is equipped with the following sensors:
- **RPLIDAR A1**
- **MPU6050**
- **Intel RealSense D435i**
    
### Agilex Scout
The Scout is a bigger 4 wheel drive custom robot more suited for outdoor environments...

It is equipped with the following sensors:
- **VLP16 LIDAR**
- **IMU mpu6050**
- **Intel Realsense D435if**
- **GPS sensor**

## Launch files

### Visualizing the robots

To visualize the Novamob robot in Gazebo, use the following command:

```bash
ros2 launch novamob_nav2_gz display.launch.py
```

To visualize the Agilex Scout robot in Gazebo, use the following command:

```bash
ros2 launch scout_nav2_gz display.launch.py
```

These launch files also have several launch options that allow them to use differente configurations

- `use_sim_time` : Use simulation time.
- `world` : Path to the world file.
- `use_rviz` : Launch RViz for visualization.
- `model` (default: `novamob`): Robot model to use.
- `use_trailer` : Use trailer with the robot.
- `use_localization` : Use localization node.
- `x` : Initial x position of the robot.
- `y` : Initial y position of the robot.
- `z` : Initial z position of the robot.

### Running the Navigation Stack

To run the navigation stack for the Novamob robot, use the following command:

```bash
ros2 launch novamob_nav2_gz complete_navigation.launch.py
```

To run the navigation stack for the Agilex Scout robot, use the following command:

```bash
ros2 launch scout_nav2_gz complete_navigation.launch.py
```

This launch file has the same launch options listed above. You can them save the map of the world using this command:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Running with a Known Map

After having a map, we can then use autonomous navigation to navigate the environment. First, launch the robot and world simulation, without rviz since we don't need it:

```bash
ros2 launch novamob_nav2_gz display.launch.py use_rviz:=False

ros2 launch scout_nav2_gz display.launch.py use_rviz:=False
```
Then in another terminal, launch nav2_bringup with the map file:

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True  map:=/path/to/map.yaml
```
Finally, in another terminal initiate rviz:

```bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=True
```

### Running GPS Navigation (Scout Only)

To run the GPS navigation for the Agilex Scout robot, use the following command:

```bash
ros2 launch scout_nav2_gz gps_waypoint_follower.launch.py
```

If you wish to run each launch file independently, in different terminals run these 3 commands:

```bash
ros2 launch scout_nav2_gz gazebo_gps_world.launch.py
ros2 launch scout_nav2_gz dual_efk_navsat.launch.py

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

## Useful Resources

- [Nav2 Documentation](https://docs.nav2.org/concepts/index.html)
