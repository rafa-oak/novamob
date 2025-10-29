## NOVAMOB


This repository provides simulation environments for the **Novamob** and **Agilex Scout** robots in **ROS2**, compatible with both **Gazebo Classic** (only basic features) and **Gazebo Fortress** (Ignition Gazebo). This packages include different robot models, sensors, controllers, and configuration files necessary for seamless testing and navigation.

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



## Opennav_Coverage 

[Open Navigation's Nav2 Complete Coverage ](https://github.com/ros-navigation/navigation2/issues/4251#issuecomment-2441750468) is a repository that allows for complete coverage of open-fields or rows (such as in farms/vineyards). Its capabilities can be used along with Novamob's multiple environments and robots, taking into account certain constraints.

### Integration
The opennav_coverage integration must be done in a separate workspace from the rest of the project, since sourcing opennav_coverage breaks the nav2 bt_navigator, as seen in this [issue](https://github.com/ros-navigation/navigation2/issues/4251#issuecomment-2441750468). Therefore, to run any Novamob launch file (especially the ones that use GPS navigation), it is recommended to do it in the previously created workspace. To launch the opennav_coverage capabilities, a new ROS2 workspace must be setup.

Start by creating a workspace:
```
cd ~
mkdir open_navigation_ws
mkdir open_navigation_ws/src
cd open_navigation_ws/src
```

Then clone the necessary packages:
```
git clone https://github.com/Fields2Cover/Fields2Cover.git -b v1.2.1
git clone https://github.com/rafa-oak/opennav_coverage.git -b humble
git clone https://github.com/rafa-oak/novamob.git
```
We then need to build the fields2cover library before any of the other packages:
```
cd ~/open_navigation_ws/src/Fields2Cover
mkdir -p build; 
cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; 
make -j$(nproc); 
sudo make install;
```
Finally, we can build the rest of the packages:
```
source /opt/ros/humble/setup.bash
cd ~/open_navigation_ws
colcon build --base-paths src/ --symlink-install
```

### Launch Files


### Troubleshooting
It is possible that an error occurs in the fields2cover package when building, which causes the opennav_coverage package to also fail building.

To fix this, the fields2cover package must be removed from the workspace, along with all files in the computer related to fields2cover and the build, install and log folders in the workspace
```rm -r build install log```

After that is done, follow the integration instructions again and the error should be fixed. (As seen in this[ issue](https://github.com/open-navigation/opennav_coverage/issues/58))


### Bashrc example


```
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
#source ~/ros2_ws/install/local_setup.bash
source ~/farm_ws/install/local_setup.bash
#source ~/test_ws/install/local_setup.bash
#source ~/maize_ws/install/local_setup.bash
source /usr/share/gazebo/setup.sh
source /usr/share/gazebo-11/setup.sh

export TURTLEBOT3_MODEL=waffle
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
#export GAZEBO_MODEL_PATH=/home/rafael/Documents/objects

export GAZEBO_MODEL_PATH=/home/rafael/farm_ws/src/scout_description/models


#force Nvidia GPU to be used
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

#Use Cyclone DDS 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#Use Fast DDS 
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```


## Video Demonstrations

These test videos were recorded to exemplify the many functionalities of this project.  
Alternatively, they can be watched in succession in this [YouTube playlist](https://www.youtube.com/playlist?list=PLAreMUJ4OLkDdXZTOYVWaCbDlmRvhL1az).

### Test 1: Launching Different Robot Configurations  
[![Test 1 Video](https://img.youtube.com/vi/de5w_UzkWr4/0.jpg)](https://www.youtube.com/watch?v=de5w_UzkWr4)

### Test 2: SLAM Mapping of an Unknown Environment  
[![Test 2 Video](https://img.youtube.com/vi/yfOf5FuHmV4/0.jpg)](https://www.youtube.com/watch?v=yfOf5FuHmV4)

### Test 3: Navigation on a Known Map  
[![Test 3 Video](https://img.youtube.com/vi/Uz4oWvS0Dzc/0.jpg)](https://www.youtube.com/watch?v=Uz4oWvS0Dzc)

### Test 4: GPS-Based Navigation in Outdoor Scenarios  
[![Test 4 Video](https://img.youtube.com/vi/K5aH1Chey7Y/0.jpg)](https://www.youtube.com/watch?v=K5aH1Chey7Y)

### Test 5: Static Obstacle Avoidance  
[![Test 5 Video](https://img.youtube.com/vi/-IbJUOouiAg/0.jpg)](https://www.youtube.com/watch?v=-IbJUOouiAg)

### Test 6: Dynamic Obstacle Avoidance  
**Using LIDAR:**  
[![Test 6 Video](https://img.youtube.com/vi/WpcBOu0J4lU/0.jpg)](https://www.youtube.com/watch?v=WpcBOu0J4lU)

**Using GPS:**  
[![Test 6 GPS Video](https://img.youtube.com/vi/9sccmkO9_lE/0.jpg)](https://www.youtube.com/watch?v=9sccmkO9_lE)

### Test 7: Navigating Over Ramps and Speed Bumps  
**Scout robot:**  
[![Test 7 Scout Video](https://img.youtube.com/vi/Ldx0hbSa1Gc/0.jpg)](https://www.youtube.com/watch?v=Ldx0hbSa1Gc)  

**Novamob robot:**  
[![Test 7 Novamob Video](https://img.youtube.com/vi/d8450XNd1dw/0.jpg)](https://www.youtube.com/watch?v=d8450XNd1dw)

**Novamob robot with lowered wheels:**  
[![Test 7 Novamob LoweredVideo](https://img.youtube.com/vi/vmbk2GNjAQo/0.jpg)](https://www.youtube.com/watch?v=vmbk2GNjAQo)

### Test 8: Trailer Navigation  
**Novamob robot:**  
[![Test 8 Video](https://img.youtube.com/vi/9dGqhdYVlVo/0.jpg)](https://www.youtube.com/watch?v=9dGqhdYVlVo)

**Scout robot:**  
[![Test 8 Scout Video](https://img.youtube.com/vi/nffpbeH8KjQ/0.jpg)](https://www.youtube.com/watch?v=nffpbeH8KjQ)

### Test 9: Open-Field Complete Coverage Path Planning  
[![Test 9 Video](https://img.youtube.com/vi/SusDtpd-aWM/0.jpg)](https://www.youtube.com/watch?v=SusDtpd-aWM)

### Test 10: Row Coverage in Maize Field  
[![Test 10 Video](https://img.youtube.com/vi/gBNX1hG__eo/0.jpg)](https://www.youtube.com/watch?v=gBNX1hG__eo)

### Test 11: Simulator Performance Comparison  
**Baseline scenario:**  
[![Test 11 Video](https://img.youtube.com/vi/wc4LrwbLAgo/0.jpg)](https://www.youtube.com/watch?v=wc4LrwbLAgo)

**Static Obstacles scenario:**  
[![Test 11 Video](https://img.youtube.com/vi/VsdMkV3HwZI/0.jpg)](https://www.youtube.com/watch?v=VsdMkV3HwZI)

**Dynamic Obstacles scenario:**  
[![Test 11 Video](https://img.youtube.com/vi/Suyu6FRf_LA/0.jpg)](https://www.youtube.com/watch?v=Suyu6FRf_LA)




## Useful Resources

### Core Documentation
- **Nav2 Documentation:** https://docs.nav2.org
- **Nav2 GitHub:** https://github.com/ros-navigation/navigation2
- **ROS2 Humble Docs:** https://docs.ros.org/en/humble
- **Gazebo Fortress Docs:** https://gazebosim.org/docs/fortress
- **Gazebo Classic Docs:** https://classic.gazebosim.org/tutorials

### Complete Coverage
- **OpenNav Coverage Repository:**
  https://github.com/open-navigation/opennav_coverage
- **Fields2Cover:**
  https://github.com/Fields2Cover/Fields2Cover

### Repositories Used
- **Novamob Repository (this project):**
  https://github.com/rafa-oak/novamob
- **OpenNav Coverage Fork (Only Humble Compatibility):**
  https://github.com/rafa-oak/opennav_coverage
- **Virtual Maize Field Fork :**
  https://github.com/rafa-oak/virtual_maize_field




