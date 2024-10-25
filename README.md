## NOVAMOB

Novamob is an initiative ...

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

2. Clone this repository into your workspace:

    ```bash
    git clone https://github.com/rafa-oak/novamob.git
    ```
3. Install necessary dependencies:

    ```bash
    ...
    ```

 4. Add necessary environment variables:

    ```bash
    cd ros2/ws
    gedit ~/.bashrc
    ```
    Then add the following lines to the file and save it:

    ```bash
    ...
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


