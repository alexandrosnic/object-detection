# Object Detection

## Objective

This project sets up a ROS 2 Humble-based object detection system using YOLOv5 in Docker and deploys it on Kubernetes.

## Approach

* **OS**
We will work on Ubuntu Jammy (22.04)
* **Environment** 
Docker
* **Middleware**
ROS2 Humble


## Setup Instructions

Follow the steps below to set up and run the project.

### Prerequisites

- Docker
- Kubernetes
- ROS 2 Humble

### Steps 

**1. Clone the repository**
```
$ git clone https://github.com/alexandrosnic/object-detection.git
$ cd object-detection
```

**2. Build image and start docker**
```
$ chmod +x docker/usr/local/bin/entrypoint.sh
$ ./start_docker.sh 
```

**3. Build the workspace**
Run
```
colcon build
source install/setup.bash
```

**4. Run the launch file**
```
cd src/iot_sim_launch/launch/
ros2 launch iot_simulation.launch.py
```

Or run each node independently:

**4.1. Run the rosbag**
```
ros2 bag play data/r2b_hope.db3 --loop --rate 0.5
```

**4.2. Visualize the data**

Open another terminal and connect to the docker
```
docker exec -it iot-simulation bash
```
Open Rviz2
```
rviz2 -d camera_vis.rviz
```

**4.3. Run the logger**

Open another terminal, connect to the docker, and then run the logger
```
docker exec -it iot-simulation bash
ros2 run iot_logger iot_logger
```

**4.4. Run the uploader**

Open another terminal, connect to the docker, and then run the uploader
```
docker exec -it iot-simulation bash
ros2 run iot_uploader iot_uploader
```

### Troubleshoot
1. In case there are issues with docker, make sure shell files have executable permissions:
```
chmod +x entrypoint.sh
```