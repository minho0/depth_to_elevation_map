# ROS 2 Elevation Mapping CUPY

**Status**: 🚧 Under Development  
<!-- ![Elevation Map in ROS 2 Humble with Gazebo ](https://github.com/user-attachments/assets/0dd9ebbe-a90d-486f-9871-81921308fab9) -->

---

## Installation

You will need to install the following:

- Docker
- NVIDIA Container Toolkit
- NVIDIA CUDA Toolkit

## Setup

### ① Clone the Repository

```bash
git clone https://github.com/minho0/depth_to_elevation_map.git
```



### ② Build the Docker Container
#### If the computer has not GPU(s), please remove the docker option '--gpus all'.
```bash
cd depth_to_elevation_map/docker
docker build -t rcv_dtc/elevation_mapping_cupy:0.1 .
./run.sh
```



### ③ Setup the Workspace

```bash
cd docker
./setup.sh
```



### ④ Build the Workspace

```bash
cd ~/workspace
colcon build --symlink-install
source install/setup.bash
source /opt/ros/humble/setup.bash
```



## Running the Demo

### ① (Current Terminal) Launch the `elevation_mapping_cupy` node

```bash
ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py
```

### ② (Second Terminal) Launch the `depth_to_pointcloud` node inside the container

```bash
docker exec -it elevation_mapping_cupy bash
cd ~/workspace
source install/setup.bash
ros2 launch depth_to_pointcloud_pub depth_to_pointcloud.launch.py
```

### ③ (Third Terminal) Play the `.mcap` data

```bash
docker cp <host_path> elevation_mapping_cupy:/home/ros/workspace/src/elevation_mapping_cupy/
docker exec -it elevation_mapping_cupy bash
sudo apt update
sudo apt install ros-humble-rosbag2-storage-mcap
ros2 bag play data.mcap
```

### (Option) (Fourth terminal) play the 'image'

To play the image synchronized with the rosbag, run:

```bash
docker exec -it elevation_mapping_cupy bash
python3 image_play.py
```


## Single Pointcloud Mode

To enable single pointcloud mode, set the following parameter to true.

To accumulate pointclouds over time, set it to false instead:
```yaml
clear_map_before_update: true # or false
```

in the configuration file:

```
/home/ros/workspace/src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/core_param.yaml
```
