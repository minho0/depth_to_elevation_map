# ROS 2 Elevation Mapping CUPY

**Status**: 🚧 Under Development  
<!-- ![Elevation Map in ROS 2 Humble with Gazebo ](https://github.com/user-attachments/assets/0dd9ebbe-a90d-486f-9871-81921308fab9) -->

---

## Installation

You will need to install the following:

- VS Code
- Docker
- NVIDIA Container Toolkit
- NVIDIA CUDA Toolkit

## Setup

### ① Clone the Repository

```bash
git clone https://github.com/minho0/elevation_map_singlepcd.git
```



### ② Build the Docker Container

```bash
cd elevation_map_singlepcd/docker
docker build -t mhlee/elevation_mapping_cupy:latest .
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

### ① Launch the `elevation_mapping_cupy` node

```bash
ros2 launch elevation_mapping_cupy elevation_mapping_cupy.launch.py
```

### ② Launch the `depth_to_pointcloud` node inside the container

```bash
docker exec -it elevation_mapping_cupy bash
cd ~/workspace
source install/setup.bash
ros2 launch depth_to_pointcloud_pub depth_to_pointcloud.launch.py
```

### ③ Play the `.mcap` data

```bash
docker cp <host_path> elevation_mapping_cupy:/home/ros/workspace/src/elevation_mapping_cupy/
docker exec -it elevation_mapping_cupy bash
sudo apt update
sudo apt install ros-humble-rosbag2-storage-mcap
ros2 bag play data.mcap
```



## Single Pointcloud Mode

To enable **single pointcloud mode**, modify the following parameter:

```yaml
clear_map_before_update: true
```

in the configuration file:

```
/home/ros/workspace/src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/core_param.yaml
```
