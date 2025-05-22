# ROS2 Elevation Mapping Cupy 
 **Status**: Under Development 🚧

<!-- ![Elevation Map in ROS 2 Humble with Gazebo ](https://github.com/user-attachments/assets/0dd9ebbe-a90d-486f-9871-81921308fab9) -->

## Installation
you will need to install:
- VS Code
- Docker
- NVIDIA Container Toolkit
- NVIDIA CUDA Toolkit

---------------------------------------------------------


## Run the Container

#### Clone the Elevation Mapping CUPY Repository
```bash
git clone https://github.com/minho0/elevation_map_singlepcd.git
```

#### Building the Docker Container

  ```bash
  cd elevation_map_singlepcd/docker

  docker build -t mhlee/elevation_mapping_cupy:latest .

  ./run.sh
  ```

#### Setup the workspace 
  ```bash
  cd docker

  ./setup.sh
  ```

#### Build the workspace
  ```bash
  cd ~/workspace

  colcon build --symlink-install

  source install/setup.bash

  source /opt/ros/humble/setup.bash
  ```

#### Running demo

In a current terminal launch the elevation_mapping_cupy :
```bash
ros2 launch depth_to_pointcloud_pub depth_to_pointcloud.launch.py
``` 

In a second terminal launch the depth_to_pointcloud node:
```bash
docker exec -it elevation_mapping_cupy bash

cd ~/workspace

source install/setup.bash

ros2 launch depth_to_pointcloud_pub depth_to_pointcloud.launch.py
``` 

In a third terminal play the mcap data :
```bash
docker cp <'host_path'> elevation_mapping_cupy:/home/ros/workspace/src/elevation_mapping_cupy/data/

docker exec -it elevation_mapping_cupy bash

cd data

sudo apt update

sudo apt install ros-humble-rosbag2-storage-mcap

ros2 bag play data.mcap

``` 