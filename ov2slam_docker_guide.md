# OV2SLAM Docker Build and Run Guide

This guide covers building the OV2SLAM Docker image, launching it with host networking to access a connected USB camera, and running the node structure with RViz.

## 1. Build the Docker Image

Run the following command from the root of the `ov2slam` repository to build the image (it uses the Dockerfile located in the `docker` directory):

```bash
cd ~/slam/ov2slam
docker build --network=host -t ov2slam_docker -f docker/Dockerfile .
```

> **Note:** Building might take a while as it will compile Eigen, OpenCV, OpenGV, Sophus, Ceres, and the workspace itself.

## 2. Allow X11 forwarding from Docker

To visualize the camera output and SLAM results on RViz, you'll need to allow Docker to access your local X server display:

```bash
xhost +local:root
```
*(If the above fails, you can try `xhost +local:root` or just `xhost +` temporarily)*

## 3. Run the Docker Container

Run the built image using your computer's host network (`--net=host`). We also bind `/dev/video0` (which is typically your mono camera) and share the X11 UNIX socket. If your camera is on a different index, replace `/dev/video0` appropriately.

```bash
docker run -it --rm \
    --net=host \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device="/dev/video0:/dev/video0" \
    ov2slam_docker \
    bash
```

## 4. Run the ROS2 Nodes (Inside the Container)

Once inside the container, you will be in the `/ws` directory.

### Sourcing the Workspaces
First, source the main ROS distro and your built workspace:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

### Launching the Camera Node
OV2SLAM expects a stream of images over ROS2. You can use the standard `v4l2_camera` to read from `/dev/video0`. If the package isn't installed in the image, you can quickly install it:

```bash
apt-get update && apt-get install -y ros-humble-v4l2-camera
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[640,480] --remap /image_raw:=/cam0/image_raw &
```
*(This will publish your camera feed to `/image_raw`)*

### Launching the OV2SLAM Node
Now start the OV2SLAM node using a monocular parameter file. We will use the EuRoC mono parameter configuration as a template (you can adjust this later depending on your camera's calibration).

```bash
ros2 run ov2slam ov2slam_node src/ov2slam/parameters_files/fast/euroc/euroc_mono.yaml
```

**Important:** The `euroc_mono.yaml` file natively expects the camera topic to be `/cam0/image_raw`. The command above uses `--remap /image_raw:=/cam0/image_raw` so that `v4l2_camera` correctly provides the images to the SLAM node.

### Launching RViz Validation
Open a new terminal, jump into the same container using `docker exec -it -e DISPLAY=$DISPLAY <container_name> bash`, source the workspace, and run RViz using the provided config:

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
rviz2 -d /ws/src/ov2slam/ov2slam_visualization.rviz
```
### Launching mapper node
Open a new terminal, jump into the same container using `docker exec -it -e DISPLAY=$DISPLAY <container_name> bash`, source the workspace, and run mapper using the provided config:
```bash
ros2 run ov2slam map2d_node
```



You should now be able to see the SLAM pipeline attempting to output tracking and mapping results based on your live mono camera!
