### Starter Code of Polaris GEM e2 Simulator for [ECE484](https://publish.illinois.edu/safe-autonomy/) and [CS588](http://luthuli.cs.uiuc.edu/~daf//courses/MAAV-22/588-2022-home.html)

#### University of Illinois at Urbana-Champaign

#### System: Ubuntu 20.04 + ROS Noetic (Gazebo 11)

#### Author: Hang Cui (hangcui1201@gmail.com)

This simulator was initially developed with ROS Melodic and Gazebo 9 in Ubuntu 18.04 for personal research in fall 2019. The Polaris GEM e2 vehicle was measured and modeled by Hang Cui and Jiaming Zhang using Solidworks. The compatible URDF files of simulator for RViz and Gazebo were constructed by Hang Cui. Later, this project was funded by the [Center of Autonomy](https://autonomy.illinois.edu/) at University of Illinois at Urbana-Champaign. It was further developed and merged into ROS Noetic and Gazeno 11 in the summer of 2021. This simulator is currently under development for research and teaching at University of Illinois at Urbana-Champaign.  

#### Polaris GEM e2 Vehicle

<a href="url"><img src="./images/Polaris_GEM_e2.png" width="600"></a>  

## Running the stack on Ubuntu 20.04 or 22.04 with Docker
> [!NOTE]
> Make sure to check the Nvidia Driver and supported CUDA version before proceeding by following the steps in the previous section.

#### Prerequisites
- Docker (In Linux - Make sure to follow the post-installation steps from [here](https://docs.docker.com/engine/install/linux-postinstall/))
- Nvidia Container Toolkit

Try running the sample workload from the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html) to check if your system is compatible.

```bash
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

You should see the nvidia-smi output similar to [this](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html#:~:text=all%20ubuntu%20nvidia%2Dsmi-,Your%20output%20should%20resemble%20the%20following%20output%3A,-%2B%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2D%2B%0A%7C%20NVIDIA%2DSMI%20535.86.10).

If you see the output, you are good to go. Otherwise, you will need to install the Docker and NVidia Container Toolkit by following the instructions. 
- For **Docker**, follow the instructions [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
  
- For **Nvidia Container Toolkit**, run `setup/get_nvidia_container.sh` from this directory to install, or see [this](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for more details.

#### Building the Docker image

To build a Docker image with all these prerequisites, you can use the provided Dockerfile by running.

```bash
bash setup/build_docker_image.sh
```

#### Running the Docker container

To run the container, you can use the provided Docker Compose file by running.
> [!NOTE]
> If you want to open multiple terminals to run the container, you can use the same command. It will automatically start a new terminal inside the same container.
```bash
bash run_docker_container.sh
```

#### Usage Tips and Instructions

##### Using Host Volume

You can use the host volume under the container's home directory inside the `<username>` folder. This allows you to build and run files that are on the host machine. For example, if you have a file on the host machine at `/home/<username>/project`, you can access it inside the container at `/home/<username>/host/project`.

##### Using Dev Containers Extension in VSCode

To have a good developer environment inside the Docker container, you can use the Dev Containers extension in VSCode. Follow these steps:

1. Install the Dev Containers extension in VSCode.
2. Open the cloned repository in VSCode.
3. Press `ctrl+shift+p`(or select the remote explorer icon from the left bar) and select `Dev-Containers: Attach to Running Container...`.
4. Select the container name `gem_stack-container`.
5. Once attached, Select `File->Open Folder...`.
6. Select the folder/workspace you want to open in the container.

This will set up the development environment inside the Docker container, allowing you to use VSCode features seamlessly.

#### Stopping the Docker container

To stop the container, you can use the provided stop script by running.

```bash
bash stop_docker_container.sh
```

## Building the Simulator and its packages

To build the simulator and its packages, you can use the provided docker run script and then navigating to the place your simulator workspace is located and then build using,
> [!NOTE]
> Make sure you have cloned this repository inside the `src` folder of your simulator workspace before building the packages.

```bash
cd <path_to_your_simulator_workspace>
# You should see the src folder inside your simulator workspace
catkin_make
```

## Running the Simulator
#### Track1 Environment

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch world_name:="track1.world"
```

```bash
source devel/setup.bash
roslaunch gem_launch gem_sensor_info.launch
```

<a href="url"><img src="./images/simple_track_rviz.png" width="600"></a>

<a href="url"><img src="./images/simple_track_gazebo.png" width="600"></a>

##### Demo of Pure Pursuit Controller

```bash
source devel/setup.bash
rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
```

<a href="url"><img src="./images/pp_controller.gif" width="600"></a>

##### Demo of Stanley Controller

```bash
source devel/setup.bash
rosrun gem_stanley_sim stanley_sim.py
```

<a href="url"><img src="./images/stanley_controller_rviz.gif" width="600"></a>

<a href="url"><img src="./images/stanley_controller_gazebo.gif" width="600"></a>

### Track2 Environment

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch world_name:="track2.world" y:=-98.5
```

<a href="url"><img src="./images/track2_gazebo.png" width="600"></a>

### Example Environment

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch
```

<a href="url"><img src="./images/example_rviz.png" width="600"></a>

<a href="url"><img src="./images/example_gazebo.png" width="600"></a>

### Highbay Environment

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch world_name:="highbay_track.world" x:=-1.5 y:=-21 yaw:=3.1416
```

<a href="url"><img src="./images/highbay_rviz.png" width="600"></a>

<a href="url"><img src="./images/highbay_gazebo.png" width="600"></a>

### E4 Vehicle in Parking world

Set the parameter `vehicle_name` to `e4` in the `gem_init.launch` file to use the E4 vehicle. By default, the vehicle name is `e2`.

```bash
source devel/setup.bash
roslaunch gem_launch gem_init.launch vehicle_name:="e4"
```

<a href="url"><img src="./images/e4_gazebo_rviz.png" width="600"></a>











