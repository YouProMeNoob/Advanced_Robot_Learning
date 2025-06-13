# Advanced Robot Learning - Final Project

## Authors: Group 2
- Dominic Khang 12012414
- Ryan Lee
- Dominik Pusztai
- Pascal Zoehrer 

## Installation

Installation can be done by either using an existing image that you know works or by building the docker image from scratch using the `cpu.Dockerfile`. After going through one of the installation methods refer to the commands listed in **Usage** to preserve your container state.

### Using an existing image
Download the `om_noetic_v2.tar` from TUWEL or use an existing image. If you downloaded the TUWEL image then run 
``` bash
docker load -i om_noetic_v2.tar
```

Otherwise, make sure the image is named om_noetic_v2:latest. If it is not then run this command to give the image a name and tag:
``` bash
docker tag <existing-image-id or name> om_noetic_v2:latest
```

Once you have an image then you need to run docker compose to create a container from the image and then use docker exec to start an interactive session inside the container.
- cpu
``` bash
xhost +local:root
docker compose --profile cpu up -d
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && bash"
```
- GPU
``` bash
xhost +local:root
docker compose --profile gpu up -d
docker exec -it arl_gpu_cont bash -c "source devel/setup.bash && bash"
```

It is very likely that your `catkin_ws/src` is outdated! Use this script that will download the latest commit from github and will run `catkin_make` for you
``` bash
chmod 777 /root/catkin_ws/shared/update_noetic.sh && /root/catkin_ws/shared/update_noetic.sh
```

You can try runnning
``` bash
cd /root/catkin_ws/shared
python3 motion_test.py
```

If python is missing dependencies then run this installation script that will download the necessary pip dependencies
- CPU
``` bash
chmod 777 /root/catkin_ws/shared/install_requirements.sh && /root/catkin_ws/shared/install_requirements.sh
```
- GPU 
``` bash
chmod 777 /root/catkin_ws/shared/install_gpu_requirements.sh && /root/catkin_ws/shared/install_gpu_requirements.sh
```

### Building the image
To build from scratch run
``` bash
xhost +local:root
docker builder prune
docker compose --profile cpu up --build -d  # or use --profile gpu
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && bash" # arl_gpu_cont
```

Now only the pip dependencies need to be installed if they are missing
- CPU
``` bash
chmod 777 /root/catkin_ws/shared/install_requirements.sh && /root/catkin_ws/shared/install_requirements.sh
```
- GPU

``` bash
chmod 777 /root/catkin_ws/shared/install_gpu_requirements.sh && /root/catkin_ws/shared/install_gpu_requirements.sh
```

## Usage
### Starting and Stopping the Docker Container
Give permission to access X-Server
``` bash
xhost +local:root
```

After the Installation the container can be restarted with
- cpu
``` bash
docker compose --profile cpu start 
```
- GPU
``` bash
docker compose --profile gpu start
```

Create a new terminal inside the container
- CPU
``` bash
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && cd /root/catkin_ws/shared && bash"
```
- GPU
``` bash
docker exec -it arl_gpu_cont bash -c "source devel/setup.bash && cd /root/catkin_ws/shared && bash"
```

Open terminator inside the container (will be installed with install_requirements.sh). Then you dont have to do docker exec for each new terminal window.
``` bash
terminator
```

The container can be stopped with
- CPU
``` bash
docker stop arl_cpu_cont 
```
- GPU
``` bash
docker stop arl_gpu_cont 
```

### Running in Simulation (om_6dof_controller)
This shows the deployment of the base controller (build by the team of Robotis)
1. **Start ROS master** in one terminal:
   ```bash
   roscore
   ```

2. **Launch the controller** in a new terminal:
   ```bash
   roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
   ```

3. **Launch Gazebo simulation** in another terminal:

    **Important** Change Line 30 in `/root/catkin_ws/src/open_manipulator_6dof_simulations/open_manipulator_6dof_gazebo/launch/open_manipulator_6dof_gazebo.launch` to:  
    ```
    args="-urdf -model open_manipulator_6dof -z 0.0 -param robot_description -x 0.0 -y 0.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0 -J joint1 0.0 -J joint2 -0.78 -J joint3 1.5 -J joint4 0.0 -J joint5 0.8 -J joint6 0.0"/>
    ```
    otherwise you will not see the robot arm.

    Then you can launch
    ```bash
    roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch controller:=position
    ```

4. **Launch the GUI control panel**:
   ```bash
   roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
   ```

### Running in Simulation (om_position_controller)
If you’re unable to access or work with the real robot, a simulation environment is provided via the `om_position_controller` package. Follow these steps:

1. **Start ROS master** in one terminal:
   ```bash
   roscore
   ```

2. **Launch the controller** in a new terminal:
   ```bash
   roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
   ```

3. **Start the position controller (simulation mode):**
   ```bash
   roslaunch om_position_controller position_control.launch sim:=true
   ```

4. **Start the republishing of the cube coordinates**
   ```bash 
   roscd om_position_controller/scripts && \
   python3 4_rs_detect_sim.py 
   ```

5. **(Optional)Visualise your recordings**
   First play the rosbag and then execute this script
   ```bash 
   roscd om_position_controller/scripts && \
   python3 simulated_trajectory.py
   ```

6. **Execute the pick_and_place script.**
   ```bash 
   roscd om_position_controller/scripts && \
   python3 pick_and_place.py
   ```

7. **For going back to the home pose execute:**
   ```bash 
   roscd om_position_controller/scripts && \
   python3 move_to_home.py 
   ```

#### ROS Startup Issues

If you encounter problems starting ROS (e.g., `roscore` fails with a network error like `Unable to contact my own server`), it's usually due to networking or hostname resolution issues.

To fix this, try setting the following environment variables **before starting `roscore`**:

```bash
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
```



