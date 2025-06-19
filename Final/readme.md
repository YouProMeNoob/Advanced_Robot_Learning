# Advanced Robot Learning - Final Project

## Authors: Group 2
- Dominic Khang 12012414
- Ryan Lee
- Dominik Pusztai
- Pascal Zoehrer 

## Installation

Installation can be only be done by downloading the existing image from TUWEL. After going through one of the installation refer to the commands listed in **Usage** to preserve your container state.

### Using an existing image
``` bash
xhost +local:root
docker compose --profile toh_update up -d
docker exec -it asl_om_toh_cont bash
```
Inside the container run
``` bash
chmod 777 /root/catkin_ws/shared/toh_update_bash.sh && /root/catkin_ws/shared/toh_update_bash.sh && source ~/.bashrc
```

The container can be stopped with
``` bash
docker stop asl_om_toh_cont
```

## Usage
``` bash
xhost +local:root
docker compose --profile toh_update start
docker exec -it asl_om_toh_cont terminator
```

1. **Start ROS master** in one terminal:
   ```bash
   roscore
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



