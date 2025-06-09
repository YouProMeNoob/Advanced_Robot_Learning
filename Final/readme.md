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
``` bash
xhost +local:root
docker compose --profile cpu up -d
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && bash"
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
``` bash
chmod 777 /root/catkin_ws/shared/install_requirements.sh && /root/catkin_ws/shared/install_requirements.sh
```

### Building the image
To build from scratch run
``` bash
xhost +local:root
docker builder prune
docker compose --profile cpu up --build -d
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && bash"
```

Now only the pip dependencies need to be installed if they are missing
``` bash
chmod 777 /root/catkin_ws/shared/install_requirements.sh && /root/catkin_ws/shared/install_requirements.sh
```


## Usage
Give permission to access X-Server
``` bash
xhost +local:root
```

After the Installation the container can be restarted with
``` bash
docker compose --profile cpu start
```

Create a new terminal inside the container
``` bash
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && cd /root/catkin_ws/shared && bash"
```

The container can be stopped with 
``` bash
docker stop arl_cpu_cont 
```