# Advanced Robot Learning - Final Project

## Authors: Group 2
- Dominic Khang 12012414
- Ryan Lee
- Dominik Pusztai
- Pascal Zoehrer 

## Installation
### Using an existing image
Download the `om_noetic_v2.tar` from TUWEL or use an existing image. Make sure the image is named om_noetic_v2:latest otherwise run this command to give the image a name and tag
``` bash
docker tag <existing-image-id or name> <new-name>:<new-tag>
```

Execute 
``` bash
docker load -i om_noetic_v2.tar
```

Once the image is loaded it can be started with 
``` bash
xhost +local:root
docker compose --profile cpu up -d
docker exec -it arl_cpu_cont bash -c "source devel/setup.bash && bash"
```

Make sure to use the latest github commit
``` bash
chmod 777 /root/catkin_ws/shared/update_noetic.sh && /root/catkin_ws/shared/update_noetic.sh
```

Inside the container execute the installation script
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

Now only the pip dependencies need to obe installed
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