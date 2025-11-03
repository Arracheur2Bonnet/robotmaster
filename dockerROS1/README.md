# WELCOME TO FIT-UVGS ASTROBEE DOCKER DEBUG VERSION
This allows benchtesting using bags with rviz, with minimal effort to install dependencies

## Requirements
- You must have Docker installed in your system. 
- If you are using a Nvidia Graphics Card, you may need to install the Nvidia Container Toolkit. 

## Building Image

Assuming you're inside the cloned repository directory
````bash
cd dockerROS1

sudo docker build -t rosnoetic_carolus -f dockerstuff/Dockerfile_ros_20 .
````
This will build the image with all debug tools and extra dependencies. The image is tagged as ***rosnoetic_carolus***.

## Running
- Indicate where the directory libuvgs_astrobee is:
````bash
FOLDER=ADDYOURLOCATION #make sure to change ADDYOURLOCATION to your path
````
- Connect virtual displays
````bash
xhost +local:root
````
- Run container (W/O Nvidia) -- If this is not your case, use the other command for Nvidia users
````bash
sudo docker run -it --rm --name carolus -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/root/ros_ws/src" rosnoetic_carolus \
    bash -c "catkin_make && source devel/setup.bash && exec bash"
````
- Run container with Nvidia Toolkit
````bash
 sudo docker run -it --rm --name carolus --runtime=nvidia --gpus all -e "DISPLAY=$DISPLAY" \
     -e "QT_X11_NO_MITSHM=1" \
     -e "LIBGL_ALWAYS_SOFTWARE=1" \
     -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     -v "$FOLDER:/root/ros_ws/src"  rosnoetic_carolus \
     bash -c "catkin_make && source devel/setup.bash && exec bash"
````
- Connect other terminals to the container, repeat this for as many terminals you want to connect
````bash
sudo docker exec -it carolus bash
````
### Compile and Run
- Extract the zip to your ROS_WS, i.e., folder_ws->src->libuvgs_astrobee
- (carolus_astrobee.cpp) Change lines 82-91 with your proper camera proprieties
- (carolus_astrobee.cpp) Change line 120 to your proper camera topic
- catkin_make

IF YOU DON'T HAVE ROSMASTER RUNNING, THEN

Terminal 1
````bash
roscore
````

Terminal 2 -- Assuming you're inside *ros_ws*
````bash
source devel/setup.bash
rosrun carolus_astrobee carolus_astrobee
````
Terminal 3
````bash
rviz
````
### Troubleshooting
- Open Rviz and check the /postprocessed/image topic, if is not publishing regularly, it means the target couldn't be detected. Feel free to change the preprocessing method/values, as it is hardware dependent
- Only supporting RED LED target or Mono Images

### Nvidia Container Toolkit
These commands were taken for the original [Nvidia Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) website for installing with ***apt***. I have personally tested on Ubuntu 20.04 LTS and Ubuntu 22.04 LTS. Note that this asumes you have the proper Nvidia driver already working in your machine.

If you don't know if you have the proper driver installed, check as follows:

````bash
sudo nvidia-smi #you may try this without sudo 
````
Sample PARTIAL Output for a Geforce RTX 3080 running alongside an Intel discrete graphics card (Ubuntu 20.04)
````bash
---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.183.01             Driver Version: 535.183.01   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 3080 ...    On  | 00000000:01:00.0 Off |                  N/A |
| N/A   44C    P8              14W / 135W |     15MiB / 16384MiB |      0%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
````                                                                                         

**Note that GPU is set ON**
```bash
0  NVIDIA GeForce RTX 3080 ...    On 
```
If you see **Off** instead of **On**, force all GPUs to persistent mode

````bash
sudo nvidia-smi -pm 1
````
Installing the Container Toolkit,

- Configure the repository
````bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
````
- Update packages list and install
````bash
sudo apt-get update
````
````bash
sudo apt-get install -y nvidia-container-toolkit
````
- Configure the Nvidia Toolkit and restart Docker
````bash
sudo nvidia-ctk runtime configure --runtime=docker
````
````bash
sudo systemctl restart docker
````
- Run a sample container to verify the installation was sucessfull 
````bash
sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi #you may call docker without sudo if you have previously configured so
````



