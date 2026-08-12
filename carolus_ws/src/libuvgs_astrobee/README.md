# WELCOME TO FIT-UVGS ASTROBEE VERSION
As you can see we kept most files that allows you to change the engine as desired, although we feel confidently that intial tests shall be sucessful with the current engine, as we have expierienced with our in-lab conditions

## Building
### Dependencies 
Astrobee should have natively all the required dependencies to build the code, but feel free to check the CMakeLists for further detail.

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

Terminal 2
````bash
cd $folder_ws
source devel/setup.bash
rosrun carolus_astrobee carolus_astrobee
````
### Troubleshooting
- Open Rviz and check the /postprocessed/image topic, if is not publishing regularly, it means the target couldn't be detected. Feel free to change the preprocessing method/values, as it is hardware dependent
- Only supporting RED LED target
