# Siemens_Project_2022-23
This repo is the code deliverable for 2022-23 Siemens project phase 2.
It implements LEGO assembly and disassembly from human demonstration.

## Prerequisites
Linux (All the modules were tested on Linux Ubuntu 18.04 and 20.04.)

ROS

Eigen 3.3.7

OpenCV

## Build
1. Clone the repo to `/path/to/catkin_ws/src/`
2. `cd /path/to/catkin_ws/`
3. `catkin_make`

## Learn LEGO task from Human Demonstration
Download demonstration video to `/path/to/repo/human_demonstration/`. An example video is available at [this link](https://drive.google.com/file/d/1B8meDY02TvG3Zd5gK1jp-HvyWr81xG2g/view?usp=share_link).
```
cd /path/to/repo/
python3 human_demonstration/learn_from_human_demonstration.py
```
The task graph will be stored at `/path/to/repo/config/assembly_tasks/`.


## Use Robot
### Setup Ethernet
1. Connect to the robot via an Ethernet cable.
2. Set PC IP address to `192.168.1.xxx`, except `192.168.1.100` (robot IP). A valid IP address could be `192.168.1.101`. 
3. Connect to the designated Ethernet.

### Run task
1. Modify `./config/user_config.json` as needed. Set `Use_Robot` to `1`.
2. Open a terminal and `cd /path/to/catkin_ws/`.
3. `source devel/setup.bash`
4. `roslaunch lego_assembly controller_node.launch`
5. Open a terminal and `cd /path/to/catkin_ws/`.
6. `source devel/setup.bash`
7. `roslaunch lego_assembly task_planning_cartesian_node.launch`

## Digital Twin
### Rviz
`roslaunch lego_assembly fanuc_rviz.launch`


### Gazebo for LEGO Assembly
1. Modify `./config/user_config.json` as needed. Set `Use_Robot` to `0`.
2. Open a terminal and `cd /path/to/catkin_ws/`.
3. `source devel/setup.bash`
4. `roslaunch lego_assembly fanuc_gazebo.launch`
5. Open a terminal and `cd /path/to/catkin_ws/`.
6. `source devel/setup.bash`
7. `roslaunch lego_assembly controller_node.launch`
8. Open a terminal and `cd /path/to/catkin_ws/`.
9. `source devel/setup.bash`
10. `roslaunch lego_assembly task_planning_cartesian_node.launch`