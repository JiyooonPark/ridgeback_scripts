# Ridgeback

*Tested on **Ubuntu 18.04** with **ROS Melodic**.*

<img src="./doc/img/go_to_goal_full_correct.gif" width="600">

## Build and Compile

1. Clone this repository:
  ```sh
  cd your/catkin_ws
  cd src
  git clone https://github.com/JiyooonPark/ros_repo
  ```

2. Install the dependencies:
  ```sh
  git clone https://github.com/ros-teleop/teleop_twist_keyboard
  git clone https://github.com/JiyooonPark/ridgeback_simulator
  git clone https://github.com/JiyooonPark/ridgeback
  cd .. 
  cd catkin_make
  ```

3. Source the workspace:
  ```sh
  source ~/.bashrc
  ```

## Demo
Run the following commands in respective terminals.

set up before run:
```sh
    roslaunch ros_repo ridgeback_env.launch
```
for better performance, also run:
```sh
    roslaunch ridgeback_navigation amcl_demo.launch
```

ridgeback example:
```sh
    rosrun ros_repo go_to_goal_full.py
    rosrun ros_repo cmd_vel_cylinder.py
```

