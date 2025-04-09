# hunav_gazebo_fortress_wrapper

**This is a work in progress version**

Wrapper to use [HuNavSim](https://github.com/robotics-upo/hunav_sim) with the simulator Gazebo Fortress and ROS 2 Humble.

## Wrapper functioning

The wrapper is based on two components:

* The *hunav_gazebo_world_generator* node. It is in charge of reading the simulation configuration parameters (human agents parameters) from the *hunav_loader* node, and the base Gazebo world file. With these parameters, the node modifies the base world file and includes the indicated human agents and the Gazebo plugin to control them (HuNavPlugin).

* The *HuNavPlugin* is a Gazebo plugin in charge of interacting with the HuNavSim to control the human movements. This interaction is performed through ROS2 services. 


## Generator parameters

The *hunav_gazebo_world_generator* node has these parameters:

*  ```gazebo_world_file```. Name of the base Gazebo world file. This world should contain the static models (obstacles, rooms, furniture, etc). The human agents will be added automatically to this world. The world files must be placed inside the directory *worlds*.  
 

### Plugin params

The plugin parameters are read as ROS2 parameters also by the *hunav_gazebo_world_generator* node, which is in charge of adding the plugin to the Gazebo world file.

*  ```update_rate```. Update rate (Hz) of the plugin proccess.
*  ```robot_name```. Name of the Gazebo model corresponding to the robot that will be spawned.
*  ```use_gazebo_obs```. If True, the closest obstacle for each agent will be computed and added to the agent features (Default: True).
* ```global_frame_to_publish```. Coordinate frame in which the pedestrian positions are provided.
* ```use_navgoal_to_start```. Boolean to indicate whether the plugin must wait for the robot to receive a navigation goal to start the movement of the human agents (Default: False).  
* * ```navgoal_topic```. Name topic in which the robot navigation goal will be received. The message must be type *geometry_msgs/msg/PoseStamped*. This parameter is used only if the parameter ```use_navgoal_to_start``` is True. (Default: */goal_pose*).
* ```ignore_models```. List of the Gazebo models than must be ignored by the pedestrians like the ground plane.

An example snippet of the HuNavPlugin which is automatically inserted in the world file:

```xml
<plugin filename="libHuNavSystemPluginIGN.so" name="HuNavSystemPluginIGN">
    <update_rate>100.0</update_rate>
    <robot_name>robot</robot_name>
    <use_gazebo_obs>True</use_gazebo_obs>
    <global_frame_to_publish>map</global_frame_to_publish>
    <use_navgoal_to_start>false</use_navgoal_to_start>
    <navgoal_topic>goal_pose</navgoal_topic>
    <ignore_models>
    	<model>cafe</model>
        <model>ground_plane</model>
    </ignore_models>
</plugin>
```

## Dependencies

* The HuNavSim: https://github.com/robotics-upo/hunav_sim
* The Gazebo Fortress (Ignition Gazebo 6)  
* The ROS package ros_gz (*sudo apt install ros-humble-ros-gz*)

## Compilation

* This is a ROS2 package so it must be placed inside a ROS2 workspace and compiled through the regular colcon tool. 
```sh
colcon build --packages-select hunav_gazebo_fortress_wrapper
```

## Example launching

* Example of a Gazebo world of a café with a static actor playing the role of the robot:
```sh
ros2 launch hunav_gazebo_fortress_wrapper cafe_fortress_test.launch.py
```
* You can modify the hunav agents spawned by modifiying the agents' configuration file in:
  ```[your_workspace]/src/hunav_sim/hunav_agent_manager/config/agents.yaml```
     
* If you prefer to create the agents' configuration file by using the GUI, you can do it by launching:
```sh
ros2 launch hunav_rviz2_panel hunav_rviz2_launch.py
```

## ToDos

- [] Fix the path to found the animation models and activate the change of animation.
- [] Prepare different environment with a robot navigating in ROS 2.
- [] Provide a container to deploy the system. 



