# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
import os
from pathlib import Path
from os import environ, pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, 
                            SetEnvironmentVariable, SetLaunchConfiguration, AppendEnvironmentVariable) 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro
from launch_pal.actions import CheckPublicSim
from launch_pal.robot_arguments import CommonArgs
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.include_utils import include_scoped_launch_py_description
from pmb2_description.launch_arguments import PMB2Args


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    wheel_model: DeclareLaunchArgument = PMB2Args.wheel_model
    laser_model: DeclareLaunchArgument = PMB2Args.laser_model
    add_on_module: DeclareLaunchArgument = PMB2Args.add_on_module
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim
    world_name: DeclareLaunchArgument = CommonArgs.world_name
    #navigation: DeclareLaunchArgument = CommonArgs.navigation
    #slam: DeclareLaunchArgument = CommonArgs.slam
    advanced_navigation: DeclareLaunchArgument = CommonArgs.advanced_navigation
    x: DeclareLaunchArgument = CommonArgs.x
    y: DeclareLaunchArgument = CommonArgs.y
    yaw: DeclareLaunchArgument = CommonArgs.yaw


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    # Set use_sim_time to True
    set_sim_time = SetLaunchConfiguration('use_sim_time', 'True')
    launch_description.add_action(set_sim_time)

    set_public_sim = SetLaunchConfiguration('is_public_sim', 'True')
    launch_description.add_action(set_public_sim)

    set_slam = SetLaunchConfiguration('slam', 'False')
    launch_description.add_action(set_slam)

    set_nav = SetLaunchConfiguration('navigation', 'True')
    launch_description.add_action(set_nav)

    set_log_level = SetLaunchConfiguration('log_level', 'debug')
    #launch_description.add_action(set_log_level)

    # Shows error if is_public_sim is not set to True when using public simulation
    public_sim_check = CheckPublicSim()
    launch_description.add_action(public_sim_check)

    robot_name = 'pmb2'
    # packages = ['pmb2_description']
    # model_path = get_model_paths(packages)

    # print("model_path: ", model_path)

    # gazebo_model_path_env_var = SetEnvironmentVariable(
    #     'GAZEBO_MODEL_PATH', model_path)
    # launch_description.add_action(gazebo_model_path_env_var)

    pkg_path = get_package_prefix("pmb2_description")
    model_path = os.path.join(pkg_path, "share")
    resource_path = pkg_path

    # packages = ["pmb2_description"]

    # # model_path = get_model_paths(packages)

    # if "GAZEBO_MODEL_PATH" in environ:
    #     model_path += pathsep + environ["GAZEBO_MODEL_PATH"]

    # if "GAZEBO_RESOURCE_PATH" in environ:
    #     resource_path += pathsep + environ["GAZEBO_RESOURCE_PATH"]

    # # model_path = "/home/upo/pmb2_public_ws/install/pmb2_description/share:/usr/share/gazebo-11/models:"

    # print("\n\n\n\nmodel in pal_test after first if:", model_path, "\n\n\n")

    set_env_gz_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            model_path)
    launch_description.add_action(
        set_env_gz_resources
    )

    # gazebo = include_scoped_launch_py_description(
    #     pkg_name='pal_gazebo_worlds',
    #     paths=['launch', 'pal_gazebo.launch.py'],
    #     env_vars=[gazebo_model_path_env_var],
    #     launch_arguments={
    #         'world_name':  launch_args.world_name,
    #         'model_paths': packages,
    #         'resource_paths': packages,
    #     })
    # launch_description.add_action(gazebo)



    # ----------------------------------------------------------
    #    NAVIGATION
    # ----------------------------------------------------------
    # navigation = include_scoped_launch_py_description(
    #     pkg_name='pmb2_2dnav',
    #     paths=['launch', 'pmb2_nav_bringup.launch.py'],
    #     launch_arguments={
    #         'robot_name':  robot_name,
    #         'laser':  launch_args.laser_model,
    #         'is_public_sim': launch_args.is_public_sim,
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'world_name': launch_args.world_name,
    #         'slam': launch_args.slam,
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('navigation')))
    # launch_description.add_action(navigation)

    # nav_launch = PathJoinSubstitution([
    #     FindPackageShare("pmb2_2dnav"),
    #     "launch",
    #     "pmb2_nav_bringup.launch.py"
    # ],)
    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([nav_launch]),
    #     launch_arguments={
    #         'robot_name': 'pmb2',  #robot_name,
    #         'laser': LaunchConfiguration('laser_model'), #launch_args.laser_model,
    #         'is_public_sim': LaunchConfiguration('is_public_sim'), #launch_args.is_public_sim,
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'world_name': LaunchConfiguration('world_name'), #launch_args.world_name,
    #         'slam': 'False', #launch_args.slam,
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('navigation'))
    # )
    # launch_description.add_action(navigation)

    
    pmb2_2dnav = get_package_share_directory("pmb2_2dnav")

    nav_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "navigation_launch.py"
    ],)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        launch_arguments={
            "params_file": os.path.join(
                pmb2_2dnav, "params", "pmb2_nav_public_sim.yaml"
            ),
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )

    slam_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "slam_launch.py"
    ],)
    slam_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments={
            "params_file": os.path.join(
                pmb2_2dnav, "params", "pmb2_nav_public_sim.yaml"
            ),
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    loc_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "localization_launch.py"
    ],)
    loc_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([loc_launch]),
        launch_arguments={
            "params_file": os.path.join(
                pmb2_2dnav, "params", "pmb2_nav_public_sim.yaml"
            ),
            "map": LaunchConfiguration("world_name"),
            "use_sim_time": "True",
        }.items(),
        condition=(IfCondition(LaunchConfiguration("navigation"))),
    )

    rviz_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "rviz_launch.py"
    ],)
    rviz_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments={
            "rviz_config": os.path.join(
                pmb2_2dnav, "config", "rviz", "navigation.rviz"
            ),
            "use_sim_time": "True",
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation"))
    )

    launch_description.add_action(nav2_bringup_launch)
    launch_description.add_action(loc_bringup_launch)
    launch_description.add_action(slam_bringup_launch)
    launch_description.add_action(rviz_bringup_launch)



    # ----------------------------------------------------------
    #    ROBOT SPAWN
    # ----------------------------------------------------------
    print("Launching robot_spawn.launch.py")
    print("robot name: ", robot_name)
    #print("launch_args.x: ", launch_args.x)
    #print("LaunchConfiguration('x'): ", LaunchConfiguration('x'))
    # robot_spawn = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pmb2_gazebo, "launch", "robot_spawn.launch.py")
    #     ),
    #     launch_arguments={
    #         "robot_name": robot_name,
    #         "x": LaunchConfiguration('x'), #launch_args.x,
    #         "y": LaunchConfiguration('y'), #launch_args.y,
    #         "yaw": LaunchConfiguration('yaw') #launch_args.yaw
    #     }.items(),
    # )

    # ORIGINAL
    robot_spawn = include_scoped_launch_py_description(
        pkg_name='pmb2_gazebo',
        paths=['launch', 'robot_spawn.launch.py'],
        launch_arguments={
            "robot_name": robot_name,
            "x": launch_args.x,
            "y": launch_args.y,
            "z": "0.15",
            "yaw": launch_args.yaw,
        }
    )

    # robot_spawn = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-topic',
    #         'robot_description',
    #         '-entity',
    #         LaunchConfiguration('robot_name'),
    #         '-x', LaunchConfiguration('x'),
    #         '-y', LaunchConfiguration('y'),
    #         '-Y', LaunchConfiguration('yaw'),
    #     ],

    #     output='screen',
    # )
    launch_description.add_action(robot_spawn)


    #----------------------------------------------------------
    #   ROBOT BRINGUP
    #----------------------------------------------------------
    print("Launching pmb2_bringup.launch.py")
    pmb2_bringup = include_scoped_launch_py_description(
        pkg_name='pmb2_bringup', paths=['launch', 'pmb2_bringup.launch.py'],
        launch_arguments={
            #'wheel_model': launch_args.wheel_model,
            'laser_model': launch_args.laser_model,
            'add_on_module': launch_args.add_on_module,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'is_public_sim': launch_args.is_public_sim,
        }
    )
    launch_description.add_action(pmb2_bringup)



def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']

    return model_paths
