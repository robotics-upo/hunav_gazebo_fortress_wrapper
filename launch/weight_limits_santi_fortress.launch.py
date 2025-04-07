from os import path
from os import environ
from os import pathsep

import os
from launch.actions import AppendEnvironmentVariable

from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, ExecuteProcess, Shutdown, 
                            RegisterEventHandler, TimerAction, LogInfo, AppendEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration, 
                            PythonExpression, EnvironmentVariable)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():

    # to activate the use of nvidia gpu
    use_nvidia_gpu = [
        '__NV_PRIME_RENDER_OFFLOAD=1 ',
        '__GLX_VENDOR_LIBRARY_NAME=nvidia ',
    ]

    # World generation parameters
    world_file_name = LaunchConfiguration('base_world')
    gz_obs = LaunchConfiguration('use_gazebo_obs')
    rate = LaunchConfiguration('update_rate')
    robot_name = LaunchConfiguration('robot_name')
    global_frame = LaunchConfiguration('global_frame_to_publish')
    use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    ignore_models = LaunchConfiguration('ignore_models')


    # agent configuration file
    agent_conf_file = PathJoinSubstitution([
        FindPackageShare('hunav_agent_manager'),
        'config',
        LaunchConfiguration('configuration_file')
    ])

    # Read the yaml file and load the parameters
    hunav_loader_node = Node(
        package='hunav_agent_manager',
        executable='hunav_loader',
        output='screen',
        parameters=[agent_conf_file]
    )

    # world base file
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        world_file_name
    ])

    # the node looks for the base_world file in the directory 'worlds'
    # of the package hunav_gazebo_plugin direclty. So we do not need to 
    # indicate the path
    hunav_gazebo_worldgen_node = Node(
        package='hunav_gazebo_wrapper',
        executable='hunav_gazebo_world_generator',
        output='screen',
        parameters=[{'base_world': world_file},
        {'use_gazebo_obs': gz_obs},
        {'update_rate': rate},
        {'robot_name': robot_name},
        {'global_frame_to_publish': global_frame},
        {'use_navgoal_to_start': use_navgoal},
        {'ignore_models': ignore_models}]
        #arguments=['--ros-args', '--params-file', conf_file]
    )

    ordered_launch_event = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_loader_node,
            on_start=[
                LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    actions=[hunav_gazebo_worldgen_node],
                )
            ]
        )
    )

    # Then, launch the generated world in Gazebo 
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'models',
    ])

    config_file_name = 'params.yaml' 
    pkg_dir = get_package_share_directory('hunav_gazebo_wrapper') 
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    config_file = path.join(pkg_dir, 'launch', config_file_name) 

    
    model, plugin, media = GazeboRosPaths.get_paths()

    print(f"gazebo models: {my_gazebo_models}\n")

    print(f"plugin path: {plugin}\n")

    #print('model:', model)

    if 'IGN_GAZEBO_RESOURCE_PATH' in environ:
        model += pathsep + environ['IGN_GAZEBO_RESOURCE_PATH']
        print("here1")
    if 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH' in environ:
        plugin += pathsep + environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH']
        print("here2")
    if 'IGN_GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep + environ['IGN_GAZEBO_RESOURCE_PATH']
        model += pathsep + environ['IGN_GAZEBO_RESOURCE_PATH']
        print("here3")

    env = {
        'IGN_GAZEBO_MODEL_PATH': model,
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': plugin,
        'IGN_GAZEBO_RESOURCE_PATH': media
    }

    print('env:', env)

    print(f"gazebo models: {my_gazebo_models}\n")

    print(f"plugin path: {plugin}\n")

    ign_model_path = os.getenv('IGN_GAZEBO_MODEL_PATH', '')
    ign_plugin_path = os.getenv('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    ign_resource_path = os.getenv('IGN_GAZEBO_RESOURCE_PATH', '')

    print(f"ign_model_path: {ign_model_path}\n")
    print(f"ign_plugin_path: {ign_plugin_path}\n")
    print(f"ign_resource_path: {ign_resource_path}\n")

    set_env_gazebo_model = SetEnvironmentVariable(
        name='IGN_GAZEBO_MODEL_PATH',
        value=[EnvironmentVariable('IGN_GAZEBO_MODEL_PATH', default_value=''), my_gazebo_models]
    )
    set_env_gazebo_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value=[EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''), my_gazebo_models]
    )
    set_env_gazebo_plugin = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', 
        value=[EnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default_value=''), plugin]
    )

    print(f"set_env_gazebo_model 1: {set_env_gazebo_model} ")
    print(f"set_env_gazebo_resource 1: {set_env_gazebo_resource} ")
    print(f"set_env_gazebo_plugin 1: {set_env_gazebo_plugin} \n")


    set_env_gazebo_model = AppendEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            my_gazebo_models)
    
    set_env_gazebo_resource = AppendEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            my_gazebo_models)
    
    set_env_gazebo_plugin = AppendEnvironmentVariable(
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            plugin)

    print(f"set_env_gazebo_model 2: {set_env_gazebo_model} ")
    print(f"set_env_gazebo_resource 2: {set_env_gazebo_resource} ")
    print(f"set_env_gazebo_plugin 2: {set_env_gazebo_plugin} \n")

    print(f"ign_model_path: {ign_model_path}\n")
    print(f"ign_plugin_path: {ign_plugin_path}\n")
    print(f"ign_resource_path: {ign_resource_path}\n")

    # the world generator will create this world
    # in this path
    world_path = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_wrapper'),
        'worlds',
        'generatedWorld.world'
    ])

    print("Gazebo server\n")

    print(f"World path: {world_path}\n")

    print(f"gz_args: {['-r', '-s', '-v4', world_path]}\n")

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r', '-s', '-v4', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    print("Gazebo client\n")

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    print("Set env vars\n")

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('hunav_gazebo_wrapper'),
                        'models'))
 
    print("Gz server process\n")

    # gzserver_process = ExecuteProcess(
    #     cmd=gzserver_cmd,
    #     output='screen',
    #     shell=True,
    #     on_exit=Shutdown(),
    # )

    # print("Gz client process\n")

    # gzclient_process = ExecuteProcess(
    #     cmd=gzclient_cmd,
    #     output='screen',
    #     shell=True,
    #     on_exit=Shutdown(),
    # )

    # Do not launch Gazebo until the world has been generated
    ordered_launch_event2 = RegisterEventHandler(
        OnProcessStart(
            target_action=hunav_gazebo_worldgen_node,
            on_start=[
                LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
                TimerAction(
                    period=2.0,
                    # actions=[gzserver_process, gzclient_process],
                    actions=[gzserver_cmd, gzclient_cmd],
                )
            ]
        )
    )

    # hunav_manager node
    hunav_manager_node = Node(
        package='hunav_agent_manager',
        executable='hunav_agent_manager',
        name='hunav_agent_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    metrics_file = PathJoinSubstitution([
        FindPackageShare('hunav_evaluator'),
        'config',
        LaunchConfiguration('metrics_file')
    ])

    hunav_evaluator_node = Node(
        package='hunav_evaluator',
        executable='hunav_evaluator_node',
        output='screen',
        parameters=[metrics_file]
    )

    # DO NOT Launch this if any robot localization is launched
    static_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # other option: arguments = "0 0 0 0 0 0 pmb2 base_footprint".split(' ')
    )

    declare_agents_conf_file = DeclareLaunchArgument(
        'configuration_file', default_value='agents_weights_santi_test_1.yaml',
        description='Specify configuration file name in the cofig directory'
    )

    declare_metrics_conf_file = DeclareLaunchArgument(
        'metrics_file', default_value='metrics.yaml',
        description='Specify the name of the metrics configuration file in the cofig directory'
    )

    declare_arg_world = DeclareLaunchArgument(
        # 'base_world', default_value='cafe_santi_test_1.world',
        'base_world', default_value='cafe_santi_test_1_plugin.sdf',
        description='Specify world file name'
    )
    declare_gz_obs = DeclareLaunchArgument(
        'use_gazebo_obs', default_value='true',
        description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    )
    declare_update_rate = DeclareLaunchArgument(
        'update_rate', default_value='100.0',
        description='Update rate of the plugin'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='actor3',
        description='Specify the name of the robot Gazebo model'
    )
    declare_frame_to_publish = DeclareLaunchArgument(
        'global_frame_to_publish', default_value='map',
        description='Name of the global frame in which the position of the agents are provided'
    )
    declare_use_navgoal = DeclareLaunchArgument(
        'use_navgoal_to_start', default_value='false',
        description='Whether to start the agents movements when a navigation goal is received or not'
    )
    declare_ignore_models = DeclareLaunchArgument(
        'ignore_models', default_value='ground_plane cafe',
        description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )


    ld = LaunchDescription()

    # set environment variables
    ld.add_action(set_env_gazebo_model)
    ld.add_action(set_env_gazebo_resource)
    ld.add_action(set_env_gazebo_plugin)

    # Declare the launch arguments
    ld.add_action(declare_agents_conf_file)
    ld.add_action(declare_metrics_conf_file)
    ld.add_action(declare_arg_world)
    ld.add_action(declare_gz_obs)
    ld.add_action(declare_update_rate)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_frame_to_publish)
    ld.add_action(declare_use_navgoal)
    ld.add_action(declare_ignore_models)
    ld.add_action(declare_arg_verbose)

    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    ld.add_action(hunav_loader_node)
    ld.add_action(ordered_launch_event)


    # hunav behavior manager node
    ld.add_action(hunav_manager_node)
    # hunav evaluator
    ld.add_action(hunav_evaluator_node)


    # launch Gazebo after worldGenerator 
    # (wait a bit for the world generation) 
    #ld.add_action(gzserver_process)
    ld.add_action(ordered_launch_event2)
    #ld.add_action(gzclient_process)

    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)

    # spawn robot in Gazebo
    # ld.add_action(spawn_robot)



    return ld

    


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd


# probar a añadir el plugin en el world de test, de momento tengo esta variable export IGN_GAZEBO_RESOURCE_PATH=/home/upo/hunavsim/src/hunav_gazebo_wrapper/worlds