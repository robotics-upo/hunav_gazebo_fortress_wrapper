import os
#from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                            DeclareLaunchArgument, AppendEnvironmentVariable,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
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
    # gz_obs = LaunchConfiguration('use_gazebo_obs')
    # rate = LaunchConfiguration('update_rate')
    # robot_name = LaunchConfiguration('robot_name')
    # global_frame = LaunchConfiguration('global_frame_to_publish')
    # use_navgoal = LaunchConfiguration('use_navgoal_to_start')
    # ignore_models = LaunchConfiguration('ignore_models')
    
    # world base file
    world_file = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_fortress_wrapper'),
        'worlds',
        world_file_name
    ])

    # # agent configuration file
    # agent_conf_file = PathJoinSubstitution([
    #     FindPackageShare('hunav_agent_manager'),
    #     'config',
    #     LaunchConfiguration('configuration_file')
    # ])

    # # Read the yaml file and load the parameters
    # hunav_loader_node = Node(
    #     package='hunav_agent_manager',
    #     executable='hunav_loader',
    #     output='screen',
    #     parameters=[agent_conf_file]
    # )


    # # the node looks for the base_world file in the directory 'worlds'
    # # of the package hunav_gazebo_plugin direclty. So we do not need to 
    # # indicate the path
    # hunav_gazebo_worldgen_node = Node(
    #     package='hunav_gazebo_fortress_wrapper',
    #     executable='hunav_gazebo_world_generator',
    #     output='screen',
    #     parameters=[{'base_world': world_file},
    #     {'use_gazebo_obs': gz_obs},
    #     {'update_rate': rate},
    #     {'robot_name': robot_name},
    #     {'global_frame_to_publish': global_frame},
    #     {'use_navgoal_to_start': use_navgoal},
    #     {'ignore_models': ignore_models}]
    #     #arguments=['--ros-args', '--params-file', conf_file]
    # )

    # ordered_launch_event = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=hunav_loader_node,
    #         on_start=[
    #             LogInfo(msg='HunNavLoader started, launching HuNav_Gazebo_world_generator after 2 seconds...'),
    #             TimerAction(
    #                 period=2.0,
    #                 actions=[hunav_gazebo_worldgen_node],
    #             )
    #         ]
    #     )
    # )



    # Add models to gazebo path
    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_fortress_wrapper'),
        'models',
    ])
    set_env_gz_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            my_gazebo_models)
    
    # Add plugin to gazebo path
    my_gazebo_plugins = PathJoinSubstitution([
        FindPackageShare('hunav_gazebo_fortress_wrapper'),
        'plugins',
    ])
    set_env_gz_plugins = AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            my_gazebo_plugins)
    

    # # the world generator will create this world
    # # in this path
    # generated_world = PathJoinSubstitution([
    #     FindPackageShare('hunav_gazebo_fortress_wrapper'),
    #     'worlds',
    #     'generatedWorld.sdf'
    # ])

    # Gazebo server
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
    )


    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )


     # Do not launch Gazebo until the world has been generated
    # ordered_launch_event2 = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=hunav_gazebo_worldgen_node,
    #         on_start=[
    #             LogInfo(msg='GenerateWorld started, launching Gazebo after 2 seconds...'),
    #             TimerAction(
    #                 period=2.0,
    #                 actions=[gzserver_cmd, gzclient_cmd],
    #             )
    #         ]
    #     )
    # )


    # Declare the launch arguments
    declare_arg_world = DeclareLaunchArgument(
       'base_world', default_value='example_cafe_test.sdf',
       description='Specify world file name'
    )
    # declare_arg_world = DeclareLaunchArgument(
    #     'base_world', default_value='worldTest.sdf',
    #     description='Specify world file name'
    # )
    declare_arg_verbose = DeclareLaunchArgument(
        'verbose', default_value='false',
        description='Set "true" to increase messages written to terminal.'
    )
    # declare_agents_file = DeclareLaunchArgument(
    #     'configuration_file', default_value='agents_test.yaml',
    #     description='Specify configuration file name in the cofig directory'
    # )
    # declare_gz_obs = DeclareLaunchArgument(
    #     'use_gazebo_obs', default_value='true',
    #     description='Whether to fill the agents obstacles with closest Gazebo obstacle or not'
    # )
    # declare_update_rate = DeclareLaunchArgument(
    #     'update_rate', default_value='100.0',
    #     description='Update rate of the plugin'
    # )
    # declare_robot_name = DeclareLaunchArgument(
    #     'robot_name', default_value='actor3',
    #     description='Specify the name of the robot Gazebo model'
    # )
    # declare_frame_to_publish = DeclareLaunchArgument(
    #     'global_frame_to_publish', default_value='map',
    #     description='Name of the global frame in which the position of the agents are provided'
    # )
    # declare_use_navgoal = DeclareLaunchArgument(
    #     'use_navgoal_to_start', default_value='false',
    #     description='Whether to start the agents movements when a navigation goal is received or not'
    # )
    # declare_ignore_models = DeclareLaunchArgument(
    #     'ignore_models', default_value='ground_plane cafe',
    #     description='list of Gazebo models that the agents should ignore as obstacles as the ground_plane. Indicate the models with a blank space between them'
    # )
    

    ld = LaunchDescription()

    # set environment variables
    ld.add_action(set_env_gz_resources)
    ld.add_action(set_env_gz_plugins)

    # Declare the launch arguments
    ld.add_action(declare_arg_world)
    ld.add_action(declare_arg_verbose)
    # ld.add_action(declare_agents_file)
    # ld.add_action(declare_gz_obs)
    # ld.add_action(declare_update_rate)
    # ld.add_action(declare_robot_name)
    # ld.add_action(declare_frame_to_publish)
    # ld.add_action(declare_use_navgoal)
    # ld.add_action(declare_ignore_models)

    # launch Gazebo
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    # Generate the world with the agents
    # launch hunav_loader and the WorldGenerator
    # 2 seconds later
    # ld.add_action(hunav_loader_node)
    # ld.add_action(ordered_launch_event)
    # ld.add_action(ordered_launch_event2)

    return ld


