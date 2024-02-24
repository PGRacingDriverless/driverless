#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    GAZEBO_WORLD = os.environ['GAZEBO_WORLD']

    # ===== GAZEBO CLIENT AND SERVER =====
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        GAZEBO_WORLD + '.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # mb gazebo.launch.py starts gzserver and gzclient, hz
    gui = LaunchConfiguration('gui')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Whether to launch the Gazebo GUI or not (headless)')
    
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world' : world, 'gui' : gui}.items()
    )
    # ==============================
    
    # ===== STATE PUBLISHER =====
    # xacro version
    gpu = LaunchConfiguration('gpu')
    organize_cloud = LaunchConfiguration('organize_cloud')

    declare_gpu_cmd = DeclareLaunchArgument(
        'gpu',
        default_value='False',
        description='Whether to use Gazebo gpu_ray or ray')
    declare_organize_cloud_cmd = DeclareLaunchArgument(
        'organize_cloud',
        default_value='False',
        description='Organize PointCloud2 into 2D array with NaN placeholders, otherwise 1D array and leave out invlaid points')
    

    xacro_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro'
    )

    robot_description = Command(['xacro',' ', xacro_path, ' gpu:=', gpu, ' organize_cloud:=', organize_cloud])
    
    # Read urdf from file
    # urdf_path = os.path.join(
    #     get_package_share_directory('turtlebot3_gazebo'),
    #     'urdf',
    #     'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    # )
    
    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    # ==============================

    # ===== GAZEBO SPAWN ENTITY =====
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path_to_model = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='-2.0',
        description='Specify namespace of the robot')
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='-0.5',
        description='Specify namespace of the robot')

    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_' + TURTLEBOT3_MODEL,
            '-file', urdf_path_to_model,   # urdf model version
            # '-topic', 'robot_description', # xacro version
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.03'
        ],
        output='screen',
    )
    # ==============================

    # ===== RVIZ2 START =====
    rviz_config_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'example.rviz'
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    # ==============================

    # ===== RVIZ2 EXIT HANDLER (HZ NAHUJA) =====
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
        )
    )
    # ==============================

    # ===== LAUNCH DESCRIPTION =====
    # Declare the launch description
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_gpu_cmd)
    ld.add_action(declare_organize_cloud_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    # Add the commands to the launch description
    ld.add_action(gazebo_cmd)
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)
    # ==============================

    return ld
