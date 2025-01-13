import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Note: We can create nodes here in the same launch file, but for more
    # flexibility and future development, separate launch files for each node
    # have been created and are included here.

    # Envivonment varialbles
    env_action = SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "40")  # 40 - only ERROR

    # The current event (mission) 
    # Possible values (lowercase):
    # - "acceleration" : acceleration test
    # - "skidpad"      : figure-eight handling test
    # - "autocross"    : obstacle avoidance (slalom)
    # - "trackdrive"   : multiple laps on a track
    # - "endurance"    : long-distance reliability and efficiency test
    current_mission = LaunchConfiguration("mission")
    current_mission_arg = DeclareLaunchArgument(
        "mission",
        default_value="trackdrive",
        description="Select current mission"
    )

    # Cone detection
    cone_detection_launch_path = os.path.join(
        get_package_share_directory("cone_detection"),
        "launch",
        "cone_detection.launch.py"
    )
    cone_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            cone_detection_launch_path
        )
    )


    # Path planner
    path_planner_launch_path = os.path.join(
        get_package_share_directory("path_planning"),
        "launch",
        "path_planning.launch.py"
    )
    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path_planner_launch_path
        )
    )


    # Control
    control_launch_path = os.path.join(
        get_package_share_directory("control"),
        "launch",
        "control.launch.py"
    )
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            control_launch_path
        )
    )


    # RViz2
    launch_rviz = LaunchConfiguration("rviz")

    launch_rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Launch RViz2"
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("dv_master_launch"),
        "rviz",
        "dv_master.rviz"
    )
    rviz_node = Node(
        executable="rviz2",
        package="rviz2",
        name="RViz2",
        arguments=["-d", rviz_config_path],
        output="screen",
        # A string will be considered True if it matches 'true' or '1'.
        condition=IfCondition(launch_rviz)
    )


    # Launch description
    launch_description = LaunchDescription()
    launch_description.add_action(env_action)
    launch_description.add_action(current_mission_arg)
    launch_description.add_action(cone_detection_launch)
    launch_description.add_action(path_planner_launch)
    launch_description.add_action(control_launch)
    launch_description.add_action(launch_rviz_arg)
    #launch_description.add_action(rviz_node)

    return launch_description
