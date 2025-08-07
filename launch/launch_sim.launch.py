import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # Package name
    package_name = "khepa_bot"

    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name), "launch", "rsp.launch.py"
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "use_ros2_control": "true"}.items(),
    )

    # Joystick and Twist Mux nodes
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch",
                    "joystick.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    # Gazebo Sim
    default_world = os.path.join(
        get_package_share_directory(package_name), "worlds", "empty.world"
    )
    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="World to load",
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Spawner for the robot entity in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "my_bot", "-z", "0.1"],
        output="screen",
    )

    ## CHANGE 2: Define the controller spawners. We will launch them sequentially.

    # This spawner will be launched directly.
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # This spawner will be launched by the event handler AFTER the joint_broad_spawner has finished.
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # The event handler that ensures sequential loading of controllers
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    # Gazebo Bridges
    bridge_params = os.path.join(
        get_package_share_directory(package_name), "config", "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            joystick,
            twist_mux,
            world_arg,
            gazebo,
            spawn_entity,
            ## CHANGE 3: Launch the first spawner and the event handler.
            # Do NOT add 'diff_drive_spawner' here directly.
            joint_broad_spawner,
            delayed_diff_drive_spawner,
            ros_gz_bridge,
            ros_gz_image_bridge,
        ]
    )
