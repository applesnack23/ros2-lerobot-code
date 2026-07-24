import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory(
        "so_arm101_description"
    )

    urdf_path = os.path.join(
        pkg_path,
        "urdf",
        "so_arm101.urdf",
    )

    with open(
        urdf_path,
        "r",
        encoding="utf-8",
    ) as urdf_file:
        robot_description = urdf_file.read()

    ros_gz_sim_path = get_package_share_directory(
        "ros_gz_sim"
    )

    gazebo_resource_path = os.path.dirname(pkg_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_path,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "so_arm101",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gazebo_resource_path,
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])