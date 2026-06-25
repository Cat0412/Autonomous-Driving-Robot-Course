import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    front_2d_lidar_config = os.path.join(
        get_package_share_directory("storagy"),
        "config",
        "sick_scan2",
        "sick_tim_5xx_front.yaml",
    )

    front_depth_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("orbbec_camera"),
                        "launch",
                        "astra_stereo_u3.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "publish_tf": "false",
            # Safety approach uses depth values inside RGB detections.
            # Register depth to the color optical frame in the camera driver.
            "depth_registration": "true",
        }.items(),
    )

    new_front_2d_lidar_config = os.path.join(
        get_package_share_directory("storagy"),
        "config",
        "sick_scan_xd",
        "new_sick_tim_5xx_front.yaml",
    )
    
    front_2d_lidar = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="sick_scan_xd",
        output="screen",
        parameters=[new_front_2d_lidar_config],
        # arguments=[sick_scan_launch_file, 'hostname:=169.254.223.140', 'frame_id:=front_base_scan', 'range_max:=25.0', 'tf_base_frame_id:=base_link'],
    )

    motor_driver = Node(
        package="motor_driver2",
        executable="motor_driver2",
        name="motor_driver2",
    )

    velocity_mux = Node(
        package="fallen_person_safety",
        executable="velocity_mux",
        name="safety_velocity_mux",
        output="screen",
    )

    return LaunchDescription(
        [
            # DeclareLaunchArgument
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            # Launch
            motor_driver,
            velocity_mux,
            front_depth_camera,
            front_2d_lidar,
        ]
    )
