from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/rohan/kitti_bag_seq_00',
        description='Path to the ROS 2 bag'
    )

    play_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_path')],
        output='screen'
    )

    static_velo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velo',
        arguments=['0.8077', '-0.3319', '0.8007',
                   '0.0007553', '0.0020358 ', '-0.0148235', 'base_link', 'velo_link'],
        output='screen'
    )

    static_cam0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_camera',
        arguments=['0.27290', '-0.00197', '-0.07229',
                   '-1.563', '0.0006166', '-1.557', 'velo_link', 'cam0_link'],
        output='screen'
    )

    static_cam1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam0_to_cam1',
        arguments=['0.53716', '0.00529', '-0.00405',
                   '0.01861', '0.03084', '0.00900', 'cam0_link', 'cam1_link'],
        output='screen'
    )

    static_cam2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam0_to_cam2',
        arguments=['-0.05981', '0.0000915', '-0.00231',
                   '0.00525', '-0.00457', '0.00339', 'cam0_link', 'cam2_link'],
        output='screen'
    )

    static_cam3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam0_to_cam3',
        arguments=['0.473103', '-0.002382', '-0.006047',
                   '0.017', '-0.02431', '-0.00181', 'cam0_link', 'cam3_link'],
        output='screen'
    )

    # Define package name and URDF file
    package_name = 'kitti_to_ros2bag'
    xacro_file_name = 'vehicle.xacro'

    # Get the path to the URDF file
    xacro_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        xacro_file_name
    )

    # Process xacro to generate URDF
    robot_description_config = xacro.process_file(xacro_path).toxml()
    vehicle_sim = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    rviz_config_file = 'kitti.rviz'
    # Get the full path to the installed rviz config file
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_config_file
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        bag_path_arg,
        static_velo,
        static_cam0,
        static_cam1,
        static_cam2,
        static_cam3,
        play_bag,
        vehicle_sim,
        rviz_launch
    ])
