from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    return LaunchDescription([
        bag_path_arg,
        static_velo,
        static_cam0,
        static_cam1,
        static_cam2,
        static_cam3,
        play_bag
    ])
