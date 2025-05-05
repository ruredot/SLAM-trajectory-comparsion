import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # get pkg path 
    fast_lio_path = get_package_share_directory('fast_lio')
    pointcloud_to_laserscan_path = get_package_share_directory('pointcloud_to_laserscan')
    
    default_config_path = os.path.join(fast_lio_path, 'config')
    default_rviz_config_path = os.path.join(fast_lio_path, 'rviz', 'fastlio.rviz')

    # set lauch file arg
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    namespace = LaunchConfiguration('namespace')

    # decalre arg
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='turtlebot4_velodyne.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace for all nodes'
    )

    # LaserScan to PointCloud2 transform node
    laserscan_to_pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'queue_size': 10
        }],
        remappings=[
            ('scan_in', '/scan'),
            ('cloud', '/velodyne_points')
        ],
        output='screen'
    )

    # Fast-LIO node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                   {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    # namespace group
    namespace_group = GroupAction([
        PushRosNamespace(namespace),
        laserscan_to_pointcloud_node,
        fast_lio_node,
        rviz_node
    ])

    # create final launch description 
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_namespace_cmd)
    
    ld.add_action(namespace_group)

    return ld