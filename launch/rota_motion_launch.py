import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node

def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    rota_dir = get_package_share_directory('rota_motion')
    configured_params = RewrittenYaml(
        source_file=os.path.join(rota_dir, 'config', 'nav2_params.yaml'),
        root_key="", param_rewrites="", convert_types=True
    )

    localization = Node(
        package='rota_motion',
        executable='px4_tf_publisher',
        name='px4_tf_publisher',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': os.path.join(rota_dir, 'config', 'map.yaml')},
            {'use_sim_time': False},
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': configured_params,
            'autostart': 'true',
        }.items(),
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}],
    )

    ld = LaunchDescription()
    ld.add_action(localization)
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(navigation)
    return ld