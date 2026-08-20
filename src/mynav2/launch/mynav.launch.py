import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 配置文件
    amcl_config = os.path.join(
        get_package_share_directory('mynav2'),
        'maps',
        'room.yaml',
    )
    nav_config = os.path.join(
        get_package_share_directory('mynav2'),
        'config',
        'nav2_params.yaml',
    )

    # 路径
    pkg_dir_lidar=get_package_share_directory("lslidar_driver")
    pkg_dir_nav=get_package_share_directory("nav2_bringup")

    # launch路径 bringup_launch.py包括navigation_launch.py和localization_launch.py
    lidar_launch = os.path.join(
        pkg_dir_lidar,"launch","lsn10_launch.py"
    )
    nav_launch = os.path.join(
        pkg_dir_nav,"launch","bringup_launch.py"
    )

    # action
    action_lidar_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            lidar_launch
        )
    )
    action_nav_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav_launch
        ),
        launch_arguments={
            'map': amcl_config,
            'params_file': nav_config,
            'use_localization': 'True',
        }.items()
    )
    action_map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': amcl_config}],
    )
    action_lifecycle_map = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
    )

    # 4) 完美定位：静态 map = odom
    action_static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # return
    return launch.LaunchDescription([
        action_lidar_launch,
        action_map_server,
        action_lifecycle_map,
        action_static_tf,
        action_nav_launch,
    ])