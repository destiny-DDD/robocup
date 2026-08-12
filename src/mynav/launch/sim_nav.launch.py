import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # 配置文件
    sim_config = os.path.join(
        get_package_share_directory('mynav'),
        'config',
        'nav2_params_sim.yaml',
    )
    map_yaml = os.path.join(
        get_package_share_directory('mynav'),
        'maps',
        'room.yaml',
    )

    # 各包路径
    pkg_dir_nav = get_package_share_directory('nav2_bringup')
    pkg_dir_control = get_package_share_directory('mycontrol')
    pkg_dir_system = get_package_share_directory('mysystem')

    nav_launch = os.path.join(pkg_dir_nav, 'launch', 'navigation_launch.py')
    control_launch = os.path.join(pkg_dir_control, 'launch', 'mycontrol.launch.py')
    system_launch = os.path.join(pkg_dir_system, 'launch', 'mysystem.launch.py')

    # 1) 虚拟串口 + mycontrol（odom 回环）
    action_control = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            control_launch))

    # 2) URDF + rviz2（用改过的 rviz2.rviz）
    action_system = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            system_launch))

    # 3) map_server（生命周期节点，用独立 lifecycle manager 激活）
    action_map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}],
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

    # 5) 导航栈（不含 localization/AMCL）
    action_nav = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav_launch),
        launch_arguments={
            'params_file': sim_config,
            'use_sim_time': 'False',
        }.items(),
    )

    return launch.LaunchDescription([
        action_control,
        action_system,
        action_map_server,
        action_lifecycle_map,
        action_static_tf,
        action_nav,
    ])
