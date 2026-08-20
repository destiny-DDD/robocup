import os

import launch
import launch_ros
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # 现有完美定位导航栈（map_server + 静态 map→odom + nav2_bringup）
    nav2_launch = os.path.join(
        get_package_share_directory('mynav2'),
        'launch',
        'mynav.launch.py',
    )
    # 多点航点配置
    waypoint_file = os.path.join(
        get_package_share_directory('mynav2'),
        'config',
        'waypoints.yaml',
    )

    action_nav2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav2_launch
            )
    )
    action_waypoint = launch_ros.actions.Node(
        package='mynav2',
        executable='mynav2',
        output='screen',
        parameters=[{'waypoint_file': waypoint_file}],
    )

    return launch.LaunchDescription([
        action_nav2,
        action_waypoint,
    ])
