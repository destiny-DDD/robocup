import os

import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 现有完美定位导航栈（map_server + 静态 map→odom + nav2_bringup）
    nav_launch = os.path.join(
        get_package_share_directory('mynav'),
        'launch',
        'mynav.launch.py',
    )
    # 多点航点配置
    waypoint_file = os.path.join(
        get_package_share_directory('mynav'),
        'config',
        'waypoints.yaml',
    )

    action_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch)
    )
    action_runner = launch_ros.actions.Node(
        package='mynav',
        executable='waypoint_runner',
        name='waypoint_runner',
        output='screen',
        parameters=[{'waypoint_file': waypoint_file}],
    )

    return launch.LaunchDescription([
        action_nav,
        action_runner,
    ])
