import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 配置文件
    nav_config = os.path.join(
        get_package_share_directory('mynav'),
        'maps',
        'room.yaml',
    )

    # 路径
    pkg_dir_nav=get_package_share_directory("nav2_bringup")

    # launch路径
    nav_launch = os.path.join(
        pkg_dir_nav,"launch","localization_launch.py"
    )

    # action
    action_nav_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            nav_launch
        ),
        launch_arguments={'map': nav_config}.items()
    )

    # return
    return launch.LaunchDescription([
        action_nav_launch,
    ])