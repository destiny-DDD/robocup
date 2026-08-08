import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory

def generate_launch_description():
    setup_cmd=launch.actions.ExecuteProcess(
        cmd=['socat','-d','-d','pty,raw,echo=0,link=/tmp/vtx','pty,raw,echo=0,link=/tmp/vrx'],
        output='screen'
    )
    config = os.path.join(
        get_package_share_directory('mycontrol'),
        'config',
        'params.yaml',
    )
    action_mycontrol=launch_ros.actions.Node(
        package='mycontrol',
        executable='mycontrol',
        parameters=[config],
        output='screen',
    )
    return launch.LaunchDescription([
        setup_cmd,
        action_mycontrol,
    ])