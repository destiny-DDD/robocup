import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # 命令行
    # setup_cmd=launch.actions.ExecuteProcess(
    #     cmd=['socat','-d','-d','pty,raw,echo=0,link=/tmp/vtx','pty,raw,echo=0,link=/tmp/vrx'],
    #     output='screen'
    # )

    # 配置文件
    config = os.path.join(
        get_package_share_directory('mycontrol'),
        'config',
        'params_ekf.yaml',
    )
    ekf_config = os.path.join(
        get_package_share_directory('myekf'),
        'config',
        'ekf.yaml',
    )

    # 路径
    pkg_dir_imu=get_package_share_directory("yesense_std_ros2")
    pkg_dir_lidar=get_package_share_directory("lslidar_driver")
    pkg_dir_rf2o=get_package_share_directory("rf2o_laser_odometry")

    # launch路径
    imu_launch = os.path.join(
        pkg_dir_imu,"launch","yesense_node.launch.py"
    )
    lidar_launch = os.path.join(
        pkg_dir_lidar,"launch","lsn10_net_launch.py"
    )
    rf2o_launch = os.path.join(
        pkg_dir_rf2o,"launch","rf2o_laser_odometry.launch.py"
    )

    # action
    action_mycontrol=launch_ros.actions.Node(
        package='mycontrol',
        executable='mycontrol',
        parameters=[config],
        output='screen',
    )
    action_imu_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            imu_launch
        )
    )
    action_lidar_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            lidar_launch
        )
    )
    action_rf2o_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            rf2o_launch
        )
    )
    action_ekf=launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_config],
    )

    # return
    return launch.LaunchDescription([
        # setup_cmd,
        action_mycontrol,
        action_imu_launch,
        action_lidar_launch,
        action_rf2o_launch,
        action_ekf,
    ])