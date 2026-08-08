import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_dir_system = get_package_share_directory("mysystem")

    # 默认 xacro 文件路径
    xacro_file = os.path.join(pkg_dir_system, "urdf", "car.urdf.xacro")
    default_rviz_config_path=os.path.join(pkg_dir_system,"config","rviz.rviz")

    action_declare_arg_mode_path=launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=xacro_file,
        description='加载的文件模型路径'
    )

    action_robot_state_publisher=launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description':launch.substitutions.Command([
                'xacro ',launch.substitutions.LaunchConfiguration('model')
                ])}]
    )

    action_joint_state_publisher_gui=launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    action_joint_state_publisher=launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    action_rviz_node=launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d',default_rviz_config_path],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher_gui,
        action_joint_state_publisher,
        action_rviz_node,
    ])
