import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import xacro

def generate_launch_description():
    # 获取 URDF 文件路径
    urdf_path = os.path.join(
        get_package_share_directory('damiao_ros2_control'),
        'urdf',
        'damiao_motor.urdf.xacro'
    )

    # 处理 xacro 文件
    robot_description = Command(['xacro ', urdf_path])

    # 控制器配置文件路径（后续创建）
    controller_config = os.path.join(
        get_package_share_directory('damiao_ros2_control'),
        'config',
        'damiao_controllers.yaml'
    )

    # 节点1：发布机器人描述
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 节点2：控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_config],
        output='screen'
    )

    # 节点3：加载位置控制器
    load_joint_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_position_controller', '--controller-manager', '/controller_manager']
    )

    # 节点4：加载状态发布控制器
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_joint_position_controller
    ])
