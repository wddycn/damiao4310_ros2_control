import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取功能包路径
    pkg_share = FindPackageShare(package='damiao_ros2_control').find('damiao_ros2_control')
    
    # 核心修改：直接指定纯 URDF 文件路径（不再用xacro）
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'damiao_motor.urdf'])

    # 读取 URDF 文件内容（替代xacro解析）
    with open(os.path.join(pkg_share, 'urdf', 'damiao_motor.urdf'), 'r') as f:
        robot_description = f.read()

    # 控制器配置文件路径
    controller_config_path = PathJoinSubstitution([pkg_share, 'config', 'damiao_controllers.yaml'])

    # 1. 机器人状态发布器（加载纯 URDF）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 2. ros2_control 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_config_path
        ],
        output='screen'
    )

    # 3. 加载并激活关节状态发布器
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--activate'
        ],
        output='screen'
    )

    # 4. 加载并激活位置控制器
    load_joint_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_position_controller',
            '--controller-manager', '/controller_manager',
            '--activate'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_joint_position_controller
    ])
