# 配置功能包
1.安装相关依赖
```bash
sudo apt install ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-controller-manager
```
2.创建功能包
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake damiao_ros2_control \
  --dependencies rclcpp hardware_interface ros2_control lifecycle_msgs
```
3.复制驱动文件：

将原本的 damiao.h、SerialPort.h 复制到 damiao_ros2_control/include/damiao_ros2_control/ 目录下

# 编写 ros2_control 硬件接口（核心）
1.在 damiao_ros2_control/src/ 下创建 damiao_system.cpp:

2.在 damiao_ros2_control/include/damiao_ros2_control/ 下创建 damiao_system.hpp：

3.在 damiao_ros2_control/ 下创建 plugin_description.xml：

4.修改 damiao_ros2_control/CMakeLists.txt，添加编译规则、头文件路径、插件安装等：

5.ros2_control 需要通过 URDF 配置硬件接口和控制器，创建一个 urdf/damiao_motor.urdf.xacro 文件（在 damiao_ros2_control/urdf/ 下，手动创建 urdf 目录）：

6.在 damiao_ros2_control/launch/ 下创建 damiao_control.launch.py（手动创建 launch 目录）：

7.在 damiao_ros2_control/config/ 下创建 damiao_controllers.yaml（手动创建 config 目录）：

# 测试
1.编译
```bash
cd ~/ros2_ws
colcon build --packages-select damiao_ros2_control
source install/setup.bash
```
2.启动控制器
```bash
ros2 launch damiao_ros2_control damiao_control.launch.py
```
3.测试电机控制
```bash
ros2 topic pub /joint_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.5708]" --once
```
4.查看电机状态（关节状态发布器）：
```bash
ros2 topic echo /joint_states
```
# 配置moveit
首先我觉得这个是个很神奇的事

1.在原来的硬件接口功能包里注释掉ros2_control标签，然后根据这个urdf配置一个moveit_config功能包，控制器名字随便起

2.下载依赖
```bash
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-resources-panda-moveit-config
```
3.配置
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
4.一些常见问题
- 一般配置出来的joint_limit里面的参数是int类型，得改成float类型
- moveit_controllers.yaml里面如果没有action_ns参数的话得自己添加
```yaml
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - eyou_arm_controller

  eyou_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory  # 得自己添加
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
```
5.取消掉之前硬件接口urdf文件当中的ros2_control标签注释（所以如果还有其他机械臂也有控制器的话，只改urdf里面的ros2_control标签名称就行了）

6.编译、启动demo.launch就行了

7.但是每次启动之后电机总会先自己回到初始位置，这样很不安全，于是可以在硬件接口的cpp文件当中的on_activate函数当中做下修改，一开始就读一次电机的位置值来覆盖掉默认0位，这样就不会每次一上电就回0位了