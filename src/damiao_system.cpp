#include "damiao_ros2_control/damiao_system.hpp"
#include "unistd.h"
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace damiao_ros2_control
{
// 构造函数：初始化硬件接口
DamiaoSystem::DamiaoSystem()
: hardware_interface::SystemInterface(),
  logger_(rclcpp::get_logger("DamiaoSystem"))  // 初始化日志对象
{
  // 初始化电机1（使用智能指针修复不完整类型错误）
  motor1_ = std::make_unique<damiao::Motor>(damiao::DM4310, 0x01, 0x11);
}

// 配置硬件接口（从参数服务器读取配置，如串口路径、波特率等）
hardware_interface::CallbackReturn DamiaoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 先调用父类初始化
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 从 URDF/参数服务器读取配置（示例：串口路径、波特率）
  try
  {
    serial_port_ = info_.hardware_parameters["serial_port"];  // 串口路径，如 /dev/ttyACM0
    
    // 修复波特率转换：字符串转speed_t（ROS 2参数是字符串，需要手动映射）
    std::string baud_str = info_.hardware_parameters["baud_rate"];
    if (baud_str == "921600") {
      baud_rate_ = B921600;
    } else if (baud_str == "115200") {
      baud_rate_ = B115200;
    } else {
      RCLCPP_ERROR(logger_, "不支持的波特率: %s", baud_str.c_str());
      return CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(logger_, "读取硬件参数失败: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // 初始化电机状态存储（位置、速度、电流）
  motor_state_.resize(info_.joints.size(), {0.0, 0.0, 0.0});
  motor_command_.resize(info_.joints.size(), {0.0, 0.0, 0.0});

  return CallbackReturn::SUCCESS;
}

// 导出状态接口（供 ros2_control 读取电机状态：位置、速度、电流）
std::vector<hardware_interface::StateInterface> DamiaoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // 位置状态接口（使用硬件接口常量）
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motor_state_[i].position));
    // 速度状态接口
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motor_state_[i].velocity));
    // 电流（力矩）状态接口
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motor_state_[i].effort));
  }
  return state_interfaces;
}

// 导出命令接口（供 ros2_control 下发控制指令：位置、速度）
std::vector<hardware_interface::CommandInterface> DamiaoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // 位置命令接口
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motor_command_[i].position));
    // 速度命令接口（可选，根据你的控制模式）
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motor_command_[i].velocity));
  }
  return command_interfaces;
}

// 启动硬件（打开串口、初始化电机、切换控制模式、使能电机）
hardware_interface::CallbackReturn DamiaoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "激活电机硬件...");

  // 1. 初始化串口
  serial_ptr_ = std::make_shared<SerialPort>(serial_port_.c_str(), baud_rate_);
  if (!serial_ptr_)
  {
    RCLCPP_ERROR(logger_, "串口初始化失败！");
    return CallbackReturn::ERROR;
  }

  // 2. 初始化电机控制器
  dm_control_ = std::make_shared<damiao::Motor_Control>(serial_ptr_);
  dm_control_->addMotor(motor1_.get());  // 传递电机指针

  // 3. 先读取电机当前物理位置（关键！覆盖默认0值）
  sleep(1);  // 等待串口稳定
  dm_control_->refresh_motor_status(*motor1_);  // 刷新电机状态
  double current_motor_pos = motor1_->Get_Position();  // 读取电机当前位置（弧度）
  RCLCPP_INFO(logger_, "电机当前物理位置：%.4f rad", current_motor_pos);

  // 4. 覆盖指令变量的默认0值！让控制器认为目标位置=当前位置
  motor_command_[0].position = current_motor_pos;
  motor_state_[0].position = current_motor_pos;  // 状态也同步

  // 5. 切换控制模式+使能（只上电，不下发位置指令）
  if (dm_control_->switchControlMode(*motor1_, damiao::POS_VEL_MODE))
  {
    RCLCPP_INFO(logger_, "切换到 POS_VEL_MODE 成功");
  }
  else
  {
    RCLCPP_ERROR(logger_, "切换控制模式失败！");
    return CallbackReturn::ERROR;
  }

  dm_control_->save_motor_param(*motor1_);
  dm_control_->enable(*motor1_);  // 只使能，不移动
  sleep(1);

  RCLCPP_INFO(logger_, "电机硬件激活成功，保持当前位置！");
  return CallbackReturn::SUCCESS;
}


// 停止硬件（失能电机、关闭串口）
hardware_interface::CallbackReturn DamiaoSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "停止电机硬件...");

  // 失能电机
  if (dm_control_ && motor1_)
  {
    dm_control_->disable(*motor1_);  // 假设你的驱动有 disable 接口，没有则注释
  }

  // 释放串口和控制器资源
  serial_ptr_.reset();
  dm_control_.reset();

  RCLCPP_INFO(logger_, "电机硬件已停止！");
  return CallbackReturn::SUCCESS;
}

// 读取电机状态（将驱动中的电机状态同步到 ros2_control 的状态变量）
hardware_interface::return_type DamiaoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!dm_control_ || !motor1_) {
    RCLCPP_WARN(logger_, "电机控制器未初始化，跳过状态读取");
    return hardware_interface::return_type::ERROR;
  }

  // 刷新电机状态
  dm_control_->refresh_motor_status(*motor1_);

  // 将电机1的状态写入 state 变量（供 ros2_control 读取）
  motor_state_[0].position = motor1_->Get_Position();  // 位置（弧度）
  motor_state_[0].velocity = motor1_->Get_Velocity();  // 速度（rad/s）
  motor_state_[0].effort = motor1_->Get_tau();         // 电流/力矩

  // 打印调试信息（可选）
  RCLCPP_DEBUG(
    logger_,
    "读取电机状态 - POS: %.4f, VEL: %.4f, CUR: %.4f",
    motor_state_[0].position, motor_state_[0].velocity, motor_state_[0].effort);

  return hardware_interface::return_type::OK;
}

// 写入控制指令（将 ros2_control 下发的命令同步到驱动，控制电机）
hardware_interface::return_type DamiaoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!dm_control_ || !motor1_) {
    RCLCPP_WARN(logger_, "电机控制器未初始化，跳过指令下发");
    return hardware_interface::return_type::ERROR;
  }

  // 从 command 变量读取 ros2_control 下发的位置、速度指令，控制电机
  dm_control_->control_pos_vel(
    *motor1_, motor_command_[0].position, 5.0);

  // 打印调试信息（可选）
  RCLCPP_DEBUG(
    logger_,
    "下发电机指令 - POS: %.4f, VEL: %.4f",
    motor_command_[0].position, 5.0);

  return hardware_interface::return_type::OK;
}

}  // namespace damiao_ros2_control

// 注册硬件接口到 ros2_control 插件系统
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  damiao_ros2_control::DamiaoSystem, hardware_interface::SystemInterface)
