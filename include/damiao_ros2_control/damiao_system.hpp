#ifndef DAMIAO_ROS2_CONTROL_DAMIAO_SYSTEM_HPP_
#define DAMIAO_ROS2_CONTROL_DAMIAO_SYSTEM_HPP_

// 必须的ROS 2头文件（修复日志/常量未定义）
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_lifecycle/state.hpp"

// ros2_control核心头文件
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"  // 修复HW_IF_*常量

// 直接包含驱动头文件（放弃错误的前置声明）
#include "damiao.h"
#include "SerialPort.h"

#include <memory>
#include <string>
#include <vector>

namespace damiao_ros2_control
{
// 存储电机状态（位置、速度、力矩/电流）
struct MotorData
{
  double position = 0.0;  // 弧度
  double velocity = 0.0;  // rad/s
  double effort = 0.0;    // 电流/A 或 力矩/N·m
};

class DamiaoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DamiaoSystem)  // 修复多余分号警告

  // 构造函数
  DamiaoSystem();

  // 初始化硬件接口
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 导出状态接口（读取电机状态）
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 导出命令接口（下发控制指令）
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 激活硬件（启动电机、打开串口）
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 停止硬件（失能电机、关闭串口）
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 读取电机状态
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 写入控制指令
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 修复不完整类型错误：使用指针/智能指针存储电机对象
  std::unique_ptr<damiao::Motor> motor1_;
  // 串口指针
  std::shared_ptr<SerialPort> serial_ptr_;
  // 电机控制器指针
  std::shared_ptr<damiao::Motor_Control> dm_control_;
  
  // ROS 2日志对象（修复get_logger错误）
  rclcpp::Logger logger_;
  
  // 串口配置
  std::string serial_port_;
  speed_t baud_rate_;  // 修复波特率类型错误（SerialPort需要speed_t类型）

  // 电机状态和命令存储
  std::vector<MotorData> motor_state_;
  std::vector<MotorData> motor_command_;
};

}  // namespace damiao_ros2_control

#endif  // DAMIAO_ROS2_CONTROL_DAMIAO_SYSTEM_HPP_
