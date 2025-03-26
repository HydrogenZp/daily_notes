以下是带有完整详细注释的源代码，按照你的要求，我没有改动原始代码，仅添加了详细的注释以解释代码的功能和逻辑。注释涵盖了每个函数的目的、主要变量的含义以及关键代码段的作用。代码分为头文件和实现文件两部分，分别进行注释。

------

带有注释的头文件 (gimbal_base.h)

cpp

```cpp
/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/16/21.
//

#pragma once

// 包含必要的头文件，提供控制器所需的接口和工具
#include <effort_controllers/joint_velocity_controller.h> // 关节速度控制器
#include <controller_interface/multi_interface_controller.h> // 多接口控制器基类
#include <hardware_interface/joint_command_interface.h> // 关节命令接口
#include <hardware_interface/imu_sensor_interface.h> // IMU传感器接口
#include <rm_common/hardware_interface/robot_state_interface.h> // 机器人状态接口
#include <rm_common/filters/filters.h> // 滤波器工具
#include <rm_msgs/GimbalCmd.h> // 云台命令消息
#include <rm_msgs/TrackData.h> // 跟踪数据消息
#include <rm_msgs/GimbalDesError.h> // 云台目标误差消息
#include <rm_msgs/GimbalPosState.h> // 云台位置状态消息
#include <rm_gimbal_controllers/GimbalBaseConfig.h> // 云台配置参数
#include <rm_gimbal_controllers/bullet_solver.h> // 子弹求解器
#include <tf2_eigen/tf2_eigen.h> // TF2与Eigen的转换工具
#include <Eigen/Eigen> // Eigen线性代数库
#include <control_toolbox/pid.h> // PID控制器
#include <urdf/model.h> // URDF模型解析
#include <dynamic_reconfigure/server.h> // 动态参数调整服务器
#include <realtime_tools/realtime_publisher.h> // 实时发布工具

namespace rm_gimbal_controllers
{
// 定义云台配置结构体，存储控制参数
struct GimbalConfig
{
  double yaw_k_v_, pitch_k_v_, k_chassis_vel_; // 偏航和俯仰的速度增益，底盘速度补偿增益
  double accel_pitch_, accel_yaw_; // 俯仰和偏航的加速度限制
};

// ChassisVel类：管理底盘速度的估计和滤波
class ChassisVel
{
public:
  // 构造函数，初始化底盘速度滤波器
  ChassisVel(const ros::NodeHandle& nh)
  {
    double num_data;
    nh.param("num_data", num_data, 20.0); // 获取滤波器数据点数，默认20
    nh.param("debug", is_debug_, true); // 是否启用调试模式
    linear_ = std::make_shared<Vector3WithFilter<double>>(num_data); // 初始化线速度滤波器
    angular_ = std::make_shared<Vector3WithFilter<double>>(num_data); // 初始化角速度滤波器
    if (is_debug_)
    {
      // 如果启用调试，初始化实时发布器用于输出速度数据
      real_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "real", 1));
      filtered_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(nh, "filtered", 1));
    }
  }
  std::shared_ptr<Vector3WithFilter<double>> linear_; // 线速度滤波器
  std::shared_ptr<Vector3WithFilter<double>> angular_; // 角速度滤波器
  // 更新底盘速度数据
  void update(double linear_vel[3], double angular_vel[3], double period)
  {
    if (period < 0)
      return; // 如果周期无效，直接返回
    if (period > 0.1)
    {
      linear_->clear(); // 如果周期过长，清空滤波器数据
      angular_->clear();
    }
    linear_->input(linear_vel); // 输入线速度数据到滤波器
    angular_->input(angular_vel); // 输入角速度数据到滤波器
    if (is_debug_ && loop_count_ % 10 == 0) // 每10次循环发布一次调试数据
    {
      if (real_pub_->trylock())
      {
        real_pub_->msg_.linear.x = linear_vel[0]; // 发布原始线速度
        real_pub_->msg_.linear.y = linear_vel[1];
        real_pub_->msg_.linear.z = linear_vel[2];
        real_pub_->msg_.angular.x = angular_vel[0]; // 发布原始角速度
        real_pub_->msg_.angular.y = angular_vel[1];
        real_pub_->msg_.angular.z = angular_vel[2];
        real_pub_->unlockAndPublish();
      }
      if (filtered_pub_->trylock())
      {
        filtered_pub_->msg_.linear.x = linear_->x(); // 发布滤波后的线速度
        filtered_pub_->msg_.linear.y = linear_->y();
        filtered_pub_->msg_.linear.z = linear_->z();
        filtered_pub_->msg_.angular.x = angular_->x(); // 发布滤波后的角速度
        filtered_pub_->msg_.angular.y = angular_->y();
        filtered_pub_->msg_.angular.z = angular_->z();
        filtered_pub_->unlockAndPublish();
      }
    }
    loop_count_++; // 更新循环计数器
  }

private:
  bool is_debug_; // 是否启用调试标志
  int loop_count_; // 循环计数器
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist>> real_pub_, filtered_pub_; // 调试发布器
};

// Controller类：云台控制器主类，继承多接口控制器
class Controller : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface,
                                                                         hardware_interface::ImuSensorInterface,
                                                                         hardware_interface::EffortJointInterface>
{
public:
  Controller() = default; // 默认构造函数
  // 初始化函数，设置硬件接口和ROS节点
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  // 启动函数，初始化控制器状态
  void starting(const ros::Time& time) override;
  // 更新函数，主控制循环
  void update(const ros::Time& time, const ros::Duration& period) override;
  // 设置云台目标方向
  void setDes(const ros::Time& time, double yaw_des, double pitch_des);

private:
  // 控制模式函数：RATE模式，基于速度控制
  void rate(const ros::Time& time, const ros::Duration& period);
  // 控制模式函数：TRACK模式，跟踪目标
  void track(const ros::Time& time);
  // 控制模式函数：DIRECT模式，直接设定目标点
  void direct(const ros::Time& time);
  // 控制模式函数：TRAJ模式，跟随预定义轨迹
  void traj(const ros::Time& time);
  // 检查并限制目标角度是否在关节限制范围内
  bool setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                       const urdf::JointConstSharedPtr& joint_urdf);
  // 计算并设置关节控制命令
  void moveJoint(const ros::Time& time, const ros::Duration& period);
  // 计算前馈补偿（重力补偿）
  double feedForward(const ros::Time& time);
  // 更新底盘速度估计
  void updateChassisVel();
  // 命令回调函数，接收云台命令
  void commandCB(const rm_msgs::GimbalCmdConstPtr& msg);
  // 跟踪数据回调函数，接收跟踪数据
  void trackCB(const rm_msgs::TrackDataConstPtr& msg);
  // 动态参数调整回调函数
  void reconfigCB(rm_gimbal_controllers::GimbalBaseConfig& config, uint32_t);

  // 成员变量
  rm_control::RobotStateHandle robot_state_handle_; // 机器人状态句柄
  hardware_interface::ImuSensorHandle imu_sensor_handle_; // IMU传感器句柄
  bool has_imu_ = true; // 是否有IMU传感器
  effort_controllers::JointVelocityController ctrl_yaw_, ctrl_pitch_; // 偏航和俯仰关节速度控制器
  control_toolbox::Pid pid_yaw_pos_, pid_pitch_pos_; // 偏航和俯仰位置PID控制器

  std::shared_ptr<BulletSolver> bullet_solver_; // 子弹轨迹求解器

  // ROS接口相关
  ros::Time last_publish_time_; // 上次发布时间
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>> yaw_pos_state_pub_, pitch_pos_state_pub_; // 偏航和俯仰状态发布器
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>> error_pub_; // 误差发布器
  ros::Subscriber cmd_gimbal_sub_; // 云台命令订阅器
  ros::Subscriber data_track_sub_; // 跟踪数据订阅器
  realtime_tools::RealtimeBuffer<rm_msgs::GimbalCmd> cmd_rt_buffer_; // 实时命令缓冲区
  realtime_tools::RealtimeBuffer<rm_msgs::TrackData> track_rt_buffer_; // 实时跟踪数据缓冲区
  urdf::JointConstSharedPtr pitch_joint_urdf_, yaw_joint_urdf_; // 俯仰和偏航关节的URDF信息

  rm_msgs::GimbalCmd cmd_gimbal_; // 当前云台命令
  rm_msgs::TrackData data_track_; // 当前跟踪数据
  std::string gimbal_des_frame_id_, imu_name_; // 云台目标帧ID和IMU名称
  double publish_rate_; // 发布频率
  bool state_changed_; // 状态是否改变标志
  bool pitch_des_in_limit_, yaw_des_in_limit_; // 俯仰和偏航目标是否在限制范围内
  int loop_count_; // 循环计数器

  // 坐标变换相关
  geometry_msgs::TransformStamped odom2gimbal_des_, odom2pitch_, odom2base_, last_odom2base_; // 坐标变换数据

  // 重力补偿相关
  geometry_msgs::Vector3 mass_origin_; // 质量中心位置
  double gravity_; // 重力加速度
  bool enable_gravity_compensation_; // 是否启用重力补偿

  // 底盘相关
  std::shared_ptr<ChassisVel> chassis_vel_; // 底盘速度对象

  // 动态参数调整相关
  bool dynamic_reconfig_initialized_; // 动态参数是否初始化
  GimbalConfig config_; // 当前配置
  realtime_tools::RealtimeBuffer<GimbalConfig> config_rt_buffer_; // 实时配置缓冲区
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>* d_srv_; // 动态参数服务器

  RampFilter<double>* ramp_rate_pitch_, *ramp_rate_yaw_; // 俯仰和偏航速率的斜坡滤波器

  // 控制模式枚举
  enum
  {
    RATE,  // 速率模式
    TRACK, // 跟踪模式
    DIRECT, // 直接模式
    TRAJ   // 轨迹模式
  };
  int state_ = RATE; // 当前控制模式，默认为RATE
  bool start_ = false; // 是否初次启动标志
};

}  // namespace rm_gimbal_controllers
```

------

带有注释的实现文件 (gimbal_base.cpp)

新底盘速度数据
  void update(double linear_cpp

```cpp
/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditionsstad::cout << "modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/16/21.
//

// 包含头文件
#include "rm_gimbal_controllers/gimbal_base.h"
#include <string>
#include <angles/angles.h> // 角度计算工具
#include <rm_common/ros_utilities.h> // ROS常用工具
#include <rm_common/ori_tool.h> // 方向工具
#include <pluginlib/class_list_macros.hpp> // 插件宏定义
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // TF2与几何消息转换
#include <tf/transform_datatypes.h> // TF变换数据类型

namespace rm_gimbal_controllers
{
/**
 * @brief 初始化云台控制器，设置硬件接口、参数和ROS节点
 * @param robot_hw 机器人硬件接口指针
 * @param root_nh ROS根节点句柄
 * @param controller_nh 控制器专用节点句柄
 * @return 初始化是否成功
 */
bool Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  XmlRpc::XmlRpcValue xml_rpc_value;
  bool enable_feedforward;
  // 检查是否启用前馈控制
  enable_feedforward = controller_nh.getParam("feedforward", xml_rpc_value);
  if (enable_feedforward)
  {
    // 如果启用前馈控制，确保参数中包含必要的子参数
    ROS_ASSERT(xml_rpc_value.hasMember("mass_origin"));
    ROS_ASSERT(xml_rpc_value.hasMember("gravity"));
    ROS_ASSERT(xml_rpc_value.hasMember("enable_gravity_compensation"));
  }
  // 初始化质量中心和重力参数，若未启用前馈则设为0
  mass_origin_.x = enable_feedforward ? (double)xml_rpc_value["mass_origin"][0] : 0.;
  mass_origin_.z = enable_feedforward ? (double)xml_rpc_value["mass_origin"][2] : 0.;
  gravity_ = enable_feedforward ? (double)xml_rpc_value["gravity"] : 0.;
  enable_gravity_compensation_ = enable_feedforward && (bool)xml_rpc_value["enable_gravity_compensation"];

  // 设置底盘速度和子弹求解器的节点句柄并初始化
  ros::NodeHandle chassis_vel_nh(controller_nh, "chassis_vel");
  chassis_vel_ = std::make_shared<ChassisVel>(chassis_vel_nh);
  ros::NodeHandle nh_bullet_solver = ros::NodeHandle(controller_nh, "bullet_solver");
  bullet_solver_ = std::make_shared<BulletSolver>(nh_bullet_solver);

  // 设置偏航和俯仰控制的节点句柄
  ros::NodeHandle nh_yaw = ros::NodeHandle(controller_nh, "yaw");
  ros::NodeHandle nh_pitch = ros::NodeHandle(controller_nh, "pitch");
  ros::NodeHandle nh_pid_yaw_pos = ros::NodeHandle(controller_nh, "yaw/pid_pos");
  ros::NodeHandle nh_pid_pitch_pos = ros::NodeHandle(controller_nh, "pitch/pid_pos");

  // 从参数服务器获取配置参数并初始化
  config_ = { .yaw_k_v_ = getParam(nh_yaw, "k_v", 0.),           // 偏航速度增益
              .pitch_k_v_ = getParam(nh_pitch, "k_v", 0.),       // 俯仰速度增益
              .k_chassis_vel_ = getParam(controller_nh, "yaw/k_chassis_vel", 0.), // 底盘速度补偿增益
              .accel_pitch_ = getParam(controller_nh, "pitch/accel", 99.), // 俯仰加速度限制
              .accel_yaw_ = getParam(controller_nh, "yaw/accel", 99.) };   // 偏航加速度限制
  config_rt_buffer_.initRT(config_); // 初始化实时配置缓冲区
  // 设置动态参数调整服务器
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::GimbalBaseConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); }; // 定义回调函数
  d_srv_->setCallback(cb);

  // 获取努力关节接口
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  // 初始化偏航和俯仰控制器以及位置PID控制器
  if (!ctrl_yaw_.init(effort_joint_interface, nh_yaw) || !ctrl_pitch_.init(effort_joint_interface, nh_pitch) ||
      !pid_yaw_pos_.init(nh_pid_yaw_pos) || !pid_pitch_pos_.init(nh_pid_pitch_pos))
    return false;

  // 获取机器人状态句柄
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  // 检查是否配置了IMU传感器
  if (!controller_nh.hasParam("imu_name"))
    has_imu_ = false;
  if (has_imu_)
  {
    imu_name_ = getParam(controller_nh, "imu_name", static_cast<std::string>("gimbal_imu")); // 获取IMU名称
    hardware_interface::ImuSensorInterface* imu_sensor_interface =
        robot_hw->get<hardware_interface::ImuSensorInterface>();
    imu_sensor_handle_ = imu_sensor_interface->getHandle(imu_name_); // 获取IMU句柄
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu."); // 如果无IMU，使用电机数据
  }

  // 加载URDF模型以获取关节信息
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", controller_nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  pitch_joint_urdf_ = urdf.getJoint(ctrl_pitch_.getJointName()); // 获取俯仰关节信息
  yaw_joint_urdf_ = urdf.getJoint(ctrl_yaw_.getJointName());    // 获取偏航关节信息
  if (!pitch_joint_urdf_)
  {
    ROS_ERROR("Could not find joint pitch in urdf");
    return false;
  }
  if (!yaw_joint_urdf_)
  {
    ROS_ERROR("Could not find joint yaw in urdf");
    return false;
  }

  // 设置坐标变换帧
  gimbal_des_frame_id_ = pitch_joint_urdf_->child_link_name + "_des"; // 云台目标帧ID
  odom2gimbal_des_.header.frame_id = "odom"; // odom到目标帧的变换
  odom2gimbal_des_.child_frame_id = gimbal_des_frame_id_;
  odom2gimbal_des_.transform.rotation.w = 1.; // 初始化为单位四元数
  odom2pitch_.header.frame_id = "odom"; // odom到俯仰帧的变换
  odom2pitch_.child_frame_id = pitch_joint_urdf_->child_link_name;
  odom2pitch_.transform.rotation.w = 1.;
  odom2base_.header.frame_id = "odom"; // odom到基座帧的变换
  odom2base_.child_frame_id = yaw_joint_urdf_->parent_link_name;
  odom2base_.transform.rotation.w = 1.;

  // 设置订阅器
  cmd_gimbal_sub_ = controller_nh.subscribe<rm_msgs::GimbalCmd>("command", 1, &Controller::commandCB, this); // 云台命令订阅
  data_track_sub_ = controller_nh.subscribe<rm_msgs::TrackData>("/track", 1, &Controller::trackCB, this);    // 跟踪数据订阅
  // 设置发布频率和发布器
  publish_rate_ = getParam(controller_nh, "publish_rate", 100.);
  error_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalDesError>(controller_nh, "error", 100)); // 误差发布器
  yaw_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_yaw, "pos_state", 1)); // 偏航状态发布器
  pitch_pos_state_pub_.reset(new realtime_tools::RealtimePublisher<rm_msgs::GimbalPosState>(nh_pitch, "pos_state", 1)); // 俯仰状态发布器

  // 初始化速率斜坡滤波器
  ramp_rate_pitch_ = new RampFilter<double>(0, 0.001); // 俯仰速率滤波器，初始值为0，步长0.001
  ramp_rate_yaw_ = new RampFilter<double>(0, 0.001);   // 偏航速率滤波器

  return true; // 初始化成功
}

/**
 * @brief 启动控制器，初始化状态
 * @param time 当前时间（未使用）
 */
void Controller::starting(const ros::Time& /*unused*/)
{
  state_ = RATE;       // 设置初始状态为RATE模式
  state_changed_ = true; // 标记状态已改变
  start_ = true;       // 标记为初次启动
}

/**
 * @brief 主控制循环，周期性调用以更新云台控制
 * @param time 当前时间
 * @param period 控制周期
 */
void Controller::update(const ros::Time& time, const ros::Duration& period)
{
  // 从实时缓冲区读取命令和跟踪数据
  cmd_gimbal_ = *cmd_rt_buffer_.readFromRT();
  data_track_ = *track_rt_buffer_.readFromNonRT();
  config_ = *config_rt_buffer_.readFromRT(); // 读取当前配置
  // 设置斜坡滤波器的加速度
  ramp_rate_pitch_->setAcc(config_.accel_pitch_);
  ramp_rate_yaw_->setAcc(config_.accel_yaw_);
  // 输入并滤波速率命令
  ramp_rate_pitch_->input(cmd_gimbal_.rate_pitch);
  ramp_rate_yaw_->input(cmd_gimbal_.rate_yaw);
  cmd_gimbal_.rate_pitch = ramp_rate_pitch_->output();
  cmd_gimbal_.rate_yaw = ramp_rate_yaw_->output();
  // 更新坐标变换
  try
  {
    odom2pitch_ = robot_state_handle_.lookupTransform("odom", pitch_joint_urdf_->child_link_name, time); // odom到俯仰的变换
    odom2base_ = robot_state_handle_.lookupTransform("odom", yaw_joint_urdf_->parent_link_name, time);  // odom到基座的变换
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "%s\n", ex.what()); // 如果变换失败，警告并返回
    return;
  }
  updateChassisVel(); // 更新底盘速度估计
  // 检查状态是否改变
  if (state_ != cmd_gimbal_.mode)
  {
    state_ = cmd_gimbal_.mode;
    state_changed_ = true;
  }
  // 根据当前状态调用相应的控制函数
  switch (state_)
  {
    case RATE:
      rate(time, period); // 速率模式
      break;
    case TRACK:
      track(time);        // 跟踪模式
      break;
    case DIRECT:
      direct(time);       // 直接模式
      break;
    case TRAJ:
      traj(time);         // 轨迹模式
      break;
  }
  moveJoint(time, period); // 移动关节
}

/**
 * @brief 设置云台的目标方向，考虑关节限制
 * @param time 当前时间
 * @param yaw_des 目标偏航角度
 * @param pitch_des 目标俯仰角度
 */
void Controller::setDes(const ros::Time& time, double yaw_des, double pitch_des)
{
  tf2::Quaternion odom2base, odom2gimbal_des;
  tf2::Quaternion base2gimbal_des;
  tf2::fromMsg(odom2base_.transform.rotation, odom2base); // 将odom到基座的旋转转换为四元数
  odom2gimbal_des.setRPY(0, pitch_des, yaw_des); // 设置目标方向的四元数（Roll固定为0）
  base2gimbal_des = odom2base.inverse() * odom2gimbal_des; // 计算基座到目标的相对方向
  double roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw;
  // 将基座到目标的四元数转换为欧拉角
  quatToRPY(toMsg(base2gimbal_des), roll_temp, base2gimbal_current_des_pitch, base2gimbal_current_des_yaw);
  double pitch_real_des, yaw_real_des;

  // 检查俯仰目标是否在限制范围内
  pitch_des_in_limit_ = setDesIntoLimit(pitch_real_des, pitch_des, base2gimbal_current_des_pitch, pitch_joint_urdf_);
  if (!pitch_des_in_limit_) // 如果超出限制
  {
    double yaw_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = pitch_joint_urdf_->limits ? pitch_joint_urdf_->limits->upper : 1e16; // 上限
    lower_limit = pitch_joint_urdf_->limits ? pitch_joint_urdf_->limits->lower : -1e16; // 下限
    // 选择距离最近的限制边界
    base2new_des.setRPY(0,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_pitch, lower_limit)) ?
                            upper_limit :
                            lower_limit,
                        base2gimbal_current_des_yaw);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_real_des, yaw_temp); // 计算调整后的俯仰
  }

  // 检查偏航目标是否在限制范围内
  yaw_des_in_limit_ = setDesIntoLimit(yaw_real_des, yaw_des, base2gimbal_current_des_yaw, yaw_joint_urdf_);
  if (!yaw_des_in_limit_) // 如果超出限制
  {
    double pitch_temp;
    tf2::Quaternion base2new_des;
    double upper_limit, lower_limit;
    upper_limit = yaw_joint_urdf_->limits ? yaw_joint_urdf_->limits->upper : 1e16; // 上限
    lower_limit = yaw_joint_urdf_->limits ? yaw_joint_urdf_->limits->lower : -1e16; // 下限
    // 选择距离最近的限制边界
    base2new_des.setRPY(0, base2gimbal_current_des_pitch,
                        std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, upper_limit)) <
                                std::abs(angles::shortest_angular_distance(base2gimbal_current_des_yaw, lower_limit)) ?
                            upper_limit :
                            lower_limit);
    quatToRPY(toMsg(odom2base * base2new_des), roll_temp, pitch_temp, yaw_real_des); // 计算调整后的偏航
  }

  // 设置最终的目标变换
  odom2gimbal_des_.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0., pitch_real_des, yaw_real_des);
  odom2gimbal_des_.header.stamp = time;
  robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers"); // 更新机器人状态
}

/**
 * @brief RATE模式：基于速度控制云台
 * @param time 当前时间
 * @param period 控制周期
 */
void Controller::rate(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_) // 进入RATE模式时
  {
    state_changed_ = false; // 重置状态改变标志
    ROS_INFO("[Gimbal] Enter RATE"); // 输出日志
    if (start_) // 如果是初次启动
    {
      odom2gimbal_des_.transform.rotation = odom2pitch_.transform.rotation; // 初始化目标为当前方向
      odom2gimbal_des_.header.stamp = time;
      robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers");
      start_ = false; // 重置初次启动标志
    }
  }
  else
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(odom2gimbal_des_.transform.rotation, roll, pitch, yaw); // 获取当前目标欧拉角
    // 根据速率命令更新目标方向
    setDes(time, yaw + period.toSec() * cmd_gimbal_.rate_yaw, pitch + period.toSec() * cmd_gimbal_.rate_pitch);
  }
}

/**
 * @brief TRACK模式：跟踪目标
 * @param time 当前时间
 */
void Controller::track(const ros::Time& time)
{
  if (state_changed_) // 进入TRACK模式时
  {
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRACK");
  }
  double roll_real, pitch_real, yaw_real;
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real); // 获取当前实际方向
  double yaw_compute = yaw_real;
  double pitch_compute = -pitch_real;
  geometry_msgs::Point target_pos = data_track_.position; // 获取目标位置
  geometry_msgs::Vector3 target_vel{};
  if (data_track_.id != 12)
    target_vel = data_track_.velocity; // 如果目标ID不是12，获取目标速度
  try
  {
    if (!data_track_.header.frame_id.empty()) // 如果目标有参考帧
    {
      // 将目标位置和速度转换到odom帧
      geometry_msgs::TransformStamped transform =
          robot_state_handle_.lookupTransform("odom", data_track_.header.frame_id, data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
    }
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what()); // 如果变换失败，输出警告
  }
  // 根据时间差调整偏航角度
  double yaw = data_track_.yaw + data_track_.v_yaw * ((time - data_track_.header.stamp).toSec());
  while (yaw > M_PI)
    yaw -= 2 * M_PI; // 规范化偏航角度到[-π, π]
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  // 更新目标位置，考虑时间差和当前云台位置
  target_pos.x += target_vel.x * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.x;
  target_pos.y += target_vel.y * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.y;
  target_pos.z += target_vel.z * (time - data_track_.header.stamp).toSec() - odom2pitch_.transform.translation.z;
  // 减去底盘速度影响
  target_vel.x -= chassis_vel_->linear_->x();
  target_vel.y -= chassis_vel_->linear_->y();
  target_vel.z -= chassis_vel_->linear_->z();
  // 使用子弹求解器计算目标偏航和俯仰
  bool solve_success = bullet_solver_->solve(target_pos, target_vel, cmd_gimbal_.bullet_speed, yaw, data_track_.v_yaw,
                                             data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                             data_track_.armors_num, chassis_vel_->angular_->z());
  bullet_solver_->judgeShootBeforehand(time, data_track_.v_yaw); // 判断是否提前射击

  // 发布误差和状态信息
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    if (error_pub_->trylock())
    {
      // 计算云台误差
      double error =
          bullet_solver_->getGimbalError(target_pos, target_vel, data_track_.yaw, data_track_.v_yaw,
                                         data_track_.radius_1, data_track_.radius_2, data_track_.dz,
                                         data_track_.armors_num, yaw_compute, pitch_compute, cmd_gimbal_.bullet_speed);
      error_pub_->msg_.stamp = time;
      error_pub_->msg_.error = solve_success ? error : 1.0; // 如果求解失败，误差设为1.0
      error_pub_->unlockAndPublish();
    }
    bullet_solver_->bulletModelPub(odom2pitch_, time); // 发布子弹模型
    last_publish_time_ = time;
  }

  if (solve_success)
    setDes(time, bullet_solver_->getYaw(), bullet_solver_->getPitch()); // 设置求解出的目标方向
  else
  {
    odom2gimbal_des_.header.stamp = time;
    robot_state_handle_.setTransform(odom2gimbal_des_, "rm_gimbal_controllers"); // 如果求解失败，保持当前方向
  }
}

/**
 * @brief DIRECT模式：直接设定目标点
 * @param time 当前时间
 */
void Controller::direct(const ros::Time& time)
{
  if (state_changed_) // 进入DIRECT模式时
  {
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter DIRECT");
  }
  geometry_msgs::Point aim_point_odom = cmd_gimbal_.target_pos.point; // 获取目标点
  try
  {
    if (!cmd_gimbal_.target_pos.header.frame_id.empty()) // 如果目标点有参考帧
    {
      // 将目标点转换到odom帧
      tf2::doTransform(aim_point_odom, aim_point_odom,
                       robot_state_handle_.lookupTransform("odom", cmd_gimbal_.target_pos.header.frame_id,
                                                           cmd_gimbal_.target_pos.header.stamp));
    }
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
  // 计算目标偏航和俯仰角度
  double yaw = std::atan2(aim_point_odom.y - odom2pitch_.transform.translation.y,
                          aim_point_odom.x - odom2pitch_.transform.translation.x);
  double pitch = -std::atan2(aim_point_odom.z - odom2pitch_.transform.translation.z,
                             std::sqrt(std::pow(aim_point_odom.x - odom2pitch_.transform.translation.x, 2) +
                                       std::pow(aim_point_odom.y - odom2pitch_.transform.translation.y, 2)));
  setDes(time, yaw, pitch); // 设置目标方向
}

/**
 * @brief TRAJ模式：跟随预定义轨迹
 * @param time 当前时间
 */
void Controller::traj(const ros::Time& time)
{
  if (state_changed_) // 进入TRAJ模式时
  {
    state_changed_ = false;
    ROS_INFO("[Gimbal] Enter TRAJ");
  }
  setDes(time, cmd_gimbal_.traj_yaw, cmd_gimbal_.traj_pitch); // 设置轨迹目标方向
}

/**
 * @brief 检查目标角度是否在关节限制范围内
 * @param real_des 实际目标角度（输出）
 * @param current_des 当前目标角度
 * @param base2gimbal_current_des 基座到云台的当前目标角度
 * @param joint_urdf 关节URDF信息
 * @return 是否在限制范围内
 */
bool Controller::setDesIntoLimit(double& real_des, double current_des, double base2gimbal_current_des,
                                 const urdf::JointConstSharedPtr& joint_urdf)
{
  double upper_limit, lower_limit;
  upper_limit = joint_urdf->limits ? joint_urdf->limits->upper : 1e16; // 获取上限，默认为大值
  lower_limit = joint_urdf->limits ? joint_urdf->limits->lower : -1e16; // 获取下限，默认为小值
  // 检查当前目标是否在限制范围内（包括2π补角）
  if ((base2gimbal_current_des <= upper_limit && base2gimbal_current_des >= lower_limit) ||
      (angles::two_pi_complement(base2gimbal_current_des) <= upper_limit &&
       angles::two_pi_complement(base2gimbal_current_des) >= lower_limit))
    real_des = current_des; // 如果在范围内，使用当前目标
  else
    return false; // 否则返回false
  return true;
}

/**
 * @brief 计算并设置关节控制命令
 * @param time 当前时间
 * @param period 控制周期
 */
void Controller::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, angular_vel_pitch, angular_vel_yaw;
  if (has_imu_) // 如果有IMU传感器
  {
    gyro.x = imu_sensor_handle_.getAngularVelocity()[0]; // 获取IMU角速度
    gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
    gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
    try
    {
      // 将IMU角速度转换到俯仰和偏航帧
      tf2::doTransform(gyro, angular_vel_pitch,
                       robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
      tf2::doTransform(gyro, angular_vel_yaw,
                       robot_state_handle_.lookupTransform(yaw_joint_urdf_->child_link_name,
                                                           imu_sensor_handle_.getFrameId(), time));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  else // 如果无IMU，使用电机速度
  {
    angular_vel_yaw.z = ctrl_yaw_.joint_.getVelocity();
    angular_vel_pitch.y = ctrl_pitch_.joint_.getVelocity();
  }
  double roll_real, pitch_real, yaw_real, roll_des, pitch_des, yaw_des;
  quatToRPY(odom2gimbal_des_.transform.rotation, roll_des, pitch_des, yaw_des); // 获取目标欧拉角
  quatToRPY(odom2pitch_.transform.rotation, roll_real, pitch_real, yaw_real);  // 获取实际欧拉角
  // 计算角度误差
  double yaw_angle_error = angles::shortest_angular_distance(yaw_real, yaw_des);
  double pitch_angle_error = angles::shortest_angular_distance(pitch_real, pitch_des);
  pid_pitch_pos_.computeCommand(pitch_angle_error, period); // 计算俯仰PID命令
  pid_yaw_pos_.computeCommand(yaw_angle_error, period);     // 计算偏航PID命令

  double yaw_vel_des = 0., pitch_vel_des = 0.; // 初始化目标速度
  if (state_ == RATE) // RATE模式下使用命令速度
  {
    yaw_vel_des = cmd_gimbal_.rate_yaw;
    pitch_vel_des = cmd_gimbal_.rate_pitch;
  }
  else if (state_ == TRACK) // TRACK模式下计算目标速度
  {
    geometry_msgs::Point target_pos;
    geometry_msgs::Vector3 target_vel;
    // 获取选定装甲板的位置和速度
    bullet_solver_->getSelectedArmorPosAndVel(target_pos, target_vel, data_track_.position, data_track_.velocity,
                                              data_track_.yaw, data_track_.v_yaw, data_track_.radius_1,
                                              data_track_.radius_2, data_track_.dz, data_track_.armors_num);
    tf2::Vector3 target_pos_tf, target_vel_tf;
    try
    {
      // 将目标位置和速度转换到基座帧
      geometry_msgs::TransformStamped transform = robot_state_handle_.lookupTransform(
          yaw_joint_urdf_->parent_link_name, data_track_.header.frame_id, data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
      tf2::fromMsg(target_pos, target_pos_tf);
      tf2::fromMsg(target_vel, target_vel_tf);
      // 计算偏航目标速度
      yaw_vel_des = target_pos_tf.cross(target_vel_tf).z() / std::pow((target_pos_tf.length()), 2);
      transform = robot_state_handle_.lookupTransform(pitch_joint_urdf_->parent_link_name, data_track_.header.frame_id,
                                                      data_track_.header.stamp);
      tf2::doTransform(target_pos, target_pos, transform);
      tf2::doTransform(target_vel, target_vel, transform);
      tf2::fromMsg(target_pos, target_pos_tf);
      tf2::fromMsg(target_vel, target_vel_tf);
      // 计算俯仰目标速度
      pitch_vel_des = target_pos_tf.cross(target_vel_tf).y() / std::pow((target_pos_tf.length()), 2);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
  if (!pitch_des_in_limit_)
    pitch_vel_des = 0.; // 如果俯仰超出限制，速度设为0
  if (!yaw_des_in_limit_)
    yaw_vel_des = 0.;   // 如果偏航超出限制，速度设为0

  pid_pitch_pos_.computeCommand(pitch_angle_error, period); // 再次计算PID命令（可能冗余）
  pid_yaw_pos_.computeCommand(yaw_angle_error, period);

  // 发布状态信息
  if (loop_count_ % 10 == 0) // 每10次循环发布一次
  {
    if (yaw_pos_state_pub_ && yaw_pos_state_pub_->trylock())
    {
      yaw_pos_state_pub_->msg_.header.stamp = time;
      yaw_pos_state_pub_->msg_.set_point = yaw_des; // 目标角度
      yaw_pos_state_pub_->msg_.set_point_dot = yaw_vel_des; // 目标速度
      yaw_pos_state_pub_->msg_.process_value = yaw_real; // 实际角度
      yaw_pos_state_pub_->msg_.error = angles::shortest_angular_distance(yaw_real, yaw_des); // 角度误差
      yaw_pos_state_pub_->msg_.command = pid_yaw_pos_.getCurrentCmd(); // PID命令
      yaw_pos_state_pub_->unlockAndPublish();
    }
    if (pitch_pos_state_pub_ && pitch_pos_state_pub_->trylock())
    {
      pitch_pos_state_pub_->msg_.header.stamp = time;
      pitch_pos_state_pub_->msg_.set_point = pitch_des;
      pitch_pos_state_pub_->msg_.set_point_dot = pitch_vel_des;
      pitch_pos_state_pub_->msg_.process_value = pitch_real;
      pitch_pos_state_pub_->msg_.error = angles::shortest_angular_distance(pitch_real, pitch_des);
      pitch_pos_state_pub_->msg_.command = pid_pitch_pos_.getCurrentCmd();
      pitch_pos_state_pub_->unlockAndPublish();
    }
  }
  loop_count_++; // 更新计数器

  // 设置偏航和俯仰关节命令，包括PID、前馈和底盘补偿
  ctrl_yaw_.setCommand(pid_yaw_pos_.getCurrentCmd() - config_.k_chassis_vel_ * chassis_vel_->angular_->z() +
                       config_.yaw_k_v_ * yaw_vel_des + ctrl_yaw_.joint_.getVelocity() - angular_vel_yaw.z);
  ctrl_pitch_.setCommand(pid_pitch_pos_.getCurrentCmd() + config_.pitch_k_v_ * pitch_vel_des +
                         ctrl_pitch_.joint_.getVelocity() - angular_vel_pitch.y);

  ctrl_yaw_.update(time, period); // 更新偏航控制器
  ctrl_pitch_.update(time, period); // 更新俯仰控制器
  ctrl_pitch_.joint_.setCommand(ctrl_pitch_.joint_.getCommand() + feedForward(time)); // 添加重力前馈补偿
}

/**
 * @brief 计算重力前馈补偿
 * @param time 当前时间
 * @return 前馈补偿值
 */
double Controller::feedForward(const ros::Time& time)
{
  Eigen::Vector3d gravity(0, 0, -gravity_); // 重力向量
  // 将重力向量转换到俯仰帧
  tf2::doTransform(gravity, gravity,
                   robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name, "base_link", time));
  Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z); // 质量中心向量
  double feedforward = -mass_origin.cross(gravity).y(); // 计算重力引起的扭矩
  if (enable_gravity_compensation_) // 如果启用重力补偿
  {
    Eigen::Vector3d gravity_compensation(0, 0, gravity_);
    // 将补偿向量转换到俯仰帧
    tf2::doTransform(gravity_compensation, gravity_compensation,
                     robot_state_handle_.lookupTransform(pitch_joint_urdf_->child_link_name,
                                                         pitch_joint_urdf_->parent_link_name, time));
    feedforward -= mass_origin.cross(gravity_compensation).y(); // 添加补偿扭矩
  }
  return feedforward;
}

/**
 * @brief 更新底盘速度估计
 */
void Controller::updateChassisVel()
{
  // 计算变换周期
  double tf_period = odom2base_.header.stamp.toSec() - last_odom2base_.header.stamp.toSec();
  // 计算线速度
  double linear_x = (odom2base_.transform.translation.x - last_odom2base_.transform.translation.x) / tf_period;
  double linear_y = (odom2base_.transform.translation.y - last_odom2base_.transform.translation.y) / tf_period;
  double linear_z = (odom2base_.transform.translation.z - last_odom2base_.transform.translation.z) / tf_period;
  double last_angular_position_x, last_angular_position_y, last_angular_position_z, angular_position_x,
      angular_position_y, angular_position_z;
  // 获取当前和上次的欧拉角
  quatToRPY(odom2base_.transform.rotation, angular_position_x, angular_position_y, angular_position_z);
  quatToRPY(last_odom2base_.transform.rotation, last_angular_position_x, last_angular_position_y,
            last_angular_position_z);
  // 计算角速度
  double angular_x = angles::shortest_angular_distance(last_angular_position_x, angular_position_x) / tf_period;
  double angular_y = angles::shortest_angular_distance(last_angular_position_y, angular_position_y) / tf_period;
  double angular_z = angles::shortest_angular_distance(last_angular_position_z, angular_position_z) / tf_period;
  double linear_vel[3]{ linear_x, linear_y, linear_z };
  double angular_vel[3]{ angular_x, angular_y, angular_z };
  chassis_vel_->update(linear_vel, angular_vel, tf_period); // 更新底盘速度
  last_odom2base_ = odom2base_; // 保存当前变换
}

/**
 * @brief 云台命令回调函数
 * @param msg 云台命令消息
 */
void Controller::commandCB(const rm_msgs::GimbalCmdConstPtr& msg)
{
  cmd_rt_buffer_.writeFromNonRT(*msg); // 将命令写入实时缓冲区
}

/**
 * @brief 跟踪数据回调函数
 * @param msg 跟踪数据消息
 */
void Controller::trackCB(const rm_msgs::TrackDataConstPtr& msg)
{
  if (msg->id == 0)
    return; // 如果ID为0，无效数据，直接返回
  track_rt_buffer_.writeFromNonRT(*msg); // 将跟踪数据写入实时缓冲区
}

/**
 * @brief 动态参数调整回调函数
 * @param config 新配置参数
 * @param unused 未使用参数
 */
void Controller::reconfigCB(rm_gimbal_controllers::GimbalBaseConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Gimbal Base] Dynamic params change"); // 输出日志
  if (!dynamic_reconfig_initialized_) // 如果未初始化
  {
    GimbalConfig init_config = *config_rt_buffer_.readFromNonRT(); // 从缓冲区读取初始配置
    config.yaw_k_v_ = init_config.yaw_k_v_;         // 设置初始参数
    config.pitch_k_v_ = init_config.pitch_k_v_;
    config.k_chassis_vel_ = init_config.k_chassis_vel_;
    config.accel_pitch_ = init_config.accel_pitch_;
    config.accel_yaw_ = init_config.accel_yaw_;
    dynamic_reconfig_initialized_ = true;           // 标记为已初始化
  }
  // 创建新的配置并写入缓冲区
  GimbalConfig config_non_rt{ .yaw_k_v_ = config.yaw_k_v_,
                              .pitch_k_v_ = config.pitch_k_v_,
                              .k_chassis_vel_ = config.k_chassis_vel_,
                              .accel_pitch_ = config.accel_pitch_,
                              .accel_yaw_ = config.accel_yaw_ };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}

}  // namespace rm_gimbal_controllers

// 注册控制器插件
PLUGINLIB_EXPORT_CLASS(rm_gimbal_controllers::Controller, controller_interface::ControllerBase)
```

------

代码说明

这段代码是一个用于机器人云台（gimbal）控制的控制器实现，支持偏航（yaw）和俯仰（pitch）两个自由度的控制。它基于ROS（Robot Operating System）和硬件接口实现，主要功能包括：

1. 初始化：配置硬件接口、参数、订阅器和发布器，加载URDF模型以获取关节信息。
2. 控制模式：
	- RATE：基于速度命令控制云台。
	- TRACK：跟踪目标，使用子弹求解器计算目标方向。
	- DIRECT：直接设定目标点方向。
	- TRAJ：跟随预定义轨迹。
3. 运动控制：使用PID控制器、前馈补偿（重力补偿）和底盘速度补偿计算关节命令。
4. 实时性：通过实时缓冲区和发布器确保数据处理和发布的实时性。
5. 动态调整：支持运行时参数调整。

注释详细解释了每个函数的用途、参数含义和关键逻辑，帮助理解代码的实现细节和控制流程。