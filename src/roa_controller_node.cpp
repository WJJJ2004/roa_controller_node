// TODO 
// * policy action scale ? Raw Action 처리 방식 ? 
// * action_to_q_target
// * handle first lastaciton obs
#include "node/roa_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>  // std::isfinite

using namespace std::chrono_literals;

namespace roa_controller_node
{
RoaControllerNode::RoaControllerNode(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("roa_controller_node", options)
{
  RCLCPP_INFO(get_logger(),
  "roa_controller_node constructed ");
}

RoaControllerNode::CallbackReturn
RoaControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  is_activate = false;
  motor_cmd = setInitPose();
  RCLCPP_INFO(get_logger(), "[Lifecycle] on_configure()");

  declareAndLoadParams();
  setupRosInterfaces();
  setupTimers();

  // // init cmd buffer
  // last_hw_cmd_.data.clear();
  // last_hw_cmd_.data.resize(static_cast<size_t>(std::max(0, walk_len_)), 0.0f);

  // 내부 상태 초기화
  last_safe_rsu_ = {0.0f, 0.0f, 0.0f, 0.0f};
  rsu_seq_ = 0;
  policy_loaded_ = false;
  last_policy_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);


  if (!init_policy()) {
    RCLCPP_ERROR(get_logger(), "init_policy() failed in on_configure()");
    return CallbackReturn::FAILURE;
  }

  {
    std::ostringstream oss;
    oss << "\n";
    oss << "RoaControllerNode configuration summary\n";
    oss << "  rates\n";
    oss << "    hw_rate_hz               : " << std::fixed << std::setprecision(1) << hw_rate_hz_ << "\n";
    oss << "    policy_rate_hz           : " << std::fixed << std::setprecision(1) << policy_rate_hz_ << "\n";
    oss << "    status_rate_hz           : " << std::fixed << std::setprecision(1) << status_rate_hz_ << "\n";
    oss << "    hw_period_ms             : " << std::fixed << std::setprecision(3) << (1000.0 / std::max(1.0, hw_rate_hz_)) << "\n";
    oss << "    policy_period_ms         : " << std::fixed << std::setprecision(3) << (1000.0 / std::max(1.0, policy_rate_hz_)) << "\n";
    oss << "    status_period_ms         : " << std::fixed << std::setprecision(3) << (1000.0 / std::max(1.0, status_rate_hz_)) << "\n";

    oss << "  timeouts [ms]\n";
    oss << "    rsu_timeout_ms           : " << rsu_timeout_ms_ << "\n";
    oss << "    cmd_timeout_ms           : " << cmd_timeout_ms_ << "\n";
    oss << "    imu_timeout_ms           : " << imu_timeout_ms_ << "\n";
    oss << "    gravity_timeout_ms       : " << gravity_timeout_ms_ << "\n";
    oss << "    motor_state_timeout_ms   : " << motor_state_timeout_ms_ << "\n";
    oss << "    policy_cmd_timeout_ms    : " << policy_cmd_timeout_ms_ << "\n";

    oss << "  topics\n";
    oss << "    walk_policy_cmd          : " << topic_walk_cmd_ << "\n";
    oss << "    imu_data                 : " << topic_imu_data_ << "\n";
    oss << "    imu_gravity              : " << topic_imu_gravity_ << "\n";
    oss << "    rsu_solution             : " << topic_rsu_solution_ << "\n";
    oss << "    rsu_state                : " << topic_rsu_status_ << "\n";
    oss << "    rsu_target               : " << topic_rsu_target_ << "\n";
    oss << "    motor_command            : " << topic_motor_command_ << "\n";
    oss << "    motor_state              : " << topic_motor_state_ << "\n";
    oss << "    controller_status        : " << topic_controller_status_ << "\n";

    oss << "  policy\n";
    oss << "    model_path               : " << roa::policy::iface::Policy12DofV1::default_model_path() << "\n";
    oss << "    obs_dim                  : " << kObsDim << "\n";
    oss << "    act_dim                  : " << kActDim << "\n";
    oss << "    policy_loaded            : " << (policy_loaded_ ? "true" : "false") << "\n";

    oss << "  control semantics\n";
    oss << "    ankle_obs_source         : virtual RSU joint state (/rsu/state)\n";
    oss << "    ankle_target_space       : virtual ankle joint target\n";
    oss << "    ankle_actuator_source    : RSU solver output (/rsu/solution)\n";
    oss << "    cmd_stale_behavior       : zero cmd input to policy\n";
    oss << "    rsu_stale_behavior       : hold last safe RSU actuator command\n";
    oss << "    timers_start_on_activate : true\n";

    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }
  return CallbackReturn::SUCCESS;
}

RoaControllerNode::CallbackReturn
RoaControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  is_activate = true;
  RCLCPP_INFO(get_logger(), "[Lifecycle] on_activate()");

  if (!policy_loaded_) {
    RCLCPP_ERROR(get_logger(), "Cannot activate: policy not loaded");
    return CallbackReturn::FAILURE;
  }

  if (rsu_target_pub_) {
    rsu_target_pub_->on_activate();
  }
  if (motor_packit_pub_) {
    motor_packit_pub_->on_activate();
  }

  if (hw_timer_) {
    hw_timer_->reset();
  }
  if (policy_timer_) {
    policy_timer_->reset();
  }
  if (status_timer_) {
    status_timer_->reset();
  }

  RCLCPP_INFO(get_logger(), "Controller activated");
  return CallbackReturn::SUCCESS;
}

RoaControllerNode::CallbackReturn
RoaControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  is_activate = false;
  RCLCPP_WARN(get_logger(), "[Lifecycle] on_deactivate()");

  if (hw_timer_) {
    hw_timer_->cancel();
  }
  if (policy_timer_) {
    policy_timer_->cancel();
  }
  if (status_timer_) {
    status_timer_->cancel();
  }

  if (rsu_target_pub_) {
    rsu_target_pub_->on_deactivate();
  }
  if (motor_packit_pub_) {
    motor_packit_pub_->on_deactivate();
  }

  return CallbackReturn::SUCCESS;
}

RoaControllerNode::CallbackReturn
RoaControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  is_activate = false;
  RCLCPP_INFO(get_logger(), "[Lifecycle] on_cleanup()");

  if (hw_timer_) {
    hw_timer_->cancel();
    hw_timer_.reset();
  }
  if (policy_timer_) {
    policy_timer_->cancel();
    policy_timer_.reset();
  }
  if (status_timer_) {
    status_timer_->cancel();
    status_timer_.reset();
  }

  cmd_sub_.reset();
  imu_sub_.reset();
  gravity_sub_.reset();
  rsu_solution_sub_.reset();
  motor_state_sub_.reset();
  rsu_state_sub_.reset();

  rsu_target_pub_.reset();
  motor_packit_pub_.reset();
  controller_status_pub_.reset();

  policy_loaded_ = false;

  last_safe_rsu_ = {0.0f, 0.0f, 0.0f, 0.0f};
  rsu_seq_ = 0;

  return CallbackReturn::SUCCESS;
}

RoaControllerNode::CallbackReturn
RoaControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_WARN(get_logger(), "[Lifecycle] on_shutdown()");
  return CallbackReturn::SUCCESS;
}

void RoaControllerNode::declareAndLoadParams()
{
  // control mode 
  if (!this->has_parameter("REALTIME_CONTROL_MODE")) {
    this->declare_parameter<bool>("REALTIME_CONTROL_MODE", is_realtime_control_mode_);
  }
  // rates
  if (!this->has_parameter("hw_rate_hz")) {
    this->declare_parameter<double>("hw_rate_hz", hw_rate_hz_);
  }
  if (!this->has_parameter("policy_rate_hz")) {
    this->declare_parameter<double>("policy_rate_hz", policy_rate_hz_);
  }
  if (!this->has_parameter("status_rate_hz")) {
    this->declare_parameter<double>("status_rate_hz", status_rate_hz_);
  }

  // timeouts [ms]
  if (!this->has_parameter("rsu_timeout_ms")) {
    this->declare_parameter<double>("rsu_timeout_ms", static_cast<double>(rsu_timeout_ms_));
  }
  if (!this->has_parameter("cmd_timeout_ms")) {
    this->declare_parameter<double>("cmd_timeout_ms", static_cast<double>(cmd_timeout_ms_));
  }
  if (!this->has_parameter("imu_timeout_ms")) {
    this->declare_parameter<double>("imu_timeout_ms", static_cast<double>(imu_timeout_ms_));
  }
  if (!this->has_parameter("gravity_timeout_ms")) {
    this->declare_parameter<double>("gravity_timeout_ms", static_cast<double>(gravity_timeout_ms_));
  }
  if (!this->has_parameter("motor_state_timeout_ms")) {
    this->declare_parameter<double>("motor_state_timeout_ms", static_cast<double>(motor_state_timeout_ms_));
  }
  if (!this->has_parameter("policy_cmd_timeout_ms")) {
    this->declare_parameter<double>("policy_cmd_timeout_ms", static_cast<double>(policy_cmd_timeout_ms_));
  }

  // command length
  if (!this->has_parameter("motor_command_len")) {
    this->declare_parameter<int>("motor_command_len", walk_len_);
  }

  // frame
  if (!this->has_parameter("rsu_frame_id")) {
    this->declare_parameter<std::string>("rsu_frame_id", rsu_frame_id_);
  }

  // topics
  if (!this->has_parameter("topics.walk_policy_cmd")) {
    this->declare_parameter<std::string>("topics.walk_policy_cmd", topic_walk_cmd_);
  }
  if (!this->has_parameter("topics.imu_data")) {
    this->declare_parameter<std::string>("topics.imu_data", topic_imu_data_);
  }
  if (!this->has_parameter("topics.imu_gravity")) {
    this->declare_parameter<std::string>("topics.imu_gravity", topic_imu_gravity_);
  }
  if (!this->has_parameter("topics.rsu_solution")) {
    this->declare_parameter<std::string>("topics.rsu_solution", topic_rsu_solution_);
  }
  if (!this->has_parameter("topics.rsu_target")) {
    this->declare_parameter<std::string>("topics.rsu_target", topic_rsu_target_);
  }
  if (!this->has_parameter("topics.motor_command")) {
    this->declare_parameter<std::string>("topics.motor_command", topic_motor_command_);
  }
  if (!this->has_parameter("topics.motor_state_sub")) {
    this->declare_parameter<std::string>("topics.motor_state_sub", topic_motor_state_);
  }
  if (!this->has_parameter("topics.controller_status")) {
    this->declare_parameter<std::string>("topics.controller_status", topic_controller_status_);
  }
  if (!this->has_parameter("topics.rsu_state_sub")) {
    this->declare_parameter<std::string>("topics.rsu_state_sub", topic_rsu_status_);
  }

  is_realtime_control_mode_ = this->get_parameter("REALTIME_CONTROL_MODE").as_bool();
  if (is_realtime_control_mode_) {
    control_mode_ = CONTROL_MODE::RT_CONTROL;
  }
  else {
    control_mode_ = CONTROL_MODE::DEBUG;
  }

  RCLCPP_INFO(get_logger(), "Control mode: %s", (control_mode_ == CONTROL_MODE::RT_CONTROL) ? "REALTIME_CONTROL_MODE" : "DEBUG_MODE");
    
  // load rates
  hw_rate_hz_ = this->get_parameter("hw_rate_hz").as_double();
  policy_rate_hz_ = this->get_parameter("policy_rate_hz").as_double();
  status_rate_hz_ = this->get_parameter("status_rate_hz").as_double();

  // load timeouts [ms]
  rsu_timeout_ms_ = this->get_parameter("rsu_timeout_ms").as_double();
  cmd_timeout_ms_ = this->get_parameter("cmd_timeout_ms").as_double();
  imu_timeout_ms_ = this->get_parameter("imu_timeout_ms").as_double();
  gravity_timeout_ms_ = this->get_parameter("gravity_timeout_ms").as_double();
  motor_state_timeout_ms_ = this->get_parameter("motor_state_timeout_ms").as_double();
  policy_cmd_timeout_ms_ = this->get_parameter("policy_cmd_timeout_ms").as_double();

  // load misc
  walk_len_ = this->get_parameter("motor_command_len").as_int();
  rsu_frame_id_ = this->get_parameter("rsu_frame_id").as_string();

  // load topics
  topic_walk_cmd_ = this->get_parameter("topics.walk_policy_cmd").as_string();
  topic_imu_data_ = this->get_parameter("topics.imu_data").as_string();
  topic_imu_gravity_ = this->get_parameter("topics.imu_gravity").as_string();
  topic_rsu_solution_ = this->get_parameter("topics.rsu_solution").as_string();
  topic_rsu_target_ = this->get_parameter("topics.rsu_target").as_string();
  topic_motor_command_ = this->get_parameter("topics.motor_command").as_string();
  topic_motor_state_ = this->get_parameter("topics.motor_state_sub").as_string();
  topic_controller_status_ = this->get_parameter("topics.controller_status").as_string();
  topic_rsu_status_ = this->get_parameter("topics.rsu_state_sub").as_string();

  // convert ms -> Duration
  rsu_timeout_ = rclcpp::Duration::from_seconds(rsu_timeout_ms_ / 1000.0); // /rsu/state and /rsu/solution
  cmd_timeout_ = rclcpp::Duration::from_seconds(cmd_timeout_ms_ / 1000.0);
  imu_timeout_ = rclcpp::Duration::from_seconds(imu_timeout_ms_ / 1000.0);
  gravity_timeout_ = rclcpp::Duration::from_seconds(gravity_timeout_ms_ / 1000.0);
  motor_state_timeout_ = rclcpp::Duration::from_seconds(motor_state_timeout_ms_ / 1000.0);
  policy_cmd_timeout_ = rclcpp::Duration::from_seconds(policy_cmd_timeout_ms_ / 1000.0);
}

bool RoaControllerNode::init_policy()
{
  roa::policy::Options opt{};
  std::string model_path = roa::policy::iface::Policy12DofV1::default_model_path();

  if (!driver_.load(model_path, opt)) {
    RCLCPP_ERROR(get_logger(), "Failed to load policy: %s", model_path.c_str());
    return false;
  }

  if (driver_.input_dim() != kObsDim || driver_.output_dim() != kActDim) {
    RCLCPP_ERROR(
      get_logger(),
      "Policy dim mismatch. got in=%d out=%d expected in=%d out=%d",
      driver_.input_dim(), driver_.output_dim(), kObsDim, kActDim);
    return false;
  }

  last_action_.fill(0.0f);
  obs_buffer_.fill(0.0f);
  act_buffer_.fill(0.0f);

  policy_loaded_ = true;
  return true;
}

bool RoaControllerNode::isFreshRx(
  const rclcpp::Time& now_time,
  const rclcpp::Time& rx_time,
  const rclcpp::Duration& timeout) const
{
  if (rx_time.nanoseconds() <= 0) {
    return false;
  }
  return (now_time - rx_time) < timeout;
}

bool RoaControllerNode::isFreshStamp(
  const rclcpp::Time& now_time,
  const builtin_interfaces::msg::Time& stamp,
  const rclcpp::Duration& timeout) const
{
  const rclcpp::Time stamp_time(stamp);

  if (stamp_time.nanoseconds() <= 0) {
    return false;
  }
  return (now_time - stamp_time) < timeout;
}

bool RoaControllerNode::build_observation(const rclcpp::Time& tnow)
{
  // 1) cmd
  auto [cmd_msg, cmd_rx_time] = cmd_latch_.get();
  const bool cmd_ok = (cmd_msg != nullptr) && isFreshRx(tnow, cmd_rx_time, cmd_timeout_);
  if (cmd_ok) {
    obs_.cmd[0] = static_cast<float>(cmd_msg->linear.x);
    obs_.cmd[1] = static_cast<float>(cmd_msg->linear.y);
    obs_.cmd[2] = static_cast<float>(cmd_msg->angular.z);
  } else {
    obs_.cmd[0] = 0.0f;
    obs_.cmd[1] = 0.0f;
    obs_.cmd[2] = 0.0f;
  }

  // 2) imu
  auto [imu_msg, imu_rx_time] = imu_latch_.get();
  const bool imu_ok =
    (imu_msg != nullptr) &&
    isFreshStamp(tnow, imu_msg->header.stamp, imu_timeout_) &&
    std::isfinite(imu_msg->angular_velocity.x) &&
    std::isfinite(imu_msg->angular_velocity.y) &&
    std::isfinite(imu_msg->angular_velocity.z);

  if (!imu_ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "IMU data stale or invalid. Observation build failed.");
    return false;
  }

  obs_.imu_omega_body[0] = static_cast<float>(imu_msg->angular_velocity.x);
  obs_.imu_omega_body[1] = static_cast<float>(imu_msg->angular_velocity.y);
  obs_.imu_omega_body[2] = static_cast<float>(imu_msg->angular_velocity.z);

  // 3) last_action
  for (int i = 0; i < kDof; ++i) {
    obs_.last_action[i] = last_action_[i];
  }

  // 4) motor state decode
  auto [motor_state_msg, motor_state_rx_time] = motor_state_latch_.get();
  const bool motor_state_ok =
    (motor_state_msg != nullptr) &&
    isFreshStamp(tnow, motor_state_msg->header.stamp, motor_state_timeout_);

  if (!motor_state_ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "MotorState stale or invalid. Observation build failed.");
    return false;
  }

  PacketManager::HardwareState hw{};
  std::string hw_error;
  if (!PacketManager::decode_motor_state(*motor_state_msg, hw, &hw_error)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "MotorState decode failed: %s", hw_error.c_str());
    return false;
  }

  // 5) virtual RSU state
  auto [rsu_state_msg, rsu_state_rx_time] = rsu_state_latch_.get();
  const bool rsu_state_ok =
    (rsu_state_msg != nullptr) &&
    isFreshStamp(tnow, rsu_state_msg->header.stamp, rsu_timeout_) &&
    rsu_state_msg->feasible &&
    std::isfinite(rsu_state_msg->q.left_rsu_roll) &&
    std::isfinite(rsu_state_msg->q.left_rsu_pitch) &&
    std::isfinite(rsu_state_msg->q.right_rsu_roll) &&
    std::isfinite(rsu_state_msg->q.right_rsu_pitch) &&
    std::isfinite(rsu_state_msg->q_dot.left_rsu_roll) &&
    std::isfinite(rsu_state_msg->q_dot.left_rsu_pitch) &&
    std::isfinite(rsu_state_msg->q_dot.right_rsu_roll) &&
    std::isfinite(rsu_state_msg->q_dot.right_rsu_pitch);

  if (!rsu_state_ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "RSU state stale or invalid. Observation build failed.");
    return false;
  }

  using P = roa::policy::iface::Policy12DofV1;
  std::array<float, kDof> q_cur{};
  std::array<float, kDof> qd_cur{};

  // real encoder state for non-ankle joints
  q_cur[P::L_HIP_PITCH]   = hw.left_hip_pitch.position;
  q_cur[P::R_HIP_PITCH]   = hw.right_hip_pitch.position;
  q_cur[P::L_HIP_ROLL]    = hw.left_hip_roll.position;
  q_cur[P::R_HIP_ROLL]    = hw.right_hip_roll.position;
  q_cur[P::L_HIP_YAW]     = hw.left_hip_yaw.position;
  q_cur[P::R_HIP_YAW]     = hw.right_hip_yaw.position;
  q_cur[P::L_KNEE_PITCH]  = hw.left_knee_pitch.position;
  q_cur[P::R_KNEE_PITCH]  = hw.right_knee_pitch.position;

  qd_cur[P::L_HIP_PITCH]  = hw.left_hip_pitch.velocity;
  qd_cur[P::R_HIP_PITCH]  = hw.right_hip_pitch.velocity;
  qd_cur[P::L_HIP_ROLL]   = hw.left_hip_roll.velocity;
  qd_cur[P::R_HIP_ROLL]   = hw.right_hip_roll.velocity;
  qd_cur[P::L_HIP_YAW]    = hw.left_hip_yaw.velocity;
  qd_cur[P::R_HIP_YAW]    = hw.right_hip_yaw.velocity;
  qd_cur[P::L_KNEE_PITCH] = hw.left_knee_pitch.velocity;
  qd_cur[P::R_KNEE_PITCH] = hw.right_knee_pitch.velocity;

  // virtual ankle state for inference
  q_cur[P::L_ANKLE_PITCH]  = rsu_state_msg->q.left_rsu_pitch;
  q_cur[P::R_ANKLE_PITCH]  = rsu_state_msg->q.right_rsu_pitch;
  q_cur[P::L_ANKLE_ROLL]   = rsu_state_msg->q.left_rsu_roll;
  q_cur[P::R_ANKLE_ROLL]   = rsu_state_msg->q.right_rsu_roll;

  qd_cur[P::L_ANKLE_PITCH] = rsu_state_msg->q_dot.left_rsu_pitch;
  qd_cur[P::R_ANKLE_PITCH] = rsu_state_msg->q_dot.right_rsu_pitch;
  qd_cur[P::L_ANKLE_ROLL]  = rsu_state_msg->q_dot.left_rsu_roll;
  qd_cur[P::R_ANKLE_ROLL]  = rsu_state_msg->q_dot.right_rsu_roll;

  // q_rel / qd_rel
  for (int i = 0; i < kDof; ++i) {
    obs_.q_rel[i]  = q_cur[i] - default_angles_[i];
    obs_.qd_rel[i] = qd_cur[i];
  }

  roa::policy::iface::Policy12DofV1::pack_obs(obs_, obs_buffer_.data());
  return true;
}

void RoaControllerNode::setupRosInterfaces()
{
  // QoS
  auto imu_qos = rclcpp::SensorDataQoS();

  rclcpp::QoS rsu_qos(rclcpp::KeepLast(1));
  rsu_qos.best_effort();

  rclcpp::QoS cmd_qos(rclcpp::KeepLast(1));
  cmd_qos.reliable();

  rclcpp::QoS motor_status_qos(rclcpp::KeepLast(1));
  motor_status_qos.best_effort();

  // Publishers
  rsu_target_pub_ = create_publisher<roa_interfaces::msg::RsuTarget>(topic_rsu_target_, rsu_qos);
  motor_packit_pub_ = create_publisher<roa_interfaces::msg::MotorCommandArray>(topic_motor_command_, cmd_qos);
  controller_status_pub_ = create_publisher<roa_interfaces::msg::SystemStatus>(topic_controller_status_,10);

  // Subscriptions
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic_walk_cmd_, rclcpp::QoS(1), std::bind(&RoaControllerNode::onCmd, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    topic_imu_data_, imu_qos, std::bind(&RoaControllerNode::onImu, this, std::placeholders::_1));

  gravity_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    topic_imu_gravity_, imu_qos, std::bind(&RoaControllerNode::onGravity, this, std::placeholders::_1));

  rsu_solution_sub_ = create_subscription<roa_interfaces::msg::RsuSolution>(
    topic_rsu_solution_, rsu_qos, std::bind(&RoaControllerNode::onRsuSolution, this, std::placeholders::_1));
  motor_state_sub_  = create_subscription<roa_interfaces::msg::MotorStateArray>(
    topic_motor_state_, motor_status_qos, std::bind(&RoaControllerNode::onMotorStatus, this, std::placeholders::_1));
  rsu_state_sub_ = create_subscription<roa_interfaces::msg::RsuStateArray>(
    topic_rsu_status_, rsu_qos, std::bind(&RoaControllerNode::onRsuStatus, this, std::placeholders::_1));
}

void RoaControllerNode::setupTimers()
{
  const auto hw_ns  = static_cast<int64_t>(1e9 / std::max(1.0, hw_rate_hz_));
  const auto pol_ns = static_cast<int64_t>(1e9 / std::max(1.0, policy_rate_hz_));
  const auto stat_ns = static_cast<int64_t>(1e9 / std::max(1.0, status_rate_hz_));


  hw_timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(hw_ns),
    std::bind(&RoaControllerNode::ControlLoop, this));

  policy_timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(pol_ns),
    std::bind(&RoaControllerNode::InferenceLoop, this));
  status_timer_ = this->create_wall_timer(
    std::chrono::nanoseconds(stat_ns),
    std::bind(&RoaControllerNode::publish_controller_status, this));

  // inactive/configured 상태에서는 timer stop
  hw_timer_->cancel();
  policy_timer_->cancel();
}

void RoaControllerNode::onImu(sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_latch_.set(std::move(msg), now());
}

void RoaControllerNode::onGravity(geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  gravity_latch_.set(std::move(msg), now());
}

void RoaControllerNode::onRsuSolution(roa_interfaces::msg::RsuSolution::SharedPtr msg)
{
  rsu_latch_.set(std::move(msg), now());
}

void RoaControllerNode::onCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_latch_.set(std::move(msg), now());
}

void RoaControllerNode::onMotorStatus(roa_interfaces::msg::MotorStateArray::SharedPtr msg)
{
  motor_state_latch_.set(std::move(msg), now());
}

void RoaControllerNode::onRsuStatus(roa_interfaces::msg::RsuStateArray::SharedPtr msg)
{
  rsu_state_latch_.set(std::move(msg), now());
}


void RoaControllerNode::publish_rsu_target(
  float left_roll, float left_pitch,
  float right_roll, float right_pitch)
{
  roa_interfaces::msg::RsuTarget msg;
  msg.header.stamp = now();
  msg.header.frame_id = rsu_frame_id_; // 당장은 활용 X
  msg.seq = rsu_seq_++;

  msg.left_roll = left_roll;
  msg.left_pitch = left_pitch;
  msg.right_roll = right_roll;
  msg.right_pitch = right_pitch;

  rsu_target_pub_->publish(msg);
}

void RoaControllerNode::InferenceLoop()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  if (!policy_loaded_) {
    RCLCPP_ERROR(get_logger(), "Policy is not loaded");
    return;
  }

  const auto tnow = now();

  if (!build_observation(tnow)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Observation build failed");
    return;
  }

  const bool ok = driver_.run(
    obs_buffer_.data(), kObsDim,
    act_buffer_.data(), kActDim);

  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Policy inference failed");
    return;
  }

  last_policy_update_time_ = tnow;

  for (int i = 0; i < kDof; ++i) {
    last_action_[i] = act_buffer_[i];
  }

  using P = roa::policy::iface::Policy12DofV1;
  auto q_target = P::action_to_q_target(
    act_buffer_, default_angles_, action_scale_);

  {
    std::lock_guard<std::mutex> lk(cmd_m_);

    motor_cmd.left_hip_pitch   = q_target[P::L_HIP_PITCH];
    motor_cmd.right_hip_pitch  = q_target[P::R_HIP_PITCH];
    motor_cmd.left_hip_roll    = q_target[P::L_HIP_ROLL];
    motor_cmd.right_hip_roll   = q_target[P::R_HIP_ROLL];
    motor_cmd.left_hip_yaw     = q_target[P::L_HIP_YAW];
    motor_cmd.right_hip_yaw    = q_target[P::R_HIP_YAW];
    motor_cmd.left_knee_pitch  = q_target[P::L_KNEE_PITCH];
    motor_cmd.right_knee_pitch = q_target[P::R_KNEE_PITCH];
  }

  publish_rsu_target(
    q_target[P::L_ANKLE_ROLL],
    q_target[P::L_ANKLE_PITCH],
    q_target[P::R_ANKLE_ROLL],
    q_target[P::R_ANKLE_PITCH]
  );
}

void RoaControllerNode::ControlLoop()
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  const auto tnow = now();

  // latest non-RSU command snapshot
  PacketManager::Command12Dof cmd;
  {
    std::lock_guard<std::mutex> lk(cmd_m_);
    cmd = motor_cmd;
  }

  // latest RSU solution check
  auto [rsu_msg, rsu_rx_time] = rsu_latch_.get();

  const bool rsu_ok =
    (rsu_msg != nullptr) &&
    rsu_msg->feasible &&
    isFreshStamp(tnow, rsu_msg->header.stamp, rsu_timeout_);

  if (rsu_ok) {
    const bool finite =
      std::isfinite(rsu_msg->left_actuator_1)  &&
      std::isfinite(rsu_msg->left_actuator_2)  &&
      std::isfinite(rsu_msg->right_actuator_1) &&
      std::isfinite(rsu_msg->right_actuator_2);

    if (finite) {
      last_safe_rsu_[0] = rsu_msg->left_actuator_1;
      last_safe_rsu_[1] = rsu_msg->left_actuator_2;
      last_safe_rsu_[2] = rsu_msg->right_actuator_1;
      last_safe_rsu_[3] = rsu_msg->right_actuator_2;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "RSU solution contains NaN/Inf. Holding previous safe RSU command.");
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "RSU solution stale or infeasible. Holding previous safe RSU command.");
  }

  // stale / infeasible이면 hold
  cmd.left_rsu_upper  = last_safe_rsu_[0];
  cmd.left_rsu_lower  = last_safe_rsu_[1];
  cmd.right_rsu_upper = last_safe_rsu_[2];
  cmd.right_rsu_lower = last_safe_rsu_[3];

  if (control_mode_ == CONTROL_MODE::RT_CONTROL) {
    auto msg = PacketManager::build(cmd, this->now(), "/CTRL/RT_CONTROL");
    motor_packit_pub_->publish(msg);
  }
  else {
    // Send initial position to hardware
    auto msg = PacketManager::build(setInitPose(), this->now(), "/CTRL/DEBUG_MODE");
    motor_packit_pub_->publish(msg);

    // Print computed command for debugging
    RCLCPP_INFO(get_logger(), 
      "[INFER] Policy Cmd - Hip: L(%.3f,%.3f) R(%.3f,%.3f) | "
      "Knee: L%.3f R%.3f | RSU: L(%.3f,%.3f) R(%.3f,%.3f)",
      cmd.left_hip_pitch, cmd.left_hip_roll,
      cmd.right_hip_pitch, cmd.right_hip_roll,
      cmd.left_knee_pitch, cmd.right_knee_pitch,
      cmd.left_rsu_upper, cmd.left_rsu_lower,
      cmd.right_rsu_upper, cmd.right_rsu_lower);
  }
}

void RoaControllerNode::publish_controller_status()
{
  if (!controller_status_pub_) {
    return;
  }

  const auto tnow = now();

  roa_interfaces::msg::SystemStatus msg;
  msg.header.stamp = tnow;

  msg.ready = compute_ready();
  msg.healthy = compute_healthy();
  msg.rt_ok = compute_rt_ok(tnow);
  msg.error_code = compute_controller_error_code(tnow);

  controller_status_pub_->publish(msg);
}

bool RoaControllerNode::compute_ready() const
{
  const bool timers_ok =
    (hw_timer_ != nullptr) &&
    (policy_timer_ != nullptr) &&
    (status_timer_ != nullptr);

  const bool pubs_ok =
    (rsu_target_pub_ != nullptr) &&
    (motor_packit_pub_ != nullptr) &&
    (controller_status_pub_ != nullptr);

  const bool subs_ok =
    (cmd_sub_ != nullptr) &&
    (imu_sub_ != nullptr) &&
    (gravity_sub_ != nullptr) &&
    (rsu_solution_sub_ != nullptr) &&
    (motor_state_sub_ != nullptr) &&
    (rsu_state_sub_ != nullptr);

  return policy_loaded_ && timers_ok && pubs_ok && subs_ok;
}

bool RoaControllerNode::compute_healthy() const
{
  // 현재 구조에서는 policy 미로드를 사실상 치명 상태로 봄
  return policy_loaded_;
}

bool RoaControllerNode::compute_rt_ok(const rclcpp::Time& tnow) const
{
  auto [imu_msg, imu_rx_time] = imu_latch_.get();
  (void)imu_rx_time;
  const bool imu_ok =
    (imu_msg != nullptr) &&
    isFreshStamp(tnow, imu_msg->header.stamp, imu_timeout_) &&
    std::isfinite(imu_msg->angular_velocity.x) &&
    std::isfinite(imu_msg->angular_velocity.y) &&
    std::isfinite(imu_msg->angular_velocity.z);

  auto [motor_state_msg, motor_state_rx_time] = motor_state_latch_.get();
  (void)motor_state_rx_time;
  const bool motor_state_ok =
    (motor_state_msg != nullptr) &&
    isFreshStamp(tnow, motor_state_msg->header.stamp, motor_state_timeout_);

  auto [rsu_state_msg, rsu_state_rx_time] = rsu_state_latch_.get();
  (void)rsu_state_rx_time;
  const bool rsu_state_ok =
    (rsu_state_msg != nullptr) &&
    isFreshStamp(tnow, rsu_state_msg->header.stamp, rsu_timeout_) &&
    rsu_state_msg->feasible &&
    std::isfinite(rsu_state_msg->q.left_rsu_roll) &&
    std::isfinite(rsu_state_msg->q.left_rsu_pitch) &&
    std::isfinite(rsu_state_msg->q.right_rsu_roll) &&
    std::isfinite(rsu_state_msg->q.right_rsu_pitch) &&
    std::isfinite(rsu_state_msg->q_dot.left_rsu_roll) &&
    std::isfinite(rsu_state_msg->q_dot.left_rsu_pitch) &&
    std::isfinite(rsu_state_msg->q_dot.right_rsu_roll) &&
    std::isfinite(rsu_state_msg->q_dot.right_rsu_pitch);

    bool policy_ok = false;
    if (is_activate) {
      policy_ok =
        (last_policy_update_time_.nanoseconds() > 0) &&
        ((tnow - last_policy_update_time_) < policy_cmd_timeout_);
    }
    else
    {
      policy_ok = true; // 비활성 상태에서는 정책 신선도 체크 안함
    }
  return imu_ok && motor_state_ok && rsu_state_ok && policy_ok;
}

uint32_t RoaControllerNode::compute_controller_error_code(const rclcpp::Time& tnow) const
{
  uint32_t err = ERR_NONE;

  if (!policy_loaded_) {
    err |= ERR_POLICY_NOT_LOADED;
  }

  auto [imu_msg, imu_rx_time] = imu_latch_.get();
  (void)imu_rx_time;
  const bool imu_ok =
    (imu_msg != nullptr) &&
    isFreshStamp(tnow, imu_msg->header.stamp, imu_timeout_) &&
    std::isfinite(imu_msg->angular_velocity.x) &&
    std::isfinite(imu_msg->angular_velocity.y) &&
    std::isfinite(imu_msg->angular_velocity.z);
  if (!imu_ok) {
    err |= ERR_IMU_STALE;
  }

  auto [motor_state_msg, motor_state_rx_time] = motor_state_latch_.get();
  (void)motor_state_rx_time;
  const bool motor_state_ok =
    (motor_state_msg != nullptr) &&
    isFreshStamp(tnow, motor_state_msg->header.stamp, motor_state_timeout_);
  if (!motor_state_ok) {
    err |= ERR_MOTOR_STATE_STALE;
  }

  auto [rsu_state_msg, rsu_state_rx_time] = rsu_state_latch_.get();
  (void)rsu_state_rx_time;
  const bool rsu_state_ok =
    (rsu_state_msg != nullptr) &&
    isFreshStamp(tnow, rsu_state_msg->header.stamp, rsu_timeout_) &&
    rsu_state_msg->feasible;
  if (!rsu_state_ok) {
    err |= ERR_RSU_STATE_STALE;
  }

  if (is_activate) {
    const bool policy_ok =
      (last_policy_update_time_.nanoseconds() > 0) &&
      ((tnow - last_policy_update_time_) < policy_cmd_timeout_);
    if (!policy_ok) {
      err |= ERR_POLICY_STALE;
    }
  }
  return err;
}


}  // namespace roa_controller_node

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roa_controller_node::RoaControllerNode>();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();

  return 0;
}
