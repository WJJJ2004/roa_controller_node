#pragma once
#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <roa_interfaces/msg/rsu_target.hpp>
#include <roa_interfaces/msg/rsu_solution.hpp>
#include <roa_interfaces/msg/system_status.hpp>
#include "roa_interfaces/msg/motor_command.hpp"
#include "roa_interfaces/msg/motor_command_array.hpp"
#include "roa_interfaces/msg/motor_state.hpp"
#include "roa_interfaces/msg/motor_state_array.hpp"
#include "roa_interfaces/msg/rsu_state.hpp"
#include "roa_interfaces/msg/rsu_state_array.hpp"

#include "util/latch.hpp"
#include "util/rsu_solution_latch.hpp"
#include "util/packet_manager.hpp"

#include <roa_policy_driver/policy_driver.hpp>
#include <roa_policy_driver/interfaces/policy_12dof_v1.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

// 서있는 자세를 위한 보행 초기 자세 -> offset pose 
#define HIP_INIT_POS 0.418879f
#define KNEE_INIT_POS 0.698131f
#define ANKLE_INIT_POS 0.458105f

// 추론 모델의 상대각도 기준점 -> default anlge
#define HIP_PITCH_DEF 0.349066f  // 20 degrees 
#define KNEE_PITCH_DEF 0.872665f // 50 degrees
#define ANKLE_PITCH_DEF 0.523599f // 30 degrees

namespace roa_controller_node
{

enum class CONTROL_MODE{
  RT_CONTROL = 0,
  DEBUG = 1
};

class RoaControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RoaControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // fixed dimensions
  static constexpr int kObsDim = roa::policy::iface::Policy12DofV1::kObsDim;
  static constexpr int kActDim = roa::policy::iface::Policy12DofV1::kActDim;
  static constexpr int kDof    = roa::policy::iface::Policy12DofV1::kDof;

  enum ControllerErrorCode : uint32_t
  {
    ERR_NONE              = 0u,
    ERR_POLICY_NOT_LOADED = 1u << 0,
    ERR_IMU_STALE         = 1u << 1,
    ERR_MOTOR_STATE_STALE = 1u << 2,
    ERR_RSU_STATE_STALE   = 1u << 3,
    ERR_POLICY_STALE      = 1u << 4,
  };

  void onCmd(geometry_msgs::msg::Twist::SharedPtr msg);
  void onImu(sensor_msgs::msg::Imu::SharedPtr msg);
  void onGravity(geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void onRsuSolution(roa_interfaces::msg::RsuSolution::SharedPtr msg);
  void onMotorStatus(roa_interfaces::msg::MotorStateArray::SharedPtr msg);
  void onRsuStatus(roa_interfaces::msg::RsuStateArray::SharedPtr msg);

  void publish_rsu_target(float left_roll, float left_pitch, float right_roll, float right_pitch);
  void publish_controller_status(); // 10 hz status pub loop

  void InferenceLoop(); // 50Hz
  void ControlLoop();     // 100Hz

  void declareAndLoadParams();
  void setupRosInterfaces();
  void setupTimers();

  uint32_t compute_controller_error_code(const rclcpp::Time& tnow) const;
  bool compute_ready() const;
  bool compute_healthy() const;
  bool compute_rt_ok(const rclcpp::Time& tnow) const;

private:
  PacketManager::Command12Dof
  setInitPose() 
  {  
    PacketManager::Command12Dof init_pos_{};

    init_pos_.left_hip_pitch   = -HIP_INIT_POS;
    init_pos_.left_hip_roll    = 0.0f;
    init_pos_.left_hip_yaw     = 0.0f;
    init_pos_.left_knee_pitch  =  KNEE_INIT_POS;

    init_pos_.right_hip_pitch  =  HIP_INIT_POS;
    init_pos_.right_hip_roll   = 0.0f;
    init_pos_.right_hip_yaw    = 0.0f;
    init_pos_.right_knee_pitch = -KNEE_INIT_POS;

    init_pos_.left_rsu_upper   = -ANKLE_INIT_POS;
    init_pos_.left_rsu_lower   =  ANKLE_INIT_POS;
    init_pos_.right_rsu_upper  =  ANKLE_INIT_POS;
    init_pos_.right_rsu_lower  = -ANKLE_INIT_POS;

    return init_pos_;
  }

  static std::array<float, kActDim>
  make_default_angles()
  {
    using P = roa::policy::iface::Policy12DofV1;
    std::array<float, P::kActDim> q{};

    q[P::L_HIP_PITCH]    = -HIP_PITCH_DEF;
    q[P::R_HIP_PITCH]    =  HIP_PITCH_DEF;
    q[P::L_HIP_ROLL]     =  0.00f;
    q[P::R_HIP_ROLL]     =  0.00f;
    q[P::L_HIP_YAW]      =  0.00f;
    q[P::R_HIP_YAW]      =  0.00f;
    q[P::L_KNEE_PITCH]   =  KNEE_PITCH_DEF;
    q[P::R_KNEE_PITCH]   = -KNEE_PITCH_DEF;
    q[P::L_ANKLE_PITCH]  = -ANKLE_PITCH_DEF;
    q[P::R_ANKLE_PITCH]  =  ANKLE_PITCH_DEF;
    q[P::L_ANKLE_ROLL]   =  0.00f;
    q[P::R_ANKLE_ROLL]   =  0.00f;

    return q;
  }
  const std::array<float, kActDim> default_angles_ = make_default_angles();
  const float action_scale_ = 0.5;
  
  bool is_activate{false};
  bool is_realtime_control_mode_ = false;
  CONTROL_MODE control_mode_;

  using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  // pubs/subs
  rclcpp_lifecycle::LifecyclePublisher<roa_interfaces::msg::RsuTarget>::SharedPtr rsu_target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<roa_interfaces::msg::MotorCommandArray>::SharedPtr motor_packit_pub_;
  rclcpp::Publisher<roa_interfaces::msg::SystemStatus>::SharedPtr controller_status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_sub_;
  rclcpp::Subscription<roa_interfaces::msg::RsuSolution>::SharedPtr rsu_solution_sub_;
  rclcpp::Subscription<roa_interfaces::msg::MotorStateArray>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<roa_interfaces::msg::RsuStateArray>::SharedPtr rsu_state_sub_;

  rclcpp::TimerBase::SharedPtr hw_timer_;
  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // RT data helper
  rclcpp::Time last_policy_update_time_{0, 0, RCL_ROS_TIME};

  bool isFreshRx(
    const rclcpp::Time& now_time,
    const rclcpp::Time& rx_time,
    const rclcpp::Duration& timeout) const;

  bool isFreshStamp(
    const rclcpp::Time& now_time,
    const builtin_interfaces::msg::Time& stamp,
    const rclcpp::Duration& timeout) const;

  // latches & Containers
  Latch<sensor_msgs::msg::Imu> imu_latch_;
  Latch<geometry_msgs::msg::Vector3Stamped> gravity_latch_;
  Latch<roa_interfaces::msg::MotorStateArray> motor_state_latch_;
  Latch<geometry_msgs::msg::Twist> cmd_latch_;
  Latch<roa_interfaces::msg::RsuSolution> rsu_latch_;
  Latch<roa_interfaces::msg::RsuStateArray> rsu_state_latch_;

  // std::array<float, 3> cmd_latch_{0.0f, 0.0f, 0.0f};

  // ++++++++++++++++++ roa policy driver sdk +++++++++++++++++++++++++
  roa::policy::PolicyDriver driver_;
  bool policy_loaded_{false};

  // buffers
  std::array<float, kObsDim> obs_buffer_{};
  std::array<float, kActDim> act_buffer_{};
  std::array<float, kDof> last_action_{};

  // // structured obs/act
  roa::policy::iface::Policy12DofV1::Obs obs_{};
  // roa::policy::iface::Policy12DofV1::Act act_{};


  bool init_policy();
  bool build_observation(const rclcpp::Time& tnow);
  // +++++++++++++ MOTOR CONTROL ++++++++++++

  PacketManager::Command12Dof motor_cmd{};
  std::array<float, 4> last_safe_rsu_{0.f, 0.f, 0.f, 0.f};

  // ++++++++++++++++++++++++++++++++++++++++
  // last hw command latch
  mutable std::mutex cmd_m_;
  // std_msgs::msg::Float32MultiArray last_hw_cmd_;

  // parameters
  double hw_rate_hz_{100.0};
  double policy_rate_hz_{50.0};
  double status_rate_hz_{10.0};

  double rsu_timeout_ms_{50.0};
  double cmd_timeout_ms_{50.0};
  double imu_timeout_ms_{50.0};
  double gravity_timeout_ms_{50.0};
  double motor_state_timeout_ms_{50.0};
  double policy_cmd_timeout_ms_{50.0};

  int walk_len_{40};

  std::string rsu_frame_id_{"base_link"};

  std::string topic_walk_cmd_{"/walk_policy/cmd_vel"};
  std::string topic_imu_data_{"/imu/data"};
  std::string topic_imu_gravity_{"/imu/gravity"};
  std::string topic_rsu_solution_{"/rsu/solution"};
  std::string topic_rsu_target_{"/rsu/target"};
  std::string topic_motor_command_{"/hardware_interface/command"};
  std::string topic_motor_state_{"/hardware_interface/state"};
  std::string topic_controller_status_{"/controller/status"};
  std::string topic_rsu_status_{"/rsu/state"};

  rclcpp::Duration rsu_timeout_{0, 0};
  rclcpp::Duration cmd_timeout_{0, 0};
  rclcpp::Duration imu_timeout_{0, 0};
  rclcpp::Duration gravity_timeout_{0, 0};
  rclcpp::Duration motor_state_timeout_{0, 0};
  rclcpp::Duration policy_cmd_timeout_{0, 0};

  uint32_t rsu_seq_{0};
};

}  // namespace roa_controller_node