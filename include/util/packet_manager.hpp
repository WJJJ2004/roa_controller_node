#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roa_interfaces/msg/motor_command.hpp"
#include "roa_interfaces/msg/motor_command_array.hpp"
#include "roa_interfaces/msg/motor_state_array.hpp"

namespace roa_controller_node
{

class PacketManager
{
public:
  struct Command12Dof
  {
    float left_hip_pitch  = 0.0f;   // ID 10
    float right_hip_pitch = 0.0f;   // ID 11
    float left_hip_roll   = 0.0f;   // ID 12
    float right_hip_roll  = 0.0f;   // ID 13
    float left_hip_yaw    = 0.0f;   // ID 14
    float right_hip_yaw   = 0.0f;   // ID 15
    float left_knee_pitch = 0.0f;   // ID 16
    float right_knee_pitch= 0.0f;   // ID 17
    float left_rsu_upper  = 0.0f;   // ID 18
    float right_rsu_upper = 0.0f;   // ID 19
    float left_rsu_lower  = 0.0f;   // ID 20
    float right_rsu_lower = 0.0f;   // ID 21
  };

  struct JointMeta
  {
    const char* name;
    uint16_t motor_id;
    float kp;
    float kd;
  };

  struct MotorSample
  {
    float position = 0.0f;
    float velocity = 0.0f;
    float current  = 0.0f;
    bool valid = false;
  };

  struct HardwareState
  {
    // raw motor-space state
    MotorSample torso_yaw;
    MotorSample left_hip_pitch;
    MotorSample right_hip_pitch;
    MotorSample left_hip_roll;
    MotorSample right_hip_roll;
    MotorSample left_hip_yaw;
    MotorSample right_hip_yaw;
    MotorSample left_knee_pitch;
    MotorSample right_knee_pitch;
    MotorSample left_rsu_upper;
    MotorSample right_rsu_upper;
    MotorSample left_rsu_lower;
    MotorSample right_rsu_lower;
  };

  static constexpr std::size_t kMotorCount = 13;
  static constexpr int kMinMotorId = 9;
  static constexpr int kMaxMotorId = 21;

  static constexpr std::array<JointMeta, kMotorCount> kJointMetaTable{{
    {"torso_yaw",         9,  50.0f,  2.0f},
    {"left_hip_pitch",   10, 150.0f, 24.722f},
    {"right_hip_pitch",  11, 150.0f, 24.722f},
    {"left_hip_roll",    12, 200.0f, 26.387f},
    {"right_hip_roll",   13, 200.0f, 26.387f},
    {"left_hip_yaw",     14, 100.0f,  3.419f},
    {"right_hip_yaw",    15, 100.0f,  3.419f},
    {"left_knee_pitch",  16, 150.0f,  8.654f},
    {"right_knee_pitch", 17, 150.0f,  8.654f},
    {"left_rsu_upper",   18,  40.0f,  0.99f},
    {"right_rsu_upper",  19,  40.0f,  0.99f},
    {"left_rsu_lower",   20,  40.0f,  0.99f},
    {"right_rsu_lower",  21,  40.0f,  0.99f},
  }};

  static constexpr int motor_id_to_slot(int motor_id)
  {
    return (motor_id >= kMinMotorId && motor_id <= kMaxMotorId)
      ? (motor_id - kMinMotorId)
      : -1;
  }

  static constexpr const char* motor_id_to_name(int motor_id)
  {
    const int slot = motor_id_to_slot(motor_id);
    return (slot >= 0) ? kJointMetaTable[static_cast<std::size_t>(slot)].name : "invalid_motor_id";
  }

  static bool decode_motor_state(
    const roa_interfaces::msg::MotorStateArray& msg,
    HardwareState& out,
    std::string* error = nullptr)
  {
    std::array<MotorSample, kMotorCount> samples{};

    for (const auto& s : msg.states) {
      const int slot = motor_id_to_slot(static_cast<int>(s.motor_id));
      if (slot < 0) {
        continue;
      }

      if (!std::isfinite(s.position) || !std::isfinite(s.velocity) || !std::isfinite(s.current)) {
        if (error) {
          *error = std::string("Non-finite motor state: ") + motor_id_to_name(s.motor_id);
        }
        return false;
      }

      auto& dst = samples[static_cast<std::size_t>(slot)];
      dst.position = s.position;
      dst.velocity = s.velocity;
      dst.current  = s.current;
      dst.valid = true;
    }

    auto require = [&](int motor_id) -> const MotorSample* {
      const int slot = motor_id_to_slot(motor_id);
      if (slot < 0) return nullptr;
      const auto& m = samples[static_cast<std::size_t>(slot)];
      return m.valid ? &m : nullptr;
    };

    const auto* torso_yaw         = require(9);
    const auto* left_hip_pitch    = require(10);
    const auto* right_hip_pitch   = require(11);
    const auto* left_hip_roll     = require(12);
    const auto* right_hip_roll    = require(13);
    const auto* left_hip_yaw      = require(14);
    const auto* right_hip_yaw     = require(15);
    const auto* left_knee_pitch   = require(16);
    const auto* right_knee_pitch  = require(17);
    const auto* left_rsu_upper    = require(18);
    const auto* right_rsu_upper   = require(19);
    const auto* left_rsu_lower    = require(20);
    const auto* right_rsu_lower   = require(21);

    if (!torso_yaw || !left_hip_pitch || !right_hip_pitch ||
        !left_hip_roll || !right_hip_roll ||
        !left_hip_yaw || !right_hip_yaw ||
        !left_knee_pitch || !right_knee_pitch ||
        !left_rsu_upper || !right_rsu_upper ||
        !left_rsu_lower || !right_rsu_lower) {
      if (error) {
        *error = "Required motor states missing";
      }
      return false;
    }

    out.torso_yaw         = *torso_yaw;
    out.left_hip_pitch    = *left_hip_pitch;
    out.right_hip_pitch   = *right_hip_pitch;
    out.left_hip_roll     = *left_hip_roll;
    out.right_hip_roll    = *right_hip_roll;
    out.left_hip_yaw      = *left_hip_yaw;
    out.right_hip_yaw     = *right_hip_yaw;
    out.left_knee_pitch   = *left_knee_pitch;
    out.right_knee_pitch  = *right_knee_pitch;
    out.left_rsu_upper    = *left_rsu_upper;
    out.right_rsu_upper   = *right_rsu_upper;
    out.left_rsu_lower    = *left_rsu_lower;
    out.right_rsu_lower   = *right_rsu_lower;
    return true;
  }

  static roa_interfaces::msg::MotorCommandArray build(
    const Command12Dof& cmd,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "")
  {
    roa_interfaces::msg::MotorCommandArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.commands.reserve(kMotorCount);

    // ID 9 : torso_yaw is internally fixed to zero
    msg.commands.push_back(make_command(
      kJointMetaTable[0].motor_id,
      0.0f, // position
      kJointMetaTable[0].kp,
      kJointMetaTable[0].kd));

    // ID 10 ~ 21 from external 12DOF command contract
    // clang-format off
    msg.commands.push_back(make_command(kJointMetaTable[1].motor_id,  cmd.left_hip_pitch,   kJointMetaTable[1].kp,  kJointMetaTable[1].kd));
    msg.commands.push_back(make_command(kJointMetaTable[2].motor_id,  cmd.right_hip_pitch,  kJointMetaTable[2].kp,  kJointMetaTable[2].kd));
    msg.commands.push_back(make_command(kJointMetaTable[3].motor_id,  cmd.left_hip_roll,    kJointMetaTable[3].kp,  kJointMetaTable[3].kd));
    msg.commands.push_back(make_command(kJointMetaTable[4].motor_id,  cmd.right_hip_roll,   kJointMetaTable[4].kp,  kJointMetaTable[4].kd));
    msg.commands.push_back(make_command(kJointMetaTable[5].motor_id,  cmd.left_hip_yaw,     kJointMetaTable[5].kp,  kJointMetaTable[5].kd));
    msg.commands.push_back(make_command(kJointMetaTable[6].motor_id,  cmd.right_hip_yaw,    kJointMetaTable[6].kp,  kJointMetaTable[6].kd));
    msg.commands.push_back(make_command(kJointMetaTable[7].motor_id,  cmd.left_knee_pitch,  kJointMetaTable[7].kp,  kJointMetaTable[7].kd));
    msg.commands.push_back(make_command(kJointMetaTable[8].motor_id,  cmd.right_knee_pitch, kJointMetaTable[8].kp,  kJointMetaTable[8].kd));
    msg.commands.push_back(make_command(kJointMetaTable[9].motor_id,  cmd.left_rsu_upper,   kJointMetaTable[9].kp,  kJointMetaTable[9].kd));
    msg.commands.push_back(make_command(kJointMetaTable[10].motor_id, cmd.right_rsu_upper,  kJointMetaTable[10].kp, kJointMetaTable[10].kd));
    msg.commands.push_back(make_command(kJointMetaTable[11].motor_id, cmd.left_rsu_lower,   kJointMetaTable[11].kp, kJointMetaTable[11].kd));
    msg.commands.push_back(make_command(kJointMetaTable[12].motor_id, cmd.right_rsu_lower,  kJointMetaTable[12].kp, kJointMetaTable[12].kd));
    // clang-format on

    return msg;
  }

private:
  static roa_interfaces::msg::MotorCommand make_command(
    uint16_t motor_id,
    float    position,
    float    kp,
    float    kd)
  {
    roa_interfaces::msg::MotorCommand cmd;

    cmd.motor_id = motor_id;
    cmd.torque   = 0.0f;
    cmd.position = position;
    cmd.velocity = 0.0f;
    cmd.kp       = kp;
    cmd.kd       = kd;

    return cmd;
  }

};

}  // namespace roa_controller_node