#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include <roa_interfaces/msg/rsu_solution.hpp>

namespace roa_controller_node
{

class RsuSolutionLatch
{
public:
  void set(roa_interfaces::msg::RsuSolution::SharedPtr msg, const rclcpp::Time& rx_time)
  {
    std::lock_guard<std::mutex> lk(m_);
    msg_ = std::move(msg);
    rx_time_ = rx_time;
  }

  roa_interfaces::msg::RsuSolution::SharedPtr get_msg() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return msg_;
  }

  // usable rule:
  // - feasible == true
  // - now - stamp < timeout
  bool usable(const rclcpp::Time& now, const rclcpp::Duration& timeout) const
  {
    std::lock_guard<std::mutex> lk(m_);
    if (!msg_) return false;
    if (!msg_->feasible) return false;

    const auto stamp = rclcpp::Time(msg_->header.stamp);
    if (stamp.nanoseconds() <= 0) return false;

    return (now - stamp) < timeout;
  }

private:
  mutable std::mutex m_;
  roa_interfaces::msg::RsuSolution::SharedPtr msg_;
  rclcpp::Time rx_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace roa_controller_node