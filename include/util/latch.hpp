#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <utility>

namespace roa_controller_node
{

template <typename MsgT>
class Latch
{
public:
  void set(typename MsgT::SharedPtr msg, const rclcpp::Time& rx_time)
  {
    std::lock_guard<std::mutex> lk(m_);
    msg_ = std::move(msg);
    rx_time_ = rx_time;
  }

  std::pair<typename MsgT::SharedPtr, rclcpp::Time> get() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return {msg_, rx_time_};
  }

private:
  mutable std::mutex m_;
  typename MsgT::SharedPtr msg_;
  rclcpp::Time rx_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace roa_controller_node