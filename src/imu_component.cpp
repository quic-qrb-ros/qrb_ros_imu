/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "qrb_ros_imu/imu_component.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

#include "qrb_ros_imu/imu_type_adapter.hpp"
#include "sensor_client.h"

namespace qrb
{
namespace ros
{
ImuComponent::ImuComponent(const rclcpp::NodeOptions& options) : Node("imu_node", options)
{
  retry_ = 0;
  running_ = false;
  is_working = true;
  this->declare_parameter<bool>("debug", false);
  if (!this->init()) {
    RCLCPP_ERROR(this->get_logger(), "imu component init failed");
    RCLCPP_ERROR(this->get_logger(), "will try to reinit again");
  } else {
    RCLCPP_INFO(this->get_logger(), "imu component running...");
  }
}

ImuComponent::~ImuComponent()
{
  RCLCPP_INFO(this->get_logger(), "stopping...");
  running_ = false;
  is_working = false;
  if (thread_publish_msg_) {
    thread_publish_msg_->join();
  }
  sensor_client_.DisconnectServer();
}

bool ImuComponent::init()
{
  if (!sensor_client_.CreateConnection()) {
    RCLCPP_ERROR(this->get_logger(), "imu client connect failed.");
    timer_ = this->create_wall_timer(std::chrono::seconds(RETRY_INTERVAL),
                                     std::bind(&ImuComponent::retry_connection, this));
    return false;
  }

  connect_success();
  return true;
}

void ImuComponent::connect_success()
{
  RCLCPP_INFO(this->get_logger(), "imu client connect success");
  running_ = true;

  rclcpp::PublisherOptions pub_options;
  pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  publisher_ = this->create_publisher<TOPIC_TYPE>(TOPIC_NAME, 30, pub_options);
  thread_publish_msg_ =
      std::make_shared<std::thread>(std::mem_fn(&ImuComponent::publish_msg), this);
}

void ImuComponent::retry_connection()
{
  RCLCPP_INFO(this->get_logger(), "retry to connect");
  bool success = sensor_client_.CreateConnection();
  if (success && !running_) {
    connect_success();
    this->timer_->cancel();
    return;
  } else {
    RCLCPP_ERROR(this->get_logger(), "reconnect service failed");
    RCLCPP_ERROR(this->get_logger(), "retry %d/%d after %d seconds...", ++retry_, RETRY_MAX,
                 RETRY_INTERVAL);
  }
  if (retry_ >= RETRY_MAX) {
    RCLCPP_ERROR(this->get_logger(), "reconnect service failed");
    this->timer_->cancel();
    is_working = false;
  }
}

void ImuComponent::publish_msg()
{
  int32_t pack_num = 0;
  int i = 0;
  sensors_event_t* accel_ptr;
  sensors_event_t* gyro_ptr;
  std::unique_ptr<qrb::ros::ImuTypeAdapter> container;
  long time_nanosec;
  long long latency_sum = 0;

  while (running_) {
    pack_num = 0;
    this->get_parameter("debug", debug_);
    if (!sensor_client_.GetImuData(&accel_ptr, &gyro_ptr, &pack_num)) {
      RCLCPP_DEBUG(this->get_logger(), "get imu data failed");
      if (!debug_) {
        RCLCPP_DEBUG(this->get_logger(), "thread will sleep 2 ms");
        std::this_thread::sleep_for(std::chrono::microseconds(2000));
      }
      continue;
    }
    for (i = 0; i < pack_num; i++) {
      container = std::make_unique<qrb::ros::ImuTypeAdapter>(*accel_ptr, *gyro_ptr);
      if (debug_) {
        time_nanosec = (get_clock()->now()).nanoseconds();
        if (accel_ptr->timestamp > gyro_ptr->timestamp) {
          latency_sum += time_nanosec - accel_ptr->timestamp;
        } else {
          latency_sum += time_nanosec - gyro_ptr->timestamp;
        }
        container->header.stamp = get_clock()->now();
      }
      publisher_->publish(std::move(container));
      accel_ptr += 1;
      gyro_ptr += 1;
    }
    if (debug_) {
      RCLCPP_INFO(this->get_logger(), "latency: %lld ns", latency_sum / pack_num);
      latency_sum = 0;
    }
  }
}
}  // namespace ros
}  // namespace qrb

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(qrb::ros::ImuComponent)
