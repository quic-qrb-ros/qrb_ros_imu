/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef QRB_ROS_IMU__IMU_TYPE_ADAPTER_HPP_
#define QRB_ROS_IMU__IMU_TYPE_ADAPTER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_client.h"

namespace qrb
{
namespace ros
{
class ImuTypeAdapter
{
public:
  explicit ImuTypeAdapter(const sensor_msgs::msg::Imu& sensor_msgs_imu);

  ImuTypeAdapter(const sensors_event_t& accel_event, const sensors_event_t& gyro_event);

  ImuTypeAdapter(){};

  std_msgs::msg::Header header;
  sensors_event_t sensor_ptr[2];
};
}  // namespace ros
}  // namespace qrb

template <>
struct rclcpp::TypeAdapter<qrb::ros::ImuTypeAdapter, sensor_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = qrb::ros::ImuTypeAdapter;
  using ros_message_type = sensor_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type& source, ros_message_type& destination)
  {
    destination.header = source.header;
    destination.linear_acceleration.x = source.sensor_ptr[0].acceleration.x;
    destination.linear_acceleration.y = source.sensor_ptr[0].acceleration.y;
    destination.linear_acceleration.z = source.sensor_ptr[0].acceleration.z;
    destination.angular_velocity.x = source.sensor_ptr[1].gyro.x;
    destination.angular_velocity.y = source.sensor_ptr[1].gyro.y;
    destination.angular_velocity.z = source.sensor_ptr[1].gyro.z;
  }

  static void convert_to_custom(const ros_message_type& source, custom_type& destination)
  {
    destination = qrb::ros::ImuTypeAdapter(source);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb::ros::ImuTypeAdapter, sensor_msgs::msg::Imu);
#endif