/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "qrb_ros_imu/imu_type_adapter.hpp"

#include <memory>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_client.h"

namespace qrb
{
namespace ros
{
ImuTypeAdapter::ImuTypeAdapter(const sensors_event_t& accel_event,
                               const sensors_event_t& gyro_event)
{
  if (accel_event.timestamp == 0 || gyro_event.timestamp == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("QTI IMU"), "data error");
  }
  if (accel_event.timestamp < gyro_event.timestamp) {
    header.stamp.nanosec = accel_event.timestamp % 1000000000LL;
    header.stamp.sec = accel_event.timestamp / 1000000000LL;
  } else {
    header.stamp.nanosec = gyro_event.timestamp % 1000000000LL;
    header.stamp.sec = gyro_event.timestamp / 1000000000LL;
  }
  header.frame_id = "imu";
  sensors_event_t* ptr = sensor_ptr;
  *ptr = accel_event;
  *(ptr + 1) = gyro_event;
}

ImuTypeAdapter::ImuTypeAdapter(const sensor_msgs::msg::Imu& sensor_msgs_imu)
{
  sensors_event_t* accel = &sensor_ptr[0];
  sensors_event_t* gyro = &sensor_ptr[1];
  header.stamp = sensor_msgs_imu.header.stamp;
  header.frame_id = sensor_msgs_imu.header.frame_id;
  accel->timestamp =
      sensor_msgs_imu.header.stamp.nanosec + sensor_msgs_imu.header.stamp.sec * 1000000000LL;
  gyro->timestamp =
      sensor_msgs_imu.header.stamp.nanosec + sensor_msgs_imu.header.stamp.sec * 1000000000LL;
  accel->acceleration.x = sensor_msgs_imu.linear_acceleration.x;
  accel->acceleration.y = sensor_msgs_imu.linear_acceleration.y;
  accel->acceleration.z = sensor_msgs_imu.linear_acceleration.z;

  gyro->gyro.x = sensor_msgs_imu.angular_velocity.x;
  gyro->gyro.y = sensor_msgs_imu.angular_velocity.y;
  gyro->gyro.z = sensor_msgs_imu.angular_velocity.z;
}

}  // namespace ros
}  // namespace qrb