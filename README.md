# QRB ROS IMU

qrb_ros_imu is a package to publish the imu data from sensor service.

## Overview

Qualcomm Sensor See framework provides IMU data that obtained from the DSP side via FastRPC direct channel.
qrb_ros_imu uses this framework to get the latest imu data with high performance.

Sensor client, which is a dynamic library, is base on this framework for helping developers to utilize this feature. This will greatly reduce the latency between the ROS node and the driver. This time consumption measurement is around 0.4ms, which is several tens of times better than the performance where copying occurred before.

IMU data is widely used in robot localization, such as: SLAM (Simultaneous localization and mapping).
These localization applications have more precise performance after integrating IMU data to predict position.

This package is accelerated by [QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport), it leverages type adaption and intra process communication to optimize message formats and
dramatically accelerate communication between participating nodes.

> **Note:** To change the frame rate of the imu data, we need to change sensor service's configure file.
>

## QuickStart

1. Clone this repository under `${QRB_ROS_WS}/src`
    ```bash
    git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
    ```
2. Build this project
    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    ```
3. Using in your package by add this in your launch file to run this package

    ```python
    ComposableNode(
        package='qrb_ros_imu',
        plugin='qrb::ros::ImuComponent',
        name='imu_node'
    )
    ```

## Acceleration

This package is powered by [QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport) to optimize message formats and accelerate communication between participating nodes.

## Packages

Will update in the future.

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html)

## Contributions

Thanks for your interest in contributing to dmabuf_transport! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_imu is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
