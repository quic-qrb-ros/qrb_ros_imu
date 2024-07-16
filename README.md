# QRB ROS IMU

qrb_ros_imu is a package to publish the imu data from sensor service.

## Overview

Qualcomm Sensor See framework provides IMU data that obtained from the DSP side via FastRPC direct channel.
qrb_ros_imu uses this framework to get the latest imu data with high performance.

qrb_sensor_client, which is a dynamic library, is base on this framework for helping developers to utilize this feature. This will greatly reduce the latency between the ROS node and the driver. This time consumption measurement is around 0.4ms, which is several tens of times better than the performance where copying occurred before.

IMU data is widely used in robot localization, such as: SLAM (Simultaneous localization and mapping).
These localization applications have more precise performance after integrating IMU data to predict position.

This package is accelerated by [QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport), it leverages type adaption and intra process communication to optimize message formats and
dramatically accelerate communication between participating nodes.

> **Note:** To change the frame rate of the imu data, we need to change sensor service's configure file.
>

## Build

Currently, we only support use QCLINUX to build

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`
     ```bash
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
     ```
4. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```
5. Push to the device & Install
     ```bash
     cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
     tar czvf qrb_ros_imu.tar.gz lib share
     scp qrb_ros_imu.tar.gz root@[ip-addr]:/opt/
     ssh root@[ip-addr]
     (ssh) tar -zxf /opt/qrb_ros_imu.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```

## Run

This package supports running it directly from the command or by dynamically adding it to the ros2 component container.

a.Run with command

1. Source this file to set up the environment on your device:
    ```bash
    ssh root@[ip-addr]
    (ssh) export HOME=/opt
    (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
    (ssh) export ROS_DOMAIN_ID=xx
    (ssh) source /usr/bin/ros_setup.bash
    ```

2. Use this command to run this package
    ```bash
    (ssh) ros2 run qrb_ros_imu imu_node
    ```

b. Dynamically add it to the ros2 component container
```python
ComposableNode(
    package='qrb_ros_imu',
    plugin='qrb_ros::imu::ImuComponent',
    name='imu'
)
```

## Acceleration

This package is powered by [QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport) to optimize message formats and accelerate communication between participating nodes.

## Packages

Will update in the future.

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html)

## Contributions

Thanks for your interest in contributing to qrb_ros_imu! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_imu is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
