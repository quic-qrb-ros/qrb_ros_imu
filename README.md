# QRB ROS IMU

## Overview

`qrb_ros_imu` is a package to publish the imu data from sensor service.
- This package uses Qualcomm Sensor See framework to get the latest imu data with high performance.
- The IMU data is widely used in robot localization, such as SLAM(Simultaneous localization and mapping).

## Getting Started

<details><summary>Cross Compile with QCLINUX SDK</summary>

#### Cross Compile with QCLINUX SDK

Setup QCLINUX SDK environments:
- Reference [QRB ROS Documents: Getting Started](https://quic-qrb-ros.github.io/getting_started/environment_setup.html)

Create workspace in QCLINUX SDK environment and clone source code

```bash
mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
```

Build source code with QCLINUX SDK

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

Install ROS package to device

```bash
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install
tar czvf qrb_ros_imu.tar.gz lib share
scp qrb_ros_imu.tar.gz root@[ip-addr]:/opt/
ssh ssh root@[ip-addr]
(ssh) tar -zxf /opt/qrb_ros_imu.tar.gz -C /opt/qcom/qirp-sdk/usr/
```

Login to device and run

```bash
ssh root@[ip-addr]
(ssh) export HOME=/opt
(ssh) source /usr/bin/ros_setup.bash
(ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
(ssh) ros2 run qrb_ros_imu imu_node
```

</details>

<details open><summary>Native Build on Ubuntu</summary>

#### Native Build on Ubuntu

Prerequisites

- ROS 2: [Install ROS2 on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- sensor dependency install:
```bash
sudo apt install libimage-transport-dev
sudo add-apt-repository ppa:carmel-team/jammy-release --login
sudo apt upgrade
sudo apt install camx-kt pulseaudio-module-pal-card gstreamer1.0-tools gstreamer1.0-qcom-sample-apps weston-qcom
sudo reboot
```

Create workspace and clone source code from GitHub:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
```
Build source code

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

Run system monitor

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run qrb_ros_imu imu_node
```

</details>


## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcomm RB3 gen2.

| Hardware                                                                                   | Software                                                   |
| ------------------------------------------------------------------------------------------ | ---------------------------------------------------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | `LE.QCROBOTICS.1.0`, `Canonical Ubuntu Image for RB3 gen2` |


## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)


## Authors

* **Zhanye Lin** - *Initial work* - [quic-zhanlin](https://github.com/quic-zhanlin)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

