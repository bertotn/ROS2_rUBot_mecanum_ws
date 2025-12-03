# Install Intel RealSense D435/i

This camera could be installed on:
- PC Ubuntu 22 with ROS2 Humble
- Raspberrypi4 Ubuntu 22 with ROS2 Humble

The installation process is the same in both cases.

Proceed with:
```bash
sudo apt install -y git cmake build-essential \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

mkdir -p ~/repos && cd ~/repos
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo ./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=true -DBUILD_EXAMPLES=true
make -j$(nproc)
sudo make install

realsense-viewer
```
To install the wrapper ROS2:
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
# Launch with default params values
ros2 launch realsense2_camera rs_launch.py
# Llançament amb resolució més baixa per anar més fi
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x360x15 \
  pointcloud.enable:=false
rviz2
```
When camera is on raspberrypi is recommeded:
```bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x360x15 \
  rgb_camera.color_profile:=640x480x15 \
  pointcloud.enable:=false \
  enable_gyro:=false \
  enable_accel:=false
```

# Install Orbbec DaBai

This camera could be installed on:
- PC Ubuntu 22 with ROS2 Humble
- Raspberrypi4 Ubuntu 22 with ROS2 Humble

The installation process is the same in both cases.

Install dependencies:
```bash
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
     ros-humble-image-transport ros-humble-camera-info-manager \
     ros-humble-image-publisher libusb-1.0-0-dev libeigen3-dev
```
Clone the ROS2 package in your workspace:
```bash
cd ~/ROS2_rUBot_mecanum_ws/src
git clone --branch v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
Define the UDEV rules:
```bash
cd src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Launch:
```bash
ros2 launch orbbec_camera dabai_a.launch.py
```
Or with custom parameters:
```bash
ros2 launch orbbec_camera dabai_a.launch.py \
  color_width:=640  color_height:=480  color_fps:=15 \
  depth_width:=640  depth_height:=480  depth_fps:=15 \
  enable_point_cloud:=false
```