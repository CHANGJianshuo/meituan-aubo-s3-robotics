#!/usr/bin/env bash
# 一次性安装 ROS 2 Jazzy + MoveIt 2 + ros2_control（Ubuntu 24.04 / WSL）。
# 只装系统包，不 clone、不编译、不连接任何机器人。可重复执行。
# 用法：bash src/mtc_moveit_planner/scripts/setup_ros_jazzy.sh   （中途会要求输入一次 sudo 密码）
set -euo pipefail

SUDO="sudo -E"   # -E 保留当前终端的代理设置

echo "[1/4] 基础工具与 universe 源"
$SUDO apt-get -o Acquire::Retries=5 update
$SUDO apt-get -o Acquire::Retries=5 install -y software-properties-common curl ca-certificates gnupg lsb-release
$SUDO add-apt-repository -y universe

echo "[2/4] ROS 2 apt 源"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  V=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  test -n "$V"
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
  $SUDO dpkg -i /tmp/ros2-apt-source.deb
fi
$SUDO apt-get -o Acquire::Retries=5 update

echo "[3/4] ROS 2 Jazzy / MoveIt 2 / ros2_control 及编译测试依赖"
$SUDO apt-get -o Acquire::Retries=5 install -y \
  build-essential cmake git libboost-dev python3-pytest python3-yaml socat \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-controllers-test-nodes \
  ros-jazzy-moveit ros-jazzy-moveit-setup-assistant ros-jazzy-warehouse-ros \
  ros-jazzy-rviz2 ros-jazzy-xacro ros-jazzy-urdf \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-tf2-tools ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-ament-cmake-gtest ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
  ros-jazzy-launch-testing-ament-cmake

echo "[4/4] rosdep 初始化"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  $SUDO rosdep init
fi
if [ "$(id -u)" -ne 0 ]; then rosdep update --rosdistro jazzy; fi

set +u
source /opt/ros/jazzy/setup.bash
echo "完成：ROS_DISTRO=$ROS_DISTRO"
dpkg -s ros-jazzy-moveit | grep -E '^(Package|Version)'
dpkg -s ros-jazzy-controller-manager | grep -E '^(Package|Version)'
