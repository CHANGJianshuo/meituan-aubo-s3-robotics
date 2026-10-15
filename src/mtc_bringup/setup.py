"""ament_python 构建入口（mtc_bringup）。

入口点
------
* ``bringup_mock``   —— 离线 Mock 组合启动（**纯 Python，不 import rclpy**）。
  它只打印将要启动的 Mock 组合与配置摘要，并显式声明“未连接真实 AUBO S3”。
  真正的 ROS 2 节点组合请使用 ``launch/mock_bringup.launch.py``。
* ``validate_config`` —— 配置校验 CLI（退出码 0 通过 / 1 失败）。

注：本机仅有 ROS 2 Humble 且未执行 ``colcon build``；本文件按 ROS 2
ament_python 规范编写，尚未在任何 ROS 2 发行版上完成编译/安装验证。
"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mtc_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="meituan_challenge team",
    maintainer_email="team@example.invalid",
    description="启动编排、参数组合与离线配置校验（Mock 默认；真实设备默认禁用）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bringup_mock = mtc_bringup.bringup_mock:main",
            "validate_config = mtc_bringup.validate_config:main",
        ],
    },
)
