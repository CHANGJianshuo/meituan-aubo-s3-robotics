"""ament_python 构建入口。

入口点：
* ``task_executor`` —— 整轮任务 FSM 的 ROS 2 Action Server 节点
  （Action ``/mtc/task/execute``，状态话题 ``/mtc/task/state``）。

注：本机仅有 ROS 2 Humble，且**未执行 colcon build**；本文件按 ROS 2
ament_python 规范编写，尚未在任何 ROS 2 发行版上完成编译/安装验证。
"""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mtc_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "docs"), glob("*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="meituan_challenge team",
    maintainer_email="team@example.invalid",
    description="整轮比赛任务状态机（Task FSM）与 ExecuteTask Action Server",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_executor = mtc_task.task_executor:main",
        ],
    },
)
