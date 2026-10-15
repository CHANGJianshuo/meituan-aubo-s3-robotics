from setuptools import find_packages, setup

package_name = "mtc_manipulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/test", ["test/test_pick_place_fsm.py", "test/test_server_shell.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="meituan_challenge_ws maintainers",
    maintainer_email="maintainer@example.invalid",
    description=(
        "单块电池抓放状态机（第二层 PickPlace FSM）：纯 Python FSM 核心 + rclpy Action Server 外壳 + V1/V2 工具策略"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pick_place_server = mtc_manipulation.pick_place_server:main",
        ],
    },
)
