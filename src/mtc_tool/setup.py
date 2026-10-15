from setuptools import setup

package_name = "mtc_tool"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="meituan_challenge_ws tool layer",
    maintainer_email="todo@example.invalid",
    description="V1 舌规 / V2 电磁锁止末端策略的统一管理节点（工具层权威实现）",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tool_manager = mtc_tool.tool_manager_node:main",
        ],
    },
)
