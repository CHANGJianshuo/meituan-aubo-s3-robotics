#!/usr/bin/env python3
"""Isolate MoveIt shutdown without a driver, planner client, or motion request.

Source scripts/env.sh first. Use a separate, empty ROS domain. Generates the
AUBO S3 fake model and upstream MoveIt parameters, then signals only the process
it started. --pin-plugins is a diagnostic control, not a deployment fix.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from check_launch_shutdown import shutdown_errors


def flatten(data, prefix=""):
    result = {}
    for key, value in data.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, name + "."))
        else:
            result[name] = value
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--mode", choices=("direct", "launch", "gdb"), default="direct")
    parser.add_argument("--rmw", choices=("rmw_cyclonedds_cpp", "rmw_fastrtps_cpp"),
                        default="rmw_cyclonedds_cpp")
    parser.add_argument("--domain", type=int, default=131)
    parser.add_argument("--pin-plugins", action="store_true")
    parser.add_argument("--disable-execution", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("domain must be in [0, 232]")
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), RMW_IMPLEMENTATION=args.rmw,
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", ROS_LOG_DIR=str(out / "ros_logs"))
    for key in ("ROS_STATIC_PEERS", "ROS_LOCALHOST_ONLY", "CYCLONEDDS_URI",
                "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "LD_PRELOAD"):
        env.pop(key, None)
    graph = subprocess.run(["ros2", "node", "list", "--no-daemon", "--spin-time", "2"],
                           env=env, capture_output=True, text=True, timeout=15, check=True)
    if graph.stdout.strip():
        raise RuntimeError(f"ROS domain {args.domain} is not empty: {graph.stdout}")

    description = Path(get_package_share_directory("aubo_description"))
    config = Path(get_package_share_directory("aubo_moveit_config"))
    moveit = (MoveItConfigsBuilder("aubo", package_name="aubo_moveit_config")
              .robot_description(file_path=str(description / "urdf/xacro/inc/aubo_ros2.xacro"),
                                 mappings={"aubo_type": "aubo_S3", "use_fake_hardware": "true"})
              .robot_description_semantic(file_path=str(config / "config/aubo_robot.srdf"))
              .robot_description_kinematics(file_path=str(config / "config/kinematics.yaml"))
              .joint_limits(file_path=str(config / "config/joint_limits.yaml"))
              .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
              .trajectory_execution(file_path=str(config / "config/moveit_controllers.yaml"),
                                    moveit_manage_controllers=False)
              .to_moveit_configs().to_dict())
    plugins = [p.text.strip() for p in ET.fromstring(moveit["robot_description"]).findall(
        "ros2_control/hardware/plugin")]
    if plugins != ["mock_components/GenericSystem"]:
        raise RuntimeError(f"unexpected hardware plugin: {plugins}")
    params = flatten(moveit)
    controllers = yaml.safe_load((config / "config/moveit_controllers.yaml").read_text())
    params.update(flatten({"moveit_simple_controller_manager": controllers}))
    params["moveit_controller_manager"] = "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    # There is deliberately no joint-state publisher in this empty-node probe.
    # This only removes the startup wait; no planning or execution is requested.
    params["planning_scene_monitor_options.wait_for_initial_state_timeout"] = 0.0
    params["allow_trajectory_execution"] = not args.disable_execution
    params_path = out / "move_group.yaml"
    params_path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    executable = Path(get_package_prefix("moveit_ros_move_group")) / "lib/moveit_ros_move_group/move_group"
    node_env = {}
    if args.pin_plugins:
        node_env["LD_PRELOAD"] = ":".join(str(Path(get_package_prefix(package)) / "lib" / library)
            for package, library in (
                ("moveit_simple_controller_manager", "libmoveit_simple_controller_manager.so"),
                ("moveit_ros_move_group", "libmoveit_move_group_default_capabilities.so")))
    cmd = [str(executable), "--ros-args", "--params-file", str(params_path)]
    if args.mode == "launch":
        launch_file = out / "shutdown_probe.launch.py"
        launch_file.write_text(
            "from launch import LaunchDescription\nfrom launch_ros.actions import Node\n"
            "def generate_launch_description():\n"
            "    return LaunchDescription([Node(package='moveit_ros_move_group', "
            "executable='move_group', output='screen', "
            f"parameters=[{str(params_path)!r}], additional_env={node_env!r})])\n")
        cmd = ["ros2", "launch", str(launch_file)]
    elif args.mode == "gdb":
        cmd = ["gdb", "-q", "-batch", "-ex", "set pagination off",
               "-ex", "handle SIGINT nostop print pass"] + (
                   ["-ex", "set environment LD_PRELOAD=" + node_env["LD_PRELOAD"]] if node_env else []) + [
               "-ex", "run", "-ex", "thread apply all bt", "-ex", "info sharedlibrary", "--args"] + cmd
    else:
        env.update(node_env)
    log = out / "process.log"
    result = dict(mode=args.mode, rmw=args.rmw, domain=args.domain, pin_plugins=args.pin_plugins,
                  allow_trajectory_execution=not args.disable_execution, command=cmd,
                  core_pattern=Path("/proc/sys/kernel/core_pattern").read_text().strip(),
                  motion_requests=0, ready=False, escalated=False)
    result["versions"] = subprocess.check_output([
        "dpkg-query", "-W", "ros-jazzy-moveit-ros-move-group", "ros-jazzy-moveit-ros-planning",
        "ros-jazzy-rclcpp", "ros-jazzy-rmw-cyclonedds-cpp", "ros-jazzy-rmw-fastrtps-cpp"], text=True)
    with log.open("w") as stream:
        process = subprocess.Popen(cmd, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline and process.poll() is None:
                if "You can start planning now" in log.read_text(errors="replace"):
                    result["ready"] = True
                    break
                time.sleep(0.1)
            if result["ready"]:
                pid = process.pid
                if args.mode == "gdb":
                    children = Path(f"/proc/{pid}/task/{pid}/children").read_text().split()
                    if len(children) != 1:
                        raise RuntimeError(f"unexpected gdb children: {children}")
                    pid = int(children[0])
                time.sleep(0.5)
                start = time.monotonic()
                os.kill(pid, signal.SIGINT)
                while process.poll() is None and time.monotonic() - start < 12:
                    status = Path(f"/proc/{pid}/status")
                    try:
                        status_text = status.read_text()
                        if "CoreDumping:\t1" in status_text and "core_dump_observation" not in result:
                            result["core_dump_observation"] = {
                                "status": status_text,
                                "wchan": Path(f"/proc/{pid}/wchan").read_text(),
                                "seconds_after_sigint": round(time.monotonic() - start, 3)}
                    except FileNotFoundError:
                        pass
                    time.sleep(0.1)
                result["seconds_after_sigint"] = round(time.monotonic() - start, 3)
        finally:
            if process.poll() is None:
                result["escalated"] = True
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
    result["supervisor_returncode"] = process.returncode
    content = log.read_text(errors="replace")
    if args.mode == "launch":
        result["shutdown_errors"] = shutdown_errors(content)
        clean = not result["shutdown_errors"]
    elif args.mode == "gdb":
        # gdb exits zero even when the inferior stopped at a segfault.
        clean = "exited normally]" in content and "received signal SIGSEGV" not in content
    else:
        clean = process.returncode == 0
    result["clean_exit"] = clean and result["ready"] and not result["escalated"]
    (out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result[key] for key in (
        "mode", "rmw", "ready", "pin_plugins", "supervisor_returncode", "clean_exit")}), flush=True)
    return 0 if result["clean_exit"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
