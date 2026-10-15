#!/usr/bin/env python3
"""Planning Scene 对象 ID 隔离反证（只读写碰撞体，不发任何运动指令）。

  seed   : 写入"其他模块"的同名对象 table、perception/battery_3（放在机器人工作空间外），
           以及一个本模块的遗留对象 mtc_moveit_planner/old_fixture（模拟进程重启前写入）。
  check  : 规划工具 apply_scene 之后检查：其他模块对象原样保留、遗留对象被清掉、
           本模块桌面以 mtc_moveit_planner/table 存在、没有写入不带前缀的规划对象。
  cleanup: 删除本探针自己写入的"其他模块"对象。
"""
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

FOREIGN = ['table', 'perception/battery_3']
STALE = 'mtc_moveit_planner/old_fixture'
OWNED_TABLE = 'mtc_moveit_planner/table'


def box(object_id, op=CollisionObject.ADD):
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = 'world'
    obj.operation = op
    if op == CollisionObject.ADD:
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
        pose = Pose()
        pose.position.x = pose.position.y = pose.position.z = -3.0  # 远离工作空间
        pose.orientation.w = 1.0
        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
    return obj


def call(node, client, request):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('service %s unavailable' % client.srv_name)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        raise RuntimeError('service %s timed out' % client.srv_name)
    return future.result()


def apply(node, objects):
    req = ApplyPlanningScene.Request()
    req.scene = PlanningScene(is_diff=True)
    req.scene.world.collision_objects = objects
    if not call(node, node.create_client(ApplyPlanningScene, '/apply_planning_scene'), req).success:
        raise RuntimeError('apply_planning_scene failed')


def world_ids(node):
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
    res = call(node, node.create_client(GetPlanningScene, '/get_planning_scene'), req)
    return sorted(o.id for o in res.scene.world.collision_objects)


def main():
    mode = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('mtc_scene_isolation_probe')
    try:
        if mode == 'seed':
            apply(node, [box(i) for i in FOREIGN + [STALE]])
            print('seeded world objects:', world_ids(node))
            return 0
        if mode == 'cleanup':
            apply(node, [box(i, CollisionObject.REMOVE) for i in FOREIGN])
            print('after cleanup:', world_ids(node))
            return 0
        ids = world_ids(node)
        print('world objects after planner apply_scene:', ids)
        checks = [
            ('foreign objects untouched', all(i in ids for i in FOREIGN)),
            ('stale owned object removed', STALE not in ids),
            ('planner table is namespaced', OWNED_TABLE in ids),
            ('no other planner object outside namespace',
             all(i in FOREIGN or i.startswith('mtc_moveit_planner/') for i in ids)),
        ]
        for name, ok in checks:
            print('%s %s' % ('PASS' if ok else 'FAIL', name))
        return 0 if all(ok for _, ok in checks) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
