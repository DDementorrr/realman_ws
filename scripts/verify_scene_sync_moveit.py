#!/usr/bin/env python3
"""Probe a running simulation MoveIt scene and remove only the probe object.

Run in the ROS domain of the disposable simulation, never against hardware.
"""

import time
import uuid

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from shape_msgs.msg import SolidPrimitive

from pick_place_scene.scene_sync import SceneSync


def main():
    rclpy.init()
    object_id = 'stage0_probe_' + uuid.uuid4().hex
    node = SceneSync(parameter_overrides=[Parameter('allowed_ids', value=[object_id])])
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def call(client, request):
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=6)
        if not future.done() or future.result() is None:
            raise RuntimeError('MoveIt service response missing')
        return future.result()

    def objects():
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        return call(node.scene_client, request).scene.world.collision_objects

    attempted = False
    removal_confirmed = False
    try:
        assert node.scene_client.wait_for_service(timeout_sec=5), 'query service unavailable'
        assert node.apply_client.wait_for_service(timeout_sec=5), 'apply service unavailable'
        obj = CollisionObject(id=object_id)
        obj.header.frame_id = 'base_link'
        obj.header.stamp = node.get_clock().now().to_msg()
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02, 0.03, 0.04])]
        pose = Pose()
        pose.position.x, pose.position.z, pose.orientation.w = 1.5, 1.0, 1.0
        obj.primitive_poses = [pose]
        attempted = True
        node.on_observation(obj)
        deadline = time.monotonic() + 8
        while node.transactions and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert object_id in node.confirmed_ids, 'scene update not confirmed'
        assert any(item.id == object_id for item in objects()), 'probe missing'
        print('PASS SceneSync confirms actual MoveIt canonical geometry')
        removal = CollisionObject(id=object_id)
        removal.header.frame_id = 'base_link'
        removal.header.stamp = node.get_clock().now().to_msg()
        removal.operation = CollisionObject.REMOVE
        node.on_observation(removal)
        deadline = time.monotonic() + 8
        while node.transactions and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert object_id not in node.confirmed_ids, 'probe removal not confirmed'
        assert all(item.id != object_id for item in objects()), 'probe removal left object'
        removal_confirmed = True
        print('PASS SceneSync confirms actual MoveIt removal')
    finally:
        try:
            if attempted and not removal_confirmed:
                request = ApplyPlanningScene.Request()
                request.scene = PlanningScene(is_diff=True)
                request.scene.robot_state.is_diff = True
                request.scene.world.collision_objects = [CollisionObject(
                    id=object_id, operation=CollisionObject.REMOVE)]
                assert call(node.apply_client, request).success, 'probe removal rejected'
                assert all(item.id != object_id for item in objects()), 'probe removal unconfirmed'
                print('PASS probe cleanup removed and absence confirmed')
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
