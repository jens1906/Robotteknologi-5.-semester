#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import trimesh
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
import math

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q

class EnvironmentPublisher(Node):
    def __init__(self):
        super().__init__('environment_publisher')
        
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')
            
        self.publish_environment()

    def load_mesh(self, filename, scale_factor=0.001):
        pkg_path = get_package_share_directory('ur3e_workstation')
        file_path = os.path.join(pkg_path, 'urdf', filename)
        
        if not os.path.exists(file_path):
            self.get_logger().error(f"Mesh file not found: {file_path}")
            return None
            
        mesh_data = trimesh.load(file_path)
        
        msg = Mesh()
        
        # Vertices
        for v in mesh_data.vertices:
            p = Point()
            p.x = float(v[0]) * scale_factor
            p.y = float(v[1]) * scale_factor
            p.z = float(v[2]) * scale_factor
            msg.vertices.append(p)
            
        # Triangles
        for f in mesh_data.faces:
            t = MeshTriangle()
            t.vertex_indices = [int(f[0]), int(f[1]), int(f[2])]
            msg.triangles.append(t)
            
        return msg

    def create_collision_object(self, id, mesh_filename, x, y, z, roll, pitch, yaw):
        co = CollisionObject()
        co.id = id
        co.header.frame_id = 'world'
        co.operation = CollisionObject.ADD
        
        mesh = self.load_mesh(mesh_filename)
        if mesh:
            co.meshes.append(mesh)
            
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            
            q = quaternion_from_euler(roll, pitch, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            co.mesh_poses.append(pose)
            return co
        return None

    def publish_environment(self):
        scene = PlanningScene()
        scene.is_diff = True
        
        # Bordplade
        # <origin xyz="0 0 0" rpy="0 0 0"/>
        co_table = self.create_collision_object('bordplade', 'Bundplade.stl', 0, 0, 0, 0, 0, 0)
        if co_table:
            scene.world.collision_objects.append(co_table)
            
        # Mount
        # <origin xyz="0.200 0.0 0.250" rpy="0 0 0"/>
        co_mount = self.create_collision_object('mount', 'UR3e_mount.stl', 0.2, 0.0, 0.25, 0, 0, 0)
        if co_mount:
            scene.world.collision_objects.append(co_mount)
            
        # Testplade
        # <origin xyz="0.200 2.94 -0.4" rpy="1.5708 0 0"/>
        co_test = self.create_collision_object('testpladekurve', 'TestpladeKurve_v3.stl', 0.2, 2.94, -0.4, 1.5708, 0, 0)
        if co_test:
            scene.world.collision_objects.append(co_test)
            
        req = ApplyPlanningScene.Request()
        req.scene = scene
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('Environment published to planning scene!')
        else:
            self.get_logger().error('Failed to publish environment')

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentPublisher()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
