#!/usr/bin/env python

import re
import pyassimp
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point


def compare_poses(pose_1, pose_2, epsilon_translation=0.001, epsilon_rotation=0.01):
    poses_equal = True
    if abs(pose_1.position.x - pose_2.position.x) > epsilon_translation or \
        abs(pose_1.position.y - pose_2.position.y) > epsilon_translation or \
        abs(pose_1.position.z - pose_2.position.z) > epsilon_translation or \
        abs(pose_1.orientation.x - pose_2.orientation.x) > epsilon_rotation or \
        abs(pose_1.orientation.y - pose_2.orientation.y) > epsilon_rotation or \
        abs(pose_1.orientation.z - pose_2.orientation.z) > epsilon_rotation or \
        abs(pose_1.orientation.w - pose_2.orientation.w) > epsilon_rotation:
        poses_equal = False
    return poses_equal


def stl_to_mesh(filename, scale=(1, 1, 1)):
    mesh = Mesh()
    scene = pyassimp.load(filename)
    first_face = scene.meshes[0].faces[0]
    if hasattr(first_face, '__len__'):
        for face in scene.meshes[0].faces:
            if len(face) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face[0], face[1], face[2]]
                mesh.triangles.append(triangle)
    elif hasattr(first_face, 'indices'):
        for face in scene.meshes[0].faces:
            if len(face.indices) == 3:
                triangle = MeshTriangle()
                triangle.vertex_indices = [face.indices[0], face.indices[1], face.indices[2]]
                mesh.triangles.append(triangle)
    else:
        raise MoveItCommanderException("Unable to build triangles from mesh due to mesh object structure")
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0] * scale[0]
        point.y = vertex[1] * scale[1]
        point.z = vertex[2] * scale[2]
        mesh.vertices.append(point)
    pyassimp.release(scene)
    return mesh


def get_object_type_from_name(object_instance):
    object_type = re.sub('_[0-9]*$', '', object_instance)
    object_type = re.sub('@.*$', '', object_type)
    return object_type


def get_object_mesh_path(object_name, description_repo_path):
    return '{}/models/{}/meshes/{}.stl'.format(description_repo_path, object_name, object_name)
