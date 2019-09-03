#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import re
import os
import rospy
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


def get_object_name_from_instance(object_instance):
    object_type = re.sub('_[0-9]*$', '', object_instance)
    object_type = re.sub('@.*$', '', object_type)
    return object_type


def get_object_mesh_path(object_name, description_repo_path):
    file_name = '{}.stl'.format(object_name)
    path_to_mesh = None
    for dir, sub_dirs, files in os.walk(description_repo_path):
        for file in files:
            if file == file_name:
                path_to_mesh = os.path.abspath('{}/{}'.format(dir, file_name))

    if path_to_mesh is not None:
        return path_to_mesh
    else:
        raise IOError("Mesh for {} not found".format(object_name))
