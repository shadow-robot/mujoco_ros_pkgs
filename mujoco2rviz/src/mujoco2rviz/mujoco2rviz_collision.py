#!/usr/bin/env python

import rospy
import rospkg
import re
from shape_msgs.msg import Mesh, MeshTriangle
import argparse
import pyassimp
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject
from mujoco_ros_msgs.msg import FreeObjectsStates
from visualization_msgs.msg import Marker

class Mujoco2Marker():
    def __init__(self):
        self.model_cache = {}
        self.description_repo_path = rospy.get_param("description_repo_path",
                                                     rospkg.RosPack().get_path('sr_description_common'))
        self.objects_states_subscriber = rospy.Subscriber('mujoco/free_objects_states', FreeObjectsStates, self.objects_states_cb)
        self.collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=5,
                                                          latch=True)

    def objects_states_cb(self, objects_states_msg):
        for model_instance_name, model_instance_pose in zip(objects_states_msg.name, objects_states_msg.pose):
            if not model_instance_name in self.model_cache:
                print model_instance_name, model_instance_pose
                self.model_cache[model_instance_name] = self.add_new_collision_object(model_instance_name, model_instance_pose)

            self.collision_object_publisher.publish(self.model_cache[model_instance_name])

        #     self.sdf2moveit.update_collision_object_with_pose(self.model_cache[model_instance_name], model_instance_name, objects_states_msg.pose[model_idx])

        # for model_instance_name in list(self.model_cache):
        #     if model_instance_name not in objects_states_msg.name:
        #         rospy.loginfo("Object %s deleted from gazebo" % model_instance_name)
        #         self.sdf2moveit.delete_collision_object(model_instance_name)
        #         del self.model_cache[model_instance_name]

    def add_new_collision_object(self, model_instance_name, model_pose, add=True):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = '{}__link'.format(model_instance_name)
        if add:
            object_mesh_path = self.get_object_mesh_path(self.get_object_type(model_instance_name))
            print object_mesh_path
            collision_object.operation = CollisionObject.ADD
            collision_object.meshes = [self.stl_to_mesh(object_mesh_path)]
            collision_object.mesh_poses = [model_pose]
        else:
            collision_object.operation = CollisionObject.REMOVE
        return collision_object

    def get_object_mesh_path(self, object_name):
        return '{}/models/{}/meshes/{}.stl'.format(self.description_repo_path, object_name, object_name)

    def get_object_type(self, object_instance):
        object_type = re.sub('_[0-9]*$', '', object_instance)
        object_type = re.sub('@.*$', '', object_type)
        return object_type

    def stl_to_mesh(self, filename, scale=(1, 1, 1)):
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