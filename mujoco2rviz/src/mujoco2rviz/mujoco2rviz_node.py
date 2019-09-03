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

import rospy
import rospkg
import re
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from mujoco_ros_msgs.msg import ModelStates
from shape_msgs.msg import SolidPrimitive
from sr_utilities_common.shutdown_handler import ShutdownHandler
from mujoco2rviz.utilities import compare_poses, stl_to_mesh, get_object_mesh_path, get_object_name_from_instance


class Mujoco2Rviz():
    def __init__(self):
        self._model_cache = {}
        self._ignored_models = []
        self._description_repo_path = rospy.get_param('~description_repo_path',
                                                      rospkg.RosPack().get_path('sr_description_common'))
        self._static_only = rospy.get_param('~static_only', True)
        self._objects_states_subscriber = rospy.Subscriber('mujoco/model_states', ModelStates,
                                                           self._objects_states_cb)
        self._collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=2,
                                                           latch=True)

    def publish_objects_to_rviz(self, publishing_rate=20):
        rate = rospy.Rate(publishing_rate)
        while not rospy.is_shutdown():
            for model_instance_name in self._model_cache.keys():
                self._collision_object_publisher.publish(self._model_cache[model_instance_name])
            rate.sleep()

    def clean_up(self):
        rospy.loginfo("Cleaning up!")
        for model_instance_name in self._model_cache.keys():
            self._model_cache[model_instance_name].operation = CollisionObject.REMOVE
            self._collision_object_publisher.publish(self._model_cache[model_instance_name])
            rospy.sleep(1)
        rospy.loginfo("All cleaned up, shutting down...")

    def _objects_states_cb(self, objects_states_msg):
        self._add_and_publish_objects(objects_states_msg)
        self._update_objects(objects_states_msg)

    def _add_and_publish_objects(self, message):
        for model_idx, model_instance_name in enumerate(message.name):
            if self._static_only and not message.is_static[model_idx]:
                continue

            if model_instance_name not in self._model_cache:
                try:
                    self._model_cache[model_instance_name] = self._create_collision_object_from_msg(message,
                                                                                                    model_idx)
                    if model_instance_name in self._ignored_models:
                        self._ignored_models.remove(model_instance_name)
                    rospy.loginfo("Added object {} to rviz".format(model_instance_name))
                except (TypeError, IOError) as e:
                    if model_instance_name not in self._ignored_models:
                        self._ignored_models.append(model_instance_name)
                        rospy.logwarn("Failed to add {} collision object: {}".format(model_instance_name, e))

    def _update_objects(self, message):
        for model_idx, model_instance_name in enumerate(message.name):
            if model_instance_name in self._model_cache:
                if ModelStates.MESH == message.type[model_idx]:
                    if not compare_poses(message.pose[model_idx],
                                         self._model_cache[model_instance_name].mesh_poses[0]):
                        self._model_cache[model_instance_name].operation = CollisionObject.MOVE
                        self._model_cache[model_instance_name].mesh_poses[0] = message.pose[model_idx]
                else:
                    if not compare_poses(message.pose[model_idx],
                                         self._model_cache[model_instance_name].primitive_poses[0]):
                        self._model_cache[model_instance_name].operation = CollisionObject.MOVE
                        self._model_cache[model_instance_name].primitive_poses[0] = message.pose[model_idx]

    def _create_collision_object_from_msg(self, message, model_idx):
        if ModelStates.MESH == message.type[model_idx]:
            collision_object = self._create_collision_object_from_mesh(message.name[model_idx],
                                                                       message.pose[model_idx])
        else:
            collision_object = self._create_collision_object_from_primitive(message.name[model_idx],
                                                                            message.pose[model_idx],
                                                                            message.type[model_idx],
                                                                            message.size[model_idx].data)
        return collision_object

    def _create_collision_object_from_mesh(self, model_instance_name, model_pose):
        collision_object = self._create_collision_object_base(model_instance_name)
        object_type = get_object_name_from_instance(model_instance_name)
        object_mesh_path = get_object_mesh_path(object_type, self._description_repo_path)
        object_mesh = stl_to_mesh(object_mesh_path)
        collision_object.meshes = [object_mesh]
        collision_object.mesh_poses = [model_pose]
        return collision_object

    def _create_collision_object_from_primitive(self, model_instance_name, model_pose, model_type, size):
        collision_object = self._create_collision_object_base(model_instance_name)
        primitive = SolidPrimitive()
        if ModelStates.BOX == model_type:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [i * 2 for i in size]
        elif ModelStates.CYLINDER == model_type:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [size[1] * 2, size[0]]
        elif ModelStates.SPHERE == model_type:
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [size[0]]
        else:
            raise TypeError("Primitive type {} not supported".format(model_type))
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(model_pose)
        return collision_object

    def _create_collision_object_base(self, model_instance_name):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = '{}__link'.format(model_instance_name)
        collision_object.operation = CollisionObject.ADD
        return collision_object

if __name__ == '__main__':
    rospy.init_node('mujoco_to_rviz', anonymous=True)
    mujoco_to_rviz = Mujoco2Rviz()
    shutdown_handler = ShutdownHandler(mujoco_to_rviz, 'clean_up()')
    mujoco_to_rviz.publish_objects_to_rviz()
