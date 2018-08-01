#!/usr/bin/env python

import rospy
import rospkg
import re
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from mujoco_ros_msgs.msg import FreeObjectsStates
from shape_msgs.msg import SolidPrimitive
from mujoco2rviz.utilities import compare_poses, stl_to_mesh, get_object_mesh_path, get_object_type_from_name


class Mujoco2Rviz():
    def __init__(self):
        self.model_cache = {}
        self.description_repo_path = rospy.get_param('~description_repo_path',
                                                     rospkg.RosPack().get_path('sr_description_common'))
        self.objects_states_subscriber = rospy.Subscriber('mujoco/free_objects_states', FreeObjectsStates, self.objects_states_cb)
        self.collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=5,
                                                          latch=True)

        self.pose = Pose()
        self.pose.position.x = 0.5
        self.pose.position.y = 0.5
        self.pose.position.z = 0.5
        self.pose.orientation.x = 0
        self.pose.orientation.y = 0
        self.pose.orientation.z = 0
        self.pose.orientation.w = 1

        self.publish_objects_to_rviz()

    def objects_states_cb(self, objects_states_msg):
        # create collision object if name is not present in model_cache
        for model_instance_name, model_instance_pose in zip(objects_states_msg.name, objects_states_msg.pose):
            if model_instance_name == 'box':
                self.model_cache[model_instance_name] = self.create_collision_object_from_primitive(model_instance_name, self.pose, 'box', [0.1, 0.1, 0.1])
            # if not model_instance_name in self.model_cache:
            #     try:
            #         self.model_cache[model_instance_name] = self.create_collision_object_from_mesh(model_instance_name, model_instance_pose)
            #         rospy.loginfo("Added object {} to rviz".format(model_instance_name))
            #     except:
            #         rospy.logwarn("Failed to add {} collision object".format(model_instance_name))

            # # check if model moved, temporarily remove it from scene and update pose in model_cache
            # if not compare_poses(model_instance_pose, self.model_cache[model_instance_name].mesh_poses[0]):
            #     object_to_be_temporarily_removed = self.create_collision_object_from_mesh(model_instance_name, model_instance_pose, False)
            #     self.collision_object_publisher.publish(object_to_be_temporarily_removed)
            #     self.model_cache[model_instance_name].mesh_poses[0] = model_instance_pose

    def publish_objects_to_rviz(self):
        while not rospy.is_shutdown():
            for model_instance_name in self.model_cache.keys():
                self.collision_object_publisher.publish(self.model_cache[model_instance_name])

    def create_collision_object_from_mesh(self, model_instance_name, model_pose, add=True):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = '{}__link'.format(model_instance_name)
        if add:
            object_type = get_object_type_from_name(model_instance_name)
            object_mesh_path = get_object_mesh_path(object_type, self.description_repo_path)
            collision_object.operation = CollisionObject.ADD
            object_mesh = stl_to_mesh(object_mesh_path)
            collision_object.meshes = [object_mesh]
            collision_object.mesh_poses = [model_pose]
        else:
            collision_object.operation = CollisionObject.REMOVE
        return collision_object

    def create_collision_object_from_primitive(self, model_instance_name, model_pose, type, size, add=True):
        collision_object = CollisionObject()
        primitive = SolidPrimitive()
        collision_object.header.frame_id = 'world'
        collision_object.id = '{}__link'.format(model_instance_name)
        if add:
            collision_object.operation = CollisionObject.ADD
        else:
            collision_object.operation = CollisionObject.REMOVE

        if 'box' == type:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = size

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(model_pose)
        return collision_object

if __name__ == '__main__':
    rospy.init_node('mujoco_to_rviz', anonymous=True)
    m2m = Mujoco2Rviz()
