#!/usr/bin/env python
"""
Publish all collision elements within a SDF file as MoveIt CollisionObjects
"""

import argparse
import os
try:
  from pyassimp import pyassimp
except:
  # support pyassimp > 3.0
  import pyassimp

from pyassimp.errors import AssimpError
import os.path

import rospy
from geometry_msgs.msg import PoseStamped, Point
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from tf.transformations import *

import pysdf

class Sdf2moveit(object):
    def __init__(self):
        self.ignored_submodels = []
        self.collision_objects = {}
        self.collision_objects_updated = {}


        self.planning_scene_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        while self.planning_scene_pub.get_num_connections() < 1:
            rospy.sleep(0.1)

        timeout = rospy.get_param('~timeout', 0)
        if timeout <= 0:
            timeout = None
        rospy.loginfo('Waiting {} for /get_planning_scene service to be advertised.'.format(
                      '{} seconds'.format(timeout) if timeout else 'indefinitely'))
        rospy.wait_for_service('/get_planning_scene', timeout)
        rospy.loginfo('/get_planning_scene service has been advertised, proceeding.')
        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.request = PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_NAMES)

    # Slightly modified PlanningSceneInterface.__make_mesh from moveit_commander/src/moveit_commander/planning_scene_interface.py
    def make_mesh(self, co, pose, filename, scale=(1.0, 1.0, 1.0)):
        try:
            scene = pyassimp.load(filename)
        except AssimpError as e:
            rospy.logerr("Assimp error: %s", e)
            return False
        if not scene.meshes or len(scene.meshes) == 0:
            raise MoveItCommanderException("There are no meshes in the file")
        if len(scene.meshes[0].faces) == 0:
            raise MoveItCommanderException("There are no faces in the mesh")

        mesh = Mesh()
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
                    triangle.vertex_indices = [face.indices[0],
                                               face.indices[1],
                                               face.indices[2]]
                    mesh.triangles.append(triangle)
        else:
            raise MoveItCommanderException("Unable to build triangles from mesh due to mesh object structure")
        for vertex in scene.meshes[0].vertices:
            point = Point()
            point.x = vertex[0] * scale[0]
            point.y = vertex[1] * scale[1]
            point.z = vertex[2] * scale[2]
            mesh.vertices.append(point)
        if not isinstance(co.meshes, list):
            co.meshes = []
        co.meshes += [mesh]

        if not isinstance(co.mesh_poses, list):
            co.mesh_poses = []
        co.mesh_poses += [pose.pose]

        pyassimp.release(scene)
        return co

    def is_ignored(self, model):
        model_full_name = model.get_full_name()
        for ignored_submodel in self.ignored_submodels:
            if model_full_name == ignored_submodel or model_full_name.endswith('::' + ignored_submodel):
                return True
        return False

    def link_to_collision_object(self, link, full_linkname):
        mesh_path = None
        supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']
        linkparts = getattr(link, 'collisions')
        if self.is_ignored(link.parent_model):
            print("Ignoring link %s." % full_linkname)
            return

        collision_object = CollisionObject()
        collision_object.header.frame_id = pysdf.sdf2tfname(full_linkname)

        for linkpart in linkparts:
            if linkpart.geometry_type not in supported_geometry_types:
                print("Element %s with geometry type %s not supported. Ignored." % (full_linkname, linkpart.geometry_type))
                continue

            if linkpart.geometry_type == 'mesh':
                scale = tuple(float(val) for val in linkpart.geometry_data['scale'].split())
                for models_path in pysdf.models_paths:
                    resource = linkpart.geometry_data['uri'].replace('model://', models_path)
                    if os.path.isfile(resource):
                        mesh_path = resource
                        break
                if mesh_path is not None:
                    link_pose_stamped = PoseStamped()
                    link_pose_stamped.pose = pysdf.homogeneous2pose_msg(linkpart.pose)
                    if not self.make_mesh(collision_object, link_pose_stamped, mesh_path, scale):
                        return None
            elif linkpart.geometry_type == 'box':
                scale = tuple(float(val) for val in linkpart.geometry_data['size'].split())
                box = SolidPrimitive()
                box.type = SolidPrimitive.BOX
                box.dimensions = scale
                collision_object.primitives.append(box)
                collision_object.primitive_poses.append(pysdf.homogeneous2pose_msg(linkpart.pose))
            elif linkpart.geometry_type == 'sphere':
                sphere = SolidPrimitive()
                sphere.type = SolidPrimitive.SPHERE
                sphere.dimensions = [float(linkpart.geometry_data['radius'])]
                collision_object.primitives.append(sphere)
                collision_object.primitive_poses.append(pysdf.homogeneous2pose_msg(linkpart.pose))
            elif linkpart.geometry_type == 'cylinder':
                cylinder = SolidPrimitive()
                cylinder.type = SolidPrimitive.CYLINDER
                cylinder.dimensions = tuple(
                    (float(linkpart.geometry_data['length']), float(linkpart.geometry_data['radius'])))
                collision_object.primitives.append(cylinder)
                collision_object.primitive_poses.append(pysdf.homogeneous2pose_msg(linkpart.pose))
        return collision_object

    def convert_to_collision_object(self, link, full_linkname, **kwargs):
        if 'name' in kwargs:
            modelinstance_name = kwargs['name']

        full_linkname_mod = modelinstance_name + "::" + full_linkname.split("::")[1]
        collision_object = self.link_to_collision_object(link, full_linkname_mod)
        if not collision_object:
            return

        link_root = collision_object.header.frame_id
        if link_root not in self.collision_objects:
            self.collision_objects[link_root] = CollisionObject()
            self.collision_objects[link_root].id = link_root
            self.collision_objects[link_root].operation = CollisionObject.ADD
        self.append_to_collision_object(self.collision_objects[link_root], collision_object)

    def append_to_collision_object(self, sink_collision_object, source_collision_object):
        sink_collision_object.primitives.extend(source_collision_object.primitives)
        sink_collision_object.primitive_poses.extend(source_collision_object.primitive_poses)
        sink_collision_object.meshes.extend(source_collision_object.meshes)
        sink_collision_object.mesh_poses.extend(source_collision_object.mesh_poses)
        sink_collision_object.planes.extend(source_collision_object.planes)
        sink_collision_object.plane_poses.extend(source_collision_object.plane_poses)

    def update_collision_object(self, link, full_linkname, **kwargs):
        if 'name' in kwargs:
            modelinstance_name = kwargs['name']
    
        full_linkname_mod = modelinstance_name + "::" + full_linkname.split("::")[1]
        link_root = pysdf.sdf2tfname(full_linkname_mod)
        self.collision_objects_updated[link_root] = CollisionObject()
        self.collision_objects_updated[link_root].id = link_root
        self.collision_objects_updated[link_root].operation = CollisionObject.MOVE
        if 'pose' in kwargs:
            updated_pose = kwargs['pose']
            self.move_collision_object(self.collision_objects_updated[link_root], self.collision_objects[link_root], updated_pose)

    def move_collision_object(self, sink_collision_object, source_collision_object, updated_pose):
        link_world = pysdf.pose_msg2homogeneous(updated_pose)
        for pose in source_collision_object.primitive_poses:
            primitive_pose_in_link = pysdf.pose_msg2homogeneous(pose)
            primitive_pose_in_world = pysdf.homogeneous2pose_msg(concatenate_matrices(link_world, primitive_pose_in_link))
            sink_collision_object.primitive_poses.extend([primitive_pose_in_world])
        for pose in source_collision_object.mesh_poses:
            mesh_pose_in_link = pysdf.pose_msg2homogeneous(pose)
            mesh_pose_in_world = pysdf.homogeneous2pose_msg(concatenate_matrices(link_world, mesh_pose_in_link))
            sink_collision_object.mesh_poses.extend([mesh_pose_in_world])
        for pose in source_collision_object.plane_poses:
            plane_pose_in_link = pysdf.pose_msg2homogeneous(pose)
            plane_pose_in_world = pysdf.homogeneous2pose_msg(concatenate_matrices(link_world, plane_pose_in_link))
            sink_collision_object.plane_poses.extend([plane_pose_in_world])

    def add_new_collision_object(self, model_name, modelinstance_name):
        sdf = pysdf.SDF(model=model_name)
        num_collision_objects = len(self.collision_objects)
        model = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
        if model:
            model.for_all_links(self.convert_to_collision_object, name=modelinstance_name)
            if len(self.collision_objects) == num_collision_objects:
                rospy.logerr('Unable to load model: %s' % model_name)
                return None
            planning_scene_msg = PlanningScene()
            planning_scene_msg.is_diff = True
            for (collision_object_root, collision_object) in self.collision_objects.iteritems():
                if collision_object_root in self.ignored_submodels:
                    pass
                else:
                    planning_scene_msg.world.collision_objects.append(collision_object)
                    planning_scene_msg.world.collision_objects[-1].header.frame_id = 'world'
            self.planning_scene_pub.publish(planning_scene_msg)
            rospy.loginfo('Loaded model: %s' % modelinstance_name)
            return model
        else:
            rospy.logerr('Unable to load model: %s' % model_name)
            return None

    def delete_collision_object(self, modelinstance_name):
        for id in [object.id for key, object in self.collision_objects_updated.items() if modelinstance_name in key.lower()]:
            del self.collision_objects_updated[id]
            planning_scene_msg = PlanningScene()
            planning_scene_msg.is_diff = True
            deleted_obj = CollisionObject()
            deleted_obj.id = id
            deleted_obj.operation = CollisionObject.REMOVE
            planning_scene_msg.world.collision_objects.append(deleted_obj)
            planning_scene_msg.world.collision_objects[-1].header.frame_id = 'world'
            self.planning_scene_pub.publish(planning_scene_msg)

    def update_collision_object_with_pose(self, model, modelinstance_name, pose):
        if model:
            model.for_all_links(self.update_collision_object, name=modelinstance_name, pose=pose)
        else:
            return

        response = self.get_planning_scene(self.request)
        current_scene_objects = [object.id for object in response.scene.world.collision_objects]
 
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True
        for (collision_object_root, collision_object) in self.collision_objects_updated.iteritems():
            if collision_object_root in current_scene_objects:
                # Object is present in the planning scene
                if collision_object_root in self.ignored_submodels:
                    pass
                else:
                    planning_scene_msg.world.collision_objects.append(collision_object)
                    planning_scene_msg.world.collision_objects[-1].header.frame_id = 'world'
        self.planning_scene_pub.publish(planning_scene_msg)
