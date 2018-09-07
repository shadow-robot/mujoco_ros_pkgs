#!/usr/bin/python

# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import rospkg
import subprocess
import os.path
import xml.etree.ElementTree as xmlTool
from xml.dom import minidom
from mujoco_ros_msgs.srv import SpawnObjects, SpawnObjectsResponse
from std_srvs.srv import Trigger, TriggerResponse


class SpawnSimulation(object):
    """
    Class for spawning Mujoco Simulations with given objects
    """

    def __init__(self):
        self._xml_config_dir = rospkg.RosPack().get_path("fh_description") + "/mujoco_models"
        base_mujoco_env_filename = rospy.get_param("~base_mujoco_env_filename", "ur10_fh_environment.xml")
        self._generated_mujoco_env_filename = rospy.get_param("~generated_mujoco_env_filename", "test.xml")
        self._base_config_xml = xmlTool.parse('{}/{}'.format(self._xml_config_dir, base_mujoco_env_filename))
        self._obj_names_list = []
        self._subprocess = []
        rospy.Service("mujoco/spawn_sim_environment", SpawnObjects, self._spawn_sim_environment_service)
        rospy.Service("mujoco/terminate_sim", Trigger, self._terminate_sim_service)

    def _get_file_mesh_directory(self, mesh_name):
        """
        Search for stl file of the object in the mesh directory
        specified by the user
        @param mesh_name - string
        """
        mesh_directory_path = None
        absolute_mesh_dir_path = rospy.get_param("~mesh_directory", self._xml_config_dir + "/meshes")
        rospy.loginfo("Looking for mesh in path {}".format(absolute_mesh_dir_path))
        for dir, sub_dirs, files in os.walk(absolute_mesh_dir_path):
            for file in files:
                if mesh_name == file:
                    rospy.loginfo("Mesh found")
                    mesh_directory_path = os.path.relpath(dir, self._xml_config_dir)
        if mesh_directory_path is not None:
            return mesh_directory_path
        else:
            raise IOError("Mesh {} not found".format(mesh_name))

    def _spawn_sim_environment_service(self, req):
        """
        Service to receive request of spawning mujoco environment
        with list of objects
        @param req - SpawnObject.srv request RecognizedObjectArray msg
        """
        try:
            for obj in req.objects.objects:
                self._obj_names_list.append(obj.type.key)
                self._append_object_to_xml(obj.type.key, obj.pose.pose.pose)
        except IOError as e:
            rospy.logerr("Could not load objects: {}".format(e))
        else:
            rospy.loginfo("Starting simulation..")
            try:
                process = subprocess.Popen(['xterm -e roslaunch fh_robot_launch fh_ur10_and_fh2_mujoco.launch \
                                            sim:=true grasp_controller:=true scene:=false \
                                            robot_model_path:={}/{}'.format(self._xml_config_dir,
                                            self._generated_mujoco_env_filename)], shell=True)
            except OSError as e:
                rospy.logerr("Could not spawn simulation")
                process.kill()
                success = False
            else:
                rospy.loginfo("Simulation successfully spawned")
                success = True
                self._subprocess.append(process)
            return SpawnObjectsResponse(success)

    def _append_object_to_xml(self, obj_name, obj_pose):
        """
        Write the xml mujoco file of the requested environment
        @param obj_name - string
        @param obj_pose - geometry_msgs/Pose
        """
        obj_instances_nr = self._obj_names_list.count(obj_name)
        obj_name = obj_name + "_{}".format(obj_instances_nr)
        mesh_name = obj_name[:-2] + '.stl'
        mesh_directory_name = self._get_file_mesh_directory(mesh_name)

        obj_position = [obj_pose.position.x, obj_pose.position.y, obj_pose.position.z]
        obj_orientation = [obj_pose.orientation.w, obj_pose.orientation.x,
                           obj_pose.orientation.y, obj_pose.orientation.z]
        obj_position_string = " ".join(map(str, obj_position))
        obj_orientation_string = " ".join(map(str, obj_orientation))

        rospy.loginfo("Adding {} to Mujoco environment file..".format("obj_name"))

        # append new body to xml file
        for child in self._base_config_xml.getroot():
            if child.tag == "asset":
                mesh_tag = xmlTool.SubElement(child, "mesh", {'name': obj_name, 'file': mesh_directory_name +
                                                              '/' + mesh_name})
            if child.tag == "worldbody":
                body_tag = xmlTool.SubElement(child, "body", {'name': obj_name, 'pos': obj_position_string,
                                                              'quat': obj_orientation_string})
                joint_tag = xmlTool.SubElement(body_tag, "joint", {'type': 'free', 'armature': '0.01'})
                geom_tag = xmlTool.SubElement(body_tag, "geom", {'type': 'mesh', 'rgba': '0.7 0.7 0.7 1',
                                                                 'mesh': obj_name, 'condim': '4',
                                                                 'friction': '1.7 0.5 0.1', 'solimp': '0.99 0.99 0.01',
                                                                 'solref': '0.01 1', 'contype': '1'})
        self._base_config_xml.write("{}/{}".format(self._xml_config_dir, self._generated_mujoco_env_filename))

    def _terminate_sim_service(self, req):
        """
        Service to terminate simulation
        """
        rospy.loginfo("Terminating simulation..")
        for process in self._subprocess:
            process.returncode = 0
            process.kill()
            process.wait()
            if process.poll() == process.returncode:
                success = True
                msg = "Simulation correctly terminated"
                rospy.loginfo("Simulation terminated")
            else:
                success = False
                msg = "Could not terminate simulation"
        return TriggerResponse(success, msg)


if __name__ == '__main__':
    rospy.init_node("spawn_simulation_node")
    spawn_sim = SpawnSimulation()
    rospy.spin()
