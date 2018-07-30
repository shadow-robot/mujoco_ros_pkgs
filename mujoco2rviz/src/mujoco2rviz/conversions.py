import copy
import rospy
from rospkg import RosPack, ResourceNotFound
from visualization_msgs.msg import Marker
import pysdf
import os.path

protoMarkerMsg = Marker()
protoMarkerMsg.frame_locked = True
protoMarkerMsg.id = 0
protoMarkerMsg.action = Marker.ADD
protoMarkerMsg.mesh_use_embedded_materials = True
protoMarkerMsg.color.a = 0.0
protoMarkerMsg.color.r = 0.0
protoMarkerMsg.color.g = 0.0
protoMarkerMsg.color.b = 0.0
supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']

gazebo_rospack = RosPack()


def link2marker_msg(link, full_linkname, use_collision = False, lifetime = rospy.Duration(0)):
  marker_msg = None
  linkpart = None
  if use_collision:
    linkparts = getattr(link, 'collisions')
  else: # visual
    linkparts = getattr(link, 'visuals')

  msgs = []

  for linkpart in linkparts:
    if not linkpart.geometry_type in supported_geometry_types:
      if linkpart.geometry_type:
        print("Element %s with geometry type %s not supported. Ignored." % (full_linkname, linkpart.geometry_type))
        return None

    marker_msg = copy.deepcopy(protoMarkerMsg)
    marker_msg.header.frame_id = pysdf.sdf2tfname(full_linkname)
    marker_msg.header.stamp = rospy.get_rostime()
    marker_msg.lifetime = lifetime
    marker_msg.ns = pysdf.sdf2tfname(full_linkname + "::" + linkpart.name)
    marker_msg.pose = pysdf.homogeneous2pose_msg(linkpart.pose)

    if linkpart.geometry_type == 'mesh':
      marker_msg.type = Marker.MESH_RESOURCE
      #print('linkpart.geometry_data: %s' % linkpart.geometry_data['uri'])
      for models_path in pysdf.models_paths:
        resource = linkpart.geometry_data['uri'].replace('model://', models_path + '/')
        #print('resource: %s' % resource)
        if os.path.isfile(resource):
          marker_msg.mesh_resource = 'file://' + resource
          #print('found resource %s at %s' % (linkpart.geometry_data['uri'], resource))
          break
      # support URDF-like resource paths starting with model://
      if not marker_msg.mesh_resource and linkpart.geometry_data['uri'].startswith('model://'):
        stripped_uri = linkpart.geometry_data['uri'].replace('model://', '')
        uri_parts = stripped_uri.split('/', 1)

        if len(uri_parts) == 2:
          package_name = uri_parts[0]
          try:
            package_path = gazebo_rospack.get_path(package_name)
            mesh_path = os.path.join(package_path, uri_parts[1])
            if os.path.isfile(mesh_path):
              marker_msg.mesh_resource = 'file://' + mesh_path
          except ResourceNotFound, e:
            pass

      if not marker_msg.mesh_resource:
        print('ERROR! could not find resource: %s' % linkpart.geometry_data['uri'])
        return None

      scale = (float(val) for val in linkpart.geometry_data['scale'].split())
      marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
    else:
      marker_msg.color.a = 1
      marker_msg.color.r = marker_msg.color.g = marker_msg.color.b = 0.5

    if linkpart.geometry_type == 'box':
      marker_msg.type = Marker.CUBE
      scale = (float(val) for val in linkpart.geometry_data['size'].split())
      marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
    elif linkpart.geometry_type == 'sphere':
      marker_msg.type = Marker.SPHERE
      marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 2.0 * float(linkpart.geometry_data['radius'])
    elif linkpart.geometry_type == 'cylinder':
      marker_msg.type = Marker.CYLINDER
      marker_msg.scale.x = marker_msg.scale.y = 2.0 * float(linkpart.geometry_data['radius'])
      marker_msg.scale.z = float(linkpart.geometry_data['length'])

    #print(marker_msg)
    msgs.append(marker_msg)
  return msgs
