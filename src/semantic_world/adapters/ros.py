from __future__ import annotations

import atexit
import logging
import threading
import time
from dataclasses import dataclass

# from roslaunch.core import get_ros_package_path as get_ros_package_paths

from ..geometry import Shape, Mesh, Sphere, Cylinder, Box, Color
from ..spatial_types import Vector3, Quaternion
from ..utils import IDGenerator
from ..world import World

try:
    import rospy
    from rospy.rostime import Time as ROSTime
    from visualization_msgs.msg import MarkerArray, Marker
    from geometry_msgs.msg import (Vector3 as ROSVector3, Quaternion as ROSQuaternion, Pose as ROSPose,
                                   PoseStamped as ROSPoseStamped, Header as ROSHeader, Color as ROSColor, )
except ImportError:
    logging.warn("Importing ros adapter without rospy available.")


def get_ros_package_path(package_name: str) -> str:
    """
    Placeholder for ros integration
    """
    return [p for p in get_ros_package_paths().split(':') if p.endswith(package_name)][0]


id_generator = IDGenerator()

#
# def vector3_to_ros_message(obj: Vector3) -> ROSVector3:
#     return ROSVector3(x=obj.x, y=obj.y, z=obj.z)
#
#
# def quaternion_to_ros_message(obj: Quaternion) -> ROSQuaternion:
#     return ROSQuaternion(x=obj.x, y=obj.y, z=obj.z, w=obj.w)
#
#
# def pose_to_ros_message(obj: Pose) -> ROSPose:
#     return ROSPose(position=vector3_to_ros_message(obj.position),
#                    orientation=quaternion_to_ros_message(obj.orientation))
#
#
# def header_to_ros_message(obj: Header) -> ROSHeader:
#     stamp = ROSTime.from_sec(obj.timestamp.timestamp())
#     return ROSHeader(frame_id=obj.frame_id, stamp=stamp, seq=obj.sequence)
#
#
# def pose_stamped_to_ros_message(obj: PoseStamped) -> PoseStamped:
#     return ROSPoseStamped(pose=pose_to_ros_message(obj.pose), header=header_to_ros_message(obj.header))
#
#
# def color_to_ros_message(obj: Color):
#     return ROSColor(rgba=vector3_to_ros_message(obj.rgba), )


def shape_to_ros_message(obj: Shape) -> Marker:
    marker = Marker()
    marker.header = header_to_ros_message(obj.origin.header)
    marker.action = Marker.ADD
    marker.pose = pose_to_ros_message(obj.origin.pose)
    marker.scale = vector3_to_ros_message(Vector3(1., 1., 1.))
    marker.color = color_to_ros_message(Color())
    marker.lifetime = rospy.Duration.from_sec(10)
    marker.ns = "shapes"
    marker.id = id_generator(id(obj))

    return adapter_methods[type(obj)](obj, marker)


def mesh_to_ros_message(obj: Mesh, marker: Marker) -> Marker:
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "file://" + obj.filename
    marker.mesh_use_embedded_materials = True
    marker.scale = vector3_to_ros_message(obj.scale)
    return marker


def sphere_to_ros_message(obj: Sphere, marker: Marker) -> Marker:
    marker.color = color_to_ros_message(obj.color)
    marker.type = Marker.SPHERE
    marker.scale = vector3_to_ros_message(Vector3(obj.radius * 2, obj.radius * 2, obj.radius * 2))
    return marker


def cylinder_to_ros_message(obj: Cylinder, marker: Marker) -> Marker:
    marker.color = color_to_ros_message(obj.color)
    marker.type = Marker.CYLINDER
    marker.scale = vector3_to_ros_message(Vector3(obj.radius * 2, obj.radius * 2, obj.length))
    return marker


def box_to_ros_message(obj: Box, marker: Marker) -> Marker:
    marker.color = color_to_ros_message(obj.color)
    marker.type = Marker.CUBE
    marker.scale = vector3_to_ros_message(Vector3(obj.length, obj.width, obj.height))
    return marker


adapter_methods = {
    Mesh: mesh_to_ros_message,
    Sphere: sphere_to_ros_message,
    Cylinder: cylinder_to_ros_message,
    Box: box_to_ros_message,
}


@dataclass
class WorldPublisher:
    """
    Publishes the visuals of every link in the world to a ros topic.
    The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the
    World. This Array is published with a rate of `interval`.
    The publisher runs in its own thread.
    """

    world: World
    """
    The world to read the links from.
    """

    topic_name: str = "/world/viz_marker"
    """
    The name of the topic to publish the visuals to.
    """

    interval: float = 0.1
    """
    The interval in seconds at which to publish the visuals.
    """

    def __post_init__(self):
        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)
        self.thread = threading.Thread(target=self._publish, name="WorldPublisher")
        self.kill_event = threading.Event()
        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()
            self.pub.publish(marker_array)
            time.sleep(self.interval)

    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        marker_array = MarkerArray()
        for body in self.world.bodies:
            for shape in body.visual:
                marker = shape.ros_message()
                marker_array.markers.append(marker)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()