import atexit
import threading
import time

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Vector3, Point, PoseStamped, Quaternion, Pose
from scipy.spatial.transform import Rotation
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from ..geometry import Mesh, Box, Sphere, Cylinder, Primitive, TriangleMesh
from ..world import World

class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the World
    """

    def __init__(self, world: World, node, topic_name="/semworld/viz_marker", interval=0.1, reference_frame="map"):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each body in the World and publishes
        it to the given topic name at a fixed interval. The publisher automatically stops publishing when the process
        is killed.

        :param world: The World to which the Visualization Marker should be published.
        :param node: The ROS2 node that will be used to publish the visualization marker.
        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        :param reference_frame: The reference frame of the visualization marker.
        """
        self.interval = interval
        self.reference_frame = reference_frame
        self.world = world
        self.node = node

        self.pub = self.node.create_publisher(MarkerArray, topic_name, 10)

        self.thread = threading.Thread(target=self._publish, name="VizMarkerPublisher")
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
            for i, collision in enumerate(body.collision):
                msg = Marker()
                msg.header.frame_id = self.reference_frame
                msg.ns = body.name.name
                msg.id = i
                msg.action = Marker.ADD
                msg.pose = self.transform_to_pose(self.world.compute_forward_kinematics_np(self.world.root, body) @ collision.origin.to_np())
                msg.color = body.color if isinstance(body, Primitive) else ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                msg.lifetime = Duration(sec=1)

                if isinstance(collision, Mesh):
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + collision.filename
                    msg.scale = Vector3(x=float(collision.scale.x), y=float(collision.scale.y), z=float(collision.scale.z))
                    msg.mesh_use_embedded_materials = True
                elif isinstance(collision, TriangleMesh):
                    f = collision.file
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + f.name
                    msg.scale = Vector3(x=float(collision.scale.x), y=float(collision.scale.y),
                                        z=float(collision.scale.z))
                    msg.mesh_use_embedded_materials = True
                elif isinstance(collision, Cylinder):
                    msg.type = Marker.CYLINDER
                    msg.scale = Vector3(x=float(collision.width), y=float(collision.width), z=float(collision.height))
                elif isinstance(collision, Box):
                    msg.type = Marker.CUBE
                    msg.scale = Vector3(x=float(collision.scale.x), y=float(collision.scale.y), z=float(collision.scale.z))
                elif isinstance(collision, Sphere):
                    msg.type = Marker.SPHERE
                    msg.scale = Vector3(x=float(collision.radius * 2), y=float(collision.radius * 2), z=float(collision.radius * 2))

                marker_array.markers.append(msg)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()

    @staticmethod
    def transform_to_pose(transform: np.ndarray) -> Pose:
        """
        Converts a 4x4 transformation matrix to a PoseStamped message.

        :param transform: The transformation matrix to convert.
        :return: A PoseStamped message.
        """
        pose = Pose()
        pose.position = Point(**dict(zip(["x", "y", "z"], transform[:3, 3])))
        pose.orientation = Quaternion(**dict(zip(["x", "y", "z", "w"], Rotation.from_matrix(transform[:3, :3]).as_quat())))
        return pose

