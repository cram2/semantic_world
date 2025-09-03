from __future__ import annotations

import os
from dataclasses import dataclass
from typing_extensions import Optional, Tuple, Union, List, Dict

from urdf_parser_py import urdf as urdfpy

from ..world_description import (RevoluteConnection, PrismaticConnection, FixedConnection, DegreeOfFreedom)
from ..exceptions import ParsingError
from ..world_description.geometry import Box, Sphere, Cylinder, Mesh, Scale, Shape, Color
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types import spatial_types as cas
from ..spatial_types.derivatives import Derivatives, DerivativeMap
from ..spatial_types.spatial_types import TransformationMatrix, Vector3
from ..utils import (
    suppress_stdout_stderr,
    hacky_urdf_parser_fix,
    robot_name_from_urdf_string,
)
from ..world import World, Body, Connection

connection_type_map = {  # 'unknown': JointType.UNKNOWN,
    "revolute": RevoluteConnection,
    "continuous": RevoluteConnection,
    "prismatic": PrismaticConnection,
    # 'floating': JointType.FLOATING,
    # 'planar': JointType.PLANAR,
    "fixed": FixedConnection,
}


def urdf_joint_to_limits(
    urdf_joint: urdfpy.Joint,
) -> Tuple[DerivativeMap[float], DerivativeMap[float]]:
    """
    Maps the URDF joint specifications to lower and upper joint limits, including
    position and velocity constraints. Mimics and safety controller parameters are
    also considered when determining the limits.

    :param urdf_joint: A URDF (Unified Robot Description Format) joint object
                       which contains the joint's type, limits, safety controller,
                       and mimic information.
    :return: A tuple containing two DerivativeMap objects, representing the lower
             and upper limits of the joint in terms of position and velocity.
    """
    lower_limits = DerivativeMap()
    upper_limits = DerivativeMap()
    if not urdf_joint.type == "continuous":
        try:
            lower_limits.position = max(
                urdf_joint.safety_controller.soft_lower_limit, urdf_joint.limit.lower
            )
            upper_limits.position = min(
                urdf_joint.safety_controller.soft_upper_limit, urdf_joint.limit.upper
            )
        except AttributeError:
            try:
                lower_limits.position = urdf_joint.limit.lower
                upper_limits.position = urdf_joint.limit.upper
            except AttributeError:
                pass
    try:
        lower_limits.velocity = -urdf_joint.limit.velocity
        upper_limits.velocity = urdf_joint.limit.velocity
    except AttributeError:
        pass
    if urdf_joint.mimic is not None:
        if urdf_joint.mimic.multiplier is not None:
            multiplier = urdf_joint.mimic.multiplier
        else:
            multiplier = 1
        if urdf_joint.mimic.offset is not None:
            offset = urdf_joint.mimic.offset
        else:
            offset = 0
        for d2 in Derivatives.range(Derivatives.position, Derivatives.velocity):
            lower_limits.data[d2] -= offset
            upper_limits.data[d2] -= offset
            if multiplier < 0:
                upper_limits.data[d2], lower_limits.data[d2] = (
                    lower_limits.data[d2],
                    upper_limits.data[d2],
                )
            upper_limits.data[d2] /= multiplier
            lower_limits.data[d2] /= multiplier
    return lower_limits, upper_limits


@dataclass
class URDFParser:
    """
    Class to parse URDF files to worlds.
    Must set either urdf or file_path.
    """

    urdf: str
    """
    The URDF string.
    """

    prefix: Optional[str] = None
    """
    The prefix for every name used in this world.
    """

    package_resolver: Optional[Dict[str, str]] = None
    """
    The package resolver to use for resolving package paths in the URDF file. If ROS is installed, ROS will be used
     to resolve the paths, otherwise the package_resolver must be provided if the URDF file contains package paths.
     The key is the package name and the value is the path to the package. You can also set the environment variable
     `ROS_PACKAGE_PATH` to a colon-separated list of package paths, which will be used as the package resolver.
    """

    def __post_init__(self):
        self.urdf = hacky_urdf_parser_fix(self.urdf)
        self.parsed = urdfpy.URDF.from_xml_string(self.urdf)
        if self.prefix is None:
            self.prefix = robot_name_from_urdf_string(self.urdf)
        if self.package_resolver is None:
            package_paths = os.environ.get("ROS_PACKAGE_PATH", "").split(":")
            self.package_resolver = {
                os.path.basename(path): path
                for path in package_paths
                if os.path.exists(path)
            }

    @classmethod
    def from_file(cls, file_path: str, prefix: Optional[str] = None) -> URDFParser:
        if file_path is not None:
            with open(file_path, "r") as file:
                # Since parsing URDF causes a lot of warning messages which can't be deactivated, we suppress them
                with suppress_stdout_stderr():
                    urdf = file.read()
        return URDFParser(urdf=urdf, prefix=prefix)

    def parse(self) -> World:
        prefix = self.parsed.name
        links = [
            self.parse_link(link, PrefixedName(link.name, prefix))
            for link in self.parsed.links
        ]
        root = [link for link in links if link.name.name == self.parsed.get_root()][0]
        world = World()
        world.name = self.prefix
        with world.modify_world():
            world.add_kinematic_structure_entity(root)
            joints = []
            for joint in self.parsed.joints:
                parent = [link for link in links if link.name.name == joint.parent][0]
                child = [link for link in links if link.name.name == joint.child][0]
                parsed_joint = self.parse_joint(joint, parent, child, world, prefix)
                joints.append(parsed_joint)

            [world.add_connection(joint) for joint in joints]
            [world.add_kinematic_structure_entity(link) for link in links]

        return world

    def parse_joint(
        self, joint: urdfpy.Joint, parent: Body, child: Body, world: World, prefix: str
    ) -> Connection:
        """
        Parses a given URDF joint and creates a corresponding connection object.

        The function processes the provided joint data, extracting necessary
        information including translation offsets, rotation offsets, connection type,
        and relevant joint limits. It maps URDF joint types to predefined connection
        types and either retrieves or creates a degree of freedom (DOF) in the world
        context. It generates and returns a connection object representing the
        relationship between a parent and a child body.

        :param joint: The URDF joint to be parsed.
        :param parent: The parent body to be connected by the joint.
        :param child: The child body to be connected by the joint.
        :param world: The world instance containing degrees of freedom.
        :param prefix: The prefix for naming connections and DOFs.
        :return: A connection object representing the parsed joint.
        """
        connection_name = PrefixedName(joint.name, prefix)
        connection_type = connection_type_map.get(joint.type, Connection)
        if joint.origin is not None:
            translation_offset = joint.origin.xyz
            rotation_offset = joint.origin.rpy
        else:
            translation_offset = None
            rotation_offset = None
        if translation_offset is None:
            translation_offset = [0, 0, 0]
        if rotation_offset is None:
            rotation_offset = [0, 0, 0]
        parent_T_child = cas.TransformationMatrix.from_xyz_rpy(
            x=translation_offset[0],
            y=translation_offset[1],
            z=translation_offset[2],
            roll=rotation_offset[0],
            pitch=rotation_offset[1],
            yaw=rotation_offset[2],
        )
        if connection_type == FixedConnection:
            return connection_type(
                name=connection_name,
                parent=parent,
                child=child,
                origin_expression=parent_T_child,
            )

        lower_limits, upper_limits = urdf_joint_to_limits(joint)
        is_mimic = joint.mimic is not None
        multiplier = None
        offset = None
        if is_mimic:
            if joint.mimic.multiplier is not None:
                multiplier = joint.mimic.multiplier
            else:
                multiplier = 1
            if joint.mimic.offset is not None:
                offset = joint.mimic.offset
            else:
                offset = 0

            dof_name = PrefixedName(joint.mimic.joint, prefix)
        else:
            dof_name = connection_name

        try:
            dof = world.get_degree_of_freedom_by_name(dof_name)
        except KeyError as e:
            dof = DegreeOfFreedom(
                name=dof_name,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
            )
            world.add_degree_of_freedom(dof)

        result = connection_type(
            name=connection_name,
            parent=parent,
            child=child,
            origin_expression=parent_T_child,
            multiplier=multiplier,
            offset=offset,
            axis=Vector3(*map(int, joint.axis), reference_frame=parent),
            dof=dof,
        )
        return result

    def parse_link(self, link: urdfpy.Link, parent_frame: PrefixedName) -> Body:
        """
        Parses a URDF link to a link object.
        :param link: The URDF link to parse.
        :param parent_frame: The parent frame of the link, used for transformations of collisions and visuals.
        :return: The parsed link object.
        """
        name = PrefixedName(prefix=self.prefix, name=link.name)
        visuals = self.parse_geometry(link.visuals, parent_frame)
        collisions = self.parse_geometry(link.collisions, parent_frame)
        return Body(name=name, visual=visuals, collision=collisions)

    def parse_geometry(
        self,
        geometry: Union[List[urdfpy.Collision], List[urdfpy.Visual]],
        parent_frame: PrefixedName,
    ) -> List[Shape]:
        """
        Parses a URDF geometry to the corresponding shapes.
        :param geometry: The URDF geometry to parse either the collisions of visuals.'
        :param parent_frame: The parent frame of the geometry, used for transformations.
        :return: A List of shapes corresponding to the URDF geometry.
        """
        res = []
        material_dict = dict(
            zip(
                [material.name for material in self.parsed.materials],
                [
                    material.color.rgba if material.color else None
                    for material in self.parsed.materials
                ],
            )
        )
        for i, geom in enumerate(geometry):
            params = (
                (*(geom.origin.xyz + geom.origin.rpy),)
                if geom.origin
                else (
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            )
            origin_transform = TransformationMatrix.from_xyz_rpy(*params)
            if isinstance(geom.geometry, urdfpy.Box):
                color = (
                    Color(*material_dict.get(geom.material.name, (1, 1, 1, 1)))
                    if hasattr(geom, "material") and geom.material
                    else Color(1, 1, 1, 1)
                )
                res.append(
                    Box(
                        origin=origin_transform,
                        scale=Scale(*geom.geometry.size),
                        color=color,
                    )
                )
            elif isinstance(geom.geometry, urdfpy.Sphere):
                color = (
                    Color(*material_dict.get(geom.material.name, (1, 1, 1, 1)))
                    if hasattr(geom, "material") and geom.material
                    else Color(1, 1, 1, 1)
                )
                res.append(
                    Sphere(
                        origin=origin_transform,
                        radius=geom.geometry.radius,
                        color=color,
                    )
                )
            elif isinstance(geom.geometry, urdfpy.Cylinder):
                color = (
                    Color(*material_dict.get(geom.material.name, (1, 1, 1, 1)))
                    if hasattr(geom, "material") and geom.material
                    else Color(1, 1, 1, 1)
                )
                res.append(
                    Cylinder(
                        origin=origin_transform,
                        width=geom.geometry.radius,
                        height=geom.geometry.length,
                        color=color,
                    )
                )
            elif isinstance(geom.geometry, urdfpy.Mesh):
                if geom.geometry.filename is None:
                    raise ValueError("Mesh geometry must have a filename.")
                res.append(
                    Mesh(
                        origin=origin_transform,
                        filename=self.parse_file_path(geom.geometry.filename),
                        scale=Scale(*(geom.geometry.scale or (1, 1, 1))),
                    )
                )
        return res

    def parse_file_path(self, file_path: str) -> str:
        """
        Parses a file path which contains a ros package to a path in the local file system.

        :param file_path: The path to the URDF file.
        :return: The parsed and processed file path.
        """
        if "package://" in file_path:
            # Splits the file path at '//' to get the package  and the rest of the path
            package_split = file_path.split("//")
            # Splits the path after the // to get the package name and the rest of the path
            package_name = package_split[1].split("/")[0]
            try:
                from ament_index_python.packages import get_package_share_directory

                package_path = get_package_share_directory(package_name)
            except ImportError:
                if self.package_resolver:
                    if package_name in self.package_resolver:
                        package_path = self.package_resolver[package_name]
                    else:
                        raise ParsingError(
                            msg=f"Package '{package_name}' not found in package resolver and "
                            f"ROS is not installed."
                        )
                else:
                    raise ParsingError(
                        msg="No ROS install found while the URDF file contains references to "
                        "ROS packages."
                    )
            file_path = file_path.replace("package://" + package_name, package_path)
        if "file://" in file_path:
            file_path = file_path.replace("file://", "./")
        return file_path
