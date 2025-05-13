import os
from dataclasses import dataclass
from typing import Optional, Tuple, Dict
import semantic_world.spatial_types.spatial_types as cas
from urdf_parser_py import urdf

from ..connections import RevoluteConnection, PrismaticConnection, FixedConnection
from ..prefixed_name import PrefixedName
from ..spatial_types.derivatives import Derivatives
from ..utils import suppress_stdout_stderr, hacky_urdf_parser_fix
from ..world import World, Body, Connection

connection_type_map = {  # 'unknown': JointType.UNKNOWN,
    'revolute': RevoluteConnection,
    'continuous': RevoluteConnection,
    'prismatic': PrismaticConnection,
    # 'floating': JointType.FLOATING,
    # 'planar': JointType.PLANAR,
    'fixed': FixedConnection}


def urdf_joint_to_limits(urdf_joint: urdf.Joint) -> Tuple[Dict[Derivatives, float], Dict[Derivatives, float]]:
    lower_limits = {}
    upper_limits = {}
    if not urdf_joint.type == 'continuous':
        try:
            lower_limits[Derivatives.position] = max(urdf_joint.safety_controller.soft_lower_limit,
                                                     urdf_joint.limit.lower)
            upper_limits[Derivatives.position] = min(urdf_joint.safety_controller.soft_upper_limit,
                                                     urdf_joint.limit.upper)
        except AttributeError:
            try:
                lower_limits[Derivatives.position] = urdf_joint.limit.lower
                upper_limits[Derivatives.position] = urdf_joint.limit.upper
            except AttributeError:
                pass
    try:
        lower_limits[Derivatives.velocity] = -urdf_joint.limit.velocity
        upper_limits[Derivatives.velocity] = urdf_joint.limit.velocity
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
            lower_limits[d2] -= offset
            upper_limits[d2] -= offset
            if multiplier < 0:
                upper_limits[d2], lower_limits[d2] = lower_limits[d2], upper_limits[d2]
            upper_limits[d2] /= multiplier
            lower_limits[d2] /= multiplier
    return lower_limits, upper_limits


@dataclass
class URDFParser:
    """
    Class to parse URDF files to worlds.
    """

    file_path: str
    """
    The file path of the URDF.
    """

    prefix: Optional[str] = None
    """
    The prefix for every name used in this world.
    """

    def __post_init__(self):
        if self.prefix is None:
            self.prefix = os.path.basename(self.file_path).split('.')[0]

    def parse(self) -> World:
        # cache_dir = os.path.join(os.getcwd(), '..', '..', '../resources', 'cache')
        # file_name = os.path.basename(self.file_path)
        # new_file_path = os.path.join(cache_dir, file_name)
        # generate_from_description_file(self.file_path, new_file_path)

        with open(self.file_path, 'r') as file:
            # Since parsing URDF causes a lot of warning messages which can't be deactivated, we suppress them
            with suppress_stdout_stderr():
                parsed = urdf.URDF.from_xml_string(hacky_urdf_parser_fix(file.read()))

        links = [self.parse_link(link) for link in parsed.links]
        root = [link for link in links if link.name.name == parsed.get_root()][0]
        world = World(root=root)

        joints = []
        for joint in parsed.joints:
            parent = [link for link in links if link.name.name == joint.parent][0]
            child = [link for link in links if link.name.name == joint.child][0]
            parsed_joint = self.parse_joint(joint, parent, child, world)
            joints.append(parsed_joint)

        [world.add_connection(joint) for joint in joints]
        [world.add_body(link) for link in links]

        return world

    def parse_joint(self, joint: urdf.Joint, parent: Body, child: Body, world: World) -> Connection:
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
        parent_T_child = cas.TransformationMatrix.from_xyz_rpy(x=translation_offset[0],
                                                               y=translation_offset[1],
                                                               z=translation_offset[2],
                                                               roll=rotation_offset[0],
                                                               pitch=rotation_offset[1],
                                                               yaw=rotation_offset[2])
        if connection_type == FixedConnection:
            return connection_type(parent=parent, child=child, origin=parent_T_child)

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

            free_variable_name = PrefixedName(joint.mimic.joint)
        else:
            free_variable_name = PrefixedName(joint.name)

        if free_variable_name in world.free_variables:
            free_variable = world.free_variables[free_variable_name]
        else:
            free_variable = world.create_free_variable(name=PrefixedName(joint.name),
                                                       lower_limits=lower_limits, upper_limits=upper_limits)

        result = connection_type(parent=parent, child=child, origin=parent_T_child,
                                 multiplier=multiplier, offset=offset,
                                 free_variable=free_variable)
        return result

    def parse_link(self, link: urdf.Link) -> Body:
        """
        Parses a URDF link to a link object.
        :param link: The URDF link to parse.
        :return: The parsed link object.
        """
        name = PrefixedName(prefix=self.prefix, name=link.name)
        return Body(name=name)
