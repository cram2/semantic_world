import os
from dataclasses import dataclass
from typing import Optional

from urdf_parser_py import urdf

from ..connections import RevoluteConnection, PrismaticConnection, FixedConnection
from ..prefixed_name import PrefixedName
from ..utils import suppress_stdout_stderr
from ..world import World, Body, Connection

connection_type_map = {  # 'unknown': JointType.UNKNOWN,
    'revolute': RevoluteConnection,
    # 'continuous': JointType.CONTINUOUS,
    'prismatic': PrismaticConnection,
    # 'floating': JointType.FLOATING,
    # 'planar': JointType.PLANAR,
    'fixed': FixedConnection}


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
                parsed = urdf.URDF.from_xml_string(file.read())

        links = [self.parse_link(link) for link in parsed.links]
        joints = []
        for joint in parsed.joints:
            parent = [link for link in links if link.name.name == joint.parent][0]
            child = [link for link in links if link.name.name == joint.child][0]
            parsed_joint = self.parse_joint(joint, parent, child)
            joints.append(parsed_joint)

        world = World(root=links[0])
        [world.add_connection(joint) for joint in joints]
        [world.add_body(link) for link in links]

        return world

    def parse_joint(self, joint: urdf.Joint, parent: Body, child: Body) -> Connection:
        connection_type = connection_type_map.get(joint.type, Connection)
        result = connection_type(parent=parent, child=child)
        return result

    def parse_link(self, link: urdf.Link) -> Body:
        """
        Parses a URDF link to a link object.
        :param link: The URDF link to parse.
        :return: The parsed link object.
        """
        name = PrefixedName(prefix=self.prefix, name=link.name)
        return Body(name=name)
