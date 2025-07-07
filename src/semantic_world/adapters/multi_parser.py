import os
from dataclasses import dataclass
from typing import Optional

import numpy
from multiverse_parser import (InertiaSource,
                               UsdImporter, MjcfImporter, UrdfImporter,
                               BodyBuilder,
                               JointBuilder, JointType)
from pxr import UsdUrdf

from ..connections import RevoluteConnection, PrismaticConnection, FixedConnection
from ..spatial_types.derivatives import Derivatives
from ..prefixed_name import PrefixedName
from ..spatial_types import spatial_types as cas
from ..world import World, Body, Connection


@dataclass
class MultiParser:
    """
    Class to parse any scene description files to worlds.
    """

    file_path: str
    """
    The file path of the scene.
    """

    prefix: Optional[str] = None
    """
    The prefix for every name used in this world.
    """

    def __post_init__(self):
        if self.prefix is None:
            self.prefix = os.path.basename(self.file_path).split('.')[0]

    def parse(self) -> World:
        fixed_base = True
        root_name = None
        with_physics = True
        with_visual = True
        with_collision = True
        inertia_source = InertiaSource.FROM_SRC
        default_rgba = numpy.array([0.9, 0.9, 0.9, 1.0])

        file_ext = os.path.splitext(self.file_path)[1]
        if file_ext in [".usd", ".usda", ".usdc"]:
            add_xform_for_each_geom = True
            factory = UsdImporter(file_path=self.file_path,
                                  fixed_base=fixed_base,
                                  root_name=root_name,
                                  with_physics=with_physics,
                                  with_visual=with_visual,
                                  with_collision=with_collision,
                                  inertia_source=inertia_source,
                                  default_rgba=default_rgba,
                                  add_xform_for_each_geom=add_xform_for_each_geom)
        elif file_ext == ".urdf":
            factory = UrdfImporter(file_path=self.file_path,
                                   fixed_base=fixed_base,
                                   root_name=root_name,
                                   with_physics=with_physics,
                                   with_visual=with_visual,
                                   with_collision=with_collision,
                                   inertia_source=inertia_source,
                                   default_rgba=default_rgba)
        elif file_ext == ".xml":
            if root_name is None:
                root_name = "world"
            factory = MjcfImporter(file_path=self.file_path,
                                   fixed_base=fixed_base,
                                   root_name=root_name,
                                   with_physics=with_physics,
                                   with_visual=with_visual,
                                   with_collision=with_collision,
                                   inertia_source=inertia_source,
                                   default_rgba=default_rgba)
        else:
            raise NotImplementedError(f"Importing from {file_ext} is not supported yet.")

        factory.import_model()
        bodies = [self.parse_body(body_builder) for body_builder in factory.world_builder.body_builders]
        world = World()
        world.add_body(bodies[0])

        with world.modify_world():
            for body in bodies:
                world.add_body(body)
            joints = []
            for body_builder in factory.world_builder.body_builders:
                joints += self.parse_joints(body_builder=body_builder, world=world)
            for joint in joints:
                world.add_connection(joint)

        return world

    def parse_joints(self, body_builder: BodyBuilder, world: World) -> list[Connection]:
        """
        Parses joints from a BodyBuilder instance.
        :param body_builder: The BodyBuilder instance to parse.
        :param world: The World instance to add the connections to.
        :return: A list of Connection instances representing the parsed joints.
        """
        connections = []
        for joint_builder in body_builder.joint_builders:
            parent_body = world.get_body_by_name(joint_builder.parent_prim.GetName())
            child_body = world.get_body_by_name(joint_builder.child_prim.GetName())
            connection = self.parse_joint(joint_builder, parent_body, child_body, world)
            connections.append(connection)
        if len(body_builder.joint_builders) == 0 and not body_builder.xform.GetPrim().GetParent().IsPseudoRoot():
            parent_body = world.get_body_by_name(body_builder.xform.GetPrim().GetParent().GetName())
            child_body = world.get_body_by_name(body_builder.xform.GetPrim().GetName())
            transform = body_builder.xform.GetLocalTransformation()
            pos = transform.ExtractTranslation()
            quat = transform.ExtractRotationQuat()
            origin = cas.TransformationMatrix.from_xyz_quat(pos_x=pos[0],
                                                            pos_y=pos[1],
                                                            pos_z=pos[2],
                                                            quat_w=quat.GetReal(),
                                                            quat_x=quat.GetImaginary()[0],
                                                            quat_y=quat.GetImaginary()[1],
                                                            quat_z=quat.GetImaginary()[2])
            connection = FixedConnection(parent=parent_body, child=child_body, origin=origin)
            connections.append(connection)

        return connections

    def parse_joint(self, joint_builder: JointBuilder, parent_body: Body, child_body: Body, world: World) -> Connection:
        joint_prim = joint_builder.joint.GetPrim()
        joint_name = joint_prim.GetName()
        joint_pos = joint_builder.pos
        joint_quat = joint_builder.quat
        origin = cas.TransformationMatrix.from_xyz_quat(pos_x=joint_pos[0],
                                                        pos_y=joint_pos[1],
                                                        pos_z=joint_pos[2],
                                                        quat_w=joint_quat.GetReal(),
                                                        quat_x=joint_quat.GetImaginary()[0],
                                                        quat_y=joint_quat.GetImaginary()[1],
                                                        quat_z=joint_quat.GetImaginary()[2])
        free_variable_name = PrefixedName(joint_name)
        offset = None
        multiplier = None
        if joint_prim.HasAPI(UsdUrdf.UrdfJointAPI):
            urdf_joint_api = UsdUrdf.UrdfJointAPI(joint_prim)
            if len(urdf_joint_api.GetJointRel().GetTargets()) > 0:
                free_variable_name = PrefixedName(urdf_joint_api.GetJointRel().GetTargets()[0].name)
                offset = urdf_joint_api.GetOffsetAttr().Get()
                multiplier = urdf_joint_api.GetMultiplierAttr().Get()
        if joint_builder.type == JointType.FREE:
            raise NotImplementedError("Free joints are not supported yet.")
        elif joint_builder.type == JointType.FIXED:
            return FixedConnection(parent=parent_body, child=child_body, origin=origin)
        elif joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC]:
            axis = (float(joint_builder.axis.to_array()[0]),
                    float(joint_builder.axis.to_array()[1]),
                    float(joint_builder.axis.to_array()[2]))
            if free_variable_name in world.degrees_of_freedom:
                dof = world.degrees_of_freedom[free_variable_name]
            else:
                if joint_builder.type == JointType.CONTINUOUS:
                    dof = world.create_degree_of_freedom(name=PrefixedName(joint_name),
                                                         lower_limits={Derivatives.position: joint_builder.joint.GetLowerLimitAttr().Get()},
                                                         upper_limits={Derivatives.position: joint_builder.joint.GetUpperLimitAttr().Get()})
                else:
                    dof = world.create_degree_of_freedom(name=PrefixedName(joint_name))
            if joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
                connection = RevoluteConnection(parent=parent_body, child=child_body, origin=origin,
                                                multiplier=multiplier, offset=offset,
                                                axis=axis, dof=dof)
            else:
                connection = PrismaticConnection(parent=parent_body, child=child_body, origin=origin,
                                                 multiplier=multiplier, offset=offset,
                                                 axis=axis, dof=dof)
            return connection
        else:
            raise NotImplementedError(f"Joint type {joint_builder.type} is not supported yet.")

    def parse_body(self, body_builder: BodyBuilder) -> Body:
        """
        Parses a body from a BodyBuilder instance.
        :param body_builder: The BodyBuilder instance to parse.
        :return: A Body instance representing the parsed body.
        """
        name = PrefixedName(prefix=self.prefix, name=body_builder.xform.GetPrim().GetName())
        return Body(name=name)
