import logging
import os
from dataclasses import dataclass
from typing_extensions import Optional

import numpy

try:
    from multiverse_parser import (
        InertiaSource,
        UsdImporter,
        MjcfImporter,
        UrdfImporter,
        BodyBuilder,
        JointBuilder,
        JointType,
    )
    from pxr import UsdUrdf
except ImportError as e:
    logging.info(e)
    InertiaSource = None
    UsdImporter = None
    MjcfImporter = None
    UrdfImporter = None
    BodyBuilder = None
    JointBuilder = None
    JointType = None
    UsdUrdf = None

from ..world_description.connections import (
    RevoluteConnection,
    PrismaticConnection,
    FixedConnection,
)
from ..world_description.degree_of_freedom import DegreeOfFreedom
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types import spatial_types as cas
from ..spatial_types.derivatives import DerivativeMap
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
            self.prefix = os.path.basename(self.file_path).split(".")[0]

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
            factory = UsdImporter(
                file_path=self.file_path,
                fixed_base=fixed_base,
                root_name=root_name,
                with_physics=with_physics,
                with_visual=with_visual,
                with_collision=with_collision,
                inertia_source=inertia_source,
                default_rgba=default_rgba,
                add_xform_for_each_geom=add_xform_for_each_geom,
            )
        elif file_ext == ".urdf":
            factory = UrdfImporter(
                file_path=self.file_path,
                fixed_base=fixed_base,
                root_name=root_name,
                with_physics=with_physics,
                with_visual=with_visual,
                with_collision=with_collision,
                inertia_source=inertia_source,
                default_rgba=default_rgba,
            )
        elif file_ext == ".xml":
            if root_name is None:
                root_name = "world"
            factory = MjcfImporter(
                file_path=self.file_path,
                fixed_base=fixed_base,
                root_name=root_name,
                with_physics=with_physics,
                with_visual=with_visual,
                with_collision=with_collision,
                inertia_source=inertia_source,
                default_rgba=default_rgba,
            )
        else:
            raise NotImplementedError(
                f"Importing from {file_ext} is not supported yet."
            )

        factory.import_model()
        bodies = [
            self.parse_body(body_builder)
            for body_builder in factory.world_builder.body_builders
        ]
        world = World()

        with world.modify_world():
            world.add_kinematic_structure_entity(bodies[0])
            for body in bodies:
                world.add_kinematic_structure_entity(body)
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
            parent_body = world.get_kinematic_structure_entity_by_name(
                joint_builder.parent_prim.GetName()
            )
            child_body = world.get_kinematic_structure_entity_by_name(
                joint_builder.child_prim.GetName()
            )
            connection = self.parse_joint(joint_builder, parent_body, child_body, world)
            connections.append(connection)
        if (
            len(body_builder.joint_builders) == 0
            and not body_builder.xform.GetPrim().GetParent().IsPseudoRoot()
        ):
            parent_body = world.get_kinematic_structure_entity_by_name(
                body_builder.xform.GetPrim().GetParent().GetName()
            )
            child_body = world.get_kinematic_structure_entity_by_name(
                body_builder.xform.GetPrim().GetName()
            )
            transform = body_builder.xform.GetLocalTransformation()
            pos = transform.ExtractTranslation()
            quat = transform.ExtractRotationQuat()
            point_expr = cas.Point3(pos[0], pos[1], pos[2])
            quaternion_expr = cas.Quaternion(
                quat.GetImaginary()[0],
                quat.GetImaginary()[1],
                quat.GetImaginary()[2],
                quat.GetReal(),
            )
            origin = cas.TransformationMatrix.from_point_rotation_matrix(
                point=point_expr, rotation_matrix=quaternion_expr.to_rotation_matrix()
            )
            connection = FixedConnection(
                parent=parent_body,
                child=child_body,
                parent_T_connection_expression=origin,
            )
            connections.append(connection)

        return connections

    def parse_joint(
        self,
        joint_builder: JointBuilder,
        parent_body: Body,
        child_body: Body,
        world: World,
    ) -> Connection:
        joint_prim = joint_builder.joint.GetPrim()
        joint_name = joint_prim.GetName()
        joint_pos = joint_builder.pos
        joint_quat = joint_builder.quat
        point_expr = cas.Point3(joint_pos[0], joint_pos[1], joint_pos[2])
        quaternion_expr = cas.Quaternion(
            joint_quat.GetImaginary()[0],
            joint_quat.GetImaginary()[1],
            joint_quat.GetImaginary()[2],
            joint_quat.GetReal(),
        )
        origin = cas.TransformationMatrix.from_point_rotation_matrix(
            point=point_expr, rotation_matrix=quaternion_expr.to_rotation_matrix()
        )
        free_variable_name = joint_name
        offset = None
        multiplier = None
        if joint_prim.HasAPI(UsdUrdf.UrdfJointAPI):
            urdf_joint_api = UsdUrdf.UrdfJointAPI(joint_prim)
            if len(urdf_joint_api.GetJointRel().GetTargets()) > 0:
                free_variable_name = urdf_joint_api.GetJointRel().GetTargets()[0].name
                offset = urdf_joint_api.GetOffsetAttr().Get()
                multiplier = urdf_joint_api.GetMultiplierAttr().Get()
        if joint_builder.type == JointType.FREE:
            raise NotImplementedError("Free joints are not supported yet.")
        elif joint_builder.type == JointType.FIXED:
            return FixedConnection(
                parent=parent_body,
                child=child_body,
                parent_T_connection_expression=origin,
            )
        elif joint_builder.type in [
            JointType.REVOLUTE,
            JointType.CONTINUOUS,
            JointType.PRISMATIC,
        ]:
            axis = cas.Vector3(
                float(joint_builder.axis.to_array()[0]),
                float(joint_builder.axis.to_array()[1]),
                float(joint_builder.axis.to_array()[2]),
                reference_frame=parent_body,
            )
            try:
                dof = world.get_degree_of_freedom_by_name(free_variable_name)
            except KeyError:
                if joint_builder.type == JointType.CONTINUOUS:
                    lower_limits = DerivativeMap()
                    lower_limits.position = (
                        joint_builder.joint.GetLowerLimitAttr().Get()
                    )
                    upper_limits = DerivativeMap()
                    upper_limits.position = (
                        joint_builder.joint.GetUpperLimitAttr().Get()
                    )
                    dof = DegreeOfFreedom(
                        name=PrefixedName(joint_name),
                        lower_limits=lower_limits,
                        upper_limits=upper_limits,
                    )
                    world.add_degree_of_freedom(dof)
                else:
                    dof = DegreeOfFreedom(
                        name=PrefixedName(joint_name),
                    )
                    world.add_degree_of_freedom(dof)
            if joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
                connection = RevoluteConnection(
                    parent=parent_body,
                    child=child_body,
                    parent_T_connection_expression=origin,
                    multiplier=multiplier,
                    offset=offset,
                    axis=axis,
                    dof_name=dof.name,
                )
            else:
                connection = PrismaticConnection(
                    parent=parent_body,
                    child=child_body,
                    parent_T_connection_expression=origin,
                    multiplier=multiplier,
                    offset=offset,
                    axis=axis,
                    dof_name=dof.name,
                )
            return connection
        else:
            raise NotImplementedError(
                f"Joint type {joint_builder.type} is not supported yet."
            )

    def parse_body(self, body_builder: BodyBuilder) -> Body:
        """
        Parses a body from a BodyBuilder instance.
        :param body_builder: The BodyBuilder instance to parse.
        :return: A Body instance representing the parsed body.
        """
        name = PrefixedName(
            prefix=self.prefix, name=body_builder.xform.GetPrim().GetName()
        )
        return Body(name=name)
