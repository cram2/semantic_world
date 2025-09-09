import logging
import os
from dataclasses import dataclass

from typing_extensions import Optional, Set, List

import numpy
from multiverse_parser import (Factory,
                               InertiaSource,
                               UsdImporter, MjcfImporter, UrdfImporter,
                               BodyBuilder,
                               JointBuilder, JointType, GeomType)
from multiverse_parser.utils import get_relative_transform
from pxr import UsdUrdf, UsdGeom, UsdPhysics, Gf  # type: ignore

from ..world_description.geometry import Box, Sphere, Cylinder, Scale, Shape, Color, TriangleMesh
from ..spatial_types.spatial_types import TransformationMatrix

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

from ..world_description.connections import RevoluteConnection, PrismaticConnection, FixedConnection, Connection6DoF
from ..world_description.degree_of_freedom import DegreeOfFreedom
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types import spatial_types as cas
from ..spatial_types.derivatives import DerivativeMap
from ..world import World, Body, Connection


def get_free_body_names(factory: Factory) -> Set[str]:
    """
    Get the names of all free bodies in the world.
    :param factory: The factory instance that contains the world builder.
    :return: A set of names of free bodies.
    """
    constrained_body_names = set()
    stage = factory.world_builder.stage
    for joint in [UsdPhysics.Joint(joint_prim) for joint_prim in stage.TraverseAll() if  # type: ignore
                  joint_prim.IsA(UsdPhysics.Joint)]:  # type: ignore
        for child_body_path in joint.GetBody1Rel().GetTargets():
            child_body_prim = stage.GetPrimAtPath(child_body_path)
            constrained_body_names.add(child_body_prim.GetName())
            for each_child_body_prim in child_body_prim.GetAllChildren():
                if each_child_body_prim.IsA(UsdGeom.Xform):  # type: ignore
                    constrained_body_names.add(each_child_body_prim.GetName())

    free_body_names = set()
    for xform_prim in [prim for prim in stage.TraverseAll() if
                       prim.IsA(UsdGeom.Xform) and  # type: ignore
                       prim.HasAPI(UsdPhysics.MassAPI) and  # type: ignore
                       prim.HasAPI(UsdPhysics.RigidBodyAPI)]:  # type: ignore
        if xform_prim.GetName() in constrained_body_names:
            continue
        free_body_names.add(xform_prim.GetName())

    return free_body_names


def parse_geometry(body_builder: BodyBuilder) -> tuple[List[Shape], List[Shape]]:
    """
    Parses the visual and collision geometry from a BodyBuilder instance.

    :param body_builder: The BodyBuilder instance to parse.
    :return: A tuple containing two lists: the first list contains the visual shapes, and the second list contains the collision shapes.
    """

    visuals = []
    collisions = []
    for geom_builder in body_builder.geom_builders:
        gprim = geom_builder.gprim
        gprim_prim = gprim.GetPrim()
        local_transformation: Gf.Matrix4d = gprim.GetLocalTransformation().RemoveScaleShear()
        translation: Gf.Vec3d = local_transformation.ExtractTranslation()
        rotation: Gf.Rotation = local_transformation.ExtractRotation()
        quat: Gf.Quatd = rotation.GetQuat()
        origin_transform = TransformationMatrix.from_xyz_quat(pos_x=translation[0],
                                                              pos_y=translation[1],
                                                              pos_z=translation[2],
                                                                quat_x=quat.GetImaginary()[0],
                                                                quat_y=quat.GetImaginary()[1],
                                                                quat_z=quat.GetImaginary()[2],
                                                                quat_w=quat.GetReal())
        color = (Color(*geom_builder.rgba))
        if geom_builder.type == GeomType.CUBE:  # type: ignore
            size = numpy.array([gprim.GetLocalTransformation().GetRow(i).GetLength() for i in range(3)]) * 2
            shape = Box(
                origin=origin_transform,
                scale=Scale(*size),
                color=color,
            )
        elif geom_builder.type == GeomType.SPHERE:
            sphere = UsdGeom.Sphere(gprim_prim)
            shape = Sphere(
                origin=origin_transform,
                radius=sphere.GetRadiusAttr().Get(),
                color=color,
            )
        elif geom_builder.type == GeomType.CYLINDER:
            cylinder = UsdGeom.Cylinder(gprim_prim)
            shape = Cylinder(
                origin=origin_transform,
                width=cylinder.GetRadiusAttr().Get()*2,
                height=cylinder.GetHeightAttr().Get(),
                color=color,
            )
        elif geom_builder.type == GeomType.MESH:
            scale = [local_transformation.GetRow(i).GetLength() for i in range(3)]
            data = {"mesh": {}, "origin": {}, "scale": {}}
            data["mesh"]["vertices"] = gprim.GetVertices()
            data["mesh"]["faces"] = gprim.GetFaceVertexCounts()
            data["origin"]["position"] = [translation[0], translation[1], translation[2]]
            data["origin"]["quaternion"] =[quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]]
            data["scale"]["x"] = scale[0]
            data["scale"]["y"] = scale[1]
            data["scale"]["z"] = scale[2]
            TriangleMesh.from_json(data=data)
            shape = TriangleMesh.from_json(data=data)
        else:
            logging.warning(f"Geometry type {geom_builder.type} is not supported yet.")
            continue
        if gprim_prim.HasAPI(UsdPhysics.CollisionAPI):
            collisions.append(shape)
        else:
            visuals.append(shape)
    return visuals, collisions


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

    def parse(self, fixed_base=True) -> World:
        root_name = None
        with_physics = True
        with_visual = True
        with_collision = True
        inertia_source = InertiaSource.FROM_SRC
        default_rgba = numpy.array([0.9, 0.9, 0.9, 1.0])

        file_ext = os.path.splitext(self.file_path)[1]
        if file_ext in [".usd", ".usda", ".usdc"]:
            factory = UsdImporter(file_path=self.file_path,
                                  fixed_base=True,
                                  root_name=root_name,
                                  with_physics=with_physics,
                                  with_visual=with_visual,
                                  with_collision=with_collision,
                                  inertia_source=inertia_source,
                                  default_rgba=default_rgba)
        elif file_ext == ".urdf":
            factory = UrdfImporter(
                file_path=self.file_path,
                fixed_base=False,
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
                fixed_base=False,
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

        root = bodies[0]
        if root.name.name != "world":
            with world.modify_world():
                root = Body(name=PrefixedName("world"))
                world.add_body(root)

        with world.modify_world():
            for body in bodies:
                world.add_kinematic_structure_entity(body)
            joints = []
            for body_builder in factory.world_builder.body_builders:
                joints += self.parse_joints(body_builder=body_builder, world=world)
            for joint in joints:
                world.add_connection(joint)

            free_body_names = get_free_body_names(factory=factory)

            for free_body_name in free_body_names:
                body = world.get_body_by_name(free_body_name)
                if body.name.name == root.name.name:
                    continue
                if fixed_base:
                    joint = FixedConnection(parent=root, child=body)
                else:
                    joint = Connection6DoF(parent=root, child=body, _world=world)
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
            transform = get_relative_transform(from_prim=joint_builder.parent_prim,
                                               to_prim=joint_builder.child_prim)
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
            connection = self.parse_joint(joint_builder, parent_body, child_body, origin, world)
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
                parent=parent_body, child=child_body, origin_expression=origin
            )
            connections.append(connection)

        return connections

    def parse_joint(
        self,
        joint_builder: JointBuilder,
        parent_body: Body,
        child_body: Body,
        origin: TransformationMatrix,
        world: World,
    ) -> Connection:
        joint_prim = joint_builder.joint.GetPrim()
        joint_name = joint_prim.GetName()
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
                parent=parent_body, child=child_body, origin_expression=origin
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
                    dof = DegreeOfFreedom(
                        name=PrefixedName(joint_name),
                    )
                    world.add_degree_of_freedom(dof)
                else:
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

            if joint_builder.type in [JointType.REVOLUTE, JointType.CONTINUOUS]:
                connection = RevoluteConnection(
                    parent=parent_body,
                    child=child_body,
                    origin_expression=origin,
                    multiplier=multiplier,
                    offset=offset,
                    axis=axis,
                    dof=dof,
                )
            else:
                connection = PrismaticConnection(
                    parent=parent_body,
                    child=child_body,
                    origin_expression=origin,
                    multiplier=multiplier,
                    offset=offset,
                    axis=axis,
                    dof=dof,
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
        visuals, collisions = parse_geometry(body_builder)
        return Body(name=name, visual=visuals, collision=collisions)
