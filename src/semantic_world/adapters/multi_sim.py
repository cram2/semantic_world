import inspect
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from types import NoneType
from typing import (
    Dict,
    List,
    Any,
    ClassVar,
    Type,
    Optional,
    Union,
    Tuple,
)

import numpy
from mujoco_connector import MultiverseMujocoConnector
import mujoco
from multiverse_simulator import (
    MultiverseSimulator,
    MultiverseSimulatorState,
    MultiverseViewer,
    MultiverseAttribute,
    MultiverseCallbackResult,
)
from random_events.utils import recursive_subclasses
from scipy.spatial.transform import Rotation

from ..callbacks.callback import ModelChangeCallback
from ..datastructures.prefixed_name import PrefixedName
from ..spatial_types.spatial_types import TransformationMatrix
from ..world import World
from ..world_description.connections import (
    RevoluteConnection,
    PrismaticConnection,
    ActiveConnection1DOF,
    FixedConnection,
)
from ..world_description.geometry import Box, Cylinder, Sphere, Shape
from ..world_description.world_entity import (
    Region,
    Body,
    KinematicStructureEntity,
    Connection,
    WorldEntity,
)
from ..world_description.world_modification import (
    AddKinematicStructureEntityModification,
)
from ..spatial_types.symbol_manager import symbol_manager


def cas_pose_to_list(pose: TransformationMatrix) -> List[float]:
    """
    Converts a CAS TransformationMatrix to a list of 7 floats (position + quaternion).

    :param pose: The CAS TransformationMatrix to convert.
    :return: A list of 7 floats ([px, py, pz, qw, qx, qy, qz]) representing the position and quaternion.
    """
    pos = pose.to_position()
    quat = pose.to_quaternion()
    px, py, pz, _ = symbol_manager.evaluate_expr(pos).tolist()
    qx, qy, qz, qw = symbol_manager.evaluate_expr(quat).tolist()
    return [px, py, pz, qw, qx, qy, qz]


class EntityConverter(ABC):
    """
    A converter to convert an entity object (WorldEntity, Shape) to a dictionary of properties for Multiverse simulator.
    """

    entity_type: ClassVar[Type[Any]] = Any
    name_str: str = "name"

    @classmethod
    def convert(cls, entity: entity_type, **kwargs) -> Dict[str, Any]:  # type: ignore
        """
        Converts an entity object to a dictionary of properties for Multiverse simulator.

        :param entity: The object to convert.
        :return: A dictionary of properties.
        """
        for subclass in recursive_subclasses(cls):
            if (
                not inspect.isabstract(subclass)
                and not inspect.isabstract(subclass.entity_type)
                and type(entity) is subclass.entity_type
            ):
                entity_props = subclass()._convert(entity, **kwargs)
                return subclass()._post_convert(entity, entity_props, **kwargs)
        raise NotImplementedError(f"No converter found for entity type {type(entity)}.")

    def _convert(self, entity: entity_type, **kwargs) -> Dict[str, Any]:  # type: ignore
        """
        The actual conversion method to be implemented by subclasses.

        :param entity: The object to convert.
        :return: A dictionary of properties, by default containing the name.
        """
        return {
            self.name_str: (
                entity.name.name
                if hasattr(entity, "name") and isinstance(entity.name, PrefixedName)
                else f"{type(entity).__name__.lower()}_{id(entity)}"
            )
        }

    @abstractmethod
    def _post_convert(
        self, entity: entity_type, entity_props: Dict[str, Any], **kwargs  # type: ignore
    ) -> Dict[str, Any]:
        """
        Post-processes the converted entity properties. This method can be overridden by subclasses to update the properties after conversion.

        :param entity: The object that was converted.
        :param entity_props: The dictionary of properties that was converted.
        :return: The updated dictionary of properties.
        """
        raise NotImplementedError


class KinematicStructureEntityConverter(EntityConverter, ABC):
    entity_type: ClassVar[Type[KinematicStructureEntity]] = KinematicStructureEntity
    pos_str: str
    quat_str: str

    def _convert(self, entity: entity_type, **kwargs) -> Dict[str, Any]:
        """
        Converts a KinematicStructureEntity object to a dictionary of body properties for Multiverse.

        :param entity: The KinematicStructureEntity object to convert.
        :return: A dictionary of body properties, by default containing position and quaternion.
        """

        kinematic_structure_entity_props = EntityConverter._convert(self, entity)
        px, py, pz, qw, qx, qy, qz = cas_pose_to_list(
            entity.parent_connection.origin_expression
        )
        kinematic_structure_entity_pos = [px, py, pz]
        kinematic_structure_entity_quat = [qw, qx, qy, qz]
        kinematic_structure_entity_props.update(
            {
                self.pos_str: kinematic_structure_entity_pos,
                self.quat_str: kinematic_structure_entity_quat,
            }
        )
        return kinematic_structure_entity_props


class BodyConverter(KinematicStructureEntityConverter, ABC):
    entity_type: ClassVar[Type[WorldEntity]] = Body
    mass_str: str
    inertia_pos_str: str
    inertia_quat_str: str
    diagonal_inertia_str: str

    def _convert(self, entity: Body, **kwargs) -> Dict[str, Any]:
        body_props = KinematicStructureEntityConverter._convert(self, entity)
        mass, inertia_pos, inertia_quat, diagonal_inertia = (
            self._compute_inertial(  # TODO: get values from entity
                mass=1e-3,
                inertia_pos=[0.0, 0.0, 0.0],
                inertia_quat=[1.0, 0.0, 0.0, 0.0],
                diagonal_inertia=[1.5e-8, 1.5e-8, 1.5e-8],  # Either this
                inertia=None,  # Or this
                inertia_matrix=None,  # Or this
            )
        )
        body_props[self.mass_str] = mass
        body_props[self.inertia_pos_str] = inertia_pos
        body_props[self.inertia_quat_str] = inertia_quat
        body_props[self.diagonal_inertia_str] = diagonal_inertia
        return body_props

    @staticmethod
    def _compute_inertial(
        mass: float,
        inertia_pos: List[float],
        inertia_quat: List[float],
        diagonal_inertia: Optional[List[float]] = None,
        inertia: Optional[List[float]] = None,
        inertia_matrix: Optional[List[float]] = None,
    ) -> Tuple[float, List[float], List[float], List[float]]:
        """
        Computes the inertial properties of a body.
        If `diagonal_inertia` is given, it is used directly.
        If `inertia` or `inertia_matrix` is given, they are converted to diagonal form and `inertia_quat` is updated.
        If none are provided, default values are applied.
        The order of precedence is: diagonal_inertia > inertia > inertia_matrix > default values.

        :param mass: The mass of the body.
        :param inertia_pos: The position of the inertia frame relative to the body frame.
        :param inertia_quat: The orientation of the inertia frame relative to the body frame as a quaternion [qw, qx, qy, qz].
        :param diagonal_inertia: The diagonal elements of the inertia tensor [Ixx, Iyy, Izz].
        :param inertia: The inertia tensor in the form [Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
        :param inertia_matrix: The inertia tensor as a 3x3 matrix in row-major order [Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz].
        :return: A tuple containing the resulting mass, inertia position (center of mass), inertia quaternion, and diagonal inertia.
        """
        # Case 1: diagonal_inertia is provided directly
        if diagonal_inertia is not None:
            return mass, inertia_pos, inertia_quat, diagonal_inertia

        # Case 2: inertia is provided as [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        if inertia is not None:
            Ixx, Iyy, Izz, Ixy, Ixz, Iyz = inertia
            I = numpy.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])
        # Case 3: inertia_matrix is provided as row-major 3x3
        elif inertia_matrix is not None:
            I = numpy.array(inertia_matrix).reshape(3, 3)
        # Case 4: default inertia
        else:
            I = numpy.eye(3) * 1.5e-8

        # Check if inertia is already (approximately) diagonal
        off_diag = I - numpy.diag(numpy.diag(I))
        if numpy.allclose(off_diag, 0.0):
            return mass, inertia_pos, inertia_quat, numpy.diag(I).tolist()

        # Diagonalize inertia tensor
        eigenvalues, eigenvectors = numpy.linalg.eigh(I)

        # Sort eigenvalues/eigenvectors for consistency
        idx = numpy.argsort(eigenvalues)
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]

        # Ensure right-handed basis
        if numpy.linalg.det(eigenvectors) < 0:
            eigenvectors[:, 0] *= -1

        R_orig = Rotation.from_quat(quat=inertia_quat, scalar_first=True)  # type: ignore
        R_diag = Rotation.from_matrix(eigenvectors)  # type: ignore
        R_new = R_orig * R_diag
        inertia_quat = R_new.as_quat(scalar_first=True).tolist()

        return mass, inertia_pos, inertia_quat, eigenvalues.tolist()


class RegionConverter(KinematicStructureEntityConverter, ABC):
    entity_type: ClassVar[Type[WorldEntity]] = Region


class ShapeConverter(EntityConverter, ABC):
    entity_type: ClassVar[Type[Shape]] = Shape
    pos_str: str
    quat_str: str
    rgba_str: str

    def _convert(self, entity: Shape, **kwargs) -> Dict[str, Any]:
        """
        Converts a Shape object to a dictionary of shape properties for Multiverse.

        :param entity: The Shape object to convert.
        :return: A dictionary of shape properties, by default containing position, quaternion, and RGBA color.
        """
        geom_props = EntityConverter._convert(self, entity)
        px, py, pz, qw, qx, qy, qz = cas_pose_to_list(entity.origin)
        geom_pos = [px, py, pz]
        geom_quat = [qw, qx, qy, qz]
        r, g, b, a = (
            entity.color.R,
            entity.color.G,
            entity.color.B,
            entity.color.A,
        )
        geom_color = [r, g, b, a]
        geom_props.update(
            {
                self.pos_str: geom_pos,
                self.quat_str: geom_quat,
                self.rgba_str: geom_color,
            }
        )
        return geom_props


class BoxConverter(ShapeConverter, ABC):
    entity_type: ClassVar[Type[Box]] = Box


class SphereConverter(ShapeConverter, ABC):
    entity_type: ClassVar[Type[Sphere]] = Sphere


class CylinderConverter(ShapeConverter, ABC):
    entity_type: ClassVar[Type[Cylinder]] = Cylinder


class ConnectionConverter(EntityConverter, ABC):
    entity_type: ClassVar[Type[Connection]] = Connection
    pos_str: str
    quat_str: str

    def _convert(self, entity: Connection, **kwargs) -> Dict[str, Any]:
        """
        Converts a Connection object to a dictionary of joint properties for Multiverse.

        :param entity: The Connection object to convert.
        :return: A dictionary of joint properties, by default containing position and quaternion.
        """
        joint_props = EntityConverter._convert(self, entity)
        px, py, pz, qw, qx, qy, qz = cas_pose_to_list(entity.origin)
        joint_pos = [px, py, pz]
        joint_quat = [qw, qx, qy, qz]
        joint_props.update(
            {
                self.pos_str: joint_pos,
                self.quat_str: joint_quat,
            }
        )
        return joint_props


class Connection1DOFConverter(ConnectionConverter, ABC):
    entity_type: ClassVar[Type[ActiveConnection1DOF]] = ActiveConnection1DOF
    axis_str: str
    range_str: str

    def _convert(self, entity: ActiveConnection1DOF, **kwargs) -> Dict[str, Any]:
        """
        Converts an ActiveConnection1DOF object to a dictionary of joint properties for Multiverse.

        :param entity: The ActiveConnection1DOF object to convert.
        :return: A dictionary of joint properties, including additional axis and range properties.
        """
        joint_props = ConnectionConverter._convert(self, entity)
        assert len(entity.dofs) == 1, "ActiveConnection1DOF must have exactly one DOF."
        dof = list(entity.dofs)[0]
        joint_props.update(
            {
                self.axis_str: entity.axis.to_np().tolist()[:3],
                self.range_str: [dof.lower_limits.position, dof.upper_limits.position],
            }
        )
        return joint_props


class ConnectionRevoluteConverter(Connection1DOFConverter, ABC):
    entity_type: ClassVar[Type[RevoluteConnection]] = RevoluteConnection


class ConnectionPrismaticConverter(Connection1DOFConverter, ABC):
    entity_type: ClassVar[Type[PrismaticConnection]] = PrismaticConnection


class MujocoGeomConverter(ShapeConverter, ABC):
    pos_str: str = "pos"
    quat_str: str = "quat"
    rgba_str: str = "rgba"
    type: mujoco.mjtGeom

    def _post_convert(
        self, entity: Shape, shape_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        shape_props.update(
            {
                "type": self.type,
            }
        )
        if not kwargs.get("visible", True):
            shape_props[self.rgba_str][3] = 0.0
        if not kwargs.get("collidable", True):
            shape_props["contype"] = 0
            shape_props["conaffinity"] = 0
        return shape_props


class MujocoBoxConverter(MujocoGeomConverter, BoxConverter):
    type: mujoco.mjtGeom = mujoco.mjtGeom.mjGEOM_BOX

    def _post_convert(
        self, entity: Box, shape_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        shape_props.update(MujocoGeomConverter._post_convert(self, entity, shape_props))
        shape_props.update(
            {"size": [entity.scale.x / 2, entity.scale.y / 2, entity.scale.z / 2]}
        )
        return shape_props


class MujocoSphereConverter(MujocoGeomConverter, SphereConverter):
    type: mujoco.mjtGeom = mujoco.mjtGeom.mjGEOM_SPHERE

    def _post_convert(
        self, entity: Sphere, shape_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        shape_props.update(MujocoGeomConverter._post_convert(self, entity, shape_props))
        shape_props.update({"size": [entity.radius, entity.radius, entity.radius]})
        return shape_props


class MujocoCylinderConverter(MujocoGeomConverter, CylinderConverter):
    type: mujoco.mjtGeom = mujoco.mjtGeom.mjGEOM_CYLINDER

    def _post_convert(
        self, entity: Cylinder, shape_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        shape_props.update(MujocoGeomConverter._post_convert(self, entity, shape_props))
        shape_props.update({"size": [entity.width / 2, entity.height, 0.0]})
        return shape_props


class MujocoKinematicStructureEntityConverter(KinematicStructureEntityConverter, ABC):
    pos_str: str = "pos"
    quat_str: str = "quat"


class MujocoBodyConverter(MujocoKinematicStructureEntityConverter, BodyConverter):
    mass_str: str = "mass"
    inertia_pos_str: str = "ipos"
    inertia_quat_str: str = "iquat"
    diagonal_inertia_str: str = "inertia"

    def _post_convert(
        self, entity: Body, body_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        return body_props


class MujocoRegionConverter(MujocoKinematicStructureEntityConverter, RegionConverter):
    def _post_convert(
        self, entity: Region, region_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        return region_props


class MujocoJointConverter(ConnectionConverter, ABC):
    pos_str: str = "pos"
    quat_str: str = "quat"
    type: mujoco.mjtJoint

    def _post_convert(
        self, entity: Connection, joint_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        joint_props["type"] = self.type
        return joint_props


class Mujoco1DOFJointConverter(MujocoJointConverter, Connection1DOFConverter):
    axis_str: str = "axis"
    range_str: str = "range"

    def _post_convert(
        self, entity: ActiveConnection1DOF, joint_props: Dict[str, Any], **kwargs
    ) -> Dict[str, Any]:
        joint_props = MujocoJointConverter._post_convert(self, entity, joint_props)
        if not numpy.allclose(joint_props["quat"], [1.0, 0.0, 0.0, 0.0]):
            joint_axis = numpy.array(joint_props["axis"])
            R_joint = Rotation.from_quat(quat=joint_props["quat"], scalar_first=True)  # type: ignore
            joint_props["axis"] = R_joint.apply(joint_axis).tolist()
        del joint_props["quat"]
        return joint_props


class MujocoRevoluteJointConverter(
    Mujoco1DOFJointConverter, ConnectionRevoluteConverter
):
    type: mujoco.mjtJoint = mujoco.mjtJoint.mjJNT_HINGE


class MujocoPrismaticJointConverter(
    Mujoco1DOFJointConverter, ConnectionPrismaticConverter
):
    type: mujoco.mjtJoint = mujoco.mjtJoint.mjJNT_SLIDE


@dataclass
class MultiSimBuilder(ABC):
    """
    A builder to build a world in the Multiverse simulator.
    """

    def build_world(self, world: World, file_path: str):
        """
        Builds the world in the simulator and saves it to a file.

        :param world: The world to build.
        :param file_path: The file path to save the world to.
        """

        self._start_build(file_path=file_path)

        for body in world.bodies:
            self.build_body(body=body)

        for region in world.regions:
            self.build_region(region=region)

        for connection in world.connections:
            self._build_connection(connection)

        self._end_build(file_path=file_path)

    def build_body(self, body: Body):
        """
        Builds a body in the simulator including its shapes.

        :param body: The body to build.
        """
        self._build_body(body=body)
        for shape in {id(s): s for s in body.visual + body.collision}.values():
            self._build_shape(
                parent=body,
                shape=shape,
                visible=shape in body.visual or not body.visual,
                collidable=shape in body.collision,
            )

    def build_region(self, region: Region):
        """
        Builds a region in the simulator including its shapes.

        :param region: The region to build.
        """
        self._build_region(region=region)
        for shape in region.area:
            self._build_shape(
                parent=region, shape=shape, visible=True, collidable=False
            )

    @abstractmethod
    def _start_build(self, file_path: str):
        """
        Starts the building process for the simulator.

        :param file_path: The file path to save the world to.
        """
        raise NotImplementedError

    @abstractmethod
    def _end_build(self, file_path: str):
        """
        Ends the building process for the simulator and saves the world to a file.

        :param file_path: The file path to save the world to.
        """
        raise NotImplementedError

    @abstractmethod
    def _build_body(self, body: Body):
        """
        Builds a body in the simulator.

        :param body: The body to build.
        """
        raise NotImplementedError

    @abstractmethod
    def _build_region(self, region: Region):
        """
        Builds a region in the simulator.

        :param region: The region to build.
        """
        raise NotImplementedError

    @abstractmethod
    def _build_shape(
        self, parent: Union[Body, Region], shape: Shape, visible: bool, collidable: bool
    ):
        """
        Builds a shape in the simulator and attaches it to its parent body or region.

        :param parent: The parent body or region to attach the shape to.
        :param shape: The shape to build.
        :param visible: Whether the shape is visible.
        :param collidable: Whether the shape is collidable.
        """
        raise NotImplementedError

    @abstractmethod
    def _build_connection(self, connection: Connection):
        """
        Builds a connection in the simulator.

        :param connection: The connection to build.
        """
        raise NotImplementedError


@dataclass
class MujocoBuilder(MultiSimBuilder):
    """
    A builder to build a world in the Mujoco simulator.
    """

    spec: mujoco.MjSpec = field(default=mujoco.MjSpec())

    def _start_build(self, file_path: str):
        self.spec = mujoco.MjSpec()
        self.spec.modelname = "scene"

    def _end_build(self, file_path: str):
        self.spec.compile()
        self.spec.to_file(file_path)
        try:
            mujoco.MjModel.from_xml_path(file_path)
        except ValueError as e:
            if (
                "Error: mass and inertia of moving bodies must be larger than mjMINVAL"
                in str(e)
            ):  # Fix mujoco error
                import xml.etree.ElementTree as ET

                tree = ET.parse(file_path)
                root = tree.getroot()
                for body_id, body_element in enumerate(root.findall(".//body")):
                    body_spec = self.spec.bodies[body_id + 1]
                    inertial_element = ET.SubElement(body_element, "inertial")
                    inertial_element.set("mass", f"{body_spec.mass}")
                    inertial_element.set(
                        "diaginertia", " ".join(map(str, body_spec.inertia.tolist()))
                    )
                    inertial_element.set(
                        "pos", " ".join(map(str, body_spec.ipos.tolist()))
                    )
                    inertial_element.set(
                        "quat", " ".join(map(str, body_spec.iquat.tolist()))
                    )
                tree.write(file_path)

    def _build_body(self, body: Body):
        self._build_mujoco_body(body=body)

    def _build_region(self, region: Region):
        self._build_mujoco_body(body=region)

    def _build_shape(
        self, parent: Union[Body, Region], shape: Shape, visible: bool, collidable: bool
    ):
        geom_props = MujocoGeomConverter.convert(
            shape, visible=visible, collidable=collidable
        )
        assert geom_props is not None, f"Failed to convert shape {id(shape)}."
        parent_body_name = parent.name.name
        parent_body_spec = self._find_entity("body", parent_body_name)
        assert (
            parent_body_spec is not None
        ), f"Parent body {parent_body_name} not found."
        geom_id = id(shape)
        geom_spec = parent_body_spec.add_geom(**geom_props)
        assert (
            geom_spec is not None
        ), f"Failed to add geom {geom_id} to body {parent_body_name}."

    def _build_connection(self, connection: Connection):
        if isinstance(connection, FixedConnection):
            return
        joint_props = MujocoJointConverter.convert(connection)
        assert (
            joint_props is not None
        ), f"Failed to convert connection {connection.name.name}."
        child_body_name = connection.child.name.name
        child_body_spec = self._find_entity("body", child_body_name)
        assert child_body_spec is not None, f"Child body {child_body_name} not found."
        joint_name = connection.name.name
        joint_spec = child_body_spec.add_joint(**joint_props)
        assert (
            joint_spec is not None
        ), f"Failed to add joint {joint_name} to body {child_body_name}."

    def _build_mujoco_body(self, body: Union[Region, Body]):
        """
        Builds a body in the Mujoco spec. In Mujoco, regions are also represented as bodies.

        :param body: The body or region to build.
        """
        if body.name.name == "world":
            return
        body_props = MujocoKinematicStructureEntityConverter.convert(body)
        assert body_props is not None, f"Failed to convert body {body.name.name}."
        parent_body_name = body.parent_connection.parent.name.name
        parent_body_spec = self._find_entity("body", parent_body_name)
        assert (
            parent_body_spec is not None
        ), f"Parent body {parent_body_name} not found."
        body_spec = parent_body_spec.add_body(**body_props)
        assert (
            body_spec is not None
        ), f"Failed to add body {body.name.name} to parent {parent_body_name}."

    def _find_entity(
        self, entity_type: str, entity_name: str
    ) -> Optional[
        Union[mujoco.MjsBody, mujoco.MjsGeom, mujoco.MjsJoint, mujoco.MjsSite]
    ]:
        """
        Finds an entity in the Mujoco spec by its type and name.

        :param entity_type: The type of the entity (body, geom, joint, site).
        :param entity_name: The name of the entity.
        :return: The entity if found, None otherwise.
        """
        assert entity_type in [
            "body",
            "geom",
            "joint",
        ], f"Invalid entity type {entity_type}."
        if mujoco.mj_version() >= 330:
            return self.spec.__getattribute__(entity_type)(entity_name)
        else:
            return self.spec.__getattribute__(f"find_{entity_type}")(entity_name)


class WorldEntitySpawner(ABC):
    """
    A spawner to spawn a WorldEntity object in the Multiverse simulator.
    """

    entity_type: ClassVar[Type[Any]] = Any

    @classmethod
    def spawn(cls, simulator: MultiverseSimulator, entity: entity_type) -> bool:  # type: ignore
        """
        Spawns a WorldEntity object in the Multiverse simulator.

        :param simulator: The Multiverse simulator to spawn the entity in.
        :param entity: The WorldEntity object to spawn.
        :return: True if the entity was spawned successfully, False otherwise.
        """
        for subclass in recursive_subclasses(cls):
            if (
                not inspect.isabstract(subclass)
                and not inspect.isabstract(subclass.entity_type)
                and type(entity) is subclass.entity_type
            ):
                return subclass()._spawn(simulator, entity)
        raise NotImplementedError(f"No converter found for entity type {type(entity)}.")

    @abstractmethod
    def _spawn(self, simulator: MultiverseSimulator, entity: Any) -> bool:
        """
        The actual spawning method to be implemented by subclasses.

        :param simulator: The Multiverse simulator to spawn the entity in.
        :param entity: The WorldEntity object to spawn.
        :return: True if the entity was spawned successfully, False otherwise.
        """
        raise NotImplementedError


class KinematicStructureEntitySpawner(WorldEntitySpawner):
    entity_type: ClassVar[Type[KinematicStructureEntity]] = KinematicStructureEntity

    def _spawn(
        self, simulator: MultiverseSimulator, entity: KinematicStructureEntity
    ) -> bool:
        """
        Spawns a KinematicStructureEntity object in the Multiverse simulator including its shapes.

        :param simulator: The Multiverse simulator to spawn the entity in.
        :param entity: The KinematicStructureEntity object to spawn.
        :return: True if the entity and its shapes were spawned successfully, False otherwise.
        """
        return self._spawn_kinematic_structure_entity(
            simulator, entity
        ) and self._spawn_shapes(simulator, entity)

    @abstractmethod
    def _spawn_kinematic_structure_entity(
        self, simulator: MultiverseSimulator, entity: KinematicStructureEntity
    ) -> bool:
        """
        Spawns a KinematicStructureEntity object in the Multiverse simulator.

        :param simulator: The Multiverse simulator to spawn the entity in.
        :param entity: The KinematicStructureEntity object to spawn.
        :return: True if the entity was spawned successfully, False otherwise.
        """
        raise NotImplementedError

    @abstractmethod
    def _spawn_shapes(
        self, simulator: MultiverseSimulator, entity: KinematicStructureEntity
    ) -> bool:
        """
        Spawns the shapes of a KinematicStructureEntity object in the Multiverse simulator.

        :param simulator: The Multiverse simulator to spawn the shapes in.
        :param entity: The KinematicStructureEntity object whose shapes to spawn.
        :return: True if all shapes were spawned successfully, False otherwise.
        """
        raise NotImplementedError

    @abstractmethod
    def _spawn_shape(
        self,
        parent: Union[Body, Region],
        simulator: MultiverseSimulator,
        shape: Shape,
        visible: bool,
        collidable: bool,
    ) -> bool:
        """
        Spawns a shape in the Multiverse simulator and attaches it to its parent body or region.

        :param parent: The parent body or region to attach the shape to.
        :param simulator: The Multiverse simulator to spawn the shape in.
        :param shape: The shape to spawn.
        :param visible: Whether the shape is visible.
        :param collidable: Whether the shape is collidable.
        :return: True if the shape was spawned successfully, False otherwise.
        """
        raise NotImplementedError


class BodySpawner(KinematicStructureEntitySpawner, ABC):
    entity_type: ClassVar[Type[Body]] = Body

    def _spawn(self, simulator: MultiverseSimulator, entity: Body) -> bool:
        return KinematicStructureEntitySpawner._spawn(self, simulator, entity)

    def _spawn_shapes(self, simulator: MultiverseSimulator, parent: Body) -> bool:
        return all(
            self._spawn_shape(
                parent=parent,
                simulator=simulator,
                shape=shape,
                visible=shape in parent.visual or not parent.visual,
                collidable=shape in parent.collision,
            )
            for shape in {id(s): s for s in parent.visual + parent.collision}.values()
        )


class RegionSpawner(KinematicStructureEntitySpawner, ABC):
    entity_type: ClassVar[Type[Region]] = Region

    def _spawn(self, simulator: MultiverseSimulator, entity: Region) -> bool:
        return KinematicStructureEntitySpawner._spawn(self, simulator, entity)

    def _spawn_shapes(self, simulator: MultiverseSimulator, parent: Region) -> bool:
        return all(
            self._spawn_shape(
                parent=parent,
                simulator=simulator,
                shape=shape,
                visible=True,
                collidable=False,
            )
            for shape in parent.area
        )


class MujocoKinematicStructureEntitySpawner(KinematicStructureEntitySpawner, ABC):
    def _spawn_kinematic_structure_entity(
        self, simulator: MultiverseMujocoConnector, entity: KinematicStructureEntity
    ) -> bool:
        kinematic_structure_entity_props = (
            MujocoKinematicStructureEntityConverter.convert(entity)
        )
        assert (
            kinematic_structure_entity_props is not None
        ), f"Failed to convert entity {entity.name.name}."
        entity_name = kinematic_structure_entity_props["name"]
        del kinematic_structure_entity_props["name"]
        result = simulator.add_entity(
            entity_name=entity_name,
            entity_type="body",
            entity_properties=kinematic_structure_entity_props,
            parent_name=entity.parent_connection.parent.name.name,
        )
        return (
            result.type
            == MultiverseCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL
        )

    def _spawn_shape(
        self,
        parent: Body,
        simulator: MultiverseMujocoConnector,
        shape: Shape,
        visible: bool,
        collidable: bool,
    ) -> bool:
        shape_props = MujocoGeomConverter.convert(
            shape, visible=visible, collidable=collidable
        )
        assert shape_props is not None, f"Failed to convert shape {id(shape)}."
        shape_name = shape_props["name"]
        del shape_props["name"]
        result = simulator.add_entity(
            entity_name=shape_name,
            entity_type="geom",
            entity_properties=shape_props,
            parent_name=parent.name.name,
        )
        return (
            result.type
            == MultiverseCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL
        )


class MujocoBodySpawner(MujocoKinematicStructureEntitySpawner, BodySpawner): ...


class MujocoRegionSpawner(MujocoKinematicStructureEntitySpawner, RegionSpawner): ...


@dataclass
class MultiSimSynchronizer(ModelChangeCallback, ABC):
    """
    A callback to synchronize the world model with the Multiverse simulator.
    This callback will listen to the world model changes and update the Multiverse simulator accordingly.
    """

    world: World
    simulator: MultiverseSimulator
    kinematic_structure_entity_converter: Type[KinematicStructureEntityConverter] = (
        NoneType
    )
    shape_converter: Type[ShapeConverter] = NoneType
    connection_converter: Type[ConnectionConverter] = NoneType
    kinematic_structure_entity_spawner: Type[KinematicStructureEntitySpawner] = NoneType

    def notify(self):
        for modification in self.world._model_modification_blocks[-1]:
            if isinstance(modification, AddKinematicStructureEntityModification):
                entity = modification.kinematic_structure_entity
                self.kinematic_structure_entity_spawner.spawn(
                    simulator=self.simulator, entity=entity
                )

    def stop(self):
        self.world.model_change_callbacks.remove(self)


@dataclass
class MujocoSynchronizer(MultiSimSynchronizer):
    simulator: MultiverseMujocoConnector
    kinematic_structure_entity_converter: Type[KinematicStructureEntityConverter] = (
        field(default=MujocoKinematicStructureEntityConverter)
    )
    shape_converter: Type[ShapeConverter] = field(default=MujocoGeomConverter)
    connection_converter: Type[ConnectionConverter] = field(
        default=MujocoJointConverter
    )
    kinematic_structure_entity_spawner: Type[KinematicStructureEntitySpawner] = field(
        default=MujocoKinematicStructureEntitySpawner
    )


class MultiSim(ABC):
    """
    Class to handle the simulation of a world using the Multiverse simulator.
    """

    simulator_class: ClassVar[Type[MultiverseSimulator]]
    synchronizer_class: ClassVar[Type[MultiSimSynchronizer]]
    builder_class: ClassVar[Type[MultiSimBuilder]]
    simulator: MultiverseSimulator
    synchronizer: MultiSimSynchronizer
    default_file_path: str

    def __init__(
        self,
        world: World,
        viewer: MultiverseViewer,
        headless: bool = False,
        step_size: float = 1e-3,
        real_time_factor: float = 1.0,
    ):
        """
        Initializes the MultiSim class.

        :param world: The world to simulate.
        :param viewer: The MultiverseViewer to read/write objects.
        :param headless: Whether to run the simulation in headless mode.
        :param step_size: The step size for the simulation.
        :param real_time_factor: The real time factor for the simulation (1.0 = real time, 2.0 = twice as fast, -1.0 = as fast as possible).
        """
        self.builder_class().build_world(world=world, file_path=self.default_file_path)
        self.simulator = self.simulator_class(
            file_path=self.default_file_path,
            viewer=viewer,
            headless=headless,
            step_size=step_size,
            real_time_factor=real_time_factor,
        )
        self.synchronizer = self.synchronizer_class(
            world=world,
            simulator=self.simulator,
        )
        self._viewer = viewer

    def start_simulation(self):
        """
        Starts the simulation. This will start a physics simulation thread and render it at 60Hz.

        :return: None
        """
        assert (
            self.simulator.state != MultiverseSimulatorState.RUNNING
        ), "Simulation is already running."
        self.simulator.start()

    def stop_simulation(self):
        """
        Stops the simulation. This will stop the physics simulation and the rendering.

        :return: None
        """
        self.synchronizer.stop()
        self.simulator.stop()

    def pause_simulation(self):
        """
        Pauses the simulation. This will pause the physics simulation but not the rendering.

        :return: None
        """
        if self.simulator.state != MultiverseSimulatorState.PAUSED:
            self.simulator.pause()

    def unpause_simulation(self):
        """
        Unpauses the simulation. This will unpause the physics simulation.

        :return: None
        """
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.unpause()

    def reset_simulation(self):
        """
        Resets the simulation. This will reset the physics simulation to the initial state.

        :return: None
        """
        self.simulator.reset()

    def set_write_objects(self, write_objects: Dict[str, Dict[str, List[float]]]):
        """
        Sets the objects to be written to the simulator.
        For example, to set the position and quaternion of an object, you can use the following format:
        {
            "object_name": {
                "position": [x, y, z],
                "quaternion": [w, x, y, z]
            }
        }

        :param write_objects: The objects to be written to the simulator.
        :return: None
        """
        self._viewer.write_objects = write_objects
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.step()

    def set_read_objects(self, read_objects: Dict[str, Dict[str, List[float]]]):
        """
        Sets the objects to be read from the simulator.

        For example, to read the position and quaternion of an object, you can use the following format:
        {
            "object_name": {
                "position": [0.0, 0.0, 0.0], # Default value
                "quaternion": [1.0, 0.0, 0.0], # Default value
            }
        }
        :param read_objects: The objects to be read from the simulator.
        :return: None
        """
        self._viewer.read_objects = read_objects
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.step()

    def get_read_objects(self) -> Dict[str, Dict[str, MultiverseAttribute]]:
        """
        Gets the objects that are being read from the simulator.
        For example, if you have set the read objects as follows:
        {
            "object_name": {
                "position": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0],
            }
        }
        You will get the following format:
        {
            "object_name": {
                "position": MultiverseAttribute(...),
                "quaternion": MultiverseAttribute(...),
            }
        }
        where MultiverseAttribute contains the values of the attribute via the .values() method.
        It will return the values that are being read from the simulator in every simulation step.

        :return: The objects that are being read from the simulator.
        """
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.step()
        return self._viewer.read_objects

    def is_stable(
        self, body_names: List[str], max_simulation_steps: int = 100, atol: float = 1e-2
    ) -> bool:
        """
        Checks if an object is stable in the world. Stable meaning that it's pose will not change after simulating
        physics in the World. This function will pause the simulation, set the read objects to the given body names,
        unpause the simulation, and check if the pose of the objects change after a certain number of simulation steps.
        If the pose of the objects change, the function will return False. If the pose of the objects do not change,
        the function will return True. After checking, the function will restore the read objects and the simulation state.

        :param body_names: The names of the bodies to check for stability
        :param max_simulation_steps: The maximum number of simulation steps to run
        :param atol: The absolute tolerance for comparing the pose
        :return: True if the object is stable, False otherwise
        """

        origin_read_objects = self.get_read_objects()
        origin_state = self.simulator.state

        self.pause_simulation()
        self.set_read_objects(
            read_objects={
                body_name: {
                    "position": [0.0, 0.0, 0.0],
                    "quaternion": [1.0, 0.0, 0.0, 0.0],
                }
                for body_name in body_names
            }
        )
        initial_body_state = numpy.array(self._viewer.read_data)
        current_simulation_step = self.simulator.current_number_of_steps
        self.unpause_simulation()
        stable = True
        while (
            self.simulator.current_number_of_steps
            < current_simulation_step + max_simulation_steps
        ):
            if numpy.abs(initial_body_state - self._viewer.read_data).max() > atol:
                stable = False
                break
            time.sleep(1e-3)
        self._viewer.read_objects = origin_read_objects
        if origin_state == MultiverseSimulatorState.PAUSED:
            self.pause_simulation()
        return stable


class MujocoSim(MultiSim):
    simulator_class: ClassVar[Type[MultiverseSimulator]] = MultiverseMujocoConnector
    synchronizer_class: ClassVar[Type[MultiSimSynchronizer]] = MujocoSynchronizer
    builder_class: ClassVar[Type[MultiSimBuilder]] = MujocoBuilder
    simulator: MultiverseMujocoConnector
    synchronizer: Type[MultiSimSynchronizer] = MujocoSynchronizer
    default_file_path: str = "/tmp/scene.xml"
