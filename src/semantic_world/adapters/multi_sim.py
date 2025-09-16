import inspect
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from types import NoneType
from typing import Dict, List, Any, ClassVar, Type, Optional, get_type_hints, Union

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

from ..callbacks.callback import ModelChangeCallback
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
    WorldModelModificationBlock,
    AddDegreeOfFreedomModification,
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


class WorldEntityConverter(ABC):
    """
    A converter to convert a WorldEntity object to a dictionary of properties for Multiverse simulator.
    """

    @classmethod
    def convert(cls, entity: Any) -> Optional[Dict[str, Any]]:
        """
        Converts a WorldEntity object to a dictionary of properties for Multiverse.

        :param entity: The WorldEntity object to convert.
        :return: A dictionary of properties if conversion is successful, None otherwise.
        """
        if type(entity) is cls.get_type():
            return cls()._convert(entity)
        for subclass in recursive_subclasses(cls):
            result = subclass.convert(entity)
            if result is not None:
                return result
        return None

    @abstractmethod
    def _convert(self, entity: Any) -> Dict[str, Any]:
        """
        The actual conversion method to be implemented by subclasses.

        :param entity: The WorldEntity object to convert.
        :return: A dictionary of properties.
        """
        raise NotImplementedError

    @classmethod
    def get_type(cls) -> Optional[Type]:
        """
        Gets the type of WorldEntity that this converter can convert.
        :return: The type of WorldEntity, or None if the converter is abstract.
        """
        if inspect.isabstract(cls):
            return None
        hints = get_type_hints(cls._convert)
        return hints.get("entity", NoneType)


class KinematicStructureEntityConverter(WorldEntityConverter):
    pos_name: str
    quat_name: str

    def _convert(self, entity: KinematicStructureEntity) -> Dict[str, Any]:
        """
        Converts a KinematicStructureEntity object to a dictionary of body properties for Multiverse.

        :param entity: The KinematicStructureEntity object to convert.
        :return: A dictionary of body properties, by default containing position and quaternion.
        """
        px, py, pz, qw, qx, qy, qz = cas_pose_to_list(
            entity.parent_connection.origin_expression
        )
        kinematic_structure_entity_pos = [px, py, pz]
        kinematic_structure_entity_quat = [qw, qx, qy, qz]
        kinematic_structure_entity_props = {
            self.pos_name: kinematic_structure_entity_pos,
            self.quat_name: kinematic_structure_entity_quat,
        }
        kinematic_structure_entity_props.update(
            self._convert_kinematic_structure_entity(entity)
        )
        return kinematic_structure_entity_props

    @abstractmethod
    def _convert_kinematic_structure_entity(
        self, entity: KinematicStructureEntity
    ) -> Dict[str, Any]:
        """
        Converts a KinematicStructureEntity object to a dictionary of additional properties for Multiverse.

        :param entity: The KinematicStructureEntity object to convert.
        :return: A dictionary of additional properties.
        """
        raise NotImplementedError


class BodyConverter(KinematicStructureEntityConverter, ABC):
    def _convert(self, entity: Body) -> Dict[str, Any]:
        return KinematicStructureEntityConverter._convert(self, entity)


class RegionConverter(KinematicStructureEntityConverter, ABC):
    def _convert(self, entity: Region) -> Dict[str, Any]:
        return KinematicStructureEntityConverter._convert(self, entity)


class ShapeConverter(WorldEntityConverter):
    pos_name: str
    quat_name: str
    rgba_name: str
    visible: bool

    def _convert(self, entity: Shape) -> Dict[str, Any]:
        """
        Converts a Shape object to a dictionary of shape properties for Multiverse.

        :param entity: The Shape object to convert.
        :return: A dictionary of shape properties, by default containing position, quaternion, and RGBA color.
        """
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
        geom_props = {
            self.pos_name: geom_pos,
            self.quat_name: geom_quat,
            self.rgba_name: geom_color,
        }
        geom_props.update(self._convert_shape(entity))
        return geom_props

    @abstractmethod
    def _convert_shape(self, entity: Shape) -> Dict[str, Any]:
        """
        Converts a Shape object to a dictionary of additional properties for Multiverse.

        :param entity: The Shape object to convert.
        :return: A dictionary of additional properties.
        """
        raise NotImplementedError


class BoxConverter(ShapeConverter, ABC):
    def _convert(self, entity: Box) -> Dict[str, Any]:
        return ShapeConverter._convert(self, entity)


class SphereConverter(ShapeConverter, ABC):
    def _convert(self, entity: Sphere) -> Dict[str, Any]:
        return ShapeConverter._convert(self, entity)


class CylinderConverter(ShapeConverter, ABC):
    def _convert(self, entity: Cylinder) -> Dict[str, Any]:
        return ShapeConverter._convert(self, entity)


class ConnectionConverter(WorldEntityConverter):
    @abstractmethod
    def _convert(self, entity: Connection) -> Dict[str, Any]:
        """
        Converts a Connection object to a dictionary of joint properties for Multiverse.

        :param entity: The Connection object to convert.
        :return: A dictionary of joint properties.
        """
        raise NotImplementedError

    @abstractmethod
    def _convert_joint(self, entity: Connection) -> Dict[str, Any]:
        """
        Converts a Connection object to a dictionary of additional joint properties for Multiverse.

        :param entity: The Connection object to convert.
        :return: A dictionary of additional joint properties.
        """
        raise NotImplementedError


class Connection1DOFConverter(ConnectionConverter):
    type: Any
    axis_name: str
    range_name: str

    @abstractmethod
    def _convert(self, entity: ActiveConnection1DOF) -> Dict[str, Any]:
        """
        Converts a 1-DOF Connection object to a dictionary of joint properties for Multiverse.

        :param entity: The 1-DOF Connection object to convert.
        :return: A dictionary of joint properties, by default containing axis and range.
        """
        assert len(entity.dofs) == 1, "ActiveConnection1DOF must have exactly one DOF."
        dof = list(entity.dofs)[0]
        joint_props = {
            self.axis_name: entity.axis.to_np().tolist()[:3],
            self.range_name: [dof.lower_limits.position, dof.upper_limits.position],
        }
        joint_props.update(self._convert_joint(entity))
        return joint_props


class ConnectionRevoluteConverter(Connection1DOFConverter, ABC):
    def _convert(self, entity: RevoluteConnection) -> Dict[str, Any]:
        return Connection1DOFConverter._convert(self, entity)


class ConnectionPrismaticConverter(Connection1DOFConverter, ABC):
    def _convert(self, entity: PrismaticConnection) -> Dict[str, Any]:
        return Connection1DOFConverter._convert(self, entity)


class MujocoGeomConverter(ShapeConverter):
    pos_name: str = "pos"
    quat_name: str = "quat"
    rgba_name: str = "rgba"
    type: mujoco.mjtGeom

    def _convert_shape(self, entity: Shape) -> Dict[str, Any]:
        return {
            "type": self.type,
            "size": self._get_size(entity),
        }

    @abstractmethod
    def _get_size(self, entity: Shape) -> List[float]:
        """
        Gets the size of the shape for Mujoco.

        :param entity: The Shape object to get the size from.
        :return: A list of floats representing the size.
        """
        raise NotImplementedError


class MujocoBoxConverter(MujocoGeomConverter, BoxConverter):
    type: mujoco.mjtGeom = mujoco.mjtGeom.mjGEOM_BOX

    def _get_size(self, entity: Box) -> List[float]:
        return [entity.scale.x / 2, entity.scale.y / 2, entity.scale.z / 2]


class MujocoSphereConverter(MujocoGeomConverter, SphereConverter):
    type: mujoco.mjtGeom = mujoco.mjtGeom.mjGEOM_SPHERE

    def _get_size(self, entity: Sphere) -> List[float]:
        return [entity.radius, entity.radius, entity.radius]


class MujocoCylinderConverter(MujocoGeomConverter, CylinderConverter):
    type: mujoco.mjtGeom = mujoco.mjtGeom.mjGEOM_CYLINDER

    def _get_size(self, entity: Cylinder) -> List[float]:
        return [entity.width / 2, entity.height, 0.0]


class MujocoKinematicStructureEntityConverter(KinematicStructureEntityConverter):
    pos_name: str = "pos"
    quat_name: str = "quat"

    def _convert_kinematic_structure_entity(
        self, entity: KinematicStructureEntity
    ) -> Dict[str, Any]:
        return {}


class MujocoBodyConverter(MujocoKinematicStructureEntityConverter, BodyConverter): ...


class MujocoRegionConverter(
    MujocoKinematicStructureEntityConverter, RegionConverter
): ...


class MujocoJointConverter(ConnectionConverter, ABC):
    type: mujoco.mjtJoint


class Mujoco1DOFJointConverter(MujocoJointConverter, Connection1DOFConverter, ABC):
    axis_name: str = "axis"
    range_name: str = "range"

    def _convert_joint(self, entity: ActiveConnection1DOF) -> Dict[str, Any]:
        return {
            "type": self.type,
        }


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

    def _build_body(self, body: Body):
        self._build_mujoco_body(body=body)

    def _build_region(self, region: Region):
        self._build_mujoco_body(body=region)

    def _build_shape(
        self, parent: Union[Body, Region], shape: Shape, visible: bool, collidable: bool
    ):
        geom_props = MujocoGeomConverter.convert(shape)
        assert geom_props is not None, f"Failed to convert shape {id(shape)}."
        if not visible:
            geom_props["rgba"][3] = 0.0
        if not collidable:
            geom_props["contype"] = 0
            geom_props["conaffinity"] = 0
        parent_body_name = parent.name.name
        parent_body_spec = self._find_entity("body", parent_body_name)
        assert (
            parent_body_spec is not None
        ), f"Parent body {parent_body_name} not found."
        geom_id = id(shape)
        geom_spec = parent_body_spec.add_geom(
            name=f"{parent_body_name}_{geom_id}", **geom_props
        )
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
        joint_spec = child_body_spec.add_joint(
            name=joint_name,
            **joint_props,
        )
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
        body_name = body.name.name
        body_spec = parent_body_spec.add_body(name=body_name, **body_props)
        assert (
            body_spec is not None
        ), f"Failed to add body {body_name} to parent {parent_body_name}."

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

    @classmethod
    def spawn(cls, simulator: MultiverseSimulator, entity: Any) -> bool:
        """
        Spawns a WorldEntity object in the Multiverse simulator.

        :param simulator: The Multiverse simulator to spawn the entity in.
        :param entity: The WorldEntity object to spawn.
        :return: True if the entity was spawned successfully, False otherwise.
        """
        if type(entity) is cls.get_type():
            return cls()._spawn(simulator, entity)
        for subclass in recursive_subclasses(cls):
            result = subclass.spawn(simulator, entity)
            if result:
                return True
        return False

    @abstractmethod
    def _spawn(self, simulator: MultiverseSimulator, entity: Any) -> bool:
        """
        The actual spawning method to be implemented by subclasses.

        :param simulator: The Multiverse simulator to spawn the entity in.
        :param entity: The WorldEntity object to spawn.
        :return: True if the entity was spawned successfully, False otherwise.
        """
        raise NotImplementedError

    @classmethod
    def get_type(cls) -> Optional[Type]:
        """
        Gets the type of WorldEntity that this spawner can spawn.

        :return: The type of WorldEntity, or None if the spawner is abstract.
        """
        if inspect.isabstract(cls):
            return None
        hints = get_type_hints(cls._spawn)
        return hints.get("entity", NoneType)


class KinematicStructureEntitySpawner(WorldEntitySpawner):
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
        result = simulator.add_entity(
            entity_name=entity.name.name,
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
        shape_props = MujocoGeomConverter.convert(shape)
        assert shape_props is not None, f"Failed to convert shape {id(shape)}."
        if not visible:
            shape_props["rgba"][3] = 0.0
        if not collidable:
            shape_props["contype"] = 0
            shape_props["conaffinity"] = 0
        parent_name = parent.name.name
        result = simulator.add_entity(
            entity_name=f"{parent_name}_{id(shape)}",
            entity_type="geom",
            entity_properties=shape_props,
            parent_name=parent_name,
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
    kinematic_structure_entity_converter: Type[KinematicStructureEntityConverter]
    shape_converter: Type[ShapeConverter]
    connection_converter: Type[ConnectionConverter]
    kinematic_structure_entity_spawner: Type[KinematicStructureEntitySpawner]

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
            world=world, simulator=self.simulator
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
