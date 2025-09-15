import time
from dataclasses import dataclass
from typing import Dict, List, Any

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

from ..callbacks.callback import ModelChangeCallback
from ..spatial_types.spatial_types import TransformationMatrix
from ..world import World
from ..world_description.connections import RevoluteConnection, PrismaticConnection
from ..world_description.geometry import Box, Cylinder, Sphere, Shape
from ..world_description.world_entity import Region, Body, KinematicStructureEntity
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


@dataclass
class MultiverseSynchronizer(ModelChangeCallback):
    """
    A callback to synchronize the world model with the Multiverse simulator.
    This callback will listen to the world model changes and update the Multiverse simulator accordingly.
    """

    world: World
    simulator: MultiverseSimulator

    def notify(self):
        for modification in self.world._model_modification_blocks[-1]:
            if isinstance(modification, AddKinematicStructureEntityModification):
                entity = modification.kinematic_structure_entity
                if not self.spawn_kinematic_structure_entity(entity):
                    raise RuntimeError(
                        f"Failed to spawn kinematic structure entity {entity.name.name}."
                    )
                if isinstance(entity, Region):
                    for shape in entity.area:
                        if not self.spawn_geometry(
                            entity=entity,
                            shape=shape,
                        ):
                            raise RuntimeError(
                                f"Failed to spawn geometry {shape} for region {entity.name.name}."
                            )
                elif isinstance(entity, Body):
                    for shape in {
                        id(s): s for s in entity.visual + entity.collision
                    }.values():
                        if not self.spawn_geometry(
                            entity=entity,
                            shape=shape,
                        ):
                            raise RuntimeError(
                                f"Failed to spawn geometry {shape} for body {entity.name.name}."
                            )
                else:
                    raise NotImplementedError(
                        f"Can't parse entity type {type(entity)}."
                    )

    def stop(self):
        self.world.model_change_callbacks.remove(self)

    def build_world(self, file_path: str):
        """
        Builds the world in the simulator from the given file path.

        :param file_path: The file path to the world file.
        """
        raise NotImplementedError

    def spawn_kinematic_structure_entity(self, entity: KinematicStructureEntity) -> bool:
        """
        Spawns a kinematic structure entity in the simulator.

        :param entity: The kinematic structure entity to spawn.
        :return: True if the entity was spawned successfully, False otherwise.
        """
        raise NotImplementedError

    def spawn_geometry(self, entity: KinematicStructureEntity, shape: Shape) -> bool:
        """
        Spawns a geometry in the simulator.

        :param entity: The parent entity of the geometry to spawn.
        :param shape: The shape of the geometry to spawn.
        :return: True if the geometry was spawned successfully, False otherwise.
        """
        raise NotImplementedError


class MujocoSynchronizer(MultiverseSynchronizer):
    simulator: MultiverseMujocoConnector

    def build_world(self, file_path: str):
        spec = mujoco.MjSpec()
        spec.modelname = "scene"
        body_map = {"world": spec.worldbody}
        for body in self.world.bodies:
            if body.name.name == "world":
                continue
            body_name = body.name.name
            parent_body_name = body.parent_connection.parent.name.name
            px, py, pz, qw, qx, qy, qz = cas_pose_to_list(body.parent_connection.origin)
            body_pos = [px, py, pz]
            body_quat = [qw, qx, qy, qz]
            body_spec = body_map[parent_body_name].add_body(
                name=body.name.name, pos=body_pos, quat=body_quat
            )
            body_map[body_name] = body_spec
            for geom in {id(s): s for s in body.visual + body.collision}.values():
                px, py, pz, qw, qx, qy, qz = cas_pose_to_list(geom.origin)
                geom_pos = [px, py, pz]
                geom_quat = [qw, qx, qy, qz]
                if isinstance(geom, Box):
                    size = [geom.scale.x / 2, geom.scale.y / 2, geom.scale.z / 2]
                    geom_spec = body_spec.add_geom(
                        type=mujoco.mjtGeom.mjGEOM_BOX,
                        pos=geom_pos,
                        quat=geom_quat,
                        size=size,
                    )
                elif isinstance(geom, Cylinder):
                    size = [geom.width / 2, geom.height, 0.0]
                    geom_spec = body_spec.add_geom(
                        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        pos=geom_pos,
                        quat=geom_quat,
                        size=size,
                    )
                elif isinstance(geom, Sphere):
                    size = [geom.radius, geom.radius, geom.radius]
                    geom_spec = body_spec.add_geom(
                        type=mujoco.mjtGeom.mjGEOM_SPHERE,
                        pos=geom_pos,
                        quat=geom_quat,
                        size=size,
                    )
        for connection in self.world.connections:
            add_prefix = len(connection.dofs) == 1
            joint_name = connection.name.name
            child_body_name = connection.child.name.name
            child_body_spec = body_map[child_body_name]
            for i, dof in enumerate(connection.dofs):
                if isinstance(connection, RevoluteConnection):
                    joint_axis = connection.axis.to_np().tolist()[:3]
                    joint_range = [
                        dof.lower_limits.position,
                        dof.upper_limits.position,
                    ]
                    child_body_spec.add_joint(
                        name=joint_name if add_prefix else f"{joint_name}_{i}",
                        type=mujoco.mjtJoint.mjJNT_HINGE,
                        axis=joint_axis,
                        range=joint_range,
                    )
                elif isinstance(connection, PrismaticConnection):
                    joint_axis = connection.axis.to_np().tolist()[:3]
                    joint_range = [
                        dof.lower_limits.position,
                        dof.upper_limits.position,
                    ]
                    child_body_spec.add_joint(
                        name=joint_name if add_prefix else f"{joint_name}_{i}",
                        type=mujoco.mjtJoint.mjJNT_SLIDE,
                        axis=joint_axis,
                        range=joint_range,
                    )
                else:
                    raise NotImplementedError(
                        f"Connection type {type(connection)} is not implemented yet."
                    )
        spec.compile()
        spec.to_file(file_path)

    def spawn_kinematic_structure_entity(self, entity: Body) -> bool:
        parent_body_name = entity.parent_connection.parent.name.name
        entity_name = entity.name.name
        entity_pose = entity.parent_connection.origin_expression
        px, py, pz, qw, qx, qy, qz = cas_pose_to_list(entity_pose)
        body_pos = [px, py, pz]
        body_quat = [qw, qx, qy, qz]
        body_props = {
            "pos": body_pos,
            "quat": body_quat,
        }
        add_body_result = self.simulator.add_entity(
            entity_name=entity_name,
            entity_type="body",
            entity_properties=body_props,
            parent_name=parent_body_name,
        )
        return add_body_result.type == MultiverseCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL

    def spawn_geometry(self, entity: KinematicStructureEntity, shape: Shape) -> bool:
        parent_name = entity.name.name
        if isinstance(entity, Region):
            site_props = self.shape_to_geom_props(shape)
            add_site_result = self.simulator.add_entity(
                entity_name=f"{parent_name}_{id(shape)}",
                entity_type="site",
                entity_properties=site_props,
                parent_name=parent_name,
            )
            return add_site_result.type == MultiverseCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL
        elif isinstance(entity, Body):
            geom_props = self.shape_to_geom_props(shape)
            if shape in entity.collision and entity.visual:
                geom_props["rgba"][3] = 0.0
            add_geom_result = self.simulator.add_entity(
                entity_name=f"{parent_name}_{id(shape)}",
                entity_type="geom",
                entity_properties=geom_props,
                parent_name=parent_name,
            )
            return add_geom_result.type == MultiverseCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL
        else:
            raise NotImplementedError(f"Can't parse entity type {type(entity)}.")

    @staticmethod
    def shape_to_geom_props(shape: Shape) -> Dict[str, Any]:
        """
        Converts a Shape object to a dictionary of geometry properties for Multiverse.

        :param shape: The Shape object to convert.
        :return: A dictionary of geometry properties.
        """
        px, py, pz, qw, qx, qy, qz = cas_pose_to_list(shape.origin)
        geom_pos = [px, py, pz]
        geom_quat = [qw, qx, qy, qz]
        r, g, b, a = (
            shape.color.R,
            shape.color.G,
            shape.color.B,
            shape.color.A,
        )
        geom_color = [r, g, b, a]
        if isinstance(shape, Box):
            size = [
                symbol_manager.evaluate_expr(shape.scale.x) / 2,
                symbol_manager.evaluate_expr(shape.scale.y) / 2,
                symbol_manager.evaluate_expr(shape.scale.z) / 2,
            ]
            return {
                "type": mujoco.mjtGeom.mjGEOM_BOX,
                "pos": geom_pos,
                "quat": geom_quat,
                "size": size,
                "rgba": geom_color,
            }
        elif isinstance(shape, Cylinder):
            size = [
                symbol_manager.evaluate_expr(shape.width) / 2,
                symbol_manager.evaluate_expr(shape.height),
                0.0,
            ]
            return {
                "type": mujoco.mjtGeom.mjGEOM_CYLINDER,
                "pos": geom_pos,
                "quat": geom_quat,
                "size": size,
                "rgba": geom_color,
            }
        elif isinstance(shape, Sphere):
            size = [
                symbol_manager.evaluate_expr(shape.radius),
                symbol_manager.evaluate_expr(shape.radius),
                symbol_manager.evaluate_expr(shape.radius),
            ]
            return {
                "type": mujoco.mjtGeom.mjGEOM_SPHERE,
                "pos": geom_pos,
                "quat": geom_quat,
                "size": size,
                "rgba": geom_color,
            }
        else:
            raise NotImplementedError(f"Shape type {type(shape)} is not implemented yet.")

class MultiSim:
    """
    Class to handle the simulation of a world using the Multiverse simulator.
    """

    simulator: MultiverseSimulator
    synchronizer: MultiverseSynchronizer

    def __init__(
        self,
        world: World,
        viewer: MultiverseViewer,
        headless: bool = False,
        step_size: float = 1e-3,
        simulator: str = "mujoco",
        real_time_factor: float = 1.0,
    ):
        """
        Initializes the MultiSim class.

        :param world: The world to simulate.
        :param viewer: The MultiverseViewer to read/write objects.
        :param headless: Whether to run the simulation in headless mode.
        :param step_size: The step size for the simulation.
        :param simulator: The simulator to use. Currently only "mujoco" is supported.
        :param real_time_factor: The real time factor for the simulation (1.0 = real time, 2.0 = twice as fast, -1.0 = as fast as possible).
        """
        if simulator == "mujoco":
            Simulator = MultiverseMujocoConnector
            file_path = "/tmp/scene.xml"
            self.simulator = Simulator(
                file_path=file_path,
                viewer=viewer,
                headless=headless,
                step_size=step_size,
                real_time_factor=real_time_factor,
            )
            self.synchronizer = MujocoSynchronizer(world, self.simulator)
        else:
            raise NotImplementedError(f"Simulator {simulator} is not implemented yet.")

        self._viewer = viewer
        self.synchronizer.build_world(file_path=file_path)

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
