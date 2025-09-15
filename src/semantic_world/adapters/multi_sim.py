import os
import time
from typing import Dict, List, Optional

import numpy
from mujoco_connector import MultiverseMujocoConnector
from multiverse_simulator import (
    MultiverseSimulator,
    MultiverseSimulatorState,
    MultiverseViewer,
    MultiverseAttribute,
    MultiverseCallbackResult,
)

from ..world import World
from ..world_description.connections import RevoluteConnection, PrismaticConnection
from ..world_description.geometry import Box, Cylinder, Sphere
from ..world_description.world_modification import (
    WorldModelModificationBlock,
    AddDegreeOfFreedomModification,
    AddBodyModification,
)
from ..spatial_types.symbol_manager import symbol_manager


class MultiSim:
    """
    Class to handle the simulation of a world using the Multiverse simulator.
    """

    world: World
    simulator: MultiverseSimulator

    def __init__(
        self,
        world: World,
        viewer: MultiverseViewer,
        headless: bool = False,
        step_size: float = 1e-3,
        simulator: str = "mujoco",
        real_time_factor: float = 1.0,
    ):
        self.world = world
        if simulator == "mujoco":
            Simulator = MultiverseMujocoConnector
            file_path = "/tmp/scene.xml"
            self.build_world(file_path=file_path)
        else:
            raise NotImplementedError(f"Simulator {simulator} is not implemented yet.")

        self._viewer = viewer
        self.simulator = Simulator(
            file_path=file_path,
            viewer=viewer,
            headless=headless,
            step_size=step_size,
            real_time_factor=real_time_factor,
        )

        self.callback_lambda = lambda: self.model_change_callback()
        self.world.model_change_callbacks.append(self.callback_lambda)

    def build_world(self, file_path: str):
        file_ext = os.path.splitext(file_path)[1]
        if file_ext == ".xml":
            import mujoco

            spec = mujoco.MjSpec()
            spec.modelname = "scene"
            body_map = {"world": spec.worldbody}
            for body in self.world.bodies:
                if body.name.name == "world":
                    continue
                body_name = body.name.name
                parent_body_name = body.parent_connection.parent.name.name
                body_pos = (
                    body.parent_connection.origin.to_position().to_np().tolist()[:3]
                )
                body_quat = (
                    body.parent_connection.origin.to_quaternion().to_np().tolist()[:4]
                )
                body_quat = [body_quat[3], body_quat[0], body_quat[1], body_quat[2]]
                body_spec = body_map[parent_body_name].add_body(
                    name=body.name.name, pos=body_pos, quat=body_quat
                )
                body_map[body_name] = body_spec
                for geom in {id(s): s for s in body.visual + body.collision}.values():
                    geom_pos = geom.origin.to_position().to_np().tolist()[:3]
                    geom_quat = geom.origin.to_quaternion().to_np().tolist()[:4]
                    geom_quat = [geom_quat[3], geom_quat[0], geom_quat[1], geom_quat[2]]
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
        return file_path

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
        self.world.model_change_callbacks.remove(self.callback_lambda)
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

    def model_change_callback(self):
        latest_changes = WorldModelModificationBlock.from_modifications(
            self.world._atomic_modifications[-1]
        )
        self.update_world(latest_changes)

    def update_world(self, latest_changes: WorldModelModificationBlock):
        for modification in latest_changes.modifications:
            if isinstance(modification, AddBodyModification):
                body = modification.body
                parent_body_name = body.parent_connection.parent.name.name
                body_name = body.name.name
                body_pose = body.parent_connection.origin_expression
                body_pos = body_pose.to_position()
                px, py, pz, _ = symbol_manager.evaluate_expr(body_pos).tolist()
                body_quat = body_pose.to_quaternion()
                qx, qy, qz, qw = symbol_manager.evaluate_expr(body_quat).tolist()
                if isinstance(self.simulator, MultiverseMujocoConnector):
                    body_props = {
                        "pos": [px, py, pz],
                        "quat": [qw, qx, qy, qz],
                    }
                    add_body_result = self.simulator.add_entity_to_body(
                        entity_name=body_name,
                        entity_type="body",
                        entity_properties=body_props,
                        body_name=parent_body_name,
                    )
                    assert (
                        add_body_result.type
                        == MultiverseCallbackResult.ResultType.SUCCESS_AFTER_EXECUTION_ON_MODEL
                    )
                else:
                    continue
