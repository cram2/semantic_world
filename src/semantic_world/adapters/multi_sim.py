from typing import Dict, List
import numpy
import time
from mujoco_connector import MultiverseMujocoConnector
from multiverse_simulator import MultiverseSimulator, MultiverseSimulatorState, MultiverseViewer, MultiverseAttribute
from ..world import World

class MultiSim:
    """
    Class to handle the simulation of a world using the Multiverse simulator.
    """
    world: World
    simulator: MultiverseSimulator

    def __init__(self,
                 file_path: str,
                 world: World,
                 viewer: MultiverseViewer,
                 headless: bool = False,
                 step_size: float = 1E-3,
                 simulator: str = "mujoco",
                 real_time_factor: float = 1.0):
        if simulator == "mujoco":
            Simulator = MultiverseMujocoConnector
        else:
            raise NotImplementedError(f"Simulator {simulator} is not implemented yet.")

        self.world = world
        self._viewer = viewer
        self.simulator = Simulator(file_path=file_path,
                                   viewer=viewer,
                                   headless=headless,
                                   step_size=step_size,
                                   real_time_factor=real_time_factor)

    def start_simulation(self):
        """
        Starts the simulation. This will start two threads, one for the physics simulation and one for the rendering.

        :return: None
        """
        assert self.simulator.state != MultiverseSimulatorState.RUNNING, "Simulation is already running."
        self.simulator.start()

    def stop_simulation(self):
        """
        Stops the simulation. This will stop the physics simulation and the rendering.

        :return: None
        """
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

    def is_stable(self, body_names: List[str], max_simulation_steps: int = 100, atol: float = 1E-2) -> bool:
        """
        Checks if an object is stable in the world. Stable meaning that it's pose will not change after simulating
        physics in the World. This will be done by simulating the world for 10 seconds and compare
        the previous coordinates with the coordinates after the simulation.

        :param body_names: The names of the bodies to check for stability
        :param max_simulation_steps: The maximum number of simulation steps to run
        :param atol: The absolute tolerance for comparing the pose
        :return: True if the object is stable, False otherwise
        """

        origin_read_objects = self.get_read_objects()
        origin_state = self.simulator.state

        self.pause_simulation()
        self.set_read_objects(read_objects = {body_name: {
            "position": [0.0, 0.0, 0.0],
            "quaternion": [1.0, 0.0, 0.0, 0.0]
        } for body_name in body_names})
        initial_body_state = numpy.array(self._viewer.read_data)
        current_simulation_step = self.simulator.current_number_of_steps
        self.unpause_simulation()
        stable = True
        while self.simulator.current_number_of_steps < current_simulation_step + max_simulation_steps:
            if numpy.abs(initial_body_state - self._viewer.read_data).max() > atol:
                stable = False
                break
            time.sleep(1E-3)
        self._viewer.read_objects = origin_read_objects
        if origin_state == MultiverseSimulatorState.PAUSED:
            self.pause_simulation()
        return stable
