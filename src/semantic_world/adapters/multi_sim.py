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
        self.simulator.start()

    def stop_simulation(self):
        self.simulator.stop()

    def pause_simulation(self):
        if self.simulator.state != MultiverseSimulatorState.PAUSED:
            self.simulator.pause()

    def unpause_simulation(self):
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.unpause()

    def reset_simulation(self):
        self.simulator.reset()

    def set_write_objects(self, write_objects: Dict[str, Dict[str, List[float]]]):
        self._viewer.write_objects = write_objects
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.step()

    def set_read_objects(self, read_objects: Dict[str, Dict[str, List[float]]]):
        self._viewer.read_objects = read_objects
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.step()

    def get_read_objects(self) -> Dict[str, Dict[str, MultiverseAttribute]]:
        if self.simulator.state == MultiverseSimulatorState.PAUSED:
            self.simulator.step()
        return self._viewer.read_objects

    def is_stable(self, body_names: List[str], max_simulation_steps: int = 100, atol: float = 1E-2) -> bool:
        """
        Checks if an object is stable in the world. Stable meaning that it's position will not change after simulating
        physics in the World. This will be done by simulating the world for 10 seconds and compare
        the previous coordinates with the coordinates after the simulation.

        :param body_names: The names of the bodies to check for stability
        :param max_simulation_steps: The maximum number of simulation steps to run
        :param atol: The absolute tolerance for comparing the positions
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
