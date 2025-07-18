from typing import Dict, List

from mujoco_connector import MultiverseMujocoConnector
from multiverse_simulator import MultiverseSimulator, MultiverseSimulatorState, MultiverseViewer, MultiverseAttribute
from ..world import World
import mujoco

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
                 simulator: str = "mujoco"):
        if simulator == "mujoco":
            Simulator = MultiverseMujocoConnector
        else:
            raise NotImplementedError(f"Simulator {simulator} is not implemented yet.")

        self.world = world
        self._viewer = viewer
        self.simulator = Simulator(file_path=file_path,
                                   viewer=viewer,
                                   headless=headless,
                                   step_size=step_size)

    def start_simulation(self):
        self.simulator.start()

    def stop_simulation(self):
        self.simulator.stop()

    def pause_simulation(self):
        self.simulator.pause()

    def unpause_simulation(self):
        self.simulator.unpause()

    def reset_simulation(self):
        self.simulator.reset()

    def set_write_objects(self, write_objects: Dict[str, Dict[str, List[float]]]):
        self._viewer.write_objects = write_objects
        if self.simulator.state == MultiverseSimulatorState.PAUSED and isinstance(self.simulator, MultiverseMujocoConnector):
            self.simulator.step()

    def get_read_objects(self) -> Dict[str, Dict[str, MultiverseAttribute]]:
        return self._viewer.read_objects