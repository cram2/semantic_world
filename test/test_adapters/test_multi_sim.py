import os
import unittest
import time
from mujoco_connector import MultiverseMujocoConnector
from multiverse_simulator import MultiverseSimulatorState, MultiverseViewer
from semantic_world.adapters.multi_sim import MultiSim
from semantic_world.world import World

mjcf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "mjcf")

class MultiverseMujocoConnectorTestCase(unittest.TestCase):
    file_path = os.path.normpath(os.path.join(mjcf_dir, "mjx_single_cube_no_mesh.xml"))
    Simulator = MultiverseMujocoConnector
    headless = False
    step_size = 5E-4

    def test_read_and_write_data_in_the_loop(self):
        viewer = MultiverseViewer()
        simulator = self.Simulator(viewer=viewer,
                                   file_path=self.file_path,
                                   headless=self.headless,
                                   step_size=self.step_size)
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)
        self.assertIs(simulator.headless, self.headless)
        self.assertIsNone(simulator.stop_reason)
        self.assertIsNone(simulator.simulation_thread)
        simulator.start(simulate_in_thread=False)
        for step in range(1000):
            if step == 100:
                read_objects = {
                    "joint1": {
                        "joint_rvalue": [0.0],
                        "joint_angular_velocity": [0.0]
                    },
                    "joint2": {
                        "joint_rvalue": [0.0],
                        "joint_angular_velocity": [0.0]
                    }
                }
                viewer.read_objects = read_objects
            elif step == 101:
                read_objects = {
                    "joint1": {
                        "joint_angular_velocity": [0.0]
                    },
                    "joint2": {
                        "joint_rvalue": [0.0],
                        "joint_torque": [0.0]
                    }
                }
                viewer.read_objects = read_objects
            elif step == 102:
                write_objects = {
                    "joint1": {
                        "joint_rvalue": [1.0]
                    },
                    "actuator2": {
                        "cmd_joint_rvalue": [2.0]
                    },
                    "box": {
                        "position": [1.1, 2.2, 3.3],
                        "quaternion": [0.707, 0.0, 0.707, 0.0]
                    }
                }
                read_objects = {
                    "joint1": {
                        "joint_rvalue": [0.0],
                        "joint_angular_velocity": [0.0]
                    },
                    "actuator2": {
                        "cmd_joint_rvalue": [0.0]
                    },
                    "box": {
                        "position": [0.0, 0.0, 0.0],
                        "quaternion": [0.0, 0.0, 0.0, 0.0]
                    }
                }
                viewer.write_objects = write_objects
                viewer.read_objects = read_objects
            else:
                viewer.read_objects = {}
            simulator.step()
            if step == 100:
                self.assertEqual(viewer.read_data.shape, (1, 4))
                self.assertEqual(viewer.read_objects["joint1"]["joint_rvalue"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_rvalue"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint1"]["joint_angular_velocity"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_angular_velocity"].values.shape, (1, 1))
            elif step == 101:
                self.assertEqual(viewer.read_data.shape, (1, 3))
                self.assertEqual(viewer.read_objects["joint1"]["joint_angular_velocity"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_rvalue"].values.shape, (1, 1))
                self.assertEqual(viewer.read_objects["joint2"]["joint_torque"].values.shape, (1, 1))
            elif step == 102:
                self.assertEqual(viewer.write_data.shape, (1, 9))
                self.assertEqual(viewer.write_objects["joint1"]["joint_rvalue"].values[0], (1.0,))
                self.assertEqual(viewer.write_objects["actuator2"]["cmd_joint_rvalue"].values[0], (2.0,))
                self.assertEqual(viewer.write_objects["box"]["position"].values[0][0], 1.1)
                self.assertEqual(viewer.write_objects["box"]["position"].values[0][1], 2.2)
                self.assertEqual(viewer.write_objects["box"]["position"].values[0][2], 3.3)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][0], 0.707)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][1], 0.0)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][2], 0.707)
                self.assertEqual(viewer.write_objects["box"]["quaternion"].values[0][3], 0.0)
                self.assertEqual(viewer.read_data.shape, (1, 10))
                self.assertAlmostEqual(viewer.read_objects["joint1"]["joint_rvalue"].values[0][0], 1.0, places=3)
                self.assertEqual(viewer.read_objects["actuator2"]["cmd_joint_rvalue"].values[0][0], 2.0)
                self.assertEqual(viewer.read_objects["box"]["position"].values[0][0], 1.1)
                self.assertEqual(viewer.read_objects["box"]["position"].values[0][1], 2.2)
                self.assertAlmostEqual(viewer.read_objects["box"]["position"].values[0][2], 3.3, places=3)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][0], 0.7071067811865475)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][1], 0.0)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][2], 0.7071067811865475)
                self.assertAlmostEqual(viewer.read_objects["box"]["quaternion"].values[0][3], 0.0)
            else:
                self.assertEqual(viewer.read_data.shape, (1, 0))
        simulator.stop()
        self.assertIs(simulator.state, MultiverseSimulatorState.STOPPED)

class MultiSimTestCase(unittest.TestCase):
    file_path = os.path.normpath(os.path.join(mjcf_dir, "mjx_single_cube_no_mesh.xml"))
    headless = False
    step_size = 5E-4

    def test_multi_sim_in_5s(self):
        world = World()
        multi_sim = MultiSim(file_path=self.file_path, world=world, headless=self.headless, step_size=self.step_size)
        self.assertIsInstance(multi_sim.simulator, MultiverseMujocoConnector)
        self.assertEqual(multi_sim.simulator.file_path, self.file_path)
        self.assertIs(multi_sim.simulator.headless, self.headless)
        self.assertEqual(multi_sim.simulator.step_size, self.step_size)
        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()
        self.assertAlmostEqual(time.time() - start_time, 5.0, delta=0.1)

    def test_read_objects_from_multi_sim_in_5s(self):
        world = World()
        read_objects = {
            "joint1": {
                "joint_rvalue": [0.0],
                "joint_angular_velocity": [0.0]
            },
            "joint2": {
                "joint_rvalue": [0.0],
                "joint_angular_velocity": [0.0]
            },
            "actuator1": {
                "cmd_joint_rvalue": [0.0]
            },
            "actuator2": {
                "cmd_joint_rvalue": [0.0]
            },
            "world": {
                "energy": [0.0, 0.0]
            },
            "box": {
                "position": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0]
            }
        }
        viewer = MultiverseViewer(read_objects=read_objects)
        multi_sim = MultiSim(file_path=self.file_path, viewer=viewer, world=world, headless=self.headless, step_size=self.step_size)
        self.assertIsInstance(multi_sim.simulator, MultiverseMujocoConnector)
        self.assertEqual(multi_sim.simulator.file_path, self.file_path)
        self.assertIs(multi_sim.simulator.headless, self.headless)
        self.assertEqual(multi_sim.simulator.step_size, self.step_size)
        multi_sim.start_simulation()
        start_time = time.time()
        for _ in range(5):
            print(f"Time: {multi_sim.simulator.current_simulation_time} - Objects: {multi_sim.get_read_objects()}")
            time.sleep(1)
        multi_sim.stop_simulation()
        self.assertAlmostEqual(time.time() - start_time, 5.0, delta=0.1)

    def test_write_objects_to_multi_sim_in_5s(self):
        world = World()
        write_objects = {
            "box": {
                "position": [0.0, 0.0, 0.0]
            }
        }
        viewer = MultiverseViewer(write_objects=write_objects)
        multi_sim = MultiSim(file_path=self.file_path, viewer=viewer, world=world, headless=self.headless, step_size=self.step_size)
        self.assertIsInstance(multi_sim.simulator, MultiverseMujocoConnector)
        self.assertEqual(multi_sim.simulator.file_path, self.file_path)
        self.assertIs(multi_sim.simulator.headless, self.headless)
        self.assertEqual(multi_sim.simulator.step_size, self.step_size)
        multi_sim.start_simulation()
        box_positions = [[0.0, 0.0, 1.0],
                         [1.0, 0.0, 1.0],
                         [1.0, 1.0, 1.0],
                         [0.0, 1.0, 1.0],
                         [0.0, 0.0, 1.0]]
        multi_sim.pause_simulation()
        start_time = time.time()
        for box_position in box_positions:
            write_objects["box"]["position"] = box_position
            multi_sim.set_write_objects(write_objects=write_objects)
            time.sleep(1)
        multi_sim.stop_simulation()
        self.assertAlmostEqual(time.time() - start_time, 5.0, delta=0.1)

    def test_read_and_write_objects_to_multi_sim_in_5s(self):
        world = World()
        read_objects = {
            "box": {
                "position": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0]
            }
        }
        write_objects = {
            "box": {
                "position": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0]
            }
        }
        viewer = MultiverseViewer()
        multi_sim = MultiSim(file_path=self.file_path, viewer=viewer, world=world, headless=self.headless, step_size=self.step_size)
        self.assertIsInstance(multi_sim.simulator, MultiverseMujocoConnector)
        self.assertEqual(multi_sim.simulator.file_path, self.file_path)
        self.assertIs(multi_sim.simulator.headless, self.headless)
        self.assertEqual(multi_sim.simulator.step_size, self.step_size)
        multi_sim.start_simulation()
        start_time = time.time()
        box_positions = [[0.0, 0.0, 1.0],
                         [1.0, 0.0, 1.0],
                         [1.0, 1.0, 1.0],
                         [0.0, 1.0, 1.0],
                         [0.0, 0.0, 1.0]]
        for box_position in box_positions:
            write_objects["box"]["position"] = box_position
            multi_sim.pause_simulation()
            # multi_sim.set_write_objects(write_objects=write_objects)
            time.sleep(2)
            multi_sim.unpause_simulation()
            # multi_sim.set_write_objects(write_objects={})
            time.sleep(2)
        multi_sim.stop_simulation()
        self.assertAlmostEqual(time.time() - start_time, 20.0, delta=0.1)

if __name__ == '__main__':
    unittest.main()