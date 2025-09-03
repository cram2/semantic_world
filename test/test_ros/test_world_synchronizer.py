import threading
import time
import unittest
from unittest import skipUnless

import sqlalchemy
from ormatic.utils import drop_database
from sqlalchemy import select
from sqlalchemy.orm import Session

from semantic_world.world_description import Connection6DoF
from semantic_world.orm.ormatic_interface import Base, WorldMappingDAO
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.utils import rclpy_installed
from semantic_world.world import World
from semantic_world.world_description import Body

if rclpy_installed():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from semantic_world.adapters.ros.world_synchronizer import StateSynchronizer, ModelReloadSynchronizer, \
        ModelSynchronizer


@skipUnless(rclpy_installed(), "rclpy required")
class WorldSynchronizerTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    @staticmethod
    def create_dummy_world():
        w = World()
        b1 = Body(name=PrefixedName("b1"))
        b2 = Body(name=PrefixedName("b2"))
        with w.modify_world():
            w.add_kinematic_structure_entity(b1)
            w.add_kinematic_structure_entity(b2)
            w.add_connection(Connection6DoF(b1, b2, _world=w))
        return w

    def test_state_synchronization(self):

        # Create an isolated node per test to avoid cross-talk across tests
        node = rclpy.create_node(f"WorldStatePublisher_test_state_synchronization")

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        synch_thread = threading.Thread(target=executor.spin, daemon=True)
        synch_thread.start()
        time.sleep(0.1)

        w1 = self.create_dummy_world()
        w2 = self.create_dummy_world()

        synchronizer_1 = StateSynchronizer(
            node=node,
            world=w1,
        )
        synchronizer_2 = StateSynchronizer(
            node=node,
            world=w2,
        )

        # Allow time for publishers/subscribers to connect on unique topics
        time.sleep(0.2)

        w1.state.data[0, 0] = 1.0
        w1.notify_state_change()
        time.sleep(0.2)
        assert w1.state.data[0, 0] == 1.0
        assert w1.state.data[0, 0] == w2.state.data[0, 0]

        synchronizer_1.close()
        synchronizer_2.close()
        node.destroy_node()
        synch_thread.join(timeout=1)

    def test_model_reload(self):

        # Create an isolated node per test to avoid cross-talk across tests
        node = rclpy.create_node(f"WorldStatePublisher_test_model_reload")

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        synch_thread = threading.Thread(target=executor.spin, daemon=True)
        synch_thread.start()

        engine = sqlalchemy.create_engine(
            "sqlite+pysqlite:///file::memory:?cache=shared",
            connect_args={"check_same_thread": False, "uri": True},
        )
        drop_database(engine)
        Base.metadata.create_all(engine)
        session_maker = sqlalchemy.orm.sessionmaker(bind=engine)
        session1 = session_maker()
        session2 = session_maker()

        w1 = self.create_dummy_world()
        w2 = World()

        synchronizer_1 = ModelReloadSynchronizer(
            node,
            w1,
            session=session1,
        )
        synchronizer_2 = ModelReloadSynchronizer(
            node,
            w2,
            session=session2,
        )

        synchronizer_1.publish_reload_model()
        time.sleep(1.0)
        self.assertEqual(len(w2.kinematic_structure_entities), 2)

        query = session1.scalars(select(WorldMappingDAO)).all()
        assert len(query) == 1
        assert w2.get_kinematic_structure_entity_by_name("b2")

        synchronizer_1.close()
        synchronizer_2.close()
        node.destroy_node()
        synch_thread.join(timeout=1)

    def test_model_synchronization_body_only(self):
        # Create an isolated node per test to avoid cross-talk across tests
        node = rclpy.create_node(
            f"WorldStatePublisher_test_model_synchronization_body_only"
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Start spinning in a thread
        synch_thread = threading.Thread(target=executor.spin, daemon=True)
        synch_thread.start()
        time.sleep(0.1)

        w1 = World(name="w1")
        w2 = World(name="w2")

        synchronizer_1 = ModelSynchronizer(
            node=node,
            world=w1,
        )
        synchronizer_2 = ModelSynchronizer(
            node=node,
            world=w2,
        )

        with w1.modify_world():
            new_body = Body(name=PrefixedName("b3"))
            w1.add_kinematic_structure_entity(new_body)
            self.assertEqual(len(w1.kinematic_structure_entities), 1)

        time.sleep(0.1)
        self.assertEqual(len(w1.kinematic_structure_entities), 1)
        self.assertEqual(len(w2.kinematic_structure_entities), 1)

        assert w2.get_kinematic_structure_entity_by_name("b3")

        synchronizer_1.close()
        synchronizer_2.close()
        node.destroy_node()
        synch_thread.join(timeout=1.0)

    def test_model_synchronization_creation_only(self):
        # Create an isolated node per test to avoid cross-talk across tests
        node = rclpy.create_node(
            f"WorldStatePublisher_test_model_synchronization_creation_only"
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        synch_thread = threading.Thread(target=executor.spin, daemon=True)
        synch_thread.start()
        time.sleep(0.1)
        w1 = World(name="w1")
        w2 = World(name="w2")

        synchronizer_1 = ModelSynchronizer(
            node=node,
            world=w1,
        )
        synchronizer_2 = ModelSynchronizer(
            node=node,
            world=w2,
        )

        with w1.modify_world():
            b2 = Body(name=PrefixedName("b2"))
            w1.add_kinematic_structure_entity(b2)

            new_body = Body(name=PrefixedName("b3"))
            w1.add_kinematic_structure_entity(new_body)

            c = Connection6DoF(b2, new_body, _world=w1)
            w1.add_connection(c)
        time.sleep(0.1)
        self.assertEqual(len(w1.kinematic_structure_entities), 2)
        self.assertEqual(len(w2.kinematic_structure_entities), 2)
        self.assertEqual(len(w1.connections), 1)
        self.assertEqual(len(w2.connections), 1)

        synchronizer_1.close()
        synchronizer_2.close()
        node.destroy_node()
        synch_thread.join(timeout=1)


if __name__ == "__main__":
    unittest.main()
