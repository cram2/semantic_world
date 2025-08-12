import time
import unittest

import rclpy

from semantic_world.adapters.viz_marker import VizMarkerPublisher
from semantic_world.geometry import Scale
from semantic_world.prefixed_name import PrefixedName
from semantic_world.views.factories import HandleFactory, Alignment


class MyTestCase(unittest.TestCase):
    def test_handle_factory(self):
        handle_factory = HandleFactory(
            name=PrefixedName("test_handle"), scale=Scale(0.05, 0.1, 0.02)
        )
        world = handle_factory.create()

        # rclpy.init()
        #
        # node = rclpy.create_node("viz_marker")
        #
        # p = VizMarkerPublisher(world, node)
        # time.sleep(100)
        # p._stop_publishing()
        # rclpy.shutdown()

if __name__ == "__main__":
    unittest.main()
