import os.path
import unittest

from semantic_world.adapters.urdf import URDFParser
from semantic_world.connections import FixedConnection


class URDFParserTestCase(unittest.TestCase):
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    table = os.path.join(urdf_dir, "table.urdf")
    kitchen = os.path.join(urdf_dir, "kitchen-small.urdf")
    apartment = os.path.join(urdf_dir, "apartment.urdf")
    pr2 = os.path.join(urdf_dir, "pr2_kinematic_tree.urdf")

    def setUp(self):
        self.table_parser = URDFParser(self.table)
        self.kitchen_parser = URDFParser(self.kitchen)
        self.apartment_parser = URDFParser(self.apartment)
        self.pr2_parser = URDFParser(self.pr2)

    def test_table_parsing(self):
        world = self.table_parser.parse()
        world.validate()
        self.assertEqual(len(world.bodies), 6)

        origin_left_front_leg_joint = world.get_connection(world.root, world.bodies[1])
        self.assertIsInstance(origin_left_front_leg_joint, FixedConnection)

    def test_kitchen_parsing(self):
        world = self.kitchen_parser.parse()
        world.validate()
        self.assertTrue(len(world.bodies) > 0)
        self.assertTrue(len(world.connections) > 0)

    def test_apartment_parsing(self):
        world = self.apartment_parser.parse()
        world.validate()
        self.assertTrue(len(world.bodies) > 0)
        self.assertTrue(len(world.connections) > 0)

    def test_pr2_parsing(self):
        world = self.pr2_parser.parse()
        world.validate()
        self.assertTrue(len(world.bodies) > 0)
        self.assertTrue(len(world.connections) > 0)
        self.assertTrue(world.root.name.name == 'base_footprint')


if __name__ == '__main__':
    unittest.main()
