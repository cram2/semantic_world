import os.path
import unittest

from semantic_world.adapters.multi_parser import MultiParser
from semantic_world.world_description.connections import FixedConnection, Connection6DoF


class MultiParserTestCase(unittest.TestCase):
    urdf_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf"
    )
    mjcf_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "mjcf"
    )
    table_urdf = os.path.join(urdf_dir, "table.urdf")
    kitchen_urdf = os.path.join(urdf_dir, "kitchen-small.urdf")
    apartment_urdf = os.path.join(urdf_dir, "apartment.urdf")
    pr2_urdf = os.path.join(urdf_dir, "pr2_kinematic_tree.urdf")
    table_xml = os.path.join(mjcf_dir, "table.xml")
    kitchen_xml = os.path.join(mjcf_dir, "kitchen-small.xml")
    apartment_xml = os.path.join(mjcf_dir, "apartment.xml")
    pr2_xml = os.path.join(mjcf_dir, "pr2_kinematic_tree.xml")
    single_cube = os.path.join(mjcf_dir, "mjx_single_cube_no_mesh.xml")

    def setUp(self):
        self.table_urdf_parser = MultiParser(self.table_urdf)
        self.kitchen_urdf_parser = MultiParser(self.kitchen_urdf)
        self.apartment_urdf_parser = MultiParser(self.apartment_urdf)
        self.pr2_urdf_parser = MultiParser(self.pr2_urdf)
        self.table_xml_parser = MultiParser(self.table_xml)
        self.kitchen_xml_parser = MultiParser(self.kitchen_xml)
        self.apartment_xml_parser = MultiParser(self.apartment_xml)
        self.pr2_xml_parser = MultiParser(self.pr2_xml)
        self.single_cube_parser = MultiParser(self.single_cube)

    def test_table_parsing(self):
        total_body_num = 7
        for world, body_num in zip(
            [self.table_urdf_parser.parse(), self.table_xml_parser.parse()], [total_body_num, total_body_num]
        ):
            world.validate()
            self.assertTrue(len(world.kinematic_structure_entities) == body_num)

            table_to_world = world.get_body_by_name("table_origin").parent_connection
            self.assertIsInstance(table_to_world, FixedConnection)

            origin_left_front_leg_joint = world.get_body_by_name("left_front_leg").parent_connection
            self.assertIsInstance(origin_left_front_leg_joint, FixedConnection)

    def test_kitchen_parsing(self):
        for world in [
            self.kitchen_urdf_parser.parse(),
            self.kitchen_xml_parser.parse(),
        ]:
            world.validate()
            self.assertTrue(len(world.kinematic_structure_entities) > 0)
            self.assertTrue(len(world.connections) > 0)

    def test_apartment_parsing(self):
        for world in [
            self.apartment_urdf_parser.parse(),
            self.apartment_xml_parser.parse(),
        ]:
            world.validate()
            self.assertTrue(len(world.kinematic_structure_entities) > 0)
            self.assertTrue(len(world.connections) > 0)

    def test_pr2_parsing(self):
        for world in [self.pr2_urdf_parser.parse(), self.pr2_xml_parser.parse()]:
            world.validate()
            self.assertTrue(len(world.kinematic_structure_entities) > 0)
            self.assertTrue(len(world.connections) > 0)
            self.assertTrue(world.root.name.name == "world")

    def test_free_bodies(self):
        world = self.single_cube_parser.parse(fixed_base=False)
        self.assertIsInstance(world.get_body_by_name("box").parent_connection, Connection6DoF)


if __name__ == "__main__":
    unittest.main()
