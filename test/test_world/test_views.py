import os
import unittest

from semantic_world.adapters.urdf import URDFParser


class ViewTestCase(unittest.TestCase):
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    apartment = os.path.join(urdf_dir, "apartment.urdf")

    def setUp(self):
        self.apartment_parser = URDFParser(self.apartment)

    def test_apartment_views(self):
        world = self.apartment_parser.parse()
        world.validate()



if __name__ == '__main__':
    unittest.main()
