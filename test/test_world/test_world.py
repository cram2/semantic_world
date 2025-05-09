import unittest

from semantic_world.prefixed_name import PrefixedName
from semantic_world.world import World, Body, Connection


class WorldTestCase(unittest.TestCase):

    def setUp(self):
        self.world = World()
        b1 = Body(
            PrefixedName("b1")
        )
        b2 = Body(
            PrefixedName("b2")
        )
        b3 = Body(
            PrefixedName("b3")
        )
        c1 = Connection(b1, b2)
        c2 = Connection(self.world.root, b3)
        c3 = Connection(self.world.root, b1)
        self.world.add_connection(c1)
        self.world.add_connection(c2)
        self.world.add_connection(c3)

    def test_construction(self):
        self.world.validate()
        self.assertEqual(len(self.world.connections), 3)
        self.assertEqual(len(self.world.bodies), 4)





if __name__ == '__main__':
    unittest.main()
