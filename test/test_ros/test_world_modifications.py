import unittest

from semantic_world.world_description.connection_factories import (
    ConnectionFactory,
    FixedConnectionFactory,
)
from semantic_world.world_description.connections import (
    FixedConnection,
    Connection6DoF,
    PrismaticConnection,
)
from semantic_world.datastructures.prefixed_name import PrefixedName
from semantic_world.spatial_types.spatial_types import Vector3
from semantic_world.world import World
from semantic_world.world_description.world_entity import Body
from semantic_world.world_description.world_modification import (
    WorldModelModificationBlock,
    UnknownWorldModification,
    AddBodyModification,
    AddConnectionModification,
    AddDegreeOfFreedomModification,
)


class ConnectionModificationTestCase(unittest.TestCase):

    def test_single_modification(self):
        w = World()

        with w.modify_world():
            b1 = Body(name=PrefixedName("b1"))
            b2 = Body(name=PrefixedName("b2"))
            w.add_kinematic_structure_entity(b1)
            w.add_kinematic_structure_entity(b2)

            connection = FixedConnection(b1, b2, _world=w)
            w.add_connection(connection)

        connection = w.connections[0]
        factory = ConnectionFactory.from_connection(connection)
        assert isinstance(factory, FixedConnectionFactory)

    def test_raising(self):
        with self.assertRaises(UnknownWorldModification):
            WorldModelModificationBlock.from_modifications([(print, {"a": 123})])

    def test_many_modifications(self):
        w = World()

        with w.modify_world():
            b1 = Body(name=PrefixedName("b1"))
            b2 = Body(name=PrefixedName("b2"))
            b3 = Body(name=PrefixedName("b3"))
            w.add_kinematic_structure_entity(b1)
            w.add_kinematic_structure_entity(b2)
            w.add_kinematic_structure_entity(b3)
            w.add_connection(Connection6DoF(b1, b2, _world=w))
            w.add_connection(
                PrismaticConnection(
                    parent=b2, child=b3, _world=w, axis=Vector3.from_iterable([0, 0, 1])
                )
            )

        modifications = WorldModelModificationBlock.from_modifications(
            w._atomic_modifications[-1]
        )
        self.assertEqual(len(modifications.modifications), 13)

        add_body_modifications = [
            m for m in modifications.modifications if isinstance(m, AddBodyModification)
        ]
        self.assertEqual(len(add_body_modifications), 3)

        add_dof_modifications = [
            m
            for m in modifications.modifications
            if isinstance(m, AddDegreeOfFreedomModification)
        ]
        self.assertEqual(len(add_dof_modifications), 8)

        add_connection_modifications = [
            m
            for m in modifications.modifications
            if isinstance(m, AddConnectionModification)
        ]
        self.assertEqual(len(add_connection_modifications), 2)

        # reconstruct this world
        w2 = World()

        # copy modifications
        modifications_copy = WorldModelModificationBlock.from_json(
            modifications.to_json()
        )
        modifications_copy.apply(w2)
        self.assertEqual(len(w2.bodies), 3)
        self.assertEqual(len(w2.connections), 2)

        with w.modify_world():
            w.remove_connection(w.connections[-1])
            w.remove_kinematic_structure_entity(
                w.get_kinematic_structure_entity_by_name("b3")
            )

        modifications = WorldModelModificationBlock.from_modifications(
            w._atomic_modifications[-1]
        )
        self.assertEqual(len(modifications.modifications), 3)

        modifications_copy = WorldModelModificationBlock.from_json(
            modifications.to_json()
        )
        modifications_copy.apply(w2)
        self.assertEqual(len(w2.bodies), 2)
        self.assertEqual(len(w2.connections), 1)


if __name__ == "__main__":
    unittest.main()
