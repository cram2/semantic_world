import unittest

import numpy as np

from semantic_world.connections import PrismaticConnection, RevoluteConnection
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.derivatives import Derivatives
from semantic_world.world import World, Body, Connection


class WorldTestCase(unittest.TestCase):

    def setUp(self):
        self.world = World()
        l1 = Body(
            PrefixedName('l1')
        )
        l2 = Body(
            PrefixedName('l2')
        )
        r1 = Body(
            PrefixedName('r1')
        )
        r2 = Body(
            PrefixedName('r2')
        )

        dof = self.world.create_free_variable(name=PrefixedName('dof'),
                                              lower_limits={Derivatives.velocity: -1},
                                              upper_limits={Derivatives.velocity: 1})

        c_l1_l2 = PrismaticConnection(l1, l2, free_variable=dof, axis=(1, 0, 0))
        c_r1_r2 = RevoluteConnection(r1, r2, free_variable=dof, axis=(0, 0, 1))
        c_root_l1 = Connection(self.world.root, l1)
        c_root_r1 = Connection(self.world.root, r1)
        self.world.add_connection(c_l1_l2)
        self.world.add_connection(c_r1_r2)
        self.world.add_connection(c_root_l1)
        self.world.add_connection(c_root_r1)

    def test_construction(self):
        self.world.validate()
        self.assertEqual(len(self.world.connections), 4)
        self.assertEqual(len(self.world.bodies), 5)
        assert self.world.position_state[0] == 0

    def test_chain(self):
        result = self.world.compute_chain(root_link_name=PrefixedName('root'),
                                          tip_link_name=PrefixedName('l2'),
                                          add_joints=True, add_links=True, add_fixed_joints=True,
                                          add_non_controlled_joints=True)
        assert result == [PrefixedName('root'), PrefixedName('root_T_l1'), PrefixedName('l1'), PrefixedName('l1_T_l2'),
                          PrefixedName('l2')]

    def test_split_chain(self):
        result = self.world.compute_split_chain(root_link_name=PrefixedName('r2'),
                                                tip_link_name=PrefixedName('l2'),
                                                add_joints=True, add_links=True, add_fixed_joints=True,
                                                add_non_controlled_joints=True)
        assert result == ([PrefixedName(name='r2'), PrefixedName(name='r1_T_r2'), PrefixedName(name='r1'),
                           PrefixedName(name='root_T_r1')],
                          [PrefixedName(name='root')],
                          [PrefixedName(name='root_T_l1'), PrefixedName(name='l1'), PrefixedName(name='l1_T_l2'),
                           PrefixedName(name='l2')])

    def test_compute_fk(self):
        self.world.init_all_fks()
        self.world._recompute_fks()
        fk = self.world.compute_fk_np(PrefixedName('l2'), PrefixedName('r2'))
        np.testing.assert_array_equal(fk, np.eye(4))

        self.world.position_state[0] = 1
        self.world._recompute_fks()
        fk = self.world.compute_fk_np(PrefixedName('l2'), PrefixedName('r2'))
        np.testing.assert_array_almost_equal(fk, np.array([[0.540302, -0.841471, 0., -1.],
                                                           [0.841471, 0.540302, 0., 0.],
                                                           [0., 0., 1., 0.],
                                                           [0., 0., 0., 1.]]))

        fk = self.world.compute_fk_np(PrefixedName('r2'), PrefixedName('l2'))
        np.testing.assert_array_almost_equal(fk, np.array([[0.540302, 0.841471, 0., 0.540302],
                                                           [-0.841471, 0.540302, 0., -0.841471],
                                                           [0., 0., 1., 0.],
                                                           [0., 0., 0., 1.]]))


if __name__ == '__main__':
    unittest.main()
