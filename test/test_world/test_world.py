import os
import unittest
import pytest

import numpy as np

from semantic_world.adapters.urdf import URDFParser
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

        with self.world.modify_world():
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
        assert self.world._position_state[0] == 0

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
        fk = self.world.compute_fk_np(PrefixedName('l2'), PrefixedName('r2'))
        np.testing.assert_array_equal(fk, np.eye(4))

        self.world._state[0, 0] = 1.
        self.world.notify_state_change()
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

    def test_apply_control_commands(self):
        cmd = np.array([100.])
        dt = 0.1
        self.world.apply_control_commands(cmd, dt, Derivatives.jerk)
        assert self.world._state[0, Derivatives.jerk] == 100.
        assert self.world._state[0, Derivatives.acceleration] == 100. * dt
        assert self.world._state[0, Derivatives.velocity] == 100. * dt * dt
        assert self.world._state[0, Derivatives.position] == 100. * dt * dt * dt



class PR2WorldTests(unittest.TestCase):
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    pr2 = os.path.join(urdf_dir, "pr2_kinematic_tree.urdf")

    def setUp(self):
        pr2_parser = URDFParser(self.pr2)
        self.world = pr2_parser.parse()

    def test_get_chain(self):
        root_link = 'base_footprint'
        tip_link = 'r_gripper_tool_frame'
        real = self.world.compute_chain(root_link_name=PrefixedName(root_link),
                                        tip_link_name=PrefixedName(tip_link),
                                        add_joints=True,
                                        add_links=True,
                                        add_fixed_joints=True,
                                        add_non_controlled_joints=True)
        real = [x.name for x in real]
        assert real == [PrefixedName(name='base_footprint', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='base_footprint_T_base_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='base_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='base_link_T_torso_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='torso_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='torso_lift_link_T_r_shoulder_pan_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_pan_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_pan_link_T_r_shoulder_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_lift_link_T_r_upper_arm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_roll_link_T_r_upper_arm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_link_T_r_elbow_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_elbow_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_elbow_flex_link_T_r_forearm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_roll_link_T_r_forearm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_link_T_r_wrist_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_flex_link_T_r_wrist_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_roll_link_T_r_gripper_palm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_gripper_palm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_gripper_palm_link_T_r_gripper_tool_frame', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_gripper_tool_frame', prefix='pr2_kinematic_tree')]

    def test_get_chain2(self):
        with pytest.raises(ValueError):
            self.world.compute_chain(PrefixedName('l_gripper_tool_frame'),
                                     PrefixedName('r_gripper_tool_frame'),
                                     add_joints=True, add_links=True, add_fixed_joints=True,
                                     add_non_controlled_joints=True)

    def test_get_split_chain(self):
        chain1, connection, chain2 = self.world.compute_split_chain(PrefixedName('l_gripper_r_finger_tip_link'),
                                                                    PrefixedName('l_gripper_l_finger_tip_link'),
                                                                    add_joints=True, add_links=True,
                                                                    add_fixed_joints=True,
                                                                    add_non_controlled_joints=True)
        chain1 = [n.name.name for n in chain1]
        connection = [n.name.name for n in connection]
        chain2 = [n.name.name for n in chain2]
        assert chain1 == ['l_gripper_r_finger_tip_link', 'l_gripper_r_finger_link_T_l_gripper_r_finger_tip_link',
                          'l_gripper_r_finger_link',
                          'l_gripper_palm_link_T_l_gripper_r_finger_link']
        assert connection == ['l_gripper_palm_link']
        assert chain2 == ['l_gripper_palm_link_T_l_gripper_l_finger_link', 'l_gripper_l_finger_link',
                          'l_gripper_l_finger_link_T_l_gripper_l_finger_tip_link',
                          'l_gripper_l_finger_tip_link']

    def test_compute_fk_np(self):
        tip = self.world.get_body_by_name('r_gripper_tool_frame').name
        root = self.world.get_body_by_name('l_gripper_tool_frame').name
        fk = self.world.compute_fk_np(root, tip)
        np.testing.assert_array_almost_equal(fk, np.array([[1.0, 0.0, 0.0, -0.0356],
                                                           [0, 1.0, 0.0, -0.376],
                                                           [0, 0.0, 1.0, 0.0],
                                                           [0.0, 0.0, 0.0, 1.0]]))


if __name__ == '__main__':
    unittest.main()
