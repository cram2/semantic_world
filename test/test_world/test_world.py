import os
import unittest
import pytest

import numpy as np
from networkx.exception import NetworkXNoPath

from semantic_world.adapters.urdf import URDFParser
from semantic_world.connections import PrismaticConnection, RevoluteConnection, Connection6DoF, OmniDrive, \
    FixedConnection
from semantic_world.prefixed_name import PrefixedName
from semantic_world.spatial_types.derivatives import Derivatives
from semantic_world.spatial_types.symbol_manager import symbol_manager
from semantic_world.world import World, Body, Connection


class WorldTestCase(unittest.TestCase):

    def setUp(self):
        self.world = World()
        self.l1 = Body(PrefixedName('l1'))
        self.l2 = Body(PrefixedName('l2'))
        self.bf = Body(PrefixedName('bf'))
        self.r1 = Body(PrefixedName('r1'))
        self.r2 = Body(PrefixedName('r2'))

        with self.world.modify_world():
            dof = self.world.create_degree_of_freedom(name=PrefixedName('dof'),
                                                      lower_limits={Derivatives.velocity: -1},
                                                      upper_limits={Derivatives.velocity: 1})

            c_l1_l2 = PrismaticConnection(self.l1, self.l2, dof=dof, axis=(1, 0, 0))
            c_r1_r2 = RevoluteConnection(self.r1, self.r2, dof=dof, axis=(0, 0, 1))
            bf_root_l1 = FixedConnection(self.bf, self.l1)
            bf_root_r1 = FixedConnection(self.bf, self.r1)
            c_root_bf = Connection6DoF(parent=self.world.root, child=self.bf, _world=self.world)
            self.world.add_connection(c_root_bf)
            self.world.add_connection(c_l1_l2)
            self.world.add_connection(c_r1_r2)
            self.world.add_connection(bf_root_l1)
            self.world.add_connection(bf_root_r1)

    def test_construction(self):
        self.world.validate()
        self.assertEqual(len(self.world.connections), 5)
        self.assertEqual(len(self.world.bodies), 6)
        assert self.world.state[Derivatives.position, 0] == 0

    def test_chain_of_bodies(self):
        result = self.world.compute_chain_of_bodies(root=self.world.root, tip=self.l2)
        result = [x.name for x in result]
        assert result == [PrefixedName(name='root', prefix='world'),
                          PrefixedName(name='bf', prefix=None),
                          PrefixedName(name='l1', prefix=None),
                          PrefixedName(name='l2', prefix=None)]

    def test_chain_of_connections(self):
        result = self.world.compute_chain_of_connections(root=self.world.root, tip=self.l2)
        result = [x.name for x in result]
        assert result == [PrefixedName(name='root_T_bf', prefix=None),
                          PrefixedName(name='bf_T_l1', prefix=None),
                          PrefixedName(name='l1_T_l2', prefix=None)]

    def test_split_chain_of_bodies(self):
        result = self.world.compute_split_chain_of_bodies(root=self.r2, tip=self.l2)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([PrefixedName(name='r2', prefix=None),
                           PrefixedName(name='r1', prefix=None)],
                          [PrefixedName(name='bf', prefix=None)],
                          [PrefixedName(name='l1', prefix=None),
                           PrefixedName(name='l2', prefix=None)])

    def test_split_chain_of_bodies_adjacent1(self):
        result = self.world.compute_split_chain_of_bodies(root=self.r2, tip=self.r1)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([PrefixedName(name='r2', prefix=None)],
                          [PrefixedName(name='r1', prefix=None)],
                          [])

    def test_split_chain_of_bodies_adjacent2(self):
        result = self.world.compute_split_chain_of_bodies(root=self.r1, tip=self.r2)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([],
                          [PrefixedName(name='r1', prefix=None)],
                          [PrefixedName(name='r2', prefix=None)])

    def test_split_chain_of_bodies_identical(self):
        result = self.world.compute_split_chain_of_bodies(root=self.r1, tip=self.r1)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([],
                          [PrefixedName(name='r1', prefix=None)],
                          [])

    def test_split_chain_of_connections(self):
        result = self.world.compute_split_chain_of_connections(root=self.r2, tip=self.l2)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([PrefixedName(name='r1_T_r2', prefix=None),
                           PrefixedName(name='bf_T_r1', prefix=None)],
                          [PrefixedName(name='bf_T_l1', prefix=None),
                           PrefixedName(name='l1_T_l2', prefix=None)])

    def test_split_chain_of_connections_adjacent1(self):
        result = self.world.compute_split_chain_of_connections(root=self.r2, tip=self.r1)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([PrefixedName(name='r1_T_r2', prefix=None)], [])

    def test_split_chain_of_connections_adjacent2(self):
        result = self.world.compute_split_chain_of_connections(root=self.r1, tip=self.r2)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([], [PrefixedName(name='r1_T_r2', prefix=None)])

    def test_split_chain_of_connections_identical(self):
        result = self.world.compute_split_chain_of_connections(root=self.r1, tip=self.r1)
        result = tuple([x.name for x in y] for y in result)
        assert result == ([], [])

    def test_compute_fk_connection6dof(self):
        fk = self.world.compute_fk_np(self.world.root, self.bf)
        np.testing.assert_array_equal(fk, np.eye(4))

        self.world.state[Derivatives.position, 1] = 1.
        self.world.state[Derivatives.position, -1] = 0
        self.world.state[Derivatives.position, -2] = 1
        self.world.notify_state_change()
        np.testing.assert_array_equal(fk, [[-1., 0., 0., 1.],
                                           [0., -1., 0., 0.],
                                           [0., 0., 1., 0.],
                                           [0., 0., 0., 1.]])

    def test_compute_fk(self):
        fk = self.world.compute_fk_np(self.l2, self.r2)
        np.testing.assert_array_equal(fk, np.eye(4))

        self.world.state[Derivatives.position, 0] = 1.
        self.world.notify_state_change()
        fk = self.world.compute_fk_np(self.l2, self.r2)
        np.testing.assert_array_almost_equal(fk, np.array([[0.540302, -0.841471, 0., -1.],
                                                           [0.841471, 0.540302, 0., 0.],
                                                           [0., 0., 1., 0.],
                                                           [0., 0., 0., 1.]]))

    def test_compute_fk_expression(self):
        self.world.state[Derivatives.position, 0] = 1.
        self.world.notify_state_change()
        fk = self.world.compute_fk_np(self.r2, self.l2)
        fk_expr = self.world.compose_forward_kinematics_expression(self.r2, self.l2)
        fk_expr_compiled = fk_expr.compile()
        fk2 = fk_expr_compiled.fast_call(symbol_manager.resolve_symbols(fk_expr_compiled.params))
        np.testing.assert_array_almost_equal(fk, fk2)

    def test_apply_control_commands(self):
        cmd = np.array([100., 0, 0, 0, 0, 0, 0, 0])
        dt = 0.1
        self.world.apply_control_commands(cmd, dt, Derivatives.jerk)
        assert self.world.state[Derivatives.jerk, 0] == 100.
        assert self.world.state[Derivatives.acceleration, 0] == 100. * dt
        assert self.world.state[Derivatives.velocity, 0] == 100. * dt * dt
        assert self.world.state[Derivatives.position, 0] == 100. * dt * dt * dt


class PR2WorldTests(unittest.TestCase):
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    pr2 = os.path.join(urdf_dir, "pr2_kinematic_tree.urdf")

    def setUp(self):
        self.world = World()
        with self.world.modify_world():
            pr2_parser = URDFParser(self.pr2)
            pr2_world = pr2_parser.parse()
            self.world.merge_world(pr2_world)
            c_root_bf = OmniDrive(parent=self.world.root, child=pr2_world.root, _world=self.world)
            self.world.add_connection(c_root_bf)

    def test_compute_chain_of_bodies(self):
        root_link = self.world.get_body_by_name('base_footprint')
        tip_link = self.world.get_body_by_name('r_gripper_tool_frame')
        real = self.world.compute_chain_of_bodies(root=root_link, tip=tip_link)
        real = [x.name for x in real]
        assert real == [PrefixedName(name='base_footprint', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='base_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='torso_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_pan_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_elbow_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_gripper_palm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_gripper_tool_frame', prefix='pr2_kinematic_tree')]

    def test_compute_chain_of_connections(self):
        root_link = self.world.get_body_by_name('base_footprint')
        tip_link = self.world.get_body_by_name('r_gripper_tool_frame')
        real = self.world.compute_chain_of_connections(root=root_link, tip=tip_link)
        real = [x.name for x in real]
        assert real == [PrefixedName(name='base_footprint_T_base_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='base_link_T_torso_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='torso_lift_link_T_r_shoulder_pan_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_pan_link_T_r_shoulder_lift_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_shoulder_lift_link_T_r_upper_arm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_roll_link_T_r_upper_arm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_upper_arm_link_T_r_elbow_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_elbow_flex_link_T_r_forearm_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_roll_link_T_r_forearm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_forearm_link_T_r_wrist_flex_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_flex_link_T_r_wrist_roll_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_wrist_roll_link_T_r_gripper_palm_link', prefix='pr2_kinematic_tree'),
                        PrefixedName(name='r_gripper_palm_link_T_r_gripper_tool_frame', prefix='pr2_kinematic_tree')]

    def test_compute_chain_of_bodies_error(self):
        root = self.world.get_body_by_name('r_gripper_tool_frame')
        tip = self.world.get_body_by_name('base_footprint')
        with pytest.raises(NetworkXNoPath):
            self.world.compute_chain_of_bodies(root, tip)

    def test_compute_chain_of_connections_error(self):
        root = self.world.get_body_by_name('r_gripper_tool_frame')
        tip = self.world.get_body_by_name('base_footprint')
        with pytest.raises(NetworkXNoPath):
            self.world.compute_chain_of_connections(root, tip)

    def test_compute_split_chain_of_bodies(self):
        root = self.world.get_body_by_name('l_gripper_r_finger_tip_link')
        tip = self.world.get_body_by_name('l_gripper_l_finger_tip_link')
        chain1, connection, chain2 = self.world.compute_split_chain_of_bodies(root, tip)
        chain1 = [n.name.name for n in chain1]
        connection = [n.name.name for n in connection]
        chain2 = [n.name.name for n in chain2]
        assert chain1 == ['l_gripper_r_finger_tip_link', 'l_gripper_r_finger_link', ]
        assert connection == ['l_gripper_palm_link']
        assert chain2 == ['l_gripper_l_finger_link', 'l_gripper_l_finger_tip_link']

    def test_get_split_chain(self):
        root = self.world.get_body_by_name('l_gripper_r_finger_tip_link')
        tip = self.world.get_body_by_name('l_gripper_l_finger_tip_link')
        chain1, chain2 = self.world.compute_split_chain_of_connections(root, tip)
        chain1 = [n.name.name for n in chain1]
        chain2 = [n.name.name for n in chain2]
        assert chain1 == ['l_gripper_r_finger_link_T_l_gripper_r_finger_tip_link',
                          'l_gripper_palm_link_T_l_gripper_r_finger_link']
        assert chain2 == ['l_gripper_palm_link_T_l_gripper_l_finger_link',
                          'l_gripper_l_finger_link_T_l_gripper_l_finger_tip_link']

    def test_compute_fk_np(self):
        tip = self.world.get_body_by_name('r_gripper_tool_frame')
        root = self.world.get_body_by_name('l_gripper_tool_frame')
        fk = self.world.compute_fk_np(root, tip)
        np.testing.assert_array_almost_equal(fk, np.array([[1.0, 0.0, 0.0, -0.0356],
                                                           [0, 1.0, 0.0, -0.376],
                                                           [0, 0.0, 1.0, 0.0],
                                                           [0.0, 0.0, 0.0, 1.0]]))

    def test_compute_fk_np_l_elbow_flex_joint(self):
        tip = self.world.get_body_by_name('l_elbow_flex_link')
        root = self.world.get_body_by_name('l_upper_arm_link')

        fk_expr = self.world.compose_forward_kinematics_expression(root, tip)
        fk_expr_compiled = fk_expr.compile()
        fk2 = fk_expr_compiled.fast_call(symbol_manager.resolve_symbols(fk_expr_compiled.params))

        np.testing.assert_array_almost_equal(fk2, np.array([[0.988771, 0., -0.149438, 0.4],
                                                            [0., 1., 0., 0.],
                                                            [0.149438, 0., 0.988771, 0.],
                                                            [0., 0., 0., 1.]]))

    def test_apply_control_commands_omni_drive(self):
        omni_drive: OmniDrive = self.world.get_connection_by_name(PrefixedName(name='root_T_base_footprint',
                                                                               prefix='pr2_kinematic_tree'))
        cmd = np.zeros((len(self.world.degrees_of_freedom)), dtype=float)
        cmd[-3] = 100
        cmd[-2] = 100
        cmd[-1] = 100
        dt = 0.1
        self.world.apply_control_commands(cmd, dt, Derivatives.jerk)
        assert self.world.state[Derivatives.jerk, omni_drive.yaw.state_idx] == 100.
        assert self.world.state[Derivatives.acceleration, omni_drive.yaw.state_idx] == 100. * dt
        assert self.world.state[Derivatives.velocity, omni_drive.yaw.state_idx] == 100. * dt * dt
        assert self.world.state[Derivatives.position, omni_drive.yaw.state_idx] == 100. * dt * dt * dt

        assert self.world.state[Derivatives.jerk, omni_drive.x_vel.state_idx] == 100.
        assert self.world.state[Derivatives.acceleration, omni_drive.x_vel.state_idx] == 100. * dt
        assert self.world.state[Derivatives.velocity, omni_drive.x_vel.state_idx] == 100. * dt * dt
        assert self.world.state[Derivatives.position, omni_drive.x_vel.state_idx] == 0

        assert self.world.state[Derivatives.jerk, omni_drive.y_vel.state_idx] == 100.
        assert self.world.state[Derivatives.acceleration, omni_drive.y_vel.state_idx] == 100. * dt
        assert self.world.state[Derivatives.velocity, omni_drive.y_vel.state_idx] == 100. * dt * dt
        assert self.world.state[Derivatives.position, omni_drive.y_vel.state_idx] == 0

        assert self.world.state[Derivatives.jerk, omni_drive.x.state_idx] == 0.
        assert self.world.state[Derivatives.acceleration, omni_drive.x.state_idx] == 0.
        assert self.world.state[Derivatives.velocity, omni_drive.x.state_idx] == 0.8951707486311977
        assert self.world.state[Derivatives.position, omni_drive.x.state_idx] == 0.08951707486311977

        assert self.world.state[Derivatives.jerk, omni_drive.y.state_idx] == 0.
        assert self.world.state[Derivatives.acceleration, omni_drive.y.state_idx] == 0.
        assert self.world.state[Derivatives.velocity, omni_drive.y.state_idx] == 1.094837581924854
        assert self.world.state[Derivatives.position, omni_drive.y.state_idx] == 0.1094837581924854


if __name__ == '__main__':
    unittest.main()
