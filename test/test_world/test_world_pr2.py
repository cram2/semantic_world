import os
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


@pytest.fixture
def pr2_world():
    urdf_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "resources", "urdf")
    pr2 = os.path.join(urdf_dir, "pr2_kinematic_tree.urdf")
    world = World()
    with world.modify_world():
        pr2_parser = URDFParser(pr2)
        pr2_world = pr2_parser.parse()
        world.merge_world(pr2_world)
        c_root_bf = OmniDrive(parent=world.root, child=pr2_world.root, _world=world)
        world.add_connection(c_root_bf)
    return world


def test_compute_chain_of_bodies_pr2(pr2_world):
    root_link = pr2_world.get_body_by_name('base_footprint')
    tip_link = pr2_world.get_body_by_name('r_gripper_tool_frame')
    real = pr2_world.compute_chain_of_bodies(root=root_link, tip=tip_link)
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


def test_compute_chain_of_connections_pr2(pr2_world):
    root_link = pr2_world.get_body_by_name('base_footprint')
    tip_link = pr2_world.get_body_by_name('r_gripper_tool_frame')
    real = pr2_world.compute_chain_of_connections(root=root_link, tip=tip_link)
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


def test_compute_chain_of_bodies_error_pr2(pr2_world):
    root = pr2_world.get_body_by_name('r_gripper_tool_frame')
    tip = pr2_world.get_body_by_name('base_footprint')
    with pytest.raises(NetworkXNoPath):
        pr2_world.compute_chain_of_bodies(root, tip)


def test_compute_chain_of_connections_error_pr2(pr2_world):
    root = pr2_world.get_body_by_name('r_gripper_tool_frame')
    tip = pr2_world.get_body_by_name('base_footprint')
    with pytest.raises(NetworkXNoPath):
        pr2_world.compute_chain_of_connections(root, tip)


def test_compute_split_chain_of_bodies_pr2(pr2_world):
    root = pr2_world.get_body_by_name('l_gripper_r_finger_tip_link')
    tip = pr2_world.get_body_by_name('l_gripper_l_finger_tip_link')
    chain1, connection, chain2 = pr2_world.compute_split_chain_of_bodies(root, tip)
    chain1 = [n.name.name for n in chain1]
    connection = [n.name.name for n in connection]
    chain2 = [n.name.name for n in chain2]
    assert chain1 == ['l_gripper_r_finger_tip_link', 'l_gripper_r_finger_link', ]
    assert connection == ['l_gripper_palm_link']
    assert chain2 == ['l_gripper_l_finger_link', 'l_gripper_l_finger_tip_link']


def test_get_split_chain_pr2(pr2_world):
    root = pr2_world.get_body_by_name('l_gripper_r_finger_tip_link')
    tip = pr2_world.get_body_by_name('l_gripper_l_finger_tip_link')
    chain1, chain2 = pr2_world.compute_split_chain_of_connections(root, tip)
    chain1 = [n.name.name for n in chain1]
    chain2 = [n.name.name for n in chain2]
    assert chain1 == ['l_gripper_r_finger_link_T_l_gripper_r_finger_tip_link',
                      'l_gripper_palm_link_T_l_gripper_r_finger_link']
    assert chain2 == ['l_gripper_palm_link_T_l_gripper_l_finger_link',
                      'l_gripper_l_finger_link_T_l_gripper_l_finger_tip_link']


def test_compute_fk_np_pr2(pr2_world):
    tip = pr2_world.get_body_by_name('r_gripper_tool_frame')
    root = pr2_world.get_body_by_name('l_gripper_tool_frame')
    fk = pr2_world.compute_forward_kinematics_np(root, tip)
    np.testing.assert_array_almost_equal(fk, np.array([[1.0, 0.0, 0.0, -0.0356],
                                                       [0, 1.0, 0.0, -0.376],
                                                       [0, 0.0, 1.0, 0.0],
                                                       [0.0, 0.0, 0.0, 1.0]]))


def test_compute_fk_np_l_elbow_flex_joint_pr2(pr2_world):
    tip = pr2_world.get_body_by_name('l_elbow_flex_link')
    root = pr2_world.get_body_by_name('l_upper_arm_link')

    fk_expr = pr2_world.compose_forward_kinematics_expression(root, tip)
    fk_expr_compiled = fk_expr.compile()
    fk2 = fk_expr_compiled.fast_call(symbol_manager.resolve_symbols(fk_expr_compiled.symbol_parameters))

    np.testing.assert_array_almost_equal(fk2, np.array([[0.988771, 0., -0.149438, 0.4],
                                                        [0., 1., 0., 0.],
                                                        [0.149438, 0., 0.988771, 0.],
                                                        [0., 0., 0., 1.]]))


def test_apply_control_commands_omni_drive_pr2(pr2_world):
    omni_drive: OmniDrive = pr2_world.get_connection_by_name(PrefixedName(name='root_T_base_footprint',
                                                                          prefix='pr2_kinematic_tree'))
    cmd = np.zeros((len(pr2_world.degrees_of_freedom)), dtype=float)
    cmd[-3] = 100
    cmd[-2] = 100
    cmd[-1] = 100
    dt = 0.1
    pr2_world.apply_control_commands(cmd, dt, Derivatives.jerk)
    assert pr2_world.state[Derivatives.jerk, omni_drive.yaw.state_idx] == 100.
    assert pr2_world.state[Derivatives.acceleration, omni_drive.yaw.state_idx] == 100. * dt
    assert pr2_world.state[Derivatives.velocity, omni_drive.yaw.state_idx] == 100. * dt * dt
    assert pr2_world.state[Derivatives.position, omni_drive.yaw.state_idx] == 100. * dt * dt * dt

    assert pr2_world.state[Derivatives.jerk, omni_drive.x_vel.state_idx] == 100.
    assert pr2_world.state[Derivatives.acceleration, omni_drive.x_vel.state_idx] == 100. * dt
    assert pr2_world.state[Derivatives.velocity, omni_drive.x_vel.state_idx] == 100. * dt * dt
    assert pr2_world.state[Derivatives.position, omni_drive.x_vel.state_idx] == 0

    assert pr2_world.state[Derivatives.jerk, omni_drive.y_vel.state_idx] == 100.
    assert pr2_world.state[Derivatives.acceleration, omni_drive.y_vel.state_idx] == 100. * dt
    assert pr2_world.state[Derivatives.velocity, omni_drive.y_vel.state_idx] == 100. * dt * dt
    assert pr2_world.state[Derivatives.position, omni_drive.y_vel.state_idx] == 0

    assert pr2_world.state[Derivatives.jerk, omni_drive.x.state_idx] == 0.
    assert pr2_world.state[Derivatives.acceleration, omni_drive.x.state_idx] == 0.
    assert pr2_world.state[Derivatives.velocity, omni_drive.x.state_idx] == 0.8951707486311977
    assert pr2_world.state[Derivatives.position, omni_drive.x.state_idx] == 0.08951707486311977

    assert pr2_world.state[Derivatives.jerk, omni_drive.y.state_idx] == 0.
    assert pr2_world.state[Derivatives.acceleration, omni_drive.y.state_idx] == 0.
    assert pr2_world.state[Derivatives.velocity, omni_drive.y.state_idx] == 1.094837581924854
    assert pr2_world.state[Derivatives.position, omni_drive.y.state_idx] == 0.1094837581924854
