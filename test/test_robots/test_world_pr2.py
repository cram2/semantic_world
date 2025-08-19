import os
from typing import List

import numpy as np
import pytest
from rustworkx import NoPathFound

from semantic_world.adapters.urdf import URDFParser
from semantic_world.connections import OmniDrive
from semantic_world.ik_solver import MaxIterationsException, UnreachableException
from semantic_world.prefixed_name import PrefixedName
from semantic_world.robots import PR2, KinematicChain
from semantic_world.spatial_types.derivatives import Derivatives
from semantic_world.spatial_types.symbol_manager import symbol_manager
from semantic_world.world import World, Body
from semantic_world.testing import pr2_world
from semantic_world.world_entity import RootedView


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
    with pytest.raises(NoPathFound):
        pr2_world.compute_chain_of_bodies(root, tip)


def test_compute_chain_of_connections_error_pr2(pr2_world):
    root = pr2_world.get_body_by_name('r_gripper_tool_frame')
    tip = pr2_world.get_body_by_name('base_footprint')
    with pytest.raises(NoPathFound):
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
    np.testing.assert_array_almost_equal(fk, np.array(
        [[1.0, 0.0, 0.0, -0.0356], [0, 1.0, 0.0, -0.376], [0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]))


def test_compute_fk_np_l_elbow_flex_joint_pr2(pr2_world):
    tip = pr2_world.get_body_by_name('l_elbow_flex_link')
    root = pr2_world.get_body_by_name('l_upper_arm_link')

    fk_expr = pr2_world.compose_forward_kinematics_expression(root, tip)
    fk_expr_compiled = fk_expr.compile()
    fk2 = fk_expr_compiled.fast_call(symbol_manager.resolve_symbols(*fk_expr_compiled.symbol_parameters))

    np.testing.assert_array_almost_equal(fk2, np.array(
        [[0.988771, 0., -0.149438, 0.4], [0., 1., 0., 0.], [0.149438, 0., 0.988771, 0.], [0., 0., 0., 1.]]))

def test_compute_ik(pr2_world):
    bf = pr2_world.root
    eef = pr2_world.get_body_by_name('r_gripper_tool_frame')
    fk = pr2_world.compute_forward_kinematics_np(bf, eef)
    fk[0, 3] -= 0.2
    joint_state = pr2_world.compute_inverse_kinematics(bf, eef, fk)
    for joint, state in joint_state.items():
        pr2_world.state[joint.name].position = state
    pr2_world.notify_state_change()
    actual_fk = pr2_world.compute_forward_kinematics_np(bf, eef)
    assert np.allclose(actual_fk, fk, atol=1e-3)

def test_compute_ik_max_iter(pr2_world):
    bf = pr2_world.root
    eef = pr2_world.get_body_by_name('r_gripper_tool_frame')
    fk = pr2_world.compute_forward_kinematics_np(bf, eef)
    fk[2, 3] = 10
    with pytest.raises(MaxIterationsException):
        pr2_world.compute_inverse_kinematics(bf, eef, fk)

def test_compute_ik_unreachable(pr2_world):
    bf = pr2_world.root
    eef = pr2_world.get_body_by_name('base_footprint')
    fk = pr2_world.compute_forward_kinematics_np(bf, eef)
    fk[2, 3] = -1
    with pytest.raises(UnreachableException):
        pr2_world.compute_inverse_kinematics(bf, eef, fk)

def test_apply_control_commands_omni_drive_pr2(pr2_world):
    omni_drive: OmniDrive = pr2_world.get_connection_by_name('odom_combined_T_base_footprint')
    cmd = np.zeros((len(pr2_world.degrees_of_freedom)), dtype=float)
    cmd[pr2_world.state._index[omni_drive.x_vel.name]] = 100
    cmd[pr2_world.state._index[omni_drive.y_vel.name]] = 100
    cmd[pr2_world.state._index[omni_drive.yaw.name]] = 100
    dt = 0.1
    pr2_world.apply_control_commands(cmd, dt, Derivatives.jerk)
    assert pr2_world.state[omni_drive.yaw.name].jerk == 100.
    assert pr2_world.state[omni_drive.yaw.name].acceleration == 100. * dt
    assert pr2_world.state[omni_drive.yaw.name].velocity == 100. * dt * dt
    assert pr2_world.state[omni_drive.yaw.name].position == 100. * dt * dt * dt

    assert pr2_world.state[omni_drive.x_vel.name].jerk == 100.
    assert pr2_world.state[omni_drive.x_vel.name].acceleration == 100. * dt
    assert pr2_world.state[omni_drive.x_vel.name].velocity == 100. * dt * dt
    assert pr2_world.state[omni_drive.x_vel.name].position == 0

    assert pr2_world.state[omni_drive.y_vel.name].jerk == 100.
    assert pr2_world.state[omni_drive.y_vel.name].acceleration == 100. * dt
    assert pr2_world.state[omni_drive.y_vel.name].velocity == 100. * dt * dt
    assert pr2_world.state[omni_drive.y_vel.name].position == 0

    assert pr2_world.state[omni_drive.x.name].jerk == 0.
    assert pr2_world.state[omni_drive.x.name].acceleration == 0.
    assert pr2_world.state[omni_drive.x.name].velocity == 0.8951707486311977
    assert pr2_world.state[omni_drive.x.name].position == 0.08951707486311977

    assert pr2_world.state[omni_drive.y.name].jerk == 0.
    assert pr2_world.state[omni_drive.y.name].acceleration == 0.
    assert pr2_world.state[omni_drive.y.name].velocity == 1.094837581924854
    assert pr2_world.state[omni_drive.y.name].position == 0.1094837581924854

def test_pr2_view(pr2_world):
    pr2 = PR2.from_world(pr2_world)

    # Ensure there are no loose bodies
    pr2_world._notify_model_change()

    assert len(pr2.manipulators) == 2
    assert len(pr2.manipulator_chains) == 2
    assert len(pr2.sensors) == 1
    assert len(pr2.sensor_chains) == 1
    assert pr2.neck == list(pr2.sensor_chains)[0]
    assert pr2.torso.name.name == 'torso'
    assert len(pr2.torso.sensors) == 0
    assert list(pr2.sensor_chains)[0].sensors == pr2.sensors


def test_kinematic_chains(pr2_world):
    pr2 = PR2.from_world(pr2_world)
    kinematic_chain_views: List[KinematicChain] = pr2_world.get_views_by_type(KinematicChain)
    for chain in kinematic_chain_views:
        assert chain.root
        assert chain.tip
